from bliss.controllers.motor import Controller
from bliss.common import log as elog
from bliss.common.axis import AxisState
from bliss.common import event
from bliss.controllers.motor import axis_method, add_axis_method
from bliss.controllers.motor import add_axis_attribute

import math
import time
import random
import functools

"""
mockup.py : a mockup controller for bliss.
To be used as skeleton to write bliss plugin controller.
"""

"""
config :
 'velocity' in unit/s
 'acceleration' in unit/s^2
 'steps_per_unit' in unit^-1  (default 1)
 'backlash' in unit
"""

config_xml = """
<config>
  <controller class="mockup">
    <axis name="robx" class="MockupAxis">
      <velocity  value="1"/>
      <acceleration value="3"/>
      <steps_per_unit value="10"/>
      <backlash value="2"/>
    </axis>
  </controller>
</config>
"""


class Mockup(Controller):
    def __init__(self, name, config, axes, encoders):
        Controller.__init__(self, name, config, axes, encoders)

        self._axis_moves = {}
        self.__encoders = {}

        # Custom attributes.
        self.__voltages = {}
        self.__cust_attr_float = {}

        self.__error_mode = False
        self._hw_status = AxisState("READY")
        self.__hw_limit = (None, None)

        self._hw_status.create_state("PARKED", "mot au parking")

        # Access to the config.
        try:
            self.host = self.config.get("host")
        except:
            elog.debug("no 'host' defined in config for %s" % name)

        # Adds Mockup-specific settings.
        self.axis_settings.add('init_count', int)

    """
    Controller initialization actions.
    """
    def initialize(self):
        # hardware initialization
        for axis_name, axis in self.axes.iteritems():
            axis.settings.set('init_count', 0)
            axis.settings.set('cust_attr_float', 67.89)
            axis.settings.set('cust_attr_bool', True)

            axis.__vel = None
            axis.__acc = None

    """
    Axes initialization actions.
    """
    def initialize_axis(self, axis):
        def set_pos(move_done, axis=axis):
            if move_done:
                self.set_position(axis, axis.dial()*axis.steps_per_unit)

        self._axis_moves[axis] = {
            "measured_simul": False,
            "end_t": 0,
            "end_pos": 0,
            "move_done_cb": set_pos }

        event.connect(axis, "move_done", set_pos)

        # this is to test axis are initialized only once
        axis.settings.set('init_count', axis.settings.get('init_count') + 1)

        add_axis_attribute(axis,
                           name="voltage",
                           fget=self.get_voltage,
                           fset=self.set_voltage,
                           type_info="int")

        add_axis_attribute(axis,
                           name="cust_attr_float",
                           fget=self.get_cust_attr_float,
                           fset=self.set_cust_attr_float,
                           type_info='float')

        add_axis_attribute(axis, self.custom_get_measured_noise,
                           self.custom_set_measured_noise, name='Measured_Noise',
                           type_info="float")

        if axis.encoder:
            self.__encoders.setdefault(axis.encoder, {})["axis"] = axis

    def initialize_encoder(self, encoder):
        self.__encoders.setdefault(encoder, {})["measured_noise"] = 0.0
        self.__encoders[encoder]["steps"] = None

    """
    Actions to perform at controller closing.
    """
    def finalize(self):
        pass

    def set_hw_limit(self, axis, low_limit, high_limit):
        if low_limit is not None:
            ll= axis.user2dial(low_limit)*axis.steps_per_unit
        else:
            ll = None
        if high_limit is not None:
            hl = axis.user2dial(high_limit)*axis.steps_per_unit
        else:
            hl = None
        if hl is not None and hl < ll:
          self.__hw_limit = (hl, ll)
        else:
          self.__hw_limit = (ll, hl)

    def start_all(self, *motion_list):
        if self.__error_mode:
            raise RuntimeError("Cannot start because error mode is set")
        t0 = time.time()
        for motion in motion_list:
            self.start_one(motion, t0=t0)

    def start_one(self, motion, t0=None):
        if self.__error_mode:
            raise RuntimeError("Cannot start because error mode is set")
        axis = motion.axis
        t0 = t0 or time.time()
        pos = self.read_position(axis)
        v = self.read_velocity(axis)
        ll, hl = self.__hw_limit
        if hl is not None and motion.target_pos > hl:
            d=hl-motion.target_pos
            motion.delta -= d
            motion.target_pos = hl
        if ll is not None and motion.target_pos < ll:
            d=ll-motion.target_pos
            motion.delta -= d
            motion.target_pos = ll
        self._axis_moves[axis].update({
            "start_pos": pos,
            "delta": motion.delta,
            "end_pos": motion.target_pos,
            "end_t": t0 +
            math.fabs(
                motion.delta) /
            float(v),
            "t0": t0})

    def read_position(self, axis):
        """
        Returns the position (measured or desired) taken from controller
        in controller unit (steps).
        """

        # handle read out during a motion
        if self._axis_moves[axis]["end_t"]:
            # motor is moving
            t = time.time()
            v = self.read_velocity(axis)
            d = math.copysign(1, self._axis_moves[axis]["delta"])
            dt = t - self._axis_moves[axis]["t0"]  # t0=time at start_one.
            pos = self._axis_moves[axis]["start_pos"] + d * dt * v
        else:
            pos = self._axis_moves[axis]["end_pos"]

        return int(round(pos))

    def read_encoder(self, encoder):
        """
        returns encoder position.
        unit : 'encoder steps'
        """

        axis = self.__encoders[encoder]["axis"]

        if self.__encoders[encoder]["measured_noise"] != 0.0:
            # Simulates noisy encoder.
            amplitude = self.__encoders[encoder]["measured_noise"]
            noise_mm = random.uniform(-amplitude, amplitude)

            _pos = self.read_position(axis) / axis.steps_per_unit
            _pos += noise_mm

            self.__encoders[encoder]["steps"] = _pos * encoder.steps_per_unit

        else:
            # print "Perfect encoder"
            if self.__encoders[encoder]["steps"] is None:
                _axis_pos = self.read_position(axis) / axis.steps_per_unit
                self.__encoders[encoder]["steps"] = _axis_pos * encoder.steps_per_unit

        return self.__encoders[encoder]["steps"]

    def set_encoder(self, encoder, encoder_steps):
        self.__encoders[encoder]["steps"]=encoder_steps

    """
    VELOCITY
    """
    def read_velocity(self, axis):
        """
        Returns the current velocity taken from controller
        in motor units.
        """
        return axis.__vel

    def set_velocity(self, axis, new_velocity):
        """
        <new_velocity> is in motor units
        """
        axis.__vel = new_velocity

    """
    ACCELERATION
    """
    def read_acceleration(self, axis):
        """
        must return acceleration in controller units / s2
        """
        return axis.__acc

    def set_acceleration(self, axis, new_acceleration):
        """
        <new_acceleration> is in controller units / s2
        """
        axis.__acc = new_acceleration

    """
    ON / OFF
    """
    def set_on(self, axis):
        self._hw_status = "READY"

    def set_off(self, axis):
        self._hw_status = "OFF"

    """
    Hard limits
    """
    def _check_hw_limits(self, axis):
        ll, hl = self.__hw_limit
        pos = self.read_position(axis)
        if ll is not None and pos <= ll:
            return AxisState("READY", "LIMNEG")
        elif hl is not None and pos >= hl:
            return AxisState("READY", "LIMPOS")
        return AxisState("READY")

    """
    STATE
    """
    def state(self, axis):
        if self._hw_status == "PARKED":
            return AxisState("PARKED")

        if self._hw_status == "OFF":
            return AxisState("OFF")

        if self._axis_moves[axis]["end_t"] > time.time():
           return AxisState("MOVING")
        else:
           self._axis_moves[axis]["end_t"]=0
           return self._check_hw_limits(axis)

    """
    Must send a command to the controller to abort the motion of given axis.
    """
    def stop(self, axis):
        self._axis_moves[axis]["end_pos"] = self.read_position(axis)
        self._axis_moves[axis]["end_t"] = 0

    def stop_all(self, *motion_list):
        for motion in motion_list:
            axis = motion.axis
            self._axis_moves[axis]["end_pos"] = self.read_position(axis)
            self._axis_moves[axis]["end_t"] = 0

    """
    HOME and limits search
    """
    def home_search(self, axis, switch):
        self._axis_moves[axis]["start_pos"] = self._axis_moves[axis]["end_pos"]
        self._axis_moves[axis]["end_pos"] = 0
        self._axis_moves[axis]["delta"] = 0
        self._axis_moves[axis]["end_t"] = 0
        self._axis_moves[axis]["t0"] = time.time()
        self._axis_moves[axis]["home_search_start_time"] = time.time()

#    def home_set_hardware_position(self, axis, home_pos):
#        raise NotImplementedError

    def home_state(self, axis):
        if(time.time() - self._axis_moves[axis]["home_search_start_time"]) > 2:
            return AxisState("READY")
        else:
            return AxisState("MOVING")

    def limit_search(self, axis, limit):
        self._axis_moves[axis]["start_pos"] = self._axis_moves[axis]["end_pos"]
        self._axis_moves[axis]["end_pos"] = 1E6 if limit > 0 else -1E6
        self._axis_moves[axis]["delta"] = self._axis_moves[axis]["end_pos"] #this is just for direction sign
        self._axis_moves[axis]["end_pos"] *= axis.steps_per_unit
        self._axis_moves[axis]["end_t"] = time.time() + 2
        self._axis_moves[axis]["t0"] = time.time()

    def get_info(self, axis):
        return "turlututu chapo pointu : %s" % (axis.name)

    def raw_write(self, axis, com):
        print ("raw_write:  com = %s" % com)

    def raw_write_read(self, axis, com):
        return com + ">-<" + com

    def set_position(self, axis, pos):
        self._axis_moves[axis]["end_pos"] = pos
        self._axis_moves[axis]["end_t"] = 0
        return pos

    def put_discrepancy(self, axis, disc):
        self._axis_moves[axis]["end_pos"] += disc

    """
    Custom axis methods
    """
    # VOID VOID
    @axis_method
    def custom_park(self, axis):
        print "parking"
        self._hw_status.clear()
        self._hw_status.set("PARKED")

    # VOID LONG
    @axis_method(types_info=("None", "int"))
    def custom_get_forty_two(self, axis):
        return 42

    # LONG LONG  + renaming.
    @axis_method(name= "CustomGetTwice", types_info=("int", "int"))
    def custom_get_twice(self, axis, LongValue):
        return LongValue * 2

    # STRING STRING
    @axis_method(types_info=("str", "str"))
    def custom_get_chapi(self, axis, value):
        if value == "chapi":
            return "chapo"
        elif value == "titi":
            return "toto"
        else:
            return "bla"

    # STRING VOID
    @axis_method(types_info=("str", "None"))
    def custom_send_command(self, axis, value):
        print "command=", value

    # BOOL NONE
    @axis_method(name="Set_Closed_Loop", types_info=("bool", "None"))
    def _set_closed_loop(self, axis, onoff = True):
        print "I set the closed loop ", onoff

    # Types by default (None, None)
    @axis_method
    def custom_command_no_types(self, axis):
        print "print with no types"

    def custom_get_measured_noise(self, axis):
        noise = 0.0
        if not axis.encoder in self.__encoders:
            raise KeyError("cannot read measured noise: %s "
                           "doesn't have encoder" % axis.name)
        noise = self.__encoders[axis.encoder].get("measured_noise", 0.0)

    @axis_method(types_info=("float", "None"))
    def custom_set_measured_noise(self, axis, noise):
        """
        Custom axis method to add a random noise, given in user units,
        to measured positions. Set noise value to 0 to have a measured
        position equal to target position.
        By the way we add a ref to the coresponding axis.
        """
        self.__encoders[axis.encoder]["measured_noise"] = noise
        self.__encoders[axis.encoder]["axis"] = axis

    def set_error(self, error_mode):
        self.__error_mode = error_mode


    """
    Custom attributes methods
    """
    def get_voltage(self, axis):
        return self.__voltages.setdefault(axis, 220)

    def set_voltage(self, axis, voltage):
        self.__voltages[axis] = voltage

    def get_cust_attr_float(self, axis):
        return self.__cust_attr_float.setdefault(axis, 0)

    def set_cust_attr_float(self, axis, value):
        self.__cust_attr_float[axis]

