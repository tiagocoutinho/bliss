# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

import math
import time
import random

from bliss.controllers.motor import Controller
from bliss.common import log as elog
from bliss.common.axis import Axis,AxisState
from bliss.common import event

from bliss.common.hook import MotionHook
from bliss.common.utils import object_method
from bliss.common.utils import object_attribute_get, object_attribute_set

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

class Mockup(Controller):
    def __init__(self, *args, **kwargs):
        Controller.__init__(self, *args, **kwargs)

        self._axis_moves = {}
        self.__encoders = {}

        # Custom attributes.
        self.__voltages = {}
        self.__cust_attr_float = {}

        self.__error_mode = False
        self._hw_state = AxisState("READY")
        self.__hw_limit = (None, None)

        self._hw_state.create_state("PARKED", "mot au parking")

        # Access to the config.
        try:
            self.host = self.config.get("host")
        except:
            elog.debug("no 'host' defined in config for %s" % self.name)

        # Adds Mockup-specific settings.
        self.axis_settings.add('init_count', int)
        self.axis_settings.add('hw_position', float)

    """
    Controller initialization actions.
    """
    def initialize(self):
        # hardware initialization
        for axis_name, axis in self.axes.iteritems():
            axis.settings.set('init_count', 0)

    """
    Axes initialization actions.
    """
    def initialize_axis(self, axis):
        # this is to protect position reading,
        # indeed the mockup controller uses redis to store
        # a 'hardware position', and it is not allowed
        # to read a position before it has been written
        def set_pos(move_done, axis=axis):
            if move_done:
                self.set_position(axis, axis.dial()*axis.steps_per_unit)

        self._axis_moves[axis] = {
            "end_t": None,
            "move_done_cb": set_pos }

        if axis.settings.get('hw_position') is None:
            axis.settings.set('hw_position', 0)
        self._axis_moves[axis]['start_pos'] = self.read_position(axis)
        self._axis_moves[axis]['target'] = self._axis_moves[axis]['start_pos']

        event.connect(axis, "move_done", set_pos)

        self.__voltages[axis] = axis.config.get("default_voltage", 
                                                int, default=220)
        self.__cust_attr_float[axis] = axis.config.get("default_cust_attr",
                                                       float, default=3.14)

        # this is to test axis are initialized only once
        axis.settings.set('init_count', axis.settings.get('init_count') + 1)

        if axis.encoder:
            self.__encoders.setdefault(axis.encoder, {})["axis"] = axis

    def initialize_encoder(self, encoder):
        self.__encoders.setdefault(encoder, {})["measured_noise"] = None
        self.__encoders[encoder]["steps"] = None

    """
    Actions to perform at controller closing.
    """
    def finalize(self):
        pass

    def set_hw_limits(self, axis, low_limit, high_limit):
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
        end_pos = motion.target_pos
        if hl is not None and end_pos > hl:
            end_pos = hl
        if ll is not None and end_pos < ll:
            end_pos = ll
        delta = motion.delta + end_pos - motion.target_pos
        self._axis_moves[axis].update({
            "start_pos": pos,
            "delta": delta,
            "end_t": t0 + math.fabs(delta) / float(v),
            "target": end_pos,
            "t0": t0})

    def start_jog(self, axis, velocity, direction):
        t0 = time.time() 
        pos = self.read_position(axis)
        self.set_velocity(axis, velocity)
        self._axis_moves[axis].update({ 
            "start_pos": pos,
            "delta": direction,
            "end_t": t0+1E9,
            "t0": t0})

    def read_position(self, axis, t=None):
        """
        Returns the position (measured or desired) taken from controller
        in controller unit (steps).
        """
        t = t or time.time()
        # handle read out during a motion
        end_t = self._axis_moves[axis]["end_t"]
        if end_t is not None and t >= end_t:
            pos = self._axis_moves[axis]["target"]
            axis.settings.set('hw_position', pos)
        elif end_t:
            # motor is moving
            v = self.read_velocity(axis)
            a = self.read_acceleration(axis)
            d = math.copysign(1, self._axis_moves[axis]["delta"])
            dt = t - self._axis_moves[axis]["t0"]  # t0=time at start_one.
            acctime = min(dt, v/a)
            dt -= acctime
            pos = self._axis_moves[axis]["start_pos"] + d*a*0.5*acctime**2 
            if dt > 0:
                pos += d * dt * v
            axis.settings.set('hw_position', pos)
        else:
            pos = axis.settings.get('hw_position')

        return int(round(pos))

    def read_encoder(self, encoder):
        """
        returns encoder position.
        unit : 'encoder steps'
        """
        if self.__encoders[encoder]["steps"] is not None:
            enc_steps = self.__encoders[encoder]["steps"]
        else:
            axis = self.__encoders[encoder]["axis"]

            _pos = self.read_position(axis) / float(axis.steps_per_unit)

            if self.__encoders[encoder]["measured_noise"] > 0:
                # Simulates noisy encoder.
                amplitude = self.__encoders[encoder]["measured_noise"]
                noise_mm = random.uniform(-amplitude, amplitude)

                _pos += noise_mm

                enc_steps = _pos * encoder.steps_per_unit
            else:
                # "Perfect" encoder
                enc_steps = _pos * encoder.steps_per_unit

        self.__encoders[encoder]["steps"] = None

        return enc_steps

    def set_encoder(self, encoder, encoder_steps):
        self.__encoders[encoder]["steps"] = encoder_steps

    """
    VELOCITY
    """
    def read_velocity(self, axis):
        """
        Returns the current velocity taken from controller
        in motor units.
        """
        return axis.settings.get('velocity')*abs(axis.steps_per_unit)

    def set_velocity(self, axis, new_velocity):
        """
        <new_velocity> is in motor units
        """
        vel = new_velocity/abs(axis.steps_per_unit)
        axis.settings.set('velocity', vel)
        return vel

    """
    ACCELERATION
    """
    def read_acceleration(self, axis):
        """
        must return acceleration in controller units / s2
        """
        return axis.settings.get('acceleration')*abs(axis.steps_per_unit)

    def set_acceleration(self, axis, new_acceleration):
        """
        <new_acceleration> is in controller units / s2
        """
        acc = new_acceleration/abs(axis.steps_per_unit)
        axis.settings.set('acceleration', acc) 
        return acc

    """
    ON / OFF
    """
    def set_on(self, axis):
        self._hw_state.clear()
        self._hw_state.set("READY")

    def set_off(self, axis):
        self._hw_state.set("OFF")

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
        if self._hw_state == "OFF":
            return AxisState("OFF")
        else:
            s = AxisState(self._hw_state)
            s.set("READY")
            return s

    """
    STATE
    """
    def state(self, axis):
        if self._axis_moves[axis]["end_t"] > time.time():
           return AxisState("MOVING")
        else:
           self.read_position(axis, t=self._axis_moves[axis]["end_t"])
           self._axis_moves[axis]["end_t"] = None
           self._axis_moves[axis]["delta"] = 0
           return self._check_hw_limits(axis)

    """
    Must send a command to the controller to abort the motion of given axis.
    """
    def stop(self, axis, t=None):
        if self._axis_moves[axis]["end_t"]:
            self._axis_moves[axis]["target"] = self.read_position(axis, t=t)
            self._axis_moves[axis]["end_t"] = None

    def stop_all(self, *motion_list):
        t = time.time()
        for motion in motion_list:
            self.stop(motion.axis, t=t)

    """
    HOME and limits search
    """
    def home_search(self, axis, switch):
        self._axis_moves[axis]["delta"] = switch
        self._axis_moves[axis]["end_t"] = None
        self._axis_moves[axis]["t0"] = time.time()
        self._axis_moves[axis]["home_search_start_time"] = time.time()

#    def home_set_hardware_position(self, axis, home_pos):
#        raise NotImplementedError

    def home_state(self, axis):
        if(time.time() - self._axis_moves[axis]["home_search_start_time"]) > 1:
            axis.settings.set("hw_position", 0)
            return AxisState("READY")
        else:
            return AxisState("MOVING")

    def limit_search(self, axis, limit):
        self._axis_moves[axis]["target"] = 1e6*limit*axis.steps_per_unit
        self._axis_moves[axis]["delta"] = limit
        self._axis_moves[axis]["end_t"] = time.time() + 1
        self._axis_moves[axis]["t0"] = time.time()

    def get_info(self, axis):
        return "turlututu chapo pointu : %s" % (axis.name)

    def get_id(self, axis):
        return "MOCKUP AXIS %s" % (axis.name)

    def set_position(self, axis, pos):
        if self._axis_moves[axis]["end_t"]:
            raise RuntimeError("Cannot set position while moving !")
        
        axis.settings.set('hw_position', pos)
        self._axis_moves[axis]['target'] = pos
        self._axis_moves[axis]["end_t"] = None

        return pos

    def put_discrepancy(self, axis, disc):
        self.set_position(axis, self.read_position(axis)+disc)

    """
    Custom axis methods
    """
    # VOID VOID
    @object_method
    def custom_park(self, axis):
        elog.debug("custom_park : parking")
        self._hw_state.set("PARKED")

    # VOID LONG
    @object_method(types_info=("None", "int"))
    def custom_get_forty_two(self, axis):
        return 42

    # LONG LONG  + renaming.
    @object_method(name= "CustomGetTwice", types_info=("int", "int"))
    def custom_get_twice(self, axis, LongValue):
        return LongValue * 2

    # STRING STRING
    @object_method(types_info=("str", "str"))
    def custom_get_chapi(self, axis, value):
        if value == "chapi":
            return "chapo"
        elif value == "titi":
            return "toto"
        else:
            return "bla"

    # STRING VOID
    @object_method(types_info=("str", "None"))
    def custom_send_command(self, axis, value):
        elog.debug("custom_send_command(axis=%s value=%r):" % (axis.name, value))

    # BOOL NONE
    @object_method(name="Set_Closed_Loop", types_info=("bool", "None"))
    def _set_closed_loop(self, axis, onoff = True):
        pass #print "I set the closed loop ", onoff

    # Types by default (None, None)
    @object_method
    def custom_command_no_types(self, axis):
        print "print with no types"

    @object_method
    def generate_error(self, axis):
        # For testing purposes.
        raise RuntimeError("Testing Error")

    def custom_get_measured_noise(self, axis):
        noise = 0.0
        if not axis.encoder in self.__encoders:
            raise KeyError("cannot read measured noise: %s "
                           "doesn't have encoder" % axis.name)
        noise = self.__encoders[axis.encoder].get("measured_noise", None)

    @object_method(types_info=("float", "None"))
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

    @object_attribute_get(type_info="int")
    def get_voltage(self, axis):
        return self.__voltages.setdefault(axis, 10000)

    @object_attribute_set(type_info="int")
    def set_voltage(self, axis, voltage):
        self.__voltages[axis] = voltage

    @object_attribute_get(type_info="float")
    def get_cust_attr_float(self, axis):
        return self.__cust_attr_float.setdefault(axis, 9.999)

    @object_attribute_set(type_info="float")
    def set_cust_attr_float(self, axis, value):
        self.__cust_attr_float[axis] = value

class MockupAxis(Axis):

    def __init__(self, *args, **kwargs):
        Axis.__init__(self, *args, **kwargs)

    def prepare_move(self, *args, **kwargs):
        self.backlash_move = 0
        return Axis.prepare_move(self, *args, **kwargs)

    def _handle_move(self, motion, polling_time):
        self.target_pos = motion.target_pos
        self.backlash_move = motion.target_pos / \
            self.steps_per_unit if motion.backlash else 0
        return Axis._handle_move(self, motion, polling_time)


class MockupHook(MotionHook):
    """Motion hook used for pytest"""

    class Error(Exception):
        """Mockup hook error"""
        pass

    def __init__(self, name, config):
        super(MockupHook, self).__init__()
        self.name = name
        self.config = config
        self.nb_pre_move = 0
        self.nb_post_move = 0
        self.last_pre_move_args = ()
        self.last_post_move_args = ()

    def pre_move(self, motion_list):
        if self.config.get('pre_move_error', False):
            raise self.Error('cannot pre_move')
        self.nb_pre_move += 1
        self.last_pre_move_args = motion_list

    def post_move(self, motion_list):
        if self.config.get('post_move_error', False):
            raise self.Error('cannot post_move')
        self.nb_post_move += 1
        self.last_post_move_args = motion_list
