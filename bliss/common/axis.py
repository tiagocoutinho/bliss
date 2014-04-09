
from bliss.common.task_utils import *
from bliss.config.motors.static import StaticConfig
from bliss.controllers.motor_settings import AxisSettings
import time

READY, MOVING, FAULT, UNKNOWN = ("READY", "MOVING", "FAULT", "UNKNOWN")


class Motion(object):

    def __init__(self, axis, target_pos, delta):
        self.__axis = axis
        self.target_pos = target_pos
        self.delta = delta
        self.backlash = 0

    @property
    def axis(self):
        return self.__axis


class Axis(object):

    def __init__(self, name, controller, config):
        self.__name = name
        self.__controller = controller
        self.__config = StaticConfig(config)
        self.__settings = AxisSettings(self)
        self.__settings.set("offset", 0)
        self.__move_done = gevent.event.Event()
        self.__move_done.set()

    @property
    def name(self):
        return self.__name

    @property
    def controller(self):
        return self.__controller

    @property
    def config(self):
        return self.__config

    @property
    def settings(self):
        return self.__settings

    @property
    def is_moving(self):
        return not self.__move_done.is_set()

    @property
    def offset(self):
        return self.__settings.get("offset")

    def has_tag(self, tag):
        for t, axis_list in self.__controller._tagged.iteritems():
            if t != tag:
                continue
            if self.name in [axis.name for axis in axis_list]:
                return True
        return False

    def measured_position(self):
        return self.__controller.read_position(self, measured=True)

    def step_size(self):
        return self.config.get("step_size", float, 1)

    def position(self, new_pos=None):
        if self.is_moving:
            if new_pos is not None:
                raise RuntimeError("Can't set axis position \
                                    while it is moving")
            pos = self.settings.get("position")
            if pos is None:
                pos = self._position()
                self.settings.set("position", pos)
            return pos
        else:
            pos = self._position(new_pos)
            if new_pos is not None:
                self.settings.set("position", pos)
            return pos

    def _position(self, new_pos=None, measured=False):
        new_pos = new_pos * self.step_size() if new_pos is not None else None
        if new_pos is not None:
            try:
                return self.__controller.set_position(self, new_pos) / self.step_size()
            except NotImplementedError:
                self.__settings.set("offset", (self.__controller.read_position(self) - new_pos) / self.step_size())
                return self.position()
        else:
            return (self.__controller.read_position(self, measured) / self.step_size()) - self.offset

    def state(self):
        if self.is_moving:
            return MOVING
        # really read from hw
        return self.__controller.state(self)

    def velocity(self, new_velocity=None):
        if new_velocity is not None:
            _vel = self.__controller.set_velocity(self, new_velocity)
        else:
            _vel = self.__controller.read_velocity(self)
        self.settings.set("velocity", _vel)
        return _vel

    def acctime(self, new_acctime=None):
        if new_acctime is not None:
            _acctime = self.__controller.set_acctime(self, new_acctime)
        else:
            _acctime = self.__controller.read_acctime(self)
        self.settings.set("acctime", _acctime)
        return _acctime

    def _handle_move(self, motion):
        def update_settings():
            state = self.__controller.state(self)
            self.settings.set("state", state, write=False)
            pos = self._position()
            self.settings.set("position", pos)
            return state

        with cleanup(update_settings):
            while True:
                state = self.__controller.state(self)
                self.settings.set("state", state, write=False)
                if state != MOVING:
                    break
                pos = self._position()
                self.settings.set("position", pos, write=False)
                time.sleep(0.02)

            if motion.backlash:
                # axis has moved to target pos - backlash;
                # now do the final motion to reach original target
                final_pos = motion.target_pos + motion.backlash
                backlash_motion = Motion(self, final_pos, motion.backlash)
                self.__controller.prepare_move(backlash_motion)
                self.__controller.start_one(backlash_motion)
                self._handle_move(backlash_motion)

    def prepare_move(self, user_target_pos, relative=False):
        initial_pos = self.position()
        if relative:
            user_target_pos += initial_pos
        # all positions are converted to controller units
        backlash = self.config.get("backlash", float, 0) * self.step_size()
        delta = (user_target_pos - initial_pos) * self.step_size()
        target_pos = (user_target_pos + self.offset) * self.step_size()

        if backlash:
            if cmp(delta, 0) != cmp(backlash, 0):
                # move and backlash are not in the same direction;
                # apply backlash correction, the move will happen
                # in 2 steps
                target_pos -= backlash
                delta -= backlash
            else:
                # don't do backlash correction
                backlash = 0

        motion = Motion(self, target_pos, delta)
        motion.backlash = backlash
        self.__controller.prepare_move(motion)

        return motion

    def _set_move_done(self, move_task):
        self.__move_done.set()

    def _check_ready(self):
        initial_state = self.state()
        if initial_state != READY:
            raise RuntimeError("motor %s state is \
                                %r" % (self.name, initial_state))

    def move(self, user_target_pos, wait=True, relative=False):
        self._check_ready()

        motion = self.prepare_move(user_target_pos, relative)

        # indicates that axis is MOVING.
        self.__move_done.clear()

        move_task = self._do_move(motion, wait=False)
        move_task.link(self._set_move_done)

        if wait:
            move_task.get()
        else:
            return move_task

    @task
    def _do_move(self, motion, wait=True):
        with error_cleanup(self.stop):
            self.__controller.start_one(motion)

            self._handle_move(motion)

    def rmove(self, user_delta_pos, wait=True):
        return self.move(user_delta_pos, wait, relative=True)

    def wait_move(self):
        self.__move_done.wait()

    def stop(self):
        if self.is_moving:
            self.__controller.stop(self)
            self.__move_done.set()

    def home(self, home_pos=None, wait=True):
        self._check_ready()

        self.__move_done.clear()

        home_task = self._do_home(home_pos, wait=False)
        home_task.link(self._set_move_done)

        if wait:
            home_task.get()
        else:
            return home_task

    @task
    def _do_home(self, home_pos):
        with error_cleanup(self.stop):

            # flag "must the position to be set ?"
            _set_pos = False

            if home_pos is not None:
                try:
                    self.__controller.home_set_hardware_position(self, home_pos)
                except NotImplementedError:
                    _set_pos = True

            self.__controller.home_search(self)
            while True:
                state = self.__controller.home_state(self)
                self.settings.set("state", state, write=False)
                if state != MOVING:
                    break
                time.sleep(0.02)

        if _set_pos:
            self._position(home_pos)


class AxisRef(object):

    def __init__(self, name, _, config):
        self.__name = name
        self.__config = config
        self.settings = AxisSettings(None)

    @property
    def name(self):
        return self.__name

    @property
    def config(self):
        return self.__config
