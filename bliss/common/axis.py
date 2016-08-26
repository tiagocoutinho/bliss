# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

from bliss.common import log as elog
from bliss.common.task_utils import *
from bliss.controllers.motor_settings import AxisSettings
from bliss.common import event
import time
import gevent
import re
import types

DEFAULT_POLLING_TIME = 0.02


class Null(object):
    __slots__ = []


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
    def lazy_init(func):
        def func_wrapper(self, *args, **kwargs):
            self.controller._initialize_axis(self)
            return func(self, *args, **kwargs)
        return func_wrapper

    def __init__(self, name, controller, config):
        self.__name = name
        self.__controller = controller
        from bliss.config.motors import StaticConfig
        self.__config = StaticConfig(config)
        self.__settings = AxisSettings(self)
        self.__move_done = gevent.event.Event()
        self.__move_done.set()
        self.__custom_methods_list = list()
        self.__custom_attributes_dict = dict()
        self.__move_task = None
        self.__stopped = False
        self.no_offset = False

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
    def _hw_control(self):
        """Return whether axis is currently driving hardware"""
        if self.__move_task is not None:
            return self.is_moving 
        return False

    @property
    def offset(self):
        offset = self.settings.get("offset")
        if offset is None:
            offset = 0
            self.settings.set('offset', 0)
        return offset

    @property
    def backlash(self):
        return self.config.get("backlash", float, 0)

    @property
    def sign(self):
        return self.config.get("sign", int, 1)

    @property
    def steps_per_unit(self):
        return self.config.get("steps_per_unit", float, 1)

    @property
    def tolerance(self):
        return self.config.get("tolerance", float, 1E-4)

    @property
    def encoder(self):
        try:
            encoder_name = self.config.get("encoder")
        except KeyError:
            return None
        else:
            from bliss.config.motors import get_encoder
            return get_encoder(encoder_name)

    @property
    def custom_methods_list(self):
        # Returns a *copy* of the custom methods list.
        return self.__custom_methods_list[:]

    @property
    def custom_attributes_list(self):
        ad = self.__custom_attributes_dict

        # Converts dict into list...
        _attr_list = [(a_name, ad[a_name][0], ad[a_name][1]) for i, a_name in enumerate(ad)]

        # Returns a *copy* of the custom attributes list.
        return _attr_list[:]

    def set_setting(self, *args):
        self.settings.set(*args)

    def get_setting(self, *args):
        return self.settings.get(*args)

    def has_tag(self, tag):
        for t, axis_list in self.__controller._tagged.iteritems():
            if t != tag:
                continue
            if self.name in [axis.name for axis in axis_list]:
                return True
        return False

    def _add_custom_method(self, method, name, types_info=(None, None)):
        setattr(self, name, method)
        self.__custom_methods_list.append((name, types_info))

    @lazy_init
    def on(self):
        if self.is_moving:
            return

        self.__controller.set_on(self)
        state = self.__controller.state(self)
        self.settings.set("state", state)

    @lazy_init
    def off(self):
        if self.is_moving:
            raise RuntimeError("Can't set power off while axis is moving")

        self.__controller.set_off(self)
        state = self.__controller.state(self)
        self.settings.set("state", state)

    def reset(self):
        if self.is_moving:
            raise RuntimeError("Can't reset while axis is moving")
        self.__controller.finalize_axis(self)
        self.__controller._initialize_axis(self)

    @lazy_init
    def _set_position(self, new_set_pos=None):
        if new_set_pos is None:
            sp = self.settings.get("_set_position")
            if sp is not None:
                return sp
            new_set_pos = self.position()
        self.settings.set("_set_position", new_set_pos)
        return new_set_pos

    @lazy_init
    def measured_position(self):
        """
        Returns the encoder value in user units.
        """
        return self.dial2user(self.dial_measured_position())

    @lazy_init
    def dial_measured_position(self):
        """
        Returns the dial encoder position.
        """
        return self.__controller.read_encoder(self.encoder) / self.encoder.steps_per_unit

    @lazy_init
    def dial(self, new_dial=None):
        """
        Returns current dial position, or set new dial if 'new_dial' argument is provided
        """
        if new_dial is None:
            dial_pos = self.settings.get("dial_position")
            if dial_pos is None:
                dial_pos = self._read_dial_and_update() 
            return dial_pos

        if self.is_moving:
            raise RuntimeError("%s: can't set axis dial position " 
                               "while moving" % self.name)

        user_pos = self.position()

        try:
            # Send the new value in motor units to the controller
            # and read back the (atomically) reported position
            new_hw = new_dial * self.steps_per_unit
            hw_pos = self.__controller.set_position(self, new_hw)
            dial_pos = hw_pos / self.steps_per_unit
            self.settings.set("dial_position", dial_pos)
        except NotImplementedError:
            dial_pos = self._read_dial_and_update(update_user=False)

        # update user_pos or offset setting
        if self.no_offset:
            user_pos = dial_pos 
        self._set_position_and_offset(user_pos)
        return dial_pos

    @lazy_init
    def position(self, new_pos=None):
        """
        if <new_pos> is None : Gets axis *user* position.
        else sets axis *user* position.
        * <new_pos> is in user units.
        * Return value is in user units.
        """
        elog.debug("axis.py : position(new_pos=%r)" % new_pos)
        if new_pos is None:
            pos = self.settings.get("position")
            if pos is None:
                pos = self.dial2user(self.dial())
            return pos

        if self.is_moving:
            raise RuntimeError("%s: can't set axis user position "
                               "while moving" % self.name)

        if self.no_offset:
            return self.dial(new_pos)
        else:
            return self._set_position_and_offset(new_pos)

    @lazy_init 
    def _read_dial_and_update(self, update_user=True, write=True):
        dial_pos = self._hw_position()
        self.settings.set("dial_position", dial_pos, write=write)
        if update_user:
            user_pos = self.dial2user(dial_pos, self.offset)
            self.settings.set("position", user_pos, write=write)
        return dial_pos

    def _hw_position(self):
        try:
            curr_pos = self.__controller.read_position(self) / self.steps_per_unit
        except NotImplementedError:
            # this controller does not have a 'position'
            # (e.g like some piezo controllers)
            curr_pos = 0
        return curr_pos

    def _calc_offset(self, new_pos, dial_pos):
        return new_pos - self.sign * dial_pos

    def _set_position_and_offset(self, new_pos):
        dial_pos = self.dial()
        prev_offset = self.offset
        self._set_position(new_pos)
        self.settings.set("offset", self._calc_offset(new_pos, dial_pos))
        # update limits
        ll, hl = self.limits()
        lim_delta = self.offset - prev_offset
        self.limits(ll + lim_delta if ll is not None else ll,
                    hl + lim_delta if hl is not None else hl)
        self.settings.set("position", new_pos, write=True)
        return new_pos

    @lazy_init
    def state(self, read_hw=False):
        if read_hw:
            state = None
        else:
            if self.is_moving:
                return AxisState("MOVING")
            state = self.settings.get_from_channel('state')

        if state is None:
            # really read from hw
            state = self.__controller.state(self)
        return state

    @lazy_init
    def get_info(self):
        return self.__controller.get_info(self)

    def sync_hard(self):
        self.settings.set("state", self.state(read_hw=True), write=True) 
        self._read_dial_and_update()
        self._set_position(self.position())
        event.send(self, "sync_hard")
        
    @lazy_init
    def velocity(self, new_velocity=None, from_config=False):
        """
        <new_velocity> is given in user units per seconds.
        """
        if from_config:
            return self.config.get("velocity", float)

        if new_velocity is not None:
            # Write -> Converts into motor units to change velocity of axis."
            self.__controller.set_velocity(
                self, new_velocity * abs(self.steps_per_unit))
            _user_vel = new_velocity
        else:
            # Read -> Returns velocity read from motor axis.
            _user_vel = self.settings.get_from_channel('velocity')
            if _user_vel is None:
                _user_vel = self.__controller.read_velocity(self) / abs(self.steps_per_unit)

        # In all cases, stores velocity in settings in uu/s
        self.settings.set("velocity", _user_vel)
        return _user_vel

    @lazy_init
    def acceleration(self, new_acc=None, from_config=False):
        """
        <new_acc> is given in user_units/s2.
        """
        if from_config:
            return self.config.get("acceleration", float)

        if new_acc is not None:
            if self.is_moving:
                raise RuntimeError("Cannot set acceleration while axis '%s` is moving." % self.name)

            # Converts into motor units to change acceleration of axis.
            self.__controller.set_acceleration(self, new_acc * abs(self.steps_per_unit))
        else:
            _acceleration = self.settings.get_from_channel('acceleration')
            if _acceleration is not None:
                return _acceleration

        # Both R or W : Reads acceleration from controller.
        _ctrl_acc = self.__controller.read_acceleration(self)
        _acceleration = _ctrl_acc / abs(self.steps_per_unit)

        if new_acc is not None:
            # W => save acceleration in settings in uu/s2.
            self.settings.set("acceleration", _acceleration)

        return _acceleration

    @lazy_init
    def acctime(self, new_acctime=None, from_config=False):
        """
        <new_acctime> given in seconds.
        """
        if from_config:
            return self.velocity(from_config=True) / self.acceleration(from_config=True)

        if new_acctime is not None:
            # Converts acctime into acceleration.
            acc = self.velocity() / new_acctime
            self.acceleration(acc)

        _acctime = self.velocity() / self.acceleration()

        return _acctime

    @lazy_init
    def limits(self, low_limit=Null(), high_limit=Null(), from_config=False):
        """
        <low_limit> and <high_limit> given in user units.
        """
        if from_config:
            ll = self.config.get("low_limit", float, None)
            hl = self.config.get("high_limit", float, None)
            return map(self.dial2user, (ll, hl))
        if not isinstance(low_limit, Null):
            self.settings.set("low_limit", low_limit)
        if not isinstance(high_limit, Null):
            self.settings.set("high_limit", high_limit)
        return self.settings.get('low_limit'), self.settings.get('high_limit')

    def _update_settings(self, state=None):
        self.settings.set("state", state if state is not None else self.state(), write=self._hw_control) 
        self._read_dial_and_update(write=self._hw_control)
 
    def _handle_move(self, motion, polling_time):
        state = self._wait_move(polling_time, update_settings=True)
        if state in ['LIMPOS', 'LIMNEG']:
            raise RuntimeError(str(state))

        # gevent-atomic
        stopped, self.__stopped = self.__stopped, False
        if stopped or motion.backlash:
            dial_pos = self._read_dial_and_update()
            user_pos = self.dial2user(dial_pos)

        if motion.backlash:
            # broadcast reached position before backlash correction
            backlash_start = motion.target_pos
            if stopped:
                self._set_position(user_pos + self.backlash)
                backlash_start = dial_pos * self.steps_per_unit
            # axis has moved to target pos - backlash (or shorter, if stopped);
            # now do the final motion (backlash) relative to current/theo. pos
            elog.debug("doing backlash (%g)" % motion.backlash)
            final_pos = backlash_start + motion.backlash
            backlash_motion = Motion(self, final_pos, motion.backlash)
            self.__controller.prepare_move(backlash_motion)
            self.__controller.start_one(backlash_motion)
            self._handle_move(backlash_motion, polling_time)
        elif stopped:
            self._set_position(user_pos)
        elif self.encoder is not None:
            self._do_encoder_reading()

    def dial2user(self, position, offset=None):
        if position is None:
            # see limits()
            return None
        if offset is None:
            offset = self.offset
        return (self.sign * position) + offset

    def user2dial(self, position):
        return (position - self.offset) / self.sign

    @lazy_init
    def prepare_move(self, user_target_pos, relative=False):
        elog.debug("user_target_pos=%g, relative=%r" % (user_target_pos, relative))
        user_initial_dial_pos = self.dial()
        hw_pos = self._read_dial_and_update()

        elog.debug("hw_position=%g user_initial_dial_pos=%g" % (hw_pos, user_initial_dial_pos))

        if abs(user_initial_dial_pos - hw_pos) > self.tolerance:
            raise RuntimeError(
					"%s: discrepancy between dial (%f) and controller position (%f), aborting" % (
                     self.name, user_initial_dial_pos, hw_pos))

        if relative:
            user_initial_pos = self._set_position()
            user_target_pos += user_initial_pos
        else:
            user_initial_pos = self.dial2user(user_initial_dial_pos)

        dial_initial_pos = self.user2dial(user_initial_pos)
        dial_target_pos = self.user2dial(user_target_pos)
        self._set_position(user_target_pos)
        if abs(dial_target_pos - dial_initial_pos) < 1E-6:
            return

        elog.debug("prepare_move : user_initial_pos=%g user_target_pos=%g" %
                   (user_initial_pos, user_target_pos) +
                   "  dial_target_pos=%g dial_intial_pos=%g relative=%s" %
                   (dial_target_pos, dial_initial_pos, relative))

        # all positions are converted to controller units
        backlash = self.backlash / self.sign * self.steps_per_unit
        delta = (dial_target_pos - dial_initial_pos) * self.steps_per_unit
        target_pos = dial_target_pos * self.steps_per_unit

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

        # check software limits
        user_low_limit, user_high_limit = self.limits()
        if user_low_limit is not None:
            low_limit = self.user2dial(user_low_limit) * self.steps_per_unit
        else:
            low_limit = None
        if user_high_limit is not None:
            high_limit = self.user2dial(user_high_limit) * self.steps_per_unit
        else:
            high_limit = None
        if high_limit is not None and high_limit < low_limit:
            high_limit, low_limit = low_limit, high_limit
            user_high_limit, user_low_limit = user_low_limit, user_high_limit

        backlash_str = " (with %f backlash)" % self.backlash if backlash else ""
        if user_low_limit is not None:
            if target_pos < low_limit:
                raise ValueError(
                    "%s: move to `%f'%s would go below low limit (%f)" %
                    (self.name, user_target_pos, backlash_str, user_low_limit))
        if user_high_limit is not None:
            if target_pos > high_limit:
                raise ValueError(
                    "%s: move to `%f' %s would go beyond high limit (%f)" %
                    (self.name, user_target_pos, backlash_str, user_high_limit))

        motion = Motion(self, target_pos, delta)
        motion.backlash = backlash

        self.__controller.prepare_move(motion)

        return motion

    def _set_moving_state(self, from_channel=False):
        self.__stopped = False
        self.__move_done.clear()
        if from_channel:
            self.__move_task = None
        self.settings.set("state", AxisState("MOVING"), write=not from_channel)
        event.send(self, "move_done", False)

    def _set_move_done(self, move_task):
        if move_task is not None:
            if not move_task._being_waited:
                try:
                    move_task.get()
                except gevent.GreenletExit:
                    pass
                except:
                    sys.excepthook(*sys.exc_info())
            # update settings;
            # as update is done before move done is set,
            # we need to read state from hardware to get it right
            # (it would return 'MOVING' otherwise)
            # this update is very important for position, to have
            # final position ok for waiters on move done event
            self._update_settings(state=self.state(read_hw=True))
        self.__move_done.set()
        event.send(self, "move_done", True)

    def _check_ready(self):
        initial_state = self.state(read_hw=True)
        if initial_state != "READY":
            raise RuntimeError("axis %s state is \
                                %r" % (self.name, str(initial_state)))

    @lazy_init
    def move(self, user_target_pos, wait=True, relative=False, polling_time=DEFAULT_POLLING_TIME):
        elog.debug("user_target_pos=%g  wait=%r relative=%r" % (user_target_pos, wait, relative))
        if self.__controller.is_busy():
            raise RuntimeError("axis %s: controller is busy" % self.name)
        self._check_ready()

        motion = self.prepare_move(user_target_pos, relative)
        if motion is None:
            return

        with error_cleanup(self._cleanup_stop):
            self.__controller.start_one(motion)
        
        self.__move_task = self._do_handle_move(motion, polling_time,wait=False)
        self._set_moving_state()
        self.__move_task._being_waited = wait
        self.__move_task.link(self._set_move_done)

        if wait:
            self.wait_move()

    def _do_encoder_reading(self):
        enc_dial = self.encoder.read()
        curr_pos = self._read_dial_and_update()
        if abs(curr_pos - enc_dial) > self.encoder.tolerance:
            raise RuntimeError("'%s' didn't reach final position.(enc_dial=%g, curr_pos=%g)" %
                               (self.name, enc_dial, curr_pos))

    @task
    def _do_handle_move(self, motion, polling_time):
        with error_cleanup(self._cleanup_stop):
            self._handle_move(motion, polling_time)

    def rmove(self, user_delta_pos, wait=True, polling_time=DEFAULT_POLLING_TIME):
        elog.debug("user_delta_pos=%g  wait=%r" % (user_delta_pos, wait))
        return self.move(user_delta_pos, wait, relative=True, polling_time=polling_time)

    def wait_move(self):
        if not self.is_moving:
            return
        if self.__move_task is None:
            # move has been started externally
            with error_cleanup(self.stop):
                self.__move_done.wait()
        else:
            self.__move_task._being_waited = True
            with error_cleanup(self.stop):
                self.__move_done.wait()
            try:
                self.__move_task.get()
            except gevent.GreenletExit:
                pass

    def _wait_move(self, polling_time=DEFAULT_POLLING_TIME,
                   update_settings=False, ctrl_state_funct='state'):
        while True:
            state_funct = getattr(self.__controller, ctrl_state_funct)
            state = state_funct(self)
            if state != "MOVING":
                return state
            if update_settings:
                self._update_settings(state)
            gevent.sleep(polling_time)
        
    def _cleanup_stop(self):
        self.__controller.stop(self)
        self._wait_move()
        self.sync_hard()

    def _do_stop(self):
        self.__controller.stop(self)
        self._set_stopped()

    def _set_stopped(self):
        self.__stopped = True

    @lazy_init
    def stop(self, wait=True):
        if self.is_moving:
            self._do_stop()
            if wait:
                self.wait_move()

    @lazy_init
    def home(self, switch=1, wait=True):
        self._check_ready()

        self.__controller.home_search(self, switch)
        self.__move_task = self._wait_home(switch, wait=False)
        self._set_moving_state()
        self.__move_task._being_waited = wait
        self.__move_task.link(self._set_move_done)

        if wait:
            self.wait_move()

    @task
    def _wait_home(self, switch):
        with cleanup(self.sync_hard):
            with error_cleanup(self._cleanup_stop):
                self._wait_move(ctrl_state_funct='home_state')

    @lazy_init
    def hw_limit(self, limit, wait=True):
        """Go to a hardware limit

        Parameters:
            limit   - integer, positive means "positive limit"
            wait    - boolean, wait for completion (default is to wait)
        """
        limit = int(limit)
        self._check_ready()

        self.__controller.limit_search(self, limit)
        self.__move_task = self._wait_limit_search(limit, wait=False)
        self._set_moving_state()
        self.__move_task._being_waited = wait
        self.__move_task.link(self._set_move_done)

        if wait:
            self.wait_move()

    @task
    def _wait_limit_search(self, limit):
        with cleanup(self.sync_hard):
            with error_cleanup(self._cleanup_stop):
                self._wait_move()

    def settings_to_config(self, velocity=True, acceleration=True, limits=True):
        """
        Saves settings (velo acc limits) into config (XML file or beacon YML).
        """
        if velocity:
            self.__config.set('velocity', self.velocity())
        if acceleration:
            self.__config.set('acceleration', self.acceleration())
        if limits:
            def limit2config(l):
                return self.user2dial(l) if l is not None else l
            ll, hl = map(limit2config, self.limits())
            self.__config.set('low_limit', ll)
            self.__config.set('high_limit', hl)
        if any((velocity, acceleration, limits)):
            self.__config.save()

    def apply_config(self, reload=True):
        """
        Applies configuration values to settings (ie: reset axis)
        """
        if reload:
            self.config.reload()

        # Applies velocity and acceleration only if possible.
        # Try to execute <config_name> function to check if axis supports it.
        for config_param in ['velocity', 'acceleration']:
            rw_function = getattr(self, config_param)
            try:
                rw_function(rw_function(from_config=True))
            except (NotImplementedError, KeyError):
                elog.debug("'%s' for '%s' is not implemented" % (config_param, self.name))
            else:
                elog.debug("set '%s' for '%s' done." % (config_param, self.name))

        self.limits(*self.limits(from_config=True))


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


class AxisState(object):
    """
    Standard states:
      MOVING : 'Axis is moving'
      READY  : 'Axis is ready to be moved (not moving ?)'
      FAULT  : 'Error from controller'
      LIMPOS : 'Hardware high limit active'
      LIMNEG : 'Hardware low limit active'
      HOME   : 'Home signal active'
      OFF    : 'Axis is disabled (must be enabled to move (not ready ?))'
    """

    STATE_VALIDATOR = re.compile("^[A-Z0-9]+\s*$")

    _STANDARD_STATES = {
        "READY" : "Axis is READY",
        "MOVING": "Axis is MOVING",
        "FAULT" : "Error from controller",
        "LIMPOS": "Hardware high limit active",
        "LIMNEG": "Hardware low limit active",
        "HOME"  : "Home signal active",
        "OFF"   : "Axis is disabled (must be enabled to move (not ready ?))"
    }

    @property
    def READY(self):
        return "READY" in self._current_states

    @property
    def MOVING(self):
        return "MOVING" in self._current_states

    @property
    def FAULT(self):
        return "FAULT" in self._current_states

    @property
    def LIMPOS(self):
        return "LIMPOS" in self._current_states

    @property
    def LIMNEG(self):
        return "LIMNEG" in self._current_states

    @property
    def OFF(self):
        return "OFF" in self._current_states

    @property
    def HOME(self):
        return "HOME" in self._current_states

    def __init__(self, *states):
        """
        <*states> : can be one or many string or tuple of strings (state, description)
        """

        # set of active states.
        self._current_states = list()

        # dict of descriptions of states.
        self._state_desc = self._STANDARD_STATES

        for state in states:
            if isinstance(state, tuple):
                self.create_state(*state)
                self.set(state[0])
            else:
                if isinstance(state, AxisState):
                    state = state.current_states()
                self._set_state_from_string(state)

    def states_list(self):
        """
        Returns a list of available/created states for this axis.
        """
        return list(self._state_desc)

    def _check_state_name(self, state_name):
        if not isinstance(state_name, str) or not AxisState.STATE_VALIDATOR.match(state_name):
            raise ValueError(
                "Invalid state: a state must be a string containing only block letters")

    def _has_custom_states(self):
        return not self._state_desc is AxisState._STANDARD_STATES

    def create_state(self, state_name, state_desc=None):
        # Raises ValueError if state_name is invalid.
        self._check_state_name(state_name)
        if state_desc is not None and '|' in state_desc:
            raise ValueError("Invalid state: description contains invalid character '|'")

        # if it is the first time we are creating a new state, create a
        # private copy of standard states to be able to modify locally
        if not self._has_custom_states():
            self._state_desc = AxisState._STANDARD_STATES.copy()

        if state_name not in self._state_desc:
            # new description is put in dict.
            if state_desc is None:
                state_desc = "Axis is %s" % state_name
            self._state_desc[state_name] = state_desc

            # Makes state accessible via a class property.
            # NO: we can't, because the objects of this class will become unpickable,
            # as the class changes...
            # Error message is: "Can't pickle class XXX: it's not the same object as XXX"
            # add_property(self, state_name, lambda _: state_name in self._current_states)

    """
    Flags ON a given state.
    ??? what about other states : clear other states ???  -> MG : no
    ??? how to flag OFF ???-> no : on en cree un nouveau.
    """
    def set(self, state_name):
        if state_name in self._state_desc:
            if state_name not in self._current_states:
                self._current_states.append(state_name)

                # Mutual exclusion of READY and MOVING
                if state_name == "READY":
                    if self.MOVING:
                        self._current_states.remove("MOVING")
                if state_name == "MOVING":
                    if self.READY:
                        self._current_states.remove("READY")
        else:
            raise ValueError("state %s does not exist" % state_name)

    def clear(self):
        # Flags all states off.
        self._current_states = list()

    def current_states(self):
        """
        Returns a string of current states.
        """
        states = [
            "%s%s" % (state.rstrip(), " (%s)" % self._state_desc[state] if self._state_desc.get(state) else "")
            for state in map(str, list(self._current_states))]

        if len(states) == 0:
            return "UNKNOWN"

        return " | ".join(states)

    def _set_state_from_string(self, state):
        # is state_name a full list of states returned by self.current_states() ?
        # (copy constructor)
        if '(' in state:
            full_states = [s.strip() for s in state.split('|')]
            p = re.compile('^([A-Z]+)\s\((.+)\)$')
            for full_state in full_states:
                m = p.match(full_state)
                state = m.group(1)
                desc = m.group(2)
                self.create_state(state, desc)
                self.set(state)
        else:
            if state != 'UNKNOWN':
                self.create_state(state)
                self.set(state)

    def __str__(self):
        return self.current_states()

    def __eq__(self, other):
        if isinstance(other, AxisState):
            other = str(other)
        if isinstance(other, str):
            state = self.current_states()
            return other in state
        return NotImplemented

    def __ne__(self, other):
        return not self.__eq__(other)

    def new(self, share_states=True):
        """
        Create a new AxisState which contains the same possible states but no
        current state.

        If this AxisState contains custom states and *share_states* is True
        (default), the possible states are shared with the new AxisState.
        Otherwise, a copy of possible states is created for the new AxisState.

        Keyword Args:
            share_states: If this AxisState contains custom states and
                          *share_states* is True (default), the possible states
                          are shared with the new AxisState. Otherwise, a copy
                          of possible states is created for the new AxisState.

        Returns:
            AxisState: a copy of this AxisState with no current states
        """
        result = AxisState()
        if self._has_custom_states() and not share_states:
            result._state_desc = self._state_desc.copy()
        else:
            result._state_desc = self._state_desc
        return result
