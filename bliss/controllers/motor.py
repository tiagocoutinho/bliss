
import types
import functools
from bliss.config.motors.static import StaticConfig
from bliss.controllers.motor_settings import ControllerAxisSettings
from bliss.common.axis import AxisRef
from bliss.controllers.motor_group import Group
from bliss.config.motors import get_axis
from bliss.common import event


def add_axis_method(axis_object, method, tango_types=None, name=None, args=[]):
    if name is None:
        name = method.im_func.func_name

    def call(self, *args, **kwargs):
        return method.im_func(method.im_self, *args, **kwargs)

    setattr(axis_object, name, types.MethodType(
        functools.partial(call, *([axis_object] + args)), axis_object))

    axis_object.add_custom_method_in_list(
        types.MethodType( functools.partial(call, *([axis_object] + args)), axis_object),
        name, tango_types)


class Controller(object):

    def __init__(self, name, config, axes):
        self.__name = name
        self.__config = StaticConfig(config)
        self.__initialized_axis = dict()
        self._axes = dict()
        self._tagged = dict()

        self.axis_settings = ControllerAxisSettings()

        for axis_name, axis_class, axis_config in axes:
            axis = axis_class(axis_name, self, axis_config)
            self._axes[axis_name] = axis
            axis_tags = axis_config.get('tags')
            if axis_tags:
                for tag in axis_tags.split():
                    self._tagged.setdefault(tag, []).append(axis)  # _name)
            self.__initialized_axis[axis] = False

    @property
    def axes(self):
        return self._axes

    @property
    def name(self):
        return self.__name

    @property
    def config(self):
        return self.__config

    def _update_refs(self):
        for axis in self.axes.itervalues():
            if not isinstance(axis, AxisRef):
                continue
            referenced_axis = get_axis(axis.name)
            self.axes[axis.name] = referenced_axis
            self.__initialized_axis[referenced_axis] = True
            for tag, axis_list in self._tagged.iteritems():
                try:
                    i = axis_list.index(axis)
                except ValueError:
                    continue
                else:
                    axis_list[i] = referenced_axis
                    referenced_axis.controller._tagged.setdefault(
                        tag,
                        []).append(referenced_axis)

    def initialize(self):
        pass

    def finalize(self):
        pass

    def get_axis(self, axis_name):
        axis = self._axes[axis_name]

        if not self.__initialized_axis[axis]:
            self.initialize_axis(axis)
            self.__initialized_axis[axis] = True

            # load settings
            axis.settings.load_from_config()

            # apply settings or config parameters
            def get_setting_or_config_value(name, converter=float):
                value = axis.settings.get(name)
                if value is None:
                    try:
                        value = axis.config.get(name, converter)
                    except:
                        pass
                return value

            for setting_name in ('velocity', 'acctime'):
                value = get_setting_or_config_value(setting_name)
                if value is None:
                    continue
                meth = getattr(axis, setting_name)
                meth(value)

            low_limit = get_setting_or_config_value("low_limit")
            high_limit = get_setting_or_config_value("high_limit")
            axis.limits(low_limit, high_limit)

        return axis

    def initialize_axis(self, axis):
        raise NotImplementedError

    def prepare_move(self, motion):  # axis, target_pos, delta):
        return

    def start_one(self, motion):
        raise NotImplementedError

    def start_all(self, *motion_list):
        raise NotImplementedError

    def stop(self, axis):
        raise NotImplementedError

    def stop_all(self):
        raise NotImplementedError

    def state(self, axis):
        raise NotImplementedError

    def home_search(self, axis):
        raise NotImplementedError

    def home_set_hardware_position(self, axis, home_pos):
        raise NotImplementedError

    def home_state(self, axis):
        raise NotImplementedError

    def read_position(self, axis, measured=False):
        raise NotImplementedError

    def set_position(self, axis, new_position):
        raise NotImplementedError

    def read_velocity(self, axis):
        raise NotImplementedError

    def set_velocity(self, axis, new_velocity):
        raise NotImplementedError

    def set_on(self, axis):
        raise NotImplementedError

    def set_off(self, axis):
        raise NotImplementedError

    def read_acctime(self, axis):
        raise NotImplementedError

    def set_acctime(self, axis, new_acctime):
        raise NotImplementedError


class CalcController(Controller):

    def __init__(self, *args, **kwargs):
        Controller.__init__(self, *args, **kwargs)

        self._reals_group = None

    def _update_refs(self):
        Controller._update_refs(self)

        self.reals = []
        for real_axis in self._tagged['real']:
            self.reals.append(real_axis)
            event.connect(real_axis, 'position', self._calc_from_real)
            event.connect(real_axis, 'state', self._update_state_from_real)
        self._reals_group = Group(self.reals)
        self.pseudos = [
            axis for axis_name,
            axis in self.axes.iteritems() if axis not in self.reals]

    def _calc_from_real(self, *args, **kwargs):
        real_positions = self._reals_group.position()

        for tag, axis_list in self._tagged.iteritems():
            if len(axis_list) > 1:
                continue
            axis = axis_list[0]
            if axis in self.reals:
                real_positions[tag] = real_positions[axis]
                del real_positions[axis]

        new_positions = self.calc_from_real(real_positions)

        for tagged_axis_name, position in new_positions.iteritems():
            axis = self._tagged[tagged_axis_name][0]
            if axis in self.pseudos:
                self.set_position(axis, position)
            else:
                raise RuntimeError("cannot assign position to real motor")

    def calc_from_real(self, real_positions):
        """Return a dict { pseudo motor tag: new position, ... }"""
        raise NotImplementedError

    def _update_state_from_real(self, *args, **kwargs):
        state = self._reals_group.state()
        for axis in self.pseudos:
            axis.settings.set("state", state, write=False)

    def initialize_axis(self, axis):
        if axis in self.pseudos:
            self._calc_from_real()
            self._update_state_from_real()

    def start_one(self, motion):
        positions_dict = dict()
        axis_tag = None
        for tag, axis_list in self._tagged.iteritems():
            if len(axis_list) > 1:
                continue
            x = axis_list[0]
            if x in self.pseudos:
                if x == motion.axis:
                    axis_tag = tag
                    positions_dict[tag] = motion.target_pos
                else:
                    positions_dict[tag] = x.position()

        move_dict = dict()
        for axis_tag, target_pos in self.calc_to_real(axis_tag, positions_dict).iteritems():
            real_axis = self._tagged[axis_tag][0]
            move_dict[real_axis] = target_pos
        self._reals_group.move(move_dict, wait=False)

    def calc_to_real(self, axis_tag, positions_dict):
        raise NotImplementedError

    def stop(self, axis):
        self._reals_group.stop()

    def read_position(self, axis, measured=False):
        return axis.settings.get('position')

    def set_position(self, axis, new_pos):
        axis.settings.set('position', new_pos)

    def state(self, axis, new_state=None):
        return self._reals_group.state()
