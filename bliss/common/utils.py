# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

import inspect
import types
import itertools
import functools
from bliss.common.event import saferef

try:
    from collections import OrderedDict
except ImportError:             # python2.6 compatibility
    from ordereddict import OrderedDict


class WrappedMethod(object):
    def __init__(self, control, method_name):
        self.method_name = method_name
        self.control = control

    def __call__(self, this, *args, **kwargs):
        return getattr(self.control, self.method_name)(*args, **kwargs)


def wrap_methods(from_object, target_object):
    for name in dir(from_object):
        if inspect.ismethod(getattr(from_object, name)):
            if hasattr(target_object, name) and inspect.ismethod(getattr(target_object, name)):
                continue
            setattr(target_object, name, types.MethodType(WrappedMethod(
                from_object, name), target_object, target_object.__class__))


def add_conversion_function(obj, method_name, function):
    meth = getattr(obj, method_name)
    if inspect.ismethod(meth):
        if callable(function):
            def new_method(*args, **kwargs):
                values = meth(*args, **kwargs)
                return function(values)
            setattr(obj, method_name, new_method)
        else:
            raise ValueError("conversion function must be callable")
    else:
        raise ValueError("'%s` is not a method" % method_name)


def add_property(inst, name, method):
    cls = type(inst)
    if not hasattr(cls, '__perinstance'):
        cls = type(cls.__name__, (cls,), {})
        cls.__perinstance = True
        inst.__class__ = cls
    setattr(cls, name, property(method))


def grouped(iterable, n):
    "s -> (s0,s1,s2,...sn-1), (sn,sn+1,sn+2,...s2n-1), (s2n,s2n+1,s2n+2,...s3n-1), ..."
    return itertools.izip(*[iter(iterable)] * n)


def all_equal(iterable):
    g = itertools.groupby(iterable)
    return next(g, True) and not next(g, False)


"""
functions to add custom attributes and commands to an object.
"""


def add_object_method(obj, method, pre_call, name=None, args=[], types_info=(None, None)):

    if name is None:
        name = method.im_func.func_name

    def call(self, *args, **kwargs):
        if callable(pre_call):
            pre_call(self, *args, **kwargs)

        return method.im_func(method.im_self, *args, **kwargs)

    obj._add_custom_method(
        types.MethodType(functools.partial(call, *([obj] + args)),
                         obj), name, types_info)


def object_method(method=None, name=None, args=[], types_info=(None, None), filter=None):
    """
    Decorator to add a custom method to an object.

    The same as add_object_method but its purpose is to be used as a
    decorator to the controller method which is to be exported as object method.
    """
    if method is None:
        # Passes here if decorator parameters are not present ???.
        # ...
        return functools.partial(object_method, name=name, args=args,
                                 types_info=types_info, filter=filter)

    # Returns a method where _object_method_ attribute is filled with a
    # dict of elements to characterize it.
    method._object_method_ = dict(
        name=name, args=args, types_info=types_info, filter=filter)

    return method


def object_method_type(method=None, name=None, args=[], types_info=(None, None), type=None):
    def f(x): return isinstance(x, type)
    return object_method(method=method, name=name, args=args, types_info=types_info, filter=f)


def add_object_attribute(obj, name=None, fget=None, fset=None, args=[], type_info=None, filter=None):
    obj._add_custom_attribute(name, fget, fset, type_info)


"""
decorators for set/get methods to access to custom attributes
"""


def object_attribute_type_get(get_method=None, name=None, args=[], type_info=None, type=None):
    def f(x): return isinstance(x, type)
    return object_attribute_get(get_method=get_method, name=name, args=args, type_info=type_info, filter=f)


def object_attribute_get(get_method=None, name=None, args=[], type_info=None, filter=None):
    if get_method is None:
        return functools.partial(object_attribute_get, name=name, args=args,
                                 type_info=type_info, filter=filter)

    if name is None:
        name = get_method.func_name
    attr_name = name
    if attr_name.startswith("get_"):
        attr_name = attr_name[4:]  # removes leading "get_"

    get_method._object_method_ = dict(
        name=name, args=args, types_info=("None", type_info), filter=filter)

    if not hasattr(get_method, "_object_attribute_"):
        get_method._object_attribute_ = dict()
    get_method._object_attribute_.update(
        name=attr_name, fget=get_method, args=args, type_info=type_info, filter=filter)

    return get_method


def object_attribute_type_set(set_method=None, name=None, args=[], type_info=None, type=None):
    def f(x): return isinstance(x, type)
    return object_attribute_set(set_method=set_method, name=name, args=args, type_info=type_info, filter=f)


def object_attribute_set(set_method=None, name=None, args=[], type_info=None, filter=None):
    if set_method is None:
        return functools.partial(object_attribute_set, name=name, args=args,
                                 type_info=type_info, filter=filter)

    if name is None:
        name = set_method.func_name
    attr_name = name
    if attr_name.startswith("set_"):
        attr_name = attr_name[4:]  # removes leading "set_"

    set_method._object_method_ = dict(
        name=name, args=args, types_info=(type_info, "None"), filter=filter)

    if not hasattr(set_method, "_object_attribute_"):
        set_method._object_attribute_ = dict()
    set_method._object_attribute_.update(
        name=attr_name, fset=set_method, args=args, type_info=type_info, filter=filter)

    return set_method


def set_custom_members(src_obj, target_obj, pre_call=None):
    # Creates custom methods and attributes for <target_obj> object
    # using <src_object> object definitions.
    # Populates __custom_methods_list and __custom_attributes_dict
    # for tango device server.
    for name, member in inspect.getmembers(src_obj):
        # Just fills the list.
        if hasattr(member, "_object_attribute_"):
            attribute_info = dict(member._object_attribute_)
            filter = attribute_info.pop('filter', None)
            if filter is None or filter(target_obj):
                add_object_attribute(target_obj,  **member._object_attribute_)

        # For each method of <src_obj>: try to add it as a
        # custom method or as methods to set/get custom
        # attributes.
        try:
            method_info = dict(member._object_method_)
            filter = method_info.pop('filter', None)
            if filter is None or filter(target_obj):
                add_object_method(target_obj, member, pre_call, **method_info)
        except AttributeError:
            pass


def with_custom_members(klass):
    """A class decorator to enable custom attributes and custom methods"""

    def _get_custom_methods(self):
        try:
            return self.__custom_methods_list
        except AttributeError:
            self.__custom_methods_list = []
            return self.__custom_methods_list

    def custom_methods_list(self):
        """ Returns a *copy* of the custom methods """
        return self._get_custom_methods()[:]

    def _add_custom_method(self, method, name, types_info=(None, None)):
        setattr(self, name, method)
        self._get_custom_methods().append((name, types_info))

    def _get_custom_attributes(self):
        try:
            return self.__custom_attributes_dict
        except AttributeError:
            self.__custom_attributes_dict = {}
            return self.__custom_attributes_dict

    def custom_attributes_list(self):
        """
        List of custom attributes defined for this axis.
        Internal usage only
        """
        ad = self._get_custom_attributes()

        # Converts dict into list...
        return [(a_name, ad[a_name][0], ad[a_name][1]) for a_name in ad]

    def _add_custom_attribute(self, name, fget=None, fset=None, type_info=None):
        custom_attrs = self._get_custom_attributes()
        attr_info = custom_attrs.get(name)
        if attr_info:
            orig_type_info, access_mode = attr_info
            if fget and not 'r' in access_mode:
                access_mode = 'rw'
            if fset and not 'w' in access_mode:
                access_mode = 'rw'
            assert type_info == orig_type_info, '%s get/set types mismatch' % name
        else:
            access_mode = 'r' if fget else ''
            access_mode += 'w' if fset else ''
            if fget is None and fset is None:
                raise RuntimeError(
                    "impossible case: must have fget or fset...")
        custom_attrs[name] = type_info, access_mode

    klass._get_custom_methods = _get_custom_methods
    klass.custom_methods_list = property(custom_methods_list)
    klass._add_custom_method = _add_custom_method
    klass._get_custom_attributes = _get_custom_attributes
    klass.custom_attributes_list = property(custom_attributes_list)
    klass._add_custom_attribute = _add_custom_attribute

    return klass


class Null(object):
    __slots__ = []


class StripIt(object):
    """
    Encapsulate object with a short str/repr/format.
    Useful to have in log messages since it only computes the representation
    if the log message is recorded. Example::

        >>> import logging
        >>> logging.basicConfig(level=logging.DEBUG)

        >>> from bliss.common.utils import StripIt

        >>> msg_from_socket = 'Here it is my testament: ' + 50*'bla '
        >>> logging.debug('Received: %s', StripIt(msg_from_socket))
        DEBUG:root:Received: Here it is my testament: bla bla bla bla bla [...]
    """
    __slots__ = 'obj', 'max_len'

    def __init__(self, obj, max_len=50):
        self.obj = obj
        self.max_len = max_len

    def __strip(self, s):
        max_len = self.max_len
        if len(s) > max_len:
            suffix = ' [...]'
            s = s[:max_len - len(suffix)] + suffix
        return s

    def __str__(self):
        return self.__strip(str(self.obj))

    def __repr__(self):
        return self.__strip(repr(self.obj))

    def __format__(self, format_spec):
        return self.__strip(format(self.obj, format_spec))

class periodic_exec(object):
    def __init__(self, period_in_s, func):
        if not callable(func):
            self.func_ref = None
        else:
            self.func_ref = saferef.safe_ref(func)
        self.period = period_in_s
        self.__task = None

    def __enter__(self):
        if self.period > 0 and self.func_ref:
            self.__task = gevent.spawn(self._timer)

    def __exit__(self, *args):
        if self.__task is not None:
            gevent.kill(self.__task)

    def _timer(self):
        while True:
            func = self.func_ref()
            if func is None:
                return
            else:
                func()
                del func
                gevent.sleep(self.period)

