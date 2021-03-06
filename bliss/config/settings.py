# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

from .conductor import client
from bliss.common.utils import Null
from bliss import setup_globals
import weakref
import pickle
import numpy


class InvalidValue(Null):
    def __str__(self):
        raise ValueError

    def __repr__(self):
        return '#ERR'


def get_cache():
    return client.get_cache(db=0)


def boolify(s, **keys):
    if s == 'True' or s == 'true':
        return True
    if s == 'False' or s == 'false':
        return False
    raise ValueError('Not Boolean Value!')


def auto_conversion(var):
    '''guesses the str representation of the variables type'''
    if var is None:
        return None
    for caster in (boolify, int, float):
        try:
            return caster(var)
        except (ValueError, TypeError):
            pass
    return var


def pickle_loads(var):
    if var is None:
        return None
    try:
        return pickle.loads(var)
    except Exception:
        return InvalidValue()


def ttl_func(cnx, name, value=-1):
    if value is None:
        return cnx.persist(name)
    elif value is -1:
        return cnx.ttl(name)
    else:
        return cnx.expire(name, value)


def read_decorator(func):
    def _read(self, *args, **keys):
        value = func(self, *args, **keys)
        if self._read_type_conversion:
            if isinstance(value, list):
                value = [self._read_type_conversion(x) for x in value]
            elif isinstance(value, dict):
                for k, v in value.iteritems():
                    value[k] = self._read_type_conversion(v)
                if hasattr(self, 'default_values') and isinstance(self.default_values, dict):
                    tmp = dict(self._default_values)
                    tmp.update(value)
                    value = tmp
            else:
                if value is not None:
                    value = self._read_type_conversion(value)
        if value is None:
            if hasattr(self, '_default_value'):
                value = self._default_value
            elif(hasattr(self, '_default_values') and
                 hasattr(self._default_values, 'get')):
                value = self._default_values.get(args[0])
        return value
    return _read


def write_decorator_dict(func):
    def _write(self, values, **keys):
        if self._write_type_conversion:
            if not isinstance(values, dict) and values is not None:
                raise TypeError('can only be dict')

            if values is not None:
                for k, v in values.iteritems():
                    values[k] = self._write_type_conversion(v)
        return func(self, values, **keys)
    return _write


def write_decorator_multiple(func):
    def _write(self, values, **keys):
        if self._write_type_conversion:
            if not isinstance(values, (list, tuple, numpy.ndarray)) and values is not None:
                raise TypeError('Can only be tuple, list or numpy array')
            if values is not None:
                values = [self._write_type_conversion(x) for x in values]
        return func(self, values, **keys)
    return _write


def write_decorator(func):
    def _write(self, value, **keys):
        if self._write_type_conversion and value is not None:
            value = self._write_type_conversion(value)
        return func(self, value, **keys)
    return _write


def scan(match='*', count=1000, connection=None):
    if connection is None:
        connection = get_cache()
    cursor = 0
    while 1:
        cursor, values = connection.scan(cursor=cursor,
                                         match=match, count=count)
        for val in values:
            yield val
        if int(cursor) == 0:
            break


class SimpleSetting(object):
    def __init__(self, name, connection=None,
                 read_type_conversion=auto_conversion,
                 write_type_conversion=None,
                 default_value=None):
        if connection is None:
            connection = get_cache()
        self._cnx = weakref.ref(connection)
        self._name = name
        self._read_type_conversion = read_type_conversion
        self._write_type_conversion = write_type_conversion
        self._default_value = default_value

    @read_decorator
    def get(self):
        cnx = self._cnx()
        value = cnx.get(self._name)
        return value

    @write_decorator
    def set(self, value):
        cnx = self._cnx()
        cnx.set(self._name, value)

    def ttl(self, value=-1):
        return ttl_func(self._cnx(), self._name, value)

    def clear(self):
        cnx = self._cnx()
        cnx.delete(self._name)

    def __add__(self, other):
        value = self.get()
        if isinstance(other, SimpleSetting):
            other = other.get()
        return value + other

    def __iadd__(self, other):
        cnx = self._cnx()
        if cnx is not None:
            if isinstance(other, int):
                if other == 1:
                    cnx.incr(self._name)
                else:
                    cnx.incrby(self._name, other)
            elif isinstance(other, float):
                cnx.incrbyfloat(self._name, other)
            else:
                cnx.append(self._name, other)
            return self

    def __isub__(self, other):
        if isinstance(other, basestring):
            raise TypeError("unsupported operand type(s) for -=: %s" %
                            type(other).__name__)
        return self.__iadd__(-other)

    def __getitem__(self, ran):
        cnx = self._cnx()
        if cnx is not None:
            step = None
            if isinstance(ran, slice):
                i, j = ran.start, ran.stop
                step = ran.step
            elif isinstance(ran, int):
                i = j = ran
            else:
                raise TypeError('indices must be integers')

            value = cnx.getrange(self._name, i, j)
            if step is not None:
                value = value[0:-1:step]
            return value

    def __repr__(self):
        cnx = self._cnx()
        value = cnx.get(self._name)
        return '<SimpleSetting name=%s value=%s>' % (self._name, value)


class SimpleSettingProp(object):
    def __init__(self, name, connection=None,
                 read_type_conversion=auto_conversion,
                 write_type_conversion=None,
                 default_value=None,
                 use_object_name=True):
        self._name = name
        self._cnx = connection or get_cache()
        self._read_type_conversion = read_type_conversion
        self._write_type_conversion = write_type_conversion
        self._default_value = default_value
        self._use_object_name = use_object_name

    def __get__(self, obj, type=None):
        if self._use_object_name:
            name = obj.name + ':' + self._name
        else:
            name = self._name
        return SimpleSetting(name, self._cnx,
                             self._read_type_conversion,
                             self._write_type_conversion,
                             self._default_value)

    def __set__(self, obj, value):
        if isinstance(value, SimpleSetting):
            return

        if self._use_object_name:
            name = obj.name + ':' + self._name
        else:
            name = self._name

        if value is None:
            self._cnx.delete(name)
        else:
            if self._write_type_conversion:
                value = self._write_type_conversion(value)
            self._cnx.set(name, value)


class QueueSetting(object):
    def __init__(self, name, connection=None,
                 read_type_conversion=auto_conversion,
                 write_type_conversion=None):
        if connection is None:
            connection = get_cache()
        self._cnx = weakref.ref(connection)
        self._name = name
        self._read_type_conversion = read_type_conversion
        self._write_type_conversion = write_type_conversion

    @read_decorator
    def get(self, first=0, last=-1, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        if first == last:
            l = cnx.lindex(self._name, first)
        else:
            if last != -1:
                last -= 1
            l = cnx.lrange(self._name, first, last)
        return l

    @write_decorator
    def append(self, value, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        return cnx.rpush(self._name, value)

    def clear(self, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        cnx.delete(self._name)

    @write_decorator
    def prepend(self, value, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        return cnx.lpush(self._name, value)

    @write_decorator_multiple
    def extend(self, values, cnx=None):
        cnx = self._cnx()
        return cnx.rpush(self._name, *values)

    @write_decorator
    def remove(self, value, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        cnx.lrem(self._name, value)

    @write_decorator_multiple
    def set(self, values, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        cnx.delete(self._name)
        if values is not None:
            cnx.rpush(self._name, *values)

    @write_decorator
    def set_item(self, value, pos=0, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        cnx.lset(self._name, pos, value)

    @read_decorator
    def pop_front(self, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        value = cnx.lpop(self._name)
        if self._read_type_conversion:
            value = self._read_type_conversion(value)
        return value

    @read_decorator
    def pop_back(self, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        value = cnx.rpop(self._name)
        if self._read_type_conversion:
            value = self._read_type_conversion(value)
        return value

    def ttl(self, value=-1, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        return ttl_func(cnx, self._name, value)

    def __len__(self, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        return cnx.llen(self._name)

    def __repr__(self, cnx=None):
        if cnx is None:
            cnx = self._cnx()
        value = cnx.lrange(self._name, 0, -1)
        return '<QueueSetting name=%s value=%s>' % (self._name, value)

    def __iadd__(self, other, cnx=None):
        self.extend(other, cnx)
        return self

    def __getitem__(self, ran, cnx=None):
        if isinstance(ran, slice):
            i = ran.start is not None and ran.start or 0
            j = ran.stop is not None and ran.stop or -1
        elif isinstance(ran, int):
            i = j = ran
        else:
            raise TypeError('indices must be integers')
        value = self.get(first=i, last=j, cnx=cnx)
        if value is None:
            raise IndexError
        else:
            return value

    def __iter__(self, cnx = None):
        if cnx is None:
            cnx = self._cnx()
        lsize = cnx.llen(self._name)
        for first in xrange(0, lsize, 1024):
            last = first + 1024
            if last >= lsize:
                last = -1
            for value in self.get(first, last):
                yield value

    def __setitem__(self, ran, value, cnx=None):
        if isinstance(ran, slice):
            for i, v in zip(range(ran.start, ran.stop), value):
                self.set_item(v, pos=i, cnx=cnx)
        elif isinstance(ran, int):
            self.set_item(value, pos=ran, cnx=cnx)
        else:
            raise TypeError('indices must be integers')
        return self


class QueueSettingProp(object):
    def __init__(self, name, connection=None,
                 read_type_conversion=auto_conversion,
                 write_type_conversion=None,
                 use_object_name=True):
        self._name = name
        self._cnx = connection or get_cache()
        self._read_type_conversion = read_type_conversion
        self._write_type_conversion = write_type_conversion
        self._use_object_name = use_object_name

    def __get__(self, obj, type=None):
        if self._use_object_name:
            name = obj.name + ':' + self._name
        else:
            name = self._name

        return QueueSetting(name, self._cnx,
                            self._read_type_conversion,
                            self._write_type_conversion)

    def __set__(self, obj, values):
        if isinstance(values, QueueSetting):
            return

        if self._use_object_name:
            name = obj.name + ':' + self._name
        else:
            name = self._name

        proxy = QueueSetting(name, self._cnx,
                             self._read_type_conversion,
                             self._write_type_conversion)
        proxy.set(values)


class HashSetting(object):
    def __init__(self, name, connection=None,
                 read_type_conversion=auto_conversion,
                 write_type_conversion=None,
                 default_values={}):
        if connection is None:
            connection = get_cache()
        self._cnx = weakref.ref(connection)
        self._name = name
        self._read_type_conversion = read_type_conversion
        self._write_type_conversion = write_type_conversion
        self._default_values = default_values

    def __repr__(self):
        value = self.get_all()
        return '<HashSetting name=%s value=%s>' % (self._name, value)

    def __delitem__(self, key):
        cnx = self._cnx()
        cnx.hdel(self._name, key)

    def __len__(self):
        cnx = self._cnx()
        return cnx.hlen(self._name)

    def ttl(self, value=-1):
        return ttl_func(self._cnx(), self._name, value)

    def raw_get(self, *keys):
        cnx = self._cnx()
        return cnx.hget(self._name, *keys)

    @read_decorator
    def get(self, key, default=None):
        v = self.raw_get(key)
        if v is None:
            if self._write_type_conversion:
                v = self._write_type_conversion(default)
            else:
                v = default
        return v

    def _raw_get_all(self):
        cnx = self._cnx()
        return cnx.hgetall(self._name)

    def get_all(self):
        all_dict = dict(self._default_values)
        for k, raw_v in self._raw_get_all().iteritems():
            v = self._read_type_conversion(raw_v)
            if isinstance(v, InvalidValue):
                raise ValueError(
                    "%s: Invalid value '%s` (cannot deserialize %r)" % (self._name, k, raw_v))
            all_dict[k] = v
        return all_dict

    @read_decorator
    def pop(self, key, default=Null()):
        cnx = self._cnx().pipeline()
        cnx.hget(self._name, key)
        cnx.hdel(self._name, key)
        (value, worked) = cnx.execute()
        if not worked:
            if isinstance(default, Null):
                raise KeyError(key)
            else:
                value = default
        return value

    def remove(self, *keys):
        cnx = self._cnx()
        cnx.hdel(self._name, *keys)

    def keys(self):
        return list(self.iterkeys())

    def values(self):
        return list(self.itervalues())

    def clear(self):
        cnx = self._cnx()
        cnx.delete(self._name)

    def copy(self):
        return self.get()

    @write_decorator_dict
    def set(self, values):
        cnx = self._cnx()
        cnx.delete(self._name)
        if values is not None:
            cnx.hmset(self._name, values)

    @write_decorator_dict
    def update(self, values):
        cnx = self._cnx()
        if values:
            cnx.hmset(self._name, values)

    def items(self):
        values = self.get_all()
        return [(k, v) for k, v in values.iteritems()]

    @read_decorator
    def fromkeys(self, *keys):
        cnx = self._cnx()
        return cnx.hmget(self._name, *keys)

    def has_key(self, key):
        cnx = self._cnx()
        return cnx.hexists(self._name, key) or self._default_values.has_key(key)

    def iterkeys(self):
        for k, v in self.iteritems():
            yield k

    def itervalues(self):
        for k, v in self.iteritems():
            yield v

    def iteritems(self):
        cnx = self._cnx()
        next_id = 0
        seen_keys = set()
        while True:
            next_id, pd = cnx.hscan(self._name, next_id)
            for k, v in pd.iteritems():
                if self._read_type_conversion:
                    v = self._read_type_conversion(v)
                seen_keys.add(k)
                yield k, v
            if not next_id or next_id is '0':
                break

        for k, v in self._default_values.iteritems():
            if k in seen_keys:
                continue
            yield k, v

    def __getitem__(self, key):
        value = self.get(key)
        if value is None:
            if not self._default_values.has_key(key):
                raise KeyError(key)
        return value

    def __setitem__(self, key, value):
        cnx = self._cnx()
        if value is None:
            cnx.hdel(self._name, key)
            return
        if self._write_type_conversion:
            value = self._write_type_conversion(value)
        cnx.hset(self._name, key, value)

    def __contains__(self, key):
        try:
            self[key]
            return True
        except KeyError:
            return False


class HashSettingProp(object):
    def __init__(self, name, connection=None,
                 read_type_conversion=auto_conversion,
                 write_type_conversion=None,
                 default_values={},
                 use_object_name=True):
        self._name = name
        self._cnx = connection or get_cache()
        self._read_type_conversion = read_type_conversion
        self._write_type_conversion = write_type_conversion
        self._default_values = default_values
        self._use_object_name = use_object_name

    def __get__(self, obj, type=None):
        if self._use_object_name:
            name = obj.name + ':' + self._name
        else:
            name = self._name

        return HashSetting(name, self._cnx,
                           self._read_type_conversion,
                           self._write_type_conversion,
                           self._default_values)

    def __set__(self, obj, values):
        if self._use_object_name:
            name = obj.name + ':' + self._name
        else:
            name = self._name

        if isinstance(values, HashSetting):
            return

        proxy = HashSetting(name, self._cnx,
                            self._read_type_conversion,
                            self._write_type_conversion,
                            self._default_values)
        proxy.set(values)

    def get_proxy(self):
        return HashSetting(self._name, self._cnx,
                           self._read_type_conversion,
                           self._write_type_conversion,
                           self._default_values)
# helper


def _change_to_obj_marshalling(keys):
    read_type_conversion = keys.pop('read_type_conversion', pickle_loads)
    write_type_conversion = keys.pop('write_type_conversion', pickle.dumps)
    keys.update({'read_type_conversion': read_type_conversion,
                 'write_type_conversion': write_type_conversion})


class HashObjSetting(HashSetting):
    def __init__(self, name, **keys):
        _change_to_obj_marshalling(keys)
        HashSetting.__init__(self, name, **keys)


class HashObjSettingProp(HashSettingProp):
    def __init__(self, name, **keys):
        _change_to_obj_marshalling(keys)
        HashSettingProp.__init__(self, name, **keys)


class QueueObjSetting(QueueSetting):
    def __init__(self, name, **keys):
        _change_to_obj_marshalling(keys)
        QueueSetting.__init__(self, name, **keys)


class QueueObjSettingProp(QueueSettingProp):
    def __init__(self, name, **keys):
        _change_to_obj_marshalling(keys)
        QueueSettingProp.__init__(self, name, **keys)


class SimpleObjSetting(SimpleSetting):
    def __init__(self, name, **keys):
        _change_to_obj_marshalling(keys)
        SimpleSetting.__init__(self, name, **keys)


class SimpleObjSettingProp(SimpleSettingProp):
    def __init__(self, name, **keys):
        _change_to_obj_marshalling(keys)
        SimpleSettingProp.__init__(self, name, **keys)


class Struct(object):
    def __init__(self, name, **keys):
        self._proxy = HashSetting(name, **keys)

    def __dir__(self):
        return self._proxy.keys()

    def __repr__(self):
        return "<Struct with attributes: %s>" % self._proxy.keys()

    def __getattribute__(self, name):
        if name.startswith('_'):
            return object.__getattribute__(self, name)
        else:
            return self._proxy.get(name)

    def __setattr__(self, name, value):
        if name.startswith('_'):
            return object.__setattr__(self, name, value)
        else:
            self._proxy[name] = value

    def __delattr__(self, name):
        if name.startswith('_'):
            return object.__delattr__(self, name)
        else:
            self._proxy.remove(name)


class ParametersType(type):
    def __call__(cls, *args, **kwargs):
        class_dict = {'__slots__': tuple(cls.SLOTS), 'SLOTS': cls.SLOTS}
        new_cls = type(cls.__name__, (cls,), class_dict)
        return type.__call__(new_cls, *args, **kwargs)

    def __new__(cls, name, bases, attrs):
        attrs['__slots__'] = tuple(attrs['SLOTS'])
        return type.__new__(cls, name, bases, attrs)


class ParamDescriptor(object):
    OBJECT_PREFIX = 'object:'

    def __init__(self, proxy, name, value, assign=True):
        self.proxy = proxy
        self.name = name
        if assign:
            self.assign(value)

    def assign(self, value):
        if hasattr(value, 'name') and hasattr(setup_globals, value.name):
            value = '%s%s' % (ParamDescriptor.OBJECT_PREFIX, value.name)
        try:
            self.proxy[self.name] = value
        except Exception:
            raise ValueError("%s.%s: cannot set value" %
                             (self.proxy._name, self.name))

    def __get__(self, obj, obj_type):
        value = self.proxy[self.name]
        if isinstance(value, (str, unicode)) and value.startswith(ParamDescriptor.OBJECT_PREFIX):
            value = value[len(ParamDescriptor.OBJECT_PREFIX):]
            return getattr(setup_globals, value)
        return value

    def __set__(self, obj, value):
        return self.assign(value)

    def __delete__(self, *args):
        del self.proxy[self.name]


class Parameters(object):
    __metaclass__ = ParametersType
    DESCRIPTOR = ParamDescriptor
    SLOTS = ['_proxy', '__current_config']

    def __init__(self, name, **keys):
        self.__current_config = SimpleSetting(name, default_value='default')
        hash_name = '%s:%s' % (name, self.__current_config.get())
        self._proxy = HashSetting(hash_name, **keys)
        for key in self._proxy.iterkeys():
            self.add(key)

    def __dir__(self):
        return self._proxy.keys() + ['add', 'remove', 'switch', 'configs']

    def __repr__(self):
        rep_str = "Parameters (%s)\n" % self.__current_config.get()
        d = dict(self._proxy.iteritems())
        max_len = max((len(x) for x in d.keys()))
        str_format = '  .%-' + '%ds' % max_len + ' = %r\n'
        for key, value in sorted(d.iteritems()):
            rep_str += str_format % (key, value)
        return rep_str

    def add(self, name, value=None):
        setattr(self.__class__, name, self.DESCRIPTOR(self._proxy, name, value,
                                                      value is not None))

    def remove(self, name):
        self._proxy.remove(name)
        delattr(self.__class__, name)

    def switch(self, name):
        for key, value in dict(self.__class__.__dict__).iteritems():
            if isinstance(value, self.DESCRIPTOR):
                delattr(self.__class__, key)

        self.__current_config.set(name)

        basename = ":".join(self._proxy._name.split(':')[:-1])
        self._proxy._name = '%s:%s' % (basename, name)

        for key in self._proxy.keys():
            self.add(key)

    @property
    def configs(self):
        basename = ":".join(self._proxy._name.split(':')[:-1])
        return list((x.split(':')[-1] for x in scan(match='%s:*' % basename)))


if __name__ == "__main__":
    class A(object):
        x = SimpleSettingProp('counter')
        y = SimpleObjSettingProp('obj')
        q = QueueSettingProp('seb')
        ol = QueueObjSettingProp('seb-list')
        h = HashSettingProp('seb-hash')
        oh = HashObjSettingProp('seb-hash-object')

        def __init__(self, name):
            self.name = name

    a = A('m0')
    p = Struct('optics:zap:params')
