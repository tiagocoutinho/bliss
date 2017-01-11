# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

from bliss.common.event import *
from bliss.config.conductor import client
from bliss.config.settings import Struct, QueueSetting, HashObjSetting, Parameters
from bliss.config.settings import _change_to_obj_marshalling
from bliss.common.continuous_scan import AcquisitionDevice,AcquisitionMaster
from bliss.session.session import get_default as _default_session
from .event import dispatcher
from .event import saferef
from treelib import Tree
import pkgutil
import inspect
import gevent
import re
import datetime
import os
import numpy
import weakref
import string

DM = None

def DataManager():
    global DM
    if DM is None:
        DM = _DataManager()
    return DM


def to_timestamp(dt, epoch=None):
    if epoch is None:
        epoch = datetime.datetime(1970,1,1)
    td = dt - epoch
    return (td.microseconds + (td.seconds + td.days * 86400) * 10**6) / 10**6


class ScanFile:

    FILE_HEADER_TEMPLATE = "#F {0.filename}\n" + \
                           "#E {0.start_time_stamp}\n" + \
                           "#D {0.start_time_str}\n" + \
                           "#C {0.session_name} User = {0.user_name}\n"

    SCAN_HEADER_TEMPLATE = "\n#S {0.scan_nb} {0.title}\n" + \
                           "#D {0.start_time_str}\n" + \
                           "#L {motors}  {counters}\n"

    def __init__(self, env):
        self.env = env
        exists = os.path.exists(self.filename)

        # find next scan number
        if 'scan_nb' not in env:
            scan_nb = 1
            if exists:
                with file(self.filename) as f:
                    for line in iter(f.readline, ''):
                        if line.startswith("#S"):
                            scan_nb += 1
            self.env['scan_nb'] = scan_nb

        self.file_obj = file(self.filename, "a+")

        if not exists:
            self.write_file_header()

    def __getattr__(self, name):
        return self.env[name]

    def write_file_header(self):
        self.write(self.FILE_HEADER_TEMPLATE.format(self))

    def write_header(self):
        motors_str = "  ".join(self.env['actuator_names'])
        cnt_str = "  ".join(self.env['counter_names'])
        self.write(self.SCAN_HEADER_TEMPLATE.format(self, motors=motors_str, counters=cnt_str))

    def write(self, data):
        self.file_obj.write(data)
        self.file_obj.flush()

    def close(self):
        self.file_obj.write("\n\n")
        self.file_obj.close()


class Scan:

    def __init__(self, scan_actuators, npoints, counters_list, env):
        time_scan = scan_actuators == 'time'
        env['start_time'] = start_time = datetime.datetime.now()
        env['start_time_str'] = start_time.strftime("%a %b %d %H:%M:%S %Y")
        env['start_time_stamp'] = to_timestamp(start_time)
        env['actuators'] = scan_actuators
        env['counters'] = counters_list
        actuator_names = ['Time'] if time_scan else [m.name for m in scan_actuators]
        env['actuator_names'] = actuator_names
        counter_names = []
        for counter in counters_list:
            if isinstance(counter.name, (tuple, list)):
                counter_names.extend(counter.name)
            else:
                counter_names.append(counter.name)
        env['counter_names'] = counter_names
        env.setdefault('nb_cols', len(counters_list)+len(scan_actuators))
        self.env = env
        self.raw_data = []
        self.save_flag = self.save and self.filename
        if self.save_flag:
            self.scanfile = ScanFile(env)
            self.scanfile.write_header()
        dispatcher.send(
            "scan_new", self.data_manager, self,
            self.filename if self.save_flag else None,
            'Time' if scan_actuators=='time' else actuator_names,
            npoints, [c.name for c in counters_list])

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError, name
        return self.env[name]

    def add(self, values_list):
        self.raw_data.append(values_list)

        if self.save_flag:
            self.scanfile.write("%s\n" % (" ".join(map(str, values_list))))
        dispatcher.send("scan_data", self.data_manager, self, values_list)

    def end(self):
        end_time = datetime.datetime.now()
        self.env['end_time'] = end_time
        self.env['end_time_str'] = end_time.strftime("%a %b %d %H:%M:%S %Y")
        self.env['end_time_stamp'] = to_timestamp(end_time)

        data = numpy.array(self.raw_data, numpy.float)
        data.shape = (len(self.raw_data), self.nb_cols)
        self.raw_data = []
        self.data_manager._last_scan_data = data

        if self.save_flag:
            self.scanfile.close()

        dispatcher.send("scan_end", self.data_manager, self)


class Timescan(Scan):

    def __init__(self, counters_list, env):
        env['nb_cols'] = len(counters_list)+1
        Scan.__init__(self, 'time', None, counters_list, env)


class _DataManager(object):

    def __init__(self):
        self._last_scan_data = None

    def new_scan(self, motor, npoints, counters_list, env=None, save_flag=True): 
        from bliss.common.scans import ScanEnvironment
        if env is None:
            env = ScanEnvironment()
            env['save'] = False
            env['title'] = 'unnamed'

        if isinstance(motor, list):
            return Scan(motor, npoints, counters_list, env)
        else:
            # assuming old API
            # motor: filename
            # npoints: motor
            # counters_list: npoints
            # env: counters_list
            # save_flag
            scan_env = ScanEnvironment()
            scan_env['save'] = save_flag
            scan_env['filename'] = motor
            scan_env['title'] = 'unnamed'
            return Scan(npoints, counters_list, env, scan_env)

    def new_timescan(self, counters_list, env):
        return Timescan(counters_list, env)

    def last_scan_data(self):
        return self._last_scan_data

# From continuous scan
node_plugins = dict()
for importer, module_name, _ in pkgutil.iter_modules([os.path.join(os.path.dirname(__file__),'..','data')]):
    node_plugins[module_name] = importer

def _get_node_object(node_type, name, parent, connection, create=False):
    importer = node_plugins.get(node_type)
    if importer is None:
        return DataNode(node_type, name, parent, connection = connection, create = create)
    else:
        m = importer.find_module(node_type).load_module(node_type)
        classes = inspect.getmembers(m, lambda x: inspect.isclass(x) and issubclass(x, DataNode) and x!=DataNode)
        # there should be only 1 class inheriting from DataNode in the plugin
        klass = classes[0][-1]
        return klass(name, parent = parent, connection = connection, create = create)

def get_node(name, node_type = None, parent = None, connection = None):
    if connection is None:
        connection = client.get_cache(db=1)
    data = Struct(name, connection=connection)
    if node_type is None:
        node_type = data.node_type
        if node_type is None:       # node has been deleted
            return None

    return _get_node_object(node_type, name, parent, connection)

def _create_node(name, node_type = None, parent = None, connection = None):
    if connection is None:
        connection = client.get_cache(db=1)
    return _get_node_object(node_type, name, parent, connection, create=True)

def _get_or_create_node(name, node_type=None, parent=None, connection = None):
    if connection is None:
        connection = client.get_cache(db=1)
    db_name = DataNode.exists(name, parent, connection)
    if db_name:
        return get_node(db_name, connection=connection)
    else:
        return _create_node(name, node_type, parent, connection)

class DataNodeIterator(object):
    NEW_CHILD_REGEX = re.compile("^__keyspace@.*?:(.*)_children_list$")

    def __init__(self, node):
        self.node = node
        self.last_child_id = dict()
        
    def walk(self, filter=None, wait=True):  
        #print self.node.db_name(),id(self.node)
        try:
            it = iter(filter)
        except TypeError:
            if filter is not None:
                filter = [filter]
        
        if wait:
            redis = self.node.db_connection
            pubsub = redis.pubsub()
            pubsub.psubscribe("__keyspace*__:%s*_children_list" % self.node.db_name())

        db_name = self.node.db_name()
        self.last_child_id[db_name]=0

        if filter is None or self.node.type() in filter:
            yield self.node

        for i, child in enumerate(self.node.children()):
            iterator = DataNodeIterator(child)
            for n in iterator.walk(filter, wait=False):
                self.last_child_id[db_name] = i
                if filter is None or n.type() in filter:
                    yield n

        if wait:
            for msg in pubsub.listen():
                if msg['data'] == 'rpush':
                    channel = msg['channel']
                    parent_db_name = DataNodeIterator.NEW_CHILD_REGEX.match(channel).groups()[0]
                    for child in get_node(parent_db_name).children(self.last_child_id.setdefault(parent_db_name, 0), -1):
                        self.last_child_id[parent_db_name]+=1
                        if filter is None or child.type() in filter:
                            yield child
             
class _TTL_setter(object):
    def __init__(self,db_name):
        self._db_name = db_name
        self._disable = False

    def disable(self):
        self._disable = True

    def __del__(self):
        if not self._disable:
            node = get_node(self._db_name)
            if node is not None:
                node.set_ttl()

class DataNode(object):
    default_time_to_live = 24*3600 # 1 day
    
    @staticmethod
    def exists(name,parent = None, connection = None):
        if connection is None:
            connection = client.get_cache(db=1)
        db_name = '%s:%s' % (parent.db_name(),name) if parent else name
        return db_name if connection.exists(db_name) else None

    def __init__(self,node_type,name,parent = None, connection = None, create=False):
        if connection is None:
            connection = client.get_cache(db=1)
        db_name = '%s:%s' % (parent.db_name(),name) if parent else name
        self._data = Struct(db_name,
                            connection=connection)
        children_queue_name = '%s_children_list' % db_name
        self._children = QueueSetting(children_queue_name,
                                      connection=connection)
        info_hash_name = '%s_info' % db_name
        self._info = HashObjSetting(info_hash_name,
                                    connection=connection)
        self.db_connection = connection
        
        if create:
            self._data.name = name
            self._data.db_name = db_name
            self._data.node_type = node_type
            if parent: 
                self._data.parent = parent.db_name()
                parent.add_children(self)
            self._ttl_setter = _TTL_setter(self.db_name())
        else:
            self._ttl_setter = None

    def db_name(self):
        return self._data.db_name

    def name(self):
        return self._data.name

    def type(self):
        return self._data.node_type

    def iterator(self):
        return DataNodeIterator(self)

    def add_children(self,*child):
        if len(child) > 1:
            children_no = self._children.extend([c.db_name() for c in child])
        else:
            children_no = self._children.append(child[0].db_name())

    def connect(self, signal, callback):
        dispatcher.connect(callback, signal, self)

    def parent(self):
        parent_name = self._data.parent
        if parent_name:
            parent = get_node(parent_name)
            if parent is None:  # clean
                del self._data.parent
            return parent

    #@brief iter over children
    #@return an iterator
    #@param from_id start child index
    #@param to_id last child index
    def children(self,from_id = 0,to_id = -1):
        for child_name in self._children.get(from_id,to_id):
            new_child = get_node(child_name)
            if new_child is not None:
                yield new_child
            else:
                self._children.remove(child_name) # clean

    def last_child(self):
        return get_node(self._children.get(-1))

    def set_info(self,key,values):
        self._info[keys] = values
        if self._ttl > 0:
            self._info.ttl(self._ttl)

    def info_iteritems(self):
        return self._info.iteritems()

    def info_get(self,name):
        return self._info.get(name)

    def data_update(self,keys):
        self._data.update(keys)

    def set_ttl(self):
        db_names = set(self._get_db_names())
        self._set_ttl(db_names)
        if self._ttl_setter is not None:
            self._ttl_setter.disable()
        
    @staticmethod
    def _set_ttl(db_names):
        redis_conn = client.get_cache(db=1)
        pipeline = redis_conn.pipeline()
        for name in db_names:
            pipeline.expire(name,DataNode.default_time_to_live)
        pipeline.execute()

    def _get_db_names(self):
        db_name = self.db_name()
        children_queue_name = '%s_children_list' % db_name
        info_hash_name = '%s_info' % db_name
        db_names = [db_name,children_queue_name,info_hash_name]
        parent = self.parent()
        if parent:
            db_names.extend(parent._get_db_names())
        return db_names

    def store(self, signal, event_dict):
        pass


class Container(object):
    def __init__(self, name, parent=None) :
        self.root_node = parent.node if parent is not None else None
        self.__name = name
        self.node = _get_or_create_node(self.__name, "container", parent=self.root_node)
        
class ScanRecorder(object):
    def __init__(self, name="scan", parent=None, scan_info=None, writer=None,
                 data_watch_callback=None):
        """
        This class publish data and trig the writer if any.
        
        name -- usually the scan name
        parent -- the parent is the root node of the data tree.
        usually the parent is a Container like to a session,sample,experiment...
        i.e: parent = Container('eh3')
        scan_info -- should be the scan parameters as a dict
        writer -- is the final file writter (hdf5,cvs,spec file...)
        data_watch_callback -- a callback which can follow the data status of the scan.
        this callback is usually used to display the scan status.
        the callback will get:
            - data_event : a dict with Acq(Device/Master) as key and a set of signal as values
            - nodes : a dict with Acq(Device/Master) as key and the associated data node as value
        """
        self.root_node = parent.node if parent is not None else None
        self._nodes = dict()
        self._writer = writer

        if parent:
            key = self.root_node.db_name() 
            run_number = client.get_cache(db=1).hincrby(key, "%s_last_run_number" % name, 1)
        else:
            run_number = client.get_cache(db=1).incrby("%s_last_run_number" % name, 1)
	self.__name = '%s_%d' % (name, run_number)
        self._node = _create_node(self.__name, "scan", parent=self.root_node)
        self._data_watch_callback = data_watch_callback
        self._data_events = dict()
        
        if data_watch_callback is not None:
            if not callable(data_watch_callback):
                raise TypeError("data_watch_callback needs to be callable")
            data_watch_callback_event = gevent.event.Event()
            data_watch_callback_done = gevent.event.Event()
            def trig(*args):
                data_watch_callback_event.set()
            self._data_watch_task = gevent.spawn(ScanRecorder._data_watch,
                                                 weakref.proxy(self,trig),
                                                 data_watch_callback_event)
            self._data_watch_callback_event = data_watch_callback_event
        else:
            self._data_watch_task = None

    @property
    def name(self):
        return self.__name
    @property
    def writer(self):
        return self._writer
    @writer.setter
    def writer(self, writer):
        self._writer = writer
    @property
    def node(self):
        return self._node
    @property
    def nodes(self):
        return self._nodes

    def _device_event(self, event_dict=None, signal=None, sender=None):
        if signal == 'end':
            for node in self._nodes.itervalues():
                node.set_ttl()
            self._node.set_ttl()
        node = self._nodes[sender]
        if not hasattr(node,'store'): return
        node.store(signal, event_dict)
        
        if self._data_watch_callback is not None:
            event_set = self._data_events.setdefault(sender,set())
            event_set.add(signal)
            if signal == 'end':
                data_events = self._data_events
                self._data_events = dict()
                self._data_watch_callback(data_events,self.nodes)
            else:
                self._data_watch_callback_event.set()

    def prepare(self, scan_info, devices_tree):
        parent_node = self._node
        prev_level = 1
        self._nodes = dict()

        for dev in list(devices_tree.expand_tree(mode=Tree.WIDTH))[1:]:
            dev_node = devices_tree.get_node(dev)
            level = devices_tree.depth(dev_node)
            if prev_level != level:
                prev_level = level
                parent_node = self._nodes[dev_node.bpointer]

            if isinstance(dev,AcquisitionDevice) or isinstance(dev,AcquisitionMaster):
                self._nodes[dev] = _create_node(dev.name, dev.type, parent_node) 
                for signal in ('start', 'end', 'new_ref','new_data'):
                    dispatcher.connect(self._device_event, signal, dev)

        if self._writer:
            self._writer.prepare(self, scan_info, devices_tree)

    def stop(self):
        for node in self._nodes.itervalues():
            node.set_ttl()
        self._node.set_ttl()
        
    @staticmethod
    def _data_watch(scanrecorder,event):
        while True:
            event.wait()
            event.clear()
            try:
                data_events = scanrecorder._data_events
                scanrecorder._data_events = dict()
                scanrecorder._data_watch_callback(data_events,scanrecorder.nodes)
                gevent.idle()
            except ReferenceError:
                break                
    

class ScanData(Parameters):
    SLOTS = []

    def __init__(self):
        """
        This class hold the saving structure for a session.

        This class generate the *root path* of scans and the *parent* node use to publish data.

        The *root path* is generate using *base path* argument as the first part and
        use the *template* argument as the final part.
        The *template* argument is basically a (python) string format use to generate the final part of the
        root_path.
        i.e: a template like "{session}/{date}" will use the session and the date attribute
        of this class.
        attribute use in this template can also be a function with one argument (scan_data) which return a string.
        i.e: date argument can point to this method
             def get_date(scan_data): datetime.datetime.now().strftime("%Y/%m/%d")
             scan_data.add('date',get_date)

        The *parent* node should be use as parameters for the ScanRecorder.
        """

        keys = dict()
        _change_to_obj_marshalling(keys)
        Parameters.__init__(self,'%s:scan_data' % self.session,
                            default_values = {'base_path': '/tmp',
                                              'template' : '{session}/'},
                            **keys)

    def __dir__(self) :
        keys = Parameters.__dir__(self)
        return keys + ['session','get']

    @property
    def session(self):
        """ This give the name of the default session or unnamed if not session is not defined """

        session = _default_session()
        return session.name if session is not None else 'unnamed'

    def get(self):
        """
        This method will compute all configurations needed for a new acquisition.
        It will return a dictionary with:
            root_path -- compute root path with *base_path* and *template* attribute
            parent -- this DataNode should be used as a parent for new acquisition
        """
        try:
            template = self.template
            formatter = string.Formatter()
            cache_dict = self._proxy.get_all()
            cache_dict['session'] = self.session
            template_keys = [key[1] for key in formatter.parse(template)]

            if 'session' in template_keys:
                parent = None
            else:
                parent = _get_or_create_node(self.session,"container")

            for key in template_keys:
                value = cache_dict.get(key)
                if callable(value):
                    value = value(self) # call the function
                    cache_dict[key] = value
                if value is not None:
                    parent = _get_or_create_node(value,"container",
                                                 parent=parent)
            
            sub_path = template.format(**cache_dict)
        except KeyError,keyname:
            raise RuntimeError("Missing %s attribute in ScanData" % keyname)
        else:
            return {'root_path' : os.path.join(cache_dict.get('base_path'),sub_path),
                    'parent' : parent}
                    
