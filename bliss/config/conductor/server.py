# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.


# Imports

import os
import sys
import codecs
import shutil
import logging
import argparse
import weakref
import subprocess
import socket
import signal
import traceback
import pkgutil
import gevent
from gevent import select

from bliss.common import event
from . import protocol
from .. import redis as redis_conf


# Conditional imports

try:
    import posix_ipc
except ImportError:
    posix_ipc = None
else:
    class _PosixQueue(posix_ipc.MessageQueue):
        def __init__(self):
            posix_ipc.MessageQueue.__init__(
                self, None, mode=0o666, flags=posix_ipc.O_CREX)
            self._wqueue = posix_ipc.MessageQueue(
                None, mode=0o666, flags=posix_ipc.O_CREX)

        def unlink(self):
            posix_ipc.MessageQueue.unlink(self)
            self._wqueue.unlink()

        def close(self):
            posix_ipc.MessageQueue.close(self)
            self._wqueue.close()

        def names(self):
            return self._wqueue.name, self.name

        def sendall(self, msg):
            max_message_size = self.max_message_size
            for i in xrange(0, len(msg), max_message_size):
                self._wqueue.send(msg[i:i+max_message_size])

# Globals

_waitstolen = dict()
_options = None
_lock_object = {}
_client_to_object = weakref.WeakKeyDictionary()
_waiting_lock = weakref.WeakKeyDictionary()
_log = logging.getLogger('beacon')
_tlog = _log.getChild('tango')
_rlog = _log.getChild('redis')
_wlog = _log.getChild('web')


# Helpers

class _WaitStolenReply(object):
    def __init__(self,stolen_lock):
        self._stolen_lock = dict()
        for client,objects in stolen_lock.iteritems():
            self._stolen_lock[client] = '|'.join(objects)
        self._client2info = dict()

    def __enter__(self):
        for client,message in self._stolen_lock.iteritems():
            event = gevent.event.Event()
            client2sync = _waitstolen.setdefault(message,dict())
            client2sync[client] = event
            client.sendall(protocol.message(protocol.LOCK_STOLEN,message))
        return self

    def __exit__(self,*args,**keys):
        for client,message in self._stolen_lock.iteritems():
            client2sync = _waitstolen.pop(message,None)
            if client2sync is not None:
                client2sync.pop(client,None)
            if client2sync:
                _waitstolen[message] = client2sync

    def wait(self,timeout):
        with gevent.Timeout(timeout,
                            RuntimeError("some client(s) didn't reply to stolen lock")):
            for client,message in self._stolen_lock.iteritems():
                client2sync = _waitstolen.get(message)
                if client2sync is not None:
                    sync = client2sync.get(client)
                    sync.wait()

# Methods

def _releaseAllLock(client_id):
#    print '_releaseAllLock',client_id
    objset = _client_to_object.pop(client_id,set())
    for obj in objset:
#        print 'release',obj
        _lock_object.pop(obj)
    #Inform waiting client
    tmp_dict = dict(_waiting_lock)
    for client_sock,tlo in tmp_dict.iteritems():
        try_lock_object = set(tlo)
        if try_lock_object.intersection(objset):
            objs = _waiting_lock.pop(client_sock)
            client_sock.sendall(protocol.message(protocol.LOCK_RETRY))

def _lock(client_id,prio,lock_obj,raw_message) :
#    print '_lock_object',_lock_object
#    print
    all_free = True
    for obj in lock_obj:
        socket_id,compteur,lock_prio = _lock_object.get(obj,(None,None,None))
        if socket_id and socket_id != client_id:
            if prio > lock_prio : continue
            all_free = False
            break

    if all_free:
        stolen_lock = {}
        for obj in lock_obj:
            socket_id,compteur,lock_prio = _lock_object.get(obj,(client_id,0,prio))
            if socket_id != client_id: # still lock
                pre_obj = stolen_lock.get(socket_id,None)
                if pre_obj is None:
                    stolen_lock[socket_id] = [obj]
                else:
                    pre_obj.append(obj)
                _lock_object[obj] = (client_id,1,prio)
                objset = _client_to_object.get(socket_id,set())
                objset.remove(obj)
            else:
                compteur += 1
                new_prio = lock_prio > prio and lock_prio or prio
                _lock_object[obj] = (client_id,compteur,new_prio)

        try:
            with _WaitStolenReply(stolen_lock) as w:
                w.wait(3.)
        except RuntimeError:
            _log.warning("some client(s) didn't reply to the stolen lock")

        obj_already_locked = _client_to_object.get(client_id,set())
        _client_to_object[client_id] = set(lock_obj).union(obj_already_locked)

        client_id.sendall(protocol.message(protocol.LOCK_OK_REPLY,raw_message))
    else:
        _waiting_lock[client_id] = lock_obj

#    print '_lock_object',_lock_object

def _unlock(client_id,priority,unlock_obj) :
    unlock_object = []
    client_locked_obj = _client_to_object.get(client_id,None)
    if client_locked_obj is None:
        return

    for obj in unlock_obj:
        socket_id,compteur,prio = _lock_object.get(obj,(None,None,None))
#        print socket_id,compteur,prio,obj
        if socket_id and socket_id == client_id:
            compteur -= 1
            if compteur <= 0:
                _lock_object.pop(obj)
                try:
                    client_locked_obj.remove(obj)
                    _lock_object.pop(obj)
                except KeyError:
                    pass
                unlock_object.append(obj)
            else:
                _lock_object[obj] = (client_id,compteur,prio)

    unlock_object = set(unlock_object)
    tmp_dict = dict(_waiting_lock)
    for client_sock,tlo in tmp_dict.iteritems():
        try_lock_object = set(tlo)
        if try_lock_object.intersection(unlock_object) :
            objs = _waiting_lock.pop(client_sock)
            client_sock.sendall(protocol.message(protocol.LOCK_RETRY))

#    print '_lock_object',_lock_object

def _clean(client):
    _releaseAllLock(client)

def _send_redis_info(client_id,local_connection):
    port = _options.redis_port
    host = socket.gethostname()
    if local_connection:
        port = _options.redis_socket
        host = 'localhost'

    client_id.sendall(protocol.message(protocol.REDIS_QUERY_ANSWER,
                                       '%s:%s' % (host,port)))

def _send_config_file(client_id,message):
    try:
        message_key,file_path = message.split('|')
    except ValueError:          # message is bad, skip it
        return
    file_path = file_path.replace('../','') # prevent going up
    full_path = os.path.join(_options.db_path,file_path)
    try:
        with codecs.open(full_path, "r", "utf-8") as f:
            buffer = f.read().encode('utf-8')
            client_id.sendall(protocol.message(protocol.CONFIG_GET_FILE_OK,'%s|%s' % (message_key,buffer)))
    except IOError:
        client_id.sendall(protocol.message(protocol.CONFIG_GET_FILE_FAILED,"%s|File doesn't exist" % (message_key)))

def __find_module(client_id,message_key,path,parent_name = None):
    for importer,name,ispkg in pkgutil.walk_packages([path]):
        module_name = name if parent_name is None else '%s.%s' % (parent_name,name)
        client_id.sendall(protocol.message(protocol.CONFIG_GET_PYTHON_MODULE_RX,
                                           '%s|%s|%s' % (message_key,module_name,
                                                         importer.find_module(name).get_filename())))
        if ispkg:
            __find_module(client_id,message_key,os.path.join(path,name),module_name)

def _get_python_module(client_id,message):
    try:
        message_key,start_module_path = message.split('|')
    except ValueError:
        client_id.sendall(protocol.message(protocol.CONFIG_GET_PYTHON_MODULE_FAILED,"%s|Can't split message (%s)" %
                                           (message_key,message)))
        return

    start_module_path = start_module_path.replace('../','') # prevent going up
    start_module_path = os.path.join(_options.db_path,start_module_path)

    __find_module(client_id,message_key,start_module_path)
    client_id.sendall(protocol.message(protocol.CONFIG_GET_PYTHON_MODULE_END, '%s|' % message_key))



def __remove_empty_tree(base_dir=None, keep_empty_base=True):
    """
    Helper to remove empty directory tree.

    If *base_dir* is *None* (meaning start at the beacon server base directory),
    the *keep_empty_base* is forced to True to prevent the system from removing
    the beacon base path

    :param base_dir: directory to start from [default is None meaning start at
                     the beacon server base directory
    :type base_dir: str
    :param keep_empty_base: if True (default) doesn't remove the given
                            base directory. Otherwise the base directory is
                            removed if empty.
    """
    if base_dir is None:
        base_dir = _options.db_path
        keep_empty_base = False

    for dir_path, dir_names, file_names in os.walk(base_dir, topdown=False):
        if keep_empty_base and dir_path == base_dir:
            continue
        if file_names:
            continue
        for dir_name in dir_names:
            full_dir_name = os.path.join(dir_path, dir_name)
            if not os.listdir(full_dir_name): # check if directory is empty
                os.removedirs(full_dir_name)

def _remove_config_file(client_id, message):
    try:
        message_key,file_path = message.split('|')
    except ValueError:          # message is bad, skip it
        return
    file_path = file_path.replace('../','') # prevent going up
    full_path = os.path.join(_options.db_path, file_path)
    try:
        if os.path.isfile(full_path):
            os.remove(full_path)
        elif os.path.isdir(full_path):
            shutil.rmtree(full_path)

        # walk back in directory tree removing empty directories. Do this to
        # prevent future rename operations to inadvertely ending up inside a
        # "transparent" directory instead of being renamed
        __remove_empty_tree()
        msg = (protocol.CONFIG_REMOVE_FILE_OK, '%s|0' % (message_key,))
    except IOError:
        msg = (protocol.CONFIG_REMOVE_FILE_FAILED,
               "%s|File/directory doesn't exist" % message_key)
    else:
        event.send(__name__, 'config_changed')

    client_id.sendall(protocol.message(*msg))


def _move_config_path(client_id, message):
    # should work on both files and folders
    # it can be used for both move and rename
    try:
        message_key, src_path, dst_path = message.split('|')
    except ValueError:          # message is bad, skip it
        return
    src_path = src_path.replace('../','') # prevent going up
    src_full_path = os.path.join(_options.db_path, src_path)

    dst_path = dst_path.replace('../','') # prevent going up
    dst_full_path = os.path.join(_options.db_path, dst_path)

    try:
        # make sure the parent directory exists
        parent_dir = os.path.dirname(dst_full_path)
        if not os.path.isdir(parent_dir):
            os.makedirs(parent_dir)
        shutil.move(src_full_path, dst_full_path)

        # walk back in directory tree removing empty directories. Do this to
        # prevent future rename operations to inadvertely ending up inside a
        # "transparent" directory instead of being renamed
        __remove_empty_tree()
        msg = (protocol.CONFIG_MOVE_PATH_OK, '%s|0' % (message_key,))
    except IOError as ioe:
        msg = (protocol.CONFIG_MOVE_PATH_FAILED,
               "%s|%s: %s" % (message_key, ioe.filename, ioe.strerror))
    else:
        event.send(__name__, 'config_changed')
    client_id.sendall(protocol.message(*msg))


def _send_config_db_files(client_id,message):
    try:
        message_key,sub_path = message.split('|')
    except ValueError:          # message is bad, skip it
        return
    sub_path = sub_path.replace('../','') # prevent going up
    look_path = sub_path and os.path.join(_options.db_path,sub_path) or _options.db_path
    try:
        for root,dirs,files in os.walk(look_path):
            for filename in files:
                basename, ext = os.path.splitext(filename)
                if ext == '.yml':
                    full_path = os.path.join(root,filename)
                    rel_path = full_path[len(_options.db_path) + 1:]
                    try:
                        with codecs.open(full_path, "r", "utf-8") as f:
                            raw_buffer = f.read().encode('utf-8')
                            msg = protocol.message(protocol.CONFIG_DB_FILE_RX,'%s|%s|%s' % (message_key,rel_path,raw_buffer))
                            client_id.sendall(msg)
                    except Exception as e:
                        sys.excepthook(*sys.exc_info())
                        client_id.sendall(protocol.message(protocol.CONFIG_DB_FAILED, "%s|%s" % (message_key, e)))
    except Exception as e:
        sys.excepthook(*sys.exc_info())
        client_id.sendall(protocol.message(protocol.CONFIG_DB_FAILED, "%s|%s" % (message_key, e)))
    finally:
        client_id.sendall(protocol.message(protocol.CONFIG_DB_END,"%s|" % (message_key)))

def __get_directory_structure(base_dir):
    """
    Helper that creates a nested dictionary that represents the folder structure of base_dir
    """
    result = {}
    base_dir = base_dir.rstrip(os.sep)
    start = base_dir.rfind(os.sep) + 1
    for path, dirs, files in os.walk(base_dir):
        folders = path[start:].split(os.sep)
        subdir = dict.fromkeys(files)
        parent = reduce(dict.get, folders[:-1], result)
        parent[folders[-1]] = subdir
    assert len(result) == 1
    return result.popitem()

def _send_config_db_tree(client_id,message):
    try:
        message_key,sub_path = message.split('|')
    except ValueError:          # message is bad, skip it
        return
    sub_path = sub_path.replace('../','') # prevent going up
    look_path = sub_path and os.path.join(_options.db_path,sub_path) or _options.db_path

    import json
    try:
        _, tree = __get_directory_structure(look_path)
        msg = (protocol.CONFIG_GET_DB_TREE_OK,'%s|%s' % (message_key, json.dumps(tree)))
    except Exception as e:
        sys.excepthook(*sys.exc_info())
        msg = (protocol.CONFIG_GET_DB_TREE_FAILED,
               "%s|Failed to get tree: %s" % (message_key, str(e)))
    client_id.sendall(protocol.message(*msg))

def _write_config_db_file(client_id,message):
    first_pos = message.find('|')
    second_pos = message.find('|',first_pos + 1)

    if first_pos < 0 or second_pos < 0: # message malformed
        msg = protocol.message(protocol.CONFIG_SET_DB_FILE_FAILED,
                               '%s|%s' % (message_key,'Malformed message'))
        client_id.sendall(msg)
        return

    message_key = message[:first_pos]
    file_path = message[first_pos + 1:second_pos]
    content = message[second_pos + 1:].decode("utf-8")
    file_path = file_path.replace('../','') # prevent going up
    full_path = os.path.join(_options.db_path,file_path)
    full_dir = os.path.dirname(full_path)
    if not os.path.isdir(full_dir):
        os.makedirs(full_dir)
    try:
        with file(full_path,'w') as f:
            f.write(content)
            msg = protocol.message(protocol.CONFIG_SET_DB_FILE_OK,'%s|0' % message_key)
    except:
        msg = protocol.message(protocol.CONFIG_SET_DB_FILE_FAILED,
                               '%s|%s' % (message_key,traceback.format_exc()))
    else:
        event.send(__name__, 'config_changed')
    client_id.sendall(msg)

def _send_posix_mq_connection(client_id,client_hostname):
    ok_flag = False
    try:
        if(posix_ipc is not None and
           not isinstance(client_id,posix_ipc.MessageQueue)):
            if client_hostname == socket.gethostname(): # same host
                #open a message queue
                new_mq = _PosixQueue()
                client_id._pmq = new_mq
                mq_name = new_mq.names()
                ok_flag = True
    except:
        sys.excepthook(*sys.exc_info())
    finally:
        if ok_flag:
            client_id.sendall(protocol.message(protocol.POSIX_MQ_OK,'|'.join(mq_name)))
            return new_mq
        else:
            client_id.sendall(protocol.message(protocol.POSIX_MQ_FAILED))

def _send_unknow_message(client_id,message):
    client_id.sendall(protocol.message(protocol.UNKNOW_MESSAGE,message))

def _client_rx(client,local_connection):
    tcp_data = ''
    posix_queue_data = ''
    posix_queue = None
    r_listen = [client]
    try:
        stopFlag = False
        while not stopFlag:
            r,_,_ = select.select(r_listen,[],[])
            for fd in r:
                if fd == client: # tcp
                    try:
                        raw_data = client.recv(16 * 1024)
                    except:
                        raw_data = None

                    if raw_data:
                        tcp_data = '%s%s' % (tcp_data,raw_data)
                    else:
                        stopFlag = True
                        break

                    data = tcp_data
                    c_id = client
                else:
                    posix_queue_data = '%s%s' % (posix_queue_data,posix_queue.receive()[0])
                    data = posix_queue_data
                    c_id = posix_queue

                while data:
                    try:
                        messageType,message,data = protocol.unpack_message(data)
                        if messageType == protocol.LOCK:
                            lock_objects = message.split('|')
                            prio = int(lock_objects.pop(0))
                            _lock(c_id,prio,lock_objects,message)
                        elif messageType == protocol.UNLOCK:
                            lock_objects = message.split('|')
                            prio = int(lock_objects.pop(0))
                            _unlock(c_id,prio,lock_objects)
                        elif messageType == protocol.LOCK_STOLEN_OK_REPLY:
                            client2sync = _waitstolen.get(message)
                            if client2sync is not None:
                                sync = client2sync.get(c_id)
                                if sync is not None:
                                    sync.set()
                        elif messageType == protocol.REDIS_QUERY:
                            _send_redis_info(c_id,local_connection)
                        elif messageType == protocol.POSIX_MQ_QUERY:
                            posix_queue = _send_posix_mq_connection(c_id,message)
                            if posix_queue:
                                r,_,_ = select.select([posix_queue.mqd],[],[],10.)
                                posix_queue.unlink()
                                if r:
                                    raw_data = posix_queue.receive()[0]
                                    messageType,message,raw_data = protocol.unpack_message(raw_data)
                                    if messageType != protocol.POSIX_MQ_OPENED:
                                        raise RuntimeError("Client didn't send open message")
                                else:
                                    raise RuntimeError("Client didn't send open message before timeout")
                                r_listen.insert(0,posix_queue.mqd)
                        elif messageType == protocol.CONFIG_GET_FILE:
                            _send_config_file(c_id,message)
                        elif messageType == protocol.CONFIG_GET_DB_BASE_PATH:
                            _send_config_db_files(c_id,message)
                        elif messageType == protocol.CONFIG_GET_DB_TREE:
                            _send_config_db_tree(c_id,message)
                        elif messageType == protocol.CONFIG_SET_DB_FILE:
                            _write_config_db_file(c_id,message)
                        elif messageType == protocol.CONFIG_REMOVE_FILE:
                            _remove_config_file(c_id,message)
                        elif messageType == protocol.CONFIG_MOVE_PATH:
                            _move_config_path(c_id,message)
                        elif messageType == protocol.CONFIG_GET_PYTHON_MODULE:
                            _get_python_module(c_id,message)
                        else:
                            _send_unknow_message(c_id,message)
                    except ValueError:
                        sys.excepthook(*sys.exc_info())
                        break
                    except protocol.IncompleteMessage:
                        r,_,_ = select.select(r_listen,[],[],.5)
                        if not r: # if timeout, something wired, close the connection
                           data = None
                           stopFlag = True
                        break
                    except:
                        sys.excepthook(*sys.exc_info())
                        _log.error('Error with client id %r, close it', client)
                        raise

                if fd == client:
                    tcp_data = data
                else:
                    posix_queue_data = data
    except:
        sys.excepthook(*sys.exc_info())
    finally:
        _clean(client)
        client.close()
        if posix_queue:
            posix_queue.close()
            _clean(posix_queue)


def sigterm_handler(_signo, _stack_frame):
    """On signal received, close the signal pipe to do a clean exit."""
    os.write(sig_write, '!')


def start_webserver(webapp_port, beacon_port, debug=True):
    try:
        import flask
    except ImportError:
        _wlog.error(
            "flask cannot be imported: web application won't be available")
        return

    from gevent.wsgi import WSGIServer
    from werkzeug.debug import DebuggedApplication
    from .web.config_app import web_app

    _wlog.info("Web application sitting on port: %s", webapp_port)
    web_app.debug = debug
    web_app.beacon_port = beacon_port
    application = DebuggedApplication(web_app, evalex=True)
    http_server = WSGIServer(('', webapp_port), application)
    http_server.family = socket.AF_INET
    gevent.spawn(http_server.serve_forever)


# Main execution

def main(args=None):
    # Monkey patch needed for web server
    # just keep for consistency because it's already patched
    # in __init__ in bliss project
    from gevent import monkey
    monkey.patch_all(thread=False)

    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--db_path", dest="db_path",
        default=os.environ.get("BEACON_DB_PATH", "./db"),
        help="database path")
    parser.add_argument(
        "--redis_port", dest="redis_port", default=6379, type=int,
        help="redis connection port")
    parser.add_argument(
        "--redis_conf", dest="redis_conf",
        default=redis_conf.get_redis_config_path(),
        help="path to alternative redis configuration file")
    parser.add_argument(
        "--posix_queue", dest="posix_queue", type=int, default=1,
        help="enable/disable posix_queue connection")
    parser.add_argument(
        "--port", dest="port", type=int,
        default=int(os.environ.get("BEACON_PORT", 0)),
        help="server port (default to BEACON_PORT environment variable, "
        "otherwise takes a free port)")
    parser.add_argument(
        "--tango_port", dest="tango_port", type=int, default=0,
        help="tango server port (default to 0: disable)")
    parser.add_argument(
        "--tango_debug_level", dest="tango_debug_level", type=int, default=0,
        help="tango debug level (default to 0: WARNING,1:INFO,2:DEBUG)")
    parser.add_argument(
        "--webapp_port", dest="webapp_port", type=int, default=0,
        help="web server port (default to 0: disable)")
    parser.add_argument(
        "--redis_socket", dest="redis_socket", default="/tmp/redis.sock",
        help="Unix socket for redis (default to /tmp/redis.sock)")
    parser.add_argument(
        '--log_level', default='INFO', type=str,
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR'],
        help='log level')

    global _options
    _options = parser.parse_args(args)

    # Logging configuration
    log_level = _options.log_level.upper()
    log_fmt = '%(levelname)s %(asctime)-15s %(name)s: %(message)s'
    logging.basicConfig(level=log_level, format=log_fmt)

    # Signal pipe
    global sig_write
    sig_read, sig_write = os.pipe()

    # Binds system signals.
    signal.signal(signal.SIGTERM, sigterm_handler)
    signal.signal(signal.SIGHUP, sigterm_handler)
    signal.signal(signal.SIGQUIT, sigterm_handler)

    # Pimp my path
    _options.db_path = os.path.abspath(os.path.expanduser(_options.db_path))

    # Posix queues
    if not _options.posix_queue:
        global posix_ipc
        posix_ipc = None

    # Broadcast
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp.bind(("", protocol.DEFAULT_UDP_SERVER_PORT))

    # TCP
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp.bind(("", _options.port))
    beacon_port = tcp.getsockname()[1]
    _log.info("server sitting on port: %s", beacon_port)
    _log.info("configuration path: %s", _options.db_path)
    tcp.listen(512)        # limit to 512 clients

    # Tango databaseds
    if _options.tango_port > 0:
        # Stdout pipe
        _tlog.info('Database started on port: %s', _options.tango_port)
        tango_rp, tango_wp = os.pipe()
        # Environment
        env = dict(os.environ)
        env["BEACON_HOST"] = '%s:%d' % ('localhost', beacon_port)
        # Tango database executable
        args = [sys.executable]
        # Should be:
        # args += ['-m', 'tango.databaseds.database']
        # But because of a pytango bug:
        code = 'import tango.databaseds.db_access.beacon\n'
        code += 'import tango.databaseds.database\n'
        code += 'tango.databaseds.database.main()'
        args += ['-c', code]
        # Arguments
        args += ['-l', str(_options.tango_debug_level)]
        args += ['--db_access', 'beacon']
        args += ['--port', str(_options.tango_port)]
        args += ['2']
        # Fire up process
        tango_process = subprocess.Popen(
            args, stdout=tango_wp, stderr=subprocess.STDOUT, env=env)
    else:
        tango_rp = tango_process = None

    # Web application
    if _options.webapp_port > 0:
        start_webserver(_options.webapp_port, beacon_port)

    # Start redis
    rp, wp = os.pipe()
    redis_process = subprocess.Popen(['redis-server', _options.redis_conf,
                                      '--unixsocket', _options.redis_socket,
                                      '--unixsocketperm', '777',
                                      '--port', '%d' % _options.redis_port],
                                     stdout=wp, stderr=subprocess.STDOUT,
                                     cwd=_options.db_path)

    # Safe context
    try:
        fd_list = [udp, tcp, rp, sig_read] + ([tango_rp] if tango_rp else [])
        logger = {rp: _rlog, tango_rp: _tlog}
        udp_reply = '%s|%d' % (socket.gethostname(), beacon_port)

        def events():
            """Flatten the selector events."""
            while True:
                rlist, _, _ = select.select(fd_list, [], [], 1)
                for s in rlist:
                    yield s
                if not rlist:
                    yield None

        # Event loop
        for s in events():

            # UDP case
            if s == udp:
                buff, address = udp.recvfrom(8192)
                if buff.find('Hello') > -1:
                    _log.info(
                        'address request from %s. Replying with %r',
                        address, udp_reply)
                    udp.sendto(udp_reply, address)

            # TCP case
            elif s == tcp:
                newSocket, addr = tcp.accept()
                newSocket.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                newSocket.setsockopt(
                    socket.SOL_IP, socket.IP_TOS, 0x10)
                localhost = addr[0] == '127.0.0.1'
                gevent.spawn(_client_rx, newSocket, localhost)

            # Signal interruption case
            elif s == sig_read:
                _log.info('Received an interruption signal!')
                return

            # Timeout case
            elif s is None:
                redis_exit_code = redis_process.poll()
                # Redis is not alive
                if redis_exit_code is not None:
                    _rlog.critical(
                        'redis exited with code %s. Bailing out!',
                        redis_exit_code)
                    redis_process = None
                    return

            # Logging case
            else:
                msg = os.read(s, 8192)
                # Log the message properly
                if msg:
                    logger.get(s, _log).info(msg)
                # Tango DB is not alive
                elif s == tango_rp:
                    fd_list.remove(tango_rp)
                    os.close(tango_rp)
                    logger.get(s, _log).warning('database exit')

    # Ignore keyboard interrupt
    except KeyboardInterrupt:
        _log.info('Received a keyboard interrupt!')
        return

    except Exception as exc:
        _log.critical('An expected exception occured:\n%r', exc)

    # Cleanup
    finally:
        _log.info('Cleaning up the subprocesses')
        if redis_process:
            redis_process.terminate()
        if tango_process:
            tango_process.terminate()


if __name__ == "__main__":
    main()
