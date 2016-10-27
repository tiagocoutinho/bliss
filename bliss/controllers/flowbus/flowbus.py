# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

"""
Bronkhorst flowbus

Used at ESRF ID31,
"""

# Flowbus serial line configuration:
# baudrate 38400
# charlength 8
# newline \n (0x10)
# partity none
# stopbits 1


import enum
import struct
import logging
import weakref
import functools
import collections

from bliss.comm.util import get_interface


class Encoding(enum.Enum):
    ASCII = 0
    DDE   = 1


class DataType(enum.Enum):
    CHAR    = 0, 0x00
    INT8    = 1, 0x00
    INT16   = 2, 0x20
    FLOAT32 = 3, 0x40
    INT32   = 4, 0x40
    STR     = 5, 0x60


class CommandType(enum.Enum):
    '''Communication command types'''
    #: Status command
    STATUS          = 0x00
    #: Send parameter with destination address, status reply requested
    SEND_REPLY      = 0x01
    #: Send parameter with destination address, status reply *NOT* requested
    SEND            = 0x02
    SEND_SRC        = 0x03
    #: Request parameter
    REQUEST         = 0x04
    SEND_REPEAT     = 0x05
    STOP_PROCESS    = 0x06
    START_PROCESS   = 0x07
    CLAIM_PROCESS   = 0x08
    UNCLAIM_PROCESS = 0x09


class Chain(enum.Enum):
    NOT_CHAINED  = 0x00
    CHAINED      = 0x80


__ERR_CODE_04 = {
    0x00: "No error",
    0x01: "Process claimed",
    0x02: "Command error",
    0x03: "Process error",
    0x04: "Parameter error",
    0x05: "Parameter type error",
    0x06: "Parameter value error",
    0x07: "Network not active",
    0x08: "Time-out start character",
    0x09: "Time-out serial line",
    0x0A: "Hardware memory error",
    0x0B: "Node number error",
    0x0C: "General communication error",
    0x0D: "Read only parameter",
    0x0E: "Error PC-communication",
    0x0F: "No RS232 connection",
    0x10: "PC out of memory",
    0x11: "Write only parameter",
    0x12: "System configuration unknown",
    0x13: "No free node address",
    0x14: "Wrong interface type",
    0x15: "Error serial port connection",
    0x16: "Error opening communication",
    0x17: "Communication error",
    0x18: "Error interface bus master",
    0x19: "Timeout answer",
    0x1A: "No start character",
    0x1B: "Error first digit",
    0x1C: "Buffer overflow in host",
    0x1D: "Buffer overflow",
    0x1E: "No answer found",
    0x1F: "Error closing communication",
    0x20: "Synchronisation error",
    0x21: "Send error",
    0x22: "Protocol error",
    0x23: "Buffer overflow in module",
    0xff: "UNDEFINED ERROR!"
}


__ERR_CODE_01 = {
    0x03: "propar protocol error",
    0x04: "propar protocol error (or CRC error)",
    0x05: "destination node address rejected",
    0x09: "response message timeout",
    0x01: "general error",
    0x02: "general error",
    0x08: "general error",
    0xff: "UNDEFINED ERROR!"
}


class FlowBusError(Exception):
    pass


def _check_error(ans, raise_exc=True):
    r1 = ans[1:3]
    if r1 == '01':
        err = int(ans[3:5], 16)
        try:
            err_str = __ERR_CODE_01[err]
        except:
            err_str = __ERR_CODE_01[0xff]
        if raise_exc:
            raise FlowBusError(err_str)
        return err, err_str

    if r1 != '04':
        return None

    if  ans[5:7] != '00':
        return None
    else:
        err = int(ans[7:9], 16)
        if err == 0:
            return
        try:
            err_str = __ERR_CODE_04[err]
        except:
            err_str = __ERR_CODE_04[0xff]
        if raise_exc:
            raise FlowBusError(err_str)
        return err, err_str


def encode_read(node, process_nb, param_nb, dtype,
                process_chained=Chain.NOT_CHAINED,
                param_chained=Chain.NOT_CHAINED,
                param_index=0, encoding=Encoding.ASCII):

    if encoding != Encoding.ASCII:
        raise ValueError('Only ASCII encoding supported so far')

    _, dtype_value = dtype.value
    pars = [node,                                            # node address
            CommandType.REQUEST.value,                       # command type (read)
            process_chained.value | process_nb,              # process chained | process number
            param_chained.value | dtype_value | param_index, # parameter chained | parameter type | parameter index
            process_nb,                                      # process number
            dtype_value | param_nb]                          # parameter type | parameter number (FBnr)

    # append length = 0 for strings
    if dtype == DataType.STR:
        pars.append(0)

    pars.insert(0, len(pars))

    cmd = ':' + ''.join(('%02X' % c for c in pars))
    return cmd


def encode_write(node, process_nb, param_nb, dtype, value,
                 command=CommandType.SEND_REPLY,
                 process_chained=Chain.NOT_CHAINED,
                 param_chained=Chain.NOT_CHAINED,
                 encoding=Encoding.ASCII):

    if encoding != Encoding.ASCII:
        raise ValueError('Only ASCII encoding supported so far')

    if command not in (CommandType.SEND, CommandType.SEND_REPLY):
        raise ValueError('Can only build SEND or SEND_REPLY write commands')

    _, dtype_value = dtype.value
    pars = [node,                                         # node address
            command.value,                                # command type (send/send_reply)
            process_chained.value | process_nb,           # process chained | process number
            param_chained.value | dtype_value | param_nb] # parameter chained | parameter type | parameter number (FBnr)

    if dtype == DataType.STR:
        pars.append(0)
        value = value.encode('hex') + '00'
    elif dtype == DataType.CHAR:
        value = value.encode('hex')
    elif dtype == DataType.INT8:
        value = '%02X' % value
    elif dtype == DataType.INT16:
        value = '%04X' % value
    elif dtype == DataType.INT32:
        value = '%08X' % value
    else:
        raise TypeError('Cannot encode write type %s' % dtype)

    pars.insert(0, len(pars) + len(value)/2)

    cmd = ':' + ''.join(('%02X' % c for c in pars)) + value
    return cmd


def decode(data, dtype=None):
    if data[0] != ':':
        raise ValueError('Can only decode ASCII messages')

    #     :  Length Node Command Proc Param
    idx = 1 +  2   +  2 +    2    +  2 +  2
    if dtype == DataType.INT8:
        val = struct.unpack('!b', data[idx:].decode('hex'))[0]
    elif dtype == DataType.INT16:
        val = struct.unpack('!h', data[idx:].decode('hex'))[0]
    elif dtype == DataType.INT32:
        val = struct.unpack('!i', data[idx:].decode('hex'))[0]
    elif dtype == DataType.FLOAT32:
        val = struct.unpack('!f', data[idx:].decode('hex'))[0]
    elif dtype == DataType.STR:
        idx += 2 # string length
        if data.endswith('00'):
            val = data[idx:-2].decode('hex')
        else:
            val = data[idx:].decode('hex')
    else:
        raise TypeError('Cannot decode %s' % dtype)
    return val


class FlowBus(object):

    def __init__(self, name, serial, auto_scan=True):
        """
        Expects *serial* argument to be a dict<str:obj>. Example:

            FlowBus("my_flowbus", serial={'port':'/dev/ttyRP16'})
        """
        self.name = name
        if not isinstance(serial, collections.Mapping):
            raise ValueError("expect 'serial' to be a dict<str:obj>")
        serial.setdefault('baudrate', 38400)
        self.__comm, _, _ = get_interface(serial=serial)
        self._log = logging.getLogger('FlowBus({0})'.format(name))
        if auto_scan:
            self.nodes = self.scan()
        else:
            self.nodes = {}

    @property
    def comm(self):
        return self.__comm

    def write_readline(self, cmd):
        log_cmd = cmd.replace("\n", r"\n").replace("\r", r"\r")
        self._log.debug("Writing <%s>", log_cmd)
        ans = self.__comm.write_readline(cmd)
        log_ans = ans.replace("\n", r"\n").replace("\r", r"\r")
        self._log.debug("Reading <%s>", log_ans)
        return ans

    def putget(self, cmd):
        cmd = '{0}\r{1}'.format(cmd, self.__comm._eol)
        result = self.write_readline(cmd)
        if result.endswith('\r'):
            result = result[:-1]
        return result

    def scan(self, start_at=2):
        nodes = {}
        node = start_at
        while True:
            try:
                node_obj = FlowBusNode(self, node)
            except FlowBusError:
                break
            nodes[node] = node_obj
            node = node_obj.nna
            if node == 0:
                break
        return nodes

    def __getitem__(self, item):
        return self.nodes[item]

    def __str__(self):
        nodes_str = "\n  ".join((str(node) for node in self.nodes.values()))
        return 'FlowBus(name={0.name}, nodes=[\n  {1}])'.format(self, nodes_str)


TYPE_MAP = {
    'int8'  : DataType.INT8,
    'int16' : DataType.INT16,
    'int32' : DataType.INT32,
    'float32' : DataType.FLOAT32,
    'str' : DataType.STR,
}


class Param(object):

    def __init__(self, param_info):
        self.info = param_info
        dtype = TYPE_MAP[param_info['type']]
        self.info['dtype'] = dtype
        self.name = param_info['DDE']

    def __getitem__(self, item):
        return self.info[item]

    def read(self, obj):
        dtype = self['dtype']
        cmd = encode_read(obj.node, self['process'], self['parameter'], dtype)
        result = obj.putget(cmd)
        _check_error(result)
        value = decode(result, dtype=dtype)
        scale = self['scale']
        if scale:
            value /= scale
        obj.cache[self.name] = value
        return value

    def write(self, obj, value):
        scale = self['scale']
        dtype = self['dtype']
        if scale is not None:
            value *= scale
            if dtype != DataType.FLOAT32:
                value = int(value)
        cmd = encode_write(obj.node, self['process'], self['parameter'], dtype,
                           value)
        result = obj.putget(cmd)
        _check_error(result)

    def get(self, obj):
        if not self.info['read']:
            raise FlowBusError('%s is not readable' % self.name)
        if obj is None:
            return self
        if not self['cache'] or self.name not in obj.cache:
            self.read(obj)
        return obj.cache[self.name]

    def set(self, obj, value):
        if not self.info['write']:
            raise FlowBusError('%s is not writable' % self.name)
        self.write(obj, value)
        if self['cache']:
            obj.cache[self.name] = value

    def __get__(self, obj, otype):
        return self.get(obj)

    def __set__(self, obj, value):
        return self.set(obj, value)


class FlowBusNode(object):

    def __init__(self, flowbus, node, auto_init=True):
        self.__flowbus = weakref.ref(flowbus)
        log_name = flowbus._log.name + "." + str(node)
        self._log = logging.getLogger(log_name)
        self.node = node
        self.cache = {}
        if auto_init:
            self.__initialize__()

    def __initialize__(self):
        nna = self.nna # next node address
        pna = self.pna # primary node address
        assert(self.node == pna)

    def __str__(self):
        return 'Node(pna={0.pna}, nna={0.nna}, type={0.DeviceType}, ' \
            'model={0.ModelNum}, ID={0.identstrng})'.format(self)

    def __repr__(self):
        return 'FlowBusNode(pna={0.pna}, nna={0.nna}, type={0.DeviceType}, ' \
            'model={0.ModelNum}, ID={0.identstrng}) object at 0x{1:x}'.format(self, id(self))

    def putget(self, cmd):
        return self.flowbus.putget(cmd)

    @property
    def flowbus(self):
        return self.__flowbus()


from .mapping import PARAM_DDE_MAP

for param_name, param_info in PARAM_DDE_MAP.items():
    if hasattr(FlowBusNode, param_name):
        continue
    setattr(FlowBusNode, param_name, Param(param_info))


def main():
    import argparse
    parser = argparse.ArgumentParser(description=main.__doc__)

    parser.add_argument('--log-level', type=str, default='debug',
                        choices=['debug', 'info', 'warning', 'error'],
                        help='log level [default: info]')
    parser.add_argument('serial', type=str, help='serial line address (ex: /dev/ttyRP01)')

    args = parser.parse_args()
    vargs = vars(args)
    log_level = getattr(logging, vargs.pop('log_level').upper())
    logging.basicConfig(level=log_level,
                        format='%(asctime)s %(levelname)s %(name)s: %(message)s')

    flowbus = FlowBus('toto', serial={'port': args.serial})
    print flowbus

if __name__ == "__main__":
    main()

# EOF
