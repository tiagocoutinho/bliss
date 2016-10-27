# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

"""
Bronkhorst flowbus emulator helper classes

To create a flowbus device use the following configuration as
a starting point:

.. code-block:: yaml

    name: my_emulator
    devices:
        - class: FlowBus
          module: flowbus
          interfaces:
              - type: serial

To start the server you can do something like:

    $ python -m bliss.controllers.emulator my_emulator

"""

import struct
import functools

from bliss.controllers.emulator import BaseDevice

# just cheating a bit... will get the parameter map from the flowbus
# not to have to generate it again here. Ideally they should be
# generated independently to be able to find inconsistencies in the
# parameter map.
from bliss.controllers.flowbus import PARAM_DDE_MAP, PARAM_PROC_FB_MAP


def _int(data, size=None):
    if size is None:
        length = len(data)
    else:
        length = 2 * (size / 8)
    if length == 2:
        fmt = '!b'
    elif length == 4:
        fmt = '!h'
    elif length == 8:
        fmt = '!i'
    else:
        raise ValueError("Don't know how to unpack int from string size %d"
                         % length)
    return struct.unpack(fmt, data.decode('hex'))[0]

int8 = functools.partial(_int, size=8)
int16 = functools.partial(_int, size=16)
int32 = functools.partial(_int, size=32)

def float32(data):
    return struct.unpack('!f', data[:8].decode('hex'))[0]

def _str(data):
    if data.startswith('00'):
        data = data[2:-2]
    else:
        data = data[2:]
    return data.decode('hex')

CHAINED = 0x80

TYPE_MAP = {
    'int8': 0x00,
    'int16': 0x20,
    'int32': 0x40,
    'float32': 0x40,
    'str': 0x60
}


def get_parameter_info(process, parameter_nb):
    try:
        return PARAM_PROC_FB_MAP[(process, parameter_nb)]
    except KeyError:
        if process > 0 and process < 10:
            return PARAM_PROC_FB_MAP[(None, parameter_nb)]
        raise


def _build_nodes(nodes):
    """
    Build a node map from a sequence of node information (probably coming from
    a configuration
    """
    nodes = list(nodes or ())
    node_map = {}
    # if nodes are not given, emulate nodes 3, 5 and 7 are installed
    if nodes is None:
        nodes = [dict(pna=3), dict(pna=5), dict(pna=7)]

    # make sure an operation module (E-8000) is present
    for node in nodes:
        if node['pna'] == 2:
            break
    else:
        node_info = dict(pna=2, DeviceType='E-800', ModelNum='E-801')
        nodes.insert(0, node_info)

    # Fill nodes with default values
    prev_node = None
    for node in nodes:
        pna = node['pna']
        node.setdefault('identstrng', '7SN%06d' % (pna,))
        node.setdefault('SerialNum', 'SN%06dA' % (pna,))
        for param_dde, param_info in PARAM_DDE_MAP.items():
            node.setdefault(param_dde, param_info['default'])
        node_map[pna] = node
        node['nna'] = 0
        if prev_node is not None:
            prev_node['nna'] = pna
        prev_node = node
    return node_map


class FlowBusError(Exception):
    pass


class FlowBus(BaseDevice):
    """Bronkhorst flowbus device emulator"""

    def __init__(self, name, nodes=None, **opts):
        super(FlowBus, self).__init__(name)
        self.node_map = _build_nodes(nodes)

    def __getitem__(self, pna):
        return self.node_map[pna]

    def handle_line(self, line):
        line = line.strip()
        self._log.debug("processing line '%s'", line)
        if not line.startswith(':'):
            self._log.error('cannot handle non ASCII requests yet!')
            return ':\r\n'
        length = int8(line[1:3])
        pna = int8(line[3:5])
        if pna not in self.node_map:
            status = 0x0B # node number error
            return ':04{0:02X}00{1:02X}05\r\n'.format(pna, status)
        command = int8(line[5:7])
        if command == 0x04: # request parameter
            result = self.handle_read(pna, line)
        elif command in (0x01, 0x02):
            result = self.handle_write(pna, command, line)
        else:
            result = ''
            self._log.error('can only handle request parameters')
        if result is not None:
            self._log.debug("answering with ':%s'", result)
            return ':' + result + '\r\n'

    def handle_write(self, pna, command, line):
        proc = line[7:9]
        proc_data = int8(proc)
        proc_chained = proc_data & CHAINED
        proc_id = proc_data & 0b01111111

        param = line[9:11]
        param_data = int8(param)
        param_chained = param_data & CHAINED
        param_type = param_data & 0b01100000
        param_nb = param_data &0b00011111
        value = line[11:]

        if proc_chained:
            self._log.error('cannot handle chained process write yet')
            return ''

        if param_chained:
            self._log.error('cannot handle chained parameter requests yet')
            return ''

        status = 0
        try:
            self.set_value(pna, proc_id, param_nb, param_type, value)
        except FlowBusError as fbe:
            status = fbe.message

        if command == 0x01:
            return '04{0:02X}00{1:02X}05'.format(pna, status)

    def set_value(self, pna, process, param, dtype, value):
        param_info = get_parameter_info(process, param)
        dde_name = param_info['DDE']
        node_type = param_info['type']

        node = self[pna]

        if TYPE_MAP[node_type] != dtype:
            raise FlowBusError(0x05)

        if node_type == 'int8':
            result = int8(value)
        elif node_type == 'int16':
            result = int16(value)
        elif node_type == 'int32':
            result = int32(value)
        elif node_type == 'float32':
            result = float32(value)
        elif node_type == 'str':
            result = _str(value)

        node[dde_name] = result
        self._log.debug("set_value for '%s' with %s ('%s')",
                        dde_name, result, value)

    def handle_read(self, pna, line):
        proc1 = line[7:9]
        proc1_data = int8(proc1)
        proc_chained = proc1_data & CHAINED
        proc_id = proc1_data & 0b01111111

        param1 = line[9:11]
        param1_data = int8(param1)
        param_chained = param1_data & CHAINED
        param_type = param1_data & 0b01100000
        param_index = param1_data &0b00011111

        proc2 = line[11:13]
        param2 = line[13:15]
        param2_data = int8(param2)
        param_nb = param2_data &0b00011111

        if proc_chained:
            self._log.error('cannot handle chained process requests yet')
            return

        if param_chained & CHAINED:
            self._log.error('cannot handle chained parameter requests yet')
            return

        value = self.get_value(pna, proc_id, param_nb, param_type)

        response = line[3:5] + '02' + proc1 + param1 + value
        length = '%02X' % (len(response)/2,)
        return length + response

    def get_value(self, pna, process, param, dtype):
        param_info = get_parameter_info(process, param)
        dde_name = param_info['DDE']
        node_type = param_info['type']
        node = self[pna]

        value = node[dde_name]

        if node_type == 'int8':
            result = '%02X' % value
        elif node_type == 'int16':
            result = '%04X' % value
        elif node_type == 'int32':
            result = '%08X' % value
        elif node_type == 'float32':
            raise NotImplementedError
        elif node_type == 'str':
            result = '00' + value.encode('hex') + '00'
        self._log.debug("get_value for '%s' is %s ('%s')", dde_name, value,
                        result)
        return result
