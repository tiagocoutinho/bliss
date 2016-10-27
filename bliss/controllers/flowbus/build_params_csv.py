# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

"""
Build a python file containing dictionary with bronkhorst parameter information \
from a tab separated .CSV file
"""

import sys
import logging
import collections

try:
    OrderedDict = collections.OrderedDict
except AttributeError:
    from ordereddict import OrderedDict


TEMPLATE = """\
# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

\"\"\"
Generated bronkhorst flowbus parameters file
\"\"\"

__all__ = ['PARAM_DDE_MAP', 'PARAM_PROC_FB_MAP']

#: parameter map where key is the DDE name
PARAM_DDE_MAP = {params}

def __build_param_proc_fb_map(dde_map):
    r = dict()
    for param_dde_name, param_info in dde_map.items():
        key = param_info['process'], param_info['parameter']
        r[key] = param_info
    return r

#: parameter map where key is the tuple (process, param_nb)
#: (useful because read/write request come with process and parameter number)
PARAM_PROC_FB_MAP = __build_param_proc_fb_map(PARAM_DDE_MAP)

"""


def to_py(text):
    text = text.strip()
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            if text == 'Yes': return True
            elif text == 'No': return False
    if not text:
        return None
    return text


def process(filename):

    params = OrderedDict()

    with open(filename) as f:
        header = f.readline().split('\t')
        for line in f.readlines():
            pars = line.split('\t')
            py_pars = map(to_py, pars)
            name = py_pars[1]
            name_clean = ''.join(c for c in name.strip().lower() if c.isalnum() or c == ' ')
            name_clean = name_clean.replace(' ', '_')
            py_pars_dict = dict(zip(header, py_pars))
            py_pars_dict['name'] = name_clean
            pid = py_pars_dict['DDE']
            if pid in params:
                print("Error: Double parameter '{0} ({1})'".format(name, name_clean))
                sys.exit(1)

            # clean up data types
            dtype, length = py_pars_dict['type'], py_pars_dict['length']
            if dtype == 'c':
                if length is None:
                    dtype = 'int8'
                else:
                    dtype = 'str'
            elif dtype == 'i':
                dtype = 'int16'
            elif dtype == 'l':
                dtype = 'int32'
            elif dtype == 'f':
                dtype = 'float32'
            py_pars_dict['type'] = dtype
            params[pid] = py_pars_dict
    return params


def main():
    import argparse
    parser = argparse.ArgumentParser(description=main.__doc__)

    parser.add_argument('--log-level', type=str, default='debug',
                        choices=['debug', 'info', 'warning', 'error'],
                        help='log level [default: info]')
    parser.add_argument('filename', type=str, help='CSV filename (must be tab separated)')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='python output filename')

    args = parser.parse_args()
    log_level = getattr(logging, args.log_level.upper())
    logging.basicConfig(level=log_level,
                        format='%(asctime)s %(levelname)s %(name)s: %(message)s')

    params = process(args.filename)

    output = sys.stdout
    need_close = False
    if args.output:
        output = open(args.output, mode='w')
        need_close = True

    import pprint
    output.write(TEMPLATE.format(params=pprint.pformat(dict(params))))

    if need_close:
        output.close()

if __name__ == '__main__':
    main()
