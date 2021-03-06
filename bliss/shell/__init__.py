# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

'''Shell (:term:`CLI` and Web based)'''

import os
import sys
import time
import logging
import datetime
import platform
import functools

import numpy
from six import print_
from blessings import Terminal

from bliss import setup_globals
from bliss.config import static
from bliss.scanning import scan
from bliss.common.event import dispatcher
from bliss.config.conductor.client import get_default_connection
from bliss.shell.bliss_banners import print_rainbow_banner
_log = logging.getLogger('bliss.shell')


def initialize(session_name):
    config = static.get_config()
    user_ns = {"config": config}
    error_flag = False

    """ BLISS CLI welcome messages """
    t = Terminal()

    # Version
    # TODO : define an elegant version numbering.
    _version = "version 0.01"

    # Hostname
    _hostname = platform.node()

    # Beacon host/port
    try:
        _host = get_default_connection()._host
        _port = str(get_default_connection()._port)
    except:
        _host = "UNKNOWN"
        _port = "UNKNOWN"

    # Conda environment
    try:
        _conda_env = "(in {t.blue}%s{t.normal} Conda environment)".format(t=t) % os.environ['CONDA_DEFAULT_ENV']
    except KeyError:
        _conda_env = ""

    print_rainbow_banner()
    print_("")
    print_("Welcome to BLISS %s running on {t.blue}%s{t.normal} %s".format(t=t) % (_version, _hostname, _conda_env))
    print_("Copyright (c) ESRF, 2015-2017")
    print_("-")
    print_("Connected to Beacon server on {t.blue}%s{t.normal} (port %s)".format(t=t) % (_host, _port))

    """ Setup(s) """
    if session_name is not None:
        session = config.get(session_name)

        print "%s: Executing setup..." % session.name

        try:
            session.setup(env_dict=user_ns, verbose=True)
        except Exception:
            error_flag = True
            sys.excepthook(*sys.exc_info())
    else:
        session = None

    if error_flag:
        print "Warning: error(s) happened during setup, setup may not be complete."
    else:
        print_("Done.")
        print_("")

    return user_ns, session


def _find_unit(obj):
    try:
        if hasattr(obj, 'unit'):
            return obj.unit
        if hasattr(obj, 'config'):
            return obj.config.get('unit')
        if hasattr(obj, 'controller'):
            return _find_unit(obj.controller)
    except:
        return


class ScanListener:
    '''listen to scan events and compose output'''

    HEADER = "Total {npoints} points{estimation_str}\n" + \
             "{not_shown_counters_str}\n" + \
             "Scan {scan_nb} {start_time_str} {root_path} " + \
             "{session_name} user = {user_name}\n" + \
             "{title}\n\n" + \
             "{column_header}"

    DEFAULT_WIDTH = 12

    def __init__(self):
        dispatcher.connect(self.__on_scan_new, 'scan_new', scan)
        dispatcher.connect(self.__on_scan_data, 'scan_data', scan)
        dispatcher.connect(self.__on_scan_end, 'scan_end', scan)

    def __on_scan_new(self, scan_info):
        scan_info = dict(scan_info)
        self.term = term = Terminal(scan_info.get('stream'))
        scan_info = dict(scan_info)

        motors = scan_info['motors']
        counters = scan_info['counters']
        nb_points = scan_info['npoints']
        if not scan_info['save']:
            scan_info['root_path'] = '<no saving>'
        col_labels = ['#']
        real_motors = []
        for motor in motors:
            motor_name = motor.name
            # replace time_stamp with elapsed_time
            if motor_name == 'timestamp':
                motor_name = 'dt'
                unit = 's'
            else:
                real_motors.append(motor)
                if term.is_a_tty:
                    dispatcher.connect(self.__on_motor_position_changed,
                                       signal='position', sender=motor)
                unit = motor.config.get('unit', default=None)
            motor_label = motor_name
            if unit:
                motor_label += '({0})'.format(unit)
            col_labels.append(motor_label)

        for counter in counters:
            counter_label = counter.name
            unit = _find_unit(counter)
            if unit:
                counter_label += '({0})'.format(unit)
            col_labels.append(counter_label)

        self.col_labels = col_labels
        self.real_motors = real_motors
        self._point_nb = 0

        if not scan_info['save']:
            scan_info['root_path'] = '<no file>'

        if scan_info['type'] == 'ct':
            return

        estimation = scan_info.get('estimation')
        if estimation:
            total = datetime.timedelta(seconds=estimation['total_time'])
            motion = datetime.timedelta(seconds=estimation['total_motion_time'])
            count = datetime.timedelta(seconds=estimation['total_count_time'])
            estimation_str = ', {0} (motion: {1}, count: {2})'.format(total, motion, count)
        else:
            estimation_str = ''

        other_counters = scan_info.get('other_counters', list())
        if other_counters:
            not_shown_counters_str = 'Activated counters not shown: %s\n' % \
                                     ', '.join((c.name for c in other_counters))
        else:
            not_shown_counters_str = ''

        col_lens = map(lambda x: max(len(x), self.DEFAULT_WIDTH), col_labels)
        h_templ = ["{{0:>{width}}}".format(width=col_len)
                   for col_len in col_lens]
        header = "  ".join([templ.format(label)
                            for templ, label in zip(h_templ, col_labels)])
        header = self.HEADER.format(column_header=header,
                                    estimation_str=estimation_str,
                                    not_shown_counters_str=not_shown_counters_str,
                                    **scan_info)
        self.col_templ = ["{{0: >{width}g}}".format(width=col_len)
                          for col_len in col_lens]
        print_(header)

    def __on_scan_data(self, scan_info, values):
        elapsed_time = values['timestamp'] - scan_info['start_time_stamp']
        motors = scan_info['motors'][1:]  # take out timestamp placeholder
        motor_values = [values[m.name] for m in motors]
        counter_values = [values[c.name] for c in scan_info['counters']]
        values = [elapsed_time] + motor_values + counter_values
        if scan_info['type'] == 'ct':
            # ct is actually a timescan(npoints=1).
            norm_values = numpy.array(values) / scan_info['count_time']
            col_len = max(map(len, self.col_labels)) + 2
            template = '{{label:>{0}}} = {{value: >12}} ({{norm: 12}}/s)'.format(col_len)
            lines = "\n".join([template.format(label=label, value=v, norm=nv)
                               for label, v, nv in zip(self.col_labels[1:],
                                                       values, norm_values)])
            end_time_str = datetime.datetime.now().strftime("%a %b %d %H:%M:%S %Y")
            msg = '\n{0}\n\n{1}'.format(end_time_str, lines)
            print_(msg)
        else:
            values.insert(0, self._point_nb)
            self._point_nb += 1
            line = "  ".join([self.col_templ[i].format(v) for i, v in enumerate(values)])
            if self.term.is_a_tty:
                monitor = scan_info.get('output_mode', 'tail') == 'monitor'
                print_('\r' + line, end=monitor and '\r' or '\n', flush=True)
            else:
                print_(line)

    def __on_scan_end(self, scan_info):
        if scan_info['type'] == 'ct':
            return

        for motor in self.real_motors:
            dispatcher.disconnect(self.__on_motor_position_changed,
                                  signal='position', sender=motor)

        end = datetime.datetime.fromtimestamp(time.time())
        start = datetime.datetime.fromtimestamp(scan_info['start_time_stamp'])
        dt = end - start
        if scan_info.get('output_mode', 'tail') == 'monitor' and self.term.is_a_tty:
            print_()
        msg = '\nTook {0}'.format(dt)
        if 'estimation' in scan_info:
            time_estimation = scan_info['estimation']['total_time']
            msg += ' (estimation was for {0})'.format(datetime.timedelta(seconds=time_estimation))
        print_(msg)

    def __on_motor_position_changed(self, position, signal=None, sender=None):
        labels = []
        for motor in self.real_motors:
            position = '{0:.03f}'.format(motor.position())
            unit = motor.config.get('unit', default=None)
            if unit:
                position += unit
            labels.append('{0}: {1}'.format(motor.name, position))

        print_('\33[2K', end='')
        print_(*labels, sep=', ', end='\r', flush=True)
