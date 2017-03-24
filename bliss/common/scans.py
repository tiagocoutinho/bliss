# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

"""
Most common scan procedures (:func:`~bliss.common.scans.ascan`, \
:func:`~bliss.common.scans.dscan`, :func:`~bliss.common.scans.timescan`, etc)

"""

__all__ = ['ascan', 'a2scan', 'dscan', 'd2scan', 'timescan', 'ct']

import time
import logging

import numpy
import gevent

from bliss import setup_globals
from bliss.common.task_utils import *
from bliss.controllers.motor_group import Group
from bliss.common.measurement import CounterBase
from bliss.scanning.acquisition.counter import CounterAcqDevice
from bliss.scanning.chain import AcquisitionChain
from bliss.scanning import scan as scan_module
from bliss.scanning.acquisition.timer import SoftwareTimerMaster
from bliss.scanning.acquisition.motor import LinearStepTriggerMaster
from bliss.session import session,measurementgroup
from bliss.scanning.writer import hdf5

_log = logging.getLogger('bliss.scans')

def default_chain(chain,scan_pars,extra_counters) :
    count_time = scan_pars.get('count_time',1)
    npoints = scan_pars.get('npoints',1)
    timer = SoftwareTimerMaster(count_time,npoints=npoints)
    meas = measurementgroup.get_active()
    if meas is not None:
        counters = filter(None,[setup_globals.__dict__.get(c) for c in meas.enable])
    else:
        counters = list()   # todo
    counters.extend(extra_counters)
    
    for cnt in set(counters):
        if isinstance(cnt, CounterBase):
            chain.add(timer, CounterAcqDevice(cnt, expo_time=count_time, npoints=npoints))
#      elif isinstance(cnt,Lima):
#          chain.add(timer, LimaAcqDevice()))

    return timer

            

def _do_scan(chain,scan_info) :
    scandata = scan_module.ScanSaving()
    config = scandata.get()
    root_path = config['root_path']
    writer = hdf5.Writer(root_path)
    scan_info['root_path'] = root_path
    scan_info['session_name'] = scandata.session
    scan_info['user_name'] = scandata.user_name
    scan_data_watch = scan_module.StepScanDataWatch(root_path,scan_info)
    scan_recorder = scan_module.ScanRecorder(parent=config['parent'],
                                             scan_info=scan_info,
                                             writer=writer,
                                             data_watch_callback=scan_data_watch)
    scan = scan_module.Scan(chain, scan_recorder)
    scan.prepare()
    scan.start()

def ascan(motor, start, stop, npoints, count_time, *extra_counters, **kwargs):
    """
    Absolute scan

    Scans one motor, as specified by *motor*. The motor starts at the position
    given by *start* and ends at the position given by *stop*. The step size is
    `(*start*-*stop*)/(*npoints*-1)`. The number of intervals will be
    *npoints*-1. Count time is given by *count_time* (seconds).

    At the end of the scan (even in case of error) the motor will return to
    its initial position

    Args:
        motor (Axis): motor to scan
        start (float): motor start position
        stop (float): motor end position
        npoints (int): the number of points
        count_time (float): count time (seconds)
        extra_counters (BaseCounter): additional counters

    Keyword Args:
        type (str): scan type [default: 'ascan')
        title (str): scan title [default: 'ascan <motor> ... <count_time>']
        filename (str): file name [default: current value returned by \
        :func:`scanfile`]
        save (bool): save scan data to file [default: True]
        user_name (str): current user
        session_name (str): session name [default: current session name or \
        'bliss' if not inside a session]
    """
    scan_type = kwargs.setdefault('type', 'ascan')
    if 'title' not in kwargs:
        args = scan_type, motor.name, start, stop, npoints, count_time
        template = " ".join(['{{{0}}}'.format(i) for i in range(len(args))])
        kwargs['title'] = template.format(*args)

    kwargs.setdefault('npoints', npoints)
    kwargs.setdefault('total_acq_time', npoints * count_time)
    kwargs.setdefault('motors', [motor])
    kwargs.setdefault('start', [start])
    kwargs.setdefault('stop', [stop])
    kwargs.setdefault('count_time', count_time)

    chain = AcquisitionChain(parallel_prepare=True)
    timer = default_chain(chain,kwargs,extra_counters)
    top_master = LinearStepTriggerMaster(motor,start,stop,npoints)
    chain.add(top_master,timer)

    _log.info("Scanning %s from %f to %f in %d points",
              motor.name, start, stop, npoints)

    _do_scan(chain,kwargs)

def dscan(motor, start, stop, npoints, count_time, *extra_counters, **kwargs):
    """
    Relative scan

    Scans one motor, as specified by *motor*. If the motor is at position *X*
    before the scan begins, the scan will run from `X+start` to `X+end`.
    The step size is `(*start*-*stop*)/(*npoints*-1)`. The number of intervals
    will be *npoints*-1. Count time is given by *count_time* (seconds).

    At the end of the scan (even in case of error) the motor will return to
    its initial position

    Args:
        motor (Axis): motor to scan
        start (float): motor relative start position
        stop (float): motor relative end position
        npoints (int): the number of points
        count_time (float): count time (seconds)
        extra_counters (BaseCounter): additional counters

    Keyword Args:
        type (str): scan type [default: 'ascan')
        title (str): scan title [default: 'dscan <motor> ... <count_time>']
        filename (str): file name [default: current value returned by \
        :func:`scanfile`]
        save (bool): save scan data to file [default: True]
        user_name (str): current user
        session_name (str): session name [default: current session name or \
        'bliss' if not inside a session]
    """
    kwargs.setdefault('type', 'dscan')
    oldpos = motor.position()
    ascan(motor, oldpos + start, oldpos + stop, npoints, count_time,
          *extra_counters, **kwargs)
    motor.move(oldpos)


def a2scan(motor1, start1, stop1, motor2, start2, stop2, npoints, count_time,
           *extra_counters, **kwargs):
    """
    Absolute 2 motor scan

    Scans two motors, as specified by *motor1* and *motor2*. The motors start
    at the positions given by *start1* and *start2* and end at the positions
    given by *stop1* and *stop2*. The step size for each motor is given by
    `(*start*-*stop*)/(*npoints*-1)`. The number of intervals will be
    *npoints*-1. Count time is given by *count_time* (seconds).

    At the end of the scan (even in case of error) the motors will return to
    its initial positions

    Args:
        motor1 (Axis): motor1 to scan
        start1 (float): motor1 start position
        stop1 (float): motor1 end position
        motor2 (Axis): motor2 to scan
        start2 (float): motor2 start position
        stop2 (float): motor2 end position
        npoints (int): the number of points
        count_time (float): count time (seconds)
        extra_counters (BaseCounter): additional counters

    Keyword Args:
        type (str): scan type [default: 'a2scan')
        title (str): scan title [default: 'a2scan <motor1> ... <count_time>']
        filename (str): file name [default: current value returned by \
        :func:`scanfile`]
        save (bool): save scan data to file [default: True]
        user_name (str): current user
        session_name (str): session name [default: current session name or \
        'bliss' if not inside a session]
    """
    scan_type = kwargs.setdefault('type', 'a2scan')
    if 'title' not in kwargs:
        args = scan_type, motor1.name, start1, stop1, \
               motor2.name, start2, stop2, npoints, count_time
        template = " ".join(['{{{0}}}'.format(i) for i in range(len(args))])
        kwargs['title'] = template.format(*args)

    kwargs.setdefault('npoints', npoints)
    kwargs.setdefault('total_acq_time', npoints * count_time)
    kwargs.setdefault('motors', [motor1,motor2])
    kwargs.setdefault('start', [start1,start2])
    kwargs.setdefault('stop', [stop1,stop2])
    kwargs.setdefault('count_time', count_time)

    chain = AcquisitionChain(parallel_prepare=True)
    timer = default_chain(chain,kwargs,extra_counters)
    top_master = LinearStepTriggerMaster(motor1,start1,stop1,npoints,
                                         motor2,start2,stop2,npoints)
    chain.add(top_master,timer)

    _log.info(
        "Scanning %s from %f to %f and %s from %f to %f in %d points",
        motor1.name, start1, stop1, motor2.name, start2, stop2, npoints)

    _do_scan(chain,kwargs)

def d2scan(motor1, start1, stop1, motor2, start2, stop2, npoints, count_time,
           *extra_counters, **kwargs):
    """
    Relative 2 motor scan

    Scans two motors, as specified by *motor1* and *motor2*. Each motor moves
    the same number of points. If a motor is at position *X*
    before the scan begins, the scan will run from `X+start` to `X+end`.
    The step size of a motor is `(*start*-*stop*)/(*npoints*-1)`. The number
    of intervals will be *npoints*-1. Count time is given by *count_time*
    (seconds).

    At the end of the scan (even in case of error) the motor will return to
    its initial position

    Args:
        motor1 (Axis): motor1 to scan
        start1 (float): motor1 relative start position
        stop1 (float): motor1 relative end position
        motor2 (Axis): motor2 to scan
        start2 (float): motor2 relative start position
        stop2 (float): motor2 relative end position
        npoints (int): the number of points
        count_time (float): count time (seconds)
        extra_counters (BaseCounter): additional counters

    Keyword Args:
        type (str): scan type [default: 'ascan')
        title (str): scan title [default: 'd2scan <motor1> ... <count_time>']
        filename (str): file name [default: current value returned by \
        :func:`scanfile`]
        save (bool): save scan data to file [default: True]
        user_name (str): current user
        session_name (str): session name [default: current session name or \
        'bliss' if not inside a session]
    """
    kwargs.setdefault('type', 'd2scan')

    oldpos1 = motor1.position()
    oldpos2 = motor2.position()

    a2scan(motor1, oldpos1 + start1, oldpos1+stop1, motor2, oldpos2 + start2,
           oldpos2 + stop2, npoints, count_time, *extra_counters, **kwargs)

    group = Group(motor1,motor2)
    group.move(motor1,oldpos1,motor2,oldpos2)


def timescan(count_time, *extra_counters, **kwargs):
    """
    Time scan

    Args:
        count_time (float): count time (seconds)
        extra_counters (BaseCounter): additional counters

    Keyword Args:
        sleep_time (float): sleep time (seconds) [default: 0]
        type (str): scan type [default: 'ascan')
        title (str): scan title [default: 'timescan <count_time>']
        filename (str): file name [default: current value returned by \
        :func:`scanfile`]
        save (bool): save scan data to file [default: True]
        user_name (str): current user
        session_name (str): session name [default: current session name or \
        'bliss' if not inside a session]
    """
    scan_type = kwargs.setdefault('type', 'timescan')
    if 'title' not in kwargs:
        args = scan_type, count_time
        template = " ".join(['{{{0}}}'.format(i) for i in range(len(args))])
        kwargs['title'] = template.format(*args)

    npoints = kwargs.setdefault("npoints", 0)
    kwargs.setdefault('motors', [])
    kwargs.setdefault('start', [])
    kwargs.setdefault('stop', [])
    kwargs.setdefault('count_time',count_time)

    kwargs['total_acq_time'] = npoints * count_time

    _log.info("Doing %s", scan_type)

    chain = AcquisitionChain(parallel_prepare=True)
    timer = default_chain(chain,kwargs,extra_counters)
    timer.timescan_mode = True

    _do_scan(chain,kwargs)

def ct(count_time, *counters, **kwargs):
    """
    Count for a specified time

    Note:
        This function blocks the current :class:`Greenlet`

    Args:
        count_time (float): count time (seconds)
        extra_counters (BaseCounter): additional counters

    Keyword Args:
        sleep_time (float): sleep time (seconds) [default: 0]
        type (str): scan type [default: 'ascan')
        title (str): scan title [default: 'ct <count_time>']
        filename (str): file name [default: current value returned by \
        :func:`scanfile`]
        save (bool): save scan data to file [default: True]
        user_name (str): current user
        session_name (str): session name [default: current session name or \
        'bliss' if not inside a session]
    """
    kwargs.setdefault('type', 'ct')
    kwargs.setdefault('save', False)
    kwargs['npoints'] = 1
    return timescan(count_time, *counters, **kwargs)

