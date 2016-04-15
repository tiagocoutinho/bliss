# -*- coding: utf-8 -*-
#
# This file is part of the CT2 project
#
# Copyright (c) : 2015
# Beamline Control Unit, European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# Distributed under the terms of the GNU Lesser General Public License,
# either version 3 of the License, or (at your option) any later version.
# See LICENSE.txt for more info.

import sys
import numpy
import gevent
from gevent import lock
from gevent import select
from louie import dispatcher

try:
    import enum
except:
    from . import enum34 as enum

from . import ct2


ErrorSignal = "error"
StatusSignal = "status"
PointNbSignal = "point_nb"


class AcqMode(enum.Enum):
    """Acquisition mode enumeration"""

    #: Software start + internal timer trigger readout
    IntTrigReadout = 0

    #: Software start + software trigger readout
    SoftTrigReadout = 1

    #: Software start + software trigger int exposure
    IntTrigMulti = 2


class AcqStatus(enum.Enum):
    """Acquisition status"""

    #: Ready to acquire
    Ready = 0

    #: Acquiring data
    Running = 1


class BaseCT2Device(object):
    """Base abstract class for a CT2Device"""

    internal_timer_counter = 11
    internal_point_nb_counter = 12

    IntClockSrc = {
        1.25E3: ct2.CtClockSrc.CLK_1_25_KHz,
        10E3:   ct2.CtClockSrc.CLK_10_KHz,
        125E3:  ct2.CtClockSrc.CLK_125_KHz,
        1E6:    ct2.CtClockSrc.CLK_1_MHz,
        12.5E6: ct2.CtClockSrc.CLK_12_5_MHz,
        100E6:  ct2.CtClockSrc.CLK_100_MHz,
    }

    def __init__(self, config, name):
        self.__config = config
        self.__name = name

    # helper methods to fire events

    def _send_error(self, error):
        dispatcher.send(ErrorSignal, self, error)

    def _send_point_nb(self, point_nb):
        dispatcher.send(PointNbSignal, self, point_nb)

    def _send_status(self, status):
        dispatcher.send(StatusSignal, self, status)

    @property
    def config(self):
        return self.__config

    @property
    def card_config(self):
        return self.config.get_config(self.__name)

    @property
    def name(self):
        return self.__name

    @property
    def _device(self):
        raise NotImplementedError

    @property
    def use_mmap(self):
        return self._device.use_mmap

    @use_mmap.setter
    def use_mmap(self, use_mmap):
        self._device.use_mmap = use_mmap

    @property
    def acq_mode(self):
        return self._device.acq_mode

    @acq_mode.setter
    def acq_mode(self, acq_mode):
        self._device.acq_mode = acq_mode

    @property
    def acq_status(self):
        return self._device.acq_status

    @property
    def acq_nb_points(self):
        return self._device.acq_nb_points

    @acq_nb_points.setter
    def acq_nb_points(self, acq_nb_points):
        self._device.acq_nb_points = acq_nb_points

    @property
    def acq_expo_time(self):
        return self._device.acq_expo_time

    @acq_expo_time.setter
    def acq_expo_time(self, acq_expo_time):
        self._device.acq_expo_time = acq_expo_time

    @property
    def acq_channels(self):
        return self._device.acq_channels

    @acq_channels.setter
    def acq_channels(self, acq_channels):
        self._device.acq_channels = acq_channels

    def reset(self):
        self._device.reset()

    def prepare_acq(self):
        self._device.prepare_acq()

    def start_acq(self):
        self._device.start_acq()

    def stop_acq(self):
        self._device.stop_acq()

    def apply_config():
        self._device.apply_config()

    def read_data(self):
        self._device.read_data()

    @property
    def counters(self):
        return self._device.counters

    @property
    def latches(self):
        return self._device.latches


class CT2Device(BaseCT2Device):
    """
    Helper for a locally installed CT2 card (P201/C208).
    """

    StdModes = [AcqMode.IntTrigReadout,
                AcqMode.SoftTrigReadout,
                AcqMode.IntTrigMulti]
    
    def __init__(self, config, name, acq_mode=AcqMode.IntTrigReadout):
        BaseCT2Device.__init__(self, config, name)
        self.__buffer = []
        self.__buffer_lock = lock.RLock()
        self.__card = self.config.get(self.name)
        self.__acq_mode = acq_mode
        self.__acq_status = AcqStatus.Ready
        self.__acq_expo_time = 1.0
        self.__acq_nb_points = 1
        self.__acq_channels = ()
        self.__timer_freq = 1E8
        self.__event_loop = None

    def run_acq_loop(self):
        card = self.__card

        while self.__acq_status == AcqStatus.Running:
            read, write, error = select.select((card,), (), (card,))
            try:
                if error:
                    self._send_error("ct2 select error on {0}".format(error))
                if read:
                    (counters, channels, dma, fifo_half_full, err), tstamp = \
                        card.acknowledge_interrupt()

                    if self.__acq_mode in self.StdModes:
                        if self.internal_point_nb_counter in counters:
                            self.__acq_status = AcqStatus.Ready
                            just_stopped = True

                    if err:
                        self._send_error("ct2 error")

                    if dma:
                        data, fifo_status = card.read_fifo()
                        with self.__buffer_lock:
                            self.__buffer.append(data)
                        point_nb = data[-1][-1]
                        self._send_point_nb(point_nb)

                    if just_stopped:
                        self._send_status(self.__acq_status)
            except Exception as e:
                sys.excepthook(*sys.exc_info())
                self._send_error("unexpected ct2 select error: {0}".format(e))

    @property
    def _device(self):
        return self

    @property
    def card(self):
        return self.__card

    def reset(self):
        self.card.software_reset()
        self.card.reset()
        self.__buffer = []

    def __configure_std_mode(self, mode):
        card = self.__card

        timer_ct = self.internal_timer_counter
        point_nb_ct = self.internal_point_nb_counter

        timer_inc_stop = getattr(ct2.CtClockSrc, 
                                 'INC_CT_{0}_STOP'.format(timer_ct))
        timer_start_source = getattr(ct2.CtHardStartSrc, 
                                     'CT_{0}_START'.format(timer_ct))
        if mode in [AcqMode.IntTrigReadout, AcqMode.IntTrigMulti]:
            timer_stop_source = getattr(ct2.CtHardStopSrc, 
                                        'CT_{0}_EQ_CMP_{0}'.format(timer_ct))
        else:
            timer_stop_source = ct2.CtHardStopSrc.SOFTWARE
        point_nb_stop_source = getattr(ct2.CtHardStopSrc, 
                                       'CT_{0}_EQ_CMP_{0}'.format(point_nb_ct))
        point_nb_gate = getattr(ct2.CtGateSrc, 
                                'CT_{0}_GATE_ENVELOP'.format(point_nb_ct))
        point_nb_start_source = getattr(ct2.CtHardStartSrc, 
                                        'CT_{0}_START'.format(point_nb_ct))

        stop_from_hard_stop = (mode == AcqMode.IntTrigMulti)
        
        # configure counter 11 as "timer"
        clock_source = self.IntClockSrc[self.timer_freq]
        ct_config = ct2.CtConfig(clock_source=clock_source,
                                 gate_source=point_nb_gate,
                                 hard_start_source=point_nb_start_source,
                                 hard_stop_source=timer_stop_source,
                                 reset_from_hard_soft_stop=True,
                                 stop_from_hard_stop=stop_from_hard_stop)
        card.set_counter_config(timer_ct, ct_config)

        if mode == AcqMode.IntTrigReadout:
            timer_val = int(self.acq_expo_time * self.timer_freq)
            card.set_counter_comparator_value(timer_ct, timer_val)

        # configure counter 12 as "nb. points"

        ct_config = ct2.CtConfig(clock_source=timer_inc_stop,
                                 gate_source=ct2.CtGateSrc.GATE_CMPT,
                                 hard_start_source=ct2.CtHardStartSrc.SOFTWARE,
                                 hard_stop_source=point_nb_stop_source,
                                 reset_from_hard_soft_stop=True,
                                 stop_from_hard_stop=True)
        card.set_counter_config(point_nb_ct, ct_config)
        card.set_counter_comparator_value(point_nb_ct, self.acq_nb_points)

        # first, be sure interrupts are anabled
        interrupt_buffer_size = 0
        card.enable_interrupts(interrupt_buffer_size)

        # dma transfer and error will trigger DMA; also counter 12 stop
        # should trigger an interrupt (this way we know that the
        # acquisition has finished without having to query the
        # counter 12 status)
        card.set_interrupts(counters=(point_nb_ct,), dma=True, error=True)

        # make master enabled by software
        card.enable_counters_software((timer_ct, point_nb_ct))

        # ... and now for the slave channels

        channels = tuple(self.acq_channels)
        all_channels = channels + (timer_ct, point_nb_ct)

        # change the start and stop sources for the active channels
        for ch_nb in channels:
            ct_config = card.get_counter_config(ch_nb)
            ct_config.gate_source = point_nb_gate
            ct_config.hard_start_source = timer_start_source
            ct_config.hard_stop_source = timer_stop_source
            ct_config.reset_from_hard_soft_stop = True
            ct_config.stop_from_hard_stop = stop_from_hard_stop
            card.set_counter_config(ch_nb, ct_config)

        # counter 11 will latch all active counters/channels
        latch_sources = dict([(ct, timer_ct) for ct in all_channels])
        card.set_counters_latch_sources(latch_sources)

        # counter 11 counter-to-latch signal will trigger DMA; at each DMA
        # trigger, all active counters (including counters 11 (timer)
        # and 12 (point_nb)) are stored to FIFO
        card.set_DMA_enable_trigger_latch((timer_ct,), all_channels)
        card.enable_counters_software(channels)

    def apply_config(self):
        ct2.configure_card(self.card, self.card_config)

    def prepare_acq(self):
        self.stop_acq()
        if self.acq_mode in self.StdModes:
            self.__configure_std_mode(self.acq_mode)
        else:
            raise NotImplementedError

    def start_acq(self):
        self.__acq_status = AcqStatus.Running
        try:
            self.__event_loop = gevent.spawn(self.run_acq_loop)

            if self.acq_mode in self.StdModes:
                counters = (self.internal_point_nb_counter,)
                self.card.start_counters_software(counters)
            else:
                raise NotImplementedError
        except:
            self.__acq_status = AcqStatus.Ready
            raise
        self._send_status(self.__acq_status)

    def stop_acq(self):
        if self.__acq_status != AcqStatus.Running:
            if self.__event_loop is not None:
                gevent.wait([self.__event_loop])
            self.__event_loop = None
            return

        self.__acq_status = AcqStatus.Ready
        if self.acq_mode in self.StdModes:
            self.card.stop_counters_software(self.card.COUNTERS)
        else:
            raise NotImplementedError
        gevent.wait([self.__event_loop])
        self.__event_loop = None
        self._send_status(self.__acq_status)

    def trigger_latch(self, counters):
        self.card.trigger_counters_software_latch(counters)

    def trigger_point(self):
        if self.acq_mode == AcqMode.SoftTrigReadout:
            self.card.stop_counters_software((self.internal_timer_counter,))
        elif self.acq_mode == AcqMode.IntTrigMulti:
            counters_status = self.card.get_counters_status()
            if counters_status[self.internal_timer_counter]['run']:
                raise RuntimeError('Counter still running')
            self.card.start_counters_software((self.internal_timer_counter,))
        else:
            raise NotImplementedError

    @property
    def acq_mode(self):
        return self.__acq_mode

    @acq_mode.setter
    def acq_mode(self, acq_mode):
        self.__acq_mode = AcqMode(acq_mode)

    @property
    def acq_status(self):
        return self.__acq_status

    @property
    def acq_nb_points(self):
        return self.__acq_nb_points

    @acq_nb_points.setter
    def acq_nb_points(self, acq_nb_points):
        self.__acq_nb_points = acq_nb_points

    @property
    def acq_expo_time(self):
        return self.__acq_expo_time

    @acq_expo_time.setter
    def acq_expo_time(self, acq_expo_time):
        self.__acq_expo_time = acq_expo_time

    @property
    def acq_channels(self):
        return self.__acq_channels

    @acq_channels.setter
    def acq_channels(self, acq_channels):
        self.__acq_channels = acq_channels

    @property
    def timer_freq(self):
        return self.__timer_freq

    @timer_freq.setter
    def timer_freq(self, timer_freq):
        if timer_freq not in self.IntClockSrc:
            raise ValueError('Invalid timer clock: %s' % timer_freq)
        self.__timer_freq = timer_freq

    @property
    def counters(self):
        return self.card.get_counters_values()

    @property
    def latches(self):
        return self.card.get_latches_values()

    def read_data(self):
        with self.__buffer_lock:
            return self.__read_data()

    def __read_data(self):
        b = self.__buffer
        if b:
            self.__buffer = []
            data = numpy.vstack(b)
        else:
            data = numpy.array([[]], dtype=numpy.uint32)
        return data