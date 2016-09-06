# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

from bliss.common.task_utils import *
import gevent
import gevent.event
import math
from bliss.common import log

"""
Classes implemented with temperature Controller
"""

class Input(object):
    """ Implements the access to temperature sensors
    """
    def __init__(self, controller, config):
        """ Constructor """
        log.debug("On Input")
        #log.debug("  config type is: %s" % type(config))
        #log.debug("  controller type is: %s" % type(controller))
        self.__controller = controller
        self.__name = config["name"]
        self.__config = config

        # lists of custom attr and commands
        self.__custom_methods_list = list()
        self.__custom_attributes_dict = dict()

        # useful attribute for a temperature controller writer
        self._attr_dict = {}

    @property
    def controller(self):
        """ Returns the temperature controller """
        return self.__controller

    @property
    def name(self):
        """ returns the sensor name """
        return self.__name

    @property
    def config(self):
        """ returns the snsor config """
        return self.__config

    def read(self):
        """ returns the sensor value """
        log.debug("On Input:read")
        return self.controller.read_input(self)

    def state(self):
        """ returns the sensor state """
        log.debug("On Input:state")
        return self.controller.state_input(self)

    def _add_custom_method(self, method, name, types_info=(None, None)):
        """ necessary to add custom methods to a class """
        setattr(self, name, method)
        self.__custom_methods_list.append((name, types_info))

class Output(object):
    """ Implements the access to temperature heaters """
    def __init__(self, controller, config):
        """ Constructor """
        log.debug("On Output")
        self.__controller = controller
        self.__name = config["name"]
        try: 
            self.__limits = (config.get("low_limit"), config.get("high_limit"))
        except:
            self.__limits = (None,None)
        self.__setpoint_task = None
        self.__setpoint_event = gevent.event.Event()
        self.__setpoint_event_poll = 0.02
        try:
            self.__deadband = float(config["deadband"])
        except:
            self.__deadband = None
        self.__setpoint_event.set()
        self.__config = config
        self.__stopped = 0
        self.__mode = 0
        # if defined as  self.deadband, attribute available from the instance
        # if defined as  self.__deadband, not available.
        #     in that case, use of decorator property offers it (read only) to world


        # lists of custom attr and commands
        self.__custom_methods_list = list()
        self.__custom_attributes_dict = dict()

        # useful attribute for a temperature controller writer
        self._attr_dict = {}

    @property
    def controller(self):
        """ returns the temperature controller """
        return self.__controller

    @property
    def name(self):
        """ returns the heater name """
        return self.__name

    @property
    def config(self):
        """ returns the heater config """
        return self.__config

    @property
    def limits(self):
        """ returns the limits for the heater temperature setting """
        return self.__limits

    @property
    def deadband(self):
        """ returns the deadband acceptable for the heater temperature setting. 
            After a ramp or a set, the setpoint is considered to be reached
            only if heater value is within the deadband.
            While the setpoint is not reached, a wait will block on it."""
        return self.__deadband

    def read(self):
        """ returns the heater value """
        log.debug("On Output:read")
        return self.controller.read_output(self)

    def ramp(self, new_setpoint=None, wait=False, **kwargs):
        """ Starts a ramp on an output.
            - if no setpoint is provided, returns the present setpoint value
            - by default does not wait.
            - it is possible to provide kwargs arguments
            
            The related controller methods to be filled are:
            - get_setpoint
            - setpoint_stop
            - setpoint_abort
            - start_ramp
        """
        log.debug( "On Output:ramp %s" % new_setpoint)
        self.__mode = 1
        return self._ramp(new_setpoint, wait, **kwargs)

    def set(self, new_setpoint=None, wait=False, **kwargs):
        """ Sets as quickly as possible a temperature.
            - if no setpoint is provided, returns the present setpoint value
            - by default does not wait.
            - it is possible to provide kwargs arguments
            
            The related controller methods to be filled are:
            - get_setpoint
            - setpoint_stop
            - setpoint_abort
            - set
        """
        log.debug( "On Output:set %s" % new_setpoint)
        self.__mode = 0
        return self._ramp(new_setpoint, wait, **kwargs)

    def _ramp(self, new_setpoint=None, wait=False, **kwargs):
        """ starts the ramp tasks.
        """
        log.debug( "On Output:_ramp %s" % new_setpoint)
        if new_setpoint is not None:
            ll, hl = self.limits
            if ll is not None and new_setpoint < ll:
                raise RuntimeError("Invalid setpoint `%f', below low limit (%f)" % (new_setpoint, ll))
            if hl is not None and new_setpoint > hl:
                raise RuntimeError("Invalid setpoint `%f', above high limit (%f)" % (new_setpoint, hl))

            self.__setpoint_task = self._start_setpoint(new_setpoint,**kwargs)
            self.__setpoint_task.link(self.__setpoint_done)

            if wait:
                self.wait()
        else:
            return self.controller.get_setpoint(self)

    def wait(self):
        """ Waits on a setpoint task
        """
        log.debug("On Output:wait")
        print "On Output:wait"
	try:
            self.__setpoint_event.wait()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """ Stops a setpoint task.
            Calls the controller method setpoint_stop
        """
        print "On Output: stop"
        if self.__setpoint_task and not self.__setpoint_task.ready():
            self.__setpoint_task.kill()
            #added lines
            self.__setpoint_event.set()
            self.__stopped = 1
            ##
        self.controller.setpoint_stop(self)

    def abort(self):
        """ Aborts a setpoint task.
            Calls the controller method setpoint_abort
        """
        print "On Output: abort"
        if self.__setpoint_task and not self.__setpoint_task.ready():
            self.__setpoint_task.kill()
            #added lines
            self.__setpoint_event.set()
            self.__stopped = 1
            ##
        self.controller.setpoint_abort(self)

    def rampstate(self):
        """
        Returns the setpoint state:

        - RUNNING: means that output object read value
          is outside setpoint deadband
        - READY: inside the deadband
        """
        return self.controller._setpoint_state(self, self.__deadband)
        

    def __setpoint_done(self, task):
        """ stop the setpoint tasks
        """
        log.debug("On Output:__setpoint_done")
        print "On Output:__setpoint_done"
        try:
            try:
                task.get()
            except Exception:
                sys.excepthook(*sys.exc_info())
	finally:
            if self.__stopped == 0:
               self.__setpoint_event.set()
            self.__stopped = 0


    @task
    def _do_setpoint(self, setpoint, **kwargs):
        """ Subtask launching the setpoint
            Polls until setpoint is reached
            Is a gevent coroutine
        """ 
        log.debug("On Output:_do_setpoint : mode = %s" % (self.__mode))
        if self.__mode == 1:
           self.controller.start_ramp(self, setpoint, **kwargs)
        else :
           self.controller.set(self, setpoint, **kwargs)

        while self.controller._setpoint_state(self, self.__deadband) == 'RUNNING':
            gevent.sleep(self.__setpoint_event_poll)

    def _start_setpoint(self, setpoint, **kwargs):
        """ launches the coroutine doing the setpoint
        """
        log.debug("On Output:_start_setpoint")
        print "On Output:_start_setpoint"
        self.__setpoint_event.clear()
        # the "task" decorator automatically turns a function into a gevent coroutine,
        # and adds a 'wait' keyword argument, whose value is True by default;
        # setting wait to False returns the coroutine object
        return self._do_setpoint(setpoint, wait=False, **kwargs)

    def state(self):
        """ returns the the state of a heater """
        log.debug("On Output:state")
        return self.controller.state_output(self)

    def pollramp(self, new_poll=None):
        """
        Setting/reading the polling time (in seconds) in the event loop waiting
        for setpoint being reached (== read value inside the deadband)
        default is 0.02 sec
        """
        if new_poll:
           self.__setpoint_event_poll = new_poll
        else:
           return self.__setpoint_event_poll

    def ramprate(self, new_ramp=None):
        """
        Setting/reading the setpoint ramp rate value 

        """
        log.debug("On Output:ramprate: %s " % (new_ramp))
        if new_ramp:
           self.controller.set_ramprate(self,new_ramp)
        else:
           return self.controller.read_ramprate(self)

    def step(self, new_step=None):
        """
        Setting/reading the setpoint step value (for step mode ramping)

        """
        log.debug("On Output:step: %s " % (new_step))
        if new_step:
           self.controller.set_step(self,new_step)
        else:
           return self.controller.read_step(self)

    def dwell(self, new_dwell=None):
        """
        Setting/reading the setpoint dwell value (for step mode ramping)

        """
        log.debug("On Output:setpoint dwell: %s " % (new_dwell))
        if new_dwell:
           self.controller.set_dwell(self,new_dwell)
        else:
           return self.controller.read_dwell(self)


    def _add_custom_method(self, method, name, types_info=(None, None)):
        """ necessary to add custom methods to this class """
        setattr(self, name, method)
        self.__custom_methods_list.append((name, types_info))


class Loop(object):
    """ Implements the access to temperature regulation loop """
    def __init__(self, controller, config):
        """ Constructor """
        log.debug("On Loop")
        self.__controller = controller
	self.__name = config["name"]
        self.__config = config
        self.__input  = controller.get_object(config["input"][1:])
        self.__output = controller.get_object(config["output"][1:])
        self._Pval = None
        self._Ival = None
        self._Dval = None

        # lists of custom attr and commands
        self.__custom_methods_list = list()
        self.__custom_attributes_dict = dict()

        # useful attribute for a temperature controller writer
        self._attr_dict = {}

    @property
    def controller(self):
        """ returns the temperature controller """
        return self.__controller

    @property
    def name(self):
        """ returns the loop name """
        return self.__name

    @property
    def config(self):
        """ returns the loop config """
        return self.__config

    @property
    def input(self):
        """ returns the loop input object """
        return self.__input

    @property
    def output(self):
        """ returns the loop output object """
        return self.__output

    def set(self, new_setpoint=None, wait=False,**kwargs):
        """ same as a call to the the method set on its output object """
        log.debug(("On Loop: set %s") % new_setpoint)
        return self.__output.set(new_setpoint, wait, **kwargs)

    def ramp(self, new_setpoint=None, wait=False,**kwargs):
        """ same as the call to the method ramp on its output object """
        log.debug(("On Loop: ramp %s") % new_setpoint)
        return self.__output.ramp(new_setpoint, wait, **kwargs)

    def stop(self):
        """ same as the call to the method stop on its output object """
        log.debug("On Loop: stop")
        self.__output.stop()

    def on(self):
        """ Sets the regulation on
            - call to the method 'on' of the controller
        """
        log.debug("On Loop: on")
        self.controller.on(self)

    def off(self):
        """ Sets the regulation off
            - call to the method 'off' of the controller
        """
        log.debug("On Loop: off")
        self.controller.off(self)


    def kp(self, new_kp=None):
        """
        Setting/reading the P value (for PID)

        """
        log.debug("On Loop: kp (PID): ")
        if new_kp:
           self.controller.set_kp(self,new_kp)
        else:
           return self.controller.read_kp(self)


    def ki(self, new_ki=None):
        """
        Setting/reading the I value (for PID)

        """
        log.debug("On Loop: ki (PID): ")
        if new_ki:
           self.controller.set_ki(self,new_ki)
        else:
           return self.controller.read_ki(self)

    def kd(self, new_kd=None):
        """
        Setting/reading the D value (for PID)

        """
        log.debug("On Loop: kd (PID): ")
        if new_kd:
           self.controller.set_kd(self,new_kd)
        else:
           return self.controller.read_kd(self)


    def _add_custom_method(self, method, name, types_info=(None, None)):
        """ necessary to add custom methods to this class """
        setattr(self, name, method)
        self.__custom_methods_list.append((name, types_info))



