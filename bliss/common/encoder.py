
from bliss.common import log as elog
from bliss.common.task_utils import *
from bliss.common import event
import time
import gevent
import re
import types


class Encoder(object):

    def __init__(self, name, controller, config):
        self.__name = name
        self.__controller = controller
        from bliss.config.motors import StaticConfig
        self.__config = StaticConfig(config)

    @property
    def name(self):
        return self.__name

    @property
    def controller(self):
        return self.__controller

    @property
    def config(self):
        return self.__config

    @property
    def steps_per_unit(self):
        return self.config.get("steps_per_unit", float, 1)

    @property
    def tolerance(self):
        return self.config.get("tolerance", float, 0)

    def read(self):
        return self.controller.read_encoder(self)/float(self.steps_per_unit)

    def set(self, new_value):
        self.controller.set_encoder(self, new_value*self.steps_per_unit)
        return self.read()
