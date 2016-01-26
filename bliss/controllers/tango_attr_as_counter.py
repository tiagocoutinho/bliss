from bliss.common.task_utils import cleanup, error_cleanup, task
from bliss.common.measurement import CounterBase, AverageMeasurement
import PyTango.gevent
import time

class tango_attr_as_counter(CounterBase):
    def __init__(self, name, config):
        CounterBase.__init__(self, name)
        tango_uri = config.get("uri")
        self.__control = PyTango.gevent.DeviceProxy(tango_uri)
        self.attribute = config.get("attr_name")

    def read(self, acq_time=None):
        meas = AverageMeasurement()
        for reading in meas(acq_time):
            reading.value = self.__control.read_attribute(self.attribute).value
        return meas.average