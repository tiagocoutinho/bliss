from bliss.common.continuous_scan import AcquisitionDevice, AcquisitionMaster
from bliss.common.event import dispatcher
import gevent
import time

class LimaAcquisitionDevice(AcquisitionDevice):
  def __init__(self, device, acq_nb_frames=1, acq_expo_time=1, acq_trigger_mode='INTERNAL_TRIGGER', acq_mode="SINGLE", acc_time_mode="LIVE", acc_max_expo_time=1, latency_time=0):
      self.parameters = locals().copy()
      del self.parameters['self']
      del self.parameters['device']
      trigger_type = AcquisitionDevice.SOFTWARE if 'INTERNAL' in acq_trigger_mode else AcquisitionDevice.HARDWARE
      AcquisitionDevice.__init__(self, device, device.user_detector_name, "lima",
                                 trigger_type = trigger_type)
     
              
  def prepare(self):
      for param_name, param_value in self.parameters.iteritems():
          setattr(self.device, param_name, param_value)
      self.device.prepareAcq()

  def start(self):
      if self._trigger_type == AcquisitionDevice.SOFTWARE:
          return
      self.trigger()

  #def trigger_ready(self):
  #    return self.device.ready_for_next_image

  def trigger(self):
      self.device.startAcq()

  def reading(self):
      while self.device.acq_status.lower() == 'running':
          dispatcher.send("new_ref", self, { "type":"lima/image", "last_image_acquired":self.device.last_image_acquired })
          gevent.sleep(self.parameters['acq_expo_time']/2.0)
      # TODO: self.dm.send_new_ref(self, {...}) ? or DataManager.send_new_ref(...) ?
      print "end of read_data", self.device.acq_status.lower()
      dispatcher.send("new_ref", self, { "type":"lima/image", "last_image_acquired":self.device.last_image_acquired })
