import time
from PyTango.gevent import DeviceProxy
from bliss.common.task_utils import *
from bliss.comm.Exporter import *
import itertools
import logging

def grouped(iterable, n):
    "s -> (s0,s1,s2,...sn-1), (sn,sn+1,sn+2,...s2n-1), (s2n,s2n+1,s2n+2,...s3n-1), ..."
    return itertools.izip(*[iter(iterable)]*n)

class MD2S:
    def __init__(self, name, config):
        self.phases = {"Centring":1, "BeamLocation":2, "DataCollection":3, "Transfer":4}
        self.timeout = 3 #s by default
        nn, port = config.get("exporter_addr").split(":")
        self._exporter = Exporter(nn, int(port))
        self._beamviewer_server = config.get("beamviewer_server")
        self._sample_video_server = config.get("sample_video_server")
        self.thgt = config.get("thgt")
        self.ttrans = config.get("ttrans")
        self.tyb = config.get("tyb")
        self.fshut= config.get("fshut")
        self.transmission = config.get("transmission")
        self.detcover = config.get("detcover")

    def get_hwstate(self):
        return self._exporter.readProperty("HardwareState")

    def get_swstate(self):
        return self._exporter.readProperty("State")

    def _ready(self):
        if self.get_hwstate() == "Ready" and self.get_swstate() == "Ready":
            return True
        return False

    def _wait_ready(self, timeout=None):
        if timeout <= 0:
            timeout = self.timeout
        tt1 = time.time()
        while time.time() - tt1 < timeout:
             if self._ready():
                 break
             else:
                 time.sleep(0.5)

    def get_phase(self):
         return self._exporter.readProperty("CurrentPhase")

    def set_phase(self, phase, wait=False, timeout=40):
        if self.phases.has_key(phase):
            self._exporter.execute("startSetPhase", phase)
            if wait:
                self._wait_ready(timeout)

    def get_cameracalibration(self):
        #the value depends on the zoom
        px_mm_y = 1000000.0*self._exporter.readProperty("CoaxCamScaleX")
        px_mm_z = 1000000.0*self._exporter.readProperty("CoaxCamScaleY")
        return [px_mm_y, px_mm_z]

    @task
    def _simultaneous_move(self, *args):
        axis_list = [] 
        for axis, target in grouped(args, 2):
            axis_list.append(axis)
            axis.move(target, wait=False)
        return [axis.wait_move() for axis in axis_list]


    @task
    def _simultaneous_rmove(self, *args):
        axis_list = [] 
        for axis, target in grouped(args, 2):
            axis_list.append(axis)
            axis.rmove(target, wait=False)
        return [axis.wait_move() for axis in axis_list]

    def centrebeam(self):
        #prepare to see the beam
        self.set_phase("BeamLocation", wait=True, timeout=100)
        self._exporter.writeProperty("CapillaryPosition", "OFF")
        self._wait_ready(20)
        app = self._exporter.readProperty("AperturePosition")
        self._exporter.writeProperty("AperturePosition", "OFF")
        self._wait_ready(20)

        #set the camera to read one image
        self.bv_device = DeviceProxy(self._beamviewer_server)
        self.sample_video_device = DeviceProxy(self._sample_video_server)

        def restore_live():
            self.sample_video_device.video_live=True

        #get the image size and camera calibration
        img_width = self.sample_video_device.image_width
        img_height = self.sample_video_device.image_height
        px_mm_y = 1000000.0*self._exporter.readProperty("CoaxCamScaleX")
        px_mm_z =  1000000.0*self._exporter.readProperty("CoaxCamScaleY")
        #px_mm_y, px_mm_z = self.get_cameracalibration()

        def restore_table(saved_pos=(self.thgt, self.thgt.position(), self.ttrans,self.ttrans.position(), self.tyb, self.tyb.position())):
            logging.getLogger().info("Restoring table:", saved_pos)
            self._simultaneous_move(*saved_pos)

        def restore_att(old_transmission=self.transmission.transmission_get()):
            self.transmission.transmission_set(old_transmission)

        def do_centrebeam():
            with error_cleanup(restore_att):
                self.fshut.open()
                #if self.fshut.state() is not OPENED:
                #    return
                
            with cleanup(restore_live):
                self.sample_video_device.video_live=False
                res = self.bv_device.GetPosition()
      
            by = res[2]
            bz = res[3]
            if -1 in (by, bz):
                raise RuntimeError("Could not find beam")

            dy = (by - (img_width / 2)) / px_mm_y
            dz = (bz - (img_height / 2)) / px_mm_z
            if abs(dy) > 0.1 or abs(dz) > 0.1:
                raise RuntimeError("Aborting centrebeam, too big displacement")
            with error_cleanup(restore_table):
                print "moving ttrans by", -dy
                print "moving thgt by", -dz
                #self._simultaneous_rmove(self.thgt, -dz)
                self._simultaneous_rmove(self.thgt, -dz, self.ttrans, -dy, self.tyb, -dy)
            return dy, dz

        with cleanup(restore_att):
            self.transmission.transmission_set(3)
            self.detcover.set_in()
 
            for i in range(5):
                dy, dz = do_centrebeam()
                if abs(dy) < 0.001 and abs(dz) < 0.001:
                    break
            self.fshut.close()

