import unittest
import sys
import os
import time

sys.path.insert(
    0,
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..")))

import bliss
from bliss.common.axis import READY,MOVING
import bliss.common.log
bliss.common.log.level(bliss.log.DEBUG)

config_xml = """
<config>
  <controller class="GalilDMC213">
    <host value="192.168.0.2"/>
    <axis name="omega">
      <channel value="A"/>
      <steps_per_unit value="-12800"/>
      <velocity value="1664000"/>
    </axis>
  </controller>
</config>
"""


class TestGalilDMC213(unittest.TestCase):

    def setUp(self):
        bliss.load_cfg_fromstring(config_xml)
   
    """ 
    def testCommunication(self):
        o = bliss.get_axis("omega")
        self.assertEquals(o.controller._galil_query("MG 1+3"), "4.0000")
        self.assertRaises(RuntimeError, o.controller._galil_query, "BLA")
        self.assertTrue(o.controller._galil_query(chr(18)+chr(22)).startswith("DMC2112"))
    
    def testStatus(self):
        pass
   
    def testHomeSearch(self):
        o = bliss.get_axis("omega")
        o.home()

    """ 
    def testPosition(self):
        o = bliss.get_axis("omega")
        p = o.position()
        o.rmove(10)
        self.assertAlmostEquals(o.position(), p+10, places=3)
    
if __name__ == '__main__':
    unittest.main()
