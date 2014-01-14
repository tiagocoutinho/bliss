from bliss.controllers.motor import Controller
from bliss.common.motor import READY, MOVING
from bliss.common.task_utils import task, error_cleanup, cleanup
import random
import math
import time

class Mockup(Controller):
  def __init__(self, name, config):
    Controller.__init__(self, name, config)

    # Access to the config.
    self.config.get_property("host")

    # Add a "channel" paramter to axes.
    # Check that <channel> is really an integer.
    self.axis_settings.add('channel', int)

    self.axis_settings.add('init_count', int)

    for axis_name, axis in self.axes.iteritems():
      self.axis_settings.set(axis, 'position', random.randint(0,360))
      self.axis_settings.set(axis, 'state', READY)
      self.axis_settings.set(axis, 'init_count', 0)

  def initialize_axis(self, axis):
    self.axis_settings.set(axis, 'init_count',self.axis_settings.get(axis, "init_count")+1)   

  @task
  def _move(self, axis, target_pos, delta):
    v = self.read_velocity(axis)
    d = math.copysign(1, delta)
    pos = self.read_position(axis)
    t0 = time.time()
    end_t = t0 + math.fabs(delta)/float(v)

    def move_cleanup():
      self.update_state(axis, READY)

    with cleanup(move_cleanup):
      self.update_state(axis, MOVING)
      while True:
        t = time.time()
        if t < end_t:
          dt = t - t0
          pos += d*dt*v 
          self.update_position(axis, pos)
          time.sleep(0.01) 
        else:
          break
      self.update_position(axis, target_pos)

  def read_position(self, axis, measured=False):
    return self.axis_settings.get(axis, "position")

  def read_velocity(self, axis):
    return self.axis_settings.get(axis, "velocity")

  def read_state(self, axis):
    return self.axis_settings.get(axis, "state")
