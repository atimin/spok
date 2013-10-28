import numpy as np
from math import cos, sin, pi, asin, acos, atan, sqrt

#TODO: remove duplication
PHETA = 0
D     = 1
A     = 2
ALPHA = 3

class Man:

  def __init__(self, config):
    self.config = config
    self.init_config = config
    self.update_config()

  def update_config(self):
    self._A = [ np.matrix([ 
      [cos(dh[PHETA]), -1*cos(dh[ALPHA])*sin(dh[PHETA]), sin(dh[ALPHA])*sin(dh[PHETA]),      dh[A]*cos(dh[PHETA])],
      [sin(dh[PHETA]), cos(dh[ALPHA])*cos(dh[PHETA]),    -1*sin(dh[ALPHA])*cos(dh[PHETA]),   dh[A]*sin(dh[PHETA])],
      [0,               sin(dh[ALPHA]),                    cos(dh[ALPHA]),                      dh[D]],
      [0,               0,                                  0,                                    1]])

      for dh in self.config ]

  def turn_joint(self, num, delta):
    self.config[num][PHETA] += delta
    self.update_config()

  def reset(self):
    self.config = self.init_config
    self.update_config()

  def get_coord_joint(self, num):
    coord = np.matrix(np.identity(4))
    for i in xrange(num + 1):
      coord = coord * self._A[i]

    return coord * np.matrix("0;0;0;1")

  def set_tool(self, x,y,z):
    raise Exeption("Method isn't implemented")


