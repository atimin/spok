from man import Man
from math import cos, sin, pi, asin, acos, atan, sqrt

#TODO: remove duplication
PHETA = 0
D     = 1
A     = 2
ALPHA = 3

class Arm(Man):
  def __init__(self, M):
    self._M = M

    Man.__init__(self, [
      [pi/2, 76.0, 0.0, -pi/2],
      [-pi/2, 0.0, 100.0, 0.0],
      [pi/2, 0.0, 100.0, 0.0],
      [0.0, -40.0, 50.0, -pi/2],
      [0.0, M, 32.0, 0.0]])
      # [0.0, 45.0 + M, 32.0, 0.0]])

  def set_tool(self, x, y, z):
    ncfg = self.config[:]
    # Calculate coordinates of last point j3
    # TODO: It's not right!!!! You don't use absolute coordinats for compute offsets!!!
    # o
    x, y, z = float(x), float(y), float(z)
    if x > 0.0:
        alpha = atan(y/x)
    elif x < 0.0:
        alpha = pi + atan(y/x)
    else:
        alpha = pi/2
    CB = abs(ncfg[3][D])
    AB = abs(ncfg[3][A] + ncfg[4][A])
    OC = sqrt(x**2 + y**2)
    OB = sqrt(OC**2 - CB**2)
    OA = OB - AB
    gamma = asin(CB/OC)

    xm = OA*cos(alpha + gamma)
    ym = OA*sin(alpha + gamma)
    zm = z + ncfg[4][D] - ncfg[0][D]

    print("Found coordinats of joint #3 :" + str((xm, ym, zm)))

    if xm > 0.0:
        K = atan(ym/xm)
    elif xm < 0.0:
        K = pi + atan(ym/xm)
    else:
        K = pi/2

    Lp = sqrt(xm**2 + ym**2)                    
    L = sqrt(zm**2 + Lp**2)                 


    print "L = " + str(L)
    print "K = " + str(K)
    
    PHETA_1 = asin(zm / L)
    PHETA_2 = acos((ncfg[1][A]**2 + L**2 - ncfg[2][A]**2) / (2*ncfg[1][A]*L))

    print "PHETA_1 = " + str(PHETA_1)
    print "PHETA_2 = " + str(PHETA_2)

    ncfg[0][PHETA] = K
    ncfg[1][PHETA] = - (PHETA_1 + PHETA_2)
    ncfg[2][PHETA] = pi - acos((ncfg[1][A]**2 + ncfg[2][A]**2 - L**2) / (2*ncfg[1][A]*ncfg[2][A]))
    ncfg[3][PHETA] = PHETA_1 -  ncfg[2][PHETA] + PHETA_2 
    self.config = ncfg
    self.update_config()
    self.angles = [ self.config[0][PHETA],
            pi + self.config[1][PHETA],
            pi - self.config[2][PHETA],
            self.config[3][PHETA] + pi/2,
        ]

    return self.angles

    
