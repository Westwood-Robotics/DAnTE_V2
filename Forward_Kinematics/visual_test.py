from Settings.Robot import *
import Forward_Kinematics.forward_kin as FK
import math as m
DAnTE.palm.angle = m.pi/3
INDEX.angles = [0, m.pi/2, m.pi/6, m.pi/6]
INDEX_M.angles = [0, m.pi/2, m.pi/6, m.pi/6]
THUMB.angles = [0, m.pi/2, m.pi/6, m.pi/3]
FK.visual(DAnTE)

