#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.1.0"
__status__ = "Beta"

# -----------------------------
# Simple code to ping all BEARs

from pybear import Manager

bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)
bear_list = []
found = False
for i in range(0, 9):
    m_id = i
    print("Pinging BEAR with ID %d" % m_id)
    data = bear.ping(m_id)
    if data:
        print("Found BEAR with ID %d." % m_id)
        print(data)
        found = True
        bear_list.append(m_id)
if found:
    print("Search done. Total of %d BEARs found. And their IDs are:\n" % len(bear_list))
    print(bear_list)
else:
    print("Search done. No BEAR found.")
