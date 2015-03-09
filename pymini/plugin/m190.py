#
# m190.py - m190 mcode script home specified axis using limit switch.
# 
#     WARNING!!! do NOT run this script without limits switches
#
# inputs:
#  p_num = din   # digital input number (0 = INPUT0, 1 = INPUT1, 2 = INPUT2)
#  q_num = axis  # axis number (0 = x, 1 = y, 2 = z)
#
# example:
#  m190 p0 q0
#
import pyemc
from pyemc import MechStateBit
try:
   import configparser   # python3
except ImportError:
   import ConfigParser as configparser  # python2

ini_file = "rtstepper.ini"

din_map = ["INPUT0_ABORT", "INPUT1_ABORT", "INPUT2_ABORT"]
axis_section = ["AXIS_0", "AXIS_1", "AXIS_2"]
axis_letter = ["X", "Y", "Z"]
din_bit = [MechStateBit.INPUT0, MechStateBit.INPUT1, MechStateBit.INPUT2]

def run(dongle, p_num, q_num):

   din = int(p_num)
   axis = int(q_num)

   if (din < 0 or din > len(din_map)):
      return  # bail

   if (axis < 0 or axis > len(axis_section)):
      return  # bail

   cfg = configparser.ConfigParser()
   dataset = cfg.read(ini_file)
   if (len(dataset) == 0):
      return # bail, unable to load configuration file

   # Safety check, see if INPUTx will trigger estop in .ini file.
   key = din_map[din]
   val = cfg.get("TASK", key)
   if (val=="0"):
      return  # bail

   # Get soft limit for this axis.
   dist = cfg.getfloat(axis_section[axis], "MIN_LIMIT")

   # Get rapid feed rate.
   speed = cfg.getfloat(axis_section[axis], "MAX_VELOCITY")

   # Convert units/second to units/minute. */
   speed = speed * 60

   # Move to the soft limit, assumes limit switch is before the soft limit.
   dongle.mdi_cmd("G0 %s%f F%f" % (axis_letter[axis], dist, speed))
   dongle.wait_io_done()

   # Make sure we hit estop.
   if (not(dongle.get_state() & MechStateBit.ESTOP)):
      return  # bail

   # Make sure INPUTx bit is set (1).
   if (not(dongle.get_state() & din_bit[din])):
      return  # bail

   # Disable INPUTx estop trigger.
   dongle.disable_din_abort(din)

   # Reset estop.
   dongle.estop_reset()

   # Zero the axis.
   dongle.home();

   # Move to "home" position.
   dist = cfg.getfloat(axis_section[axis], "HOME")
   dongle.mdi_cmd("G0 %s%f F%f" % (axis_letter[axis], dist, speed))
   dongle.wait_io_done()

   # Zero the axis.
   dongle.home();

   # Enable INPUTx estop trigger.
   dongle.enable_din_abort(din)

if __name__ == "__main__":
   dog = pyemc.EmcMech()
   run(dog, 0, 0)
