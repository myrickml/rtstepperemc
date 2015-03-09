#
# m3.py - m3 mcode script turns spindle on clockwise.
# 
# inputs:
#  p_num = rpm   # S parameter (spindle speed)
#  q_num = n/a
#
# example:
#  m3 s250.0
#
import pyemc
import arduino

def run(dongle, p_num, q_num):
   #print dongle.get_version()

   if (not arduino.connected):
      return

   if (not arduino.inited):
      arduino.init()

   # Init data direction register for output.
   arduino.call_response("DOUT,0\r")

   # Set shield pin high.
   arduino.call_response("DSET,0\r")

if __name__ == "__main__":
   dog = pyemc.EmcMech()
   run(dog, 0, 0)

