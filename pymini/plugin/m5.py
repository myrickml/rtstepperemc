#
# m5.py - m5 mcode script turns spindle off.
# 
# inputs:
#  p_num = n/a
#  q_num = n/a
#
# example:
#  m5
#
import pyemc
import arduino

def run(dongle, p_num, q_num):

   if (not arduino.connected):
      return

   if (not arduino.inited):
      arduino.init()

   # Init data direction register for output.
   arduino.call_response("DOUT,0\r")

   # Set shield pin low.
   arduino.call_response("DCLR,0\r")

if __name__ == "__main__":
   dog = pyemc.EmcMech()
   run(dog, 0, 0)

