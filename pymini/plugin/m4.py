#
# m4.py - m4 mcode script turns spindle on counter clockwise.
# 
# inputs:
#  p_num = rpm   # S parameter (spindle speed)
#  q_num = n/a
#
# example:
#  m4 s250.0
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

   # Set shield pin high.
   arduino.call_response("DSET,0\r")

