#
# m7.py - m7 mcode script turns coolant mist on.
# 
# inputs:
#  p_num = n/a
#  q_num = n/a
#
# example:
#  m7
#
import pyemc
import arduino

def run(dongle, p_num, q_num):

   if (not arduino.connected):
      return

   if (not arduino.inited):
      arduino.init()

   # Init data direction register for output.
   arduino.call_response("DOUT,1\r")

   # Set shield pin high.
   arduino.call_response("DSET,1\r")
