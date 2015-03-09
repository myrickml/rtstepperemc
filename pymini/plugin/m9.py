#
# m9.py - m9 mcode script turns coolant off.
# 
# inputs:
#  p_num = coolant  # 1 = mist, 2 = flood
#  q_num = n/a
#
# example:
#  m9
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
   arduino.call_response("DCLR,1\r")

