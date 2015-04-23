#!/usr/bin/python
# arduino.py - Arduino Uno module. Used by the plugins for shared data.
#
# (c) 2014-2015 Copyright Eckler Software
#
# Author: David Suffield, dsuffiel@ecklersoft.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of version 2 of the GNU General Public License as published by
# the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
#
# Upstream patches are welcome. Any patches submitted to the author must be
# unencumbered (ie: no Copyright or License).
#
# Notes:
#
# Arduino Uno - original board works ok and enumerates with VID/PID and Serial Number. VID=2341 PID=0001
#
# DCcduino Uno - clone board works ok and enumerates with VID/PID, but no Serial Number. VID=1a86 PID=7523
#                Board enumerates as "QinHeng Electronics HL-340 USB-Serial adapter"
#
# For serial port permissions on Ubuntu the User must belong to 'dialout' group (adduser username dialout).
#

import time
try:
   import serial
   from serial.tools import list_ports
except ImportError:
   connected = False

connected = False      # Set to True if using Arduino Uno for IO.

#UNO_VID = "1a86"      # Set to USB Uno board vendor number
#UNO_PID = "7523"      # Set to USB Uno board product number
UNO_VID = "2341"       # Set to USB Uno board vendor number
UNO_PID = "0001"       # Set to USB Uno board product number

inited = False         # usb serial port opened: False = no, True = yes
uno = None             # serial port instance

# Perform serial port auto discovery based on Arduino Uno board USB PID/VID.
def get_device(vid, pid):
   l = list_ports.comports()
   for d, n, e in l:
      if (e.find("USB VID:PID") == 0):
         s = e.split(' ')
         s = s[1].split('=')
         s = s[1].split(':')
         if (vid == s[0] and pid == s[1]):
            return d   # found a match
   raise Exception

def init():
   global uno, inited, connected
   try:
      dev = get_device(UNO_VID, UNO_PID)
      uno = serial.Serial(dev, baudrate=9600, timeout=1.0)
   except Exception as err:
      print("unable to open Ardunio serial port: vid=%s pid=%s %s" % (UNO_VID, UNO_PID, err))
      connected = False
      return

   # The board is reset when first open. Wait for board to initialize.
   print("Ardunio is reseting...")
   time.sleep(2)

   # Establish contact and purge send/receive buffer.
   call = "VER\r"
   print("Ardunio establish contact: %s" % (call))
   uno.write(call)
   time.sleep(2)
   resp = uno.readline()
   resp = resp.strip('\r\n')
   print("Ardunio contact response: %s" % (resp))

   # Get call_response firmware version.
   call_response("VER\r")

   inited = True

def call_response(call):
   global uno
   print("Ardunio call: %s" % (call))
   uno.write(call)
   resp = uno.readline()
   resp = resp.strip('\r\n')
   print("Ardunio response: %s" % (resp))
   if(len(resp) <= 1):
      raise Exception
   return resp

if __name__ == "__main__":
   init()
