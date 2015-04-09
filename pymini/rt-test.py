#!/usr/bin/python
# rt-test.py - A command line test script. Tests rt-stepper dongle USB IO.
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

import sys, getopt
import pyemc
import version

def usage():
   print("rt-test %s, rt-stepper dongle USB IO test" % (version.Version.release))
   print("(c) 2013-2015 Copyright Eckler Software")
   print("David Suffield, dsuffiel@ecklersoft.com")
   print("usage: rt-test [-s serial_number]")
   print("WARNING!!!! Do *NOT* run with DB25 connected to any cnc controller!")

snum = ""
try:
   opt, arg = getopt.getopt(sys.argv[1:], "s:h")
   for cmd, param in opt:
      if (cmd in ("-h", "--help")):
         usage()
         sys.exit()
      if (cmd in ("-s")):
         snum = param
except:
   usage()
   sys.exit()

# Instantiate dongle.
dog = pyemc.EmcMech()

# Display dll version.
print("Opened %s %s" % (dog.LIBRARY_FILE, dog.get_version()))

# Perform usb io test.
dog.test(snum)
