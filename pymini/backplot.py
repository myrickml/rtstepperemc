#!/usr/bin/python
# backplot.py - gcode ploting module. Used by pymini.py.
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

import logging
import math
from collections import OrderedDict

class BackPlot(object):

   def __init__(self, bp):
      self.bp = bp
      self.list = OrderedDict()
      self.color = ["lime green", "black", "red", "blue", "yellow3"]

      self.x = 0.0
      self.y = 0.0
      self.z = 0.0
      self.a = 0.0
      self.b = 0.0
      self.c = 0.0

      self.x_center = 0.0
      self.y_center = 0.0

      self.x_last = 0.0
      self.y_last = 0.0
      self.x_next = 0.0
      self.y_next = 0.0

      # Active canvas width/height.
      self.width = int(self.bp.cget("width"))
      self.height = int(self.bp.cget("height"))

      # Set axis directions.  +1 one plots like graph paper with positive up and to the right.
      # To reverse an axis change its value to -1
      self.xdir = 1
      self.ydir = -1
      self.zdir = -1

      # Set the default size of the plot and can be thought of as number of
      # pixels per inch of machine motion.  The default (40) will give about .55 screen
      # inch per inch moved.
      self.mdpi = 40

      self.scaler = 1

      # Bind canvas resize event to resize().
      self.bp.bind('<Configure>', self.resize)
      # Bind mouse button scrolling events.
      self.bp.bind("<ButtonPress-1>", self.scroll_start)
      self.bp.bind("<B1-Motion>", self.scroll_move)

      # Default to 3d view.
      self.plot_3d()

   def resize(self, event):
      # Calculate the size change.
      dx = abs(event.width - self.width)
      dy = abs(event.height - self.height)

      # Set current canvas size.
      self.width = event.width
      self.height = event.height

      # Ignore small spurious size changes.
      if (dx > 10 or dy > 10):
         self.center_plot()

   def scroll_start(self, event):
      self.bp.scan_mark(event.x, event.y)

   def scroll_move(self, event):
      self.bp.scan_dragto(event.x, event.y, gain=1)

   def vector(self):
      # 3D vector conversion
      # X Y and Z point is converted into polar notation
      # then rotated about the A B and C axis.
      # Finally to be converted back into rectangular co-ords.

      # Rotate about A - X axis
      angle = self.a * 0.01745329
      if (self.y != 0 or self.z != 0):
         angle = math.atan2(self.y, self.z) + angle
      vector = math.hypot(self.y, self.z)
      self.z = vector * math.cos(angle)
      self.y = vector * math.sin(angle)
      
      # Rotate about B - Y axis
      angle = self.b * 0.01745329
      if (self.x != 0 or self.z != 0):
         angle = math.atan2(self.z, self.x) + angle
      vector = math.hypot(self.x, self.z)
      self.x = vector * math.cos(angle)
      self.z = vector * math.sin(angle)

      # Rotate about C - Z axis
      angle = self.c * 0.01745329
      if (self.x != 0 or self.y != 0):
         angle = math.atan2(self.y, self.x) + angle
      vector = math.hypot(self.x, self.y)
      self.x = vector * math.cos(angle)
      self.y = vector * math.sin(angle)

   def plot_xy(self):
      self.x_rotate = -90
      self.y_rotate = 0.0
      self.z_rotate = 0.0
      self.redraw()

   def plot_xz(self):
      self.x_rotate = 0.0
      self.y_rotate = 0.0
      self.z_rotate = 0.0
      self.redraw()

   def plot_yz(self):
      self.x_rotate = 0.0
      self.y_rotate = 0.0
      self.z_rotate = 90
      self.redraw()

   def plot_3d(self):
      self.x_rotate = -27
      self.y_rotate = 17
      self.z_rotate = 30
      self.redraw()

   def zoom_out(self):
      self.scaler *= 2.0
      self.redraw()

   def zoom_in(self):
      self.scaler *= 0.5
      self.redraw()

   def clear_plot(self):
      self.list.clear()
      self.x_last = 0.0
      self.y_last = 0.0
      self.redraw()

   def redraw(self):
      self.bp.delete("all")
      self.center_plot()

      if (len(self.list) >= 2):
         # Loop through all the positions.
         i = 0
         for k, pos in self.list.items():
            if (i == 0):
               # Set the first position.
               self.x = pos['x'] * self.mdpi * self.xdir / self.scaler
               self.y = pos['y'] * self.mdpi * self.ydir / self.scaler
               self.z = pos['z'] * self.mdpi * self.zdir / self.scaler
               self.a = self.x_rotate
               self.b = self.y_rotate
               self.c = self.z_rotate
               self.vector()
               self.x_last = self.x
               self.y_last = self.z
               i += 1
               continue

            self.x = pos['x'] * self.mdpi * self.xdir / self.scaler
            self.y = pos['y'] * self.mdpi * self.ydir / self.scaler
            self.z = pos['z'] * self.mdpi * self.zdir / self.scaler
            self.vector()
            self.x_next = self.x
            self.y_next = self.z
            self.bp.create_line(self.x_last, self.y_last, self.x_next, self.y_next, fill=self.color[pos['gcode']])
            self.x_last = self.x
            self.y_last = self.z

      # Draw red arrow tick mark.
      self.tick = self.bp.create_line(self.x_last, self.y_last, self.x_last+5, self.y_last+5, fill="red", arrow="first", tags="tick_mark")

   def center_plot(self):
      # Calculate visual window and scroll window extents.
      vx1 = -self.width / 2
      vx2 = self.width / 2
      sx1 = -self.width * 1.9 / 2
      sx2 = self.width * 1.9 / 2
      vy1 = -self.height / 2
      vy2 = self.height / 2
      sy1 = -self.height * 1.9 / 2
      sy2 = self.height * 1.9 / 2

      # Center origin (0,0) in middle of scroll window.
      self.bp.config(scrollregion=(sx1, sy1, sx2, sy2))

      # Draw crosshatch.
      self.bp.create_line(sx1, 0, sx2, 0, fill="dark gray")
      self.bp.create_line(0, sy1, 0, sy2, fill="dark gray")

      # Move visual window to center.
      self.bp.xview_moveto((vx1 - sx1) / (sx2 - sx1))
      self.bp.yview_moveto((vy1 - sy1) / (sy2 - sy1))

   def update_plot(self, line, line_num, pos):

      # Parse gcode line to determine canvas line color.
      ln = line.lower()
      if "g03" in ln:
         gcode = 3
      elif "g3" in ln:
         gcode = 3
      elif "g02" in ln:
         gcode = 2
      elif "g2" in ln:
         gcode = 2
      elif "g01" in ln:
         gcode = 1
      elif "g1" in ln:
         gcode = 1
      elif "g0" in ln:
         gcode = 0
      else:
         gcode = 1

      self.x = pos['x'] * self.mdpi * self.xdir / self.scaler
      self.y = pos['y'] * self.mdpi * self.ydir / self.scaler
      self.z = pos['z'] * self.mdpi * self.zdir / self.scaler

      self.a = self.x_rotate
      self.b = self.y_rotate
      self.c = self.z_rotate

      self.vector()
      self.x_next = self.x
      self.y_next = self.z
      item = self.bp.create_line(self.x_last, self.y_last, self.x_next, self.y_next, fill=self.color[gcode])
      self.list[item] = {'x':pos['x'], 'y':pos['y'], 'z':pos['z'], 'gcode':gcode}

      # Move the red arrow tick mark to the new position.
      self.bp.move(self.tick, self.x_next - self.x_last, self.y_next - self.y_last)

      self.x_last = self.x
      self.y_last = self.z


################################################################################################################
if __name__ == "__main__":
   # Configure logging for the application.
   logging.basicConfig(filename='log.txt',
                       level=logging.DEBUG,
                       format='%(asctime)s:%(levelname)s:%(filename)s:%(lineno)s:%(message)s')

   b = BackPlot()
   b.vector()
