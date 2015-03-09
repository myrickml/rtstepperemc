#!/usr/bin/python
# pymini.py - A python GUI that interfaces with rtstepperemc library.
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
# History:
#
# 12/12/2014 - New

import logging
import logging.handlers
import threading
import sys, os, getopt
try:
   import queue
   import tkinter
   import tkinter.filedialog as filedialog
   import configparser
except ImportError:
   import Queue as queue
   import Tkinter as tkinter
   import tkFileDialog as filedialog
   import ConfigParser as configparser
import time
import pyemc
import backplot
from pyemc import MechStateBit
from version import Version

axis_name = ["na", "X", "Y", "Z", "A"]  # AxisSel to motor_name map
pos_name = ["na", "x", "y", "z", "a"]  # AxisSel to GetPosition_Eng() map

# Note, for windows HOME = "C:\Documents and Settings\user_name"
HOME_DIR = "%s/.%s" % (os.path.expanduser("~"), Version.name)

class IniFile(object):
   name = "rtstepper.ini"

class Panel(object):
   MAX_LINE = 6

class AxisSel(object):
   """Enumeration of buttons"""
   X = 1
   Y = 2
   Z = 3
   A = 4

class JogTypeSel(object):
   """Enumeration of buttons"""
   INC = 1
   ABS = 2

class JogSel(object):
   """Enumeration of buttons"""
   NEG = 1
   POS = 2

class AutoSel(object):
   OPEN = 1
   RUN = 2

class GuiEvent(object):
   """Events from mech to gui."""
   MECH_IDLE = 1
   LOG_MSG = 2
   MECH_DEFAULT = 3
   MECH_POSITION = 4
   MECH_ESTOP = 5  # auto estop from mech
   MECH_PAUSED = 6  # M0, M1 or M60

class MechEvent(object):
   """Events from gui to mech."""
   CMD_RUN = 1
   CMD_MDI = 2
   CMD_ALL_ZERO = 3
   CMD_ESTOP_RESET = 4

def usage():
   print("%s %s %s, GUI for rtstepperemc library" % (Version.name, Version.release, Version.date))
   print("(c) 2013-2015 Copyright Eckler Software")
   print("David Suffield, dsuffiel@ecklersoft.com")
   print("usage: %s [-i your_file.ini] (default=rtstepper.ini)" % (Version.name))

class LogPanelHandler(logging.Handler):
   def __init__(self, guiq):
      logging.Handler.__init__(self)
      self.guiq = guiq

   def emit(self, record):
      """ Queue up a logger message for the gui. """
      r = self.format(record)
      e = {}
      e['id'] = GuiEvent.LOG_MSG
      e['msg'] = self.format(record)
      self.guiq.put(e)

# =======================================================================
class Mech(object):
   def __init__(self, cfg, guiq, mechq, dog):
      self.tid = self.mech_thread(cfg, guiq, mechq, dog, TK_INTERVAL=1.0)
      self.tid.start()

   class mech_thread(threading.Thread):
      def __init__(self, cfg, guiq, mechq, dog, TK_INTERVAL):
         threading.Thread.__init__(self)
         self.done = threading.Event()
         self.cfg = cfg
         self.guiq = guiq
         self.mechq = mechq
         self.dog = dog
         self.TK_INTERVAL = TK_INTERVAL  # mech_thread() sleep interval in seconds

      def shutdown(self):
         self.done.set()  # stop thread()

      # Main thread() function.
      def run(self):
         if (not(self.dog.get_state() & MechStateBit.ESTOP)):
            self.post_idle()

         while (1):
            # Check for thread shutdown.
            if (self.done.isSet()):
               break

            # Sleep for interval or until shutdown.
            self.done.wait(self.TK_INTERVAL)

            # Check queue for any commands.
            while (not self.mechq.empty()):
               e = self.mechq.get()
               if (e['id'] == MechEvent.CMD_MDI):
                  self.cmd_mdi(e['cmd'])
               elif (e['id'] == MechEvent.CMD_RUN):
                  self.cmd_auto(e['file'])
               elif (e['id'] == MechEvent.CMD_ALL_ZERO):
                  self.cmd_all_zero()
               elif (e['id'] == MechEvent.CMD_ESTOP_RESET):
                  self.cmd_estop_reset()
               else:
                  pass
               e = None

      def post_idle(self):
         """ Let gui know we are idle. """
         m = {}
         m['id'] = GuiEvent.MECH_IDLE
         self.guiq.put(m)

      def cmd_mdi(self, buf):
         for ln in buf.splitlines():
            if (self.done.isSet()):
               break
            try:
               self.dog.mdi_cmd(ln)
            except:
               logging.error("Unable to process MDI command: %s" % (ln))
         self.dog.wait_io_done()
         if (not(self.dog.get_state() & MechStateBit.ESTOP)):
            self.post_idle()

      def cmd_auto(self, gcodefile):
         try:
            # Execute gcode file.
            self.dog.auto_cmd(gcodefile)
         except:
            logging.error("Unable to process gcode file: %s" % (gcodefile))
         self.dog.wait_io_done()
         if (not(self.dog.get_state() & MechStateBit.ESTOP or self.dog.get_state() & MechStateBit.PAUSED)):
            self.post_idle()

      def cmd_all_zero(self):
         self.dog.home()
         if (not(self.dog.get_state() & MechStateBit.ESTOP)):
            self.post_idle()

      def cmd_estop_reset(self):
         result = self.dog.estop_reset()
         if (not(self.dog.get_state() & MechStateBit.ESTOP)):
            self.post_idle()

   def close(self):
      logging.info("Mech thread closing...")
      self.tid.shutdown()
      self.tid.join()
      self.tid = None

#=======================================================================
class Gui(tkinter.Tk):
   def __init__(self, parent=None):
      tkinter.Tk.__init__(self, parent)
      self.guiq = queue.Queue()  # Mech to tkinter communication
      self.mechq = queue.Queue()  # tkinter to Mech communication
      self.sel_axis = AxisSel.X  # set axis default
      self.sel_jog_type = JogTypeSel.INC  # set jog type default
      self.green_led = tkinter.PhotoImage(file='green_led.gif')
      self.red_led = tkinter.PhotoImage(file='red_led.gif')
      self.orange_led = tkinter.PhotoImage(file='orange_led.gif')
      self._update()  # kickoff _update()
      self.create_widgets()
      self.auto_setup()
      self.bp3d = backplot.BackPlot(self.backplot_canvas)

      # Instantiate dongle.
      self.dog = pyemc.EmcMech()

      # Setup dll callbacks
      self.dog.register_logger_cb()
      self.dog.register_event_cb(self.guiq)

      # Open dll.
      logging.info("Opening %s %s" % (self.dog.LIBRARY_FILE, self.dog.get_version()))
      self.dog.open(IniFile.name)

      # Kickoff mech thread().
      self.mech = Mech(self.cfg, self.guiq, self.mechq, self.dog)

   #=======================================================================
   def _update(self):
      """ Check guiq for events. """
      self.proc()
      self.after(200, self._update)

   #=======================================================================
   def proc(self):
      """ Check queue for any io initiated events. """
      while (not self.guiq.empty()):
         e = self.guiq.get()
         if (e['id'] == GuiEvent.MECH_IDLE):
            self.set_idle_state(tkinter.ACTIVE)
         elif (e['id'] == GuiEvent.LOG_MSG):
            self.display_logger_message(e)
         elif (e['id'] == GuiEvent.MECH_POSITION):
            self.update_position(e)
         elif (e['id'] == GuiEvent.MECH_ESTOP):
            self.set_estop_state()  # auto estop from mech
         elif (e['id'] == GuiEvent.MECH_PAUSED):
            self.set_idle_state(tkinter.DISABLED, resume=tkinter.ACTIVE)
         else:
            logging.info("unable to process gui event %d\n" % (e['id']))
         e = None

   #=======================================================================
   def get_ini(self, section, option, default=None):
      try:
         val = self.cfg.get(section, option)
      except:
         val = default
      return val

   #=======================================================================
   def create_widgets(self):
      self.grid()
      grow=0
      self.x_val = tkinter.Button(self, relief=tkinter.SUNKEN, command=lambda: self.axis_button(AxisSel.X))
      self.x_val.grid(row=grow, column=0, sticky='w')

      grow += 1
      self.y_val = tkinter.Button(self, relief=tkinter.FLAT, command=lambda: self.axis_button(AxisSel.Y))
      self.y_val.grid(row=grow, column=0, sticky='w')

      grow += 1
      self.z_val = tkinter.Button(self, relief=tkinter.FLAT, command=lambda: self.axis_button(AxisSel.Z))
      self.z_val.grid(row=grow, column=0, sticky='w')

      grow += 1
      self.a_val = tkinter.Button(self, relief=tkinter.FLAT, command=lambda: self.axis_button(AxisSel.A))
      self.a_val.grid(row=grow, column=0, sticky='w')

      grow += 1
      self.estop_button = tkinter.Button(self, text="EStop", command=self.toggle_estop)
      self.estop_button.grid(row=grow, column=3, sticky='news')

      self.home_button = tkinter.Button(self, text="All Zero", command=self.set_all_zero)
      self.home_button.grid(row=grow, column=0, sticky='news')

      self.jogneg_button = tkinter.Button(self, text="Jog X -", command=lambda: self.jog(JogSel.NEG))
      self.jogneg_button.grid(row=grow, column=1, sticky='news')

      self.jogpos_button = tkinter.Button(self, text="Jog X +", command=lambda: self.jog(JogSel.POS))
      self.jogpos_button.grid(row=grow, column=2, sticky='news')

      self.resume_button = tkinter.Button(self, text="Resume", command=self.resume)
      self.resume_button.grid(row=grow, column=4, sticky='news')

      grow = 1
      self.status_button = tkinter.Label(self, text="status")
      self.status_button.grid(row=grow, column=3, sticky='e')
      self.led_button = tkinter.Label(self, image=self.green_led)
      self.led_button.grid(row=grow, column=4, sticky='w')

      panelrow = 17
      lastrow = panelrow
      self.log_panel = tkinter.Text(self, state=tkinter.DISABLED, width=80, height=Panel.MAX_LINE, wrap=tkinter.NONE,
                                    bg=self.x_val['bg'], relief=tkinter.SUNKEN, borderwidth=4)
      self.log_panel.grid(row=lastrow, columnspan=5, sticky='news')

      lastrow += 1
      self.status_line_num = tkinter.Label(self)
      self.status_line_num.grid(row=lastrow, column=4, sticky='w')

      # Pipe log messages to Text widget.
      self.create_logger(self.guiq)

      # Load persistent data from .ini for MDI buttons.
      self.open(IniFile.name)

      grow = 0
      self.units = self.get_ini("TRAJ", "LINEAR_UNITS", default="inch")
      self.speed_button = tkinter.Label(self, text="Speed (%s/minute)" % (self.units))
      self.speed_button.grid(row=grow, column=1, sticky='e')
      self.speed_val = tkinter.StringVar()
      self.speed_val.set(self.get_ini("DISPLAY", "JOG_SPEED", default="6"))
      self.speed_entry = tkinter.Entry(self, textvariable=self.speed_val)  # use max z speed for default
      self.speed_entry.grid(row=grow, column=2, sticky='w')

      grow += 1
      self.inc_button = tkinter.Button(self, text="Incremental Jog (%s)" % (self.units), relief=tkinter.SUNKEN,
                                       command=lambda: self.jog_type_button(JogTypeSel.INC))
      self.inc_button.grid(row=grow, column=1, sticky='e')
      self.inc_val = tkinter.StringVar()
      self.inc_val.set(self.get_ini("DISPLAY", "INC_JOG", default="0.2"))
      self.inc_entry = tkinter.Entry(self, textvariable=self.inc_val)
      self.inc_entry.grid(row=grow, column=2, sticky='w')

      grow += 1
      self.abs_button1 = tkinter.Button(self, text="Absolute Jog (%s)" % (self.units), relief=tkinter.FLAT,
                                        command=lambda: self.jog_type_button(JogTypeSel.ABS))
      self.abs_button1.grid(row=grow, column=1, sticky='e')
      self.abs_val = tkinter.StringVar()
      self.abs_val.set(self.get_ini("DISPLAY", "ABS_JOG", default="1"))
      self.abs_entry = tkinter.Entry(self, textvariable=self.abs_val)
      self.abs_entry.grid(row=grow, column=2, sticky='w')

      grow = 5
      self.mdi_button1 = tkinter.Button(self, text=self.get_ini("DISPLAY", "MDI_LABEL_1", default="MDI-1"),
                                        command=lambda: self.mdi(self.mdi_val1))
      self.mdi_button1.grid(row=grow, column=0, sticky='news')
      self.mdi_val1 = tkinter.StringVar()
      self.mdi_val1.set(self.get_ini("DISPLAY", "MDI_CMD_1", default=""))
      self.mdi_entry1 = tkinter.Entry(self, textvariable=self.mdi_val1)
      self.mdi_entry1.grid(row=grow, column=1, columnspan=4, sticky='ew')

      grow += 1
      self.mdi_button2 = tkinter.Button(self, text=self.get_ini("DISPLAY", "MDI_LABEL_2", default="MDI-2"),
                                        command=lambda: self.mdi(self.mdi_val2))
      self.mdi_button2.grid(row=grow, column=0, sticky='news')
      self.mdi_val2 = tkinter.StringVar()
      self.mdi_val2.set(self.get_ini("DISPLAY", "MDI_CMD_2", default=""))
      self.mdi_entry2 = tkinter.Entry(self, textvariable=self.mdi_val2)
      self.mdi_entry2.grid(row=grow, column=1, columnspan=4, sticky='ew')

      grow += 1
      self.mdi_button3 = tkinter.Button(self, text=self.get_ini("DISPLAY", "MDI_LABEL_3", default="MDI-3"),
                                        command=lambda: self.mdi(self.mdi_val3))
      self.mdi_button3.grid(row=grow, column=0, sticky='news')
      self.mdi_val3 = tkinter.StringVar()
      self.mdi_val3.set(self.get_ini("DISPLAY", "MDI_CMD_3", default=""))
      self.mdi_entry3 = tkinter.Entry(self, textvariable=self.mdi_val3)
      self.mdi_entry3.grid(row=grow, column=1, columnspan=4, sticky='ew')

      grow += 1
      self.mdi_button4 = tkinter.Button(self, text=self.get_ini("DISPLAY", "MDI_LABEL_4", default="MDI-4"),
                                        command=lambda: self.mdi(self.mdi_val4))
      self.mdi_button4.grid(row=grow, column=0, sticky='news')
      self.mdi_val4 = tkinter.StringVar()
      self.mdi_val4.set(self.get_ini("DISPLAY", "MDI_CMD_4", default=""))
      self.mdi_entry4 = tkinter.Entry(self, textvariable=self.mdi_val4)
      self.mdi_entry4.grid(row=grow, column=1, columnspan=4, sticky='ew')

      grow += 1
      self.mdi_button5 = tkinter.Button(self, text=self.get_ini("DISPLAY", "MDI_LABEL_5", default="MDI-5"),
                                        command=lambda: self.mdi(self.mdi_val5))
      self.mdi_button5.grid(row=grow, column=0, sticky='news')
      self.mdi_val5 = tkinter.StringVar()
      self.mdi_val5.set(self.get_ini("DISPLAY", "MDI_CMD_5", default=""))
      self.mdi_entry5 = tkinter.Entry(self, textvariable=self.mdi_val5)
      self.mdi_entry5.grid(row=grow, column=1, columnspan=4, sticky='ew')

      grow += 1
      self.auto_button = tkinter.Button(self, text="Auto", command=lambda: self.auto(AutoSel.OPEN))
      self.auto_button.grid(row=grow, column=0, sticky='news')
      self.auto_val = tkinter.StringVar()
      self.auto_val.set(self.get_ini("DISPLAY", "AUTO_FILE", default="your_file.nc"))
      self.auto_entry = tkinter.Entry(self, textvariable=self.auto_val)
      self.auto_entry.grid(row=grow, column=1, columnspan=3, sticky='ew')
      self.run_button = tkinter.Button(self, text="Run", command=lambda: self.auto(AutoSel.RUN))
      self.run_button.grid(row=grow, column=4, sticky='news')

      grow += 1
      self.plot_3d_button = tkinter.Button(self, text="3D", command=lambda: self.bp3d.plot_3d())
      self.plot_3d_button.grid(row=grow, column=0, sticky='news')
      self.backplot_canvas = tkinter.Canvas(self, relief=tkinter.SUNKEN, borderwidth=4)
      self.backplot_canvas.grid(row=grow, rowspan=6, column=1, columnspan=4, sticky='ew')
      grow += 1
      self.plot_xy_button = tkinter.Button(self, text="X - Y", command=lambda: self.bp3d.plot_xy())
      self.plot_xy_button.grid(row=grow, column=0, sticky='news')
      grow += 1
      self.plot_xz_button = tkinter.Button(self, text="X - Z", command=lambda: self.bp3d.plot_xz())
      self.plot_xz_button.grid(row=grow, column=0, sticky='news')
      grow += 1
      self.plot_yz_button = tkinter.Button(self, text="Y - Z", command=lambda: self.bp3d.plot_yz())
      self.plot_yz_button.grid(row=grow, column=0, sticky='news')
      grow += 1
      self.zoom_in_button = tkinter.Button(self, text="Zoom In", command=lambda: self.bp3d.zoom_in())
      self.zoom_in_button.grid(row=grow, column=0, sticky='news')
      grow += 1
      self.zoom_out_button = tkinter.Button(self, text="Zoom Out", command=lambda: self.bp3d.zoom_out())
      self.zoom_out_button.grid(row=grow, column=0, sticky='news')

      #grow += 1

      self.columnconfigure(1, weight=1)
      self.columnconfigure(2, weight=1)
      self.rowconfigure(panelrow, weight=1)

   #=======================================================================
   def axis_button(self, id):
      if (id == AxisSel.X):
         self.x_val.config(relief=tkinter.SUNKEN)
         self.y_val.config(relief=tkinter.FLAT)
         self.z_val.config(relief=tkinter.FLAT)
         self.a_val.config(relief=tkinter.FLAT)
         self.jogneg_button.config(text="Jog X -")
         self.jogpos_button.config(text="Jog X +")
      elif (id == AxisSel.Y):
         self.x_val.config(relief=tkinter.FLAT)
         self.y_val.config(relief=tkinter.SUNKEN)
         self.z_val.config(relief=tkinter.FLAT)
         self.a_val.config(relief=tkinter.FLAT)
         self.jogneg_button.config(text="Jog Y -")
         self.jogpos_button.config(text="Jog Y +")
      elif (id == AxisSel.Z):
         self.x_val.config(relief=tkinter.FLAT)
         self.y_val.config(relief=tkinter.FLAT)
         self.z_val.config(relief=tkinter.SUNKEN)
         self.a_val.config(relief=tkinter.FLAT)
         self.jogneg_button.config(text="Jog Z -")
         self.jogpos_button.config(text="Jog Z +")
      else:
         self.x_val.config(relief=tkinter.FLAT)
         self.y_val.config(relief=tkinter.FLAT)
         self.z_val.config(relief=tkinter.FLAT)
         self.a_val.config(relief=tkinter.SUNKEN)
         self.jogneg_button.config(text="Jog A -")
         self.jogpos_button.config(text="Jog A +")
      self.sel_axis = id

   #=======================================================================
   def jog_type_button(self, id):
      if (id == JogTypeSel.INC):
         self.inc_button.config(relief=tkinter.SUNKEN)
         self.abs_button1.config(relief=tkinter.FLAT)
      else:
         self.inc_button.config(relief=tkinter.FLAT)
         self.abs_button1.config(relief=tkinter.SUNKEN)
      self.sel_jog_type = id

   #=======================================================================
   def update_position(self, e):
      self.cur_pos = e['pos']
      self.x_val.config(text="X  %07.3f" % (self.cur_pos['x']))
      self.y_val.config(text="Y  %07.3f" % (self.cur_pos['y']))
      self.z_val.config(text="Z  %07.3f" % (self.cur_pos['z']))
      self.a_val.config(text="A  %07.3f" % (self.cur_pos['a']))

      if (e['line'] > 0):
         # Display gcode lines that have completed plus the next line.
         beg = self.line_num
         end = e['line']
         if (beg > 1):
            beg += 2
         if (end < self.line_max):
            end += 1

         self.line_num = e['line']  # save completed line

         # Display gcode lines in log_panel.
         line_txt = self.display_gcode_lines(self.auto_val.get(), beg, end, self.line_num)

         # Display completed line in backplot.
         self.bp3d.update_plot(line_txt, self.line_num, self.cur_pos)

         # Update status line count.
         self.status_line_num.config(text="%d/%d" % (self.line_num, self.line_max))

   #=======================================================================
   def display_logger_message(self, e):
      numlines = self.log_panel.index('end - 1 line').split('.')[0]
      self.log_panel['state'] = 'normal'
      if (numlines == Panel.MAX_LINE):
         self.log_panel.delete(1.0, 2.0)
      if (self.log_panel.index('end-1c') != '1.0'):
         self.log_panel.insert('end', '\n')
         self.log_panel.see('end')
      self.log_panel.insert('end', e['msg'])
      self.log_panel['state'] = 'disabled'

   #=======================================================================
   def set_idle_state(self, flag, resume=tkinter.DISABLED, estop=tkinter.ACTIVE):
      self.estop_button.config(state=estop)
      self.home_button.config(state=flag)
      self.jogneg_button.config(state=flag)
      self.jogpos_button.config(state=flag)
      self.run_button.config(state=flag)
      self.mdi_button1.config(state=flag)
      self.mdi_button2.config(state=flag)
      self.mdi_button3.config(state=flag)
      self.mdi_button4.config(state=flag)
      self.mdi_button5.config(state=flag)
      self.resume_button.config(state=resume)

      if (self.dog.get_state() & MechStateBit.ESTOP):
         self.led_button.config(image=self.red_led)
      elif (self.dog.get_state() & MechStateBit.PAUSED):
         self.led_button.config(image=self.orange_led)
      else:
         self.led_button.config(image=self.green_led)

      if (self.dog.get_state() & MechStateBit.HOMED):
         self.x_val.config(fg="blue")
         self.y_val.config(fg="blue")
         self.z_val.config(fg="blue")
         self.a_val.config(fg="blue")
      else:
         self.x_val.config(fg="red")
         self.y_val.config(fg="red")
         self.z_val.config(fg="red")
         self.a_val.config(fg="red")

   #=======================================================================
   def jog(self, id):
      if (not self.safety_check_ok()):
         return

      self.set_idle_state(tkinter.DISABLED)
      m = {}
      m['id'] = MechEvent.CMD_MDI

      if (id == JogSel.NEG):
         # Jog negative.
         if (self.sel_jog_type == JogTypeSel.INC):
            # perform incremental move
            val = float(self.inc_val.get())
            m['cmd'] = "G1 %s%f F%s" % (
               axis_name[self.sel_axis], self.cur_pos[pos_name[self.sel_axis]] - val, self.speed_val.get())
         else:
            # perform absolute move
            val = float(self.abs_val.get())
            m['cmd'] = "G1 %s%f F%s" % (axis_name[self.sel_axis], val, self.speed_val.get())
      else:
         # Jog positive.
         if (self.sel_jog_type == JogTypeSel.INC):
            # perform incremental move
            val = float(self.inc_val.get())
            m['cmd'] = "G1 %s%f F%s" % (
               axis_name[self.sel_axis], self.cur_pos[pos_name[self.sel_axis]] + val, self.speed_val.get())
         else:
            # perform absolute move
            val = float(self.abs_val.get())
            m['cmd'] = "G1 %s%f F%s" % (axis_name[self.sel_axis], val, self.speed_val.get())

      self.mechq.put(m)

   #=======================================================================
   def mdi(self, id):
      if (not self.safety_check_ok()):
         return

      self.set_idle_state(tkinter.DISABLED)
      m = {}
      m['id'] = MechEvent.CMD_MDI
      m['cmd'] = id.get()
      self.mechq.put(m)

   #=======================================================================
   def set_all_zero(self):
      self.set_idle_state(tkinter.DISABLED)
      m = {}
      m['id'] = MechEvent.CMD_ALL_ZERO
      self.mechq.put(m)

   #=======================================================================
   def toggle_estop(self):
      self.set_idle_state(tkinter.DISABLED, estop=tkinter.DISABLED)    # disable (block) estop button during a estop
      if (self.dog.get_state() & MechStateBit.ESTOP):
         m = {}
         m['id'] = MechEvent.CMD_ESTOP_RESET
         self.mechq.put(m)        # this command goes through the mech thread
      else:
         self.dog.estop()  # this command goes through the GUI thread.

   #=======================================================================
   def set_estop_state(self):
      if (self.dog.get_state() & MechStateBit.ESTOP):
         self.set_idle_state(tkinter.DISABLED)
      else:
         self.set_idle_state(tkinter.ACTIVE)

   #=======================================================================
   def safety_check_ok(self):
      if (self.dog.get_state() & MechStateBit.ESTOP):
         logging.error("Not out of ESTOP. Try pressing the ESTOP button.")
         return False

      if (not(self.dog.get_state() & MechStateBit.HOMED)):
         logging.warn("Not all zeroed. Try pressing the 'All Zero' button.")
         return False

      return True  # ok to run mech

   #=======================================================================
   def resume(self):
      self.set_idle_state(tkinter.DISABLED)
      m = {}
      m['id'] = MechEvent.CMD_RUN
      m['file'] = "paused"
      self.mechq.put(m)

   #=======================================================================
   def display_gcode_lines(self, gcodefile, beg, end, done):
      # Attempt to open the file.
      done_line = ""
      try:
         gfile = open(gcodefile,'r')
         # Count number of lines.
         cnt = 0
         line = gfile.readline()
         while (line != ''):
            cnt += 1
            if (cnt >= beg):
               logging.info("%d: %s" % (cnt, line.rstrip()))
            if (cnt == done):
               done_line = line.rstrip()  # save this line for backplot
            if (cnt == end):
               break
            line = gfile.readline()
         gfile.close()
      except:
          pass
      return done_line

   #=======================================================================
   def display_max_line(self, gcodefile):
      # Attempt to open the file.
      try:
         gfile = open(gcodefile,'r')
         # Count number of lines.
         self.line_max = 0
         while (gfile.readline() != ''):
             self.line_max += 1
         self.line_num = 1
         self.status_line_num.config(text="%d/%d" % (self.line_num, self.line_max))
         gfile.close()
      except:
          pass

   #=======================================================================
   def auto(self, id):
      if (id == AutoSel.OPEN):
         gcodefile = str(filedialog.askopenfilename(parent=self, title="Open Gcode File"))
         if (gcodefile != ''):
            self.auto_val.set(gcodefile)
            self.display_max_line(gcodefile)
      else:
         gcodefile = self.auto_val.get()
         if (gcodefile == ''):
            return
         if (not self.safety_check_ok()):
            return

         self.display_max_line(gcodefile)
         self.set_idle_state(tkinter.DISABLED)
         m = {}
         m['id'] = MechEvent.CMD_RUN
         m['file'] = gcodefile
         self.mechq.put(m)
         self.bp3d.clear_plot()

   #=======================================================================
   def create_logger(self, panel):
      if (not os.path.exists(HOME_DIR)):
         # Create application '.' directory in user's home directory.
         os.makedirs(HOME_DIR)
      root = logging.getLogger()
      root.setLevel(level=logging.DEBUG)
      h = logging.handlers.TimedRotatingFileHandler("%s/%s" % (HOME_DIR, "log.txt"), "D", 1, backupCount=5)
      f = logging.Formatter('%(asctime)s:%(levelname)s:%(filename)s:%(lineno)s:%(message)s')
      h.setFormatter(f)
      root.addHandler(h)
      self.log_h = LogPanelHandler(panel)
      root.addHandler(self.log_h)
      logging.info("pymini %s %s" % (Version.release, Version.date))

   #=======================================================================
   def open(self, filename='rtstepper.ini'):
      logging.info("Loading configuration file: %s", filename)
      self.cfg = configparser.ConfigParser()
      dataset = self.cfg.read(filename)
      if (len(dataset) == 0):
         logging.error("Unable to load configuration file: %s" % (filename))
         raise Exception("Unable to load configuration file: %s" % (filename))

   #=======================================================================
   def close(self):
      self.dog.estop()
      root = logging.getLogger()
      root.removeHandler(self.log_h)
      self.mech.close()
      self.dog.close()

   #=======================================================================
   def auto_setup(self):
      """perform any initial application setup"""
      table = "%s/%s" % (HOME_DIR, self.get_ini("EMC", "TOOL_TABLE", default="stepper.tbl"))
      if (not os.path.exists(table)):
         # Create a default tool table with zero offsets.
         with open(table, 'w') as f:
            f.write("\n")
            for i in range(1, 11):
               f.write("%d   %d   0.0   0.0   empty\n" % (i, i))
            f.write("\n")
         f.closed

#==========================================================================
try:
   opt, arg = getopt.getopt(sys.argv[1:], "i:h")
except:
   pass
for cmd, param in opt:
   if (cmd in ("-h", "--help")):
      usage()
      sys.exit()
   if (cmd in ("-i")):
      IniFile.name = param

app = Gui()

app.title("%s v%s" % (app.cfg.get("EMC", "MACHINE"), Version.release))
app.mainloop()
app.close()







