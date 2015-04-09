/*****************************************************************************\

  ui.c - user interface support for rtstepperemc

  (c) 2008-2015 Copyright Eckler Software

  Author: David Suffield, dsuffiel@ecklersoft.com

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as published by
  the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

  Upstream patches are welcome. Any patches submitted to the author must be 
  unencumbered (ie: no Copyright or License).

  See project revision history the "configure.ac" file.

\*****************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>
#include "emc.h"
#include "ini.h"
#include "bug.h"

/* Following GUI_EVENT must match python GuiEvent. */
enum GUI_EVENT
{
   GUI_EVENT_MECH_IDLE = 1,
   GUI_EVENT_LOG_MSG = 2,
   GUI_EVENT_MECH_DEFAULT = 3,
   GUI_EVENT_MECH_POSITION = 4,
   GUI_EVENT_MECH_ESTOP = 5,
   GUI_EVENT_MECH_PAUSED = 6,
};

struct emcpose_py
{
   double x;
   double y;
   double z;
   double a;
   double b;
   double c;
   double u;
   double v;
   double w;
};

struct post_position_py
{
   int id;
   struct emcpose_py pos;
};

const char *USER_HOME_DIR;

static logger_cb_t _logger_cb = NULL;
static post_event_cb_t _post_event_cb = NULL;
static post_position_cb_t _post_position_cb = NULL;
static plugin_cb_t _plugin_cb = NULL;

static pthread_mutex_t ui_mutex = PTHREAD_MUTEX_INITIALIZER;

struct emc_session session;

/* Axis:                                    1    2    3    4     5     6     7     8      9  */
static const int _map_axes_mask[] = { 0x0, 0x1, 0x3, 0x7, 0xf, 0x1f, 0x3f, 0x7f, 0xff, 0x1ff };

static enum EmcAxisType _map_axis_type(const char *type)
{
   if (strncasecmp(type, "linear", 6) == 0)
      return EMC_AXIS_LINEAR;
   if (strncasecmp(type, "angular", 7) == 0)
      return EMC_AXIS_ANGULAR;
   return EMC_AXIS_LINEAR;
}

static double _map_linear_units(const char *units)
{
   if (strncasecmp(units, "mm", 2) == 0)
      return 1.0;
   if (strncasecmp(units, "metric", 6) == 0)
      return 1.0;
   if (strncasecmp(units, "in", 2) == 0)
      return 1 / 25.4;
   if (strncasecmp(units, "inch", 4) == 0)
      return 1 / 25.4;
   if (strncasecmp(units, "imperial", 8) == 0)
      return 1 / 25.4;
   return 0;
}

static double _map_angular_units(const char *units)
{
   if (strncasecmp(units, "deg", 3) == 0)
      return 1.0;
   if (strncasecmp(units, "degree", 6) == 0)
      return 1.0;
   if (strncasecmp(units, "grad", 4) == 0)
      return 0.9;
   if (strncasecmp(units, "gon", 3) == 0)
      return 0.9;
   if (strncasecmp(units, "rad", 3) == 0)
      return M_PI / 180;
   if (strncasecmp(units, "radian", 6) == 0)
      return M_PI / 180;
   return 0;
}

static enum EMC_RESULT _load_tool_table(const char *filename, struct CANON_TOOL_TABLE toolTable[])
{
   FILE *fp;
   int i, toolno, pocket, scanned;
   double zoffset, diameter;
   char buf[CANON_TOOL_ENTRY_LEN];
   char comment[CANON_TOOL_ENTRY_LEN];
   char path[LINELEN];

   snprintf(path, sizeof(path), "%s/.%s/%s", USER_HOME_DIR, PACKAGE_NAME, filename);

   // open tool table file
   if (NULL == (fp = fopen(path, "r")))
   {
      MSG("Warning unable to open tool table %s\n", path);
      return EMC_R_ERROR;
   }

   // clear out tool table
   for (i = 1; i < CANON_POCKETS_MAX; i++)
   {
      toolTable[i].toolno = -1;
      ZERO_EMC_POSE(toolTable[i].offset);
      toolTable[i].diameter = 0.0;
      toolTable[i].frontangle = 0.0;
      toolTable[i].backangle = 0.0;
      toolTable[i].orientation = 0;
   }

   while (fgets(buf, sizeof(buf), fp) != NULL)
   {
      // for nonrandom machines, just read the tools into pockets 1..n
      // no matter their tool numbers.  NB leave the spindle pocket 0
      // unchanged/empty.

      if ((scanned = sscanf(buf, "%d %d %lf %lf %[^\n]", &toolno, &pocket, &zoffset, &diameter, comment)) && (scanned == 4 || scanned == 5))
      {
         if (pocket < 0 || pocket >= CANON_POCKETS_MAX)
         {
            BUG("invalid tool number %d, skipping...\n", toolno);
            continue;
         }
         else
         {
            /* mill tool */
            toolTable[pocket].toolno = toolno;
            toolTable[pocket].offset.tran.z = zoffset;
            toolTable[pocket].diameter = diameter;

            // these aren't used on a mill
            toolTable[pocket].frontangle = toolTable[pocket].backangle = 0.0;
            toolTable[pocket].offset.tran.x = 0.0;
            toolTable[pocket].orientation = 0;
         }
      }
   }
   return EMC_R_OK;
}  /* _load_tool_table() */

static enum EMC_RESULT _load_axis(struct emc_session *ps, int axis)
{
   char inistring[LINELEN];
   char section[32];

   sprintf(section, "AXIS_%d", axis);
   ps->axis[axis].type = EMC_AXIS_LINEAR;       // default
   if (iniGetKeyValue(section, "TYPE", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].type = _map_axis_type(inistring);

   // set backlash
   ps->axis[axis].backlash = 0;     // default
   if (iniGetKeyValue(section, "BACKLASH", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].backlash = strtod(inistring, NULL);

   // set min position limit
   ps->axis[axis].min_pos_limit = -1e99;    // default
   if (iniGetKeyValue(section, "MIN_LIMIT", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].min_pos_limit = strtod(inistring, NULL);

   // set max position limit
   ps->axis[axis].max_pos_limit = 1e99;     // default
   if (iniGetKeyValue(section, "MAX_LIMIT", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].max_pos_limit = strtod(inistring, NULL);

   // set following error limit (at max speed)
   ps->axis[axis].max_ferror = 1;       // default
   if (iniGetKeyValue(section, "FERROR", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].max_ferror = strtod(inistring, NULL);

   // do MIN_FERROR, if it's there. If not, use value of maxFerror above
   ps->axis[axis].min_ferror = ps->axis[axis].max_ferror;       // default
   if (iniGetKeyValue(section, "MIN_FERROR", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].min_ferror = strtod(inistring, NULL);

   ps->axis[axis].home = 0; // default
   if (iniGetKeyValue(section, "HOME", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].home = strtod(inistring, NULL);

   // set maximum velocity
   ps->axis[axis].max_velocity = DEFAULT_AXIS_MAX_VELOCITY;
   if (iniGetKeyValue(section, "MAX_VELOCITY", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].max_velocity = strtod(inistring, NULL);

   // set max acceleration
   ps->axis[axis].max_acceleration = DEFAULT_AXIS_MAX_ACCELERATION;
   if (iniGetKeyValue(section, "MAX_ACCELERATION", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].max_acceleration = strtod(inistring, NULL);

   // set input scale
   ps->axis[axis].steps_per_unit = 0;
   if (iniGetKeyValue(section, "INPUT_SCALE", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].steps_per_unit = strtod(inistring, NULL);

   // set step pin
   ps->axis[axis].step_pin = 0;
   if (iniGetKeyValue(section, "STEP_PIN", inistring, sizeof(inistring)) > 0)
       ps->axis[axis].step_pin = strtod(inistring, NULL);

   // set direction pin
   ps->axis[axis].direction_pin = 0;
   if (iniGetKeyValue(section, "DIRECTION_PIN", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].direction_pin = strtod(inistring, NULL);

   // set step pen polarity
   ps->axis[axis].step_active_high = 0;
   if (iniGetKeyValue(section, "STEP_ACTIVE_HIGH", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].step_active_high = strtod(inistring, NULL);

   // set direction pin polarity
   ps->axis[axis].direction_active_high = 0;
   if (iniGetKeyValue(section, "DIRECTION_ACTIVE_HIGH", inistring, sizeof(inistring)) > 0)
      ps->axis[axis].direction_active_high = strtod(inistring, NULL);

   return EMC_R_OK;
} /* _load_axis() */

static void _emcpose2py(struct emcpose_py *pospy, EmcPose pos)
{
   pospy->x = pos.tran.x;
   pospy->y = pos.tran.y;
   pospy->z = pos.tran.z;
   pospy->a = pos.a;
   pospy->b = pos.b;
   pospy->c = pos.c;
   pospy->u = pos.u;
   pospy->v = pos.v;
   pospy->w = pos.w;
}  /* _emcpose2py() */

void esleep(double seconds)
{
   if (seconds <= 0.0)
      return;

#if (defined(__WIN32__) || defined(_WINDOWS))
   SleepEx(((unsigned long) (seconds * 1000)), FALSE);
#else
   struct timespec rqtp;
   rqtp.tv_sec = seconds;
   rqtp.tv_nsec = (seconds - rqtp.tv_sec) * 1E9;
   if (nanosleep(&rqtp, NULL) < 0)
   {
      if (errno != EINTR)
      {
         BUG("nanosleep({tv_sec=%d,tv_nsec=%ld}) error (errno=%d) %s\n", (int) rqtp.tv_sec, rqtp.tv_nsec, errno, strerror(errno));
      }
   }
#endif
   return;
}       /* esleep() */

enum EMC_RESULT emc_logger_cb(const char *fmt, ...)
{
   va_list args;
   char tmp[512];

   pthread_mutex_lock(&ui_mutex);

   va_start(args, fmt);

   vsnprintf(tmp, sizeof(tmp), fmt, args);
   tmp[sizeof(tmp) - 1] = 0;    /* force zero termination */

   if (_logger_cb)
      (_logger_cb) (tmp);  /* make direct call to python logger */
   else
      fputs(tmp, stdout);

   va_end(args);

   pthread_mutex_unlock(&ui_mutex);
   return EMC_R_OK;
} /* emc_logger_cb() */

#if 0
enum EMC_RESULT emc_post_position_cb_old(EmcPose pos)
{
   int i=0, size=0;
   char value[16 * EMC_MAX_AXIS];
   char key[4 * EMC_MAX_AXIS];
   char *pkey[EMC_MAX_AXIS + 1];
   char *pvalue[EMC_MAX_AXIS + 1];
   
   pvalue[i++] = value;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.tran.x) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.tran.y) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.tran.z) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.a) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.b) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.c) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.u) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.v) + 1);
   pvalue[i++] = value+size;
   size += (snprintf(value+size, sizeof(value)-size, "%07.3f", pos.w) + 1);

   /* Force zero termination. */
   value[sizeof(value) - 1] = 0;
   pvalue[i] = NULL;

   i = size = 0;
   pkey[i++] = key;
   size += (snprintf(key+size, sizeof(key)-size, "x") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "y") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "z") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "a") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "b") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "c") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "u") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "v") + 1);
   pkey[i++] = key+size;
   size += (snprintf(key+size, sizeof(key)-size, "w") + 1);

   /* Force zero termination. */
   key[sizeof(key) - 1] = 0;
   pkey[i] = NULL;

   if (_gui_event_cb)
      (_gui_event_cb) (GUI_EVENT_MECH_POSITION, i, pkey, pvalue);

   return EMC_R_OK;
}
#endif

enum EMC_RESULT emc_post_position_cb(int id, EmcPose pos)
{
   struct post_position_py post;

   DBG("emc_post_position_cb() line_num=%d\n", id);
   post.id = id;
   _emcpose2py(&post.pos, pos);

   if (_post_position_cb)
      (_post_position_cb) (GUI_EVENT_MECH_POSITION, &post);  /* post message to gui queue */

   return EMC_R_OK;
}

enum EMC_RESULT emc_post_estop_cb(struct emc_session *ps)
{
   ps->state_bits |= EMC_STATE_ESTOP_BIT;

   DBG("emc_post_estop_cb()\n");
   if (_post_event_cb)
      (_post_event_cb) (GUI_EVENT_MECH_ESTOP);  /* post message to gui queue */
   return EMC_R_OK;
}

enum EMC_RESULT emc_post_paused_cb(struct emc_session *ps)
{
   ps->state_bits |= EMC_STATE_PAUSED_BIT;

   DBG("emc_post_paused_cb()\n");
   if (_post_event_cb)
      (_post_event_cb) (GUI_EVENT_MECH_PAUSED);  /* post message to gui queue */
   return EMC_R_OK;
}

enum EMC_RESULT emc_plugin_cb(int mcode, double p_number, double q_number)
{
   DBG("emc_plugin_cb() M%d\n", mcode);

   if (_plugin_cb)
      (_plugin_cb) (mcode, p_number, q_number);   /* make direct call to python plugin */

   return EMC_R_OK;
}
DLL_EXPORT enum EMC_RESULT emc_ui_register_logger_cb(logger_cb_t fp)
{
   _logger_cb = fp;
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_register_gui_event_cb(post_event_cb_t fp)
{
   _post_event_cb = fp;
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_register_position_cb(post_position_cb_t fp)
{
   _post_position_cb = fp;
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_register_plugin_cb(plugin_cb_t fp)
{
   _plugin_cb = fp;
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_get_state(void *hd, unsigned long *stat)
{
   struct emc_session *ps = (struct emc_session *)hd;
   *stat = ps->state_bits;
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_get_version(const char **ver)
{
   *ver = PACKAGE_VERSION;
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_estop(void *hd)
{
   struct emc_session *ps = (struct emc_session *)hd;
   DBG("emc_ui_estop() called\n");
   return dsp_estop(ps);
}       /* emc_ui_estop() */

DLL_EXPORT enum EMC_RESULT emc_ui_estop_reset(void *hd)
{
   struct emc_session *ps = (struct emc_session *)hd;
   DBG("emc_ui_estop_reset() called\n");
   return dsp_estop_reset(ps);
}       /* emc_ui_send_estop_reset() */

DLL_EXPORT enum EMC_RESULT emc_ui_home(void *hd)
{
   struct emc_session *ps = (struct emc_session *)hd;
   DBG("emc_ui_home() called\n");
   return dsp_home(ps);
}       /* emc_ui_send_home() */

DLL_EXPORT enum EMC_RESULT emc_ui_get_position(void *hd, struct emcpose_py *pospy)
{
   struct emc_session *ps = (struct emc_session *)hd;
   _emcpose2py(pospy, ps->position);
   return EMC_R_OK;
}

DLL_EXPORT enum EMC_RESULT emc_ui_wait_io_done(void *hd)
{
   struct emc_session *ps = (struct emc_session *)hd;
   return dsp_wait_io_done(ps);
}

DLL_EXPORT enum EMC_RESULT emc_ui_mdi_cmd(void *hd, const char *mdi)
{
   struct emc_session *ps = (struct emc_session *)hd;
   return dsp_mdi(ps, mdi);
}       /* emc_ui_mdi_cmd() */

DLL_EXPORT enum EMC_RESULT emc_ui_auto_cmd(void *hd, const char *gcode_file)
{
   struct emc_session *ps = (struct emc_session *)hd;
   return dsp_auto(ps, gcode_file);
}       /* emc_ui_auto_cmd() */

DLL_EXPORT enum EMC_RESULT emc_ui_verify_cmd(void *hd, const char *gcode_file)
{
   struct emc_session *ps = (struct emc_session *)hd;
   return dsp_verify(ps, gcode_file);
}       /* emc_ui_verify_cmd() */

DLL_EXPORT enum EMC_RESULT emc_ui_verify_cancel(void *hd)
{
   struct emc_session *ps = (struct emc_session *)hd;
   DBG("emc_ui_verify_cancel() called\n");
   return dsp_verify_cancel(ps);
}       /* emc_ui_verify_cancel() */

DLL_EXPORT enum EMC_RESULT emc_ui_enable_din_abort(void *hd, int input_num)
{
   struct emc_session *ps = (struct emc_session *)hd;
   return dsp_enable_din_abort(ps, input_num);
}       /*  emc_ui_enable_din_abort() */

DLL_EXPORT enum EMC_RESULT emc_ui_disable_din_abort(void *hd, int input_num)
{
   struct emc_session *ps = (struct emc_session *)hd;
   return dsp_disable_din_abort(ps, input_num);
}       /*  emc_ui_disable_din_abort() */

DLL_EXPORT enum EMC_RESULT emc_ui_test(const char *snum)
{
   return rtstepper_test(snum);
}

DLL_EXPORT void *emc_ui_open(const char *ini_file)
{
   struct emc_session *ret = NULL, *ps = &session;
   char inistring[LINELEN];
   int i;

   DBG("[%d] emc_ui_open() ini=%s\n", getpid(), ini_file);

   if ((USER_HOME_DIR = getenv("HOME")) == NULL)
      USER_HOME_DIR = "";       /* no $HOME directory, default to top level */

   strncpy(ps->ini_file, ini_file, sizeof(ps->ini_file));
   ps->ini_file[sizeof(ps->ini_file)-1] = 0;  /* force zero termination */

   iniGetKeyValue("TASK", "SERIAL_NUMBER", ps->serial_num, sizeof(ps->serial_num));
   ps->input0_abort_enabled = 0;
   if (iniGetKeyValue("TASK", "INPUT0_ABORT", inistring, sizeof(inistring)) > 0)
      ps->input0_abort_enabled = strtod(inistring, NULL);
   ps->input1_abort_enabled = 0;
   if (iniGetKeyValue("TASK", "INPUT1_ABORT", inistring, sizeof(inistring)) > 0)
      ps->input1_abort_enabled = strtod(inistring, NULL);
   ps->input2_abort_enabled = 0;
   if (iniGetKeyValue("TASK", "INPUT2_ABORT", inistring, sizeof(inistring)) > 0)
      ps->input2_abort_enabled = strtod(inistring, NULL);

   ps->axes = 4;        /* default to XYZA */
   if (iniGetKeyValue("TRAJ", "AXES", inistring, sizeof(inistring)) > 0)
   {
      ps->axes = strtod(inistring, NULL);
   }
   if (ps->axes > EMC_MAX_JOINTS)
   {
      BUG("Invalid ini file setting: axes=%d\n", ps->axes);
      ps->axes = 4;
   }
   ps->axes_mask = _map_axes_mask[ps->axes];

   if (iniGetKeyValue("TRAJ", "LINEAR_UNITS", inistring, sizeof(inistring)) > 0)
      ps->linearUnits = _map_linear_units(inistring);

   if (iniGetKeyValue("TRAJ", "ANGULAR_UNITS", inistring, sizeof(inistring)) > 0)
      ps->angularUnits = _map_angular_units(inistring);

   ps->maxVelocity = 1e99;      // by default, use AXIS limit
   if (iniGetKeyValue("TRAJ", "MAX_VELOCITY", inistring, sizeof(inistring)) > 0)
      ps->maxVelocity = strtod(inistring, NULL);

   ps->maxAcceleration = 1e99;      // by default, use AXIS limit
   if (iniGetKeyValue("TRAJ", "MAX_ACCELERATION", inistring, sizeof(inistring)) > 0)
      ps->maxAcceleration = strtod(inistring, NULL);

   if (iniGetKeyValue("EMC", "TOOL_TABLE", inistring, sizeof(inistring)) > 0)
      _load_tool_table(inistring, ps->toolTable);

   for (i=0; i < ps->axes; i++)
      _load_axis(ps, i);

   ps->cycle_time = RTSTEPPER_PERIOD * 1e-9;  /* convert ns to sec */
   ps->cycle_freq = 1 / ps->cycle_time;

   INIT_LIST_HEAD(&ps->head.list);

   emc_post_position_cb(0, ps->position);

   if (dsp_open(ps) != EMC_R_OK || rtstepper_open(ps) != EMC_R_OK)
      emc_post_estop_cb(ps);

   ret = ps;

   return ret;  /* return an opaque handle */
}       /* emc_ui_open() */

DLL_EXPORT enum EMC_RESULT emc_ui_close(void *hd)
{
   struct emc_session *ps = (struct emc_session *)hd;
   DBG("[%d] emc_ui_close()\n", getpid());
   dsp_close(ps);
   rtstepper_close(ps);
   return EMC_R_OK;
}       /* emc_ui_close() */

static void emc_dll_init(void)
{
}       /* emc_dll_init() */

static void emc_dll_exit(void)
{
}       /* emc_dll_exit() */

#if (defined(__WIN32__) || defined(_WINDOWS))
BOOL WINAPI DllMain(HANDLE module, DWORD reason, LPVOID reserved)
{
   switch (reason)
   {
   case DLL_PROCESS_ATTACH:
      emc_dll_init();
      break;
   case DLL_PROCESS_DETACH:
      emc_dll_exit();
      break;
   default:
      break;
   }
   return TRUE;
}
#else
static void __attribute__ ((constructor)) _emc_init(void)
{
   emc_dll_init();
}

static void __attribute__ ((destructor)) _emc_exit(void)
{
   emc_dll_exit();
}
#endif
