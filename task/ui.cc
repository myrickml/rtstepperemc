/*****************************************************************************\

  ui.cc - user interface support for EMC2

  (c) 2008-2012 Copyright Eckler Software

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
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include "emc.h"
#include "ini.h"
#include "interpl.h"
#include "bug.h"

#define EMC_COMMAND_DELAY   0.1 // how long to sleep between checks

struct emc_session session;

emc_status_t _emcStatus;
emc_status_t *emcStatus = &_emcStatus;
emcio_status_t emcioStatus;
emc_command_msg_t *emcCommand;
emc_command_msg_t *emcTaskCommand;
emcmot_command_t emcmotCommand;
emcmot_status_t emcmotStatus;
emcmot_config_t emcmotConfig;
emcmot_debug_t emcmotDebug;
emcmot_joint_t joints[EMCMOT_MAX_JOINTS];

KINEMATICS_FORWARD_FLAGS fflags;
KINEMATICS_INVERSE_FLAGS iflags;

MSG_INTERP_LIST interp_list;    /* MSG Union, for interpreter */

char EMC_INIFILE[LINELEN] = DEFAULT_EMC_INIFILE;
char RS274NGC_STARTUP_CODE[LINELEN] = DEFAULT_RS274NGC_STARTUP_CODE;
double EMC_TASK_CYCLE_TIME = DEFAULT_EMC_TASK_CYCLE_TIME;
double EMC_IO_CYCLE_TIME = DEFAULT_EMC_IO_CYCLE_TIME;
int EMC_TASK_INTERP_MAX_LEN = DEFAULT_EMC_TASK_INTERP_MAX_LEN;
double TRAJ_DEFAULT_VELOCITY = DEFAULT_TRAJ_DEFAULT_VELOCITY;
double TRAJ_MAX_VELOCITY = DEFAULT_TRAJ_MAX_VELOCITY;
double AXIS_MAX_VELOCITY[EMC_AXIS_MAX];
double AXIS_MAX_ACCELERATION[EMC_AXIS_MAX];

char TOOL_TABLE_FILE[LINELEN] = DEFAULT_TOOL_TABLE_FILE;
EmcPose TOOL_CHANGE_POSITION;
unsigned char HAVE_TOOL_CHANGE_POSITION;
EmcPose TOOL_HOLDER_CLEAR;
unsigned char HAVE_TOOL_HOLDER_CLEAR;

static int emcCommandSerialNumber;
static double emcTimeout;
static enum EMC_UI_WAIT_TYPE emcWaitType;

static emc_msg_t taskPlanSynchCmd = { EMC_TASK_PLAN_SYNCH_TYPE };

enum EMC_RESULT emc_ui_update_status(void)
{
   return EMC_R_OK;     /* no need to update emcStatus */
}

enum EMC_RESULT emc_ui_update_operator_error(char *buf, int buf_size)
{
   struct emc_session *ps = &session;
   emc_command_msg_t *m;
   unsigned int last = 0;
   int lock = 0, done = 0;
   const char tag[] = "gui";

   if (buf == NULL || buf_size <= 0)
   {
      BUG("invalid input\n");
      return EMC_R_ERROR;
   }

   buf[0] = 0;

   while (!done)
   {
      peek_message(ps, &m, &last, &lock, tag);

      if (m)
      {
         if (m->msg.type == EMC_OPERATOR_ERROR_TYPE)
         {
            remove_message(ps, m, &lock, tag);
            strncpy(buf, ((emc_operator_error_msg_t *) m)->error, buf_size);
            buf[buf_size - 1] = 0;
            free(m);
            done = 1;
         }
      }
      else
      {
         break; /* no more messages */
      }
   }

   return EMC_R_OK;
}       /* emc_ui_update_operator_error() */

enum EMC_RESULT emc_ui_update_operator_text(char *buf, int buf_size)
{
   struct emc_session *ps = &session;
   emc_command_msg_t *m;
   unsigned int last = 0;
   int lock = 0, done=0;
   const char tag[] = "gui";

   if (buf == NULL || buf_size <= 0)
   {
      BUG("invalid input\n");
      return EMC_R_ERROR;
   }

   buf[0] = 0;

   while (!done)
   {
      peek_message(ps, &m, &last, &lock, tag);

      if (m)
      {
         if (m->msg.type == EMC_OPERATOR_TEXT_TYPE)
         {
            remove_message(ps, m, &lock, tag);
            strncpy(buf, ((emc_operator_text_msg_t *) m)->text, buf_size);
            buf[buf_size - 1] = 0;
            free(m);
            done = 1;
         }
      }
      else
      {
         break; /* no more messages */
      }
   }

   return EMC_R_OK;
}       /* emc_ui_update_operator_text() */

enum EMC_RESULT emc_ui_update_operator_display(char *buf, int buf_size)
{
   struct emc_session *ps = &session;
   emc_command_msg_t *m;
   unsigned int last = 0;
   int lock = 0, done=0;
   const char tag[] = "gui";

   if (buf == NULL || buf_size <= 0)
   {
      BUG("invalid input\n");
      return EMC_R_ERROR;
   }

   buf[0] = 0;
   while (!done)
   {
      peek_message(ps, &m, &last, &lock, tag);

      if (m)
      {
         if (m->msg.type == EMC_OPERATOR_DISPLAY_TYPE)
         {
            remove_message(ps, m, &lock, tag);
            strncpy(buf, ((emc_operator_display_msg_t *) m)->display, buf_size);
            buf[buf_size - 1] = 0;
            free(m);
            done = 1;
         }
      }
      else
      {
         break; /* no more messages */
      }
   }

   return EMC_R_OK;
}       /* emc_ui_update_operator_display() */

/* Wait for last command to be received by control thread. */
enum EMC_RESULT emc_ui_command_wait_received(void)
{
   double end = 0.0;

   while (emcTimeout <= 0.0 || end < emcTimeout)
   {
      emc_ui_update_status();

      if (emcStatus->echo_serial_number == emcCommandSerialNumber)
      {
         return EMC_R_OK;
      }

      esleep(EMC_COMMAND_DELAY);
      end += EMC_COMMAND_DELAY;
   }

   return EMC_R_TIMEOUT;
}       /* emc_ui_command_wait_received() */

/* Wait for last command to finish executing. */
enum EMC_RESULT emc_ui_command_wait_done(void)
{
   double end = 0.0;
   enum EMC_RESULT ret;

   // first get it there
   if ((ret = emc_ui_command_wait_received()) != EMC_R_OK)
   {
      return ret;
   }
   // now wait until it, or subsequent command (e.g., abort) is done
   while (emcTimeout <= 0.0 || end < emcTimeout)
   {
      emc_ui_update_status();

      if (emcStatus->status == RCS_DONE)
      {
         return EMC_R_OK;
      }

      if (emcStatus->status == RCS_ERROR)
      {
         return EMC_R_ERROR;
      }

      esleep(EMC_COMMAND_DELAY);
      end += EMC_COMMAND_DELAY;
   }

   return EMC_R_TIMEOUT;
}       /* emc_ui_command_wait_done() */

enum EMC_RESULT emc_ui_set_timeout(double timeout)
{
   if (timeout >= 0.0)
   {
      emcTimeout = timeout;
      return EMC_R_OK;
   }
   else
   {
      BUG("input error\n");
      return EMC_R_ERROR;
   }
}       /* emc_ui_set_timeout() */

double emc_ui_get_timeout(void)
{
   return emcTimeout;
}       /* emc_ui_get_timeout() */

enum EMC_RESULT emc_ui_set_wait_type(enum EMC_UI_WAIT_TYPE type)
{
   emcWaitType = type;
   return EMC_R_OK;
}       /* emc_ui_set_wait_type() */

enum EMC_UI_WAIT_TYPE emc_ui_get_wait_type(void)
{
   return emcWaitType;
}       /* emc_ui_get_wait_type() */

enum EMC_RESULT emc_ui_send_estop(void)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_ESTOP;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_estop() */

enum EMC_RESULT emc_ui_send_estop_reset(void)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_ESTOP_RESET;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_estop_reset() */

enum EMC_RESULT emc_ui_send_machine_on(void)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_ON;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_machine_on() */

enum EMC_RESULT emc_ui_send_machine_off(void)
{
   emc_command_msg_t mb;
   emc_task_set_state_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_set_state_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_STATE_TYPE;
   cmd->state = EMC_TASK_STATE_OFF;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_machine_off() */

enum EMC_RESULT emc_ui_send_manual(void)
{
   emc_command_msg_t mb;
   emc_task_set_mode_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_set_mode_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_MODE_TYPE;
   cmd->mode = EMC_TASK_MODE_MANUAL;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /*  emc_ui_send_manual() */

enum EMC_RESULT emc_ui_send_auto(void)
{
   emc_command_msg_t mb;
   emc_task_set_mode_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_set_mode_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_MODE_TYPE;
   cmd->mode = EMC_TASK_MODE_AUTO;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /*  emc_ui_send_auto() */

enum EMC_RESULT emc_ui_send_mdi(void)
{
   emc_command_msg_t mb;
   emc_task_set_mode_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_set_mode_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_SET_MODE_TYPE;
   cmd->mode = EMC_TASK_MODE_MDI;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /*  emc_ui_send_mdi() */

enum EMC_RESULT emc_ui_send_mist_on(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_COOLANT_MIST_ON_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_mist_on() */

enum EMC_RESULT emc_ui_send_mist_off(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_COOLANT_MIST_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_mist_off() */

enum EMC_RESULT emc_ui_send_flood_on(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_COOLANT_FLOOD_ON_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_flood_on() */

enum EMC_RESULT emc_ui_send_flood_off(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_COOLANT_FLOOD_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_flood_off() */

enum EMC_RESULT emc_ui_send_lube_on(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_LUBE_ON_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_lube_on() */

enum EMC_RESULT emc_ui_send_lube_off(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_LUBE_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_lube_off() */

enum EMC_RESULT emc_ui_send_spindle_forward(void)
{
   emc_command_msg_t mb;
   emc_spindle_on_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_spindle_on_msg_t *) & mb;
   cmd->msg.type = EMC_SPINDLE_ON_TYPE;
   cmd->speed = 500;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_forward() */

enum EMC_RESULT emc_ui_send_spindle_reverse(void)
{
   emc_command_msg_t mb;
   emc_spindle_on_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_spindle_on_msg_t *) & mb;
   cmd->msg.type = EMC_SPINDLE_ON_TYPE;
   cmd->speed = -500;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_reverse() */

enum EMC_RESULT emc_ui_send_spindle_off(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_SPINDLE_OFF_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_off() */

enum EMC_RESULT emc_ui_send_spindle_increase(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_SPINDLE_INCREASE_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_increase() */

enum EMC_RESULT emc_ui_send_spindle_decrease(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_SPINDLE_DECREASE_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_decrease() */

enum EMC_RESULT emc_ui_send_spindle_constant(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_SPINDLE_CONSTANT_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_constant() */

enum EMC_RESULT emc_ui_send_brake_engage(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_SPINDLE_BRAKE_ENGAGE_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_brake_engage() */

enum EMC_RESULT emc_ui_send_brake_release(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_SPINDLE_BRAKE_RELEASE_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_spindle_brake_release() */

enum EMC_RESULT emc_ui_send_load_tool_table(const char *file)
{
   emc_command_msg_t mb;
   emc_tool_load_tool_table_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_tool_load_tool_table_msg_t *) & mb;
   cmd->msg.type = EMC_TOOL_LOAD_TOOL_TABLE_TYPE;
   strncpy(cmd->file, file, sizeof(cmd->file));
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_load_tool_table() */

enum EMC_RESULT emc_ui_send_tool_set_offset(int toolno, double zoffset, double diameter)
{
   emc_command_msg_t mb;
   emc_tool_set_offset_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_tool_set_offset_msg_t *) & mb;
   cmd->msg.type = EMC_TOOL_SET_OFFSET_TYPE;
   cmd->toolno = toolno;
   cmd->offset.tran.z = zoffset;
   cmd->diameter = diameter;
   cmd->orientation = 0;        // mill style tool table
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_tool_set_offset() */

enum EMC_RESULT emc_ui_send_override_limits(int axis)
{
   emc_command_msg_t mb;
   emc_axis_cmd_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_axis_cmd_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_OVERRIDE_LIMITS_TYPE;
   cmd->axis = axis;    /* negative means off */
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_override_limits() */

enum EMC_RESULT emc_ui_send_mdi_cmd(const char *mdi)
{
   emc_command_msg_t mb;
   emc_task_plan_execute_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_plan_execute_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_PLAN_EXECUTE_TYPE;
   strncpy(cmd->command, mdi, sizeof(cmd->command));
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_mdi_cmd() */

enum EMC_RESULT emc_ui_send_home(int axis)
{
   emc_command_msg_t mb;
   emc_axis_cmd_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_axis_cmd_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_HOME_TYPE;
   cmd->axis = axis;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_home() */

enum EMC_RESULT emc_ui_send_unhome(int axis)
{
   emc_command_msg_t mb;
   emc_axis_cmd_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_axis_cmd_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_UNHOME_TYPE;
   cmd->axis = axis;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_unhome() */

enum EMC_RESULT emc_ui_send_jog_stop(int axis)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   // in case of TELEOP mode we really need to send an TELEOP_VECTOR message
   // not a simple AXIS_ABORT, as more than one axis would be moving
   // (hint TELEOP mode is for nontrivial kinematics)
   if (emcStatus->motion.traj.mode != EMC_TRAJ_MODE_TELEOP)
   {
      emc_axis_cmd_msg_t *cmd;
      cmd = (emc_axis_cmd_msg_t *) & mb;
      cmd->msg.type = EMC_AXIS_ABORT_TYPE;
      cmd->axis = axis;
      cmd->msg.serial_number = ++emcCommandSerialNumber;
   }
   else
   {
      emc_traj_set_teleop_vector_msg_t *cmd;
      cmd = (emc_traj_set_teleop_vector_msg_t *) & mb;
      cmd->msg.type = EMC_TRAJ_SET_TELEOP_VECTOR_TYPE;
      ZERO_EMC_POSE(cmd->vector);
      cmd->msg.serial_number = ++emcCommandSerialNumber;
   }
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_jog_stop() */

enum EMC_RESULT emc_ui_send_jog_cont(int axis, double speed)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   if (emcStatus->motion.traj.mode != EMC_TRAJ_MODE_TELEOP)
   {
      emc_axis_jog_msg_t *cmd;
      cmd = (emc_axis_jog_msg_t *) & mb;
      cmd->msg.type = EMC_AXIS_JOG_TYPE;
      cmd->axis = axis;
      cmd->vel = speed / 60.0;
      cmd->msg.serial_number = ++emcCommandSerialNumber;
   }
   else
   {
      emc_traj_set_teleop_vector_msg_t *cmd;
      cmd = (emc_traj_set_teleop_vector_msg_t *) & mb;
      cmd->msg.type = EMC_TRAJ_SET_TELEOP_VECTOR_TYPE;
      ZERO_EMC_POSE(cmd->vector);
      cmd->msg.serial_number = ++emcCommandSerialNumber;

      switch (axis)
      {
      case 0:
         cmd->vector.tran.x = speed / 60.0;
         break;
      case 1:
         cmd->vector.tran.y = speed / 60.0;
         break;
      case 2:
         cmd->vector.tran.z = speed / 60.0;
         break;
      case 3:
         cmd->vector.a = speed / 60.0;
         break;
      case 4:
         cmd->vector.b = speed / 60.0;
         break;
      case 5:
         cmd->vector.c = speed / 60.0;
         break;
      }
   }
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_jog_stop() */

enum EMC_RESULT emc_ui_send_jog_incr(int axis, double speed, double incr)
{
   emc_command_msg_t mb;
   emc_axis_incr_jog_msg_t *cmd;
   struct emc_session *ps = &session;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   cmd = (emc_axis_incr_jog_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_INCR_JOG_TYPE;
   cmd->axis = axis;
   cmd->vel = speed / 60.0;
   cmd->incr = incr;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /*  emc_ui_send_jog_incr() */

enum EMC_RESULT emc_ui_send_feed_override(double override)
{
   emc_command_msg_t mb;
   emc_traj_set_scale_msg_t *cmd;
   struct emc_session *ps = &session;

   if (override < 0.0)
   {
      override = 0.0;
   }

   cmd = (emc_traj_set_scale_msg_t *) & mb;
   cmd->msg.type = EMC_TRAJ_SET_SCALE_TYPE;
   cmd->scale = override;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_feed_override() */

enum EMC_RESULT emc_ui_send_task_plan_init(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_TASK_PLAN_INIT_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_task_plan_init() */

enum EMC_RESULT emc_ui_send_program_open(const char *program)
{
   emc_command_msg_t mb;
   emc_task_plan_open_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_plan_open_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_PLAN_OPEN_TYPE;
   strncpy(cmd->file, program, sizeof(cmd->file));
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_program_open() */

enum EMC_RESULT emc_ui_send_program_run(int line)
{
   emc_command_msg_t mb;
   emc_task_plan_run_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_task_plan_run_msg_t *) & mb;
   cmd->msg.type = EMC_TASK_PLAN_RUN_TYPE;
   cmd->line = line;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_program_run() */

enum EMC_RESULT emc_ui_send_program_pause(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_TASK_PLAN_PAUSE_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_program_pause() */

enum EMC_RESULT emc_ui_send_program_resume(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_TASK_PLAN_RESUME_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_program_resume() */

enum EMC_RESULT emc_ui_send_program_step(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_TASK_PLAN_STEP_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_program_step() */

enum EMC_RESULT emc_ui_send_abort(void)
{
   emc_command_msg_t mb;
   struct emc_session *ps = &session;

   mb.msg.type = EMC_TASK_ABORT_TYPE;
   mb.msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_abort() */

enum EMC_RESULT emc_ui_send_axis_set_backlash(int axis, double backlash)
{
   emc_command_msg_t mb;
   emc_axis_set_backlash_msg_t *cmd;
   struct emc_session *ps = &session;

   if (axis < 0 || axis >= EMC_AXIS_MAX)
   {
      BUG("invalid input axis=%d\n", axis);
      return EMC_R_ERROR;
   }

   cmd = (emc_axis_set_backlash_msg_t *) & mb;
   cmd->msg.type = EMC_AXIS_SET_BACKLASH_TYPE;
   cmd->axis = axis;
   cmd->backlash = backlash;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /*  emc_ui_send_axis_set_backlash() */

enum EMC_RESULT emc_ui_send_teleop_enable(int enable)
{
   emc_command_msg_t mb;
   emc_traj_set_teleop_enable_msg_t *cmd;
   struct emc_session *ps = &session;

   cmd = (emc_traj_set_teleop_enable_msg_t *) & mb;
   cmd->msg.type = EMC_TRAJ_SET_TELEOP_ENABLE_TYPE;
   cmd->enable = enable;
   cmd->msg.serial_number = ++emcCommandSerialNumber;
   send_message(ps, &mb, "gui");

   if (emcWaitType == EMC_UI_WAIT_RECEIVED)
   {
      return emc_ui_command_wait_received();
   }
   else if (emcWaitType == EMC_UI_WAIT_DONE)
   {
      return emc_ui_command_wait_done();
   }
   return EMC_R_OK;
}       /* emc_ui_send_teleop_enable() */

enum EMC_RESULT emc_ui_get_args(int argc, char *argv[])
{
   int i, n;
   enum EMC_RESULT stat = EMC_R_OK;

   /* process command line args, indexing argv[] from [1] */
   for (i = 1; i < argc; i++)
   {
      if (strncmp(argv[i], "-psn", 4) == 0)
         break;  /* ignore any commands, running from OSX application bundle. */ 

      if (strcmp(argv[i], "-ini") == 0)
      {
         if (i == argc - 1)
         {
            stat = EMC_R_ERROR;
            break; /* no parameter */
         }
         n = sizeof(EMC_INIFILE);
         strncpy(EMC_INIFILE, argv[i + 1], n);
         EMC_INIFILE[n - 1] = 0;
         i++;
         continue;  /* got a valid command */
      }

      stat = EMC_R_ERROR;
      break;     /* no more valid commands were done */
   }

   return stat;
}       /* emc_ui_get_args() */

void control_thread(struct emc_session *ps)
{
   emc_command_msg_t *m;
   enum RTSTEPPER_RESULT ret;
   unsigned int last = 0;
   int taskPlanError, taskExecuteError;
   int lock = 0, done;
   const char tag[] = "ctl";

   pthread_detach(pthread_self());

   ps->control_thread_active = 1;
   ps->control_thread_abort = 0;

   while (!ps->control_thread_abort)
   {
      /* With dongle connected, following query provides a 1ms cycle time. */
      ret = rtstepper_query_state(&ps->dongle);

      if (rtstepper_is_input0_triggered(&ps->dongle) == RTSTEPPER_R_INPUT0_TRUE)
      {
         BUG("INPUT0 estop...\n");
         emcOperatorError(0, "INPUT0 ESTOP...");
         emcTaskSetState(EMC_TASK_STATE_ESTOP);
      }
      if (rtstepper_is_input1_triggered(&ps->dongle) == RTSTEPPER_R_INPUT1_TRUE)
      {
         BUG("INPUT1 estop...\n");
         emcOperatorError(0, "INPUT1 ESTOP...");
         emcTaskSetState(EMC_TASK_STATE_ESTOP);
      }
      if (rtstepper_is_input2_triggered(&ps->dongle) == RTSTEPPER_R_INPUT2_TRUE)
      {
         BUG("INPUT2 estop...\n");
         emcOperatorError(0, "INPUT2 ESTOP...");
         emcTaskSetState(EMC_TASK_STATE_ESTOP);
      }

      if (ret == RTSTEPPER_R_REQ_ERROR)
         esleep(0.01);  /* No dongle connected, let other processes have cpu time */

      done = 0;
      while (!done)
      {
         /* Check all messages looking for control_thread command. */
         peek_message(ps, &m, &last, &lock, tag);

         if (m)
         {
            if (m->msg.type > MAX_EMC_TO_GUI_CMD)
            {
               /* Found GUI to EMC command. Do not remove jog command if until previous move is complete. */
               if ((m->msg.type <= MAX_GUI_TO_EMC_IMMEDIATE_CMD) || (m->msg.type > MAX_GUI_TO_EMC_IMMEDIATE_CMD && emcStatus->status != RCS_EXEC))
               {
                  remove_message(ps, m, &lock, tag);
                  if (emcCommand != NULL)
                     free(emcCommand);
                  emcCommand = m;
                  done = 1;
               }
            }
         }
         else
         {
            break; /* no more messages */
         }
      }

      taskPlanError = 0;
      taskExecuteError = 0;

      if (emcCommand)
      {
         // run control cycle
         if (emcTaskPlan() != EMC_R_OK)
            taskPlanError = 1;
         if (emcTaskExecute() != EMC_R_OK)
            taskExecuteError = 1;
      }

      // update subordinate status
      emcIoUpdate(&emcStatus->io);
      emcMotionUpdate(&emcStatus->motion);

      // check for subordinate errors, and halt task if so
      if (emcStatus->motion.status == RCS_ERROR || emcStatus->io.status == RCS_ERROR)
      {
         emcTaskAbort();
         emcIoAbort();
         emcSpindleOff();

         // clear out the pending command
         emcTaskCommand = NULL;
         interp_list.clear();
         emcStatus->task.currentLine = 0;

         // clear out the interpreter state
         emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
         emcStatus->task.execState = EMC_TASK_EXEC_DONE;

         // now queue up command to resynch interpreter
         interp_list.append((emc_command_msg_t *) & taskPlanSynchCmd);
      }

      // update task-specific status
      emcTaskUpdate(&emcStatus->task);

      if (emcCommand)
      {
         // get task status
         emcStatus->task.command_type = emcCommand->msg.type;
         emcStatus->task.echo_serial_number = emcCommand->msg.serial_number;
         // get top level status
         emcStatus->command_type = emcCommand->msg.type;
         emcStatus->echo_serial_number = emcCommand->msg.serial_number;
      }

      if (taskPlanError || taskExecuteError || emcStatus->task.execState == EMC_TASK_EXEC_ERROR ||
          emcStatus->motion.status == RCS_ERROR || emcStatus->io.status == RCS_ERROR)
      {
         emcStatus->status = RCS_ERROR;
         emcStatus->task.status = RCS_ERROR;
      }
      else if (!taskPlanError && !taskExecuteError && emcStatus->task.execState == EMC_TASK_EXEC_DONE &&
               emcStatus->motion.status == RCS_DONE && emcStatus->io.status == RCS_DONE && interp_list.len() == 0 &&
               emcTaskCommand == NULL && emcStatus->task.interpState == EMC_TASK_INTERP_IDLE)
      {
         emcStatus->status = RCS_DONE;
         emcStatus->task.status = RCS_DONE;
      }
      else
      {
         emcStatus->status = RCS_EXEC;
         emcStatus->task.status = RCS_EXEC;
      }

#if 0
      if (emcStatus->status != RCS_DONE)
      {
         DBG("**planError=%d executeError=%d execState=%d motion.status=%d interp.len=%d emcTaskCmd=%p interpState=%d\n",
            taskPlanError, taskExecuteError, emcStatus->task.execState, emcStatus->motion.status, interp_list.len(),
            emcTaskCommand, emcStatus->task.interpState);
      }
#endif
   }    /* while (!ps->control_thread_abort) */

   /* Reap any remaining messages. Free any outstanding lock. */
   while (1)
   {
      peek_message(ps, &m, &last, &lock, tag);
      if (m)
      {
         remove_message(ps, m, &lock, tag);
         free(m);
         continue;
      }
      break;
   }

   rtstepper_set_abort_wait(&ps->dongle);
   DBG("exiting control_thread()\n");
   pthread_mutex_lock(&ps->mutex);
   ps->control_thread_active = 0;
   pthread_cond_signal(&ps->control_thread_done_cond);
   pthread_mutex_unlock(&ps->mutex);

   return;
}       /* control_thread() */

enum EMC_RESULT emc_ui_init(const char *ini_file)
{
   enum EMC_RESULT stat = EMC_R_ERROR;
   struct emc_session *ps = &session;
   FILE *logfd = NULL;
   const char *hdir;
   char log_file[256], tmp_file[256];

   if (ini_file != NULL)
   {
      strncpy(EMC_INIFILE, ini_file, sizeof(EMC_INIFILE));
      EMC_INIFILE[sizeof(EMC_INIFILE) - 1] = 0;
   }

   pthread_mutex_init(&ps->mutex, NULL);
   pthread_cond_init(&ps->control_thread_done_cond, NULL);
   pthread_cond_init(&ps->control_cycle_thread_done_cond, NULL);
   INIT_LIST_HEAD(&ps->head.list);

   if ((hdir = getenv("HOME")) == NULL)
      hdir = "";        /* no $HOME directory, default to top level */
   iniGetKeyValue("TASK", "SERIAL_NUMBER", tmp_file, sizeof(tmp_file));
   snprintf(log_file, sizeof(log_file), "%s/.%s/%s%s", hdir, PACKAGE_NAME, EMC2_TASKNAME, tmp_file);

   /* Make sure we can open the log file in the user's home directory. */
   if ((logfd = fopen(log_file, "r")) == NULL)
   {
      /* Open failed, create directory. */
      snprintf(tmp_file, sizeof(tmp_file), "%s/.%s", hdir, PACKAGE_NAME);
#if (defined(__WIN32__) || defined(_WINDOWS))
      if (mkdir(tmp_file))
#else
      if (mkdir(tmp_file, 0700))
#endif
      {
         if (errno != EEXIST)
         {
            fprintf(stderr, "unable to create %s\n", tmp_file);
            emcOperatorError(0, EMC_I18N("unable to create %s\n"), tmp_file);
            goto bugout;        /* bail */
         }
      }
   }
   else
      fclose(logfd);

   rtstepper_open_log(log_file, RTSTEPPER_LOG_BACKUP);

   DBG("[%d] emc_ui_init() ini=%s\n", getpid(), ini_file);

   emcInitGlobals();

   emcWaitType = EMC_UI_WAIT_RECEIVED;
   emcCommandSerialNumber = 0;
   emcTimeout = 0.0;

   if (iniTask(EMC_INIFILE) != EMC_R_OK)
      goto bugout;

   /* Init some dongle defaults?? Was normally set by taskintf.cc (task to motion interface). */
   emcStatus->io.aux.estop = 1;
   emcStatus->motion.traj.enabled = 0;
   emcStatus->motion.traj.mode = EMC_TRAJ_MODE_FREE;    /* for manual mode */
   emcStatus->io.status = RCS_DONE;

   emcStatus->task.interpState = EMC_TASK_INTERP_IDLE;
   emcStatus->task.execState = EMC_TASK_EXEC_DONE;

   if (rtstepper_init(&ps->dongle, emc_io_error_cb) != RTSTEPPER_R_OK)
      emcOperatorError(0, EMC_I18N("unable to connnect to rt-stepper dongle"));

   emcIoInit();
   emcIoUpdate(&emcStatus->io);
   emcMotionInit();
   emcMotionUpdate(&emcStatus->motion);

   /* Initialize the interpreter. */
   if (emcTaskPlanInit() != EMC_R_OK)
   {
      BUG("can't initialize interpreter\n");
      emcOperatorError(0, EMC_I18N("can't initialize interpreter"));
      goto bugout;
   }

   emcTaskUpdate(&emcStatus->task);

   if (pthread_create(&ps->control_thread_tid, NULL, (void *(*)(void *)) control_thread, (void *) ps) != 0)
   {
      BUG("unable to creat control_thread\n");
      ps->control_thread_active = 0;
      goto bugout;      /* bail */
   }

   stat = EMC_R_OK;

 bugout:
   return stat;
}       /* emc_ui_init() */

enum EMC_RESULT emc_ui_exit(void)
{
   struct emc_session *ps = &session;

   DBG("[%d] emc_ui_exit()\n", getpid());

   if (ps->control_thread_active)
   {
      /* Gracefully kill control_thread. */
      pthread_mutex_lock(&ps->mutex);
      ps->control_thread_abort = 1;
      while (ps->control_thread_active)
         pthread_cond_wait(&ps->control_thread_done_cond, &ps->mutex);
      pthread_mutex_unlock(&ps->mutex);
   }

   emcTaskPlanExit();
   emcMotionHalt();
   emcIoHalt();
   rtstepper_exit(&ps->dongle);
   pthread_mutex_destroy(&ps->mutex);
   pthread_cond_destroy(&ps->control_thread_done_cond);
   pthread_cond_destroy(&ps->control_cycle_thread_done_cond);

   return EMC_R_OK;
}       /* emc_ui_exit() */

static void emc_dll_init(void)
{
} /* emc_dll_init() */

static void emc_dll_exit(void)
{
   struct emc_session *ps = &session;
   DBG("[%d] emc_dll_exit()\n", getpid());

   if (ps->control_thread_active)
      emc_ui_exit();

   rtstepper_close_log();
} /* emc_dll_exit() */

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
