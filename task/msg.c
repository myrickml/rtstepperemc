/************************************************************************************\

  msg.c - message queue processor for rt-stepper

  (c) 2008-2009 Copyright Eckler Software

  Author: David Suffield, dsuffiel@ecklersoft.com

\************************************************************************************/

#include <string.h>
#include "emc.h"
#include "msg.h"
#include "bug.h"

static unsigned int seq_num = 1;

const char *lookup_message(int type)
{
   switch (type)
   {
   case EMC_ABORT_TYPE:
      return "EMC_ABORT_TYPE";
   case EMC_AUX_ESTOP_RESET_TYPE:
      return "EMC_AUX_ESTOP_RESET_TYPE";
   case EMC_AUX_ESTOP_OFF_TYPE:
      return "EMC_AUX_ESTOP_OFF_TYPE";
   case EMC_AUX_ESTOP_ON_TYPE:
      return "EMC_AUX_ESTOP_ON_TYPE";
   case EMC_AUX_STAT_TYPE:
      return "EMC_AUX_STAT_TYPE";
   case EMC_AXIS_ABORT_TYPE:
      return "EMC_AXIS_ABORT_TYPE";
   case EMC_AXIS_ABS_JOG_TYPE:
      return "EMC_AXIS_ABS_JOG_TYPE";
   case EMC_AXIS_ACTIVATE_TYPE:
      return "EMC_AXIS_ACTIVATE_TYPE";
   case EMC_AXIS_DEACTIVATE_TYPE:
      return "EMC_AXIS_DEACTIVATE_TYPE";
   case EMC_AXIS_DISABLE_TYPE:
      return "EMC_AXIS_DISABLE_TYPE";
   case EMC_AXIS_ENABLE_TYPE:
      return "EMC_AXIS_ENABLE_TYPE";
   case EMC_AXIS_HALT_TYPE:
      return "EMC_AXIS_HALT_TYPE";
   case EMC_AXIS_HOME_TYPE:
      return "EMC_AXIS_HOME_TYPE";
   case EMC_AXIS_UNHOME_TYPE:
      return "EMC_AXIS_UNHOME_TYPE";
   case EMC_AXIS_INCR_JOG_TYPE:
      return "EMC_AXIS_INCR_JOG_TYPE";
   case EMC_AXIS_INIT_TYPE:
      return "EMC_AXIS_INIT_TYPE";
   case EMC_AXIS_JOG_TYPE:
      return "EMC_AXIS_JOG_TYPE";
   case EMC_AXIS_LOAD_COMP_TYPE:
      return "EMC_AXIS_LOAD_COMP_TYPE";
   case EMC_AXIS_OVERRIDE_LIMITS_TYPE:
      return "EMC_AXIS_OVERRIDE_LIMITS_TYPE";
   case EMC_AXIS_SET_AXIS_TYPE:
      return "EMC_AXIS_SET_AXIS_TYPE";
   case EMC_AXIS_SET_FERROR_TYPE:
      return "EMC_AXIS_SET_FERROR_TYPE";
   case EMC_AXIS_SET_BACKLASH_TYPE:
      return "EMC_AXIS_SET_BACKLASH_TYPE";
   case EMC_AXIS_SET_HOMING_PARAMS_TYPE:
      return "EMC_AXIS_SET_HOMING_PARAMS_TYPE";
   case EMC_AXIS_SET_MAX_POSITION_LIMIT_TYPE:
      return "EMC_AXIS_SET_MAX_POSITION_LIMIT_TYPE";
   case EMC_AXIS_SET_MAX_VELOCITY_TYPE:
      return "EMC_AXIS_SET_MAX_VELOCITY_TYPE";
   case EMC_AXIS_SET_MIN_FERROR_TYPE:
      return "EMC_AXIS_SET_MIN_FERROR_TYPE";
   case EMC_AXIS_SET_MIN_POSITION_LIMIT_TYPE:
      return "EMC_AXIS_SET_MIN_POSITION_LIMIT_TYPE";
   case EMC_AXIS_SET_UNITS_TYPE:
      return "EMC_AXIS_SET_UNITS_TYPE";
   case EMC_AXIS_STAT_TYPE:
      return "EMC_AXIS_STAT_TYPE";
   case EMC_COOLANT_FLOOD_OFF_TYPE:
      return "EMC_COOLANT_FLOOD_OFF_TYPE";
   case EMC_COOLANT_FLOOD_ON_TYPE:
      return "EMC_COOLANT_FLOOD_ON_TYPE";
   case EMC_COOLANT_MIST_OFF_TYPE:
      return "EMC_COOLANT_MIST_OFF_TYPE";
   case EMC_COOLANT_MIST_ON_TYPE:
      return "EMC_COOLANT_MIST_ON_TYPE";
   case EMC_COOLANT_STAT_TYPE:
      return "EMC_COOLANT_STAT_TYPE";
   case EMC_HALT_TYPE:
      return "EMC_HALT_TYPE";
   case EMC_INIT_TYPE:
      return "EMC_INIT_TYPE";
   case EMC_IO_ABORT_TYPE:
      return "EMC_IO_ABORT_TYPE";
   case EMC_IO_HALT_TYPE:
      return "EMC_IO_HALT_TYPE";
   case EMC_IO_INIT_TYPE:
      return "EMC_IO_INIT_TYPE";
   case EMC_IO_SET_CYCLE_TIME_TYPE:
      return "EMC_IO_SET_CYCLE_TIME_TYPE";
   case EMC_IO_STAT_TYPE:
      return "EMC_IO_STAT_TYPE";
   case EMC_LUBE_OFF_TYPE:
      return "EMC_LUBE_OFF_TYPE";
   case EMC_LUBE_ON_TYPE:
      return "EMC_LUBE_ON_TYPE";
   case EMC_LUBE_STAT_TYPE:
      return "EMC_LUBE_STAT_TYPE";
   case EMC_MOTION_ABORT_TYPE:
      return "EMC_MOTION_ABORT_TYPE";
   case EMC_MOTION_HALT_TYPE:
      return "EMC_MOTION_HALT_TYPE";
   case EMC_MOTION_INIT_TYPE:
      return "EMC_MOTION_INIT_TYPE";
   case EMC_MOTION_SET_AOUT_TYPE:
      return "EMC_MOTION_SET_AOUT_TYPE";
   case EMC_MOTION_SET_DOUT_TYPE:
      return "EMC_MOTION_SET_DOUT_TYPE";
   case EMC_MOTION_ADAPTIVE_TYPE:
      return "EMC_MOTION_ADAPTIVE_TYPE";
   case EMC_MOTION_STAT_TYPE:
      return "EMC_MOTION_STAT_TYPE";
   case EMC_OPERATOR_DISPLAY_TYPE:
      return "EMC_OPERATOR_DISPLAY_TYPE";
   case EMC_OPERATOR_ERROR_TYPE:
      return "EMC_OPERATOR_ERROR_TYPE";
   case EMC_OPERATOR_TEXT_TYPE:
      return "EMC_OPERATOR_TEXT_TYPE";
   case EMC_SYSTEM_CMD_TYPE:
      return "EMC_SYSTEM_CMD_TYPE";
   case EMC_SPINDLE_BRAKE_ENGAGE_TYPE:
      return "EMC_SPINDLE_BRAKE_ENGAGE_TYPE";
   case EMC_SPINDLE_BRAKE_RELEASE_TYPE:
      return "EMC_SPINDLE_BRAKE_RELEASE_TYPE";
   case EMC_SPINDLE_CONSTANT_TYPE:
      return "EMC_SPINDLE_CONSTANT_TYPE";
   case EMC_SPINDLE_DECREASE_TYPE:
      return "EMC_SPINDLE_DECREASE_TYPE";
   case EMC_SPINDLE_INCREASE_TYPE:
      return "EMC_SPINDLE_INCREASE_TYPE";
   case EMC_SPINDLE_OFF_TYPE:
      return "EMC_SPINDLE_OFF_TYPE";
   case EMC_SPINDLE_ON_TYPE:
      return "EMC_SPINDLE_ON_TYPE";
   case EMC_SPINDLE_SPEED_TYPE:
      return "EMC_SPINDLE_SPEED_TYPE";
   case EMC_SPINDLE_STAT_TYPE:
      return "EMC_SPINDLE_STAT_TYPE";
   case EMC_STAT_TYPE:
      return "EMC_STAT_TYPE";
   case EMC_TASK_ABORT_TYPE:
      return "EMC_TASK_ABORT_TYPE";
   case EMC_TASK_HALT_TYPE:
      return "EMC_TASK_HALT_TYPE";
   case EMC_TASK_INIT_TYPE:
      return "EMC_TASK_INIT_TYPE";
   case EMC_TASK_PLAN_CLOSE_TYPE:
      return "EMC_TASK_PLAN_CLOSE_TYPE";
   case EMC_TASK_PLAN_END_TYPE:
      return "EMC_TASK_PLAN_END_TYPE";
   case EMC_TASK_PLAN_EXECUTE_TYPE:
      return "EMC_TASK_PLAN_EXECUTE_TYPE";
   case EMC_TASK_PLAN_INIT_TYPE:
      return "EMC_TASK_PLAN_INIT_TYPE";
   case EMC_TASK_PLAN_OPEN_TYPE:
      return "EMC_TASK_PLAN_OPEN_TYPE";
   case EMC_TASK_PLAN_PAUSE_TYPE:
      return "EMC_TASK_PLAN_PAUSE_TYPE";
   case EMC_TASK_PLAN_READ_TYPE:
      return "EMC_TASK_PLAN_READ_TYPE";
   case EMC_TASK_PLAN_RESUME_TYPE:
      return "EMC_TASK_PLAN_RESUME_TYPE";
   case EMC_TASK_PLAN_RUN_TYPE:
      return "EMC_TASK_PLAN_RUN_TYPE";
   case EMC_TASK_PLAN_STEP_TYPE:
      return "EMC_TASK_PLAN_STEP_TYPE";
   case EMC_TASK_PLAN_SYNCH_TYPE:
      return "EMC_TASK_PLAN_SYNCH_TYPE";
   case EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
      return "EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE";
   case EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
      return "EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE";
   case EMC_TASK_PLAN_OPTIONAL_STOP_TYPE:
      return "EMC_TASK_PLAN_OPTIONAL_STOP_TYPE";
   case EMC_TASK_SET_MODE_TYPE:
      return "EMC_TASK_SET_MODE_TYPE";
   case EMC_TASK_SET_STATE_TYPE:
      return "EMC_TASK_SET_STATE_TYPE";
   case EMC_TASK_STAT_TYPE:
      return "EMC_TASK_STAT_TYPE";
   case EMC_TOOL_ABORT_TYPE:
      return "EMC_TOOL_ABORT_TYPE";
   case EMC_TOOL_HALT_TYPE:
      return "EMC_TOOL_HALT_TYPE";
   case EMC_TOOL_INIT_TYPE:
      return "EMC_TOOL_INIT_TYPE";
   case EMC_TOOL_LOAD_TYPE:
      return "EMC_TOOL_LOAD_TYPE";
   case EMC_TOOL_LOAD_TOOL_TABLE_TYPE:
      return "EMC_TOOL_LOAD_TOOL_TABLE_TYPE";
   case EMC_TOOL_PREPARE_TYPE:
      return "EMC_TOOL_PREPARE_TYPE";
   case EMC_TOOL_SET_OFFSET_TYPE:
      return "EMC_TOOL_SET_OFFSET_TYPE";
   case EMC_TOOL_SET_NUMBER_TYPE:
      return "EMC_TOOL_SET_NUMBER_TYPE";
   case EMC_TOOL_STAT_TYPE:
      return "EMC_TOOL_STAT_TYPE";
   case EMC_TOOL_UNLOAD_TYPE:
      return "EMC_TOOL_UNLOAD_TYPE";
   case EMC_TRAJ_ABORT_TYPE:
      return "EMC_TRAJ_ABORT_TYPE";
   case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
      return "EMC_TRAJ_CIRCULAR_MOVE_TYPE";
   case EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
      return "EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE";
   case EMC_TRAJ_DELAY_TYPE:
      return "EMC_TRAJ_DELAY_TYPE";
   case EMC_TRAJ_DISABLE_TYPE:
      return "EMC_TRAJ_DISABLE_TYPE";
   case EMC_TRAJ_ENABLE_TYPE:
      return "EMC_TRAJ_ENABLE_TYPE";
   case EMC_TRAJ_HALT_TYPE:
      return "EMC_TRAJ_HALT_TYPE";
   case EMC_TRAJ_INIT_TYPE:
      return "EMC_TRAJ_INIT_TYPE";
   case EMC_TRAJ_LINEAR_MOVE_TYPE:
      return "EMC_TRAJ_LINEAR_MOVE_TYPE";
   case EMC_TRAJ_PAUSE_TYPE:
      return "EMC_TRAJ_PAUSE_TYPE";
   case EMC_TRAJ_PROBE_TYPE:
      return "EMC_TRAJ_PROBE_TYPE";
   case EMC_AUX_INPUT_WAIT_TYPE:
      return "EMC_AUX_INPUT_WAIT_TYPE";
   case EMC_TRAJ_RIGID_TAP_TYPE:
      return "EMC_TRAJ_RIGID_TAP_TYPE";
   case EMC_TRAJ_RESUME_TYPE:
      return "EMC_TRAJ_RESUME_TYPE";
   case EMC_TRAJ_SET_ACCELERATION_TYPE:
      return "EMC_TRAJ_SET_ACCELERATION_TYPE";
   case EMC_TRAJ_SET_AXES_TYPE:
      return "EMC_TRAJ_SET_AXES_TYPE";
   case EMC_TRAJ_SET_CYCLE_TIME_TYPE:
      return "EMC_TRAJ_SET_CYCLE_TIME_TYPE";
   case EMC_TRAJ_SET_HOME_TYPE:
      return "EMC_TRAJ_SET_HOME_TYPE";
   case EMC_TRAJ_SET_MAX_ACCELERATION_TYPE:
      return "EMC_TRAJ_SET_MAX_ACCELERATION_TYPE";
   case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
      return "EMC_TRAJ_SET_MAX_VELOCITY_TYPE";
   case EMC_TRAJ_SET_MODE_TYPE:
      return "EMC_TRAJ_SET_MODE_TYPE";
   case EMC_TRAJ_SET_MOTION_ID_TYPE:
      return "EMC_TRAJ_SET_MOTION_ID_TYPE";
   case EMC_TRAJ_SET_OFFSET_TYPE:
      return "EMC_TRAJ_SET_OFFSET_TYPE";
   case EMC_TRAJ_SET_ORIGIN_TYPE:
      return "EMC_TRAJ_SET_ORIGIN_TYPE";
   case EMC_TRAJ_SET_ROTATION_TYPE:
      return "EMC_TRAJ_SET_ROTATION_TYPE";
   case EMC_TRAJ_SET_SCALE_TYPE:
      return "EMC_TRAJ_SET_SCALE_TYPE";
   case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
      return "EMC_TRAJ_SET_SPINDLE_SCALE_TYPE";
   case EMC_TRAJ_SET_FO_ENABLE_TYPE:
      return "EMC_TRAJ_SET_FO_ENABLE_TYPE";
   case EMC_TRAJ_SET_SO_ENABLE_TYPE:
      return "EMC_TRAJ_SET_SO_ENABLE_TYPE";
   case EMC_TRAJ_SET_FH_ENABLE_TYPE:
      return "EMC_TRAJ_SET_FH_ENABLE_TYPE";
   case EMC_TRAJ_SET_TELEOP_ENABLE_TYPE:
      return "EMC_TRAJ_SET_TELEOP_ENABLE_TYPE";
   case EMC_TRAJ_SET_TELEOP_VECTOR_TYPE:
      return "EMC_TRAJ_SET_TELEOP_VECTOR_TYPE";
   case EMC_TRAJ_SET_TERM_COND_TYPE:
      return "EMC_TRAJ_SET_TERM_COND_TYPE";
   case EMC_TRAJ_SET_SPINDLESYNC_TYPE:
      return "EMC_TRAJ_SET_SPINDLESYNC_TYPE";
   case EMC_TRAJ_SET_UNITS_TYPE:
      return "EMC_TRAJ_SET_UNITS_TYPE";
   case EMC_TRAJ_SET_VELOCITY_TYPE:
      return "EMC_TRAJ_SET_VELOCITY_TYPE";
   case EMC_TRAJ_STAT_TYPE:
      return "EMC_TRAJ_STAT_TYPE";
   case EMC_TRAJ_STEP_TYPE:
      return "EMC_TRAJ_STEP_TYPE";
   default:
      return "UNKNOWN";
      break;
   }
   return (NULL);
}

/* Get a message from the message queue, block if no message. */
int get_message(struct emc_session *ps, struct _emc_command_msg_t **message, unsigned int *last, int *lock, const char *tag)
{
   struct list_head *p;
   struct _emc_command_msg_t *em;

   /* Make sure the current thread does not already own the lock. */
   if (*lock == 0)
   {
      pthread_mutex_lock(&ps->mutex);
      *lock = 1;
   }

   while (1)
   {
      /* Walk the queue looking for message > "last" sequence number. */
      list_for_each(p, &ps->head.list)
      {
         em = list_entry(p, struct _emc_command_msg_t, msg.list);
         if (em->msg.n > *last)
         {
            DBG("[%s] found message id=%s n=%d\n", tag, lookup_message(em->msg.type), em->msg.n);
            *last = em->msg.n;
            *message = em;
            return 0;   /* found message > time, return with lock set */
         }
      }

      /* Found no interesting messages, wait for new one(s). Reset "last" sequence check. */
      DBG("[%s] waiting for message...\n", tag);
      *last = 0;
      pthread_cond_wait(&ps->event_cond, &ps->mutex);
   }
}       /* get_message */

/* Get a message from the message queue, return if no message. */
int peek_message(struct emc_session *ps, struct _emc_command_msg_t **message, unsigned int *last, int *lock, const char *tag)
{
   struct list_head *p;
   struct _emc_command_msg_t *em;

   /* Make sure the current thread does not already own the lock. */
   if (*lock == 0)
   {
      pthread_mutex_lock(&ps->mutex);
      *lock = 1;
   }

   *message = NULL;

   /* Walk the queue looking for message > "last" sequence number. */
   list_for_each(p, &ps->head.list)
   {
      em = list_entry(p, struct _emc_command_msg_t, msg.list);
      if (em->msg.n > *last)
      {
         DBG("[%s] found message id=%s n=%d\n", tag, lookup_message(em->msg.type), em->msg.n);
         *last = em->msg.n;
         *message = em;
         return 0;      /* found message > time, return with lock set */
      }
   }

   /* Found no interesting messages. Reset "last" sequence check. */
   *lock = *last = 0;
   pthread_mutex_unlock(&ps->mutex);

   return 1;
}       /* peek_message */

/* Remove specified message from the message queue. Must be preceded by get_message. */
int remove_message(struct emc_session *ps, struct _emc_command_msg_t *message, int *lock, const char *tag)
{
   DBG("[%s] removing message id=%s n=%d\n", tag, lookup_message(message->msg.type), message->msg.n);
   list_del(&message->msg.list);
   *lock = 0;
   pthread_mutex_unlock(&ps->mutex);
   return 0;
}       /* remove_message */

/* Add specified message to the message queue. */
int post_message(struct emc_session *ps, struct _emc_command_msg_t *message, const char *tag)
{
   pthread_mutex_lock(&ps->mutex);

   message->msg.n = seq_num++;
   if (seq_num == 0)
      seq_num = 1;      /* wrapped, don't use zero */

   DBG("[%s] posting message id=%s n=%d\n", tag, lookup_message(message->msg.type), message->msg.n);
   list_add_tail(&message->msg.list, &ps->head.list);
   pthread_cond_broadcast(&ps->event_cond);     /* signal new message */
   pthread_mutex_unlock(&ps->mutex);
   return 0;
}       /* post_message */

int send_message(struct emc_session *ps, emc_command_msg_t * message, const char *tag)
{
   struct _emc_command_msg_t *em;

   if ((em = (struct _emc_command_msg_t *) malloc(sizeof(struct _emc_command_msg_t))) == NULL)
   {
      BUG("send_message: malloc error\n");
      return 1;
   }
   else
   {
      memcpy(em, message, sizeof(struct _emc_command_msg_t));
      post_message(ps, em, tag);
      return 0;
   }
}

int dump_message(struct emc_session *ps)
{
   struct list_head *p;
   struct _emc_command_msg_t *em;
   int i = 0;
   list_for_each(p, &ps->head.list)
   {
      i++;
      em = list_entry(p, struct _emc_command_msg_t, msg.list);
      BUG("queue message %d id=%s n=%d\n", i, lookup_message(em->msg.type), em->msg.n);
   }
   if (i == 0)
      BUG("queue message cnt=0\n");
   return 0;
}       /* dump_message */
