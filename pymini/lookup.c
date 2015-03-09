/************************************************************************************\

  lookup.c - helper for displaying debug messages

  (c) 2008-2015 Copyright Eckler Software

  Author: David Suffield, dsuffiel@ecklersoft.com

\************************************************************************************/

#include <string.h>
#include "emc.h"

#if 0
const char *lookup_task_interp_state(int type)
{
   switch (type)
   {
      case EMC_TASK_INTERP_UNUSED:
         return "EMC_TASK_INTERP_UNUSED";
      case EMC_TASK_INTERP_IDLE:
         return "EMC_TASK_INTERP_IDLE";
      case EMC_TASK_INTERP_READING:
         return "EMC_TASK_INTERP_READING";
      case EMC_TASK_INTERP_PAUSED:
         return "EMC_TASK_INTERP_PAUSED";
      case EMC_TASK_INTERP_WAITING:
         return "EMC_TASK_INTERP_WAITING";
      default:
         return "UNKNOWN";
         break;
   }
   return (NULL);
}
#endif

const char *lookup_message(int type)
{
   switch (type)
   {
   case EMC_TRAJ_LINEAR_MOVE_TYPE:
      return "EMC_TRAJ_LINEAR_MOVE_TYPE";
   case EMC_TRAJ_SET_TERM_COND_TYPE:
      return "EMC_TRAJ_SET_TERM_COND_TYPE";
   case EMC_TRAJ_CIRCULAR_MOVE_TYPE:
      return "EMC_TRAJ_CIRCULAR_MOVE_TYPE";
   case EMC_TASK_PLAN_PAUSE_TYPE:
      return "EMC_TASK_PLAN_PAUSE_TYPE";
   case EMC_TASK_PLAN_END_TYPE:
      return "EMC_TASK_PLAN_END_TYPE";
   case EMC_SYSTEM_CMD_TYPE:
      return "EMC_SYSTEM_CMD_TYPE";
   case EMC_TRAJ_DELAY_TYPE:
      return "EMC_TRAJ_DELAY_TYPE";
   default:
      return "UNKNOWN";
      break;
   }
   return (NULL);
}
