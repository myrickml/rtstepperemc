/********************************************************************
* motctl.c - motion controller support for trajectory planner.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* License: GPL Version 2
*
* Copyright (c) 2004 All rights reserved.
*
* Re-purposed for rt-stepper dongle.
*
* Author: David Suffield, dsuffiel@ecklersoft.com
* (c) 2011-2015 Copyright Eckler Software
*
********************************************************************/
#include <math.h>
#include "emc.h"
#include "bug.h"

static void _compute_axis(struct emc_session *ps, struct emc_axis *axis)
{
   double a_max, v_max, v, s_to_go, ds_stop, ds_vel, ds_acc, dv_acc;

   /* determine which way the compensation should be applied */
   if (axis->vel_cmd > 0.0)
   {
      /* moving "up". apply positive backlash comp */
      axis->backlash_corr = 0.5 * axis->backlash;
   }
   else if (axis->vel_cmd < 0.0)
   {
      /* moving "down". apply negative backlash comp */
      axis->backlash_corr = -0.5 * axis->backlash;
   }
   else
   {
      /* not moving, use whatever was there before */
   }

   /* at this point, the correction has been computed, but
      the value may make abrupt jumps on direction reversal */
   /*
    * 07/09/2005 - S-curve implementation by Bas Laarhoven
    *
    * Implementation:
    *   Generate a ramped velocity profile for backlash or screw error comp.
    *   The velocity is ramped up to the maximum speed setting (if possible),
    *   using the maximum acceleration setting.
    *   At the end, the speed is ramped dowm using the same acceleration.
    *   The algorithm keeps looking ahead. Depending on the distance to go,
    *   the speed is increased, kept constant or decreased.
    *   
    * Limitations:
    *   Since the compensation adds up to the normal movement, total
    *   accelleration and total velocity may exceed maximum settings!
    *   Currently this is limited to 150% by implementation.
    *   To fix this, the calculations in get_pos_cmd should include
    *   information from the backlash corection. This makes things
    *   rather complicated and it might be better to implement the
    *   backlash compensation at another place to prevent this kind
    *   of interaction.
    *   More testing under different circumstances will show if this
    *   needs a more complicate solution.
    *   For now this implementation seems to generate smoother
    *   movements and less following errors than the original code.
    */

   /* Limit maximum accelleration and velocity 'overshoot'
    * to 150% of the maximum settings.
    * The TP and backlash shouldn't use more than 100%
    * (together) but this requires some interaction that
    * isn't implemented yet.
    */

   /* Changed following from 50% to 105% to accommodate pymini's TP. Otherwise we will not hit the "last step targets". David Suffield 1-30-2015 */
   v_max = 1.05 * axis->max_velocity;
   a_max = 1.05 * axis->max_acceleration;
   v = axis->backlash_vel;
   if (axis->backlash_corr >= axis->backlash_filt)
   {
      s_to_go = axis->backlash_corr - axis->backlash_filt; /* abs val */
      if (s_to_go > 0)
      {
	 // off target, need to move
	 ds_vel = v * ps->cycle_time;  /* abs val */
	 dv_acc = a_max * ps->cycle_time;      /* abs val */
	 ds_stop = 0.5 * (v + dv_acc) * (v + dv_acc) / a_max;        /* abs val */
         ///DBG("s_to_go=%0.9f, ds_vel=%0.9f, dv_acc=%0.9f, ds_stop=%0.9f\n", s_to_go, ds_vel, dv_acc, ds_stop);
	 if (s_to_go <= ds_stop + ds_vel)
	 {
	    // ramp down
	    if (v > dv_acc)
	    {
	       // decellerate one period
	       ds_acc = 0.5 * dv_acc * ps->cycle_time; /* abs val */
	       axis->backlash_vel -= dv_acc;
	       axis->backlash_filt += ds_vel - ds_acc;
	    }
	    else
	    {
	       // last step to target
	       axis->backlash_vel = 0.0;
	       axis->backlash_filt = axis->backlash_corr;
               //DBG("last step to target\n");
	    }
	 }
	 else
	 {
	    if (v + dv_acc > v_max)
	    {
	       dv_acc = v_max - v;   /* abs val */
	    }
	    ds_acc = 0.5 * dv_acc * ps->cycle_time;    /* abs val */
	    ds_stop = 0.5 * (v + dv_acc) * (v + dv_acc) / a_max;     /* abs val */
	    if (s_to_go > ds_stop + ds_vel + ds_acc)
	    {
	       // ramp up
	       axis->backlash_vel += dv_acc;
	       axis->backlash_filt += ds_vel + ds_acc;
	    }
	    else
	    {
	       // constant velocity
	       axis->backlash_filt += ds_vel;
               //DBG("constant vel\n");
	    }
	 }
      }
      else if (s_to_go < 0)
      {
	 // safely handle overshoot (should not occur)
	 axis->backlash_vel = 0.0;
	 axis->backlash_filt = axis->backlash_corr;
      }
   }
   else
   { /* axis->backlash_corr < 0.0 */
      s_to_go = axis->backlash_filt - axis->backlash_corr; /* abs val */
      if (s_to_go > 0)
      {
	 // off target, need to move
	 ds_vel = -v * ps->cycle_time; /* abs val */
	 dv_acc = a_max * ps->cycle_time;      /* abs val */
	 ds_stop = 0.5 * (v - dv_acc) * (v - dv_acc) / a_max;        /* abs val */
         //DBG("s_to_go=%0.9f, ds_vel=%0.9f, dv_acc=%0.9f, ds_stop=%0.9f\n", s_to_go, ds_vel, dv_acc, ds_stop);
	 if (s_to_go <= ds_stop + ds_vel)
	 {
	    // ramp down
	    if (-v > dv_acc)
	    {
	       // decellerate one period
	       ds_acc = 0.5 * dv_acc * ps->cycle_time; /* abs val */
	       axis->backlash_vel += dv_acc;        /* decrease */
	       axis->backlash_filt -= ds_vel - ds_acc;
	    }
	    else
	    {
	       // last step to target
	       axis->backlash_vel = 0.0;
	       axis->backlash_filt = axis->backlash_corr;
               //DBG("last step to target\n");
	    }
	 }
	 else
	 {
	    if (-v + dv_acc > v_max)
	    {
	       dv_acc = v_max + v;   /* abs val */
	    }
	    ds_acc = 0.5 * dv_acc * ps->cycle_time;    /* abs val */
	    ds_stop = 0.5 * (v - dv_acc) * (v - dv_acc) / a_max;     /* abs val */
	    if (s_to_go > ds_stop + ds_vel + ds_acc)
	    {
	       // ramp up
	       axis->backlash_vel -= dv_acc;        /* increase */
	       axis->backlash_filt -= ds_vel + ds_acc;
	    }
	    else
	    {
	       // constant velocity
	       axis->backlash_filt -= ds_vel;
               //DBG("constant vel\n");
	    }
	 }
      }
      else if (s_to_go < 0)
      {
	 // safely handle overshoot (should not occur)
	 axis->backlash_vel = 0.0;
	 axis->backlash_filt = axis->backlash_corr;
      }
   }
}  /* _compute_axis() */

/* Calculate leadscrew compensation (backlash). */
void compute_screw_comp(struct emc_session *ps)
{
   int i;

   for (i=0; i < ps->axes; i++)
   {
      _compute_axis(ps, &ps->axis[i]);
   }
}       /* compute_screw_comp() */

void reset_screw_comp(struct emc_session *ps)
{
   int i;

   /* After estop compensation variables can be in bad state. Reset them. */
   for (i=0; i < ps->axes; i++)
   {
      ps->axis[i].backlash_vel = 0.0;
      ps->axis[i].backlash_filt = 0.0;
   }
}

#if 0
/* Convert EmcPose structure to array. */
void emcpos2a(double *a, EmcPose pos)
{
   a[EMC_AXIS_X] = pos.tran.x;
   a[EMC_AXIS_Y] = pos.tran.y;
   a[EMC_AXIS_Z] = pos.tran.z;
   a[EMC_AXIS_A] = pos.a;
   a[EMC_AXIS_B] = pos.b;
   a[EMC_AXIS_C] = pos.c;
   a[EMC_AXIS_U] = pos.u;
   a[EMC_AXIS_V] = pos.v;
   a[EMC_AXIS_W] = pos.w;
}
#endif

void update_tp_position(struct emc_session *ps, EmcPose pos)
{
   double old_pos_cmd[EMC_MAX_AXIS];
   int i;

   for (i=0; i < EMC_MAX_AXIS; i++)
      old_pos_cmd[i] = ps->axis[i].pos_cmd;

   ps->axis[EMC_AXIS_X].pos_cmd = pos.tran.x;
   ps->axis[EMC_AXIS_Y].pos_cmd = pos.tran.y;
   ps->axis[EMC_AXIS_Z].pos_cmd = pos.tran.z;
   ps->axis[EMC_AXIS_A].pos_cmd = pos.a;
   ps->axis[EMC_AXIS_B].pos_cmd = pos.b;
   ps->axis[EMC_AXIS_C].pos_cmd = pos.c;
   ps->axis[EMC_AXIS_U].pos_cmd = pos.u;
   ps->axis[EMC_AXIS_V].pos_cmd = pos.v;
   ps->axis[EMC_AXIS_W].pos_cmd = pos.w;

   for (i=0; i < EMC_MAX_AXIS; i++)
      ps->axis[i].vel_cmd = (ps->axis[i].pos_cmd - old_pos_cmd[i]) * ps->cycle_freq;
}
