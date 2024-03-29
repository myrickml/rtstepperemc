/********************************************************************
* Description: interp_convert.cc
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "rs274ngc.h"
#include "rs274ngc_return.h"
#include "rs274ngc_interp.h"
#include "interp_internal.h"
#include "interp_queue.h"

#include "units.h"
#define TOOL_INSIDE_ARC(side, turn) (((side)==LEFT&&(turn)==1)||((side)==RIGHT&&(turn)==-1))

//#define DEBUG_EMC


// These four functions help make the rest of cutter comp
// plane-agnostic in much the same way the ARC_FEED canon call is.
// The programmer can gleefully think of only the XY plane when
// reading convert_[straight|arc]_comp[1|2].
//
// Because the STRAIGHT_[FEED|TRAVERSE] canon calls are not
// plane-agnostic, the opposite plane conversion happens in
// enqueue_STRAIGHT_[FEED|TRAVERSE] when adding to the interp queue.

int Interp::comp_get_current(setup_pointer settings, double *x, double *y, double *z) {
    switch(settings->plane) {
    case CANON_PLANE_XY:
        *x = settings->current_x;
        *y = settings->current_y;
        *z = settings->current_z;
        break;
    case CANON_PLANE_XZ:
        *x = settings->current_z;
        *y = settings->current_x;
        *z = settings->current_y;
        break;
    default:
        ERS("BUG: Invalid plane in comp_get_current");
    }
    return INTERP_OK;
}

int Interp::comp_set_current(setup_pointer settings, double x, double y, double z) {
    switch(settings->plane) {
    case CANON_PLANE_XY:
        settings->current_x = x;
        settings->current_y = y;
        settings->current_z = z;
        break;
    case CANON_PLANE_XZ:
        settings->current_x = y;
        settings->current_y = z;
        settings->current_z = x;
        break;
    default:
        ERS("BUG: Invalid plane in comp_set_current");
    }
    return INTERP_OK;
}

int Interp::comp_get_programmed(setup_pointer settings, double *x, double *y, double *z) {
    switch(settings->plane) {
    case CANON_PLANE_XY:
        *x = settings->program_x;
        *y = settings->program_y;
        *z = settings->program_z;
        break;
    case CANON_PLANE_XZ:
        *x = settings->program_z;
        *y = settings->program_x;
        *z = settings->program_y;
        break;
    default:
        ERS("BUG: Invalid plane in comp_get_programmed");
    }
    return INTERP_OK;
}

int Interp::comp_set_programmed(setup_pointer settings, double x, double y, double z) {
    switch(settings->plane) {
    case CANON_PLANE_XY:
        settings->program_x = x;
        settings->program_y = y;
        settings->program_z = z;
        break;
    case CANON_PLANE_XZ:
        settings->program_x = y;
        settings->program_y = z;
        settings->program_z = x;
        break;
    default:
        ERS("BUG: Invalid plane in comp_set_programmed");
    }
    return INTERP_OK;
}

/****************************************************************************/

/*! convert_nurbs
 *
 * Returned value: int
 * Returns a rs274ngc error code, or INTERP_OK if everything is OK.
 *
 * Side effects: Generates a nurbs move and updates the position of the tool
 */

static unsigned int nurbs_order;
static std::vector<CONTROL_POINT> nurbs_control_points;

int Interp::convert_nurbs(int mode,
      block_pointer block,     //!< pointer to a block of RS274 instructions
      setup_pointer settings)  //!< pointer to machine settings
{
    double end_z, AA_end, BB_end, CC_end, u_end, v_end, w_end;
    CONTROL_POINT CP;   

    if (mode == G_5_2)  {
	CHKS((((block->x_flag) && !(block->y_flag)) || (!(block->x_flag) && (block->y_flag))), (
             "You must specify both X and Y coordinates for Control Points"));
	CHKS((!(block->x_flag) && !(block->y_flag) && (block->p_number > 0) && 
             (!nurbs_control_points.empty())), (
             "Can specify P without X and Y only for the first control point"));

        CHKS(((block->p_number <= 0) && (!nurbs_control_points.empty())), (
             "Must specify positive weight P for every Control Point"));
        if (settings->feed_mode == UNITS_PER_MINUTE) {
            CHKS((settings->feed_rate == 0.0), (
                 "Cannot make a NURBS with 0 feedrate"));
        }
        if (nurbs_control_points.empty()) {
            CP.X = settings->current_x;
            CP.Y = settings->current_y;
            if (!(block->x_flag) && !(block->y_flag) && (block->p_number > 0)) {
                CP.W = block->p_number;
            } else {
                CP.W = 1;
            }
            nurbs_order = 3;
            nurbs_control_points.push_back(CP);
        } 
        if (block->l_number != -1 && block->l_number > 3) {
            nurbs_order = block->l_number;  
        } 
        if ((block->x_flag) && (block->y_flag)) {
            CHP(find_ends(block, settings, &CP.X, &CP.Y, &end_z, &AA_end, &BB_end, &CC_end,
                          &u_end, &v_end, &w_end));
            CP.W = block->p_number;
            nurbs_control_points.push_back(CP);
            }

//for (i=0;i<nurbs_control_points.size();i++){
//                printf( "X %8.4f, Y %8.4f, W %8.4f\n",
//              nurbs_control_points[i].X,
//               nurbs_control_points[i].Y,
//               nurbs_control_points[i].W);
//       }
//        printf("*-----------------------------------------*\n");
        settings->motion_mode = mode;
    }
    
    else if (mode == G_5_3){
        CHKS((settings->motion_mode != G_5_2), (
             "Cannot use G5.3 without G5.2 first"));
        CHKS((nurbs_control_points.size()<nurbs_order), EMC_I18N("You must specify a number of control points at least equal to the order L = %d"), nurbs_order);
	settings->current_x = nurbs_control_points[nurbs_control_points.size()-1].X;
        settings->current_y = nurbs_control_points[nurbs_control_points.size()-1].Y;
        NURBS_FEED(block->line_number, nurbs_control_points, nurbs_order);
	//printf("hello\n");
	nurbs_control_points.clear();
	//printf("%d\n", 	nurbs_control_points.size());
	settings->motion_mode = -1;
    }
    return INTERP_OK;
}

 /****************************************************************************/

/*! convert_spline
 *
 * Returned value: int
 * Returns a rs274ngc error code, or INTERP_OK if everything is OK.
 *
 * Side effects: Generates a spline move and updates the position of the tool
 */
int Interp::convert_spline(int mode,
       block_pointer block,     //!< pointer to a block of RS274 instructions
       setup_pointer settings)  //!< pointer to machine settings
{
    double x1, y1, x2, y2, x3, y3;
    double end_z, AA_end, BB_end, CC_end, u_end, v_end, w_end;
    CONTROL_POINT cp;

    CHKS((settings->cutter_comp_side != OFF), EMC_I18N("Cannot convert spline with cutter radius compensation")); // XXX

    if (settings->feed_mode == UNITS_PER_MINUTE) {
      CHKS((settings->feed_rate == 0.0),
        NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE);
    } else if (settings->feed_mode == INVERSE_TIME) {
      CHKS((block->f_number == -1.0),
        NCE_F_WORD_MISSING_WITH_INVERSE_TIME_ARC_MOVE);
    }

    CHKS((settings->plane != CANON_PLANE_XY), EMC_I18N("Splines must be in the XY plane")); // XXX
       //Error (for now): Splines must be in XY plane

    CHKS((block->z_flag == ON || block->a_flag == ON || block->b_flag == ON
          || block->c_flag == ON),
          ("Splines may not have motion in Z, A, B, or C"));

    if(mode == G_5_1) {
      CHKS(block->i_flag != ON || block->j_flag != ON,
                  ("Must specify both I and J with G5.1"));
      x1 = settings->current_x + block->i_number;
      y1 = settings->current_y + block->j_number;
      CHP(find_ends(block, settings, &x2, &y2, &end_z, &AA_end, &BB_end, &CC_end,
                    &u_end, &v_end, &w_end));
      cp.W = 1;
      cp.X = settings->current_x, cp.Y = settings->current_y;
      nurbs_control_points.push_back(cp);
      cp.X = x1, cp.Y = y1;
      nurbs_control_points.push_back(cp);
      cp.X = x2, cp.Y = y2;
      nurbs_control_points.push_back(cp);
      NURBS_FEED(block->line_number, nurbs_control_points, 3);
      nurbs_control_points.clear();
      settings->current_x = x2;
      settings->current_y = y2;
    } else {
      if(block->i_flag != ON || block->j_flag != ON) {
          CHKS(block->i_flag == ON || block->j_flag == ON,
                  ("Must specify both I and J, or neither"));
          x1 = settings->current_x + settings->cycle_i;
          y1 = settings->current_y + settings->cycle_j;
      } else {
          x1 = settings->current_x + block->i_number;
          y1 = settings->current_y + block->j_number;
      }
      CHP(find_ends(block, settings, &x3, &y3, &end_z, &AA_end, &BB_end, &CC_end,
                    &u_end, &v_end, &w_end));

      CHKS(block->p_flag != ON || block->q_flag != ON,
	      ("Must specify both P and Q with G5"));
      x2 = x3 + block->p_number;
      y2 = y3 + block->q_number;

      cp.W = 1;
      cp.X = settings->current_x, cp.Y = settings->current_y;
      nurbs_control_points.push_back(cp);
      cp.X = x1, cp.Y = y1;
      nurbs_control_points.push_back(cp);
      cp.X = x2, cp.Y = y2;
      nurbs_control_points.push_back(cp);
      cp.X = x3, cp.Y = y3;
      nurbs_control_points.push_back(cp);
      NURBS_FEED(block->line_number, nurbs_control_points, 4);
      nurbs_control_points.clear();

      settings->cycle_i = -block->p_number;
      settings->cycle_j = -block->q_number;
      settings->current_x = x3;
      settings->current_y = y3;
    }
    return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc

Returned Value: int
   If one of the following functions returns an error code,
   this returns that error code.
      convert_arc_comp1
      convert_arc_comp2
      convert_arc2
   If any of the following errors occur, this returns the error code shown.
   Otherwise, this returns INTERP_OK.
   1. The block has neither an r value nor any i,j,k values:
      NCE_R_I_J_K_WORDS_ALL_MISSING_FOR_ARC
   2. The block has both an r value and one or more i,j,k values:
      NCE_MIXED_RADIUS_IJK_FORMAT_FOR_ARC
   3. In the ijk format the XY-plane is selected and
      the block has a k value: NCE_K_WORD_GIVEN_FOR_ARC_IN_XY_PLANE
   4. In the ijk format the YZ-plane is selected and
      the block has an i value: NCE_I_WORD_GIVEN_FOR_ARC_IN_YZ_PLANE
   5. In the ijk format the XZ-plane is selected and
      the block has a j value: NCE_J_WORD_GIVEN_FOR_ARC_IN_XZ_PLANE
   6. In either format any of the following occurs.
      a. The XY-plane is selected and the block has no x or y value:
         NCE_X_AND_Y_WORDS_MISSING_FOR_ARC_IN_XY_PLANE
      b. The YZ-plane is selected and the block has no y or z value:
         NCE_Y_AND_Z_WORDS_MISSING_FOR_ARC_IN_YZ_PLANE
      c. The ZX-plane is selected and the block has no z or x value:
         NCE_X_AND_Z_WORDS_MISSING_FOR_ARC_IN_XZ_PLANE
   7. The selected plane is an unknown plane:
      NCE_BUG_PLANE_NOT_XY_YZ__OR_XZ
   8. The feed rate mode is UNITS_PER_MINUTE and feed rate is zero:
      NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE
   9. The feed rate mode is INVERSE_TIME and the block has no f word:
      NCE_F_WORD_MISSING_WITH_INVERSE_TIME_ARC_MOVE

Side effects:
   This generates and executes an arc command at feed rate
   (and, possibly a second arc command). It also updates the setting
   of the position of the tool point to the end point of the move.

Called by: convert_motion.

This converts a helical or circular arc.  The function calls:
convert_arc2 (when cutter radius compensation is off) or
convert_arc_comp1 (when cutter comp is on and this is the first move) or
convert_arc_comp2 (when cutter comp is on and this is not the first move).

If the ijk format is used, at least one of the offsets in the current
plane must be given in the block; it is common but not required to
give both offsets. The offsets are always incremental [NCMS, page 21].

If cutter compensation is in use, the path's length may increase or
decrease.  Also an arc may be added, to go around a corner, before the
original arc move.  For the purpose of calculating the feed rate when in
inverse time mode, this length increase or decrease is ignored.  The
feed is still set to the original programmed arc length divided by the F
number (with the above lower bound).  The new arc (if needed) and the
new longer or shorter original arc are taken at this feed.

*/

int Interp::convert_arc(int move,        //!< either G_2 (cw arc) or G_3 (ccw arc)    
                       block_pointer block,     //!< pointer to a block of RS274 instructions
                       setup_pointer settings)  //!< pointer to machine settings             
{
  int status;
  int first;                    /* flag set ON if this is first move after comp ON */
  int ijk_flag;                 /* flag set ON if any of i,j,k present in NC code  */
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end, v_end, w_end;

  CHKS((settings->arc_not_allowed), (EMC_I18N("The move just after exiting cutter compensation mode must be straight, not an arc")));

  ijk_flag = ((block->i_flag || block->j_flag) || block->k_flag) ? ON : OFF;
  first = settings->cutter_comp_firstmove == ON;

  CHKS(((block->r_flag != ON) && (ijk_flag != ON)),
      NCE_R_I_J_K_WORDS_ALL_MISSING_FOR_ARC);
  CHKS(((block->r_flag == ON) && (ijk_flag == ON)),
      NCE_MIXED_RADIUS_IJK_FORMAT_FOR_ARC);
  if (settings->feed_mode == UNITS_PER_MINUTE) {
    CHKS((settings->feed_rate == 0.0),
        NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE);
  } else if(settings->feed_mode == UNITS_PER_REVOLUTION) {
    CHKS((settings->feed_rate == 0.0),
        NCE_CANNOT_MAKE_ARC_WITH_ZERO_FEED_RATE);
    CHKS((settings->speed == 0.0),
	"Cannot feed with zero spindle speed in feed per rev mode");
  } else if (settings->feed_mode == INVERSE_TIME) {
    CHKS((block->f_number == -1.0),
        NCE_F_WORD_MISSING_WITH_INVERSE_TIME_ARC_MOVE);
  }

  if (ijk_flag) {
    if (settings->plane == CANON_PLANE_XY) {
      CHKS((block->k_flag), NCE_K_WORD_GIVEN_FOR_ARC_IN_XY_PLANE);
      if (block->i_flag == OFF) { /* i or j flag on to get here */
	if (settings->ijk_distance_mode == MODE_ABSOLUTE) {
	  ERS("%c word missing in absolute center arc", 'I');
	} else {
	  block->i_number = 0.0;
	}
      } else if (block->j_flag == OFF) {
	if (settings->ijk_distance_mode == MODE_ABSOLUTE) {
	  ERS("%c word missing in absolute center arc", 'J');
	} else {
	  block->j_number = 0.0;
	}
      }
    } else if (settings->plane == CANON_PLANE_YZ) {
      CHKS((block->i_flag), NCE_I_WORD_GIVEN_FOR_ARC_IN_YZ_PLANE);
      if (block->j_flag == OFF) { /* j or k flag on to get here */
	if (settings->ijk_distance_mode == MODE_ABSOLUTE) {
	  ERS("%c word missing in absolute center arc", 'J');
	} else {
	  block->j_number = 0.0;
	}
      } else if (block->k_flag == OFF) {
	if (settings->ijk_distance_mode == MODE_ABSOLUTE) {
	  ERS("%c word missing in absolute center arc", 'K');
	} else {
	  block->k_number = 0.0;
	}
      }
    } else if (settings->plane == CANON_PLANE_XZ) {
      CHKS((block->j_flag), NCE_J_WORD_GIVEN_FOR_ARC_IN_XZ_PLANE);
      if (block->i_flag == OFF) { /* i or k flag on to get here */
	if (settings->ijk_distance_mode == MODE_ABSOLUTE) {
	  ERS("%c word missing in absolute center arc", 'I');
	} else {
	  block->i_number = 0.0;
	}
      } else if (block->k_flag == OFF) {
	if (settings->ijk_distance_mode == MODE_ABSOLUTE) {
	  ERS("%c word missing in absolute center arc", 'K');
	} else {
	  block->k_number = 0.0;
	}
      }
    } else {
      ERS(NCE_BUG_PLANE_NOT_XY_YZ_OR_XZ);
    }
  } else {
    // in R format, we need some XYZ words specified because a full circle is not allowed.
    if (settings->plane == CANON_PLANE_XY) { 
        CHKS(((block->x_flag == OFF) && (block->y_flag == OFF) && (block->radius_flag == OFF) && (block->theta_flag == OFF)),
            NCE_X_AND_Y_WORDS_MISSING_FOR_ARC_IN_XY_PLANE);
    } else if (settings->plane == CANON_PLANE_YZ) {
        CHKS(((block->y_flag == OFF) && (block->z_flag == OFF)),
            NCE_Y_AND_Z_WORDS_MISSING_FOR_ARC_IN_YZ_PLANE);
    } else if (settings->plane == CANON_PLANE_XZ) {
        CHKS(((block->x_flag == OFF) && (block->z_flag == OFF)),
            NCE_X_AND_Z_WORDS_MISSING_FOR_ARC_IN_XZ_PLANE);
    }
  }


  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end, 
                &u_end, &v_end, &w_end));

  settings->motion_mode = move;


  if (settings->plane == CANON_PLANE_XY) {
    if ((settings->cutter_comp_side == OFF) ||
        (settings->cutter_comp_radius == 0.0)) {
      status =
        convert_arc2(move, block, settings,
                     &(settings->current_x), &(settings->current_y),
                     &(settings->current_z), end_x, end_y, end_z,
                     AA_end, BB_end, CC_end,
                     u_end, v_end, w_end,
                     block->i_number, block->j_number);
      CHP(status);
    } else if (first) {
      status = convert_arc_comp1(move, block, settings, end_x, end_y, end_z,
                                 block->i_number, block->j_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
      CHP(status);
    } else {
      status = convert_arc_comp2(move, block, settings, end_x, end_y, end_z,
                                 block->i_number, block->j_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
      CHP(status);
    }
  } else if (settings->plane == CANON_PLANE_XZ) {
    if ((settings->cutter_comp_side == OFF) ||
        (settings->cutter_comp_radius == 0.0)) {
      status =
        convert_arc2(move, block, settings,
                     &(settings->current_z), &(settings->current_x),
                     &(settings->current_y), end_z, end_x, end_y,
                     AA_end, BB_end, CC_end,
                     u_end, v_end, w_end,
                     block->k_number, block->i_number);
      CHP(status);
    } else if (first) {
      status = convert_arc_comp1(move, block, settings, end_z, end_x, end_y,
                                 block->k_number, block->i_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
      CHP(status);
    } else {
      status = convert_arc_comp2(move, block, settings, end_z, end_x, end_y,
                                 block->k_number, block->i_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);

      CHP(status);
    }
  } else if (settings->plane == CANON_PLANE_YZ) {
    status =
      convert_arc2(move, block, settings,
                   &(settings->current_y), &(settings->current_z),
                   &(settings->current_x), end_y, end_z, end_x,
                   AA_end, BB_end, CC_end,
                   u_end, v_end, w_end,
                   block->j_number, block->k_number);
    CHP(status);
  } else
    ERS(NCE_BUG_PLANE_NOT_XY_YZ_OR_XZ);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc2

Returned Value: int
   If arc_data_ijk or arc_data_r returns an error code,
   this returns that code.
   Otherwise, it returns INTERP_OK.

Side effects:
   This executes an arc command at feed rate. It also updates the
   setting of the position of the tool point to the end point of the move.

Called by: convert_arc.

This converts a helical or circular arc.

*/

int Interp::convert_arc2(int move,       //!< either G_2 (cw arc) or G_3 (ccw arc)    
                        block_pointer block,    //!< pointer to a block of RS274 instructions
                        setup_pointer settings, //!< pointer to machine settings             
                        double *current1,       //!< pointer to current value of coordinate 1
                        double *current2,       //!< pointer to current value of coordinate 2
                        double *current3,       //!< pointer to current value of coordinate 3
                        double end1,    //!< coordinate 1 value at end of arc        
                        double end2,    //!< coordinate 2 value at end of arc        
                        double end3,    //!< coordinate 3 value at end of arc        
                        double AA_end,  //!< a-value at end of arc                   
                        double BB_end,  //!< b-value at end of arc                   
                        double CC_end,  //!< c-value at end of arc                   
                         double u, double v, double w, //!< values at end of arc
                        double offset1, //!< center, either abs or offset from current
                        double offset2)
{
  double center1;
  double center2;
  double tolerance;             /* tolerance for difference of radii          */
  int turn;                     /* number of full or partial turns CCW in arc */
  int plane = settings->plane;

  tolerance = (settings->length_units == CANON_UNITS_INCHES) ?
    TOLERANCE_INCH : TOLERANCE_MM;

  if (block->r_flag) {
      CHP(arc_data_r(move, plane, *current1, *current2, end1, end2,
                   block->r_number, &center1, &center2, &turn, tolerance));
  } else {
      CHP(arc_data_ijk(move, plane, *current1, *current2, end1, end2,
                       (settings->ijk_distance_mode == MODE_ABSOLUTE),
                       offset1, offset2,
                       &center1, &center2, &turn, tolerance));
  }
  inverse_time_rate_arc(*current1, *current2, *current3, center1, center2,
                        turn, end1, end2, end3, block, settings);

  ARC_FEED(block->line_number, end1, end2, center1, center2, turn, end3,
           AA_end, BB_end, CC_end, u, v, w);
  *current1 = end1;
  *current2 = end2;
  *current3 = end3;
  settings->AA_current = AA_end;
  settings->BB_current = BB_end;
  settings->CC_current = CC_end;
  settings->u_current = u;
  settings->v_current = v;
  settings->w_current = w;
  
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc_comp1

Returned Value: int
   If arc_data_comp_ijk or arc_data_comp_r returns an error code,
   this returns that code.
   Otherwise, it returns INTERP_OK.

Side effects:
   This executes an arc command at
   feed rate. It also updates the setting of the position of
   the tool point to the end point of the move.

Called by: convert_arc.

This function converts a helical or circular arc, generating only one
arc. This is called when cutter radius compensation is on and this is 
the first cut after the turning on.

The arc which is generated is derived from a second arc which passes
through the programmed end point and is tangent to the cutter at its
current location. The generated arc moves the tool so that it stays
tangent to the second arc throughout the move.

*/

int Interp::convert_arc_comp1(int move,  //!< either G_2 (cw arc) or G_3 (ccw arc)            
                              block_pointer block,       //!< pointer to a block of RS274/NGC instructions    
                              setup_pointer settings,    //!< pointer to machine settings                     
                              double end_x,      //!< x-value at end of programmed (then actual) arc  
                              double end_y,      //!< y-value at end of programmed (then actual) arc  
                              double end_z,      //!< z-value at end of arc                           
                              double offset_x, double offset_y, 
                              double AA_end,     //!< a-value at end of arc                     
                              double BB_end,     //!< b-value at end of arc                     
                              double CC_end,     //!< c-value at end of arc                     
                              double u_end, double v_end, double w_end) //!< uvw at end of arc
{
    double center_x, center_y;
    double gamma;                 /* direction of perpendicular to arc at end */
    int side;                     /* offset side - right or left              */
    double tolerance;             /* tolerance for difference of radii        */
    double tool_radius;
    int turn;                     /* 1 for counterclockwise, -1 for clockwise */
    double cx, cy, cz; // current
    int plane = settings->plane;

    side = settings->cutter_comp_side;
    tool_radius = settings->cutter_comp_radius;   /* always is positive */
    tolerance = (settings->length_units == CANON_UNITS_INCHES) ? TOLERANCE_INCH : TOLERANCE_MM;

    comp_get_current(settings, &cx, &cy, &cz);

    CHKS((hypot((end_x - cx), (end_y - cy)) <= tool_radius),
         "Radius of cutter compensation entry arc is not greater than the tool radius");

    if (block->r_flag) {
        CHP(arc_data_comp_r(move, plane, side, tool_radius, cx, cy, end_x, end_y, 
                            block->r_number,
                            &center_x, &center_y, &turn, tolerance));
    } else {
        CHP(arc_data_comp_ijk(move, plane, side, tool_radius, cx, cy, end_x, end_y,
                              (settings->ijk_distance_mode == MODE_ABSOLUTE),
                              offset_x, offset_y,
                              &center_x, &center_y, &turn, tolerance));
    }

    inverse_time_rate_arc(cx, cy, cz, center_x, center_y,
                          turn, end_x, end_y, end_z, block, settings);


    // the tool will end up in gamma direction from the programmed arc endpoint
    if TOOL_INSIDE_ARC(side, turn) {
        // tool inside the arc: ends up toward the center
        gamma = atan2((center_y - end_y), (center_x - end_x));
    } else {
        // outside: away from the center
        gamma = atan2((end_y - center_y), (end_x - center_x));
    }

    settings->cutter_comp_firstmove = OFF;

    comp_set_programmed(settings, end_x, end_y, end_z);

    // move endpoint to the compensated position.  This changes the radius and center.
    end_x += tool_radius * cos(gamma); 
    end_y += tool_radius * sin(gamma); 

    /* To find the new center:
       imagine a right triangle ABC with A being the endpoint of the
       compensated arc, B being the center of the compensated arc, C being
       the midpoint between start and end of the compensated arc. AB_ang
       is the direction of A->B.  A_ang is the angle of the triangle
       itself.  We need to find a new center for the compensated arc
       (point B). */

    double b_len = hypot(cy - end_y, cx - end_x) / 2.0;
    double AB_ang = atan2(center_y - end_y, center_x - end_x);
    double A_ang = atan2(cy - end_y, cx - end_x) - AB_ang;

    CHKS((fabs(cos(A_ang)) < TOLERANCE_EQUAL), NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP);
  
    double c_len = b_len/cos(A_ang);

    // center of the arc is c_len from end in direction AB
    center_x = end_x + c_len * cos(AB_ang);
    center_y = end_y + c_len * sin(AB_ang);

    /* center to endpoint distances matched before - they still should. */
    CHKS((fabs(hypot(center_x-end_x,center_y-end_y) - 
              hypot(center_x-cx,center_y-cy)) > tolerance),
        NCE_BUG_IN_TOOL_RADIUS_COMP);

    // need this move for lathes to move the tool origin first.  otherwise, the arc isn't an arc.
    if (settings->cutter_comp_orientation != 0 && settings->cutter_comp_orientation != 9) {
        enqueue_STRAIGHT_FEED(settings, block->line_number, 
                              0, 0, 0,
                              cx, cy, cz,
                              AA_end, BB_end, CC_end, u_end, v_end, w_end);
        set_endpoint(cx, cy);
    }
    
    enqueue_ARC_FEED(settings, block->line_number, 
                     find_turn(cx, cy, center_x, center_y, turn, end_x, end_y),
                     end_x, end_y, center_x, center_y, turn, end_z,
                     AA_end, BB_end, CC_end, u_end, v_end, w_end);

    comp_set_current(settings, end_x, end_y, end_z);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u_end;
    settings->v_current = v_end;
    settings->w_current = w_end;

    return INTERP_OK;
}

/****************************************************************************/

/*! convert_arc_comp2

Returned Value: int
   If arc_data_ijk or arc_data_r returns an error code,
   this returns that code.
   If any of the following errors occurs, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. A concave corner is found: NCE_CONCAVE_CORNER_WITH_CUTTER_RADIUS_COMP
   2. The tool will not fit inside an arc:
      NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP

Side effects:
   This executes an arc command feed rate. If needed, at also generates
   an arc to go around a convex corner. It also updates the setting of
   the position of the tool point to the end point of the move.
Called by: convert_arc.

This function converts a helical or circular arc. The axis must be
parallel to the z-axis. This is called when cutter radius compensation
is on and this is not the first cut after the turning on.

If one or more rotary axes is moved in this block and an extra arc is
required to go around a sharp corner, all the rotary axis motion
occurs on the main arc and none on the extra arc.  An alternative
might be to distribute the rotary axis motion over the extra arc and
the programmed arc in proportion to their lengths.

If the Z-axis is moved in this block and an extra arc is required to
go around a sharp corner, all the Z-axis motion occurs on the main arc
and none on the extra arc.  An alternative might be to distribute the
Z-axis motion over the extra arc and the main arc in proportion to
their lengths.

*/

int Interp::convert_arc_comp2(int move,  //!< either G_2 (cw arc) or G_3 (ccw arc)          
                              block_pointer block,       //!< pointer to a block of RS274/NGC instructions  
                              setup_pointer settings,    //!< pointer to machine settings                   
                              double end_x,      //!< x-value at end of programmed (then actual) arc
                              double end_y,      //!< y-value at end of programmed (then actual) arc
                              double end_z,      //!< z-value at end of arc 
                              double offset_x, double offset_y,
                              double AA_end,     //!< a-value at end of arc
                              double BB_end,     //!< b-value at end of arc
                              double CC_end,     //!< c-value at end of arc
                              double u, double v, double w) //!< uvw at end of arc
{
    double alpha;                 /* direction of tangent to start of arc */
    double arc_radius;
    double beta;                  /* angle between two tangents above */
    double centerx, centery;              /* center of arc */
    double delta;                 /* direction of radius from start of arc to center of arc */
    double gamma;                 /* direction of perpendicular to arc at end */
    double midx, midy;
    int side;
    double small_ = TOLERANCE_CONCAVE_CORNER;      /* angle for testing corners */
    double opx, opy, opz;
    double theta;                 /* direction of tangent to last cut */
    double tolerance;
    double tool_radius;
    int turn;                     /* number of full or partial circles CCW */
    int plane = settings->plane;
    double cx, cy, cz;
    double new_end_x, new_end_y;

    /* find basic arc data: center_x, center_y, and turn */

    comp_get_programmed(settings, &opx, &opy, &opz);
    comp_get_current(settings, &cx, &cy, &cz);

    tolerance = (settings->length_units == CANON_UNITS_INCHES) ?
        TOLERANCE_INCH : TOLERANCE_MM;

    if (block->r_flag) {
        CHP(arc_data_r(move, plane, opx, opy, end_x, end_y,
                       block->r_number, &centerx, &centery, &turn, tolerance));
    } else {
        CHP(arc_data_ijk(move, plane,
                         opx, opy, end_x, end_y,
                         (settings->ijk_distance_mode == MODE_ABSOLUTE),
                         offset_x, offset_y,
                         &centerx, &centery, &turn, tolerance));
    }

    inverse_time_rate_arc(opx, opy, opz, centerx, centery,
                          turn, end_x, end_y, end_z, block, settings);

    side = settings->cutter_comp_side;
    tool_radius = settings->cutter_comp_radius;   /* always is positive */
    arc_radius = hypot((centerx - end_x), (centery - end_y));
    theta = atan2(cy - opy, cx - opx);
    theta = (side == LEFT) ? (theta - M_PI_2) : (theta + M_PI_2);
    delta = atan2(centery - opy, centerx - opx);
    alpha = (move == G_3) ? (delta - M_PI_2) : (delta + M_PI_2);
    beta = (side == LEFT) ? (theta - alpha) : (alpha - theta);

    // normalize beta -90 to +270?
    beta = (beta > (1.5 * M_PI)) ? (beta - (2 * M_PI)) : (beta < -M_PI_2) ? (beta + (2 * M_PI)) : beta;

    if (((side == LEFT) && (move == G_3)) || ((side == RIGHT) && (move == G_2))) {
        // we are cutting inside the arc
        gamma = atan2((centery - end_y), (centerx - end_x));
        CHKS((arc_radius <= tool_radius),
            NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP);
    } else {
        gamma = atan2((end_y - centery), (end_x - centerx));
        delta = (delta + M_PI);
    }

    // move arc endpoint to the compensated position
    new_end_x = end_x + tool_radius * cos(gamma);
    new_end_y = end_y + tool_radius * sin(gamma);

    if (beta < -small_ || 
        beta > M_PI + small_ ||
        // special detection for convex corner on tangent arc->arc (like atop the middle of "m" shape)
        // or tangent line->arc (atop "h" shape)
        (fabs(beta - M_PI) < small_ && !TOOL_INSIDE_ARC(side, turn))
        ) {
        // concave
        if (qc().front().type != QARC_FEED) {
            // line->arc
            double cy = arc_radius * sin(beta - M_PI_2);
            double toward_nominal;
            double dist_from_center;
            double angle_from_center;
          
            if TOOL_INSIDE_ARC(side, turn) {
                // tool is inside the arc
                dist_from_center = arc_radius - tool_radius;
                toward_nominal = cy + tool_radius;
                double l = toward_nominal / dist_from_center;
                CHKS((l > 1.0 || l < -1.0), EMC_I18N("Arc move in concave corner cannot be reached by the tool without gouging"));
                if(turn == 1) {
                    angle_from_center = theta + asin(l);
                } else {
                    angle_from_center = theta - asin(l);
                }
            } else {
                dist_from_center = arc_radius + tool_radius; 
                toward_nominal = cy - tool_radius;
                double l = toward_nominal / dist_from_center;
                CHKS((l > 1.0 || l < -1.0), EMC_I18N("Arc move in concave corner cannot be reached by the tool without gouging"));
                if(turn == 1) {
                    angle_from_center = theta + M_PI - asin(l);
                } else {
                    angle_from_center = theta + M_PI + asin(l);
                }
            }          
          
            midx = centerx + dist_from_center * cos(angle_from_center);
            midy = centery + dist_from_center * sin(angle_from_center);

            CHP(move_endpoint_and_flush(settings, midx, midy));
        } else {
            // arc->arc
            struct arc_feed &prev = qc().front().data.arc_feed;
            double oldrad = hypot(prev.center2 - prev.end2, prev.center1 - prev.end1);
            double newrad;
            if TOOL_INSIDE_ARC(side, turn) {
                newrad = arc_radius - tool_radius;
            } else {
                newrad = arc_radius + tool_radius;
            }
              
            double arc_cc, pullback, cc_dir, a;
            arc_cc = hypot(prev.center2 - centery, prev.center1 - centerx);

            CHKS((oldrad == 0 || arc_cc == 0), EMC_I18N("Arc to arc motion is invalid because the arcs have the same center"));
            a = (SQ(oldrad) + SQ(arc_cc) - SQ(newrad)) / (2 * oldrad * arc_cc);
            
            CHKS((a > 1.0 || a < -1.0), (EMC_I18N("Arc to arc motion makes a corner the compensated tool can't fit in without gouging")));
            pullback = acos(a);
            cc_dir = atan2(centery - prev.center2, centerx - prev.center1);

            double dir;
            if TOOL_INSIDE_ARC(side, prev.turn) {
                if(turn == 1) 
                    dir = cc_dir + pullback;
                else
                    dir = cc_dir - pullback;
            } else {
                if(turn == 1)
                    dir = cc_dir - pullback;
                else
                    dir = cc_dir + pullback;
            }
          
            midx = prev.center1 + oldrad * cos(dir);
            midy = prev.center2 + oldrad * sin(dir);
          
            CHP(move_endpoint_and_flush(settings, midx, midy));
        }
        enqueue_ARC_FEED(settings, block->line_number, 
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    } else if (beta > small_) {           /* convex, two arcs needed */
        midx = opx + tool_radius * cos(delta);
        midy = opy + tool_radius * sin(delta);
        dequeue_canons(settings);
        enqueue_ARC_FEED(settings, block->line_number, 
                         0.0, // doesn't matter since we won't move this arc's endpoint
                         midx, midy, opx, opy, ((side == LEFT) ? -1 : 1),
                         cz,
                         AA_end, BB_end, CC_end, u, v, w);
        dequeue_canons(settings);
        set_endpoint(midx, midy);
        enqueue_ARC_FEED(settings, block->line_number, 
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    } else {                      /* convex, one arc needed */
        dequeue_canons(settings);
        set_endpoint(cx, cy);
        enqueue_ARC_FEED(settings, block->line_number, 
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    }

    comp_set_programmed(settings, end_x, end_y, end_z);
    comp_set_current(settings, new_end_x, new_end_y, end_z);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u;
    settings->v_current = v;
    settings->w_current = w;

    return INTERP_OK;
}

/****************************************************************************/

/*! convert_axis_offsets

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The function is called when cutter radius compensation is on:
      NCE_CANNOT_CHANGE_AXIS_OFFSETS_WITH_CUTTER_RADIUS_COMP
   2. The g_code argument is not G_92, G_92_1, G_92_2, or G_92_3
      NCE_BUG_CODE_NOT_IN_G92_SERIES

Side effects:
   SET_PROGRAM_ORIGIN is called, and the coordinate
   values for the axis offsets are reset. The coordinates of the
   current point are reset. Parameters may be set.

Called by: convert_modal_0.

The action of G92 is described in [NCMS, pages 10 - 11] and {Fanuc,
pages 61 - 63]. [NCMS] is ambiguous about the intent, but [Fanuc]
is clear. When G92 is executed, an offset of the origin is calculated
so that the coordinates of the current point with respect to the moved
origin are as specified on the line containing the G92. If an axis
is not mentioned on the line, the coordinates of the current point
are not changed. The execution of G92 results in an axis offset being
calculated and saved for each of the six axes, and the axis offsets
are always used when motion is specified with respect to absolute
distance mode using any of the nine coordinate systems (those designated
by G54 - G59.3). Thus all nine coordinate systems are affected by G92.

Being in incremental distance mode has no effect on the action of G92
in this implementation. [NCMS] is not explicit about this, but it is
implicit in the second sentence of [Fanuc, page 61].

The offset is the amount the origin must be moved so that the
coordinate of the controlled point has the specified value. For
example, if the current point is at X=4 in the currently specified
coordinate system and the current X-axis offset is zero, then "G92 x7"
causes the X-axis offset to be reset to -3.

Since a non-zero offset may be already be in effect when the G92 is
called, that must be taken into account.

In addition to causing the axis offset values in the _setup model to be
set, G92 sets parameters 5211 to 5216 to the x,y,z,a,b,c axis offsets.

The action of G92.2 is described in [NCMS, page 12]. There is no
equivalent command in [Fanuc]. G92.2 resets axis offsets to zero.
G92.1, also included in [NCMS, page 12] (but the usage here differs
slightly from the spec), is like G92.2, except that it also causes
the axis offset parameters to be set to zero, whereas G92.2 does not
zero out the parameters.

G92.3 is not in [NCMS]. It sets the axis offset values to the values
given in the parameters.

*/

int Interp::convert_axis_offsets(int g_code,     //!< g_code being executed (must be in G_92 series)
                                block_pointer block,    //!< pointer to a block of RS274/NGC instructions  
                                setup_pointer settings) //!< pointer to machine settings                   
{
  double *pars;                 /* short name for settings->parameters            */

  CHKS((settings->cutter_comp_side != OFF),      /* not "== ON" */
      NCE_CANNOT_CHANGE_AXIS_OFFSETS_WITH_CUTTER_RADIUS_COMP);
  CHKS((block->a_flag && settings->a_axis_wrapped && (block->a_number <= -360.0 || block->a_number >= 360.0)), (EMC_I18N("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->a_number, 'A');
  CHKS((block->b_flag && settings->b_axis_wrapped && (block->b_number <= -360.0 || block->b_number >= 360.0)), (EMC_I18N("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->b_number, 'B');
  CHKS((block->c_flag && settings->c_axis_wrapped && (block->c_number <= -360.0 || block->c_number >= 360.0)), (EMC_I18N("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->c_number, 'C');
  pars = settings->parameters;
  if (g_code == G_92) {
    pars[5210] = 1.0;
    if (block->x_flag == ON) {
      settings->axis_offset_x =
        (settings->current_x + settings->axis_offset_x - block->x_number);
      settings->current_x = block->x_number;
    }

    if (block->y_flag == ON) {
      settings->axis_offset_y =
        (settings->current_y + settings->axis_offset_y - block->y_number);
      settings->current_y = block->y_number;
    }

    if (block->z_flag == ON) {
      settings->axis_offset_z =
        (settings->current_z + settings->axis_offset_z - block->z_number);
      settings->current_z = block->z_number;
    }
    if (block->a_flag == ON) {
      settings->AA_axis_offset = (settings->AA_current +
                                  settings->AA_axis_offset - block->a_number);
      settings->AA_current = block->a_number;
    }
    if (block->b_flag == ON) {
      settings->BB_axis_offset = (settings->BB_current +
                                  settings->BB_axis_offset - block->b_number);
      settings->BB_current = block->b_number;
    }
    if (block->c_flag == ON) {
      settings->CC_axis_offset = (settings->CC_current +
                                  settings->CC_axis_offset - block->c_number);
      settings->CC_current = block->c_number;
    }
    if (block->u_flag == ON) {
      settings->u_axis_offset = (settings->u_current +
                                 settings->u_axis_offset - block->u_number);
      settings->u_current = block->u_number;
    }
    if (block->v_flag == ON) {
      settings->v_axis_offset = (settings->v_current +
                                 settings->v_axis_offset - block->v_number);
      settings->v_current = block->v_number;
    }
    if (block->w_flag == ON) {
      settings->w_axis_offset = (settings->w_current +
                                 settings->w_axis_offset - block->w_number);
      settings->w_current = block->w_number;
    }

    SET_ORIGIN_OFFSETS(settings->origin_offset_x + settings->axis_offset_x,
                       settings->origin_offset_y + settings->axis_offset_y,
                       settings->origin_offset_z + settings->axis_offset_z,
                       (settings->AA_origin_offset +
                        settings->AA_axis_offset),
                       (settings->BB_origin_offset +
                        settings->BB_axis_offset),
                       (settings->CC_origin_offset +
                        settings->CC_axis_offset),
                       (settings->u_origin_offset +
                        settings->u_axis_offset),
                       (settings->v_origin_offset +
                        settings->v_axis_offset),
                       (settings->w_origin_offset +
                        settings->w_axis_offset));
    
    pars[5211] = PROGRAM_TO_USER_LEN(settings->axis_offset_x);
    pars[5212] = PROGRAM_TO_USER_LEN(settings->axis_offset_y);
    pars[5213] = PROGRAM_TO_USER_LEN(settings->axis_offset_z);
    pars[5214] = PROGRAM_TO_USER_ANG(settings->AA_axis_offset);
    pars[5215] = PROGRAM_TO_USER_ANG(settings->BB_axis_offset);
    pars[5216] = PROGRAM_TO_USER_ANG(settings->CC_axis_offset);
    pars[5217] = PROGRAM_TO_USER_LEN(settings->u_axis_offset);
    pars[5218] = PROGRAM_TO_USER_LEN(settings->v_axis_offset);
    pars[5219] = PROGRAM_TO_USER_LEN(settings->w_axis_offset);
  } else if ((g_code == G_92_1) || (g_code == G_92_2)) {
    pars[5210] = 0.0;
    settings->current_x = settings->current_x + settings->axis_offset_x;
    settings->current_y = settings->current_y + settings->axis_offset_y;
    settings->current_z = settings->current_z + settings->axis_offset_z;
    settings->AA_current = (settings->AA_current + settings->AA_axis_offset);
    settings->BB_current = (settings->BB_current + settings->BB_axis_offset);
    settings->CC_current = (settings->CC_current + settings->CC_axis_offset);
    settings->u_current = (settings->u_current + settings->u_axis_offset);
    settings->v_current = (settings->v_current + settings->v_axis_offset);
    settings->w_current = (settings->w_current + settings->w_axis_offset);
    SET_ORIGIN_OFFSETS(settings->origin_offset_x,
                       settings->origin_offset_y, settings->origin_offset_z,
                       settings->AA_origin_offset,
                       settings->BB_origin_offset,
                       settings->CC_origin_offset,
                       settings->u_origin_offset,
                       settings->v_origin_offset,
                       settings->w_origin_offset);
    settings->axis_offset_x = 0.0;
    settings->axis_offset_y = 0.0;
    settings->axis_offset_z = 0.0;
    settings->AA_axis_offset = 0.0;
    settings->BB_axis_offset = 0.0;
    settings->CC_axis_offset = 0.0;
    settings->u_axis_offset = 0.0;
    settings->v_axis_offset = 0.0;
    settings->w_axis_offset = 0.0;
    if (g_code == G_92_1) {
      pars[5211] = 0.0;
      pars[5212] = 0.0;
      pars[5213] = 0.0;
      pars[5214] = 0.0;
      pars[5215] = 0.0;
      pars[5216] = 0.0;
      pars[5217] = 0.0;
      pars[5218] = 0.0;
      pars[5219] = 0.0;
    }
  } else if (g_code == G_92_3) {
    pars[5210] = 1.0;
    settings->current_x =
      settings->current_x + settings->axis_offset_x - USER_TO_PROGRAM_LEN(pars[5211]);
    settings->current_y =
      settings->current_y + settings->axis_offset_y - USER_TO_PROGRAM_LEN(pars[5212]);
    settings->current_z =
      settings->current_z + settings->axis_offset_z - USER_TO_PROGRAM_LEN(pars[5213]);
    settings->AA_current =
      settings->AA_current + settings->AA_axis_offset - USER_TO_PROGRAM_ANG(pars[5214]);
    settings->BB_current =
      settings->BB_current + settings->BB_axis_offset - USER_TO_PROGRAM_ANG(pars[5215]);
    settings->CC_current =
      settings->CC_current + settings->CC_axis_offset - USER_TO_PROGRAM_ANG(pars[5216]);
    settings->u_current =
      settings->u_current + settings->u_axis_offset - USER_TO_PROGRAM_LEN(pars[5217]);
    settings->v_current =
      settings->v_current + settings->v_axis_offset - USER_TO_PROGRAM_LEN(pars[5218]);
    settings->w_current =
      settings->w_current + settings->w_axis_offset - USER_TO_PROGRAM_LEN(pars[5219]);

    settings->axis_offset_x = USER_TO_PROGRAM_LEN(pars[5211]);
    settings->axis_offset_y = USER_TO_PROGRAM_LEN(pars[5212]);
    settings->axis_offset_z = USER_TO_PROGRAM_LEN(pars[5213]);
    settings->AA_axis_offset = USER_TO_PROGRAM_ANG(pars[5214]);
    settings->BB_axis_offset = USER_TO_PROGRAM_ANG(pars[5215]);
    settings->CC_axis_offset = USER_TO_PROGRAM_ANG(pars[5216]);
    settings->u_axis_offset = USER_TO_PROGRAM_LEN(pars[5217]);
    settings->v_axis_offset = USER_TO_PROGRAM_LEN(pars[5218]);
    settings->w_axis_offset = USER_TO_PROGRAM_LEN(pars[5219]);
    SET_ORIGIN_OFFSETS(settings->origin_offset_x + settings->axis_offset_x,
                       settings->origin_offset_y + settings->axis_offset_y,
                       settings->origin_offset_z + settings->axis_offset_z,
                       (settings->AA_origin_offset +
                        settings->AA_axis_offset),
                       (settings->BB_origin_offset +
                        settings->BB_axis_offset),
                       (settings->CC_origin_offset +
                        settings->CC_axis_offset),
                       (settings->u_origin_offset +
                        settings->u_axis_offset),
                       (settings->v_origin_offset +
                        settings->v_axis_offset),
                       (settings->w_origin_offset +
                        settings->w_axis_offset));

  } else
    ERS(NCE_BUG_CODE_NOT_IN_G92_SERIES);

  return INTERP_OK;
}

#define VAL_LEN 30

int Interp::convert_param_comment(char *comment, char *expanded, int len)
{
    int i;
    char param[LINELEN+1];
    int paramNumber;
    int stat;
    double value;
    char valbuf[VAL_LEN]; // max double length + room
    char *v;
    int found;

    while(*comment)
    {
        if(*comment == '#')
        {
            found = 0;
            logDebug("a parameter");

            // skip over the '#'
            comment++;
            CHKS((0 == *comment), NCE_NAMED_PARAMETER_NOT_TERMINATED);

            if(isdigit(*comment))  // is this numeric param?
            {
                logDebug("numeric parameter");
                for(i=0; isdigit(*comment)&& (i<LINELEN); i++)
                {
                    param[i] = *comment++;
                }
                param[i] = 0;
                paramNumber = atoi(param);
                if((paramNumber >= 0) &&
                   (paramNumber < RS274NGC_MAX_PARAMETERS))
                {
                    value = _setup.parameters[paramNumber];
                    found = 1;
                }
            }
            else if(*comment == '<')
            {
                logDebug("name parameter");
                // this is a name parameter
                // skip over the '<'
                comment++;
                CHKS((0 == *comment), NCE_NAMED_PARAMETER_NOT_TERMINATED);

                for(i=0; (')' != *comment) &&
                        (i<LINELEN) && (0 != *comment);)
                {
                    if('>' == *comment)
                    {
                        break;     // done
                    }
                    if(isspace(*comment)) // skip space inside the param
                    {
                        comment++;
                        continue;
                    }
                    else
                    {
		        // if tolower is a macro, may need this int
		        int c = *comment++;
                        param[i] = tolower(c);
                        i++;
                    }
                }
                if('>' != *comment)
                {
                    ERS(NCE_NAMED_PARAMETER_NOT_TERMINATED);
                    logDebug("parameter not terminated");
                }
                else
                {
                    comment++;
                }

                // terminate the name
                param[i] = 0;

                // now lookup the name
                find_named_param(param, &stat, &value);
                if(stat)
                {
                    found = 1;
                }
            }
            else
            {
                // neither numeric or name
                logDebug("neither numeric nor name");
                // just store the '#'
                *expanded++ = '#';

                CHKS((*comment == 0), NCE_NAMED_PARAMETER_NOT_TERMINATED);
                continue;
            }

            // we have a parameter -- now insert it
            // we have the value
            if(found)
            {
                int n = snprintf(valbuf, VAL_LEN, "%lf", value);
                bool fail = (n >= VAL_LEN || n < 0);
                if(fail)
                    strcpy(valbuf, "######");

            }
            else
            {
                strcpy(valbuf, "######");
            }
            logDebug("found:%d value:|%s|", found, valbuf);

            v = valbuf;
            while(*v)
            {
                *expanded++ = *v++;
            }
        }
        else  // not a '#'
        {
            *expanded++ = *comment++;
        }
    }
    *expanded = 0; // the final nul
    
    return INTERP_OK;
}

/****************************************************************************/

/*! convert_comment

Returned Value: int (INTERP_OK)

Side effects:
   The message function is called if the string starts with "MSG,".
   Otherwise, the comment function is called.

Called by: execute_block

To be a message, the first four characters of the comment after the
opening left parenthesis must be "MSG,", ignoring the case of the
letters and allowing spaces or tabs anywhere before the comma (to make
the treatment of case and white space consistent with how it is
handled elsewhere).

Messages are not provided for in [NCMS]. They are implemented here as a
subtype of comment. This is an extension to the rs274NGC language.

*/

static int streq(char *s1, char *s2) {
    return !strcmp(s1, s2);
}

static int startswith(char *haystack, char *needle) {
    return !strncmp(haystack, needle, strlen(needle));
}

int Interp::convert_comment(char *comment)       //!< string with comment
{
  enum
  { LC_SIZE = 256, EX_SIZE = 2*LC_SIZE};            // 256 from comment[256] in rs274ngc.hh
  char lc[LC_SIZE+1];
  char expanded[EX_SIZE+1];
  char MSG_STR[] = "msg,";

  //!!!KL add two -- debug => same as msg
  //!!!KL         -- print => goes to stderr
  char DEBUG_STR[] = "debug,";
  char PRINT_STR[] = "print,";
  char LOG_STR[] = "log,";
  char LOGOPEN_STR[] = "logopen,";
  char LOGCLOSE_STR[] = "logclose";
  int m, n, start;

  // step over leading white space in comment
  m = 0;
  while (isspace(comment[m]))
    m++;
  start = m;
  // copy lowercase comment to lc[]
  for (n = 0; n < LC_SIZE && comment[m] != 0; m++, n++) {
    lc[n] = tolower(comment[m]);
  }
  lc[n] = 0;                    // null terminate

  // compare with MSG, SYSTEM, DEBUG, PRINT
  if (startswith(lc, MSG_STR)) {
    MESSAGE(comment + start + strlen(MSG_STR));
    return INTERP_OK;
  }
  else if (startswith(lc, DEBUG_STR))
  {
      convert_param_comment(comment+start+strlen(DEBUG_STR), expanded,
                            EX_SIZE);
      MESSAGE(expanded);
      return INTERP_OK;
  }
  else if (startswith(lc, PRINT_STR)) 
  {
      convert_param_comment(comment+start+strlen(PRINT_STR), expanded,
                            EX_SIZE);
      fprintf(stdout, "%s\n", expanded);
      fflush(stdout);
      return INTERP_OK;
  }
  else if (startswith(lc, LOG_STR))
  {
      convert_param_comment(comment+start+strlen(LOG_STR), expanded,
                            EX_SIZE);
      LOG(expanded);
      return INTERP_OK;
  }
  else if (startswith(lc, LOGOPEN_STR))
  {
      LOGOPEN(comment + start + strlen(LOGOPEN_STR));
      return INTERP_OK;
  }
  else if (streq(lc, LOGCLOSE_STR))
  {
      LOGCLOSE();
      return INTERP_OK;
  }
  // else it's a real comment
  enqueue_COMMENT(comment + start);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_control_mode

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_61, G_61_1, G_64 : NCE_BUG_CODE_NOT_G61_G61_1_OR_G64

Side effects: See below

Called by: convert_g.

The interpreter switches the machine settings to indicate the
control mode (CANON_EXACT_STOP, CANON_EXACT_PATH or CANON_CONTINUOUS)

A call is made to SET_MOTION_CONTROL_MODE(CANON_XXX), where CANON_XXX is
CANON_EXACT_PATH if g_code is G_61, CANON_EXACT_STOP if g_code is G_61_1,
and CANON_CONTINUOUS if g_code is G_64.

Setting the control mode to CANON_EXACT_STOP on G_61 would correspond
more closely to the meaning of G_61 as given in [NCMS, page 40], but
CANON_EXACT_PATH has the advantage that the tool does not stop if it
does not have to, and no evident disadvantage compared to
CANON_EXACT_STOP, so it is being used for G_61. G_61_1 is not defined
in [NCMS], so it is available and is used here for setting the control
mode to CANON_EXACT_STOP.

It is OK to call SET_MOTION_CONTROL_MODE(CANON_XXX) when CANON_XXX is
already in force.

*/

int Interp::convert_control_mode(int g_code,     //!< g_code being executed (G_61, G61_1, || G_64)
				double tolerance,    //tolerance for the path following in G64
				double naivecam_tolerance,    //tolerance for the naivecam
                                setup_pointer settings) //!< pointer to machine settings                 
{
  CHKS((settings->cutter_comp_side != OFF),
       (EMC_I18N("Cannot change control mode with cutter radius compensation on")));
  if (g_code == G_61) {
    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);
    settings->control_mode = CANON_EXACT_PATH;
  } else if (g_code == G_61_1) {
    SET_MOTION_CONTROL_MODE(CANON_EXACT_STOP, 0);
    settings->control_mode = CANON_EXACT_STOP;
  } else if (g_code == G_64) {
	if (tolerance >= 0) {
	    SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS, tolerance);
	} else {
	    SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS, 0);
	}
	if (naivecam_tolerance >= 0) {
	    SET_NAIVECAM_TOLERANCE(naivecam_tolerance);
	} else if (tolerance >= 0) {
	    SET_NAIVECAM_TOLERANCE(tolerance);   // if no naivecam_tolerance specified use same for both
	} else {
	    SET_NAIVECAM_TOLERANCE(0);
	}
    settings->control_mode = CANON_CONTINUOUS;
  } else 
    ERS(NCE_BUG_CODE_NOT_G61_G61_1_OR_G64);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_coordinate_system

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The value of the g_code argument is not 540, 550, 560, 570, 580, 590
      591, 592, or 593:
      NCE_BUG_CODE_NOT_IN_RANGE_G54_TO_G593

Side effects:
   If the coordinate system selected by the g_code is not already in
   use, the canonical program coordinate system axis offset values are
   reset and the coordinate values of the current point are reset.

Called by: convert_g.

COORDINATE SYSTEMS (involves g10, g53, g54 - g59.3, g92)

The canonical machining functions view of coordinate systems is:
1. There are two coordinate systems: absolute and program.
2. All coordinate values are given in terms of the program coordinate system.
3. The offsets of the program coordinate system may be reset.

The RS274/NGC view of coordinate systems, as given in section 3.2
of [NCMS] is:
1. there are ten coordinate systems: absolute and 9 program. The
   program coordinate systems are numbered 1 to 9.
2. you can switch among the 9 but not to the absolute one. G54
   selects coordinate system 1, G55 selects 2, and so on through
   G56, G57, G58, G59, G59.1, G59.2, and G59.3.
3. you can set the offsets of the 9 program coordinate systems
   using G10 L2 Pn (n is the number of the coordinate system) with
   values for the axes in terms of the absolute coordinate system.
4. the first one of the 9 program coordinate systems is the default.
5. data for coordinate systems is stored in parameters [NCMS, pages 59 - 60].
6. g53 means to interpret coordinate values in terms of the absolute
   coordinate system for the one block in which g53 appears.
7. You can offset the current coordinate system using g92. This offset
   will then apply to all nine program coordinate systems.

The approach used in the interpreter mates the canonical and NGC views
of coordinate systems as follows:

During initialization, data from the parameters for the first NGC
coordinate system is used in a SET_ORIGIN_OFFSETS function call and
origin_index in the machine model is set to 1.

If a g_code in the range g54 - g59.3 is encountered in an NC program,
the data from the appropriate NGC coordinate system is copied into the
origin offsets used by the interpreter, a SET_ORIGIN_OFFSETS function
call is made, and the current position is reset.

If a g10 is encountered, the convert_setup function is called to reset
the offsets of the program coordinate system indicated by the P number
given in the same block.

If a g53 is encountered, the axis values given in that block are used
to calculate what the coordinates are of that point in the current
coordinate system, and a STRAIGHT_TRAVERSE or STRAIGHT_FEED function
call to that point using the calculated values is made. No offset
values are changed.

If a g92 is encountered, that is handled by the convert_axis_offsets
function. A g92 results in an axis offset for each axis being calculated
and stored in the machine model. The axis offsets are applied to all
nine coordinate systems. Axis offsets are initialized to zero.

*/

void Interp::rotate(double *x, double *y, double theta) {
    double xx, yy;
    double t = D2R(theta);
    xx = *x * cos(t) - *y * sin(t); 
    yy = *x * sin(t) + *y * cos(t);
    *x = xx;
    *y = yy;
}

int Interp::convert_coordinate_system(int g_code,        //!< g_code called (must be one listed above)     
                                     setup_pointer settings)    //!< pointer to machine settings                  
{
  int origin;
  double *parameters;

  CHKS((settings->cutter_comp_side != OFF),
       (EMC_I18N("Cannot change coordinate systems with cutter radius compensation on")));
  parameters = settings->parameters;
  switch (g_code) {
  case 540:
    origin = 1;
    break;
  case 550:
    origin = 2;
    break;
  case 560:
    origin = 3;
    break;
  case 570:
    origin = 4;
    break;
  case 580:
    origin = 5;
    break;
  case 590:
    origin = 6;
    break;
  case 591:
    origin = 7;
    break;
  case 592:
    origin = 8;
    break;
  case 593:
    origin = 9;
    break;
  default:
    ERS(NCE_BUG_CODE_NOT_IN_RANGE_G54_TO_G593);
  }

  if (origin == settings->origin_index) {       /* already using this origin */
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: continuing to use same coordinate system");
#endif
    return INTERP_OK;
  }

  // move the current point into the new system
  find_current_in_system(settings, origin,
                         &settings->current_x, &settings->current_y, &settings->current_z,
                         &settings->AA_current, &settings->BB_current, &settings->CC_current,
                         &settings->u_current, &settings->v_current, &settings->w_current);

  // remember that this is new system
  settings->origin_index = origin;
  parameters[5220] = (double) origin;

  // load the origin of the newly-selected system
  settings->origin_offset_x = USER_TO_PROGRAM_LEN(parameters[5201 + (origin * 20)]);
  settings->origin_offset_y = USER_TO_PROGRAM_LEN(parameters[5202 + (origin * 20)]);
  settings->origin_offset_z = USER_TO_PROGRAM_LEN(parameters[5203 + (origin * 20)]);
  settings->AA_origin_offset = USER_TO_PROGRAM_ANG(parameters[5204 + (origin * 20)]);
  settings->BB_origin_offset = USER_TO_PROGRAM_ANG(parameters[5205 + (origin * 20)]);
  settings->CC_origin_offset = USER_TO_PROGRAM_ANG(parameters[5206 + (origin * 20)]);
  settings->u_origin_offset = USER_TO_PROGRAM_LEN(parameters[5207 + (origin * 20)]);
  settings->v_origin_offset = USER_TO_PROGRAM_LEN(parameters[5208 + (origin * 20)]);
  settings->w_origin_offset = USER_TO_PROGRAM_LEN(parameters[5209 + (origin * 20)]);
  settings->rotation_xy = parameters[5210 + (origin * 20)];

  SET_ORIGIN_OFFSETS(settings->origin_offset_x + settings->axis_offset_x,
                     settings->origin_offset_y + settings->axis_offset_y,
                     settings->origin_offset_z + settings->axis_offset_z,
                     settings->AA_origin_offset + settings->AA_axis_offset,
                     settings->BB_origin_offset + settings->BB_axis_offset,
                     settings->CC_origin_offset + settings->CC_axis_offset,
                     settings->u_origin_offset + settings->u_axis_offset,
                     settings->v_origin_offset + settings->v_axis_offset,
                     settings->w_origin_offset + settings->w_axis_offset);
  SET_XY_ROTATION(settings->rotation_xy);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cutter_compensation

Returned Value: int
   If convert_cutter_compensation_on or convert_cutter_compensation_off
      is called and returns an error code, this returns that code.
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code is not G_40, G_41, or G_42:
      NCE_BUG_CODE_NOT_G40_G41_OR_G42

Side effects:
   The value of cutter_comp_side in the machine model mode is
   set to RIGHT, LEFT, or OFF. The currently active tool table index in
   the machine model (which is the index of the slot whose diameter
   value is used in cutter radius compensation) is updated.

Since cutter radius compensation is performed in the interpreter, no
call is made to any canonical function regarding cutter radius compensation.

Called by: convert_g

*/

int Interp::convert_cutter_compensation(int g_code,      //!< must be G_40, G_41, or G_42             
                                       block_pointer block,     //!< pointer to a block of RS274 instructions
                                       setup_pointer settings)  //!< pointer to machine settings             
{

  if (g_code == G_40) {
    CHP(convert_cutter_compensation_off(settings));
  } else if (g_code == G_41) {
    CHP(convert_cutter_compensation_on(LEFT, block, settings));
  } else if (g_code == G_42) {
    CHP(convert_cutter_compensation_on(RIGHT, block, settings));
  } else if (g_code == G_41_1) {
    CHP(convert_cutter_compensation_on(LEFT, block, settings));
  } else if (g_code == G_42_1) {
    CHP(convert_cutter_compensation_on(RIGHT, block, settings));
  } else
    ERS("BUG: Code not G40, G41, G41.1, G42, G42.2");

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cutter_compensation_off

Returned Value: int (INTERP_OK)

Side effects:
   A comment is made that cutter radius compensation is turned off.
   The machine model of the cutter radius compensation mode is set to OFF.
   The value of cutter_comp_firstmove in the machine model is set to ON.
     This serves as a flag when cutter radius compensation is
     turned on again.

Called by: convert_cutter_compensation

*/

int Interp::convert_cutter_compensation_off(setup_pointer settings)      //!< pointer to machine settings
{
#ifdef DEBUG_EMC
  enqueue_COMMENT("interpreter: cutter radius compensation off");
#endif
  if(settings->cutter_comp_side != OFF && settings->cutter_comp_radius > 0.0) {
      double cx, cy, cz;
      comp_get_current(settings, &cx, &cy, &cz);
      CHP(move_endpoint_and_flush(settings, cx, cy));
      dequeue_canons(settings);
      settings->current_x = settings->program_x;
      settings->current_y = settings->program_y;
      settings->current_z = settings->program_z;
      settings->arc_not_allowed = ON;
  }
  settings->cutter_comp_side = OFF;
  settings->cutter_comp_firstmove = ON;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_cutter_compensation_on

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The selected plane is not the XY plane:
      NCE_CANNOT_TURN_CUTTER_RADIUS_COMP_ON_OUT_OF_XY_PLANE
   2. Cutter radius compensation is already on:
      NCE_CANNOT_TURN_CUTTER_RADIUS_COMP_ON_WHEN_ON

Side effects:
   A COMMENT function call is made (conditionally) saying that the
   interpreter is switching mode so that cutter radius compensation is on.
   The value of cutter_comp_radius in the machine model mode is
   set to the absolute value of the radius given in the tool table.
   The value of cutter_comp_side in the machine model mode is
   set to RIGHT or LEFT. The currently active tool table index in
   the machine model is updated.

Called by: convert_cutter_compensation

check_other_codes checks that a d word occurs only in a block with g41
or g42.

Cutter radius compensation is carried out in the interpreter, so no
call is made to a canonical function (although there is a canonical
function, START_CUTTER_RADIUS_COMPENSATION, that could be called if
the primitive level could execute it).

This version uses a D word if there is one in the block, but it does
not require a D word, since the sample programs which the interpreter
is supposed to handle do not have them.  Logically, the D word is
optional, since the D word is always (except in cases we have never
heard of) the slot number of the tool in the spindle. Not requiring a
D word is contrary to [Fanuc, page 116] and [NCMS, page 79], however.
Both manuals require the use of the D-word with G41 and G42.

This version handles a negative offset radius, which may be
encountered if the programmed tool path is a center line path for
cutting a profile and the path was constructed using a nominal tool
diameter. Then the value in the tool table for the diameter is set to
be the difference between the actual diameter and the nominal
diameter. If the actual diameter is less than the nominal, the value
in the table is negative. The method of handling a negative radius is
to switch the side of the offset and use a positive radius. This
requires that the profile use arcs (not straight lines) to go around
convex corners.

*/

/* Set *result to the integer nearest to value; return TRUE if value is
 * within .0001 of an integer
 */
static int is_near_int(int *result, double value) {
    *result = (int)(value + .5);
    return fabs(*result - value) < .0001;
}

int Interp::convert_cutter_compensation_on(int side,     //!< side of path cutter is on (LEFT or RIGHT)
                                          block_pointer block,  //!< pointer to a block of RS274 instructions 
                                          setup_pointer settings)       //!< pointer to machine settings              
{
  double radius;
  int index, orientation;

  CHKS((settings->plane != CANON_PLANE_XY && settings->plane != CANON_PLANE_XZ),
      NCE_RADIUS_COMP_ONLY_IN_XY_OR_XZ);
  CHKS((settings->cutter_comp_side != OFF),
      NCE_CANNOT_TURN_CUTTER_RADIUS_COMP_ON_WHEN_ON);
  if(block->g_modes[7] == G_41_1 || block->g_modes[7] == G_42_1) {
      CHKS((block->d_flag != ON),
              EMC_I18N("G%d.1 with no D word"), block->g_modes[7]/10 );
      radius = block->d_number_float / 2;
      if(block->l_number != -1) {
          CHKS((settings->plane != CANON_PLANE_XZ), EMC_I18N("G%d.1 with L word, but plane is not G18"), block->g_modes[7]/10);
          orientation = block->l_number;
      } else {
          orientation = 0;
      }
  } else {
      if(block->d_flag == OFF) {
          index = 0;
      } else {
          int tool;
          CHKS(!is_near_int(&tool, block->d_number_float),
                  EMC_I18N("G%d requires D word to be a whole number"),
                   block->g_modes[7]/10);
          CHKS((tool < 0), NCE_NEGATIVE_D_WORD_TOOL_RADIUS_INDEX_USED);
          CHP((find_tool_pocket(settings, tool, &index)));
      }
      radius = USER_TO_PROGRAM_LEN(settings->tool_table[index].diameter) / 2.0;
      orientation = settings->tool_table[index].orientation;
      CHKS((settings->plane != CANON_PLANE_XZ && orientation != 0 && orientation != 9), EMC_I18N("G%d with lathe tool, but plane is not G18"), block->g_modes[7]/10);
  }
  if (radius < 0.0) { /* switch side & make radius positive if radius negative */
    radius = -radius;
    if (side == RIGHT)
      side = LEFT;
    else
      side = RIGHT;
  }
#ifdef DEBUG_EMC
  if (side == RIGHT)
    enqueue_COMMENT("interpreter: cutter radius compensation on right");
  else
    enqueue_COMMENT("interpreter: cutter radius compensation on left");
#endif

  settings->cutter_comp_radius = radius;
  settings->cutter_comp_orientation = orientation;
  settings->cutter_comp_side = side;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_distance_mode

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_90 or G_91: NCE_BUG_CODE_NOT_G90_OR_G91

Side effects:
   The interpreter switches the machine settings to indicate the current
   distance mode (absolute or incremental).

   The canonical machine to which commands are being sent does not have
   an incremental mode, so no command setting the distance mode is
   generated in this function. A comment function call explaining the
   change of mode is made (conditionally), however, if there is a change.

Called by: convert_g.

*/

// OK to call this in a concave corner with a deferred move, since it
// doesn't issue any CANONs

int Interp::convert_distance_mode(int g_code,    //!< g_code being executed (must be G_90 or G_91)
                                 setup_pointer settings)        //!< pointer to machine settings                 
{
  if (g_code == G_90) {
    if (settings->distance_mode != MODE_ABSOLUTE) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: distance mode changed to absolute");
#endif
      settings->distance_mode = MODE_ABSOLUTE;
    }
  } else if (g_code == G_91) {
    if (settings->distance_mode != MODE_INCREMENTAL) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: distance mode changed to incremental");
#endif
      settings->distance_mode = MODE_INCREMENTAL;
    }
  } else
    ERS(NCE_BUG_CODE_NOT_G90_OR_G91);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_ijk_distance_mode

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_90.1 or G_91.1: NCE_BUG_CODE_NOT_G90_OR_G91

Side effects:
   The interpreter switches the machine settings to indicate the current
   distance mode for arc centers (absolute or incremental).

   The canonical machine to which commands are being sent does not have
   an incremental mode, so no command setting the distance mode is
   generated in this function. A comment function call explaining the
   change of mode is made (conditionally), however, if there is a change.

Called by: convert_g.

*/

// OK to call this in a concave corner with a deferred move, since it
// doesn't issue any CANONs except comments (and who cares where the comments are)

int Interp::convert_ijk_distance_mode(int g_code,    //!< g_code being executed (must be G_90_1 or G_91_1)
                                 setup_pointer settings)        //!< pointer to machine settings                 
{
  if (g_code == G_90_1) {
    if (settings->ijk_distance_mode != MODE_ABSOLUTE) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: IJK distance mode changed to absolute");
#endif
      settings->ijk_distance_mode = MODE_ABSOLUTE;
    }
  } else if (g_code == G_91_1) {
    if (settings->ijk_distance_mode != MODE_INCREMENTAL) {
#ifdef DEBUG_EMC
      enqueue_COMMENT("interpreter: IJK distance mode changed to incremental");
#endif
      settings->ijk_distance_mode = MODE_INCREMENTAL;
    }
  } else
    ERS(NCE_BUG_CODE_NOT_G90_OR_G91);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_lathe_diameter_mode

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_07 or G_08: NCE_BUG_CODE_NOT_G07_OR_G08

Side effects:
   The interpreter switches the machine settings to indicate the current
   distance mode for arc centers (absolute or incremental).

   The canonical machine to which commands are being sent does not have
   an incremental mode, so no command setting the distance mode is
   generated in this function. A comment function call explaining the
   change of mode is made (conditionally), however, if there is a change.

Called by: convert_g.

*/

int Interp::convert_lathe_diameter_mode(int g_code,    //!< g_code being executed (must be G_90_1 or G_91_1)
                  block_pointer block,           //!< pointer to current block
                  setup_pointer settings)        //!< pointer to machine settings
{
  if (g_code == G_7) {
    if (settings->lathe_diameter_mode != ON) {
      if(block->x_flag)
      {
        block->x_number /= 2; //Apply scaling now
      }
      if(block->motion_to_be == G_76) {
          block->i_number /= 2;
          block->j_number /= 2;
          block->k_number /= 2;
      }              
#ifdef DEBUG_EMC
      COMMENT("interpreter: Lathe diameter mode changed to diameter");
#endif
      settings->lathe_diameter_mode = ON;
    }
  } else if (g_code == G_8) {
    if (settings->lathe_diameter_mode != OFF) {
      if(block->x_flag)
      {
        block->x_number *= 2; //Remove any existing scaling
      }
      if(block->motion_to_be == G_76) {
          block->i_number *= 2;
          block->j_number *= 2;
          block->k_number *= 2;
      }              
#ifdef DEBUG_EMC
      COMMENT("interpreter: Lathe diameter mode changed to radius");
#endif
      settings->lathe_diameter_mode = OFF;
    }
  } else
    ERS("BUG: Code not G7 or G8");
  return INTERP_OK;
}


/****************************************************************************/

/*! convert_dwell

Returned Value: int (INTERP_OK)

Side effects:
   A dwell command is executed.

Called by: convert_g.

*/

int Interp::convert_dwell(setup_pointer settings, double time)   //!< time in seconds to dwell  */
{
  enqueue_DWELL(time);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_feed_mode

Returned Value: int
   If any of the following errors occur, this returns an error code.
   Otherwise, it returns INTERP_OK.
   1.  g_code isn't G_93, G_94 or G_95

Side effects:
   The interpreter switches the machine settings to indicate the current
   feed mode (UNITS_PER_MINUTE or INVERSE_TIME).

   The canonical machine to which commands are being sent does not have
   a feed mode, so no command setting the distance mode is generated in
   this function. A comment function call is made (conditionally)
   explaining the change in mode, however.

Called by: execute_block.

*/

int Interp::convert_feed_mode(int g_code,        //!< g_code being executed (must be G_93, G_94 or G_95)
                             setup_pointer settings)    //!< pointer to machine settings                 
{
  if (g_code == G_93) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: feed mode set to inverse time");
#endif
    settings->feed_mode = INVERSE_TIME;
    enqueue_SET_FEED_MODE(0);
  } else if (g_code == G_94) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: feed mode set to units per minute");
#endif
    settings->feed_mode = UNITS_PER_MINUTE;
    enqueue_SET_FEED_MODE(0);
    settings->feed_rate = 0.0;
    enqueue_SET_FEED_RATE(0);
  } else if(g_code == G_95) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: feed mode set to units per revolution");
#endif
    settings->feed_mode = UNITS_PER_REVOLUTION;
    enqueue_SET_FEED_MODE(1);
    settings->feed_rate = 0.0;
    enqueue_SET_FEED_RATE(0);
  } else
    ERS("BUG: Code not G93, G94, or G95");
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_feed_rate

Returned Value: int (INTERP_OK)

Side effects:
   The machine feed_rate is set to the value of f_number in the
     block by function call.
   The machine model feed_rate is set to that value.

Called by: execute_block

This is called only if the feed mode is UNITS_PER_MINUTE or UNITS_PER_REVOLUTION.

*/

int Interp::convert_feed_rate(block_pointer block,       //!< pointer to a block of RS274 instructions
                             setup_pointer settings)    //!< pointer to machine settings             
{
  settings->feed_rate = block->f_number;
  enqueue_SET_FEED_RATE(block->f_number);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_g

Returned Value: int
   If one of the following functions is called and returns an error code,
   this returns that code.
      convert_control_mode
      convert_coordinate_system
      convert_cutter_compensation
      convert_distance_mode
      convert_ijk_distance_mode
      convert_lathe_diameter_mode
      convert_dwell
      convert_length_units
      convert_modal_0
      convert_motion
      convert_retract_mode
      convert_set_plane
      convert_tool_length_offset
   Otherwise, it returns INTERP_OK.

Side effects:
   Any g_codes in the block (excluding g93 and 94) and any implicit
   motion g_code are executed.

Called by: execute_block.

This takes a pointer to a block of RS274/NGC instructions (already
read in) and creates the appropriate output commands corresponding to
any "g" codes in the block.

Codes g93 and g94, which set the feed mode, are executed earlier by
execute_block before reading the feed rate.

G codes are are executed in the following order.
1.  mode 0, G4 only - dwell. Left here from earlier versions.
2.  mode 2, one of (G17, G18, G19) - plane selection.
3.  mode 6, one of (G20, G21) - length units.
4.  mode 15 one of (G07,G08) - lathe diameter mode
5.  mode 7, one of (G40, G41, G42) - cutter radius compensation.
6.  mode 8, one of (G43, G49) - tool length offset
7.  mode 12, one of (G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3)
    - coordinate system selection.
8.  mode 13, one of (G61, G61.1, G64, G50, G51) - control mode
9.  mode 3, one of (G90, G91) - distance mode.
10.  mode 4, one of (G90.1, G91.1) - arc i,j,k mode.
11. mode 10, one of (G98, G99) - retract mode.
12. mode 0, one of (G10, G28, G30, G92, G92.1, G92.2, G92.3) -
13. mode 1, one of (G0, G1, G2, G3, G38.2, G80, G81 to G89, G33, G33.1, G76) - motion or cancel.
    G53 from mode 0 is also handled here, if present.

Some mode 0 and most mode 1 G codes must be executed after the length units
are set, since they use coordinate values. Mode 1 codes also must wait
until most of the other modes are set.

*/

int Interp::convert_g(block_pointer block,       //!< pointer to a block of RS274/NGC instructions
                     setup_pointer settings)    //!< pointer to machine settings                 
{

  if (block->g_modes[0] == G_4) {
    CHP(convert_dwell(settings, block->p_number));
  }
  if (block->g_modes[2] != -1) {
    CHP(convert_set_plane(block->g_modes[2], settings));
  }
  if (block->g_modes[6] != -1) {
    CHP(convert_length_units(block->g_modes[6], settings));
  }
  if (block->g_modes[15] != -1) {
    CHP(convert_lathe_diameter_mode(block->g_modes[15], block, settings));
  }
  if (block->g_modes[7] != -1) {
    CHP(convert_cutter_compensation(block->g_modes[7], block, settings));
  }
  if (block->g_modes[8] != -1) {
    CHP(convert_tool_length_offset(block->g_modes[8], block, settings));
  }
  if (block->g_modes[12] != -1) {
    CHP(convert_coordinate_system(block->g_modes[12], settings));
  }
  if (block->g_modes[13] != -1) {
    CHP(convert_control_mode(block->g_modes[13], block->p_number, block->q_number, settings));
  }
  if (block->g_modes[3] != -1) {
    CHP(convert_distance_mode(block->g_modes[3], settings));
  }
  if (block->g_modes[4] != -1) {
    CHP(convert_ijk_distance_mode(block->g_modes[4], settings));
  }
  if (block->g_modes[10] != -1) {
    CHP(convert_retract_mode(block->g_modes[10], settings));
  }
  if (block->g_modes[0] != -1) {
    CHP(convert_modal_0(block->g_modes[0], block, settings));
  }
  if (block->motion_to_be != -1) {
    CHP(convert_motion(block->motion_to_be, block, settings));
  }
  return INTERP_OK;
}

int Interp::convert_savehome(int code, block_pointer block, setup_pointer s) {
    double *p = s->parameters;
    
    if(s->cutter_comp_side != OFF) {
        ERS("Cannot set reference point with cutter compensation in effect");
    }

    double x = PROGRAM_TO_USER_LEN(s->current_x + s->tool_offset.tran.x + s->origin_offset_x + s->axis_offset_x);
    double y = PROGRAM_TO_USER_LEN(s->current_y + s->tool_offset.tran.y + s->origin_offset_y + s->axis_offset_y);
    double z = PROGRAM_TO_USER_LEN(s->current_z + s->tool_offset.tran.z + s->origin_offset_z + s->axis_offset_z);
    double a = PROGRAM_TO_USER_ANG(s->AA_current + s->tool_offset.a + s->AA_origin_offset + s->AA_axis_offset);
    double b = PROGRAM_TO_USER_ANG(s->BB_current + s->tool_offset.b + s->BB_origin_offset + s->BB_axis_offset);
    double c = PROGRAM_TO_USER_ANG(s->CC_current + s->tool_offset.c + s->CC_origin_offset + s->CC_axis_offset);
    double u = PROGRAM_TO_USER_LEN(s->u_current + s->tool_offset.u + s->u_origin_offset + s->u_axis_offset);
    double v = PROGRAM_TO_USER_LEN(s->v_current + s->tool_offset.v + s->v_origin_offset + s->v_axis_offset);
    double w = PROGRAM_TO_USER_LEN(s->w_current + s->tool_offset.w + s->w_origin_offset + s->w_axis_offset);

    if(s->a_axis_wrapped) {
        a = fmod(a, 360.0);
        if(a<0) a += 360.0;
    }

    if(s->b_axis_wrapped) {
        b = fmod(b, 360.0);
        if(b<0) b += 360.0;
    }

    if(s->c_axis_wrapped) {
        c = fmod(c, 360.0);
        if(c<0) c += 360.0;
    }

    if(code == G_28_1) {
        p[5161] = x;
        p[5162] = y;
        p[5163] = z;
        p[5164] = a;
        p[5165] = b;
        p[5166] = c;
        p[5167] = u;
        p[5168] = v;
        p[5169] = w;
    } else if(code == G_30_1) {
        p[5181] = x;
        p[5182] = y;
        p[5183] = z;
        p[5184] = a;
        p[5185] = b;
        p[5186] = c;
        p[5187] = u;
        p[5188] = v;
        p[5189] = w;
    } else {
        ERS("BUG: Code not G28.1 or G38.1");
    }
    return INTERP_OK;
}


/****************************************************************************/

/*! convert_home

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. cutter radius compensation is on:
      NCE_CANNOT_USE_G28_OR_G30_WITH_CUTTER_RADIUS_COMP
   2. The code is not G28 or G30: NCE_BUG_CODE_NOT_G28_OR_G30

Side effects: 
   This executes a straight traverse to the programmed point, using
   the current coordinate system, tool length offset, and motion mode
   to interpret the coordinate values. Then it executes a straight
   traverse to move one or more axes to the location of reference
   point 1 (if G28) or reference point 2 (if G30).  If any axis words
   are specified in this block, only those axes are moved to the
   reference point.  If none are specified, all axes are moved.  It
   also updates the setting of the position of the tool point to the
   end point of the move.

   N.B. Many gcode programmers call the reference point a home
   position, and that is exactly what it is if the parameters are
   zero.  Do not confuse this with homing the axis (searching for
   a switch or index pulse).

Called by: convert_modal_0.

*/

int Interp::convert_home(int move,       //!< G code, must be G_28 or G_30            
                        block_pointer block,    //!< pointer to a block of RS274 instructions
                        setup_pointer settings) //!< pointer to machine settings             
{
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end;
  double v_end;
  double w_end;
  double end_x_home;
  double end_y_home;
  double end_z_home;
  double AA_end_home;
  double BB_end_home;
  double CC_end_home;
  double u_end_home;
  double v_end_home;
  double w_end_home;
  double *parameters;

  parameters = settings->parameters;
  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end, 
                &u_end, &v_end, &w_end));

  CHKS((settings->cutter_comp_side != OFF),
      NCE_CANNOT_USE_G28_OR_G30_WITH_CUTTER_RADIUS_COMP);

  // waypoint is in currently active coordinate system

  STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z,
                    AA_end, BB_end, CC_end,
                    u_end, v_end, w_end);

  if (move == G_28) {
      find_relative(USER_TO_PROGRAM_LEN(parameters[5161]),
                    USER_TO_PROGRAM_LEN(parameters[5162]),
                    USER_TO_PROGRAM_LEN(parameters[5163]),
                    USER_TO_PROGRAM_ANG(parameters[5164]),
                    USER_TO_PROGRAM_ANG(parameters[5165]),
                    USER_TO_PROGRAM_ANG(parameters[5166]),
                    USER_TO_PROGRAM_LEN(parameters[5167]),
                    USER_TO_PROGRAM_LEN(parameters[5168]),
                    USER_TO_PROGRAM_LEN(parameters[5169]),
                    &end_x_home, &end_y_home, &end_z_home,
                    &AA_end_home, &BB_end_home, &CC_end_home, 
                    &u_end_home, &v_end_home, &w_end_home, settings);
  } else if (move == G_30) {
      find_relative(USER_TO_PROGRAM_LEN(parameters[5181]),
                    USER_TO_PROGRAM_LEN(parameters[5182]),
                    USER_TO_PROGRAM_LEN(parameters[5183]),
                    USER_TO_PROGRAM_ANG(parameters[5184]),
                    USER_TO_PROGRAM_ANG(parameters[5185]),
                    USER_TO_PROGRAM_ANG(parameters[5186]),
                    USER_TO_PROGRAM_LEN(parameters[5187]),
                    USER_TO_PROGRAM_LEN(parameters[5188]),
                    USER_TO_PROGRAM_LEN(parameters[5189]),
                    &end_x_home, &end_y_home, &end_z_home,
                    &AA_end_home, &BB_end_home, &CC_end_home, 
                    &u_end_home, &v_end_home, &w_end_home, settings);
  } else
    ERS(NCE_BUG_CODE_NOT_G28_OR_G30);
  
  // if any axes are specified, home only those axes after the waypoint 
  // (both fanuc & haas, contrary to emc historical operation)

  if (block->x_flag == ON) end_x = end_x_home;  
  if (block->y_flag == ON) end_y = end_y_home;  
  if (block->z_flag == ON) end_z = end_z_home;  
  if (block->a_flag == ON) AA_end = AA_end_home;
  if (block->b_flag == ON) BB_end = BB_end_home;
  if (block->c_flag == ON) CC_end = CC_end_home;
  if (block->u_flag == ON) u_end = u_end_home;  
  if (block->v_flag == ON) v_end = v_end_home;  
  if (block->w_flag == ON) w_end = w_end_home;  

  // but, if no axes are specified, home all of them 
  // (haas does this, emc historical did, throws an error in fanuc)

  if (block->x_flag == OFF && block->y_flag == OFF && block->z_flag == OFF &&
      block->a_flag == OFF && block->b_flag == OFF && block->c_flag == OFF &&
      block->u_flag == OFF && block->v_flag == OFF && block->w_flag == OFF) {
      end_x = end_x_home;  
      end_y = end_y_home;  
      end_z = end_z_home;  
      AA_end = AA_end_home;
      BB_end = BB_end_home;
      CC_end = CC_end_home;
      u_end = u_end_home;  
      v_end = v_end_home;  
      w_end = w_end_home;  
  }

  STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z,
                    AA_end, BB_end, CC_end,
                    u_end, v_end, w_end);
  settings->current_x = end_x;
  settings->current_y = end_y;
  settings->current_z = end_z;
  settings->AA_current = AA_end;
  settings->BB_current = BB_end;
  settings->CC_current = CC_end;
  settings->u_current = u_end;
  settings->v_current = v_end;
  settings->w_current = w_end;
  
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_length_units

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The g_code argument isnt G_20 or G_21:
      NCE_BUG_CODE_NOT_G20_OR_G21
   2. Cutter radius compensation is on:
      NCE_CANNOT_CHANGE_UNITS_WITH_CUTTER_RADIUS_COMP

Side effects:
   A command setting the length units is executed. The machine
   settings are reset regarding length units and current position.

Called by: convert_g.

Tool length offset and diameter, work coordinate systems, feed rate,
g28/g30 home positions, and g53 motion in absolute coordinates all work
properly after switching units.  Historically these had problems but the
intention is that they work properly now.

The tool table in settings is not converted here; it is always in
inifile units and the conversion happens when reading an entry.  Tool
offsets and feed rate that are in effect are converted by rereading them
from the canon level.

Cutter diameter is not converted because radius comp is not in effect
when we are changing units.  

XXX Other distance items in the settings (such as the various parameters
for cycles) need testing.

*/

int Interp::convert_length_units(int g_code,     //!< g_code being executed (must be G_20 or G_21)
                                setup_pointer settings) //!< pointer to machine settings                 
{
  if (g_code == G_20) {
    USE_LENGTH_UNITS(CANON_UNITS_INCHES);
    if (settings->length_units != CANON_UNITS_INCHES) {
      settings->length_units = CANON_UNITS_INCHES;
      settings->current_x = (settings->current_x * INCH_PER_MM);
      settings->current_y = (settings->current_y * INCH_PER_MM);
      settings->current_z = (settings->current_z * INCH_PER_MM);
      settings->program_x = (settings->program_x * INCH_PER_MM);
      settings->program_y = (settings->program_y * INCH_PER_MM);
      settings->program_z = (settings->program_z * INCH_PER_MM);
      qc_scale(INCH_PER_MM);
      settings->cutter_comp_radius *= INCH_PER_MM;
      settings->axis_offset_x = (settings->axis_offset_x * INCH_PER_MM);
      settings->axis_offset_y = (settings->axis_offset_y * INCH_PER_MM);
      settings->axis_offset_z = (settings->axis_offset_z * INCH_PER_MM);
      settings->origin_offset_x = (settings->origin_offset_x * INCH_PER_MM);
      settings->origin_offset_y = (settings->origin_offset_y * INCH_PER_MM);
      settings->origin_offset_z = (settings->origin_offset_z * INCH_PER_MM);

      settings->u_current = (settings->u_current * INCH_PER_MM);
      settings->v_current = (settings->v_current * INCH_PER_MM);
      settings->w_current = (settings->w_current * INCH_PER_MM);
      settings->u_axis_offset = (settings->u_axis_offset * INCH_PER_MM);
      settings->v_axis_offset = (settings->v_axis_offset * INCH_PER_MM);
      settings->w_axis_offset = (settings->w_axis_offset * INCH_PER_MM);
      settings->u_origin_offset = (settings->u_origin_offset * INCH_PER_MM);
      settings->v_origin_offset = (settings->v_origin_offset * INCH_PER_MM);
      settings->w_origin_offset = (settings->w_origin_offset * INCH_PER_MM);

      settings->tool_offset.tran.x = GET_EXTERNAL_TOOL_LENGTH_XOFFSET();
      settings->tool_offset.tran.y = GET_EXTERNAL_TOOL_LENGTH_YOFFSET();
      settings->tool_offset.tran.z = GET_EXTERNAL_TOOL_LENGTH_ZOFFSET();
      settings->tool_offset.a = GET_EXTERNAL_TOOL_LENGTH_AOFFSET();
      settings->tool_offset.b = GET_EXTERNAL_TOOL_LENGTH_BOFFSET();
      settings->tool_offset.c = GET_EXTERNAL_TOOL_LENGTH_COFFSET();
      settings->tool_offset.u = GET_EXTERNAL_TOOL_LENGTH_UOFFSET();
      settings->tool_offset.v = GET_EXTERNAL_TOOL_LENGTH_VOFFSET();
      settings->tool_offset.w = GET_EXTERNAL_TOOL_LENGTH_WOFFSET();
      settings->feed_rate = GET_EXTERNAL_FEED_RATE();
    }
  } else if (g_code == G_21) {
    USE_LENGTH_UNITS(CANON_UNITS_MM);
    if (settings->length_units != CANON_UNITS_MM) {
      settings->length_units = CANON_UNITS_MM;
      settings->current_x = (settings->current_x * MM_PER_INCH);
      settings->current_y = (settings->current_y * MM_PER_INCH);
      settings->current_z = (settings->current_z * MM_PER_INCH);
      settings->program_x = (settings->program_x * MM_PER_INCH);
      settings->program_y = (settings->program_y * MM_PER_INCH);
      settings->program_z = (settings->program_z * MM_PER_INCH);
      qc_scale(MM_PER_INCH);
      settings->cutter_comp_radius *= MM_PER_INCH;
      settings->axis_offset_x = (settings->axis_offset_x * MM_PER_INCH);
      settings->axis_offset_y = (settings->axis_offset_y * MM_PER_INCH);
      settings->axis_offset_z = (settings->axis_offset_z * MM_PER_INCH);
      settings->origin_offset_x = (settings->origin_offset_x * MM_PER_INCH);
      settings->origin_offset_y = (settings->origin_offset_y * MM_PER_INCH);
      settings->origin_offset_z = (settings->origin_offset_z * MM_PER_INCH);

      settings->u_current = (settings->u_current * MM_PER_INCH);
      settings->v_current = (settings->v_current * MM_PER_INCH);
      settings->w_current = (settings->w_current * MM_PER_INCH);
      settings->u_axis_offset = (settings->u_axis_offset * MM_PER_INCH);
      settings->v_axis_offset = (settings->v_axis_offset * MM_PER_INCH);
      settings->w_axis_offset = (settings->w_axis_offset * MM_PER_INCH);
      settings->u_origin_offset = (settings->u_origin_offset * MM_PER_INCH);
      settings->v_origin_offset = (settings->v_origin_offset * MM_PER_INCH);
      settings->w_origin_offset = (settings->w_origin_offset * MM_PER_INCH);

      settings->tool_offset.tran.x = GET_EXTERNAL_TOOL_LENGTH_XOFFSET();
      settings->tool_offset.tran.y = GET_EXTERNAL_TOOL_LENGTH_YOFFSET();
      settings->tool_offset.tran.z = GET_EXTERNAL_TOOL_LENGTH_ZOFFSET();
      settings->tool_offset.a = GET_EXTERNAL_TOOL_LENGTH_AOFFSET();
      settings->tool_offset.b = GET_EXTERNAL_TOOL_LENGTH_BOFFSET();
      settings->tool_offset.c = GET_EXTERNAL_TOOL_LENGTH_COFFSET();
      settings->tool_offset.u = GET_EXTERNAL_TOOL_LENGTH_UOFFSET();
      settings->tool_offset.v = GET_EXTERNAL_TOOL_LENGTH_VOFFSET();
      settings->tool_offset.w = GET_EXTERNAL_TOOL_LENGTH_WOFFSET();
      settings->feed_rate = GET_EXTERNAL_FEED_RATE();
    }
  } else
    ERS(NCE_BUG_CODE_NOT_G20_OR_G21);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_m

Returned Value: int
   If convert_tool_change returns an error code, this returns that code.
   If input-related stuff is needed, it sets the flag input_flag = ON.
   Otherwise, it returns INTERP_OK.

Side effects:
   m_codes in the block are executed. For each m_code
   this consists of making a function call(s) to a canonical machining
   function(s) and setting the machine model.

Called by: execute_block.

This handles four separate types of activity in order:
1. changing the tool (m6) - which also retracts and stops the spindle.
2. Turning the spindle on or off (m3, m4, and m5)
3. Turning coolant on and off (m7, m8, and m9)
4. turning a-axis clamping on and off (m26, m27) - commented out.
5. enabling or disabling feed and speed overrides (m49, m49).
6. changing the loaded toolnumber (m61).
Within each group, only the first code encountered will be executed.

This does nothing with m0, m1, m2, m30, or m60 (which are handled in
convert_stop).

*/

int Interp::convert_m(block_pointer block,       //!< pointer to a block of RS274/NGC instructions
                     setup_pointer settings)    //!< pointer to machine settings                 
{
  int type;
  double timeout;               // timeout for M66

  /* The M62-65 commands are used for DIO */
  /* M62 sets a DIO synched with motion
     M63 clears a DIO synched with motion
     M64 sets a DIO imediately
     M65 clears a DIO imediately 
     M66 waits for an input
     M67 reads a digital input
     M68 reads an analog input*/

  if (block->m_modes[5] == 62) {
      CHKS((settings->cutter_comp_side != OFF),
           (EMC_I18N("Cannot set motion output with cutter radius compensation on")));  // XXX
      CHKS((block->p_flag == OFF), EMC_I18N("No valid P word with M62"));
      SET_MOTION_OUTPUT_BIT(round_to_int(block->p_number));
  } else if (block->m_modes[5] == 63) {
      CHKS((settings->cutter_comp_side != OFF),
           (EMC_I18N("Cannot set motion digital output with cutter radius compensation on")));  // XXX
      CHKS((block->p_flag == OFF), EMC_I18N("No valid P word with M63"));
      CLEAR_MOTION_OUTPUT_BIT(round_to_int(block->p_number));
  } else if (block->m_modes[5] == 64) {
      CHKS((settings->cutter_comp_side != OFF),
           (EMC_I18N("Cannot set auxiliary digital output with cutter radius compensation on")));  // XXX
      CHKS((block->p_flag == OFF), EMC_I18N("No valid P word with M64"));
      SET_AUX_OUTPUT_BIT(round_to_int(block->p_number));
  } else if (block->m_modes[5] == 65) {
      CHKS((settings->cutter_comp_side != OFF),
           (EMC_I18N("Cannot set auxiliary digital output with cutter radius compensation on")));  // XXX
      CHKS((block->p_flag == OFF), EMC_I18N("No valid P word with M65"));
      CLEAR_AUX_OUTPUT_BIT(round_to_int(block->p_number));
  } else if (block->m_modes[5] == 66) {
    //P-word = digital channel
    //E-word = analog channel
    //L-word = wait type (immediate, rise, fall, high, low)
    //Q-word = timeout
    // it is an error if:

    // P and E word are specified together
    CHKS(((block->p_flag == ON) && (block->e_flag == ON)),
	NCE_BOTH_DIGITAL_AND_ANALOG_INPUT_SELECTED);

    // L-word not 0, and timeout <= 0 
    CHKS(((block->q_number <= 0) && (block->l_flag == ON) && (round_to_int(block->l_number) > 0)),
	NCE_ZERO_TIMEOUT_WITH_WAIT_NOT_IMMEDIATE);
	
    // E-word specified (analog input) and wait type not immediate
    CHKS(((block->e_flag == ON) && (block->l_flag == ON) && (round_to_int(block->l_number) != 0)),
	NCE_ANALOG_INPUT_WITH_WAIT_NOT_IMMEDIATE);

    // missing P or E (or invalid = negative)
    CHKS( ((block->p_flag == ON) && (round_to_int(block->p_number) < 0)) || 
         ((block->e_flag == ON) && (round_to_int(block->e_number) < 0)) ||
	 ((block->p_flag == OFF) && (block->e_flag == OFF)) ,
	NCE_INVALID_OR_MISSING_P_AND_E_WORDS_FOR_WAIT_INPUT);

    if (block->p_flag == ON) { // got a digital input
	if (round_to_int(block->p_number) < 0) // safety check for negative words
	    ERS("invalid P-word with M66");
	    
	if (block->l_flag == ON) {
	    type = round_to_int(block->l_number);
	} else {
	    type = WAIT_MODE_IMMEDIATE;
        }
	    
	if (block->q_number > 0) {
	    timeout = block->q_number;
	} else {
	    timeout = 0;
        }

        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot wait for digital input with cutter radius compensation on")));

	int ret = WAIT(round_to_int(block->p_number), DIGITAL_INPUT, type, timeout);
	//WAIT returns 0 on success, -1 for out of bounds
	CHKS((ret == -1), NCE_DIGITAL_INPUT_INVALID_ON_M66);
	if (ret == 0) {
	    settings->input_flag = ON;
	    settings->input_index = round_to_int(block->p_number);
	    settings->input_digital = ON;
	}
    } else if (round_to_int(block->e_number) >= 0) { // got an analog input
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot wait for analog input with cutter radius compensation on")));

	int ret = WAIT(round_to_int(block->e_number), ANALOG_INPUT, 0, 0); //WAIT returns 0 on success, -1 for out of bounds
	CHKS((ret == -1), NCE_ANALOG_INPUT_INVALID_ON_M66);
	if (ret == 0) {
	    settings->input_flag = ON;
	    settings->input_index = round_to_int(block->e_number);
	    settings->input_digital = OFF;
	}
    } 
  } else if (block->m_modes[5] == 67) {
    //E-word = analog channel
    //Q-word = analog value
      CHKS((settings->cutter_comp_side != OFF),
           (EMC_I18N("Cannot set motion analog output with cutter radius compensation on")));  // XXX
      CHKS((block->e_flag == OFF) || (round_to_int(block->e_number) < 0), (EMC_I18N("Invalid analog index with M67")));
      SET_MOTION_OUTPUT_VALUE(round_to_int(block->e_number), block->q_number);
  } else if (block->m_modes[5] == 68) {
    //E-word = analog channel
    //Q-word = analog value
      CHKS((settings->cutter_comp_side != OFF),
           (EMC_I18N("Cannot set auxiliary analog output with cutter radius compensation on")));  // XXX
      CHKS((block->e_flag == OFF) || (round_to_int(block->e_number) < 0), (EMC_I18N("Invalid analog index with M68")));
      SET_AUX_OUTPUT_VALUE(round_to_int(block->e_number), block->q_number);
  }    

  if (block->m_modes[6] != -1) {
    // when we have M6 do the actual toolchange
    if (block->m_modes[6] == 6)
	CHP(convert_tool_change(settings));
	
    // when we have M61 we only change the number of the loaded tool (for example on startup)
    if (block->m_modes[6] == 61) {
	CHKS((round_to_int(block->q_number) < 0), (EMC_I18N("Need positive Q-word to specify tool number with M61")));
	settings->current_pocket = round_to_int(block->q_number);
	CHANGE_TOOL_NUMBER(round_to_int(block->q_number));
    }    
#ifdef DEBATABLE
    // I would like this, but it's a big change.  It changes the
    // operation of legal ngc programs, but it could be argued that
    // those programs are buggy or likely to be not what the author
    // intended.

    // It would allow you to turn on G43 after loading the first tool,
    // and then not worry about it through the program.  When you
    // finally unload the last tool, G43 mode is canceled.

    if(settings->active_g_codes[9] == G_43) {
        if(settings->selected_tool_slot > 0) {
            struct block_struct g43;
            init_block(&g43);
            block->g_modes[_gees[G_43]] = G_43;
            CHP(convert_tool_length_offset(G_43, &g43, settings));
        } else {
            struct block_struct g49;
            init_block(&g49);
            block->g_modes[_gees[G_49]] = G_49;
            CHP(convert_tool_length_offset(G_49, &g49, settings));
        }
    }
#endif
  }

  if (block->m_modes[7] == 3) {
      enqueue_START_SPINDLE_CLOCKWISE();
      settings->spindle_turning = CANON_CLOCKWISE;
  } else if (block->m_modes[7] == 4) {
      enqueue_START_SPINDLE_COUNTERCLOCKWISE();
      settings->spindle_turning = CANON_COUNTERCLOCKWISE;
  } else if (block->m_modes[7] == 5) {
      enqueue_STOP_SPINDLE_TURNING();
      settings->spindle_turning = CANON_STOPPED;
  }

  if (block->m_modes[8] == 7) {
      enqueue_MIST_ON();
      settings->mist = ON;
  } else if (block->m_modes[8] == 8) {
      enqueue_FLOOD_ON();
      settings->flood = ON;
  } else if (block->m_modes[8] == 9) {
      enqueue_MIST_OFF();
      settings->mist = OFF;
      enqueue_FLOOD_OFF();
      settings->flood = OFF;
  }

/* No axis clamps in this version
  if (block->m_modes[2] == 26)
    {
#ifdef DEBUG_EMC
      COMMENT("interpreter: automatic A-axis clamping turned on");
#endif
      settings->a_axis_clamping = ON;
    }
  else if (block->m_modes[2] == 27)
    {
#ifdef DEBUG_EMC
      COMMENT("interpreter: automatic A-axis clamping turned off");
#endif
      settings->a_axis_clamping = OFF;
    }
*/

  if (block->m_modes[9] == 48) {
    CHKS((settings->cutter_comp_side != OFF),
         (EMC_I18N("Cannot enable overrides with cutter radius compensation on")));  // XXX
    ENABLE_FEED_OVERRIDE();
    ENABLE_SPEED_OVERRIDE();
    settings->feed_override = ON;
    settings->speed_override = ON;
  } else if (block->m_modes[9] == 49) {
    CHKS((settings->cutter_comp_side != OFF),
         (EMC_I18N("Cannot disable overrides with cutter radius compensation on")));  // XXX
    DISABLE_FEED_OVERRIDE();
    DISABLE_SPEED_OVERRIDE();
    settings->feed_override = OFF;
    settings->speed_override = OFF;
  }

  if (block->m_modes[9] == 50) {
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot enable overrides with cutter radius compensation on")));  // XXX
	ENABLE_FEED_OVERRIDE();
	settings->feed_override = ON;
    } else {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot disable overrides with cutter radius compensation on")));  // XXX
        DISABLE_FEED_OVERRIDE();
	settings->feed_override = OFF;
    }
  }

  if (block->m_modes[9] == 51) {
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot enable overrides with cutter radius compensation on")));  // XXX
	ENABLE_SPEED_OVERRIDE();
	settings->speed_override = ON;
    } else {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot disable overrides with cutter radius compensation on")));  // XXX
	DISABLE_SPEED_OVERRIDE();
	settings->speed_override = OFF;
    }
  }
  
  if (block->m_modes[9] == 52) {
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot enable overrides with cutter radius compensation on")));  // XXX
	ENABLE_ADAPTIVE_FEED();
	settings->adaptive_feed = ON;
    } else {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot disable overrides with cutter radius compensation on")));  // XXX
	DISABLE_ADAPTIVE_FEED();
	settings->adaptive_feed = OFF;
    }
  }
  
  if (block->m_modes[9] == 53) {
    if (block->p_number != 0) {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot enable overrides with cutter radius compensation on")));  // XXX
	ENABLE_FEED_HOLD();
	settings->feed_hold = ON;
    } else {
        CHKS((settings->cutter_comp_side != OFF),
             (EMC_I18N("Cannot disable overrides with cutter radius compensation on")));  // XXX
	DISABLE_FEED_HOLD();
	settings->feed_hold = OFF;
    }
  }

  /* user-defined M codes */
  if (block->m_modes[10] != -1) {
    int index = block->m_modes[10];
    enqueue_M_USER_COMMAND(index,block->p_number,block->q_number);
  }
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_modal_0

Returned Value: int
   If one of the following functions is called and returns an error code,
   this returns that code.
      convert_axis_offsets
      convert_home
      convert_setup
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. code is not G_4, G_10, G_28, G_30, G_53, G92, G_92_1, G_92_2, or G_92_3:
      NCE_BUG_CODE_NOT_G4_G10_G28_G30_G53_OR_G92_SERIES

Side effects: See below

Called by: convert_g

If the g_code is g10, g28, g30, g92, g92.1, g92.2, or g92.3 (all are in
modal group 0), it is executed. The other two in modal group 0 (G4 and
G53) are executed elsewhere.

*/

int Interp::convert_modal_0(int code,    //!< G code, must be from group 0                
                           block_pointer block, //!< pointer to a block of RS274/NGC instructions
                           setup_pointer settings)      //!< pointer to machine settings                 
{

  if (code == G_10) {
      if(block->l_number == 1 || block->l_number == 10)
          CHP(convert_setup_tool(block, settings));
      else
          CHP(convert_setup(block, settings));
  } else if ((code == G_28) || (code == G_30)) {
    CHP(convert_home(code, block, settings));
  } else if ((code == G_28_1) || (code == G_30_1)) {
    CHP(convert_savehome(code, block, settings));
  } else if ((code == G_92) || (code == G_92_1) ||
             (code == G_92_2) || (code == G_92_3)) {
    CHP(convert_axis_offsets(code, block, settings));
  } else if (code == G_5_3) {
    CHP(convert_nurbs(code, block, settings));
  } else if ((code == G_4) || (code == G_53));  /* handled elsewhere */
  else
    ERS(NCE_BUG_CODE_NOT_G4_G10_G28_G30_G53_OR_G92_SERIES);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_motion

Returned Value: int
   If one of the following functions is called and returns an error code,
   this returns that code.
      convert_arc
      convert_cycle
      convert_probe
      convert_straight
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The motion code is not 0,1,2,3,38.2,80,81,82,83,84,85,86,87, 88, or 89:
      NCE_BUG_UNKNOWN_MOTION_CODE

Side effects:
   A g_code from the group causing motion (mode 1) is executed.

Called by: convert_g.

*/

int Interp::convert_motion(int motion,   //!< g_code for a line, arc, canned cycle     
                          block_pointer block,  //!< pointer to a block of RS274 instructions 
                          setup_pointer settings)       //!< pointer to machine settings              
{

  if ((motion == G_0) || (motion == G_1) || (motion == G_33) || (motion == G_33_1) || (motion == G_76)) {
    CHP(convert_straight(motion, block, settings));
  } else if ((motion == G_3) || (motion == G_2)) {
    CHP(convert_arc(motion, block, settings));
  } else if (motion == G_38_2 || motion == G_38_3 || 
             motion == G_38_4 || motion == G_38_5) {
    CHP(convert_probe(block, motion, settings));
  } else if (motion == G_80) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: motion mode set to none");
#endif
    settings->motion_mode = G_80;
  } else if (motion == G_73 || ((motion > G_80) && (motion < G_90))) {
    CHP(convert_cycle(motion, block, settings));
  } else if ((motion == G_5) || (motion == G_5_1)) {
    CHP(convert_spline(motion, block, settings));
  } else if (motion == G_5_2) {
    CHP(convert_nurbs(motion, block, settings));
  } else {
    ERS(NCE_BUG_UNKNOWN_MOTION_CODE);
  }

  return INTERP_OK;
}

/****************************************************************************/

/*! convert_probe

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. No value is given in the block for any of X, Y, or Z:
      NCE_X_Y_AND_Z_WORDS_ALL_MISSING_WITH_G38_2
   3. cutter radius comp is on: NCE_CANNOT_PROBE_WITH_CUTTER_RADIUS_COMP_ON
   4. Feed rate is zero: NCE_CANNOT_PROBE_WITH_ZERO_FEED_RATE
   5. The move is degenerate (already at the specified point)
      NCE_START_POINT_TOO_CLOSE_TO_PROBE_POINT

Side effects:
   This executes a straight_probe command.
   The probe_flag in the settings is set to ON.
   The motion mode in the settings is set to G_38_2.

Called by: convert_motion.

The approach to operating in incremental distance mode (g91) is to
put the the absolute position values into the block before using the
block to generate a move.

After probing is performed, the location of the probe cannot be
predicted. This differs from every other command, all of which have
predictable results. The next call to the interpreter (with either
Interp::read or Interp::execute) will result in updating the
current position by calls to get_external_position_x, etc.

*/

int Interp::convert_probe(block_pointer block,   //!< pointer to a block of RS274 instructions
                          int g_code,
                          setup_pointer settings)        //!< pointer to machine settings             
{
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end;
  double v_end;
  double w_end;

  /* probe_type: 
     ~1 = error if probe operation is unsuccessful (ngc default)
     |1 = suppress error, report in # instead
     ~2 = move until probe trips (ngc default)
     |2 = move until probe clears */

  unsigned char probe_type = g_code - G_38_2;
  

  CHKS((block->x_flag == OFF && block->y_flag == OFF &&
       block->z_flag == OFF && block->a_flag == OFF &&
       block->b_flag == OFF && block->c_flag == OFF &&
       block->u_flag == OFF && block->v_flag == OFF &&
       block->w_flag == OFF && block->radius_flag == OFF &&
       block->theta_flag == OFF),
       NCE_X_Y_Z_A_B_C_U_V_AND_W_WORDS_ALL_MISSING_WITH_G38_2);
  CHKS((settings->cutter_comp_side != OFF),
      NCE_CANNOT_PROBE_WITH_CUTTER_RADIUS_COMP_ON);
  CHKS((settings->feed_rate == 0.0), NCE_CANNOT_PROBE_WITH_ZERO_FEED_RATE);
  CHKS(settings->feed_mode == UNITS_PER_REVOLUTION,
	  "Cannot probe with feed per rev mode");
  CHKS((settings->feed_rate == 0.0), NCE_CANNOT_PROBE_WITH_ZERO_FEED_RATE);
  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end,
                &u_end, &v_end, &w_end));
  CHKS(((!(probe_type & 1)) && 
        settings->current_x == end_x && settings->current_y == end_y &&
        settings->current_z == end_z && settings->AA_current == AA_end &&
        settings->BB_current == BB_end && settings->CC_current == CC_end &&
        settings->u_current == u_end && settings->v_current == v_end &&
        settings->w_current == w_end),
       NCE_START_POINT_TOO_CLOSE_TO_PROBE_POINT);
       
  TURN_PROBE_ON();
  STRAIGHT_PROBE(block->line_number, end_x, end_y, end_z,
                 AA_end, BB_end, CC_end,
                 u_end, v_end, w_end, probe_type);

  TURN_PROBE_OFF();
  settings->motion_mode = g_code;
  settings->probe_flag = ON;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_retract_mode

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. g_code isn't G_98 or G_99: NCE_BUG_CODE_NOT_G98_OR_G99

Side effects:
   The interpreter switches the machine settings to indicate the current
   retract mode for canned cycles (OLD_Z or R_PLANE).

Called by: convert_g.

The canonical machine to which commands are being sent does not have a
retract mode, so no command setting the retract mode is generated in
this function.

*/

int Interp::convert_retract_mode(int g_code,     //!< g_code being executed (must be G_98 or G_99)
                                setup_pointer settings) //!< pointer to machine settings                 
{
  CHKS((settings->cutter_comp_side != OFF),
       (EMC_I18N("Cannot change retract mode with cutter radius compensation on")));
  if (g_code == G_98) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: retract mode set to old_z");
#endif
    settings->retract_mode = OLD_Z;
  } else if (g_code == G_99) {
#ifdef DEBUG_EMC
    enqueue_COMMENT("interpreter: retract mode set to r_plane");
#endif
    settings->retract_mode = R_PLANE;
  } else
    ERS(NCE_BUG_CODE_NOT_G98_OR_G99);
  return INTERP_OK;
}

// G10 L1 P[tool number] R[radius] X[x offset] Z[z offset] Q[orientation]

int Interp::convert_setup_tool(block_pointer block, setup_pointer settings) {
    int pocket = -1, toolno;
    int q;

    is_near_int(&toolno, block->p_number);

    CHP((find_tool_pocket(settings, toolno, &pocket)));

    settings->tool_table[pocket].toolno = toolno;

    if(block->x_flag) {
        double x = block->x_number;
        if (block->l_number == 10) x = settings->current_x + settings->tool_offset.tran.x - x;
        settings->tool_table[pocket].offset.tran.x = PROGRAM_TO_USER_LEN(x);
    }
    if(block->y_flag) {
        double y = block->y_number;
        if (block->l_number == 10) y = settings->current_y + settings->tool_offset.tran.y - y;
        settings->tool_table[pocket].offset.tran.y = PROGRAM_TO_USER_LEN(y);
    }
    if(block->z_flag) {
        double z = block->z_number;
        if (block->l_number == 10) z = settings->current_z + settings->tool_offset.tran.z - z;
        settings->tool_table[pocket].offset.tran.z = PROGRAM_TO_USER_LEN(z);
    }
    if(block->a_flag) {
        double a = block->a_number;
        if (block->l_number == 10) a = settings->AA_current + settings->tool_offset.a - a;
        settings->tool_table[pocket].offset.a = PROGRAM_TO_USER_ANG(a);
    }
    if(block->b_flag) {
        double b = block->b_number;
        if (block->l_number == 10) b = settings->BB_current + settings->tool_offset.b - b;
        settings->tool_table[pocket].offset.b = PROGRAM_TO_USER_ANG(b);
    }
    if(block->c_flag) {
        double c = block->c_number;
        if (block->l_number == 10) c = settings->CC_current + settings->tool_offset.c - c;
        settings->tool_table[pocket].offset.c = PROGRAM_TO_USER_ANG(c);
    }
    if(block->u_flag) {
        double u = block->u_number;
        if (block->l_number == 10) u = settings->u_current + settings->tool_offset.u - u;
        settings->tool_table[pocket].offset.u = PROGRAM_TO_USER_LEN(u);
    }
    if(block->v_flag) {
        double v = block->v_number;
        if (block->l_number == 10) v = settings->v_current + settings->tool_offset.v - v;
        settings->tool_table[pocket].offset.v = PROGRAM_TO_USER_LEN(v);
    }
    if(block->w_flag) {
        double w = block->w_number;
        if (block->l_number == 10) w = settings->w_current + settings->tool_offset.w - w;
        settings->tool_table[pocket].offset.w = PROGRAM_TO_USER_LEN(w);
    }

    if(block->r_flag) settings->tool_table[pocket].diameter = PROGRAM_TO_USER_LEN(block->r_number) * 2.;
    if(block->i_flag) settings->tool_table[pocket].frontangle = block->i_number;
    if(block->j_flag) settings->tool_table[pocket].backangle = block->j_number;
    if(block->q_number != -1.0) {
        CHKS((!is_near_int(&q, block->q_number)), "Q number in G10 is not an integer");
        CHKS((q > 9), "Invalid tool orientation");
        settings->tool_table[pocket].orientation = q;
    }

    SET_TOOL_TABLE_ENTRY(pocket,
                             settings->tool_table[pocket].toolno,
                             settings->tool_table[pocket].offset,
                             settings->tool_table[pocket].diameter,
                             settings->tool_table[pocket].frontangle,
                             settings->tool_table[pocket].backangle,
                             settings->tool_table[pocket].orientation);

    if(settings->current_pocket == pocket) {
       settings->tool_table[0] = settings->tool_table[pocket];
    }

    if (settings->tool_table[0].toolno < 0) {
      settings->parameters[5400] = 0; // -1 ==> notool
    } else {
      settings->parameters[5400] = settings->tool_table[0].toolno;
    }
    settings->parameters[5401] = settings->tool_table[0].offset.tran.x;
    settings->parameters[5402] = settings->tool_table[0].offset.tran.y;
    settings->parameters[5403] = settings->tool_table[0].offset.tran.z;
    settings->parameters[5404] = settings->tool_table[0].offset.a;
    settings->parameters[5405] = settings->tool_table[0].offset.b;
    settings->parameters[5406] = settings->tool_table[0].offset.c;
    settings->parameters[5407] = settings->tool_table[0].offset.u;
    settings->parameters[5408] = settings->tool_table[0].offset.v;
    settings->parameters[5409] = settings->tool_table[0].offset.w;
    settings->parameters[5410] = settings->tool_table[0].diameter;
    settings->parameters[5411] = settings->tool_table[0].frontangle;
    settings->parameters[5412] = settings->tool_table[0].backangle;
    settings->parameters[5413] = settings->tool_table[0].orientation;

    //persuade axis-gui to update parameters widget for current tool:
    if (   !_setup.random_toolchanger
        && toolno == settings->current_pocket) {
        SET_TOOL_TABLE_ENTRY(0,
                             settings->tool_table[pocket].toolno,
                             settings->tool_table[pocket].offset,
                             settings->tool_table[pocket].diameter,
                             settings->tool_table[pocket].frontangle,
                             settings->tool_table[pocket].backangle,
                             settings->tool_table[pocket].orientation);
    }

    return INTERP_OK;
}

/****************************************************************************/

/*! convert_setup

Returned Value: int (INTERP_OK)

Side effects:
   SET_PROGRAM_ORIGIN is called, and the coordinate
   values for the program origin are reset.
   If the program origin is currently in use, the values of the
   the coordinates of the current point are updated.

Called by: convert_modal_0.

This is called only if g10 is called. g10 L2 may be used to alter the
location of coordinate systems as described in [NCMS, pages 9 - 10] and
[Fanuc, page 65]. [Fanuc] has only six coordinate systems, while
[NCMS] has nine (the first six of which are the same as the six [Fanuc]
has). All nine are implemented here.

Being in incremental distance mode has no effect on the action of G10
in this implementation. The manual is not explicit about what is
intended.

If L is 20 instead of 2, the coordinates are relative to the current
position instead of the origin.  Like how G92 offsets are programmed,
the meaning is "set the coordinate system origin such that my current
position becomes the specified value".

See documentation of convert_coordinate_system for more information.

*/

int Interp::convert_setup(block_pointer block,   //!< pointer to a block of RS274/NGC instructions
                         setup_pointer settings)        //!< pointer to machine settings                 
{
  double x;
  double y;
  double z;
  double a;
  double b;
  double c;
  double u, v, w;
  double r;
  double *parameters;
  int p_int;

  double cx, cy, cz, ca, cb, cc, cu, cv, cw;

  CHKS((block->i_flag || block->j_flag), EMC_I18N("I J words not allowed with G10 L2"));

  parameters = settings->parameters;
  p_int = (int) (block->p_number + 0.0001);

  CHKS((block->l_number == 20 && block->a_flag && settings->a_axis_wrapped && 
        (block->a_number <= -360.0 || block->a_number >= 360.0)), 
       (EMC_I18N("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->a_number, 'A');
  CHKS((block->l_number == 20 && block->b_flag && settings->b_axis_wrapped && 
        (block->b_number <= -360.0 || block->b_number >= 360.0)), 
       (EMC_I18N("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->b_number, 'B');
  CHKS((block->l_number == 20 && block->c_flag && settings->c_axis_wrapped && 
        (block->c_number <= -360.0 || block->c_number >= 360.0)), 
       (EMC_I18N("Invalid absolute position %5.2f for wrapped rotary axis %c")), block->c_number, 'C');

  find_current_in_system(settings, p_int,
                         &cx, &cy, &cz,
                         &ca, &cb, &cc,
                         &cu, &cv, &cw);

  if (block->r_flag == ON) {
    CHKS((block->l_number == 20), "R not allowed in G10 L20");
    r = block->r_number;
    parameters[5210 + (p_int * 20)] = r;
  } else
    r = parameters[5210 + (p_int * 20)];

  if (block->l_number == 20) {
      // old position in rotated system
      double oldx = cx, oldy = cy;

      // find new desired position in rotated system
      x = cx;
      y = cy;

      if (block->x_flag == ON) {
          x = block->x_number;
      }      
      if (block->y_flag == ON) {
          y = block->y_number;
      }

      // move old current position into the unrotated system
      rotate(&oldx, &oldy, r);
      // move desired position into the unrotated system
      rotate(&x, &y, r);

      // find new offset
      x = oldx + USER_TO_PROGRAM_LEN(parameters[5201 + (p_int * 20)]) - x;
      y = oldy + USER_TO_PROGRAM_LEN(parameters[5202 + (p_int * 20)]) - y;

      // parameters are not rotated
      parameters[5201 + (p_int * 20)] = PROGRAM_TO_USER_LEN(x);
      parameters[5202 + (p_int * 20)] = PROGRAM_TO_USER_LEN(y);

      if (p_int == settings->origin_index) {
          // let the code below fix up the current coordinates correctly
          rotate(&settings->current_x, &settings->current_y, settings->rotation_xy);
          settings->rotation_xy = 0;
      }
  } else {
      if (block->x_flag == ON) {
          x = block->x_number;
          parameters[5201 + (p_int * 20)] = PROGRAM_TO_USER_LEN(x);
      } else {
          x = USER_TO_PROGRAM_LEN(parameters[5201 + (p_int * 20)]);
      }
      if (block->y_flag == ON) {
          y = block->y_number;
          parameters[5202 + (p_int * 20)] = PROGRAM_TO_USER_LEN(y);
      } else {
          y = USER_TO_PROGRAM_LEN(parameters[5202 + (p_int * 20)]);
      }
  }

  if (block->z_flag == ON) {
    z = block->z_number;
    if (block->l_number == 20) z = cz + USER_TO_PROGRAM_LEN(parameters[5203 + (p_int * 20)]) - z;
    parameters[5203 + (p_int * 20)] = PROGRAM_TO_USER_LEN(z);
  } else
    z = USER_TO_PROGRAM_LEN(parameters[5203 + (p_int * 20)]);

  if (block->a_flag == ON) {
    a = block->a_number;
    if (block->l_number == 20) a = ca + USER_TO_PROGRAM_ANG(parameters[5204 + (p_int * 20)]) - a;
    parameters[5204 + (p_int * 20)] = PROGRAM_TO_USER_ANG(a);
  } else
    a = USER_TO_PROGRAM_ANG(parameters[5204 + (p_int * 20)]);

  if (block->b_flag == ON) {
    b = block->b_number;
    if (block->l_number == 20) b = cb + USER_TO_PROGRAM_ANG(parameters[5205 + (p_int * 20)]) - b;
    parameters[5205 + (p_int * 20)] = PROGRAM_TO_USER_ANG(b);
  } else
    b = USER_TO_PROGRAM_ANG(parameters[5205 + (p_int * 20)]);

  if (block->c_flag == ON) {
    c = block->c_number;
    if (block->l_number == 20) c = cc + USER_TO_PROGRAM_ANG(parameters[5206 + (p_int * 20)]) - c;
    parameters[5206 + (p_int * 20)] = PROGRAM_TO_USER_ANG(c);
  } else
    c = USER_TO_PROGRAM_ANG(parameters[5206 + (p_int * 20)]);

  if (block->u_flag == ON) {
    u = block->u_number;
    if (block->l_number == 20) u = cu + USER_TO_PROGRAM_LEN(parameters[5207 + (p_int * 20)]) - u;
    parameters[5207 + (p_int * 20)] = PROGRAM_TO_USER_LEN(u);
  } else
    u = USER_TO_PROGRAM_LEN(parameters[5207 + (p_int * 20)]);

  if (block->v_flag == ON) {
    v = block->v_number;
    if (block->l_number == 20) v = cv + USER_TO_PROGRAM_LEN(parameters[5208 + (p_int * 20)]) - v;
    parameters[5208 + (p_int * 20)] = PROGRAM_TO_USER_LEN(v);
  } else
    v = USER_TO_PROGRAM_LEN(parameters[5208 + (p_int * 20)]);

  if (block->w_flag == ON) {
    w = block->w_number;
    if (block->l_number == 20) w = cw + USER_TO_PROGRAM_LEN(parameters[5209 + (p_int * 20)]) - w;
    parameters[5209 + (p_int * 20)] = PROGRAM_TO_USER_LEN(w);
  } else
    w = USER_TO_PROGRAM_LEN(parameters[5209 + (p_int * 20)]);

  if (p_int == settings->origin_index) {        /* system is currently used */
      
    settings->current_x += settings->origin_offset_x;
    settings->current_y += settings->origin_offset_y;
    settings->current_z += settings->origin_offset_z;
    settings->AA_current += settings->AA_origin_offset;
    settings->BB_current += settings->BB_origin_offset;
    settings->CC_current += settings->CC_origin_offset;
    settings->u_current += settings->u_origin_offset;
    settings->v_current += settings->v_origin_offset;
    settings->w_current += settings->w_origin_offset;

    settings->origin_offset_x = x;
    settings->origin_offset_y = y;
    settings->origin_offset_z = z;
    settings->AA_origin_offset = a;
    settings->BB_origin_offset = b;
    settings->CC_origin_offset = c;
    settings->u_origin_offset = u;
    settings->v_origin_offset = v;
    settings->w_origin_offset = w;

    settings->current_x -= x;
    settings->current_y -= y;
    settings->current_z -= z;
    settings->AA_current -= a;
    settings->BB_current -= b;
    settings->CC_current -= c;
    settings->u_current -= u;
    settings->v_current -= v;
    settings->w_current -= w;

    SET_ORIGIN_OFFSETS(x + settings->axis_offset_x,
                       y + settings->axis_offset_y,
                       z + settings->axis_offset_z,
                       a + settings->AA_axis_offset,
                       b + settings->BB_axis_offset,
                       c + settings->CC_axis_offset,
                       u + settings->u_axis_offset,
                       v + settings->v_axis_offset,
                       w + settings->w_axis_offset);

    // current_xy are relative to this sytem's origin, so we can rotate directly
    rotate(&settings->current_x, &settings->current_y, settings->rotation_xy - r);
    settings->rotation_xy = r;
    SET_XY_ROTATION(settings->rotation_xy);

  }
#ifdef DEBUG_EMC
  else
    enqueue_COMMENT("interpreter: setting coordinate system origin");
#endif
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_set_plane

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The user tries to change to a different plane while comp is on:
      NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);
   2. The g_code is not G_17, G_18, or G_19:
      NCE_BUG_CODE_NOT_G17_G18_OR_G19

Side effects:
   A canonical command setting the current plane is executed.

Called by: convert_g.

*/

int Interp::convert_set_plane(int g_code,        //!< must be G_17, G_18, or G_19 
                             setup_pointer settings)    //!< pointer to machine settings 
{
  CHKS((settings->cutter_comp_side != OFF && g_code == G_17 && settings->plane != CANON_PLANE_XY),
        NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);
  CHKS((settings->cutter_comp_side != OFF && g_code == G_18 && settings->plane != CANON_PLANE_XZ),
        NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);
  CHKS((settings->cutter_comp_side != OFF && g_code == G_19 && settings->plane != CANON_PLANE_YZ),
        NCE_CANNOT_CHANGE_PLANES_WITH_CUTTER_RADIUS_COMP_ON);

  CHKS((settings->cutter_comp_side != OFF && g_code == G_19), 
          NCE_RADIUS_COMP_ONLY_IN_XY_OR_XZ);

  if (g_code == G_17) {
    SELECT_PLANE(CANON_PLANE_XY);
    settings->plane = CANON_PLANE_XY;
  } else if (g_code == G_18) {
    SELECT_PLANE(CANON_PLANE_XZ);
    settings->plane = CANON_PLANE_XZ;
  } else if (g_code == G_19) {
    SELECT_PLANE(CANON_PLANE_YZ);
    settings->plane = CANON_PLANE_YZ;
  } else if (g_code == G_17_1) {
    SELECT_PLANE(CANON_PLANE_UV);
    settings->plane = CANON_PLANE_UV;
  } else if (g_code == G_18_1) {
    SELECT_PLANE(CANON_PLANE_UW);
    settings->plane = CANON_PLANE_UW;
  } else if (g_code == G_19_1) {
    SELECT_PLANE(CANON_PLANE_VW);
    settings->plane = CANON_PLANE_VW;
  } else
    ERS(NCE_BUG_CODE_NOT_G17_G18_OR_G19);
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_speed

Returned Value: int (INTERP_OK)

Side effects:
  The machine spindle speed is set to the value of s_number in the
  block by a call to SET_SPINDLE_SPEED.
  The machine model for spindle speed is set to that value.

Called by: execute_block.

*/

int Interp::convert_speed(block_pointer block,   //!< pointer to a block of RS274 instructions
                         setup_pointer settings)        //!< pointer to machine settings             
{
  enqueue_SET_SPINDLE_SPEED(block->s_number);
  settings->speed = block->s_number;
  return INTERP_OK;
}

int Interp::convert_spindle_mode(block_pointer block, setup_pointer settings)
{
    if(block->g_modes[14] == G_97) {
	enqueue_SET_SPINDLE_MODE(0);
    } else { /* G_96 */
	if(block->d_flag)
	    enqueue_SET_SPINDLE_MODE(block->d_number_float);
	else
	    enqueue_SET_SPINDLE_MODE(1e30);
    }
    return INTERP_OK;
}
/****************************************************************************/

/*! convert_stop

Returned Value: int
   When an m2 or m30 (program_end) is encountered, this returns INTERP_EXIT.
   If the code is not m0, m1, m2, m30, or m60, this returns
   NCE_BUG_CODE_NOT_M0_M1_M2_M30_M60
   Otherwise, it returns INTERP_OK.

Side effects:
   An m0, m1, m2, m30, or m60 in the block is executed.

   For m0, m1, and m60, this makes a function call to the PROGRAM_STOP
   canonical machining function (which stops program execution).
   In addition, m60 calls PALLET_SHUTTLE.

   For m2 and m30, this resets the machine and then calls PROGRAM_END.
   In addition, m30 calls PALLET_SHUTTLE.

Called by: execute_block.

This handles stopping or ending the program (m0, m1, m2, m30, m60)

[NCMS] specifies how the following modes should be reset at m2 or
m30. The descriptions are not collected in one place, so this list
may be incomplete.

G52 offsetting coordinate zero points [NCMS, page 10]
G92 coordinate offset using tool position [NCMS, page 10]

The following should have reset values, but no description of reset
behavior could be found in [NCMS].
G17, G18, G19 selected plane [NCMS, pages 14, 20]
G90, G91 distance mode [NCMS, page 15]
G93, G94 feed mode [NCMS, pages 35 - 37]
M48, M49 overrides enabled, disabled [NCMS, pages 37 - 38]
M3, M4, M5 spindle turning [NCMS, page 7]

The following should be set to some value at machine start-up but
not automatically reset by any of the stopping codes.
1. G20, G21 length units [NCMS, page 15]. This is up to the installer.
2. motion_control_mode. This is set in Interp::init but not reset here.
   Might add it here.

The following resets have been added by calling the appropriate
canonical machining command and/or by resetting interpreter
settings. They occur on M2 or M30.

1. Axis offsets are set to zero (like g92.2) and      - SET_ORIGIN_OFFSETS
   origin offsets are set to the default (like G54)
2. Selected plane is set to CANON_PLANE_XY (like G17) - SELECT_PLANE
3. Distance mode is set to MODE_ABSOLUTE (like G90)   - no canonical call
4. Feed mode is set to UNITS_PER_MINUTE (like G94)    - no canonical call
5. Feed and speed overrides are set to ON (like M48)  - ENABLE_FEED_OVERRIDE
                                                      - ENABLE_SPEED_OVERRIDE
6. Cutter compensation is turned off (like G40)       - no canonical call
7. The spindle is stopped (like M5)                   - STOP_SPINDLE_TURNING
8. The motion mode is set to G_1 (like G1)            - no canonical call
9. Coolant is turned off (like M9)                    - FLOOD_OFF & MIST_OFF

*/

int Interp::convert_stop(block_pointer block,    //!< pointer to a block of RS274/NGC instructions
                        setup_pointer settings) //!< pointer to machine settings                 
{
  int index;
  char *line;
  int length;

  double cx, cy, cz;
  comp_get_current(settings, &cx, &cy, &cz);
  CHP(move_endpoint_and_flush(settings, cx, cy));
  dequeue_canons(settings);

  if (block->m_modes[4] == 0) {
    PROGRAM_STOP();
  } else if (block->m_modes[4] == 60) {
    PALLET_SHUTTLE();
    PROGRAM_STOP();
  } else if (block->m_modes[4] == 1) {
    OPTIONAL_PROGRAM_STOP();
  } else if ((block->m_modes[4] == 2) || (block->m_modes[4] == 30)) {   /* reset stuff here */
/*1*/
    settings->current_x += settings->origin_offset_x;
    settings->current_y += settings->origin_offset_y;
    settings->current_z += settings->origin_offset_z;
    settings->AA_current += settings->AA_origin_offset;
    settings->BB_current += settings->BB_origin_offset;
    settings->CC_current += settings->CC_origin_offset;
    settings->u_current += settings->u_origin_offset;
    settings->v_current += settings->v_origin_offset;
    settings->w_current += settings->w_origin_offset;
    rotate(&settings->current_x, &settings->current_y, settings->rotation_xy);

    settings->origin_index = 1;
    settings->parameters[5220] = 1.0;
    settings->origin_offset_x = USER_TO_PROGRAM_LEN(settings->parameters[5221]);
    settings->origin_offset_y = USER_TO_PROGRAM_LEN(settings->parameters[5222]);
    settings->origin_offset_z = USER_TO_PROGRAM_LEN(settings->parameters[5223]);
    settings->AA_origin_offset = USER_TO_PROGRAM_ANG(settings->parameters[5224]);
    settings->BB_origin_offset = USER_TO_PROGRAM_ANG(settings->parameters[5225]);
    settings->CC_origin_offset = USER_TO_PROGRAM_ANG(settings->parameters[5226]);
    settings->u_origin_offset = USER_TO_PROGRAM_LEN(settings->parameters[5227]);
    settings->v_origin_offset = USER_TO_PROGRAM_LEN(settings->parameters[5228]);
    settings->w_origin_offset = USER_TO_PROGRAM_LEN(settings->parameters[5229]);
    settings->rotation_xy = settings->parameters[5230];

    rotate(&settings->current_x, &settings->current_y, -settings->rotation_xy);
    settings->current_x -= settings->origin_offset_x;
    settings->current_y -= settings->origin_offset_y;
    settings->current_z -= settings->origin_offset_z;
    settings->AA_current -= settings->AA_origin_offset;
    settings->BB_current -= settings->BB_origin_offset;
    settings->CC_current -= settings->CC_origin_offset;
    settings->u_current -= settings->u_origin_offset;
    settings->v_current -= settings->v_origin_offset;
    settings->w_current -= settings->w_origin_offset;

    SET_ORIGIN_OFFSETS(settings->origin_offset_x + settings->axis_offset_x,
                       settings->origin_offset_y + settings->axis_offset_y,
                       settings->origin_offset_z + settings->axis_offset_z,
                       settings->AA_origin_offset + settings->AA_axis_offset,
                       settings->BB_origin_offset + settings->BB_axis_offset,
                       settings->CC_origin_offset + settings->CC_axis_offset,
                       settings->u_origin_offset + settings->u_axis_offset,
                       settings->v_origin_offset + settings->v_axis_offset,
                       settings->w_origin_offset + settings->w_axis_offset);
    SET_XY_ROTATION(settings->rotation_xy);

/*2*/ if (settings->plane != CANON_PLANE_XY) {
      SELECT_PLANE(CANON_PLANE_XY);
      settings->plane = CANON_PLANE_XY;
    }

/*3*/
    settings->distance_mode = MODE_ABSOLUTE;

/*4*/ settings->feed_mode = UNITS_PER_MINUTE;
    SET_FEED_MODE(0);
    settings->feed_rate = block->f_number;
    SET_FEED_RATE(0);

/*5*/ if (settings->feed_override != ON) {
      ENABLE_FEED_OVERRIDE();
      settings->feed_override = ON;
    }
    if (settings->speed_override != ON) {
      ENABLE_SPEED_OVERRIDE();
      settings->speed_override = ON;
    }

/*6*/
    settings->cutter_comp_side = OFF;
    settings->cutter_comp_firstmove = ON;

/*7*/ STOP_SPINDLE_TURNING();
    settings->spindle_turning = CANON_STOPPED;

    /* turn off FPR */
    SET_SPINDLE_MODE(0);

/*8*/ settings->motion_mode = G_1;

/*9*/ if (settings->mist == ON) {
      MIST_OFF();
      settings->mist = OFF;
    }
    if (settings->flood == ON) {
      FLOOD_OFF();
      settings->flood = OFF;
    }

    if (block->m_modes[4] == 30)
      PALLET_SHUTTLE();
    PROGRAM_END();
    if (_setup.percent_flag == ON && _setup.file_pointer) {
      line = _setup.linetext;
      for (;;) {                /* check for ending percent sign and comment if missing */
        if (fgets(line, LINELEN, _setup.file_pointer) == NULL) {
          enqueue_COMMENT("interpreter: percent sign missing from end of file");
          break;
        }
        length = strlen(line);
        if (length == (LINELEN - 1)) {       // line is too long. need to finish reading the line
          for (; fgetc(_setup.file_pointer) != '\n';);
          continue;
        }
        for (index = (length - 1);      // index set on last char
             (index >= 0) && (isspace(line[index])); index--);
        if (line[index] == '%') // found line with % at end
        {
          for (index--; (index >= 0) && (isspace(line[index])); index--);
          if (index == -1)      // found line with only percent sign
            break;
        }
      }
    }
    return INTERP_EXIT;
  } else
    ERS(NCE_BUG_CODE_NOT_M0_M1_M2_M30_M60);
  return INTERP_OK;
}

/*************************************************************************** */

/*! convert_straight

Returned Value: int
   If convert_straight_comp1 or convert_straight_comp2 is called
   and returns an error code, this returns that code.
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The value of move is not G_0 or G_1:
      NCE_BUG_CODE_NOT_G0_OR_G1
   2. A straight feed (g1) move is called with feed rate set to 0:
      NCE_CANNOT_DO_G1_WITH_ZERO_FEED_RATE
   3. A straight feed (g1) move is called with inverse time feed in effect
      but no f word (feed time) is provided:
      NCE_F_WORD_MISSING_WITH_INVERSE_TIME_G1_MOVE
   4. A move is called with G53 and cutter radius compensation on:
      NCE_CANNOT_USE_G53_WITH_CUTTER_RADIUS_COMP
   5. A G33 move is called without the necessary support compiled in:
      NCE_G33_NOT_SUPPORTED

Side effects:
   This executes a STRAIGHT_FEED command at cutting feed rate
   (if move is G_1) or a STRAIGHT_TRAVERSE command (if move is G_0).
   It also updates the setting of the position of the tool point to the
   end point of the move. If cutter radius compensation is on, it may
   also generate an arc before the straight move. Also, in INVERSE_TIME
   feed mode, SET_FEED_RATE will be called the feed rate setting changed.

Called by: convert_motion.

The approach to operating in incremental distance mode (g91) is to
put the the absolute position values into the block before using the
block to generate a move.

In inverse time feed mode, a lower bound of 0.1 is placed on the feed
rate so that the feed rate is never set to zero. If the destination
point is the same as the current point, the feed rate would be
calculated as zero otherwise.

If cutter compensation is in use, the path's length may increase or
decrease.  Also an arc may be added, to go around a corner, before the
straight move.  For the purpose of calculating the feed rate when in
inverse time mode, this length increase or decrease is ignored.  The
feed is still set to the original programmed straight length divided by
the F number (with the above lower bound).  The new arc (if needed) and
the new longer or shorter straight move are taken at this feed.

*/


int Interp::convert_straight(int move,   //!< either G_0 or G_1                       
                            block_pointer block,        //!< pointer to a block of RS274 instructions
                            setup_pointer settings)     //!< pointer to machine settings             
{
  double end_x;
  double end_y;
  double end_z;
  double AA_end;
  double BB_end;
  double CC_end;
  double u_end, v_end, w_end;
  int status;

  settings->arc_not_allowed = OFF;

  if (move == G_1) {
    if (settings->feed_mode == UNITS_PER_MINUTE) {
      CHKS((settings->feed_rate == 0.0), NCE_CANNOT_DO_G1_WITH_ZERO_FEED_RATE);
    } else if (settings->feed_mode == UNITS_PER_REVOLUTION) {
      CHKS((settings->feed_rate == 0.0), NCE_CANNOT_DO_G1_WITH_ZERO_FEED_RATE);
      CHKS((settings->speed == 0.0), "Cannot feed with zero spindle speed in feed per rev mode");
    } else if (settings->feed_mode == INVERSE_TIME) {
      CHKS((block->f_number == -1.0),
          NCE_F_WORD_MISSING_WITH_INVERSE_TIME_G1_MOVE);
    }
  }

  settings->motion_mode = move;
  CHP(find_ends(block, settings, &end_x, &end_y, &end_z,
                &AA_end, &BB_end, &CC_end, &u_end, &v_end, &w_end));

  if (move == G_1) {
      inverse_time_rate_straight(end_x, end_y, end_z,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end,
                                 block, settings);
  }

  if ((settings->cutter_comp_side != OFF) &&    /* ! "== ON" */
      (settings->cutter_comp_radius > 0.0)) {   /* radius always is >= 0 */

    CHKS((block->g_modes[0] == G_53),
        NCE_CANNOT_USE_G53_WITH_CUTTER_RADIUS_COMP);

    if(settings->plane == CANON_PLANE_XZ) {
        if (settings->cutter_comp_firstmove == ON)
            status = convert_straight_comp1(move, block, settings, end_z, end_x, end_y,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
        else
            status = convert_straight_comp2(move, block, settings, end_z, end_x, end_y,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
    } else if(settings->plane == CANON_PLANE_XY) {
        if (settings->cutter_comp_firstmove == ON)
            status = convert_straight_comp1(move, block, settings, end_x, end_y, end_z,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
        else
            status = convert_straight_comp2(move, block, settings, end_x, end_y, end_z,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
    } else ERS("BUG: Invalid plane for cutter compensation");
    CHP(status);
  } else if (move == G_0) {
    STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z,
                      AA_end, BB_end, CC_end,
                      u_end, v_end, w_end);
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
  } else if (move == G_1) {
    STRAIGHT_FEED(block->line_number, end_x, end_y, end_z,
                  AA_end, BB_end, CC_end,
                  u_end, v_end, w_end);
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
  } else if (move == G_33) {
    CHKS(((settings->spindle_turning != CANON_CLOCKWISE) &&
           (settings->spindle_turning != CANON_COUNTERCLOCKWISE)),
          "Spindle not turning in G33");
    START_SPEED_FEED_SYNCH(block->k_number, 0);
    STRAIGHT_FEED(block->line_number, end_x, end_y, end_z, AA_end, BB_end, CC_end, u_end, v_end, w_end);
    STOP_SPEED_FEED_SYNCH();
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
  } else if (move == G_33_1) {
    CHKS(((settings->spindle_turning != CANON_CLOCKWISE) &&
           (settings->spindle_turning != CANON_COUNTERCLOCKWISE)),
          "Spindle not turning in G33.1");
    START_SPEED_FEED_SYNCH(block->k_number, 0);
    RIGID_TAP(block->line_number, end_x, end_y, end_z);
    STOP_SPEED_FEED_SYNCH();
    // after the RIGID_TAP cycle we'll be in the same spot
  } else if (move == G_76) {
    CHKS((settings->AA_current != AA_end || 
         settings->BB_current != BB_end || 
         settings->CC_current != CC_end ||
         settings->u_current != u_end ||
         settings->v_current != v_end ||
         settings->w_current != w_end), NCE_CANNOT_MOVE_ROTARY_AXES_WITH_G76);
    int result = convert_threading_cycle(block, settings, end_x, end_y, end_z);
    if(result != INTERP_OK) return result;
  } else
    ERS(NCE_BUG_CODE_NOT_G0_OR_G1);

  settings->AA_current = AA_end;
  settings->BB_current = BB_end;
  settings->CC_current = CC_end;
  settings->u_current = u_end;
  settings->v_current = v_end;
  settings->w_current = w_end;
  return INTERP_OK;
}

#define AABBCC settings->AA_current, settings->BB_current, settings->CC_current, settings->u_current, settings->v_current, settings->w_current

// make one threading pass.  only called from convert_threading_cycle.
static void 
threading_pass(setup_pointer settings, block_pointer block,
	       int boring, double safe_x, double depth, double end_depth, 
	       double start_y, double start_z, double zoff, double taper_dist,
	       int entry_taper, int exit_taper, double taper_pitch, 
	       double pitch, double full_threadheight, double target_z) {
    STRAIGHT_TRAVERSE(block->line_number, boring?
		      safe_x + depth - end_depth:
		      safe_x - depth + end_depth,
		      start_y, start_z - zoff, AABBCC); //back
    if(taper_dist && entry_taper) {
	DISABLE_FEED_OVERRIDE();
	START_SPEED_FEED_SYNCH(taper_pitch, 0);
	STRAIGHT_FEED(block->line_number, boring? 
		      safe_x + depth - full_threadheight: 
		      safe_x - depth + full_threadheight,
		      start_y, start_z - zoff, AABBCC); //in
	STRAIGHT_FEED(block->line_number, boring? safe_x + depth: safe_x - depth, //angled in
		      start_y, start_z - zoff - taper_dist, AABBCC);
	START_SPEED_FEED_SYNCH(pitch, 0);
    } else {
	STRAIGHT_TRAVERSE(block->line_number, boring? safe_x + depth: safe_x - depth, 
			  start_y, start_z - zoff, AABBCC); //in
	DISABLE_FEED_OVERRIDE();
	START_SPEED_FEED_SYNCH(pitch, 0);
    }
        
    if(taper_dist && exit_taper) {
	STRAIGHT_FEED(block->line_number, boring? safe_x + depth: safe_x - depth,  //over
		      start_y, target_z - zoff + taper_dist, AABBCC);
	START_SPEED_FEED_SYNCH(taper_pitch, 0);
	STRAIGHT_FEED(block->line_number, boring? 
		      safe_x + depth - full_threadheight: 
		      safe_x - depth + full_threadheight, 
		      start_y, target_z - zoff, AABBCC); //angled out
    } else {
	STRAIGHT_FEED(block->line_number, boring? safe_x + depth: safe_x - depth, 
		      start_y, target_z - zoff, AABBCC); //over
    }
    STOP_SPEED_FEED_SYNCH();
    STRAIGHT_TRAVERSE(block->line_number, boring? 
		      safe_x + depth - end_depth:
		      safe_x - depth + end_depth,
		      start_y, target_z - zoff, AABBCC); //out
    ENABLE_FEED_OVERRIDE();
}

int Interp::convert_threading_cycle(block_pointer block, 
				    setup_pointer settings,
				    double end_x, double end_y, double end_z) {


    CHKS((settings->cutter_comp_side != OFF),
         (EMC_I18N("Cannot use G76 threading cycle with cutter radius compensation on")));

    CHKS((block->i_number == 0),
        (EMC_I18N("In G76, I must not be 0")));
    CHKS((block->j_number <= 0),
        (EMC_I18N("In G76, J must be greater than 0")));
    CHKS((block->k_number <= block->j_number),
        (EMC_I18N("In G76, K must be greater than J")));

    double start_x = settings->current_x;
    double start_y = settings->current_y;
    double start_z = settings->current_z;

    double i_number = block->i_number;
    double j_number = block->j_number;
    double k_number = block->k_number;

    if(_setup.lathe_diameter_mode){
      i_number /= 2;
      j_number /= 2;
      k_number /= 2;
    }


    int boring = 0;

    if (i_number > 0.0)
	boring = 1;

    double safe_x = start_x;
    double full_dia_depth = fabs(i_number);
    double start_depth = fabs(i_number) + fabs(j_number);
    double cut_increment = fabs(j_number);
    double full_threadheight = fabs(k_number);
    double end_depth = fabs(k_number) + fabs(i_number);

    double pitch = block->p_number;
    double compound_angle = block->q_number;
    if(compound_angle == -1) compound_angle = 0;
    compound_angle *= M_PI/180.0;
    if(end_z > start_z) compound_angle = -compound_angle;

    int spring_cuts = block->h_flag == ON ? block->h_number: 0;

    double degression = block->r_number;
    if(degression < 1.0 || !block->r_flag) degression = 1.0;

    double taper_dist = block->e_flag? block->e_number: 0.0;
    if(taper_dist < 0.0) taper_dist = 0.0;
    double taper_pitch = taper_dist > 0.0? 
	pitch * hypot(taper_dist, full_threadheight)/taper_dist: pitch;

    if(end_z > start_z) taper_dist = -taper_dist;

    int taper_flags = block->l_number;
    if(taper_flags < 0) taper_flags = 0;

    int entry_taper = taper_flags & 1;
    int exit_taper = taper_flags & 2;

    double depth, zoff;
    int pass = 1;

    double target_z = end_z + fabs(k_number) * tan(compound_angle);

    depth = start_depth;
    zoff = (depth - full_dia_depth) * tan(compound_angle);
    while (depth < end_depth) {
	threading_pass(settings, block, boring, safe_x, depth, end_depth, start_y, 
		       start_z, zoff, taper_dist, entry_taper, exit_taper, 
		       taper_pitch, pitch, full_threadheight, target_z);
        depth = full_dia_depth + cut_increment * pow(++pass, 1.0/degression);
        zoff = (depth - full_dia_depth) * tan(compound_angle);
    } 
    // full specified depth now
    depth = end_depth;
    zoff = (depth - full_dia_depth) * tan(compound_angle);
    // cut at least once -- more if spring cuts.
    for(int i = 0; i<spring_cuts+1; i++) {
	threading_pass(settings, block, boring, safe_x, depth, end_depth, start_y, 
		       start_z, zoff, taper_dist, entry_taper, exit_taper, 
		       taper_pitch, pitch, full_threadheight, target_z);
    } 
    STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z, AABBCC);
    settings->current_x = end_x;
    settings->current_y = end_y;
    settings->current_z = end_z;
#undef AABBC
    return INTERP_OK;
}


/****************************************************************************/

/*! convert_straight_comp1

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The side is not RIGHT or LEFT:
      NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT
   2. The destination tangent point is not more than a tool radius
      away (indicating gouging): NCE_CUTTER_GOUGING_WITH_CUTTER_RADIUS_COMP
   3. The value of move is not G_0 or G_1
      NCE_BUG_CODE_NOT_G0_OR_G1

Side effects:
   This executes a STRAIGHT_MOVE command at cutting feed rate
   or a STRAIGHT_TRAVERSE command.
   It also updates the setting of the position of the tool point
   to the end point of the move and updates the programmed point.

Called by: convert_straight.

This is called if cutter radius compensation is on and
settings->cutter_comp_firstmove is ON, indicating that this is the
first move after cutter radius compensation is turned on.

The algorithm used here for determining the path is to draw a straight
line from the destination point which is tangent to a circle whose
center is at the current point and whose radius is the radius of the
cutter. The destination point of the cutter tip is then found as the
center of a circle of the same radius tangent to the tangent line at
the destination point.

*/

int Interp::convert_straight_comp1(int move,     //!< either G_0 or G_1                        
                                   block_pointer block,  //!< pointer to a block of RS274 instructions 
                                   setup_pointer settings,       //!< pointer to machine settings              
                                   double px,    //!< X coordinate of end point                
                                   double py,    //!< Y coordinate of end point                
                                   double pz,    //!< Z coordinate of end point                
                                   double AA_end,        //!< A coordinate of end point          
                                   double BB_end,        //!< B coordinate of end point          
                                   double CC_end,        //!< C coordinate of end point          
                                   double u_end, double v_end, double w_end)
{
    double alpha;
    double distance;
    double radius = settings->cutter_comp_radius; /* always will be positive */
    double end_x, end_y;

    int side = settings->cutter_comp_side;
    double cx, cy, cz;

    comp_get_current(settings, &cx, &cy, &cz);
    distance = hypot((px - cx), (py - cy));

    CHKS(((side != LEFT) && (side != RIGHT)), NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT);
    CHKS((distance <= radius), "Length of cutter compensation entry move is not greater than the tool radius");

    alpha = atan2(py - cy, px - cx) + (side == LEFT ? M_PI/2. : -M_PI/2.);

    end_x = (px + (radius * cos(alpha)));
    end_y = (py + (radius * sin(alpha)));

    // with these moves we don't need to record the direction vector.
    // they cannot get reversed because they are guaranteed to be long
    // enough.

    set_endpoint(cx, cy);

    if (move == G_0) {
        enqueue_STRAIGHT_TRAVERSE(settings, block->line_number, 
                                  cos(alpha), sin(alpha), 0, 
                                  end_x, end_y, pz,
                                  AA_end, BB_end, CC_end, u_end, v_end, w_end);
    }
    else if (move == G_1) {
        enqueue_STRAIGHT_FEED(settings, block->line_number, 
                              cos(alpha), sin(alpha), 0,
                              end_x, end_y, pz,
                              AA_end, BB_end, CC_end, u_end, v_end, w_end);
    } else
        ERS(NCE_BUG_CODE_NOT_G0_OR_G1);

    settings->cutter_comp_firstmove = OFF;

    comp_set_current(settings, end_x, end_y, pz);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u_end;
    settings->v_current = v_end;
    settings->w_current = w_end;
    comp_set_programmed(settings, px, py, pz);
    return INTERP_OK;
}
/****************************************************************************/

/*! convert_straight_comp2

Returned Value: int
   If any of the following errors occur, this returns the error shown.
   Otherwise, it returns INTERP_OK.
   1. The compensation side is not RIGHT or LEFT:
      NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT
   2. A concave corner is found:
      NCE_CONCAVE_CORNER_WITH_CUTTER_RADIUS_COMP

Side effects:
   This executes a STRAIGHT_FEED command at cutting feed rate
   or a STRAIGHT_TRAVERSE command.
   It also generates an ARC_FEED to go around a corner, if necessary.
   It also updates the setting of the position of the tool point to
   the end point of the move and updates the programmed point.

Called by: convert_straight.

This is called if cutter radius compensation is on and
settings->cutter_comp_firstmove is not ON, indicating that this is not
the first move after cutter radius compensation is turned on.

The algorithm used here is:
1. Determine the direction of the last motion. This is done by finding
   the direction of the line from the last programmed point to the
   current tool tip location. This line is a radius of the tool and is
   perpendicular to the direction of motion since the cutter is tangent
   to that direction.
2. Determine the direction of the programmed motion.
3. If there is a convex corner, insert an arc to go around the corner.
4. Find the destination point for the tool tip. The tool will be
   tangent to the line from the last programmed point to the present
   programmed point at the present programmed point.
5. Go in a straight line from the current tool tip location to the
   destination tool tip location.

This uses an angle tolerance of TOLERANCE_CONCAVE_CORNER (0.01 radian)
to determine if:
1) an illegal concave corner exists (tool will not fit into corner),
2) no arc is required to go around the corner (i.e. the current line
   is in the same direction as the end of the previous move), or
3) an arc is required to go around a convex corner and start off in
   a new direction.

If a rotary axis is moved in this block and an extra arc is required
to go around a sharp corner, all the rotary axis motion occurs on the
arc.  An alternative might be to distribute the rotary axis motion
over the arc and the straight move in proportion to their lengths.

If the Z-axis is moved in this block and an extra arc is required to
go around a sharp corner, all the Z-axis motion occurs on the straight
line and none on the extra arc.  An alternative might be to distribute
the Z-axis motion over the extra arc and the straight line in
proportion to their lengths.

This handles the case of there being no XY motion.

This handles G0 moves. Where an arc is inserted to round a corner in a
G1 move, no arc is inserted for a G0 move; a STRAIGHT_TRAVERSE is made
from the current point to the end point. The end point for a G0
move is the same as the end point for a G1 move, however.

*/

int Interp::convert_straight_comp2(int move,     //!< either G_0 or G_1                        
                                   block_pointer block,  //!< pointer to a block of RS274 instructions 
                                   setup_pointer settings,       //!< pointer to machine settings              
                                   double px,    //!< X coordinate of programmed end point     
                                   double py,    //!< Y coordinate of programmed end point     
                                   double pz,    //!< Z coordinate of end point                
                                   double AA_end,        //!< A coordinate of end point
                                   double BB_end,        //!< B coordinate of end point
                                   double CC_end,        //!< C coordinate of end point
                                   double u_end, double v_end, double w_end)
{
    double alpha;
    double beta;
    double end_x, end_y, end_z;                 /* x-coordinate of actual end point */
    double gamma;
    double mid_x, mid_y;                 /* x-coordinate of end of added arc, if needed */
    double radius;
    int side;
    double small_ = TOLERANCE_CONCAVE_CORNER;      /* radians, testing corners */
    double opx, opy, opz;      /* old programmed beginning point */
    double theta;
    double cx, cy, cz;
    int concave;

    comp_get_current(settings, &cx, &cy, &cz);
    comp_get_current(settings, &end_x, &end_y, &end_z);
    comp_get_programmed(settings, &opx, &opy, &opz);

    if ((py == opy) && (px == opx)) {     /* no XY motion */
        if (move == G_0) {
            enqueue_STRAIGHT_TRAVERSE(settings, block->line_number, 
                                      px - opx, py - opy, pz - opz, 
                                      cx, cy, pz,
                                      AA_end, BB_end, CC_end, u_end, v_end, w_end);
        } else if (move == G_1) {
            enqueue_STRAIGHT_FEED(settings, block->line_number, 
                                  px - opx, py - opy, pz - opz, 
                                  cx, cy, pz, AA_end, BB_end, CC_end, u_end, v_end, w_end);
        } else
            ERS(NCE_BUG_CODE_NOT_G0_OR_G1);
        // end already filled out, above
    } else {
        // some XY motion
        side = settings->cutter_comp_side;
        radius = settings->cutter_comp_radius;      /* will always be positive */
        theta = atan2(cy - opy, cx - opx);
        alpha = atan2(py - opy, px - opx);

        if (side == LEFT) {
            if (theta < alpha)
                theta = (theta + (2 * M_PI));
            beta = ((theta - alpha) - M_PI_2);
            gamma = M_PI_2;
        } else if (side == RIGHT) {
            if (alpha < theta)
                alpha = (alpha + (2 * M_PI));
            beta = ((alpha - theta) - M_PI_2);
            gamma = -M_PI_2;
        } else
            ERS(NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT);
        end_x = (px + (radius * cos(alpha + gamma)));
        end_y = (py + (radius * sin(alpha + gamma)));
        mid_x = (opx + (radius * cos(alpha + gamma)));
        mid_y = (opy + (radius * sin(alpha + gamma)));
    
        if ((beta < -small_) || (beta > (M_PI + small_))) {
            concave = 1;
        } else if (beta > (M_PI - small_) && 
                   (!qc().empty() && qc().front().type == QARC_FEED && 
                    ((side == RIGHT && qc().front().data.arc_feed.turn == 1) || 
                     (side == LEFT && qc().front().data.arc_feed.turn == -1)))) {
            // this is an "h" shape, tool on right, going right to left
            // over the hemispherical round part, then up next to the
            // vertical part (or, the mirror case).  there are two ways
            // to stay to the "right", either loop down and around, or
            // stay above and right.  we're forcing above and right.
            concave = 1;
        } else {
            concave = 0;
            mid_x = (opx + (radius * cos(alpha + gamma)));
            mid_y = (opy + (radius * sin(alpha + gamma)));
        }

        if (!concave && (beta > small_)) {       /* ARC NEEDED */
            CHP(move_endpoint_and_flush(settings, cx, cy));
            if(move == G_1) {
                enqueue_ARC_FEED(settings, block->line_number,
                                 0.0, // doesn't matter, since we will not move this arc's endpoint
                                 mid_x, mid_y, opx, opy,
                                 ((side == LEFT) ? -1 : 1), cz,
                                 AA_end, BB_end, CC_end, u_end, v_end, w_end);
                dequeue_canons(settings);
                set_endpoint(mid_x, mid_y);
            } else if(move == G_0) {
                // we can't go around the corner because there is no
                // arc traverse.  but, if we do this anyway, at least
                // most of our rapid will be parallel to the original
                // programmed one.  if nothing else, this will look a
                // little less confusing in the preview.
                enqueue_STRAIGHT_TRAVERSE(settings, block->line_number,
                                          0.0, 0.0, 0.0, 
                                          mid_x, mid_y, cz, 
                                          AA_end, BB_end, CC_end,
                                          u_end, v_end, w_end);
                dequeue_canons(settings);
                set_endpoint(mid_x, mid_y);
            } else ERS(NCE_BUG_CODE_NOT_G0_OR_G1);
        } else if (concave) {
            if (qc().front().type != QARC_FEED) {
                // line->line
                double retreat;
                // half the angle of the inside corner
                double halfcorner = (beta + M_PI) / 2.0;
                CHKS((halfcorner == 0.0), (EMC_I18N("Zero degree inside corner is invalid for cutter compensation")));
                retreat = radius / tan(halfcorner);
                // move back along the compensated path
                // this should replace the endpoint of the previous move
                mid_x = cx + retreat * cos(theta + gamma);
                mid_y = cy + retreat * sin(theta + gamma);
                // we actually want to move the previous line's endpoint here.  That's the same as 
                // discarding that line and doing this one instead.
                CHP(move_endpoint_and_flush(settings, mid_x, mid_y));
            } else {
                // arc->line
                // beware: the arc we saved is the compensated one.
                arc_feed prev = qc().front().data.arc_feed;
                double oldrad = hypot(prev.center2 - prev.end2, prev.center1 - prev.end1);
                double oldrad_uncomp;

                // new line's direction
                double base_dir = atan2(py - opy, px - opx);
                double theta;
                double phi;

                theta = (prev.turn == 1) ? base_dir + M_PI_2 : base_dir - M_PI_2;
                phi = atan2(prev.center2 - opy, prev.center1 - opx);
                if TOOL_INSIDE_ARC(side, prev.turn) {
                    oldrad_uncomp = oldrad + radius;
                } else {
                    oldrad_uncomp = oldrad - radius;
                }

                double alpha = theta - phi;
                // distance to old arc center perpendicular to the new line
                double d = oldrad_uncomp * cos(alpha);
                double d2;
                double angle_from_center;

                if TOOL_INSIDE_ARC(side, prev.turn) {
                    d2 = d - radius;
                    double l = d2/oldrad;
                    CHKS((l > 1.0 || l < -1.0), EMC_I18N("Arc to straight motion makes a corner the compensated tool can't fit in without gouging"));
                    if(prev.turn == 1) 
                        angle_from_center = - acos(l) + theta + M_PI;
                    else
                        angle_from_center = acos(l) + theta + M_PI;
                } else {
                    d2 = d + radius;
                    double l = d2/oldrad;
                    CHKS((l > 1.0 || l < -1.0), EMC_I18N("Arc to straight motion makes a corner the compensated tool can't fit in without gouging"));
                    if(prev.turn == 1) 
                        angle_from_center = acos(l) + theta + M_PI;
                    else
                        angle_from_center = - acos(l) + theta + M_PI;
                }
                mid_x = prev.center1 + oldrad * cos(angle_from_center);
                mid_y = prev.center2 + oldrad * sin(angle_from_center);
                CHP(move_endpoint_and_flush(settings, mid_x, mid_y));
            }
        } else {
            // no arc needed, also not concave (colinear lines or tangent arc->line)
            dequeue_canons(settings);
            set_endpoint(cx, cy);
        }
        (move == G_0? enqueue_STRAIGHT_TRAVERSE: enqueue_STRAIGHT_FEED)
            (settings, block->line_number, 
             px - opx, py - opy, pz - opz, 
             end_x, end_y, pz,
             AA_end, BB_end, CC_end, 
             u_end, v_end, w_end);
    }

    comp_set_current(settings, end_x, end_y, pz);
    settings->AA_current = AA_end;
    settings->BB_current = BB_end;
    settings->CC_current = CC_end;
    settings->u_current = u_end;
    settings->v_current = v_end;
    settings->w_current = w_end;
    comp_set_programmed(settings, px, py, pz);
    return INTERP_OK;
}

/****************************************************************************/

/*! convert_tool_change

Returned Value: int (INTERP_OK)

Side effects:
   This makes function calls to canonical machining functions, and sets
   the machine model as described below.

Called by: convert_m

This function carries out an M6 command, which changes the tool.
If M61 is called, the toolnumber gets changed (without causing an actual toolchange).

When the CHANGE_TOOL call completes, the specified tool should be
loaded.  What this means varies by machine.  According to configuration,
the interpreter may also issue commands to do one or more of the
following things before calling CHANGE_TOOL:

1. stop the spindle
2. move the quill up (Z to machine zero, like G0 G53 Z0)
3. move the axes to reference point #2 (like G30)

Further, the interpreter makes no assumptions about the axis positions
after the tool change completes.  This state is queried and the internal
model is resynched before the program continues.  This means CHANGE_TOOL
itself can also issue motion (and it currently may, according to
configuration).

This implements the "Next tool in T word" approach to tool selection.
The tool is selected when the T word is read (and the carousel may
move at that time) but is changed when M6 is read.

Note that if a different tool is put into the spindle, the current_z
location setting will be incorrect. It is assumed the program will
contain an appropriate USE_TOOL_LENGTH_OFFSET (G43) command before any
subsequent motion.  It is also assumed that the program will restart the
spindle and make new entry moves if necessary.

*/

int Interp::convert_tool_change(setup_pointer settings)  //!< pointer to machine settings
{

  if (settings->selected_pocket < 0) {
    ERS(NCE_TXX_MISSING_FOR_M6);
  }

  CHKS((settings->cutter_comp_side != OFF),
       (EMC_I18N("Cannot change tools with cutter radius compensation on")));

  if (!settings->tool_change_with_spindle_on) {
      STOP_SPINDLE_TURNING();
      settings->spindle_turning = CANON_STOPPED;
  }

  if (settings->tool_change_quill_up) {
      double up_z;
      double discard;
      find_relative(0., 0., 0., 0., 0., 0., 0., 0., 0., 
                    &discard, &discard, &up_z,
                    &discard, &discard, &discard, 
                    &discard, &discard, &discard,
                    settings);
      COMMENT("AXIS,hide");
      STRAIGHT_TRAVERSE(-1, settings->current_x, settings->current_y, up_z,
                        settings->AA_current, settings->BB_current, settings->CC_current,
                        settings->u_current, settings->v_current, settings->w_current);
      COMMENT("AXIS,show");
      settings->current_z = up_z;
  }

  if (settings->tool_change_at_g30) {
      double end_x;
      double end_y;
      double end_z;
      double AA_end;
      double BB_end;
      double CC_end;
      double u_end;
      double v_end;
      double w_end;

      find_relative(USER_TO_PROGRAM_LEN(settings->parameters[5181]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5182]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5183]),
                    USER_TO_PROGRAM_ANG(settings->parameters[5184]),
                    USER_TO_PROGRAM_ANG(settings->parameters[5185]),
                    USER_TO_PROGRAM_ANG(settings->parameters[5186]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5187]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5188]),
                    USER_TO_PROGRAM_LEN(settings->parameters[5189]),
                    &end_x, &end_y, &end_z,
                    &AA_end, &BB_end, &CC_end, 
                    &u_end, &v_end, &w_end, settings);
      COMMENT("AXIS,hide");
      STRAIGHT_TRAVERSE(-1, end_x, end_y, end_z,
                        AA_end, BB_end, CC_end,
                        u_end, v_end, w_end);
      COMMENT("AXIS,show");
      settings->current_x = end_x;
      settings->current_y = end_y;
      settings->current_z = end_z;
      settings->AA_current = AA_end;
      settings->BB_current = BB_end;
      settings->CC_current = CC_end;
      settings->u_current = u_end;
      settings->v_current = v_end;
      settings->w_current = w_end;
  }

  CHANGE_TOOL(settings->selected_pocket);

  settings->current_pocket = settings->selected_pocket;
  // tool change can move the controlled point.  reread it:
  settings->toolchange_flag = ON; 
  set_tool_parameters();
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_tool_length_offset

Returned Value: int
   If any of the following errors occur, this returns the error code shown.
   Otherwise, it returns INTERP_OK.
   1. The block has no offset index (h number): NCE_OFFSET_INDEX_MISSING
   2. The g_code argument is not G_43 or G_49:
      NCE_BUG_CODE_NOT_G43_OR_G49

Side effects:
   A USE_TOOL_LENGTH_OFFSET function call is made. Current_z,
   tool_length_offset, and length_offset_index are reset.

Called by: convert_g

This is called to execute g43 or g49.

The g49 RS274/NGC command translates into a USE_TOOL_LENGTH_OFFSET(0.0)
function call.

The g43 RS274/NGC command translates into a USE_TOOL_LENGTH_OFFSET(length)
function call, where length is the value of the entry in the tool length
offset table whose index is the H number in the block.

The H number in the block (if present) was checked for being a non-negative
integer when it was read, so that check does not need to be repeated.

*/

int Interp::convert_tool_length_offset(int g_code,       //!< g_code being executed (must be G_43 or G_49)
                                      block_pointer block,      //!< pointer to a block of RS274/NGC instructions
                                      setup_pointer settings)   //!< pointer to machine settings                 
{
  int index;
  EmcPose tool_offset;
  ZERO_EMC_POSE(tool_offset);

  CHKS((settings->cutter_comp_side != OFF),
       (EMC_I18N("Cannot change tool offset with cutter radius compensation on")));
  if (g_code == G_49) {
    index = 0;
  } else if (g_code == G_43) {
    if(block->h_flag == ON) {
        CHP((find_tool_pocket(settings, block->h_number, &index)));
    } else if (settings->toolchange_flag) {
        // we haven't loaded the tool and swapped pockets quite yet
        index = settings->current_pocket;
    } else {
        // tool change is done so pockets are swapped
        index = 0;
    }

    tool_offset.tran.x = USER_TO_PROGRAM_LEN(settings->tool_table[index].offset.tran.x);
    tool_offset.tran.y = USER_TO_PROGRAM_LEN(settings->tool_table[index].offset.tran.y);
    tool_offset.tran.z = USER_TO_PROGRAM_LEN(settings->tool_table[index].offset.tran.z);
    tool_offset.a = USER_TO_PROGRAM_ANG(settings->tool_table[index].offset.a);
    tool_offset.b = USER_TO_PROGRAM_ANG(settings->tool_table[index].offset.b);
    tool_offset.c = USER_TO_PROGRAM_ANG(settings->tool_table[index].offset.c);
    tool_offset.u = USER_TO_PROGRAM_LEN(settings->tool_table[index].offset.u);
    tool_offset.v = USER_TO_PROGRAM_LEN(settings->tool_table[index].offset.v);
    tool_offset.w = USER_TO_PROGRAM_LEN(settings->tool_table[index].offset.w);
  } else if (g_code == G_43_1) {
    tool_offset = settings->tool_offset;
    index = -1;
    if(block->x_flag == ON) tool_offset.tran.x = block->x_number;
    if(block->y_flag == ON) tool_offset.tran.y = block->y_number;
    if(block->z_flag == ON) tool_offset.tran.z = block->z_number;
    if(block->a_flag == ON) tool_offset.a = block->a_number;
    if(block->b_flag == ON) tool_offset.b = block->b_number;
    if(block->c_flag == ON) tool_offset.c = block->c_number;
    if(block->u_flag == ON) tool_offset.u = block->u_number;
    if(block->v_flag == ON) tool_offset.v = block->v_number;
    if(block->w_flag == ON) tool_offset.w = block->w_number;
  } else {
    ERS("BUG: Code not G43, G43.1, or G49");
  }
  USE_TOOL_LENGTH_OFFSET(tool_offset);

  settings->current_x += settings->tool_offset.tran.x - tool_offset.tran.x;
  settings->current_y += settings->tool_offset.tran.y - tool_offset.tran.y;
  settings->current_z += settings->tool_offset.tran.z - tool_offset.tran.z;
  settings->AA_current += settings->tool_offset.a - tool_offset.a;
  settings->BB_current += settings->tool_offset.b - tool_offset.b;
  settings->CC_current += settings->tool_offset.c - tool_offset.c;
  settings->u_current += settings->tool_offset.u - tool_offset.u;
  settings->v_current += settings->tool_offset.v - tool_offset.v;
  settings->w_current += settings->tool_offset.w - tool_offset.w;

  settings->tool_offset = tool_offset;
  settings->tool_offset_index = index;
  return INTERP_OK;
}

/****************************************************************************/

/*! convert_tool_select

Returned Value: int
   If the tool slot given in the block is larger than allowed,
   this returns NCE_SELECTED_TOOL_SLOT_NUMBER_TOO_LARGE.
   Otherwise, it returns INTERP_OK.

Side effects: See below

Called by: execute_block

A select tool command is given, which causes the changer chain to move
so that the slot with the t_number given in the block is next to the
tool changer, ready for a tool change.  The
settings->selected_tool_slot is set to the given slot.

An alternative in this function is to select by tool id. This was used
in the K&T and VGER interpreters. It is easy to code.

A check that the t_number is not negative has already been made in read_t.
A zero t_number is allowed and means no tool should be selected.

*/

// OK to select tool in a concave corner, I think?

int Interp::convert_tool_select(block_pointer block,     //!< pointer to a block of RS274 instructions
                               setup_pointer settings)  //!< pointer to machine settings             
{
  int pocket;
  CHP((find_tool_pocket(settings, block->t_number, &pocket)));
  SELECT_POCKET(pocket);
  settings->selected_pocket = pocket;
  return INTERP_OK;
}
