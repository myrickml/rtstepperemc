/********************************************************************
* emc.h - Support commands for EMC2
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef _EMC_H
#define _EMC_H

#include <pthread.h>
#include "motion.h"
#include "emc_msg.h"
#include "rtstepper.h"

#if (defined(__WIN32__) || defined(_WINDOWS))
   #define DLL_EXPORT __declspec(dllexport)
#else
   #define DLL_EXPORT __attribute__ ((visibility("default")))
#endif

struct emc_session
{
   int control_thread_active;
   int control_thread_abort;
   pthread_t control_thread_tid;
   pthread_mutex_t mutex;
   pthread_cond_t event_cond;
   pthread_cond_t control_thread_done_cond;
   pthread_cond_t control_cycle_thread_done_cond;
   int control_cycle_thread_active;
   pthread_t control_cycle_thread_tid;
   struct rtstepper_app_session dongle;
   struct _emc_msg_t head;
};

#include "msg.h"

enum EMC_RESULT
{
   EMC_R_TIMEOUT = -2,
   EMC_R_ERROR = -1,
   EMC_R_OK = 0,
};

enum EMC_UI_WAIT_TYPE
{
   EMC_UI_WAIT_NONE = 1,
   EMC_UI_WAIT_RECEIVED,
   EMC_UI_WAIT_DONE
};

// Localization helper.
#ifndef EMC_I18N
#define EMC_I18N(text) text
#endif

#define container_of(ptr, type, member) ({            \
 const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
 (type *)( (char *)__mptr - offsetof(type,member) );})

#define EMC_AXIS_MAX EMCMOT_MAX_AXIS
#define EMC_MAX_DIO EMCMOT_MAX_DIO
#define EMC_MAX_AIO EMCMOT_MAX_AIO
#define DEFAULT_RS274NGC_STARTUP_CODE ""

/* debug bitflags */
/* Note: these may be hard-code referenced by the GUI (e.g., emcdebug.tcl).
   If you change the assignments here, make sure and reflect that in
   the GUI scripts that use these. Unfortunately there's no easy way to
   get these into Tk automatically */
extern int EMC_DEBUG;
#define EMC_DEBUG_CONFIG            0x00000002
#define EMC_DEBUG_VERSIONS          0x00000008
#define EMC_DEBUG_TASK_ISSUE        0x00000010
#define EMC_DEBUG_NML               0x00000040
#define EMC_DEBUG_MOTION_TIME       0x00000080
#define EMC_DEBUG_INTERP            0x00000100
#define EMC_DEBUG_RCS               0x00000200
#define EMC_DEBUG_INTERP_LIST       0x00000800
#define EMC_DEBUG_ALL               0x7FFFFFFF  /* it's an int for %i to work  */

/* default name of EMC ini file */
#define DEFAULT_EMC_INIFILE EMC2_DEFAULT_INIFILE

/* default name of EMC NML file */
#define DEFAULT_EMC_NMLFILE EMC2_DEFAULT_NMLFILE

/* cycle time for emctask, in seconds */
#define DEFAULT_EMC_TASK_CYCLE_TIME 0.100

/* cycle time for emctio, in seconds */
#define DEFAULT_EMC_IO_CYCLE_TIME 0.100

/* default interp len */
#define DEFAULT_EMC_TASK_INTERP_MAX_LEN 1000

/* default name of EMC_TOOL tool table file */
#define DEFAULT_TOOL_TABLE_FILE "tool.tbl"

/* default feed rate, in user units per second */
#define DEFAULT_TRAJ_DEFAULT_VELOCITY 1.0

/* default traverse rate, in user units per second */
#define DEFAULT_TRAJ_MAX_VELOCITY 10.0

/* default axis traverse rate, in user units per second */
#define DEFAULT_AXIS_MAX_VELOCITY 1.0

/* default axis acceleration, in user units per second per second */
#define DEFAULT_AXIS_MAX_ACCELERATION 1.0

// values for EMC_AXIS_SET_AXIS, axisType
enum EmcAxisType
{
   EMC_AXIS_LINEAR = 1,
   EMC_AXIS_ANGULAR = 2,
};

// defs for termination conditions
#define EMC_TRAJ_TERM_COND_STOP  1
#define EMC_TRAJ_TERM_COND_BLEND 2

#define RTSTEPPER_PERIOD 42667  /* 1 / (46875hz / 2) = 0.000042667 */

/* Set the units conversion factor. @see EMC_AXIS_SET_INPUT_SCALE  */
typedef double EmcLinearUnits;
typedef double EmcAngularUnits;

extern char EMC_INIFILE[LINELEN];
extern char EMC_NMLFILE[LINELEN];
extern char RS274NGC_STARTUP_CODE[LINELEN];
extern double EMC_TASK_CYCLE_TIME;
extern double EMC_IO_CYCLE_TIME;
extern int EMC_TASK_INTERP_MAX_LEN;
extern char TOOL_TABLE_FILE[LINELEN];
extern double TRAJ_DEFAULT_VELOCITY;
extern double TRAJ_MAX_VELOCITY;
extern double AXIS_MAX_VELOCITY[EMC_AXIS_MAX];
extern double AXIS_MAX_ACCELERATION[EMC_AXIS_MAX];
extern struct EmcPose TOOL_CHANGE_POSITION;
extern unsigned char HAVE_TOOL_CHANGE_POSITION;
extern struct EmcPose TOOL_HOLDER_CLEAR;
extern unsigned char HAVE_TOOL_HOLDER_CLEAR;

//extern int num_axes;
//extern double VELOCITY;
//extern double ACCELERATION;
//extern double MAX_LIMIT;
//extern double MIN_LIMIT;
//extern double MAX_OUTPUT;
//extern double MIN_OUTPUT;
//extern int TC_QUEUE_SIZE;
//extern double MAX_FERROR;
//extern double BACKLASH;

extern struct emc_session session;

extern emc_status_t *emcStatus;
extern emc_command_msg_t *emcCommand;
extern emc_command_msg_t *emcTaskCommand;
extern emcio_status_t emcioStatus;
extern emcmot_command_t emcmotCommand;
extern emcmot_status_t emcmotStatus;
extern emcmot_config_t emcmotConfig;
extern emcmot_debug_t emcmotDebug;
extern emcmot_joint_t joints[EMCMOT_MAX_JOINTS];
extern KINEMATICS_FORWARD_FLAGS fflags;
extern KINEMATICS_INVERSE_FLAGS iflags;

#ifdef __cplusplus
extern "C"
{
#endif

   int emcGetArgs(int argc, char *argv[]);
   void emcInitGlobals();
   void esleep(double seconds_to_sleep);
   int emcOperatorError(int id, const char *fmt, ...);
   int emcOperatorText(int id, const char *fmt, ...);
   int emcOperatorDisplay(int id, const char *fmt, ...);
   int emc_io_error_cb(int result);

   int emcAxisSetBacklash(int axis, double backlash);
   int emcAxisSetMinPositionLimit(int axis, double limit);
   int emcAxisSetMaxPositionLimit(int axis, double limit);
   int emcAxisSetMotorOffset(int axis, double offset);
   int emcAxisSetFerror(int axis, double ferror);
   int emcAxisSetMinFerror(int axis, double ferror);
   int emcAxisSetHomingParams(int axis, double home, double offset, double home_final_vel, double search_vel,
                              double latch_vel, int use_index, int ignore_limits, int is_shared, int home_sequence, int volatile_home);
   int emcAxisSetMaxVelocity(int axis, double vel);
   int emcAxisSetMaxAcceleration(int axis, double acc);
   int emcAxisSetInputScale(int axis, double scale);
   int emcAxisSetStepPin(int axis, int pin);
   int emcAxisSetDirectionPin(int axis, int pin);
   int emcAxisSetStepPolarity(int axis, int polarity);
   int emcAxisSetDirectionPolarity(int axis, int polarity);
   int emcAxisInit(int axis);
   int emcAxisHalt(int axis);
   int emcAxisAbort(int axis);
   int emcAxisEnable(int axis);
   int emcAxisDisable(int axis);
   int emcAxisHome(int axis);
   int emcAxisUnhome(int axis);
   int emcAxisJog(int axis, double vel);
   int emcAxisIncrJog(int axis, double incr, double vel);
   int emcAxisAbsJog(int axis, double pos, double vel);
   int emcAxisActivate(int axis);
   int emcAxisDeactivate(int axis);
   int emcAxisOverrideLimits(int axis);
   int emcAxisLoadComp(int axis, const char *file, int type);
   int emcAxisUpdate(emcaxis_status_t * stat, int numAxes);
   int emcAxisSetAxis(int axis, unsigned char axisType);
   int emcAxisSetUnits(int axis, double units);
   int emcAxisActivate(int axis);

   int emcTrajSetAxes(int axes, int axismask);
   int emcTrajSetUnits(double linearUnits, double angularUnits);
   int emcTrajSetCycleTime(double cycleTime);
   int emcTrajSetMode(enum EMC_TRAJ_MODE_ENUM mode);
   int emcTrajSetTeleopVector(EmcPose vel);
   int emcTrajSetVelocity(double vel, double ini_maxvel);
   int emcTrajSetAcceleration(double acc);
   int emcTrajSetMaxVelocity(double vel);
   int emcTrajSetMaxAcceleration(double acc);
   int emcTrajSetScale(double scale);
   int emcTrajSetFOEnable(unsigned char mode);  //feed override enable
   int emcTrajSetFHEnable(unsigned char mode);  //feed hold enable
   int emcTrajSetSpindleScale(double scale);
   int emcTrajSetSOEnable(unsigned char mode);  //spindle speed override enable
   int emcTrajSetAFEnable(unsigned char enable);        //adaptive feed enable
   int emcTrajSetMotionId(int id);
   double emcTrajGetLinearUnits();
   double emcTrajGetAngularUnits();

   int emcTrajInit();
   int emcTrajHalt();
   int emcTrajEnable();
   int emcTrajDisable();
   int emcTrajAbort();
   int emcTrajPause();
   int emcTrajStep();
   int emcTrajResume();
   int emcTrajDelay(double delay);
   int emcTrajLinearMove(EmcPose end, int type, double vel, double ini_maxvel, double acc);
//int emcTrajCircularMove(EmcPose end, PM_CARTESIAN center, PM_CARTESIAN normal,
//      int turn, int type, double vel, double ini_maxvel, double acc);
   int emcTrajSetTermCond(int cond, double tolerance);
//int emcTrajSetSpindleSync(double feed_per_revolution, bool wait_for_index);
   int emcTrajSetOffset(EmcPose tool_offset);
   int emcTrajSetOrigin(EmcPose origin);
   int emcTrajSetRotation(double rotation);
   int emcTrajSetHome(EmcPose home);
   int emcTrajClearProbeTrippedFlag();
   int emcTrajProbe(EmcPose pos, int type, double vel, double ini_maxvel, double acc, unsigned char probe_type);
   int emcAuxInputWait(int index, int input_type, int wait_type, int timeout);
   int emcTrajRigidTap(EmcPose pos, double vel, double ini_maxvel, double acc);
   int emcTrajUpdate(emctraj_status_t * stat);

   int emcMotionInit();
   int emcMotionHalt();
   int emcMotionAbort();
   int emcMotionSetDebug(int debug);
   int emcMotionSetAout(unsigned char index, double start, double end, unsigned char now);
   int emcMotionSetDout(unsigned char index, unsigned char start, unsigned char end, unsigned char now);
   int emcMotionUpdate(emcmot_status_t * stat);

   int emcTaskInit();
   int emcTaskHalt();
   int emcTaskAbort();
   int emcTaskSetState(int state);
   int emcTaskPlanInit();
   int emcTaskPlanExit();
   int emcTaskUpdate(emctask_status_t * stat);
   int emcTaskPlan(void);
   int emcTaskExecute(void);
   int emcTaskPlanCommand(char *cmd);
   int emcTaskPlanRead();
   int emcTaskPlanExecute(const char *command);
   int emcTaskPlanExecuteEx(const char *command, int line_number);
   int emcTaskPlanOpen(const char *file);
   int emcTaskPlanClose();

   int emcToolInit();
   int emcToolHalt();
   int emcToolAbort();
   int emcToolPrepare(int tool);
   int emcToolLoad();
   int emcToolUnload();
   int emcToolLoadToolTable(const char *file);
   int emcToolSetOffset(int pocket, int toolno, EmcPose offset, double diameter, double frontangle, double backangle, int orientation);
   int emcToolSetNumber(int number);
   int emcToolUpdate(emctool_status_t * stat);
   int emcToolSetToolTableFile(const char *filename);

   int emcAuxEstopOn();
   int emcAuxEstopOff();
   int emcAuxUpdate(emcaux_status_t * stat);

   int emcSpindleAbort();
   int emcSpindleSpeed(double speed, double factor, double xoffset);
   int emcSpindleOn(double speed, double factor, double xoffset);
   int emcSpindleOff();
   int emcSpindleIncrease();
   int emcSpindleDecrease();
   int emcSpindleConstant();
   int emcSpindleBrakeRelease();
   int emcSpindleBrakeEngage();
   int emcSpindleSetMode(int mode);     //determines if Spindle needs to reset on abort
   int emcSpindleUpdate(emcspindle_status_t * stat);

   int emcCoolantMistOn();
   int emcCoolantMistOff();
   int emcCoolantFloodOn();
   int emcCoolantFloodOff();
   int emcCoolantUpdate(emccoolant_status_t * stat);

   int emcLubeOn();
   int emcLubeOff();
   int emcLubeUpdate(emclube_status_t * stat);

   int emcIoInit();
   int emcIoHalt();
   int emcIoAbort();
   int emcIoSetCycleTime(double cycleTime);
   int emcIoSetDebug(int debug);
   int emcIoUpdate(emcio_status_t * stat);
   void emciocommandHandler(emcio_command_t * emcioCommand);

   void emcmotController(long period);
   void emcmotCommandHandler(emcmot_command_t * emcmotCommand);
   void emcmot_config_change(void);
   int emcmotCheckAllHomed(void);
   void emcmot_refresh_jog_limits(emcmot_joint_t * joint);
   int emcmotSetTrajCycleTime(double secs);
   int emcmotSetServoCycleTime(double secs);

   int emcInit();
   int emcHalt();
   int emcAbort();

//   int emcUpdate(emc_status_t * stat);

   enum EMC_RESULT emc_ui_init(const char *ini_file);
   enum EMC_RESULT emc_ui_exit(void);
   enum EMC_RESULT emc_ui_update_status(void);
   enum EMC_RESULT emc_ui_update_operator_error(char *buf, int buf_size);
   enum EMC_RESULT emc_ui_update_operator_text(char *buf, int buf_size);
   enum EMC_RESULT emc_ui_update_operator_display(char *buf, int buf_size);
/* Wait for last command to be received by control thread. */
   enum EMC_RESULT emc_ui_command_wait_received(void);
/* Wait for last command to finish executing. */
   enum EMC_RESULT emc_ui_command_wait_done(void);
   enum EMC_RESULT emc_ui_set_timeout(double timeout);
   double emc_ui_get_timeout(void);
   enum EMC_RESULT emc_ui_set_wait_type(enum EMC_UI_WAIT_TYPE type);
   enum EMC_UI_WAIT_TYPE emc_ui_get_wait_type(void);
   enum EMC_RESULT emc_ui_send_estop(void);
   enum EMC_RESULT emc_ui_send_estop_reset(void);
   enum EMC_RESULT emc_ui_send_machine_on(void);
   enum EMC_RESULT emc_ui_send_machine_off(void);
   enum EMC_RESULT emc_ui_send_manual(void);
   enum EMC_RESULT emc_ui_send_auto(void);
   enum EMC_RESULT emc_ui_send_mdi(void);
   enum EMC_RESULT emc_ui_send_mist_on(void);
   enum EMC_RESULT emc_ui_send_mist_off(void);
   enum EMC_RESULT emc_ui_send_flood_on(void);
   enum EMC_RESULT emc_ui_send_flood_off(void);
   enum EMC_RESULT emc_ui_send_lube_on(void);
   enum EMC_RESULT emc_ui_send_lube_off(void);
   enum EMC_RESULT emc_ui_send_spindle_forward(void);
   enum EMC_RESULT emc_ui_send_spindle_reverse(void);
   enum EMC_RESULT emc_ui_send_spindle_off(void);
   enum EMC_RESULT emc_ui_send_spindle_increase(void);
   enum EMC_RESULT emc_ui_send_spindle_decrease(void);
   enum EMC_RESULT emc_ui_send_spindle_constant(void);
   enum EMC_RESULT emc_ui_send_brake_engage(void);
   enum EMC_RESULT emc_ui_send_brake_release(void);
   enum EMC_RESULT emc_ui_send_load_tool_table(const char *file);
   enum EMC_RESULT emc_ui_send_tool_set_offset(int toolno, double zoffset, double diameter);
   enum EMC_RESULT emc_ui_send_override_limits(int axis);
   enum EMC_RESULT emc_ui_send_mdi_cmd(const char *mdi);
   enum EMC_RESULT emc_ui_send_home(int axis);
   enum EMC_RESULT emc_ui_send_unhome(int axis);
   enum EMC_RESULT emc_ui_send_jog_stop(int axis);
   enum EMC_RESULT emc_ui_send_jog_cont(int axis, double speed);
   enum EMC_RESULT emc_ui_send_jog_incr(int axis, double speed, double incr);
   enum EMC_RESULT emc_ui_send_feed_override(double override);
   enum EMC_RESULT emc_ui_send_task_plan_init(void);
   enum EMC_RESULT emc_ui_send_program_open(const char *program);
   enum EMC_RESULT emc_ui_send_program_run(int line);
   enum EMC_RESULT emc_ui_send_program_pause(void);
   enum EMC_RESULT emc_ui_send_program_resume(void);
   enum EMC_RESULT emc_ui_send_program_step(void);
   enum EMC_RESULT emc_ui_send_abort(void);
   enum EMC_RESULT emc_ui_send_axis_set_backlash(int axis, double backlash);
   enum EMC_RESULT emc_ui_send_teleop_enable(int enable);
   enum EMC_RESULT emc_ui_get_args(int argc, char *argv[]);

#ifdef __cplusplus
}                               /* matches extern "C" at top */
#endif

#endif                          // _COMMAND_H
