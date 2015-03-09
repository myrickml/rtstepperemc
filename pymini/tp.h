#ifndef TP_H
#define TP_H

/*
  tp.h

  Trajectory planner based on TC elements

  Modification history:

  7-Dec-2001  FMP took hard-coded 1e-6 values out of tp.c and moved them
  here as EPSILON defines.
  17-Aug-2001  FMP added aout,dout motion IO
  13-Mar-2000 WPS added unused attribute to tp_h to avoid
  'defined but not used' compiler warning.
  8-Jun-1999  FMP added tpSetVlimit(), vLimit
  8-Mar-1999  FMP added tcSpace arg to tpCreate()
  18-Dec-1997  FMP took out C++ interface
  18-Dec-1997  FMP changed to PmPose
  18-Jul-1997  FMP added active depth
  16-Jul-1997  FMP added ids
  14-Jul-1997  FMP added C posemath changes (PM_POSE -> PmPose)
  24-Jun-1997  FMP added tpClear()
  13-Jun-1997  FMP added tpSetVscale(), tpIsPaused(), abort stuff
  16-Apr-1997  FMP created from C and C++ headers
*/

#include "posemath.h"
#include "tc.h"

#define TP_DEFAULT_QUEUE_SIZE 32

/* closeness to zero, for determining if a move is pure rotation */
#define TP_PURE_ROTATION_EPSILON 1e-6

/* closeness to zero, for determining if vel and accel are effectively zero */
#define TP_VEL_EPSILON 1e-6
#define TP_ACCEL_EPSILON 1e-6
  
typedef struct
{
  TC_QUEUE_STRUCT queue;
  int queueSize;
  double cycleTime;
  double vMax;                  /* vel for subsequent moves */
  double vScale, vRestore;
  double aMax;
  double vLimit;                /* absolute upper limit on all vels */
  double wMax;			/* rotational velocity max  */
  double wDotMax;		/* rotational accelleration max */
  int nextId;
  int execId;
  int termCond;
  EmcPose currentPos;
  EmcPose goalPos;
  int done;
  int depth;                    /* number of total queued motions */
  int activeDepth;              /* number of motions blending */
  int aborting;
  int pausing;
  unsigned char douts;		/* mask for douts to set */
  unsigned char doutstart;	/* mask for dout start vals */
  unsigned char doutend;	/* mask for dout end vals */
} TP_STRUCT;

#ifdef __cplusplus
extern "C"
{
#endif

int tpCreate(TP_STRUCT *tp, int _queueSize, TC_STRUCT *tcSpace);
int tpDelete(TP_STRUCT *tp);
int tpClear(TP_STRUCT *tp);
int tpInit(TP_STRUCT *tp);

int tpSetCycleTime(TP_STRUCT *tp, double secs);
int tpSetVmax(TP_STRUCT *tp, double vmax);
int tpSetWmax(TP_STRUCT *tp, double vmax);
int tpSetVlimit(TP_STRUCT *tp, double limit);
int tpSetVscale(TP_STRUCT *tp, double scale); /* 0.0 .. large */
int tpSetAmax(TP_STRUCT *tp, double amax);
int tpSetWDotmax(TP_STRUCT *tp, double amax);
int tpSetId(TP_STRUCT *tp, int id);
int tpGetNextId(TP_STRUCT *tp);
int tpGetExecId(TP_STRUCT *tp);
int tpSetTermCond(TP_STRUCT *tp, int cond);
int tpGetTermCond(TP_STRUCT *tp);
int tpSetPos(TP_STRUCT *tp, EmcPose pos);
int tpAddLine(TP_STRUCT *tp, EmcPose end);
int tpAddCircle(TP_STRUCT *tp, EmcPose end,
                       PmCartesian center, PmCartesian normal, int turn);
int tpRunCycle(TP_STRUCT *tp);
int tpPause(TP_STRUCT *tp);
int tpResume(TP_STRUCT *tp);
int tpAbort(TP_STRUCT *tp);
EmcPose tpGetPos(TP_STRUCT *tp);
int tpIsDone(TP_STRUCT *tp);
int tpIsPaused(TP_STRUCT *tp);
int tpQueueDepth(TP_STRUCT *tp);
int tpActiveDepth(TP_STRUCT *tp);
void tpPrint(TP_STRUCT *tp);
int tpSetAout(TP_STRUCT *tp, unsigned char index, double start, double end);
int tpSetDout(TP_STRUCT *tp, unsigned char index, unsigned char start, unsigned char end);

#ifdef __cplusplus
}
#endif

#endif /* TP_H */
