/************************************************************************************\

  rtstepper.h - rt-stepper dongle support for EMC2

  (c) 2011-2015 Copyright Eckler Software

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

\************************************************************************************/

#ifndef _RTSTEPPER_H
#define _RTSTEPPER_H

#include <stdint.h>
#include <pthread.h>
#include <libusb.h>
#include "list.h"
#include "emc.h"

struct rtstepper_io_req
{
   int id;
   EmcPose position;            // commanded position
   unsigned char *buf;          /* step/direction buffer */
   int buf_size;                /* buffer size in bytes */
   int total;                   /* current buffer count, number of bytes used (total < buf_size) */
   struct emc_session *session;
   struct libusb_transfer *req; 
   struct list_head list;
};

struct rtstepper_file_descriptor
{
   libusb_device_handle *hd;
   libusb_device **list_all;
   libusb_context *ctx;
   libusb_device *dev;
   int event_done;
   int event_abort_done;
   pthread_t event_tid;    /* thread handle */
   int dongle_done;
   int dongle_abort_done;
   pthread_t dongle_tid;    /* thread handle */
};

#define RTSTEPPER_STEP_STATE_ABORT_BIT 0x01     /* abort step buffer, 1=True, 0=False (R/W) */
#define RTSTEPPER_STEP_STATE_EMPTY_BIT 0x02     /* step buffer empty, 1=True, 0=False */
#define RTSTEPPER_STEP_STATE_STALL_BIT 0x04     /* 1=True, 0=False */
#define RTSTEPPER_STEP_STATE_INPUT0_BIT 0x08    /* active high INPUT0, 1=True, 0=False */
#define RTSTEPPER_STEP_STATE_INPUT1_BIT 0x10    /* active high INPUT1, 1=True, 0=False */
#define RTSTEPPER_STEP_STATE_INPUT2_BIT 0x20    /* active high INPUT2, 1=True, 0=False */

#define RTSTEPPER_MECH_THREAD 1
#define RTSTEPPER_DONGLE_THREAD 0

/* Forward declarations. */
struct emc_session;

#ifdef __cplusplus
extern "C"
{
#endif

/* Function prototypes */

   enum EMC_RESULT rtstepper_open(struct emc_session *ps);
   enum EMC_RESULT rtstepper_close(struct emc_session *ps);
   enum EMC_RESULT rtstepper_query_state(struct emc_session *ps);
   enum EMC_RESULT rtstepper_clear_abort(struct emc_session *ps);
   enum EMC_RESULT rtstepper_set_abort(struct emc_session *ps);
   enum EMC_RESULT rtstepper_encode(struct emc_session *ps, struct rtstepper_io_req *io, double index[]);
   enum EMC_RESULT rtstepper_start_xfr(struct emc_session *ps, struct rtstepper_io_req *io, EmcPose pos);
   enum EMC_RESULT rtstepper_wait_xfr(struct emc_session *ps);
   int rtstepper_is_connected(struct emc_session *ps);
   enum EMC_RESULT rtstepper_is_input0_triggered(struct emc_session *ps);
   enum EMC_RESULT rtstepper_is_input1_triggered(struct emc_session *ps);
   enum EMC_RESULT rtstepper_is_input2_triggered(struct emc_session *ps);
   enum EMC_RESULT rtstepper_input0_state(struct emc_session *ps);
   enum EMC_RESULT rtstepper_input1_state(struct emc_session *ps);
   enum EMC_RESULT rtstepper_input2_state(struct emc_session *ps);
   enum EMC_RESULT rtstepper_home(struct emc_session *ps);
   enum EMC_RESULT rtstepper_estop(struct emc_session *ps, int thread);
   struct rtstepper_io_req *rtstepper_alloc_io_req(struct emc_session *ps, int id);
#ifdef __cplusplus
}
#endif

#endif                          /* _RTSTEPPER_H */
