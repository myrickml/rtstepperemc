/*****************************************************************************\

  rtstepper.c - rt-stepper dongle support for rtstepperemc

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include "emc.h"
#include "bug.h"

#define DONGLE_VENDOR_ID 0x04d8
#define DONGLE_PRODUCT_ID 0xff45
#define DONGLE_INTERFACE 0
#define DONGLE_OUT_EP (LIBUSB_ENDPOINT_OUT | 1)

#define LIBUSB_TIMEOUT 30000    /* milliseconds */
#define LIBUSB_CONTROL_REQ_TIMEOUT 5000

/* EP0 Vendor Setup commands (bRequest). */
enum STEP_CMD
{
   STEP_SET,                    /* set step elements, clear state bits */
   STEP_QUERY,                  /* query current step and state info */
   STEP_ABORT_SET,              /* set un-synchronized stop */
   STEP_ABORT_CLEAR,            /* clear un-synchronized stop */
};

//#define STEP_BUF_CHUNK 4096
#define STEP_BUF_CHUNK 16384

static pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;  
static pthread_cond_t _write_done_cond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t _event_done_cond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t _dongle_done_cond = PTHREAD_COND_INITIALIZER;

struct __attribute__ ((packed)) step_state
{
   union
   {
      uint16_t _word;
   };
};

struct __attribute__ ((packed)) step_elements
{
   char reserved[8];
};

struct __attribute__ ((packed)) step_query
{
   struct step_state state_bits;
   unsigned char reserved;
   unsigned char trip_cnt;      /* stall trip count */
   uint32_t step;               /* running step count */
};

extern const char *USER_HOME_DIR;

static enum EMC_RESULT start_xfr(struct rtstepper_io_req *io);

/* 
 * DB25 pin to bitfield map.
 *                pin #       na, 1,  2,   3,   4,   5,    6,    7,   8,    9  
 */
static const int pin_map[] = { 0, 0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80 };

static int is_rt(libusb_device *dev, const char *sn)
{
   struct libusb_device_descriptor desc;
   libusb_device_handle *hd=NULL;
   int r, stat=0;
   char sz[128];

   if (libusb_get_device_descriptor(dev, &desc) != 0)
      goto bugout;

   if (!(desc.idVendor == DONGLE_VENDOR_ID && desc.idProduct == DONGLE_PRODUCT_ID))
      goto bugout;

   if ((r = libusb_open(dev, &hd)) != 0)
   {
      BUG("invalid usb_open: %s\n", libusb_error_name(r));
      goto bugout;
   }

   if ((r =libusb_get_string_descriptor_ascii(hd, desc.iSerialNumber, (unsigned char *)sz, sizeof(sz))) < 0)
      goto bugout;
   if (!sz[0])
      strcpy(sz, "0");

   MSG("Found rtstepper dongle serial_number=%s\n", sz);

   if (sn[0])
      if (strcmp(sz, sn) != 0)
         goto bugout;

   stat = 1;    /* found usb device */

 bugout:
   if (hd != NULL)
      libusb_close(hd);

   return stat;
}       /* is_rt() */

/* Libusb requires an event handling thread for asynchronous io. This loop calls xfr_cb(). 
 * Note, libusb_control_transfer() will also call xfr_cb(), this caused a mutex hang issue. 
 */
void event_thread(struct rtstepper_file_descriptor *pfd)
{
   struct timeval tv;

   tv.tv_sec = 0;
   tv.tv_usec = 0.1 * 1E6;   /* 100ms */

   while (!pfd->event_done)
   {
      /* If libusb has nothing to do, it will block until timeout expires. */
      libusb_handle_events_timeout_completed(pfd->ctx, &tv, NULL);
   }  /* while (!event_done) */

   DBG("event_thread() closed...\n");
   pfd->event_abort_done = 1;
   pthread_cond_signal(&_event_done_cond);
}  /* event_thread() */

void dongle_thread(struct rtstepper_file_descriptor *pfd)
{
   struct emc_session *ps;

   ps = container_of(pfd, struct emc_session, fd_table);

   while (!pfd->dongle_done)
   {
      esleep(0.05);   /* 50ms */

      rtstepper_query_state(ps);

      if (rtstepper_is_input0_triggered(ps) == RTSTEPPER_R_INPUT_TRUE)
      {
         rtstepper_estop(ps, RTSTEPPER_DONGLE_THREAD);
         emc_post_estop_cb(ps);
         MSG("INPUT0 estop...\n");
      }
      if (rtstepper_is_input1_triggered(ps) == RTSTEPPER_R_INPUT_TRUE)
      {
         rtstepper_estop(ps, RTSTEPPER_DONGLE_THREAD);
         emc_post_estop_cb(ps);
         MSG("INPUT1 estop...\n");
      }
      if (rtstepper_is_input2_triggered(ps) == RTSTEPPER_R_INPUT_TRUE)
      {
         rtstepper_estop(ps, RTSTEPPER_DONGLE_THREAD);
         emc_post_estop_cb(ps);
         MSG("INPUT2 estop...\n");
      }

      /* Check for estop and not aborted yet. */
      //if (ps->state_bits & EMC_STATE_ESTOP_BIT && !(ps->state_bits & RTSTEPPER_STEP_STATE_ABORT_BIT))
      //   rtstepper_set_abort(ps);  /* abort io in the dongle */     

   }  /* while (!dongle_done) */

   DBG("dongle_thread() closed...\n");
   pfd->dongle_abort_done = 1;
   pthread_cond_signal(&_dongle_done_cond);
}  /* dongle_thread() */

static int claim_interface(struct rtstepper_file_descriptor *pfd)
{
   int stat=1, r;

   pthread_mutex_lock(&_mutex);

   if (pfd->hd != NULL)
   {
      BUG("invalid state, interface is already claimed\n");
      goto bugout;  /* interface is already claimed */
   }

   if ((r = libusb_open(pfd->dev, &pfd->hd)) != 0)
   {
      BUG("invalid libusb_open: %s\n", libusb_error_name(r));
      goto bugout;
   }
      
   if ((r = libusb_claim_interface(pfd->hd, DONGLE_INTERFACE)) != 0)
   {
      libusb_close(pfd->hd);
      pfd->hd = NULL;
      BUG("invalid claim_interface: %s\n", libusb_error_name(r));
      goto bugout;
   }

   DBG("claimed interface %d\n", DONGLE_INTERFACE);

   stat=0;

bugout:
   pthread_mutex_unlock(&_mutex);
   return stat;   
} /* claim_interface() */

static int release_interface(struct rtstepper_file_descriptor *pfd)
{
   if (pfd->hd == NULL)
      return 0;

   pthread_mutex_lock(&_mutex);

   libusb_release_interface(pfd->hd, DONGLE_INTERFACE);
   libusb_close(pfd->hd);
   libusb_free_device_list(pfd->list_all, 1);
   libusb_exit(pfd->ctx);
   pfd->hd = NULL;

   DBG("released interface %d\n", DONGLE_INTERFACE);

   pthread_mutex_unlock(&_mutex);
   return 0;
} /* release_interface() */

static enum EMC_RESULT open_device(struct rtstepper_file_descriptor *pfd, const char *sn)
{
   enum EMC_RESULT stat = RTSTEPPER_R_DEVICE_UNAVAILABLE;
   int i, n;
   
   libusb_init(&pfd->ctx);
   libusb_set_debug(pfd->ctx, 3);
   n = libusb_get_device_list(pfd->ctx, &pfd->list_all);

   /* Look for rtstepper device(s). */
   for (i=0; i < n; i++)
   {
      if (is_rt(pfd->list_all[i], sn))
      {
         pfd->dev = pfd->list_all[i];
         if (claim_interface(pfd))
            goto bugout;

         /* Create event_thread after libusb_open. */
         pfd->event_done = pfd->event_abort_done = 0;
         pthread_create(&pfd->event_tid, NULL, (void *(*)(void *))event_thread, (void *)pfd);

         /* Create dongle_thread for reading input bits. */
         pfd->dongle_done = pfd->dongle_abort_done = 0;
         pthread_create(&pfd->dongle_tid, NULL, (void *(*)(void *))dongle_thread, (void *)pfd);

         stat = EMC_R_OK;
         break;
      }
   }

bugout:
   return stat;
}       /* open_device() */

static enum EMC_RESULT close_device(struct rtstepper_file_descriptor *pfd)
{
   if (pfd->hd != NULL)
   {
      /* Wait for dongle_thread to shutdown before calling libusb_close. */
      pfd->dongle_done = 1;
      pthread_mutex_lock(&_mutex);
      while (!pfd->dongle_abort_done)
         pthread_cond_wait(&_dongle_done_cond, &_mutex);
      pthread_mutex_unlock(&_mutex);

      /* Wait for event_thread to shutdown before calling libusb_close. */
      pfd->event_done = 1;
      pthread_mutex_lock(&_mutex);
      while (!pfd->event_abort_done)
         pthread_cond_wait(&_event_done_cond, &_mutex);
      pthread_mutex_unlock(&_mutex);

      release_interface(pfd);
   }
   return EMC_R_OK;
}       /* close_device() */

static void cancel_xfr(struct emc_session *ps)
{
   struct rtstepper_io_req *io;
   struct list_head *p, *tmp;

   pthread_mutex_lock(&_mutex);

   /* Walk the queue deleting all io requests. */
   list_for_each_safe(p, tmp, &ps->head.list)
   {
      /* Need to re-home after canceling any io request. */
      ps->state_bits &= ~EMC_STATE_HOMED_BIT;

      io = list_entry(p, struct rtstepper_io_req, list);

      /* Cancel current io transfer, xfr_cb() will complete the cancel.  */
      if (io->req != NULL)
      {
         libusb_cancel_transfer(io->req);
         continue;
      }
      
      /* Remove all pending io requests from the queue. */
      free(io->buf);
      list_del(&io->list);
      free(io);
   }

   pthread_mutex_unlock(&_mutex);
} /* cancel_xfr() */

/* Libusb asynchronous transfer complete callback function. */
static void xfr_cb(struct libusb_transfer *transfer)
{
   struct emc_session *ps;
   struct rtstepper_io_req *io;
   int empty;

   DBG("xfr_cb() io=%p, %d bytes written\n", transfer->user_data, transfer->actual_length);

   io = transfer->user_data;
   ps = io->session;

   if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
   {
      switch (transfer->status)
      {
      case LIBUSB_TRANSFER_ERROR:
         BUG("usb transfer failed\n");
         emc_post_estop_cb(ps);
         break;
      case LIBUSB_TRANSFER_TIMED_OUT:
         BUG("usb transfer timed out\n");
         emc_post_estop_cb(ps);
         break;
      case LIBUSB_TRANSFER_CANCELLED:
         MSG("Usb transfer was cancelled\n");
         break;
      case LIBUSB_TRANSFER_STALL:
         BUG("usb transfer stalled\n");
         emc_post_estop_cb(ps);
         break;
      case LIBUSB_TRANSFER_NO_DEVICE:
         BUG("usb device was disconnected\n");
         emc_post_estop_cb(ps);
         break;
      case LIBUSB_TRANSFER_OVERFLOW:
         BUG("usb transfer overflow\n");
         emc_post_estop_cb(ps);
         break;
      default:
         BUG("invalid usb transfer: %d\n", transfer->status);
         break;
      }
   }
   else
   {
      /* Transfer is ok, save current position. */
      ps->position = io->position;
      emc_post_position_cb(io->id, io->position); 
   }

   pthread_mutex_lock(&_mutex);

   libusb_free_transfer(io->req);
   free(io->buf);
   list_del(&io->list);
   free(io);
   empty = list_empty(&ps->head.list);

   pthread_mutex_unlock(&_mutex);

   if (!empty)
   {
      /* Kickoff next usb io request from the head of the queue (FIFO). */
      io = list_entry(ps->head.list.next, struct rtstepper_io_req, list);
      start_xfr(io);
   }
   else
   { 
      /* All usb io is complete. */
      DBG("broadcast write_done_cond...\n");
      pthread_cond_broadcast(&_write_done_cond);
   }

   return;
}  /* xfr_cb() */

static enum EMC_RESULT start_xfr(struct rtstepper_io_req *io)
{
   enum EMC_RESULT stat = RTSTEPPER_R_IO_ERROR;
   struct emc_session *ps = io->session;
   int r, tmo;

   if (ps->state_bits & EMC_STATE_ESTOP_BIT)
      return EMC_R_OK;  /* ESTOP active, ignore io requests. */

   DBG("start_xfr() io=%p, cnt=%d\n", io, io->total);

   tmo = (int)((double) io->total * 0.021333);      /* timeout in ms = steps * period * 1000 */
   tmo += 5000; /* plus 5 seconds */

   /* Allocate an asynchronous transfer. */
   io->req = libusb_alloc_transfer(0);
   libusb_fill_bulk_transfer(io->req, ps->fd_table.hd, DONGLE_OUT_EP, io->buf, io->total, xfr_cb, io, tmo);

   /* Kickoff the asynchronous io. */
   if ((r = libusb_submit_transfer(io->req)) != 0)
   {
      BUG("invalid start_xfr: %s\n", libusb_error_name(r));
      emc_post_estop_cb(ps);
      goto bugout;
   }

   stat = EMC_R_OK;
bugout:
   return stat;
}  /* start_xfr() */

enum EMC_RESULT rtstepper_start_xfr(struct emc_session *ps, struct rtstepper_io_req *io, EmcPose pos)
{
   enum EMC_RESULT stat = RTSTEPPER_R_IO_ERROR;
   int i, j, mid, empty;

   if (io == NULL)
      goto bugout;

   DBG("rtstepper_start_xfr: x_index=%d y_index=%d z_index=%d a_index=%d c_index=%d io=%p cnt=%d line=%d\n", ps->axis[0].master_index,
       ps->axis[1].master_index, ps->axis[2].master_index, ps->axis[3].master_index, ps->axis[5].master_index, io, io->total, io->id);

   /* Save commanded position for this io request. */
   io->position = pos;

   /* Finish last pulse for this step buffer. */
   for (i = 0; i < ps->axes; i++)
   {
      if (ps->axis[i].step_pin == 0)
         continue;   /* skip */

      if (ps->axis[i].clk_tail)
      {
         /* Stretch pulse to 50% duty cycle. */
         mid = (io->total - ps->axis[i].clk_tail) / 2;
         for (j=0; j < mid; j++)
         {
            if (ps->axis[i].step_active_high)
               io->buf[ps->axis[i].clk_tail + j] |= pin_map[ps->axis[i].step_pin]; /* set bit */
            else
               io->buf[ps->axis[i].clk_tail + j] &= ~pin_map[ps->axis[i].step_pin]; /* clear bit */
         }
      }
   }

   pthread_mutex_lock(&_mutex);

   empty = list_empty(&ps->head.list);

   /* Add io request to tail of the queue (FIFO). */
   list_add_tail(&io->list, &ps->head.list);

   pthread_mutex_unlock(&_mutex);

   if (empty)
   {
      /* Kick off first usb io request here. */
      start_xfr(io);
   }

   stat = EMC_R_OK;

 bugout:
   return stat;
}       /* rtstepper_start_xfr() */

enum EMC_RESULT rtstepper_wait_xfr(struct emc_session *ps)
{
   struct timeval tv;
   struct timespec ts;
   int rc;

   /* Wait for all IO to finish. */
   DBG("rstepper_wait_xfr()\n");

   /*
    * Use ptread_cond_timedwait() here because spurious wakeups are not guaranteed. 
    * This means predicates may never get checked after pthread_cond_wait() thus
    * causing a potential hang.
    */
   do
   {
      gettimeofday(&tv, NULL);
      ts.tv_sec = tv.tv_sec + 2;    /* 2 sec timeout */
      ts.tv_nsec = 0;
      rc=0;
      pthread_mutex_lock(&_mutex);
      while (list_empty(&ps->head.list)==0 && (ps->state_bits & EMC_STATE_ESTOP_BIT)==0 && rc==0)
         rc = pthread_cond_timedwait(&_write_done_cond, &_mutex, &ts);
      pthread_mutex_unlock(&_mutex);
   } while (rc == ETIMEDOUT);

   DBG("rstepper_wait_xfr() done...\n");
   
   return EMC_R_OK;
}

struct rtstepper_io_req *rtstepper_alloc_io_req(struct emc_session *ps, int id)
{
   struct rtstepper_io_req *io;
   
   if (ps->fd_table.hd == NULL)
      return NULL;  /* no usb dongle available */
   if (ps->state_bits & EMC_STATE_ESTOP_BIT)
      return NULL;  /* ESTOP active, ignore io requests. */

   if ((io = malloc(sizeof(struct rtstepper_io_req))) != NULL)
   {
      io->id = id;
      io->buf = NULL;
      io->buf_size = 0;
      io->total = 0;
      io->session = ps;
      io->req = NULL;
   }
   return io;
}  /* rtstepper_io_req() */

/*
 * Given a command position in counts for each axis, encode each value into a single step/direction byte. 
 * Store the byte in buffer that is big enough to hold a complete stepper motor move.
 */
enum EMC_RESULT rtstepper_encode(struct emc_session *ps, struct rtstepper_io_req *io, double index[])
{
   int i, j, step, mid, new_size, stat = RTSTEPPER_R_MALLOC_ERROR;
   static unsigned int cnt = 0;
   unsigned char *tmp;

   if (io == NULL)
      goto bugout;

   if (io->buf == NULL || (io->buf_size - io->total) < 2)
   {
      new_size = (io->buf_size < STEP_BUF_CHUNK) ? STEP_BUF_CHUNK : io->buf_size * 2;
      if ((tmp = (unsigned char *)realloc(io->buf, new_size)) == NULL)
      {
         free(io->buf);
         io->buf = NULL;
         BUG("unable to malloc step buffer size=%d\n", new_size);
         goto bugout;   /* bail */
      }
      io->buf = tmp;
      io->buf_size = new_size;
   }

   for (i = 0; i < ps->axes; i++)
   {
      /* Check DB25 pin assignments for this axis, if no pins are assigned skip this axis. Useful for XYZABC axes where AB are unused. */
      if (ps->axis[i].step_pin == 0 || ps->axis[i].direction_pin == 0)
         continue;   /* skip */

      /* Set step bit to default state, high if low_true logic or low if high_true logic. */
      if (ps->axis[i].step_active_high)
      {
         io->buf[io->total] &= ~pin_map[ps->axis[i].step_pin];
         io->buf[io->total + 1] &= ~pin_map[ps->axis[i].step_pin];
      }
      else
      {
         io->buf[io->total] |= pin_map[ps->axis[i].step_pin];
         io->buf[io->total + 1] |= pin_map[ps->axis[i].step_pin];
      }

      /* Set direction bit to default state, high if low_true logic or low if high_true logic. */
      if (ps->axis[i].direction_active_high)
      {
         io->buf[io->total] &= ~pin_map[ps->axis[i].direction_pin];
         io->buf[io->total + 1] &= ~pin_map[ps->axis[i].direction_pin];
      }
      else
      {
         io->buf[io->total] |= pin_map[ps->axis[i].direction_pin];
         io->buf[io->total + 1] |= pin_map[ps->axis[i].direction_pin];
      }

      /* Calculate the step pulse for this clock cycle */
      step = round(index[i] * ps->axis[i].steps_per_unit) - ps->axis[i].master_index;

      if (step < -1 || step > 1)
      {
         if (cnt++ < 30)
         {
            BUG("invalid step value: id=%d axis=%d cmd_pos=%0.8f master_index=%d input_scale=%0.2f step=%d\n",
                io->id, i, index[i], ps->axis[i].master_index, ps->axis[i].steps_per_unit, step);
         }
         step = 0;
      }

      if (step)
      {
         /* Got a valid step pulse this cycle. */
         if (ps->axis[i].clk_tail)
         {
            /* Using the second pulse, stretch pulse to 50% duty cycle. */
            mid = (io->total - ps->axis[i].clk_tail) / 2;
            for (j=0; j < mid; j++)
            {
               if (ps->axis[i].step_active_high)
                  io->buf[ps->axis[i].clk_tail + j] |= pin_map[ps->axis[i].step_pin]; /* set bit */
               else
                  io->buf[ps->axis[i].clk_tail + j] &= ~pin_map[ps->axis[i].step_pin]; /* clear bit */
            }
         }

         /* save old step location */
         ps->axis[i].clk_tail = io->total;

         /* save step direction */
         ps->axis[i].direction = step;
      }

      /* Set direction bit. */
      if (ps->axis[i].direction < 0)
      {
         if (ps->axis[i].direction_active_high)
         {
            io->buf[io->total] |= pin_map[ps->axis[i].direction_pin];    /* set bit */
            io->buf[io->total + 1] |= pin_map[ps->axis[i].direction_pin];  /* set bit */
         }
         else
         {
            io->buf[io->total] &= ~pin_map[ps->axis[i].direction_pin];       /* clear bit */
            io->buf[io->total + 1] &= ~pin_map[ps->axis[i].direction_pin];   /* clear bit */
         }
      }

      ps->axis[i].master_index += step;

//      DBG("axis=%d index=%0.6f master_index=%d\n", i, index[i] * ps->axis[i].steps_per_unit, ps->axis[i].master_index);
   }    /* for (i=0; i < num_axis; i++) */

   io->total += 2;

   stat = EMC_R_OK;

 bugout:
   return stat;
}       /* rtstepper_encode() */

int rtstepper_is_connected(struct emc_session *ps)
{
   return ps->fd_table.hd != NULL;
}       /* rtstepper_is_connected() */

enum EMC_RESULT rtstepper_home(struct emc_session *ps)
{
   int i;

   DBG("rtstepper_home()\n");
   for (i = 0; i < ps->axes; i++)
      ps->axis[i].master_index = 0.0;
   return EMC_R_OK;
}       /* rtstepper_home() */

enum EMC_RESULT rtstepper_estop(struct emc_session *ps, int thread)
{
   struct rtstepper_file_descriptor *pfd = &ps->fd_table;
   enum EMC_RESULT stat = RTSTEPPER_R_REQ_ERROR;

   ps->state_bits |= EMC_STATE_ESTOP_BIT;
   DBG("rtstepper_estop()\n");

   if (pfd->hd == NULL)
      goto bugout;

   if (thread != RTSTEPPER_DONGLE_THREAD)
   {
      /* Wait for dongle_thread to shutdown before sending abort. */
      pfd->dongle_done = 1;
      pthread_mutex_lock(&_mutex);
      while (!pfd->dongle_abort_done)
         pthread_cond_wait(&_dongle_done_cond, &_mutex);
      pthread_mutex_unlock(&_mutex);
   }

   rtstepper_set_abort(ps);
   stat = EMC_R_OK;

bugout:
   return stat;
}       /* rtstepper_estop() */

/* Set abort state in dongle. */
enum EMC_RESULT rtstepper_set_abort(struct emc_session *ps)
{
   enum EMC_RESULT stat;
   int len;

   DBG("rtstepper_set_abort() state=%x\n", ps->state_bits);

   if (ps->fd_table.hd == NULL)
   {
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   len = libusb_control_transfer(ps->fd_table.hd, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,      /* bmRequestType */
                         STEP_ABORT_SET,        /* bRequest */
                         0x0,   /* wValue */
                         DONGLE_INTERFACE, /* wIndex */
                         NULL, 0, LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len < 0)
   {
      BUG("set_abort failed ret=%d: %s\n", len, libusb_error_name(len));
      stat = RTSTEPPER_R_IO_ERROR;
      goto bugout;
   }

   cancel_xfr(ps);

   stat = EMC_R_OK;

 bugout:
   return stat;
}       /* rtstepper_set_abort() */

/* Clear abort state in dongle. */
enum EMC_RESULT rtstepper_clear_abort(struct emc_session *ps)
{
   enum EMC_RESULT stat;
   int len;

   if (ps->fd_table.hd == NULL)
   {
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   len = libusb_control_transfer(ps->fd_table.hd, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,      /* bmRequestType */
                         STEP_ABORT_CLEAR,      /* bRequest */
                         0x0,   /* wValue */
                         DONGLE_INTERFACE, /* wIndex */
                         NULL, 0, LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len < 0)
   {
      BUG("clear_abort failed ret=%d: %s\n", len, libusb_error_name(len));
      stat = RTSTEPPER_R_IO_ERROR;
      goto bugout;
   }

   /* Reset step_state_inputx_bit for rtstepper_is_inputx_triggered(). */
   ps->old_state_bits &= ~(RTSTEPPER_STEP_STATE_INPUT0_BIT | RTSTEPPER_STEP_STATE_INPUT1_BIT | RTSTEPPER_STEP_STATE_INPUT2_BIT);

   stat = EMC_R_OK;

 bugout:
   return stat;
}       /* rtstepper_clear_abort() */

enum EMC_RESULT rtstepper_is_input0_triggered(struct emc_session *ps)
{
   enum EMC_RESULT stat = RTSTEPPER_R_INPUT_FALSE;
   int new_bit, old_bit;

   if (ps->fd_table.hd == NULL)
      goto bugout;

   old_bit = ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT0_BIT;
   if (ps->input0_abort_enabled)
   {
      new_bit = ps->state_bits & RTSTEPPER_STEP_STATE_INPUT0_BIT;
      if ((new_bit == RTSTEPPER_STEP_STATE_INPUT0_BIT) && (old_bit == 0))
      {
         /* Found input0 low to high transition. */
         stat = RTSTEPPER_R_INPUT_TRUE;
      }
      ps->old_state_bits |= new_bit;
   }

 bugout:
   return stat;
}       /* rtstepper_is_input0_triggered() */

enum EMC_RESULT rtstepper_is_input1_triggered(struct emc_session *ps)
{
   enum EMC_RESULT stat = RTSTEPPER_R_INPUT_FALSE;
   int new_bit, old_bit;

   if (ps->fd_table.hd == NULL)
      goto bugout;

   old_bit = ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT1_BIT;
   if (ps->input1_abort_enabled)
   {
      new_bit = ps->state_bits & RTSTEPPER_STEP_STATE_INPUT1_BIT;
      if ((new_bit == RTSTEPPER_STEP_STATE_INPUT1_BIT) && (old_bit == 0))
      {
         /* Found input1 low to high transition. */
         stat = RTSTEPPER_R_INPUT_TRUE;
      }
      ps->old_state_bits |= new_bit;
   }

 bugout:
   return stat;
}       /* rtstepper_is_input1_triggered() */

enum EMC_RESULT rtstepper_is_input2_triggered(struct emc_session *ps)
{
   enum EMC_RESULT stat = RTSTEPPER_R_INPUT_FALSE;
   int new_bit, old_bit;

   if (ps->fd_table.hd == NULL)
      goto bugout;

   old_bit = ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT2_BIT;
   if (ps->input2_abort_enabled)
   {
      new_bit = ps->state_bits & RTSTEPPER_STEP_STATE_INPUT2_BIT;
      if ((new_bit == RTSTEPPER_STEP_STATE_INPUT2_BIT) && (old_bit == 0))
      {
         /* Found input2 low to high transition. */
         stat = RTSTEPPER_R_INPUT_TRUE;
      }
      ps->old_state_bits |= new_bit;
   }

 bugout:
   return stat;
}       /* rtstepper_is_input2_triggered() */

enum EMC_RESULT rtstepper_input0_state(struct emc_session *ps)
{
   enum EMC_RESULT stat = RTSTEPPER_R_INPUT_FALSE;

   if (ps->fd_table.hd == NULL)
      goto bugout;

   if (ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT0_BIT)
      stat = RTSTEPPER_R_INPUT_TRUE;

 bugout:
   return stat;
}       /* rtstepper_input0_state() */

enum EMC_RESULT rtstepper_input1_state(struct emc_session *ps)
{
   enum EMC_RESULT stat = RTSTEPPER_R_INPUT_FALSE;

   if (ps->fd_table.hd == NULL)
      goto bugout;

   if (ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT1_BIT)
      stat = RTSTEPPER_R_INPUT_TRUE;

 bugout:
   return stat;
}       /* rtstepper_input1_state() */

enum EMC_RESULT rtstepper_input2_state(struct emc_session *ps)
{
   enum EMC_RESULT stat = RTSTEPPER_R_INPUT_FALSE;

   if (ps->fd_table.hd == NULL)
      goto bugout;

   if (ps->old_state_bits & RTSTEPPER_STEP_STATE_INPUT2_BIT)
      stat = RTSTEPPER_R_INPUT_TRUE;

 bugout:
   return stat;
}       /* rtstepper_input2_state() */

/* 
 * Read dongle state bits. Blocks on 1ms intervals. Returns following bitfield definitions.
 *    abort_bit = 0x1
 *    empty_bit = 0x2 
 *    input0_bit = 0x8 
 *    input2_bit = 0x10 
 *    input3_bit = 0x20
 * All bits are active high: 1=true 0=false
 */
enum EMC_RESULT rtstepper_query_state(struct emc_session *ps)
{
   struct step_query query_response;
   enum EMC_RESULT stat;
   int len;
   static unsigned int cnt = 0;
   static int good_query = 0;

   /* Clear dongle state bits. */
   ps->state_bits &= ~(RTSTEPPER_STEP_STATE_INPUT0_BIT | RTSTEPPER_STEP_STATE_INPUT1_BIT | RTSTEPPER_STEP_STATE_INPUT2_BIT |
                       RTSTEPPER_STEP_STATE_ABORT_BIT | RTSTEPPER_STEP_STATE_EMPTY_BIT | RTSTEPPER_STEP_STATE_STALL_BIT );

   if (ps->fd_table.hd == NULL)
   {
      stat = RTSTEPPER_R_REQ_ERROR;
      goto bugout;
   }

   len = libusb_control_transfer(ps->fd_table.hd, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,      /* bmRequestType */
                         STEP_QUERY,    /* bRequest */
                         0x0,   /* wValue */
                         DONGLE_INTERFACE, /* wIndex */
                         (unsigned char *)&query_response, sizeof(query_response), LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len != sizeof(query_response))
   {
      if (cnt++ < 30)
         BUG("invalid usb query_response len=%d good_query_cnt=%d: %s\n", len, good_query, libusb_error_name(len));
      stat = RTSTEPPER_R_IO_ERROR;
      goto bugout;
   }
   else
      good_query++;

   ps->state_bits |= query_response.state_bits._word;
   stat = EMC_R_OK;

 bugout:
   return stat;
}       /* rtstepper_query_state() */

enum EMC_RESULT rtstepper_open(struct emc_session *ps)
{
   struct step_elements elements;
   enum EMC_RESULT stat = RTSTEPPER_R_IO_ERROR;
   int i, len;

   DBG("rtstepper_open() ps=%p\n", ps);

   if (ps == NULL)
      return RTSTEPPER_R_IO_ERROR;

   ps->old_state_bits = 0;
   for (i=0; i < ps->axes; i++)
   {
      ps->axis[i].clk_tail = 0;
      ps->axis[i].direction = 0;
   }

   /* Open first usb device or usb device matching specified serial number. */
   if ((stat = open_device(&ps->fd_table, ps->serial_num)) != EMC_R_OK)
   {
      if (ps->serial_num[0])
         MSG("unable to find rtstepper dongle serial number: %s\n", ps->serial_num);
      else
         MSG("unable to find rtstepper dongle\n");
      goto bugout;
   }

   /* Clear any outstanding Abort and step count. */
   memset(&elements, 0, sizeof(elements));
   len = libusb_control_transfer(ps->fd_table.hd, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,      /* bmRequestType */
                         STEP_SET,      /* bRequest */
                         0x0,   /* wValue */
                         DONGLE_INTERFACE, /* wIndex */
                         (unsigned char *) &elements, sizeof(elements), LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len != sizeof(elements))
   {
      BUG("unable to initialize dongle: %s\n", libusb_error_name(len));
      close_device(&ps->fd_table);
      goto bugout;
   }

   /* Clear any outstanding io request in the queue. */
   cancel_xfr(ps);

   stat = EMC_R_OK;

 bugout:
   return stat;
}       /* rtstepper_open() */

enum EMC_RESULT rtstepper_close(struct emc_session *ps)
{
   enum EMC_RESULT stat;

   DBG("rtstepper_close() ps=%p\n", ps);

   if (ps == NULL)
      return RTSTEPPER_R_IO_ERROR;

   stat = close_device(&ps->fd_table);

   return stat;
}       /* rtstepper_close() */
