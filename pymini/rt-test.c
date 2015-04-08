/*****************************************************************************\

  rt-test.c - test for pic firmware running on rt-stepper dongle

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
  unencumbered (ie: no Copyright or License). Patches that are accepted will 
  be applied to the GPL version, but the author reserves the rights to 
  Copyright and License.  

\*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>    /* ETIMEDOUT */
#include <sys/time.h>
#include <libusb.h>

#if (defined(__WIN32__) || defined(_WINDOWS))
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define sleep(n) Sleep(1000 * n)
#endif

#define _STRINGIZE(x) #x
#define STRINGIZE(x) _STRINGIZE(x)

#define BUG(args...) fprintf(stderr, __FILE__ " " STRINGIZE(__LINE__) ": " args)

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

static struct step_elements elements;
static struct step_query query_response;

struct rtstepper_file_descriptor
{
   libusb_device_handle *hd;
   libusb_device **list_all;
   libusb_context *ctx;
   libusb_device *dev;
};

struct rtstepper_file_descriptor fd_table;        /* usb file descriptors */

static void usage()
{
   fprintf(stdout, "rt-test %s, USB system test\n", PACKAGE_VERSION);
   fprintf(stdout, "(c) 2008-2015 Copyright Eckler Software\n");
   fprintf(stdout, "David Suffield, dsuffiel@ecklersoft.com\n");
   fprintf(stdout, "usage: rt-test [-s serial_number]\n");
   fprintf(stdout, "WARNING!!!! Do *NOT* run with parallel port connected!\n");
}

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

   BUG("Found rtstepper dongle serial_number=%s\n", sz);

   if (sn[0])
      if (strcmp(sz, sn) != 0)
         goto bugout;

   stat = 1;    /* found usb device */

 bugout:
   if (hd != NULL)
      libusb_close(hd);

   return stat;
}       /* is_rt() */

static int bulk_write(struct rtstepper_file_descriptor *pfd, const void *buf, int size, int msec)
{
   int len=-EIO, r;

   if (pfd->hd == NULL)
   {
      BUG("invalid bulk_write state\n");
      goto bugout;
   }

   r = libusb_bulk_transfer(pfd->hd, DONGLE_OUT_EP, (unsigned char *)buf, size, &len, msec);

   if (r == LIBUSB_ERROR_TIMEOUT)
   {
      len = -ETIMEDOUT;
      goto bugout;
   }

   if (r != 0)
   {
      BUG("bulk_write failed len=%d: %s\n", len, libusb_error_name(r));
      goto bugout;
   }

bugout:
   return len;
} /* musb_write() */

static int claim_interface(struct rtstepper_file_descriptor *pfd)
{
   int stat=1, r;

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

   stat=0;

bugout:
   return stat;   
} /* claim_interface() */

static int release_interface(struct rtstepper_file_descriptor *pfd)
{
   if (pfd->hd == NULL)
      return 0;

   libusb_release_interface(pfd->hd, DONGLE_INTERFACE);
   libusb_close(pfd->hd);
   libusb_free_device_list(pfd->list_all, 1);
   libusb_exit(pfd->ctx);
   pfd->hd = NULL;

   return 0;
} /* release_interface() */

static int open_device(struct rtstepper_file_descriptor *pfd, const char *sn)
{
   int stat = 1;
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

         stat = 0;
         break;
      }
   }

bugout:
   return stat;
}       /* open_device() */

int main(int argc, char *argv[])
{
   char snum[128];              /* serial number */
   char buf[65536];             /* for testing pick multiple 64-byte writes otherwise there could be a stall */
   unsigned char toggle;
   int i, ret = 10, len, tmo;

   snum[0] = 0;
   while ((i = getopt(argc, argv, "vhps:")) != -1)
   {
      switch (i)
      {
      case 's':
         strncpy(snum, optarg, sizeof(snum));
         break;
      case 'h':
         usage();
         exit(0);
      case '?':
         usage();
         fprintf(stderr, "unknown argument: %s\n", argv[1]);
         exit(-1);
      default:
         break;
      }
   }

   fprintf(stdout, "WARNING!!!! Do *NOT* run with parallel port connected!\n");

   /* Open first usb device or usb device matching specified serial number. */
   if (open_device(&fd_table, snum) != 0)
   {
      if (snum[0])
         BUG("unable to find rtstepper dongle serial number: %s\n", snum);
      else
         BUG("unable to find rtstepper dongle\n");
      goto bugout;
   }

   /* Clear device state bits and running step count. */
   len = libusb_control_transfer(fd_table.hd, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,      /* bmRequestType */
                         STEP_SET,      /* bRequest */
                         0x0,   /* wValue */
                         DONGLE_INTERFACE, /* wIndex */
                         (unsigned char *) &elements, sizeof(elements), LIBUSB_CONTROL_REQ_TIMEOUT);

   if (len != sizeof(elements))
   {
      BUG("unable to initialize dongle: %s\n", libusb_error_name(len));
      goto bugout;
   }


   /* Verify state. */
   len = libusb_control_transfer(fd_table.hd, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,      /* bmRequestType */
                         STEP_QUERY,    /* bRequest */
                         0x0,   /* wValue */
                         DONGLE_INTERFACE, /* wIndex */
                         (unsigned char *)&query_response, sizeof(query_response), LIBUSB_CONTROL_REQ_TIMEOUT);
   if (len != sizeof(query_response))
   {
      BUG("invalid query_response: %m\n");
      goto bugout;
   }

   fprintf(stdout, "query_response (Begin) len=%d trip=%d state=%x steps=%d\n", len,
           query_response.trip_cnt, query_response.state_bits._word, query_response.step);

   toggle = 0x55;
   for (i = 0; i < sizeof(buf); i++)
   {
      buf[i] = toggle;
      toggle = ~toggle;
   }

   if ((len = bulk_write(&fd_table, buf, sizeof(buf), LIBUSB_TIMEOUT)) < 0)
   {
      BUG("unable to write data len=%d snum=%s\n", len, snum);
      goto bugout;
   }
   fprintf(stdout, "wrote %d bytes\n", len);

   /* Wait for write to finish. */
   tmo = (double) sizeof(buf) * 0.000021333;    /* timeout in seconds = steps * period */
   tmo += 1;    /* plus 1 second */
   sleep(tmo);

   len = libusb_control_transfer(fd_table.hd, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,      /* bmRequestType */
                         STEP_QUERY,    /* bRequest */
                         0x0,   /* wValue */
                         DONGLE_INTERFACE, /* wIndex */
                         (unsigned char *)&query_response, sizeof(query_response), LIBUSB_CONTROL_REQ_TIMEOUT);
   if (len != sizeof(query_response))
   {
      BUG("invalid query_response: %m\n");
      goto bugout;
   }

   fprintf(stdout, "query_response (End) len=%d trip=%d state=%x steps=%d\n", len,
           query_response.trip_cnt, query_response.state_bits._word, query_response.step);

   release_interface(&fd_table);

//   if (query_response.step != sizeof(buf) || query_response.state_bits._word & STEP_STATE_STALL_BIT)
   if (query_response.step != sizeof(buf))
   {
      fprintf(stderr, "USB test failed.\n");
      fprintf(stderr, "  Suggestions in order:\n");
      fprintf(stderr, "     1. Remove any hubs, plug directly into PC.\n");
      fprintf(stderr, "     2. Try a certified HIGH-SPEED or USB 2.0 cable.\n");
      fprintf(stderr, "     3. Try a different USB port directly into PC.\n");
      fprintf(stderr, "     4. Kill all extraneous resource intensive Applications.\n");
      fprintf(stderr, "     5. Try running in a TTY terminal without Gnome or KDE.\n");
      fprintf(stderr, "     6. Try a different PC. Your USB subsytem may not support rt-stepper\n");
      ret = 1;
   }
   else
   {
      fprintf(stdout, "USB test passed.\n");
      ret = 0;
   }

 bugout:
   return ret;
}
