/************************************************************************************\

  rtstepper_log.c - Unix syslog type compatibility routines
 
  (c) 2008-2011 Copyright Eckler Software

  Author: David Suffield, dsuffiel@ecklersoft.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of version 2.1 of the GNU Lesser General Public
  License as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

  Upstream patches are welcome. Any patches submitted to the author must be 
  unencumbered (ie: no Copyright or License). Patches that are accepted will 
  be applied to the GPL version, but the author reserves the rights to 
  Copyright and License.  

\************************************************************************************/

#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include "rtstepper.h"

static FILE *logfd;
static pthread_mutex_t syslog_mutex;

static char *month[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

void rtstepper_close_log(void)
{
   if (logfd)
   {
      fclose(logfd);
      logfd = NULL;
   }
   pthread_mutex_destroy(&syslog_mutex);
}       /* rtstepper_close_log */

void rtstepper_open_log(const char *ident, int logopt)
{
   char log_file[512], backup[512];
   struct stat sb;

   if (logfd)
      rtstepper_close_log();

   sprintf(log_file, "%s.log", ident);

   if (logopt == RTSTEPPER_LOG_BACKUP && stat(log_file, &sb) == 0)
   {
      /* Save any old log file. */
      sprintf(backup, "%s.bak", ident);
      remove(backup);
      rename(log_file, backup);
   }

   logfd = fopen(log_file, "a");
   pthread_mutex_init(&syslog_mutex, NULL);
}       /* rtstepper_open_log */

void rtstepper_syslog(const char *fmt, ...)
{
   va_list args;
   struct tm *pt;
   time_t t;
   char tmp[512], msg[512];
   int n;

   if (!logfd)
      return;

   pthread_mutex_lock(&syslog_mutex);

   va_start(args, fmt);

   if ((n = vsnprintf(tmp, sizeof(tmp), fmt, args)) == -1)
      tmp[sizeof(tmp) - 1] = 0; /* output was truncated */

   t = time(NULL);
   pt = localtime(&t);
   snprintf(msg, sizeof(msg), "%s %d %d:%d:%d %s", month[pt->tm_mon], pt->tm_mday, pt->tm_hour, pt->tm_min, pt->tm_sec, tmp);
   fprintf(logfd, "%s", msg);
   fflush(logfd);

   va_end(args);

   pthread_mutex_unlock(&syslog_mutex);
}       /* rtstepper_syslog */
