/********************************************************************
* ini.c - INI file support for rtstepperemc
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* License: GPL Version 2
*    
* Copyright (c) 2004 All rights reserved.
*
* Re-written for rt-stepper dongle.
*
* Author: David Suffield, dsuffiel@ecklersoft.com
* (c) 2011-2015 Copyright Eckler Software
*
********************************************************************/

#include <unistd.h>
#include <stdio.h>      // NULL
#include <stdlib.h>     // atol(), _itoa()
#include <string.h>     // strcmp()
#include <ctype.h>      // isdigit()
#include <math.h>       // M_PI
#include <sys/types.h>
#include <sys/stat.h>
#include "emc.h"
#include "ini.h"        // these decls
#include "bug.h"

static char *_trim_tail(char *buf)
{
   int len;
   /* eat trailing white space and remove ']' */
   for (len = strlen(buf)-1; (buf[len] < ' ' || buf[len] == ']') && len >= 0; len--); 
   buf[len+1] = 0;
   return buf;
}

/* Get value for specified section and key from the ini file. */
int iniGetKeyValue(const char *section, const char *key, char *value, int value_size)
{
   struct emc_session *ps = &session;
   char rcbuf[255];
   char new_section[64];
   FILE *inFile;
   enum EMC_RESULT stat = EMC_R_INVALID_INI_FILE;
   int i, j, len;

   value[0] = j = 0;

   if((inFile = fopen(ps->ini_file, "r")) == NULL) 
   {
      BUG("unable to open %s\n", ps->ini_file);
      goto bugout;
   } 

   len = strlen(key);
   new_section[0] = 0;
   stat = EMC_R_INVALID_INI_KEY;

   /* Read the config file */
   while ((fgets(rcbuf, sizeof(rcbuf), inFile) != NULL))
   {
      if (rcbuf[0] == '[')
      {
         /* Found new section. Remove [] brackets */
         strncpy(new_section, _trim_tail(rcbuf+1), sizeof(new_section));
         new_section[sizeof(new_section) -1] = 0;
      }
      else if ((strcasecmp(new_section, section) == 0) && (strncasecmp(rcbuf, key, len) == 0))
      {
         for (i=len; rcbuf[i] == ' ' && rcbuf[i]; i++);  /* eat white space before = */

         if (rcbuf[i] != '=')
            continue;  /* keep looking... */

         for (i++; rcbuf[i] == ' ' && rcbuf[i]; i++);  /* eat white space after = */
  
         for (j=0; rcbuf[i] >= ' ' && j < value_size; i++, j++) /* copy value(s) */
            value[j] = rcbuf[i];

         if (j < value_size)
            value[j] = 0;      /* zero terminate */
         else
            value[value_size -1] = 0;  /* force zero termination */

         stat = EMC_R_OK;
         break;  /* done */
      }
   }
        
bugout:        
   if (inFile != NULL)
      fclose(inFile);

   if (stat != EMC_R_OK)
   {
      BUG("unable to find %s %s in %s\n", section, key, ps->ini_file);
      return stat;
   }
   
   return j;   /* return length does not include zero termination */
}   /* iniGetKeyValue() */


