/********************************************************************
* ini.cc - INI file support for EMC2
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
*  (c) 2012 Copyright Eckler Software
*
* Last change:
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

class EmcIniFile:public IniFile
{
 public:
 EmcIniFile(int errMask = 0):IniFile(errMask)
   {
   }

   ErrorCode Find(EmcAxisType * result, const char *tag, const char *section = NULL, int num = 1);
   ErrorCode Find(bool * result, const char *tag, const char *section, int num = 1);
   ErrorCode FindLinearUnits(EmcLinearUnits * result, const char *tag, const char *section = NULL, int num = 1);
   ErrorCode FindAngularUnits(EmcAngularUnits * result, const char *tag, const char *section = NULL, int num = 1);

   // From base class.
   ErrorCode Find(int *result, int min, int max, const char *tag, const char *section, int num = 1)
   {
      return (IniFile::Find(result, min, max, tag, section, num));
   }
   ErrorCode Find(int *result, const char *tag, const char *section = NULL, int num = 1)
   {
      return (IniFile::Find(result, tag, section, num));
   }
   ErrorCode Find(double *result, double min, double max, const char *tag, const char *section, int num = 1)
   {
      return (IniFile::Find(result, min, max, tag, section, num));
   }
   ErrorCode Find(double *result, const char *tag, const char *section = NULL, int num = 1)
   {
      return (IniFile::Find(result, tag, section, num));
   }
   const char *Find(const char *tag, const char *section = NULL, int num = 1)
   {
      return (IniFile::Find(tag, section, num));
   }

 private:
   static StrIntPair axisTypeMap[];
   static StrIntPair boolMap[];
   static StrDoublePair linearUnitsMap[];
   static StrDoublePair angularUnitsMap[];
};

/// Return TRUE if the line has a line-ending problem
static bool check_line_endings(const char *s)
{
   if (!s)
      return false;
   for (; *s; s++)
   {
      if (*s == '\r')
      {
         char c = s[1];
         if (c == '\n' || c == '\0')
         {
            static bool warned = 0;
            if (!warned)
               fprintf(stderr, "inifile: warning: File contains DOS-style line endings.\n");
            warned = true;
            continue;
         }
         fprintf(stderr, "inifile: error: File contains ambiguous carriage returns\n");
         return true;
      }
   }
   return false;
}

IniFile::StrIntPair EmcIniFile::axisTypeMap[] =
{
   {
   "LINEAR", EMC_AXIS_LINEAR}
   ,
   {
   "ANGULAR", EMC_AXIS_ANGULAR}
   ,
   {
   NULL, 0}
,};

IniFile::IniFile(int _errMask, FILE * _fp)
{
   fp = _fp;
   errMask = _errMask;
//    owned = false;

//    if(fp != NULL)
//        LockFile();
}


/*! Opens the file for reading. If a file was already open, it is closed
   and the new one opened.

   @return true on success, false on failure */
bool IniFile::Open(const char *file)
{
   char path[LINELEN] = "";

   DBG("IniFile::Open() file=%s\n", file);

   if (IsOpen())
      Close();

   TildeExpansion(file, path);

   if ((fp = fopen(path, "r")) == NULL)
      return (false);

//    owned = true;

//    if(!LockFile())
//        return(false);

   return (true);
}


/*! Closes the file descriptor..

   @return true on success, false on failure */
bool IniFile::Close()
{
   int rVal = 0;

   if (fp != NULL)
   {
//        lock.l_type = F_UNLCK;
//        fcntl(fileno(fp), F_SETLKW, &lock);

//        if(owned)
      rVal = fclose(fp);

      fp = NULL;
   }

   return (rVal == 0);
}


IniFile::ErrorCode IniFile::Find(int *result, int min, int max, const char *tag, const char *section, int num)
{
   ErrorCode errCode;
   int tmp;

   if ((errCode = Find(&tmp, tag, section, num)) != ERR_NONE)
      return (errCode);

   if ((tmp > max) || (tmp < min))
      return (ERR_LIMITS);

   *result = tmp;

   return (ERR_NONE);
}


IniFile::ErrorCode IniFile::Find(int *result, const char *tag, const char *section, int num)
{
   const char *pStr;
   int tmp;

   if ((pStr = Find(tag, section, num)) == NULL)
   {
      // We really need an ErrorCode return from Find() and should be passing
      // in a buffer. Just pick a suitable ErrorCode for now.
      return (ERR_TAG_NOT_FOUND);
   }

   if (sscanf(pStr, "%i", &tmp) != 1)
   {
      ThrowException(ERR_CONVERSION);
      return (ERR_CONVERSION);
   }

   *result = tmp;

   return (ERR_NONE);
}


IniFile::ErrorCode IniFile::Find(double *result, double min, double max, const char *tag, const char *section, int num)
{
   ErrorCode errCode;
   double tmp;

   if ((errCode = Find(&tmp, tag, section, num)) != ERR_NONE)
      return (errCode);

   if ((tmp > max) || (tmp < min))
      return (ERR_LIMITS);

   *result = tmp;

   return (ERR_NONE);
}


IniFile::ErrorCode IniFile::Find(double *result, const char *tag, const char *section, int num)
{
   const char *pStr;
   double tmp;

   if ((pStr = Find(tag, section, num)) == NULL)
   {
      // We really need an ErrorCode return from Find() and should be passing
      // in a buffer. Just pick a suitable ErrorCode for now.
      return (ERR_TAG_NOT_FOUND);
   }

   if (sscanf(pStr, "%lf", &tmp) != 1)
   {
      ThrowException(ERR_CONVERSION);
      return (ERR_CONVERSION);
   }

   *result = tmp;

   return (ERR_NONE);
}


IniFile::ErrorCode IniFile::Find(int *result, StrIntPair * pPair, const char *tag, const char *section, int num)
{
   const char *pStr;
   int tmp;

   if ((pStr = Find(tag, section, num)) == NULL)
   {
      // We really need an ErrorCode return from Find() and should be passing
      // in a buffer. Just pick a suitable ErrorCode for now.
      return (ERR_TAG_NOT_FOUND);
   }

   if (sscanf(pStr, "%i", &tmp) == 1)
   {
      *result = tmp;
      return (ERR_NONE);
   }

   while (pPair->pStr != NULL)
   {
      if (strcasecmp(pStr, pPair->pStr) == 0)
      {
         *result = pPair->value;
         return (ERR_NONE);
      }
      pPair++;
   }

   ThrowException(ERR_CONVERSION);
   return (ERR_CONVERSION);
}


IniFile::ErrorCode IniFile::Find(double *result, StrDoublePair * pPair, const char *tag, const char *section, int num)
{
   const char *pStr;
   double tmp;

   if ((pStr = Find(tag, section, num)) == NULL)
   {
      // We really need an ErrorCode return from Find() and should be passing
      // in a buffer. Just pick a suitable ErrorCode for now.
      return (ERR_TAG_NOT_FOUND);
   }

   if (sscanf(pStr, "%lf", &tmp) == 1)
   {
      *result = tmp;
      return (ERR_NONE);
   }

   while (pPair->pStr != NULL)
   {
      if (strcasecmp(pStr, pPair->pStr) == 0)
      {
         *result = pPair->value;
         return (ERR_NONE);
      }
      pPair++;
   }

   ThrowException(ERR_CONVERSION);
   return (ERR_CONVERSION);
}


/*! Finds the nth tag in section.

   @param tag Entry in the ini file to find.

   @param section The section to look for the tag.

   @param num (optionally) the Nth occurrence of the tag.

   @return pointer to the the variable after the '=' delimiter */
const char *IniFile::Find(const char *_tag, const char *_section, int _num)
{
   // WTF, return a pointer to the middle of a local buffer?
   // FIX: this is totally non-reentrant.
   static char line[LINELEN + 2] = "";  /* 1 for newline, 1 for NULL */
   char bracketSection[LINELEN + 2] = "";
   char *nonWhite;
   int newLinePos;              /* position of newline to strip */
   int len;
   char tagEnd;
   char *valueString;
   char *endValueString;
   ErrorCode stat;

   // For exceptions.
   lineNo = 0;
   tag = _tag;
   section = _section;
   num = _num;

   /* check valid file */
   if (!CheckIfOpen())
      return (NULL);

   /* start from beginning */
   rewind(fp);

   /* check for section first-- if it's non-NULL, then position file at
      line after [section] */
   if (section != NULL)
   {
      sprintf(bracketSection, "[%s]", section);

      /* find [section], and position fp just after it */
      while (!feof(fp))
      {

         if (NULL == fgets(line, LINELEN + 1, fp))
         {
            /* got to end of file without finding it */
            stat = ERR_SECTION_NOT_FOUND;
            goto bugout;
         }

         if (check_line_endings(line))
         {
            stat = ERR_CONVERSION;
            goto bugout;
         }

         /* got a line */
         lineNo++;

         /* strip off newline */
         newLinePos = strlen(line) - 1; /* newline is on back from 0 */
         if (newLinePos < 0)
         {
            newLinePos = 0;
         }
         if (line[newLinePos] == '\n')
         {
            line[newLinePos] = 0;       /* make the newline 0 */
         }

         if (NULL == (nonWhite = SkipWhite(line)))
         {
            /* blank line-- skip */
            continue;
         }

         /* not a blank line, and nonwhite is first char */
         if (strncmp(bracketSection, nonWhite, strlen(bracketSection)) != 0)
         {
            /* not on this line */
            continue;
         }

         /* it matches-- fp is now set up for search on tag */
         break;
      }
   }

   while (!feof(fp))
   {
      /* check for end of file */
      if (NULL == fgets(line, LINELEN + 1, (FILE *) fp))
      {
         /* got to end of file without finding it */
         stat = ERR_TAG_NOT_FOUND;
         goto bugout;
      }

      if (check_line_endings(line))
      {
         stat = ERR_CONVERSION;
         goto bugout;
      }

      /* got a line */
      lineNo++;

      /* strip off newline */
      newLinePos = strlen(line) - 1;    /* newline is on back from 0 */
      if (newLinePos < 0)
      {
         newLinePos = 0;
      }
      if (line[newLinePos] == '\n')
      {
         line[newLinePos] = 0;  /* make the newline 0 */
      }

      /* skip leading whitespace */
      if (NULL == (nonWhite = SkipWhite(line)))
      {
         /* blank line-- skip */
         continue;
      }

      /* check for '[' char-- if so, it's a section tag, and we're out
         of our section */
      if (NULL != section && nonWhite[0] == '[')
      {
         stat = ERR_TAG_NOT_FOUND;
         goto bugout;
      }

      len = strlen(tag);
      if (strncmp(tag, nonWhite, len) != 0)
      {
         /* not on this line */
         continue;
      }

      /* it matches the first part of the string-- if whitespace or = is
         next char then call it a match */
      tagEnd = nonWhite[len];
      if (tagEnd == ' ' || tagEnd == '\r' || tagEnd == '\t' || tagEnd == '\n' || tagEnd == '=')
      {
         /* it matches-- return string after =, or NULL */
         if (--_num > 0)
         {
            /* Not looking for this one, so skip it... */
            continue;
         }
         nonWhite += len;
         valueString = AfterEqual(nonWhite);
         /* Eliminate white space at the end of a line also. */
         if (NULL == valueString)
         {
            stat = ERR_TAG_NOT_FOUND;
            goto bugout;
         }
         endValueString = valueString + strlen(valueString) - 1;
         while (*endValueString == ' ' || *endValueString == '\t' || *endValueString == '\r')
         {
            *endValueString = 0;
            endValueString--;
         }
         DBG("IniFile::Find() sect=%s key=%s val=%s\n", section, tag, valueString);
         return (valueString);
      }
      /* else continue */
   }

   stat = ERR_TAG_NOT_FOUND;

 bugout:
   DBG("IniFile::Find() sect=%s key=%s val=NULL\n", section, tag);
   ThrowException(stat);
   return (NULL);
}


bool IniFile::CheckIfOpen(void)
{
   if (IsOpen())
      return (true);

   ThrowException(ERR_NOT_OPEN);

   return (false);
}

#if 0
bool IniFile::LockFile(void)
{
   lock.l_type = F_RDLCK;
   lock.l_whence = SEEK_SET;
   lock.l_start = 0;
   lock.l_len = 0;

   if (fcntl(fileno(fp), F_SETLK, &lock) == -1)
   {
      if (owned)
         fclose(fp);

      fp = NULL;
      return (false);
   }

   return (true);
}
#endif

/*! Expands the tilde to $(HOME) and concatenates file to it. If the first char
    If file does not start with ~/, file will be copied into path as-is. 

   @param the input filename

   @param pointer for returning the resulting expanded name

 */
void IniFile::TildeExpansion(const char *file, char *path)
{
   char *home;

   strncpy(path, file, LINELEN);
   if (strlen(file) < 2 || !(file[0] == '~' && file[1] == '/'))
   {
      /* no tilde expansion required, or unsupported
         tilde expansion type requested */
      return;
   }

   home = getenv("HOME");
   if (!home || strlen(home) + strlen(file) > LINELEN)
   {
      return;
   }

   /* Buffer overflow has already been checked. */

   strcpy(path, home);
   strcat(path, file + 1);
   return;
}


void IniFile::ThrowException(ErrorCode errCode)
{
   if (errCode & errMask)
   {
      exception.errCode = errCode;
      exception.tag = tag;
      exception.section = section;
      exception.num = num;
      exception.lineNo = lineNo;
      throw(exception);
   }
}


/*! Ignoring any tabs, spaces or other white spaces, finds the first
   character after the '=' delimiter.

   @param string Pointer to the tag

   @return NULL or pointer to first non-white char after the delimiter

   Called By: find() and section() only. */
char *IniFile::AfterEqual(const char *string)
{
   const char *spot = string;

   for (;;)
   {
      if (*spot == '=')
      {
         /* = is there-- return next non-white, or NULL if not there */
         for (;;)
         {
            spot++;
            if (0 == *spot)
            {
               /* ran out */
               return (NULL);
            }
            else if (*spot != ' ' && *spot != '\t' && *spot != '\r' && *spot != '\n')
            {
               /* matched! */
               return ((char *) spot);
            }
            else
            {
               /* keep going for the text */
               continue;
            }
         }
      }
      else if (*spot == 0)
      {
         /* end of string */
         return (NULL);
      }
      else
      {
         /* haven't seen '=' yet-- keep going */
         spot++;
         continue;
      }
   }
}


/*! Finds the first non-white character on a new line and returns a
   pointer. Ignores any line that starts with a comment char i.e. a ';' or 
   '#'.

   @return NULL if not found or a valid pointer.

   Called By: find() and section() only. */
char *IniFile::SkipWhite(const char *string)
{
   while (true)
   {
      if (*string == 0)
      {
         return (NULL);
      }

      if ((*string == ';') || (*string == '#'))
      {
         return (NULL);
      }

      if (*string != ' ' && *string != '\t' && *string != '\r' && *string != '\n')
      {
         return ((char *) string);
      }

      string++;
   }
}


void IniFile::Exception::Print(FILE * fp)
{
   const char *msg;

   switch (errCode)
   {
   case ERR_NONE:
      msg = "ERR_NONE";
      break;

   case ERR_NOT_OPEN:
      msg = "ERR_NOT_OPEN";
      break;

   case ERR_SECTION_NOT_FOUND:
      msg = "ERR_SECTION_NOT_FOUND";
      break;

   case ERR_TAG_NOT_FOUND:
      msg = "ERR_TAG_NOT_FOUND";
      break;

   case ERR_CONVERSION:
      msg = "ERR_CONVERSION";
      break;

   case ERR_LIMITS:
      msg = "ERR_LIMITS";
      break;

   default:
      msg = "UNKNOWN";
   }

   fprintf(fp, "INIFILE: %s, section=%s, tag=%s, num=%d, lineNo=%d\n", msg, section, tag, num, lineNo);
}


const char *iniFind(FILE * fp, const char *tag, const char *section)
{
   IniFile f(false, fp);

   return (f.Find(tag, section));
}

EmcIniFile::ErrorCode EmcIniFile::Find(EmcAxisType * result, const char *tag, const char *section, int num)
{
   return (IniFile::Find((int *) result, axisTypeMap, tag, section, num));
}


IniFile::StrIntPair EmcIniFile::boolMap[] =
{
   {
   "TRUE", 1},
   {
   "YES", 1},
   {
   "1", 1},
   {
   "FALSE", 0},
   {
   "NO", 0},
   {
   "0", 0},
   {
NULL, 0},};

EmcIniFile::ErrorCode EmcIniFile::Find(bool * result, const char *tag, const char *section, int num)
{
   ErrorCode errCode;
   int value;

   if ((errCode = IniFile::Find(&value, boolMap, tag, section, num)) == ERR_NONE)
   {
      *result = (bool) value;
   }

   return (errCode);
}


// The next const struct holds pairs for linear units which are 
// valid under the [TRAJ] section. These are of the form {"name", value}.
// If the name "name" is encountered in the ini, the value will be used.
EmcIniFile::StrDoublePair EmcIniFile::linearUnitsMap[] =
{
   {
   "mm", 1.0}
   ,
   {
   "metric", 1.0}
   ,
   {
   "in", 1 / 25.4}
   ,
   {
   "inch", 1 / 25.4}
   ,
   {
   "imperial", 1 / 25.4}
   ,
   {
   NULL, 0}
,};

EmcIniFile::ErrorCode EmcIniFile::FindLinearUnits(EmcLinearUnits * result, const char *tag, const char *section, int num)
{
   return (IniFile::Find((double *) result, linearUnitsMap, tag, section, num));
}


// The next const struct holds pairs for angular units which are 
// valid under the [TRAJ] section. These are of the form {"name", value}.
// If the name "name" is encountered in the ini, the value will be used.
EmcIniFile::StrDoublePair EmcIniFile::angularUnitsMap[] =
{
   {
   "deg", 1.0},
   {
   "degree", 1.0},
   {
   "grad", 0.9},
   {
   "gon", 0.9},
   {
   "rad", M_PI / 180},
   {
   "radian", M_PI / 180},
   {
NULL, 0},};

EmcIniFile::ErrorCode EmcIniFile::FindAngularUnits(EmcAngularUnits * result, const char *tag, const char *section, int num)
{
   return (IniFile::Find((double *) result, angularUnitsMap, tag, section, num));
}

/*
  loadAxis(int axis)

  Loads ini file params for axis, axis = 0, ...

  TYPE <LINEAR ANGULAR>        type of axis
  UNITS <float>                units per mm or deg
  MAX_VELOCITY <float>         max vel for axis
  MAX_ACCELERATION <float>     max accel for axis
  BACKLASH <float>             backlash
  INPUT_SCALE <float> <float>  scale, offset
  OUTPUT_SCALE <float> <float> scale, offset
  MIN_LIMIT <float>            minimum soft position limit
  MAX_LIMIT <float>            maximum soft position limit
  FERROR <float>               maximum following error, scaled to max vel
  MIN_FERROR <float>           minimum following error
  HOME <float>                 home position (where to go after home)
  HOME_FINAL_VEL <float>       speed to move from HOME_OFFSET to HOME location (at the end of homing)
  HOME_OFFSET <float>          home switch/index pulse location
  HOME_SEARCH_VEL <float>      homing speed, search phase
  HOME_LATCH_VEL <float>       homing speed, latch phase
  HOME_USE_INDEX <bool>        use index pulse when homing?
  HOME_IGNORE_LIMITS <bool>    ignore limit switches when homing?
  COMP_FILE <filename>         file of axis compensation points

  calls:

  emcAxisSetAxis(int axis, unsigned char axisType);
  emcAxisSetUnits(int axis, double units);
  emcAxisSetBacklash(int axis, double backlash);
  emcAxisSetInterpolationRate(int axis, int rate);
  emcAxisSetInputScale(int axis, double scale, double offset);
  emcAxisSetOutputScale(int axis, double scale, double offset);
  emcAxisSetMinPositionLimit(int axis, double limit);
  emcAxisSetMaxPositionLimit(int axis, double limit);
  emcAxisSetFerror(int axis, double ferror);
  emcAxisSetMinFerror(int axis, double ferror);
  emcAxisSetHomingParams(int axis, double home, double offset,
  double search_vel, double latch_vel, int use_index, int ignore_limits );
  emcAxisActivate(int axis);
  emcAxisDeactivate(int axis);
  emcAxisSetMaxVelocity(int axis, double vel);
  emcAxisSetMaxAcceleration(int axis, double acc);
  emcAxisLoadComp(int axis, const char * file);
  emcAxisLoadComp(int axis, const char * file);
  */

static int loadAxis(int axis, EmcIniFile * axisIniFile)
{
   char axisString[16];
   EmcAxisType axisType;
   double units;
   double backlash;
   double offset;
   double limit;
   double home;
   double search_vel;
   double latch_vel;
   double home_final_vel;       // moving from OFFSET to HOME
   bool use_index;
   bool ignore_limits;
   bool is_shared;
   int sequence;
   int volatile_home;
   double maxVelocity;
   double maxAcceleration;
   double ferror;
   double scale;
   int pin, retval;

   // compose string to match, axis = 0 -> AXIS_0, etc.
   sprintf(axisString, "AXIS_%d", axis);

   axisIniFile->EnableExceptions(EmcIniFile::ERR_CONVERSION);

   try
   {
      // set axis type
      axisType = EMC_AXIS_LINEAR;       // default
      axisIniFile->Find(&axisType, "TYPE", axisString);
      if ((retval = emcAxisSetAxis(axis, axisType)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetAxis\n");
         return retval;
      }

      // set units
      if (axisType == EMC_AXIS_LINEAR)
      {
         units = emcmotStatus.traj.linearUnits;
         axisIniFile->FindLinearUnits(&units, "UNITS", axisString);
      }
      else
      {
         units = emcmotStatus.traj.angularUnits;
         axisIniFile->FindAngularUnits(&units, "UNITS", axisString);
      }
      if ((retval = emcAxisSetUnits(axis, units)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetUnits\n");
         return retval;
      }

      // set backlash
      backlash = 0;     // default
      axisIniFile->Find(&backlash, "BACKLASH", axisString);
      if ((retval = emcAxisSetBacklash(axis, backlash)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetBacklash\n");
         return retval;
      }

      // set min position limit
      limit = -1e99;    // default
      axisIniFile->Find(&limit, "MIN_LIMIT", axisString);
      if ((retval = emcAxisSetMinPositionLimit(axis, limit)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetMinPositionLimit\n");
         return retval;
      }

      // set max position limit
      limit = 1e99;     // default
      axisIniFile->Find(&limit, "MAX_LIMIT", axisString);
      if ((retval = emcAxisSetMaxPositionLimit(axis, limit)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetMaxPositionLimit\n");
         return retval;
      }

      // set following error limit (at max speed)
      ferror = 1;       // default
      axisIniFile->Find(&ferror, "FERROR", axisString);
      if ((retval = emcAxisSetFerror(axis, ferror)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetFerror\n");
         return retval;
      }

      // do MIN_FERROR, if it's there. If not, use value of maxFerror above
      axisIniFile->Find(&ferror, "MIN_FERROR", axisString);
      if ((retval = emcAxisSetMinFerror(axis, ferror)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetMinFerror\n");
         return retval;
      }

      // set homing paramsters (total of 6)
      home = 0; // default
      axisIniFile->Find(&home, "HOME", axisString);
      offset = 0;       // default
      axisIniFile->Find(&offset, "HOME_OFFSET", axisString);
      search_vel = 0;   // default
      axisIniFile->Find(&search_vel, "HOME_SEARCH_VEL", axisString);
      latch_vel = 0;    // default
      axisIniFile->Find(&latch_vel, "HOME_LATCH_VEL", axisString);
      home_final_vel = -1;      // default (rapid)
      axisIniFile->Find(&home_final_vel, "HOME_FINAL_VEL", axisString);
      is_shared = false;        // default
      axisIniFile->Find(&is_shared, "HOME_IS_SHARED", axisString);
      use_index = false;        // default
      axisIniFile->Find(&use_index, "HOME_USE_INDEX", axisString);
      ignore_limits = false;    // default
      axisIniFile->Find(&ignore_limits, "HOME_IGNORE_LIMITS", axisString);
      sequence = -1;    // default
      axisIniFile->Find(&sequence, "HOME_SEQUENCE", axisString);
      volatile_home = 0;        // default
      axisIniFile->Find(&volatile_home, "VOLATILE_HOME", axisString);

      if ((retval = emcAxisSetHomingParams(axis, home, offset, home_final_vel, search_vel, latch_vel,
                                           (int) use_index, (int) ignore_limits, (int) is_shared, sequence, volatile_home)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetHomingParams\n");
         return retval;
      }

      // set maximum velocity
      maxVelocity = DEFAULT_AXIS_MAX_VELOCITY;
      axisIniFile->Find(&maxVelocity, "MAX_VELOCITY", axisString);
      if ((retval = emcAxisSetMaxVelocity(axis, maxVelocity)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetMaxVelocity\n");
         return retval;
      }

      // set max acceleration
      maxAcceleration = DEFAULT_AXIS_MAX_ACCELERATION;
      axisIniFile->Find(&maxAcceleration, "MAX_ACCELERATION", axisString);
      if ((retval != emcAxisSetMaxAcceleration(axis, maxAcceleration)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetMaxAcceleration\n");
         return retval;
      }

      // set input scale
      axisIniFile->Find(&scale, "INPUT_SCALE", axisString);
      if ((retval = emcAxisSetInputScale(axis, scale)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetInputScale\n");
         return retval;
      }

      // set step pin
      axisIniFile->Find(&pin, "STEP_PIN", axisString);
      if ((retval = emcAxisSetStepPin(axis, pin)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetStepPin\n");
         return retval;
      }

      // set direction pin
      axisIniFile->Find(&pin, "DIRECTION_PIN", axisString);
      if ((retval = emcAxisSetDirectionPin(axis, pin)) != EMC_R_OK)
      {
         BUG("bad return from emcAxisSetDirectionPin\n");
         return retval;
      }
   }

   catch(EmcIniFile::Exception & e)
   {
      e.Print();
      return EMC_R_ERROR;
   }

   // lastly, activate axis. Do this last so that the motion controller
   // won't flag errors midway during configuration
   return emcAxisActivate(axis);
}  /* loadAxis() */

/*
  iniAxis(int axis, const char *filename)

  Loads ini file parameters for specified axis, [0 .. AXES - 1]

  Looks for AXES in TRAJ section for how many to do, up to
  EMC_AXIS_MAX.
 */
int iniAxis(int axis, const char *filename)
{
   int axes;
   EmcIniFile axisIniFile(EmcIniFile::ERR_TAG_NOT_FOUND | EmcIniFile::ERR_SECTION_NOT_FOUND | EmcIniFile::ERR_CONVERSION);

   DBG("iniAxis() axis=%d\n", axis);

   if (axisIniFile.Open(filename) == false)
      return EMC_R_ERROR;

   try
   {
      axisIniFile.Find(&axes, "AXES", "TRAJ");
   }

   catch(EmcIniFile::Exception & e)
   {
      e.Print();
      return EMC_R_ERROR;
   }

   if (axis < 0 || axis >= axes)
   {
      // requested axis exceeds machine axes
      return EMC_R_ERROR;
   }

   // load its values
   return loadAxis(axis, &axisIniFile);
}       /* iniAxis() */

/*
  loadTraj()

  Loads ini file params for traj

  AXES <int>                    number of axes in system
  LINEAR_UNITS <float>          units per mm
  ANGULAR_UNITS <float>         units per degree
  CYCLE_TIME <float>            cycle time for traj calculations
  DEFAULT_VELOCITY <float>      default velocity
  MAX_VELOCITY <float>          max velocity
  MAX_ACCELERATION <float>      max acceleration
  DEFAULT_ACCELERATION <float>  default acceleration
  HOME <float> ...              world coords of home, in X Y Z R P W

  calls:

  emcTrajSetAxes(int axes);
  emcTrajSetUnits(double linearUnits, double angularUnits);
  emcTrajSetCycleTime(double cycleTime);
  emcTrajSetVelocity(double vel);
  emcTrajSetAcceleration(double acc);
  emcTrajSetMaxVelocity(double vel);
  emcTrajSetMaxAcceleration(double acc);
  emcTrajSetHome(EmcPose home);
  */

static int loadTraj(EmcIniFile * trajInifile)
{
   const char *inistring;
   EmcLinearUnits linearUnits;
   EmcAngularUnits angularUnits;
   double vel;
   double acc;
   unsigned char coordinateMark[6] = { 1, 1, 1, 0, 0, 0 };
   int t, len, retval;
   char homes[LINELEN];
   char home[LINELEN];
   EmcPose homePose = { {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
   double d;

   trajInifile->EnableExceptions(EmcIniFile::ERR_CONVERSION);

   try
   {
      int axes = 0;
      int axismask = 0;
      const char *coord = trajInifile->Find("COORDINATES", "TRAJ");
      if (coord)
      {
         if (strchr(coord, 'x') || strchr(coord, 'X'))
            axismask |= 1;
         if (strchr(coord, 'y') || strchr(coord, 'Y'))
            axismask |= 2;
         if (strchr(coord, 'z') || strchr(coord, 'Z'))
            axismask |= 4;
         if (strchr(coord, 'a') || strchr(coord, 'A'))
            axismask |= 8;
         if (strchr(coord, 'b') || strchr(coord, 'B'))
            axismask |= 16;
         if (strchr(coord, 'c') || strchr(coord, 'C'))
            axismask |= 32;
         if (strchr(coord, 'u') || strchr(coord, 'U'))
            axismask |= 64;
         if (strchr(coord, 'v') || strchr(coord, 'V'))
            axismask |= 128;
         if (strchr(coord, 'w') || strchr(coord, 'W'))
            axismask |= 256;
      }
      else
      {
         axismask = 1 | 2 | 4;  // default: XYZ machine
      }
      trajInifile->Find(&axes, "AXES", "TRAJ");

      if ((retval = emcTrajSetAxes(axes, axismask)) != EMC_R_OK)
      {
         BUG("bad return value from emcTrajSetAxes\n");
         return retval;
      }

      linearUnits = 0;
      trajInifile->FindLinearUnits(&linearUnits, "LINEAR_UNITS", "TRAJ");
      angularUnits = 0;
      trajInifile->FindAngularUnits(&angularUnits, "ANGULAR_UNITS", "TRAJ");

      if ((retval = emcTrajSetUnits(linearUnits, angularUnits)) != EMC_R_OK)
      {
         BUG("bad return value from emcTrajSetUnits\n");
         return retval;
      }

      vel = 1e99;       // by default, use AXIS limit
      trajInifile->Find(&vel, "MAX_VELOCITY", "TRAJ");

      // set the corresponding global
      TRAJ_MAX_VELOCITY = vel;

      // and set dynamic value
      if ((retval = emcTrajSetMaxVelocity(vel)) != EMC_R_OK)
      {
         BUG("bad return value from emcTrajSetMaxVelocity\n");
         return retval;
      }

      vel = 1.0;
      trajInifile->Find(&vel, "DEFAULT_VELOCITY", "TRAJ");

      // set the corresponding global
      TRAJ_DEFAULT_VELOCITY = vel;

      // and set dynamic value
      if ((retval = emcTrajSetVelocity(0, vel)) != EMC_R_OK)
      { //default velocity on startup 0
         BUG("bad return value from emcTrajSetVelocity\n");
         return retval;
      }

      acc = 1e99;       // let the axis values apply
      trajInifile->Find(&acc, "MAX_ACCELERATION", "TRAJ");

      if ((retval = emcTrajSetMaxAcceleration(acc)) != EMC_R_OK)
      {
         BUG("bad return value from emcTrajSetMaxAcceleration\n");
         return retval;
      }

      acc = 1e99;       // let the axis values apply
      trajInifile->Find(&acc, "DEFAULT_ACCELERATION", "TRAJ");

      if ((retval = emcTrajSetAcceleration(acc)) != EMC_R_OK)
      {
         BUG("bad return value from emcTrajSetAcceleration\n");
         return retval;
      }
   }

   catch(EmcIniFile::Exception & e)
   {
      e.Print();
      return EMC_R_ERROR;
   }

   // set coordinateMark[] to hold 1's for each coordinate present,
   // so that home position can be interpreted properly
   if ((inistring = trajInifile->Find("COORDINATES", "TRAJ")) != NULL)
   {
      len = strlen(inistring);
      // there's an entry in ini file, so clear all the marks out first
      // so that defaults don't apply at all
      for (t = 0; t < 6; t++)
         coordinateMark[t] = 0;

      // now set actual marks
      for (t = 0; t < len; t++)
      {
         if (inistring[t] == 'X')
            coordinateMark[0] = 1;
         else if (inistring[t] == 'Y')
            coordinateMark[1] = 1;
         else if (inistring[t] == 'Z')
            coordinateMark[2] = 1;
         else if (inistring[t] == 'R')
            coordinateMark[3] = 1;
         else if (inistring[t] == 'P')
            coordinateMark[4] = 1;
         else if (inistring[t] == 'W')
            coordinateMark[5] = 1;
      }
   }

   if ((inistring = trajInifile->Find("HOME", "TRAJ")) != NULL)
   {
      // found it, now interpret it according to coordinateMark[]
      strcpy(homes, inistring);
      len = 0;
      for (t = 0; t < 6; t++)
      {
         if (!coordinateMark[t])
            continue;   // position t at index of next non-zero mark

         // there is a mark, so read the string for a value
         if (1 == sscanf(&homes[len], "%s", home) && 1 == sscanf(home, "%lf", &d))
         {
            // got an entry, index into coordinateMark[] is 't'
            if (t == 0)
               homePose.tran.x = d;
            else if (t == 1)
               homePose.tran.y = d;
            else if (t == 2)
               homePose.tran.z = d;
            else if (t == 3)
               homePose.a = d;
            else if (t == 4)
               homePose.b = d;
            else
               homePose.c = d;

            // position string ptr past this value
            len += strlen(home);
            // and at start of next value
            while ((len < LINELEN) && (homes[len] == ' ' || homes[len] == '\t'))
               len++;

            if (len >= LINELEN)
               break;   // out of for loop

         }
         else
         {
            // badly formatted entry
            BUG("invalid inifile value for [TRAJ] HOME: %s\n", inistring);
            return EMC_R_ERROR;
         }
      } // end of for-loop on coordinateMark[]
   }

   if ((retval = emcTrajSetHome(homePose)) != EMC_R_OK)
   {
      BUG("bad return value from emcTrajSetHome\n");
      return retval;
   }

   return EMC_R_OK;
}  /* loadTraj() */

/* Loads ini file parameters for trajectory, from [TRAJ] section. */
int iniTraj(const char *filename)
{
   EmcIniFile trajInifile;

   DBG("iniTraj()\n");

   if (trajInifile.Open(filename) == false)
      return EMC_R_ERROR;

   // load trajectory values
   return loadTraj(&trajInifile);
}  /* iniTraj() */

/*
  loadTool()

  Loads ini file params for spindle from [EMCIO] section

  TOOL_TABLE <file name>  name of tool table file

  calls:

  emcToolSetToolTableFile(const char filename);
  */

static int loadTool(IniFile * toolInifile)
{
   int retval = EMC_R_ERROR;
   const char *inistring;

   if ((inistring = toolInifile->Find("TOOL_TABLE", "EMCIO")) != NULL)
   {
      if ((retval = emcToolSetToolTableFile(inistring) != EMC_R_OK))
      {
         emcOperatorError(0, EMC_I18N("emcToolSetToolTableFile() failed"));
      }
   }

   return retval;
}  /* loadTool() */

/*
  readToolChange() reads the values of [EMCIO] TOOL_CHANGE_POSITION and
  TOOL_HOLDER_CLEAR, and loads them into their associated globals 
*/
static int readToolChange(IniFile * toolInifile)
{
   int retval = 0;
   const char *inistring;

   if ((inistring = toolInifile->Find("TOOL_CHANGE_POSITION", "EMCIO")) != NULL)
   {
      /* found an entry */
      if (9 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                      &TOOL_CHANGE_POSITION.tran.x,
                      &TOOL_CHANGE_POSITION.tran.y,
                      &TOOL_CHANGE_POSITION.tran.z,
                      &TOOL_CHANGE_POSITION.a,
                      &TOOL_CHANGE_POSITION.b, &TOOL_CHANGE_POSITION.c, &TOOL_CHANGE_POSITION.u, &TOOL_CHANGE_POSITION.v, &TOOL_CHANGE_POSITION.w))
      {
         HAVE_TOOL_CHANGE_POSITION = 1;
         retval = 0;
      }
      else if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf",
                           &TOOL_CHANGE_POSITION.tran.x,
                           &TOOL_CHANGE_POSITION.tran.y,
                           &TOOL_CHANGE_POSITION.tran.z, &TOOL_CHANGE_POSITION.a, &TOOL_CHANGE_POSITION.b, &TOOL_CHANGE_POSITION.c))
      {
         TOOL_CHANGE_POSITION.u = 0.0;
         TOOL_CHANGE_POSITION.v = 0.0;
         TOOL_CHANGE_POSITION.w = 0.0;
         HAVE_TOOL_CHANGE_POSITION = 1;
         retval = 0;
      }
      else if (3 == sscanf(inistring, "%lf %lf %lf", &TOOL_CHANGE_POSITION.tran.x, &TOOL_CHANGE_POSITION.tran.y, &TOOL_CHANGE_POSITION.tran.z))
      {
         /* read them OK */
         TOOL_CHANGE_POSITION.a = 0.0;
         TOOL_CHANGE_POSITION.b = 0.0;
         TOOL_CHANGE_POSITION.c = 0.0;
         TOOL_CHANGE_POSITION.u = 0.0;
         TOOL_CHANGE_POSITION.v = 0.0;
         TOOL_CHANGE_POSITION.w = 0.0;
         HAVE_TOOL_CHANGE_POSITION = 1;
         retval = 0;
      }
      else
      {
         /* bad format */
         emcOperatorError(0, EMC_I18N("bad format for TOOL_CHANGE_POSITION"));
         HAVE_TOOL_CHANGE_POSITION = 0;
         retval = -1;
      }
   }
   else
   {
      /* didn't find an entry */
      HAVE_TOOL_CHANGE_POSITION = 0;
   }

   if ((inistring = toolInifile->Find("TOOL_HOLDER_CLEAR", "EMCIO")) != NULL)
   {
      /* found an entry */
      if (3 == sscanf(inistring, "%lf %lf %lf", &TOOL_HOLDER_CLEAR.tran.x, &TOOL_HOLDER_CLEAR.tran.y, &TOOL_HOLDER_CLEAR.tran.z))
      {
         /* read them OK */
         TOOL_HOLDER_CLEAR.a = 0.0;     // not supporting ABC for now
         TOOL_HOLDER_CLEAR.b = 0.0;
         TOOL_HOLDER_CLEAR.c = 0.0;
         HAVE_TOOL_HOLDER_CLEAR = 1;
         retval = 0;
      }
      else
      {
         /* bad format */
         emcOperatorError(0, EMC_I18N("bad format for TOOL_HOLDER_CLEAR"));
         HAVE_TOOL_HOLDER_CLEAR = 0;
         retval = -1;
      }
   }
   else
   {
      /* didn't find an entry */
      HAVE_TOOL_HOLDER_CLEAR = 0;
   }

   return retval;
}  /* readToolChange() */

/* Loads ini file parameters for tool controller, from [EMCIO] section. */
int iniTool(const char *filename)
{
   int retval = EMC_R_OK;
   IniFile toolInifile;

   DBG("iniTool()\n");

   if (toolInifile.Open(filename) == false)
      return EMC_R_ERROR;

   // load tool values
   if (loadTool(&toolInifile) != EMC_R_OK)
      retval = EMC_R_ERROR;

   // read the tool change positions
   if (readToolChange(&toolInifile) != EMC_R_OK)
      retval = EMC_R_ERROR;

   // close the inifile
   toolInifile.Close();

   return retval;
} /* iniTool() */

int iniTask(const char *filename)
{
   struct rtstepper_app_session *ps = &session.dongle;
   IniFile inifile;
   const char *inistring;
   char version[LINELEN], machine[LINELEN];
   int saveInt;

   DBG("iniTask()\n");

   // open it
   if (inifile.Open(filename) == false)
   {
      BUG("invalid ini file=%s\n", filename);
      return EMC_R_ERROR;
   }

   if ((inistring = inifile.Find("VERSION", "EMC")) != NULL)
   {
      if (sscanf(inistring, "$Revision: %s", version) != 1)
      {
         strncpy(version, "unknown", LINELEN - 1);
      }
   }
   else
   {
      strncpy(version, "unknown", LINELEN - 1);

      if ((inistring = inifile.Find("MACHINE", "EMC")) != NULL)
      {
         strncpy(machine, inistring, LINELEN - 1);
      }
      else
      {
         strncpy(machine, "unknown", LINELEN - 1);
      }
      DBG("task: machine: '%s'  version '%s'\n", machine, version);
   }

   /* Get rtstepper dongle specific configuration parameters. */
   ps->snum[0] = 0;
   if ((inistring = inifile.Find("SERIAL_NUMBER", "TASK")) != NULL)
   {
      strncpy(ps->snum, inistring, sizeof(ps->snum));
      ps->snum[sizeof(ps->snum) - 1] = 0;       /* force zero termination */
   }
   ps->input0_abort_enabled = 0;
   if ((inistring = inifile.Find("INPUT0_ABORT", "TASK")) != NULL)
      ps->input0_abort_enabled = strtod(inistring, NULL);
   ps->input1_abort_enabled = 0;
   if ((inistring = inifile.Find("INPUT1_ABORT", "TASK")) != NULL)
      ps->input1_abort_enabled = strtod(inistring, NULL);
   ps->input2_abort_enabled = 0;
   if ((inistring = inifile.Find("INPUT2_ABORT", "TASK")) != NULL)
      ps->input2_abort_enabled = strtod(inistring, NULL);

   saveInt = EMC_TASK_INTERP_MAX_LEN;   //remember default or previously set value
   if ((inistring = inifile.Find("INTERP_MAX_LEN", "TASK")) != NULL)
   {
      if (sscanf(inistring, "%d", &EMC_TASK_INTERP_MAX_LEN) == 1)
      {
         if (EMC_TASK_INTERP_MAX_LEN <= 0)
         {
            EMC_TASK_INTERP_MAX_LEN = saveInt;
         }
      }
      else
      {
         EMC_TASK_INTERP_MAX_LEN = saveInt;
      }
   }

   if ((inistring = inifile.Find("RS274NGC_STARTUP_CODE", "EMC")) != NULL)
   {
      // copy to global
      strcpy(RS274NGC_STARTUP_CODE, inistring);
   }
   else
   {
      if ((inistring = inifile.Find("RS274NGC_STARTUP_CODE", "RS274NGC")) != NULL)
      {
         // copy to global
         strcpy(RS274NGC_STARTUP_CODE, inistring);
      }
      else
      {
         // not found, use default
      }
   }

   // close it
   inifile.Close();

   return EMC_R_OK;
}                               /* iniTask() */

static int getPair(char *buf, int buf_len, char *key, char *value, char **tail)
{
   int i=0, j;

   key[0] = 0;
   value[0] = 0;

   if (buf[i] == '#')
   {
      for (; buf[i] != '\n' && i < buf_len; i++);  /* eat comment line */
      if (buf[i] == '\n')
         i++;   /* bump past '\n' */
   }

   j = 0;
   while ((buf[i] != '=') && (i < buf_len) && (j < LINELEN))
      key[j++] = buf[i++];
   for (j--; key[j] == ' ' && j > 0; j--);  /* eat white space before = */
   key[++j] = 0;

   if (buf[i] == '=')
      for (i++; buf[i] == ' ' && i < buf_len; i++);  /* eat white space after = */

   j = 0;
   while ((buf[i] != '\n') && (i < buf_len) && (j < LINELEN))
      value[j++] = buf[i++];
   for (j--; value[j] == ' ' && j > 0; j--);  /* eat white space before \n */
   value[++j] = 0;

   if (buf[i] == '\n')
     i++;   /* bump past '\n' */

   if (tail != NULL)
      *tail = buf + i;  /* tail points to next line */

   return i;
} /* getPair() */

/* Get value for specified section and key from the ini file. */
enum EMC_RESULT iniGetKeyValue(const char *section, const char *key, char *value, int value_size)
{
   char new_key[LINELEN];
   char new_value[LINELEN];
   char rcbuf[255];
   char new_section[32];
   char *tail;
   FILE *inFile;
   enum EMC_RESULT stat = EMC_R_ERROR;
   unsigned int i,j;

   value[0] = 0;

   if((inFile = fopen(EMC_INIFILE, "r")) == NULL) 
   {
      BUG("unable to open %s\n", EMC_INIFILE);
      goto bugout;
   } 

   new_section[0] = 0;

   /* Read the config file */
   while ((fgets(rcbuf, sizeof(rcbuf), inFile) != NULL))
   {
      if (rcbuf[0] == '[')
      {
         i = j = 0;
         while ((rcbuf[i] != ']') && (j < (sizeof(new_section)-2)))
            new_section[j++] = rcbuf[i++];
         new_section[j++] = rcbuf[i++];   /* ']' */
         new_section[j] = 0;        /* zero terminate */
         continue;
      }

      getPair(rcbuf, strlen(rcbuf), new_key, new_value, &tail);

      if ((strcasecmp(new_section, section) == 0) && (strcasecmp(new_key, key) == 0))
      {
         strncpy(value, new_value, value_size);
         stat = EMC_R_OK;
         break;  /* done */
      }
   }

   if (stat != EMC_R_OK)
      BUG("unable to find %s %s in %s\n", section, key, EMC_INIFILE);
        
bugout:        
   if (inFile != NULL)
      fclose(inFile);
         
   return stat;
}   /* iniGetKeyValue() */


