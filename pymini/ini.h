/********************************************************************
* ini.h - INI file support for rtstepperemc
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
#ifndef _INI_H
#define _INI_H

#include "emc.h"        // EMC_AXIS_STAT

#ifdef __cplusplus
extern "C"
{
#endif
   int iniGetKeyValue(const char *section, const char *key, char *value, int value_size);
#ifdef __cplusplus
}                               /* matches extern "C" at top */
#endif

#endif                          /* _INI_H */
