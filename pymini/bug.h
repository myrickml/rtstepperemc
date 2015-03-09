#ifndef _BUG_H
#define _BUG_H
#include "emc.h"

//#define DEBUG_RTSTEPPEREMC

#define _STRINGIZE(x) #x
#define STRINGIZE(x) _STRINGIZE(x)

#define BUG(args...) emc_logger_cb(__FILE__ " " STRINGIZE(__LINE__) ": " args)
#define MSG(args...) emc_logger_cb(args)

#ifdef DEBUG_RTSTEPPEREMC
#define DBG(args...) emc_logger_cb(__FILE__ " " STRINGIZE(__LINE__) ": " args)
#else
#define DBG(args...)
#endif

#endif /* _BUG_H */
