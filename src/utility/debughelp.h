#ifndef DEBUGHELP_H
#define	DEBUGHELP_H

#include <stdarg.h>
#include "Arduino.h"

#ifdef _DEBUG
#define DEBUG_BAUD_RATE 115200
#define DEBUG_BEGIN() {Serial.begin(DEBUG_BAUD_RATE);}
#define DEBUG_PRINT(T) {Serial.println(T);}
#define DEBUG_PRINTHEX(T, v) {Serial.print(T); Serial.println(v, HEX);}
#define DEBUG_PRINTDEC(T, v) {Serial.print(T); Serial.println(v, DEC);}
#define DEBUG_HALT() {while(Serial.available() == 0); Serial.setTimeout(1); Serial.readBytes(sDebug, 1);}

#else

#define DEBUG_BEGIN(...)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTHEX(...)
#define DEBUG_PRINTDEC(...)
#define DEBUG_HALT(...) 

#endif  /* DEBUG */

#endif	/* DEBUGHELP_H */
