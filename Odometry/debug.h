#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#if defined(DEBUG_PRINT)
  #define DebugBegin() Serial.begin(115200)
  #define DebugPrintln(a) Serial.println(a)
  #define DebugPrint(a) Serial.print(a)
#else
  #define DebugBegin() 
  #define DebugPrintln(a) 
  #define DebugPrint(a)
#endif  

#endif
