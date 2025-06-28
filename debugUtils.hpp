#pragma once

#ifdef DEBUG
#include <Arduino.h>
#define DBUG_PRINT(str)                                                       \
  Serial.print(millis());                                                      \
  Serial.print(": ");                                                          \
  Serial.print(__PRETTY_FUNCTION__);                                           \
  Serial.print(' ');                                                           \
  Serial.print(__FILE__);                                                      \
  Serial.print(':');                                                           \
  Serial.print(__LINE__);                                                      \
  Serial.print(' ');                                                           \
  Serial.println(str);

#define DBUG_PRINT_VAR(var)                                                   \
  Serial.print(millis());                                                      \
  Serial.print(": ");                                                          \
  Serial.print(__PRETTY_FUNCTION__);                                           \
  Serial.print(' ');                                                           \
  Serial.print(__FILE__);                                                      \
  Serial.print(':');                                                           \
  Serial.print(__LINE__);                                                      \
  Serial.print(' ');                                                           \
  Serial.print(#var);                                                          \
  
  Serial.print(" = ");                                                         \
  Serial.println(var);
#else
#define DBUG_PRINT_VAR(var)
#define DBUG_PRINT(str)
#endif
