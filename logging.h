#ifdef LOGGING
#include<stdarg.h>
void log(const char* format, ...) {
  char line[512];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  Serial.println(line);
}
#else
// logging is disabled
#define log(...)
#endif

