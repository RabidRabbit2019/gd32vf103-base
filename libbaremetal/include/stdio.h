#include <stddef.h>
#include <stdarg.h>


#define EOF   -1

#ifdef __cplusplus
extern "C" {
#endif
  
int snprintf(char *pString, size_t length, const char *pFormat, ...);

#ifdef __cplusplus
} // extern "C"
#endif
