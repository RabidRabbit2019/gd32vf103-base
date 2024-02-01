#ifndef	_TIME_H
#define _TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int64_t time_t;

struct tm {
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
	int tm_yday;
	int tm_isdst;
	long __tm_gmtoff;
	const char *__tm_zone;
};

struct tm *gmtime_r (const time_t *, struct tm *);
time_t timegm(struct tm *tm);

#ifdef __cplusplus
}
#endif


#endif
