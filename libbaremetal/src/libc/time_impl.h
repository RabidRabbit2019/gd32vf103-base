#include <time.h>

// int __days_in_month(int, int);
int __month_to_secs(int, int);
time_t __year_to_secs(time_t, int *);
time_t __tm_to_secs(const struct tm *);
// const char *__tm_to_tzname(const struct tm *);
int __secs_to_tm(time_t, struct tm *);
// void __secs_to_zone(time_t, int, int *, long *, long *, const char **);
// const char *__strftime_fmt_1(char (*)[100], size_t *, int, const struct tm *, locale_t, int);
// extern const char __utc[];
