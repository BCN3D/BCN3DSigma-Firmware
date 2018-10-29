#ifndef BUILD_DEFS_H

#define BUILD_DEFS_H

#define BUILDTM_YEAR (\
__DATE__[7] == '?' ? 1900 \
: (((__DATE__[7] - '0') * 1000) \
+ (__DATE__[8] - '0') * 100 \
+ (__DATE__[9] - '0') * 10 \
+ __DATE__[10] - '0'))

#define BUILDTM_MONTH (\
__DATE__ [2] == '?' ? 1 \
: __DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
: __DATE__ [2] == 'b' ? 2 \
: __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
: __DATE__ [2] == 'y' ? 5 \
: __DATE__ [2] == 'l' ? 7 \
: __DATE__ [2] == 'g' ? 8 \
: __DATE__ [2] == 'p' ? 9 \
: __DATE__ [2] == 't' ? 10 \
: __DATE__ [2] == 'v' ? 11 \
: 12)

#define BUILDTM_DAY (\
__DATE__[4] == '?' ? 1 \
: ((__DATE__[4] == ' ' ? 0 : \
((__DATE__[4] - '0') * 10)) + __DATE__[5] - '0'))




#endif // BUILD_DEFS_H

