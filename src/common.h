#ifndef COMMON_H
#define COMMON_H

#include <_types/_uint16_t.h>
#include <_types/_uint32_t.h>
#include <_types/_uint8_t.h>
#define DEBUG 1

#if DEBUG
#define SAFECHECK_ENABLED 1
#endif

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef size_t usize;

int float_eq(float a, float b);

#define ZMEM_HAVE_POSIX_MEMALIGN
#define ZMEM_L1_CACHE_LINE_SIZE 64

#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define IS_POWER_OF_TWO(x) ((x) && !((x) & ((x)-1)))
/* static inline const int is_power_of_two(int x) { */
/*   /\* First x in the below expression is for the case when x is 0 *\/ */
/*   return x && (!(x & (x - 1))); */
/* } */

// From:
// https://github.com/rsms/compis/blob/3946ed5c1da01addf5b07faa872cc6463df2521d/src/colib.h#L182
#ifndef countof
#define countof(x)                                                             \
  ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))
#endif

// From:
// https://github.com/rsms/compis/blob/3946ed5c1da01addf5b07faa872cc6463df2521d/src/colib.h#L473
#if SAFECHECK_ENABLED
#define safefail(fmt, args...)                                                 \
  _panic(__FILE__, __LINE__, __FUNCTION__, fmt, ##args)
#define safecheckf(cond, fmt, args...)                                         \
  ((cond) ? ((void)0) : safefail(fmt, ##args))
#define safecheck(cond) ((cond) ? ((void)0) : safefail("safecheck (%s)", #cond))
void _panic(const char *file, int line, const char *fun, const char *fmt, ...);
#else
#define safefail(fmt, args...) ((void)0)
#define safecheckf(...) ((void)0)
#define safecheck(cond) ((void)0)
#endif

#define TODO(fmt, args...) safefail(fmt, ##args)

#endif // COMMON_H
