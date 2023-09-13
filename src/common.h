#ifndef COMMON_H
#define COMMON_H

#include <_types/_uint16_t.h>
#include <_types/_uint32_t.h>
#include <_types/_uint8_t.h>

#if DEBUG
#define SAFECHECK_ENABLED 1
#endif

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef long isize;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef size_t usize;

int float_eq(float a, float b);
#define flte_zero(a) (a) <= FLT_EPSILON ? true : false

#define ZMEM_HAVE_POSIX_MEMALIGN
#define ZMEM_L1_CACHE_LINE_SIZE 64

#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define ZERO(a) memset(&a, 0, sizeof(a))

#define IS_POWER_OF_TWO(x) ((x) && !((x) & ((x)-1)))
/* static inline const int is_power_of_two(int x) { */
/*   /\* First x in the below expression is for the case when x is 0 *\/ */
/*   return x && (!(x & (x - 1))); */
/* } */

#define components(X, T) sizeof(X) / sizeof(T)

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

// clang-format off
#if defined(__clang__) && __has_feature(nullability)
  #ifndef maybe_null
    #define maybe_null _Nullable
  #endif
  #define ASSUME_NONNULL_BEGIN                                                \
    _Pragma("clang diagnostic push")                                              \
    _Pragma("clang diagnostic ignored \"-Wnullability-completeness\"")            \
    _Pragma("clang diagnostic ignored \"-Wnullability-inferred-on-nested-type\"") \
    _Pragma("clang assume_nonnull begin")
  #define ASSUME_NONNULL_END    \
    _Pragma("clang diagnostic pop") \
    _Pragma("clang assume_nonnull end")
#else
  #ifndef maybe_null
    #define maybe_null
  #endif
  #define ASSUME_NONNULL_BEGIN
  #define ASSUME_NONNULL_END
#endif
// clang-format on

#define __same_type(a, b)                                                      \
  __builtin_types_compatible_p(__typeof__(a), __typeof__(b))

// UNLIKELY(integralexpr)->bool
#if __has_builtin(__builtin_expect)
#define LIKELY(x) (__builtin_expect((bool)(x), true))
#define UNLIKELY(x) (__builtin_expect((bool)(x), false))
#else
#define LIKELY(x) (x)
#define UNLIKELY(x) (x)
#endif

// overflow checking

static inline bool __must_check_unlikely(bool unlikely) {
  return UNLIKELY(unlikely);
}

#define check_add_overflow(a, b, dst)                                          \
  __must_check_unlikely(({                                                     \
    __typeof__(a) a__ = (a);                                                   \
    __typeof__(b) b__ = (b);                                                   \
    __typeof__(dst) dst__ = (dst);                                             \
    (void)(&a__ == &b__);                                                      \
    (void)(&a__ == dst__);                                                     \
    __builtin_add_overflow(a__, b__, dst__);                                   \
  }))

#define check_sub_overflow(a, b, dst)                                          \
  __must_check_unlikely(({                                                     \
    __typeof__(a) a__ = (a);                                                   \
    __typeof__(b) b__ = (b);                                                   \
    __typeof__(dst) dst__ = (dst);                                             \
    (void)(&a__ == &b__);                                                      \
    (void)(&a__ == dst__);                                                     \
    __builtin_sub_overflow(a__, b__, dst__);                                   \
  }))

#define check_mul_overflow(a, b, dst)                                          \
  __must_check_unlikely(({                                                     \
    __typeof__(a) a__ = (a);                                                   \
    __typeof__(b) b__ = (b);                                                   \
    __typeof__(dst) dst__ = (dst);                                             \
    (void)(&a__ == &b__);                                                      \
    (void)(&a__ == dst__);                                                     \
    __builtin_mul_overflow(a__, b__, dst__);                                   \
  }))

// bool would_add_overflow(anyint a, anyint b)
#if __has_builtin(__builtin_add_overflow_p)
#define would_add_overflow(a, b)                                               \
  __builtin_add_overflow_p((a), (b), (__typeof__((a) + (b)))0)
#define would_mul_overflow(a, b)                                               \
  __builtin_mul_overflow_p((a), (b), (__typeof__((a) * (b)))0)
#elif __has_builtin(__builtin_add_overflow)
#define would_add_overflow(a, b)                                               \
  ({                                                                           \
    __typeof__((a) + (b)) tmp__;                                               \
    __builtin_add_overflow((a), (b), &tmp__);                                  \
  })
#define would_mul_overflow(a, b)                                               \
  ({                                                                           \
    __typeof__((a) * (b)) tmp__;                                               \
    __builtin_mul_overflow((a), (b), &tmp__);                                  \
  })
#else
// best effort (triggers ubsan if enabled)
#define would_add_overflow(a, b) ({ \
    __typeof__((a) + (b)) tmp__ = (i64)(a) + (i64)(b); \
    (u64)tmp__ < (u64)(a);
})
#define would_mul_overflow(a, b) ({ \
    __typeof__((a) * (b)) tmp__ = (i64)(a) * (i64)(b); \
    (u64)tmp__ < (u64)(a);
})
#endif

#define MIN_X(a, b) ((a) < (b) ? (a) : (b))

// ANYINT CEIL_POW2(ANYINT x) rounds up x to nearest power of two.
// Returns 1 when x is 0.
// Returns 0 when x is larger than the max pow2 for x's type (e.g. >0x80000000
// for u32)
#define CEIL_POW2(x)                                                           \
  ({                                                                           \
    __typeof__(x) xtmp__ = (x);                                                \
    CEIL_POW2_X(xtmp__);                                                       \
  })
// CEIL_POW2_X is a constant-expression implementation of CEIL_POW2
#define CEIL_POW2_X(x)                                                         \
  (((x) <= (__typeof__(x))1)                                                   \
       ? (__typeof__(x))1                                                      \
       : ((((__typeof__(x))1                                                   \
            << ILOG2(((x) - ((x) == (__typeof__(x))1)) - (__typeof__(x))1)) -  \
           (__typeof__(x))1)                                                   \
          << 1) +                                                              \
             (__typeof__(x))2)

// int co_clz(ANYUINT x) counts leading zeroes in x,
// starting at the most significant bit position.
// If x is 0, the result is undefined.
#define co_clz(x)                                                              \
  (_Generic((x), i8                                                            \
            : __builtin_clz, u8                                                \
            : __builtin_clz, i16                                               \
            : __builtin_clz, u16                                               \
            : __builtin_clz, i32                                               \
            : __builtin_clz, u32                                               \
            : __builtin_clz, long                                              \
            : __builtin_clzl, unsigned long                                    \
            : __builtin_clzl, long long                                        \
            : __builtin_clzll, unsigned long long                              \
            : __builtin_clzll)(x) -                                            \
   (32 - MIN_X(4, (int)sizeof(__typeof__(x))) * 8))

// int co_ctz(ANYUINT x) counts trailing zeroes in x,
// starting at the least significant bit position.
// If x is 0, the result is undefined.
#define co_ctz(x)                                                              \
  _Generic((x), i8                                                             \
           : __builtin_ctz, u8                                                 \
           : __builtin_ctz, i16                                                \
           : __builtin_ctz, u16                                                \
           : __builtin_ctz, i32                                                \
           : __builtin_ctz, u32                                                \
           : __builtin_ctz, long                                               \
           : __builtin_ctzl, unsigned long                                     \
           : __builtin_ctzl, long long                                         \
           : __builtin_ctzll, unsigned long long                               \
           : __builtin_ctzll)(x)

// int co_fls(ANYINT n) finds the Find Last Set bit (last = most-significant)
// (Note that this is not the same as co_ffs(x)-1).
// e.g. co_fls(0b1111111111111111) = 15
// e.g. co_fls(0b1000000000000000) = 15
// e.g. co_fls(0b1000000000000000) = 15
// e.g. co_fls(0b1000) = 3
#define co_fls(x) ((x) ? (int)(sizeof(__typeof__(x)) * 8) - co_clz(x) : 0)

// int ILOG2(ANYINT n) calculates the log of base 2, rounding down.
// e.g. ILOG2(15) = 3, ILOG2(16) = 4.
// Result is undefined if n is 0.
#define ILOG2(n) (co_fls(n) - 1)

#endif // COMMON_H
