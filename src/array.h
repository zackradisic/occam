#include "common.h"

#pragma once
ASSUME_NONNULL_BEGIN

#ifndef ARRAY_H_

typedef struct {
  u8 *maybe_null ptr;
  usize len;
} slice_t;

typedef union {
  struct {
    slice_t slice;
  };
  struct {
    u8 *maybe_null ptr;
    usize len;
    usize cap;
  };
} array_t;

#define array_type(T)                                                          \
  struct {                                                                     \
    T *maybe_null ptr;                                                         \
    usize len;                                                                 \
    usize cap;                                                                 \
  }

#define array_type_with_slice(T, Slice)                                        \
  union {                                                                      \
    struct {                                                                   \
      Slice slice;                                                             \
    };                                                                         \
                                                                               \
    struct {                                                                   \
      T *maybe_null ptr;                                                       \
      usize len;                                                               \
      usize cap;                                                               \
    };                                                                         \
  }

#define slice_type(T)                                                          \
  struct {                                                                     \
    T *maybe_null ptr;                                                         \
    usize len;                                                                 \
  }

#define slice_empty(T) ((T){.ptr = NULL, .len = 0})

#define array_index_checked(T, s, i)                                           \
  (safecheck((usize)i < (s)->len), (cast_ptr(T, (s)->ptr)[i])

#define array_ref_checked(T, s, i)                                             \
  (safecheck((usize)i < (s)->len), &cast_ptr(T, (s)->ptr)[i])

#if DEBUG
#ifndef DISABLE_SAFE_INDEX
#define SAFE_INDEX
#endif
#endif

#ifdef SAFE_INDEX
#define array_index(T, s, i) array_index_checked(T, s, i)
#define array_ref(T, s, i) array_ref_checked(T, s, i)

#else
#define array_index(T, s, i) cast_ptr(T, (s)->ptr)[i]
#define array_ref(T, s, i) &cast_ptr(T, (s)->ptr)[i]

#endif

slice_t _array_as_slice(array_t *arr) {
  return (slice_t){
      .ptr = arr->ptr,
      .len = arr->len,
  };
}

void _array_init(array_t *arr, usize cap, usize elem_size) {
  usize actual_cap = cap < 4 ? 4 : cap;
  u8 *ptr = (u8 *)malloc(elem_size * actual_cap);
  arr->ptr = ptr;
  arr->cap = actual_cap;
  arr->len = 0;
}

void _array_free(array_t *arr) {
  free(arr->ptr);
  arr->ptr = NULL;
  arr->cap = 0;
  arr->len = 0;
}

static inline bool _array_check_overlap(const array_t *a, const array_t *b,
                                        usize elem_size) {
  if (a->ptr == NULL)
    return b->ptr != NULL;
  if (a->ptr == b->ptr)
    return true;

  // make sure a->ptr < b->ptr is always true
  if ((usize)a->ptr > (usize)b->ptr) {
    const array_t *temp = a;
    a = b;
    b = temp;
  }

  return b->ptr < a->ptr + a->len * elem_size;
}

#define GROW_CAPACITY(capacity) ((capacity) < 8 ? 8 : (capacity)*2)
#define MIN_X(a, b) ((a) < (b) ? (a) : (b))

static bool array_resize(array_t *a, usize elemsize, usize newcap) {
  assert(newcap >= a->len);

  if (a->cap == newcap)
    return true;

  usize newsize;
  if (check_mul_overflow((usize)newcap, (usize)elemsize, &newsize))
    return false;

  a->ptr =
      // is realloc on NULL fucked?
      a->ptr == NULL ? ((u8 *)malloc(newsize * elemsize))
                     : (u8 *)realloc(a->ptr, newsize);

  a->cap = newcap;
  return true;
}

bool array_grow(array_t *a, usize elemsize, usize extracap) {
  usize newcap;
  if (a->cap == 0) {
    // initial allocation
    const usize ideal_nbyte = 64;
    newcap = MAX(extracap, ideal_nbyte / elemsize);
  } else {
    // grow allocation
    usize currsize = (usize)a->cap * (usize)elemsize;
    usize extrasize;
    if (check_mul_overflow((usize)extracap, (usize)elemsize, &extrasize))
      return false;
    if (currsize < 65536 && extrasize < 65536 / 2) {
      // double capacity until we hit 64KiB
      newcap = (a->cap >= extracap) ? a->cap * 2 : a->cap + extracap;
    } else {
      usize addlcap = MAX(65536u / elemsize, CEIL_POW2(extracap));
      if (check_add_overflow(a->cap, addlcap, &newcap)) {
        // try adding exactly what is needed (extracap)
        if (check_add_overflow(a->cap, extracap, &newcap))
          return false;
      }
    }
  }

  assert(newcap - a->cap >= extracap);
  return array_resize(a, elemsize, newcap);
}

void _array_reserve(array_t *arr, usize elem_size, usize amount) {
  usize avail = arr->cap - arr->len;
  if (amount <= avail)
    return;

  usize amount_to_grow = amount - avail;
  array_grow(arr, elem_size, amount_to_grow);
}

void _array_concat(array_t *dest, const array_t *src, usize elem_size) {
  _array_reserve(dest, elem_size, src->len);
  if (UNLIKELY(_array_check_overlap(dest, src, elem_size))) {
    memmove(&dest->ptr[dest->len * elem_size], src->ptr, src->len * elem_size);
  } else {
    memcpy(&dest->ptr[dest->len * elem_size], src->ptr, src->len * elem_size);
  }
  dest->len += src->len;
}

void _array_shrink_to_fit(array_t *arr) {
  if (arr->cap == arr->len) {
    return;
  }
  arr->ptr = (u8 *)realloc(arr->ptr, arr->len);
  arr->cap = arr->len;
}

void _array_copy(array_t *dest, const array_t *src, usize elem_size) {
  u8 *ptr = (u8 *)malloc(src->len * elem_size);
  dest->ptr = ptr;
  dest->len = src->len;
  dest->cap = src->len;
}

void _array_erase(array_t *arr, usize idx, usize elem_size) {
  safecheck(arr->len > 0);
  // shift all elements after to the left by one
  if (idx < arr->len - 1) {
    usize shift_amount = (arr->len - 1) - idx;
    memmove(&arr->ptr[idx * elem_size], &arr->ptr[(idx + 1) * elem_size],
            shift_amount);
  }
  arr->len -= 1;
}

#define array_empty(T) ((T){.ptr = NULL, .len = 0, .cap = 0})
#define array_init(T, a, cap) _array_init((array_t *)(a), (cap), sizeof(T))
#define array_shrink_to_fit(a) _array_shrink_to_fit((array_t *)(a))
#define array_free(a) _array_free((array_t *)a)
#define array_erase(T, a, i) _array_erase((array_t *)(a), i, sizeof(T))

#define array_push(T, a, val)                                                  \
  ({                                                                           \
    static_assert(__same_type(T, __typeof__(val)), "");                        \
    array_t *__a = (array_t *)(a);                                             \
    (__a->len >= __a->cap && UNLIKELY(!array_grow(__a, sizeof(T), 1)))         \
        ? false                                                                \
        : (((T *)__a->ptr)[__a->len++] = (val), true);                         \
  })

#define array_clear(a) (a)->len = 0;
#define array_copy(T, dest, src) _array_copy(dest, src, sizeof(T))
#define array_pop(T, a) (((T *)(a)->ptr)[--a->len])
#define array_reserve(T, a, n) _array_reserve((array_t *)(a), sizeof(T), n)
#define array_concat(T, dest, src)                                             \
  ({                                                                           \
    static_assert(__same_type(__typeof__(dest), __typeof__(src)), "");         \
    _array_concat((array_t *)(dest), (const array_t *)(src), sizeof(T));       \
  })

#define ARRAY_H_

ASSUME_NONNULL_END

#endif // ARRAY_H_