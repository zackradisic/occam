#include "common.h"

#pragma once
ASSUME_NONNULL_BEGIN

#ifndef ARRAY_H_

typedef struct {
  u8 *maybe_null ptr;
  usize len;
  usize cap;
} array_t;

#define array_type(T)                                                          \
  struct {                                                                     \
    T *maybe_null ptr;                                                         \
    usize len;                                                                 \
    usize cap;                                                                 \
  }

void _array_init(array_t *arr, usize cap, usize elem_size) {
  usize actual_cap = cap < 4 ? 4 : cap;
  u8 *ptr = malloc(elem_size * actual_cap);
  arr->ptr = ptr;
  arr->cap = actual_cap;
  arr->len = 0;
}

void array_free(array_t *arr) {
  free(arr->ptr);
  arr->ptr = NULL;
  arr->cap = 0;
  arr->len = 0;
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

  a->ptr = realloc(a->ptr, newsize);
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
  arr->len += amount;
  if (arr->len > arr->cap) {
    array_grow(arr, elem_size, arr->len);
  }
}

void _array_shrink_to_fit(array_t *arr) {
  if (arr->cap == arr->len) {
    return;
  }
  arr->ptr = (u8 *)realloc(arr->ptr, arr->len);
  arr->cap = arr->len;
}

#define array_empty(T) ((T){.ptr = NULL, .len = 0, .cap = 0})
#define array_init(T, a, cap) _array_init((array_t *)a, cap, sizeof(T))
#define array_shrink_to_fit(a) _array_shrink_to_fit((array_t *)a)

#define array_push(T, a, val)                                                  \
  ({                                                                           \
    static_assert(__same_type(T, __typeof__(val)), "");                        \
    array_t *__a = (array_t *)(a);                                             \
    (__a->len >= __a->cap && UNLIKELY(!array_grow(__a, sizeof(T), 1)))         \
        ? false                                                                \
        : (((T *)__a->ptr)[__a->len++] = (val), true);                         \
  })

#define array_pop(T, a) (((T *)(a)->ptr)[--a->len])
#define array_reserve(T, a, n) _array_reserve((array_t *)(a), sizeof(T), n)

void *reallocate(void *pointer, usize old_size, usize new_size);

#define ARRAY_H_

ASSUME_NONNULL_END

#endif // ARRAY_H_