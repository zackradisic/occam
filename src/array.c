#include "array.h"

void *reallocate(void *pointer, usize old_size, usize new_size) {
  if (new_size == 0) {
    free(pointer);
    return NULL;
  }

  void *result = realloc(pointer, new_size);
  if (result == NULL)
    exit(1);

  return result;
}