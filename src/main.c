#include "common.h"
#include "stdio.h"
#include <arena.h>

int main() {
  Arena arena = arena_new(NULL);

  usize *some_val = arena_alloc(&arena, sizeof(usize));
  *some_val = 420;
  safecheckf(*some_val == 420, "420 != %d\n", *some_val);

  return 0;
}