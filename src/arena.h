#ifndef ARENA_H_
#define ARENA_H_

#include "common.h"
#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct Arena Arena;

typedef struct ArenaBlock {
  usize alloc_size;
  u8 *data;
} ArenaBlock;

typedef struct ArenaBlockNode {
  ArenaBlock block;
  struct ArenaBlockNode *prev;
  struct ArenaBlockNode *next;
} ArenaBlockNode;

typedef struct ArenaBlockList {
  ArenaBlockNode *first;
  ArenaBlockNode *current;
  ArenaBlockNode *last;
} ArenaBlockList;

struct Arena {
  usize block_size;
  usize current_block_pos;
  size_t current_alloc_size;
  u8 *current_block;

  ArenaBlockList used_blocks;
  ArenaBlockList available_blocks;
};

Arena arena_new(usize *block_size);
void *arena_alloc(Arena *arena, usize n_bytes);
void arena_reset(Arena *arena);
void arena_free(Arena *arena);

#endif // ARENA_H_
