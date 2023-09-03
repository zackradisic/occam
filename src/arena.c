/**
 * This code is adapted from the memory arena implementation from "Physically
 * Based Rendering: From Theory To Implementation" third edition
 * */

#include "arena.h"
#include "common.h"
#include <stddef.h>

// https://www.pbr-book.org/3ed-2018/Utilities/Memory_Management#AllocAligned
void *alloc_aligned(usize size) {
#if defined(ZMEM_HAVE_POSIX_MEMALIGN)
  void *ptr;
  if (posix_memalign(&ptr, ZMEM_L1_CACHE_LINE_SIZE, size) != 0)
    ptr = NULL;
  return ptr;
#elif defined(ZMEM_HAVE__ALIGNED_MALLOC)
  return _aligned_malloc(size, ZMEM_L1_CACHE_LINE_SIZE);
#else
  return memalign(ZMEM_L1_CACHE_LINE_SIZE, size);
#endif
}

void free_aligned(void *ptr) {
  if (!ptr)
    return;
#if defined(ZMEM_HAVE__ALIGNED_MALLOC)
  _aligned_free(ptr);
#else
  free(ptr);
#endif
}

void arena_block_list_push_back(ArenaBlockList *list, ArenaBlock block);
void arena_block_list_splice(ArenaBlockList *dest, ArenaBlock *src, usize pos);
bool arena_block_list_erase(ArenaBlockList *list, ArenaBlockNode *node);

#define ARENA_ALIGN (alignof(max_align_t))
// default to 256kb
#define DEFAULT_ARENA_BLOCK_SIZE (262144)

Arena arena_new(usize *block_size) {
  usize actual_block_size =
      !block_size ? DEFAULT_ARENA_BLOCK_SIZE : *block_size;

  return (Arena){
      .block_size = actual_block_size,
      .current_block_pos = 0,
      .current_alloc_size = 0,
      .current_block = NULL,
      .used_blocks = (ArenaBlockList){NULL, NULL, NULL},
      .available_blocks = (ArenaBlockList){NULL, NULL, NULL},
  };
}

void *arena_alloc(Arena *arena, usize n_bytes) {
  static_assert(IS_POWER_OF_TWO(ARENA_ALIGN),
                "Minimum alignment not a power of two");

  const int align = ARENA_ALIGN;

  // align to the next multiple of ARENA_ALIGN
  n_bytes = (n_bytes + align - 1) & ~(align - 1);

  if (arena->current_block_pos + n_bytes > arena->current_alloc_size) {
    // Add current block to _usedBlocks_ list
    if (arena->current_block) {
      arena_block_list_push_back(&arena->used_blocks,
                                 (ArenaBlock){
                                     .alloc_size = arena->current_alloc_size,
                                     .data = arena->current_block,
                                 });
      arena->current_block = NULL;
      arena->current_alloc_size = 0;
    }

    // Get new block of memory for _MemoryArena_

    // Try to get memory block from _availableBlocks_
    for (ArenaBlockNode *cur = arena->available_blocks.first; cur != NULL;
         cur = cur->next) {
      if (cur->block.alloc_size >= n_bytes) {
        arena->current_alloc_size = cur->block.alloc_size;
        arena->current_block = cur->block.data;
        arena_block_list_erase(&arena->available_blocks, cur);
        break;
      }
    }

    if (!arena->current_block) {
      arena->current_alloc_size = MAX(n_bytes, arena->block_size);
      arena->current_block = alloc_aligned(arena->current_alloc_size);
    }
    arena->current_block_pos = 0;
  }

  void *ret = arena->current_block + arena->current_block_pos;
  arena->current_block_pos += n_bytes;

  return ret;
}

void arena_reset(Arena *arena) {
  arena->current_block_pos = 0;
  arena->available_blocks = arena->used_blocks;
  arena->used_blocks =
      (ArenaBlockList){.current = NULL, .first = NULL, .last = NULL};
}

void arena_free(Arena *arena) {
  free_aligned(arena->current_block);
  for (ArenaBlockNode *cur = arena->used_blocks.first; cur != NULL;
       cur = cur->next) {
    free_aligned(cur->block.data);
  }
  for (ArenaBlockNode *cur = arena->available_blocks.first; cur != NULL;
       cur = cur->next) {
    free_aligned(cur->block.data);
  }
}

void arena_block_list_push_back(ArenaBlockList *list, ArenaBlock block) {
  ArenaBlockNode *node = malloc(sizeof(ArenaBlockNode));
  *node = (ArenaBlockNode){
      .block = block,
      .prev = list->last,
      .next = NULL,
  };
  if (!list->last) {
    list->last = node;
  } else {
    list->last->next = node;
    list->last = node;
  }
}

bool arena_block_list_erase(ArenaBlockList *list, ArenaBlockNode *node) {
  for (ArenaBlockNode *cur = list->first; cur != NULL; cur = cur->next) {
    if (cur == node) {
      if (cur->prev) {
        cur->prev->next = node->next;
      } else {
        list->first = node->next;
      }

      if (cur->next) {
        cur->next->prev = node->prev;
      } else {
        list->last = node->prev;
      }

      free(node);

      return true;
    }
  }
  return false;
}
