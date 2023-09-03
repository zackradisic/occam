CC=clang
DEBUG=true
OPTIMIZATION=-g3 -O0
DISABLED_WARNINGS=-Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function -Wno-unused-command-line-argument -Wno-unused-parameter
C_FLAGS=-Wall -Wextra -Werror $(DISABLED_WARNINGS) -std=c11 $(OPTIMIZATION) -D_THREAD_SAFE
LD_FLAGS=-I./src
OUT_DIR=out
# LD_FLAGS=-L/opt/homebrew/lib/ -lSDL2 -I/opt/homebrew/include/ -I./src

all: main

$(OUT_DIR)/main: src/main.c $(OUT_DIR)/common.o $(OUT_DIR)/arena.o
	$(CC) $(C_FLAGS) $(LD_FLAGS) -o $@ $< src/common.o src/arena.o

main: $(OUT_DIR)/main

$(OUT_DIR)/common.o: src/common.c src/common.h
	$(CC) $(C_FLAGS) $(LD_FLAGS) -o $@ -c $<

$(OUT_DIR)/arena.o: src/arena.c src/arena.h
	$(CC) $(C_FLAGS) $(LD_FLAGS) -o $@ -c $<

.PHONY: clean
clean:
	rm -rf out/*
