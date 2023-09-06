CC=clang
DEBUG=true
OPTIMIZATION=-g3 -O0 -flto
DISABLED_WARNINGS=-Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function -Wno-unused-command-line-argument -Wno-unused-parameter -Wno-missing-braces
C_FLAGS=-Wall -Wextra -Werror $(DISABLED_WARNINGS) -std=c11 $(OPTIMIZATION) -D_THREAD_SAFE
LD_FLAGS=-I./src -I./lib/
OUT_DIR=out
MACOS_FRAMEWORK_PATH=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks
MACOS_FRAMEWORKS=-framework CoreGraphics -framework Metal -framework MetalKit -framework Cocoa -framework Foundation -framework QuartzCore
# LD_FLAGS=-L/opt/homebrew/lib/ -lSDL2 -I/opt/homebrew/include/ -I./src

all: main

$(OUT_DIR)/main: src/main.c $(OUT_DIR)/common.o $(OUT_DIR)/arena.o $(OUT_DIR)/sokol.o 
	# $(CC) $(C_FLAGS) $(LD_FLAGS) -ObjC -F$(MACOS_FRAMEWORK_PATH) -o $@ $< $(MACOS_FRAMEWORKS)
	$(CC) $(C_FLAGS) $(LD_FLAGS) -ObjC -F$(MACOS_FRAMEWORK_PATH) -o $@ $< $(MACOS_FRAMEWORKS)

main: $(OUT_DIR)/main

$(OUT_DIR)/common.o: src/common.c src/common.h
	$(CC) $(C_FLAGS) $(LD_FLAGS) -o $@ -c $<

$(OUT_DIR)/arena.o: src/arena.c src/arena.h
	$(CC) $(C_FLAGS) $(LD_FLAGS) -o $@ -c $<

$(OUT_DIR)/sokol.o: src/sokol.h 
	$(CC) $(LD_FLAGS) -ObjC -o $@ -c $<

.PHONY: clean
clean:
	rm -rf out/*
