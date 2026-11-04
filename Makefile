# Portable build for Linux / macOS (cc or gcc). On Windows use tools/build.sh,
# which drives MSVC through tools/msvc.bat.
CC ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -pedantic -Wshadow -Wconversion
FW := firmware/telltale
CORE := $(FW)/telltale_detect.c $(FW)/telltale_sha256.c $(FW)/telltale_auth.c \
        $(FW)/telltale_motion.c $(FW)/telltale_ina219.c $(FW)/telltale_persist.c
TESTS := test_detect test_crypto test_auth
UNAME := $(shell uname -s)
LIB := build/libtelltale_core.$(if $(filter Darwin,$(UNAME)),dylib,so)

.PHONY: all lib test eval clean

all: lib $(addprefix build/,$(TESTS)) build/bench

build:
	mkdir -p build

build/%: tests/c/%.c $(CORE) | build
	$(CC) $(CFLAGS) -o $@ $< $(CORE) -lm

lib: $(LIB)

$(LIB): $(CORE) | build
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $(CORE) -lm

test: all
	for t in $(TESTS); do ./build/$$t || exit 1; done
	python3 -m unittest discover -s tests -v

eval: all
	python3 -m sim.evaluate

clean:
	rm -rf build
