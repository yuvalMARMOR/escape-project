CC := gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Wpedantic -Wconversion -Wshadow
LDLIBS ?= -lm
BUILD_DIR := build

ifeq ($(OS),Windows_NT)
TARGET := $(BUILD_DIR)/escape.exe
MKDIR_CMD := cmd /C if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"
CLEAN_CMD := cmd /C if exist "$(BUILD_DIR)" rmdir /S /Q "$(BUILD_DIR)"
else
TARGET := $(BUILD_DIR)/escape
MKDIR_CMD := mkdir -p "$(BUILD_DIR)"
CLEAN_CMD := rm -rf "$(BUILD_DIR)"
endif

.PHONY: all check scan demo clean

all: $(TARGET)

$(BUILD_DIR):
	$(MKDIR_CMD)

$(TARGET): src/group7.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

check:
	$(CC) $(CFLAGS) -fsyntax-only src/group7.c

scan: $(TARGET)
	./$(TARGET) --scan

demo: $(TARGET)
	./$(TARGET) --scan
	./$(TARGET) --sort
	./$(TARGET) --route 1,1 3
	./$(TARGET) --cost 5
	./$(TARGET) --fish 30

clean:
	$(CLEAN_CMD)
