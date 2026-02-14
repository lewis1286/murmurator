# Project Name
TARGET = murmur

# Sources
CPP_SOURCES = MurmurBoids.cpp \
              boids/boids.cpp \
              ui/display.cpp \
              ui/led_grid.cpp

# Library Locations
LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR = ../DaisySP

# Optimized build with debug symbols
OPT = -O2 -g

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Build UI-only (no audio). Usage: make ui-only
ui-only: C_DEFS += -DMURMUR_UI_ONLY
ui-only: all

# Debug build (-Og for stepping through code). Usage: make debug
debug: OPT = -Og -g
debug: all

# Debug + UI-only. Usage: make debug-ui-only
debug-ui-only: OPT = -Og -g
debug-ui-only: C_DEFS += -DMURMUR_UI_ONLY
debug-ui-only: all
