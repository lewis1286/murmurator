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

# Debug optimization (override libDaisy's -O2 so debugger can inspect variables)
# Delete or comment this out for 'production' use
OPT = -Og -g

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Build UI-only (no audio). Usage: make ui-only
ui-only: C_DEFS += -DMURMUR_UI_ONLY
ui-only: all
