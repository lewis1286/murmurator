# Murmur Boids

A [Daisy Patch](https://www.electro-smith.com/daisy/patch) module where a flock of 3D boids controls oscillator voices. Each boid is an always-on oscillator — its 3D position maps directly to audio parameters: **x = pan, y = frequency, z = amplitude**.

## Features

- **3D boids flocking simulation** — separation, alignment, cohesion, and per-boid wander for continuous swooping flight
- **Up to 16 oscillator voices** (4-16, set via encoder)
- **Waveform morphing** — continuous blend from sine → triangle → square via CTRL_4
- **Scale quantization** — snap boid frequencies to a musical scale (root, mode, octave, chord progression)
- **Reverb bus** — z-axis distance model adds spatial depth to far boids
- **OLED visualization** — flock view, parameter readout, scale settings
- **LED grid** — 4×4 density visualization

## Controls

| Knob | Parameter | Range | Description |
|------|-----------|-------|-------------|
| CTRL_1 | Density | 0-1 | CCW = spread (max separation), CW = cluster (max cohesion) |
| CTRL_2 | Alignment | 0-2 | Velocity alignment — how synchronized the flock moves |
| CTRL_3 | Speed | 0.05-1.5 | Boid max speed → rate of audio parameter change |
| CTRL_4 | Wave Morph | sine→tri→square | CCW = pure sine, center = triangle, CW = square |

| Gate | Function |
|------|----------|
| GATE_1 | (reserved) |
| GATE_2 | Scatter flock (randomize all boid positions) |

| Encoder | Function |
|---------|----------|
| Rotate (normal pages) | Change number of boids/voices (4-16) |
| Rotate (Scale Settings) | Edit selected setting |
| Press (normal pages) | Cycle display pages |
| Press (Scale Settings) | Advance cursor through settings / exit |

## Display Pages

1. **Flock View** `[1/3]` — Boid triangles; size varies with z (amplitude); low freq at bottom
2. **Parameters** `[2/3]` — Density, Alignment, Speed, Wave morph, boid count, axis mapping
3. **Scale Settings** `[3/3]` — Root note, scale type, base octave, chord progression

## Boid → Audio Mapping

| Boid Axis | Audio Parameter | Notes |
|-----------|----------------|-------|
| x (0-1) | Stereo pan | Linear, L to R |
| y (0-1) | Frequency | 200 Hz base + spread; quantized to scale when active |
| z (0-1) | Amplitude | z=0 loud/close, z=1 quiet/far; never fully silent |

Waveform shape is global (CTRL_4), shared across all voices. The z-axis also opens/closes a LPF per voice — far boids are darker, close boids are brighter.

## Scale Settings

Accessed via display page 3. Encoder rotates to edit, press to advance cursor:

| Row | Setting | Options |
|-----|---------|---------|
| Root | Root note | C through B |
| Scale | Scale type | Linear (off), Major, Nat. Minor, Dorian, Pent. Major, Pent. Minor, Lydian, Mixolydian |
| Octave | Base octave | 1-5 |
| ChProg | Chord progression | OFF, 10s, 15s interval cycling I→IV→V→I |

When scale is set to Linear, frequencies map continuously across the Hz range. All other scales snap boid y-positions to the nearest scale degree.

## Building

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Make
- Debugger probe (ST-Link or compatible) for flashing

### Clone with submodules

```bash
git clone --recursive https://github.com/lewis1286/murmurator.git
cd murmurator
```

Or if already cloned:
```bash
git submodule update --init --recursive
```

### Build libraries (first time only)

```bash
cd libDaisy && make
cd ../DaisySP && make
cd ..
```

### Build and flash

```bash
cd murmur

# Full build (audio + UI)
make clean && make

# UI-only build (no audio — for testing display/knobs/boids without sound)
make clean && make ui-only

# Flash using debugger probe
make program
```

> Always run `make clean` when switching between full and UI-only builds.

### Build variants

| Command | What runs |
|---------|-----------|
| `make` | Full audio + boids + UI |
| `make ui-only` | Boids + UI only (no audio callback) |
| `make debug` | Full build with `-Og` for debugger |
| `make debug-ui-only` | UI-only with `-Og` |

## Project Structure

```
murmurator/
├── libDaisy/                      # Hardware abstraction (submodule)
├── DaisySP/                       # DSP library (submodule)
└── murmur/
    ├── MurmurBoids.cpp            # Main application
    ├── Makefile
    ├── audio/
    │   ├── osc_voice.h            # Oscillator voice (phase accumulator, waveform morph, LPF)
    │   ├── simple_reverb.h        # Reverb bus for z-axis distance model
    │   └── scale_quantizer.h      # Scale/chord quantization for y-axis frequency
    ├── boids/
    │   ├── vec3.h                 # 3D vector math + FastInvSqrt
    │   ├── boids.h/.cpp           # 3D flock simulation (separation, alignment, cohesion, wander)
    │   └── vec2.h                 # (legacy, kept for reference)
    └── ui/
        ├── display.h/.cpp         # OLED rendering (3 pages)
        └── led_grid.h/.cpp        # 4×4 LED density visualization
```

## Memory

| Region | Used | Total | % |
|--------|------|-------|---|
| FLASH | ~107 KB | 128 KB | ~83.7% |
| SRAM | ~72 KB | 512 KB | ~14.1% |

---

## Creating a New Daisy Patch Project

A guide for bootstrapping a fresh project with the same structure as this one.

### Prerequisites

Install these tools before starting:

| Tool | macOS install | Purpose |
|------|--------------|---------|
| ARM GCC | `brew install --cask gcc-arm-embedded` | Cross-compiler for STM32 |
| Make | included with Xcode CLI tools | Build system |
| OpenOCD | `brew install openocd` | On-chip debugging / flashing |
| ST-Link probe | hardware | Flash and debug over SWD |
| VSCode | [code.visualstudio.com](https://code.visualstudio.com) | IDE (optional but recommended) |
| Cortex-Debug extension | install inside VSCode | GDB integration |

Verify the toolchain:
```bash
arm-none-eabi-gcc --version
```

### 1. Create the repo and add submodules

```bash
mkdir MyDaisyProject && cd MyDaisyProject
git init

# Add the Daisy libraries as submodules
git submodule add https://github.com/electro-smith/libDaisy
git submodule add https://github.com/electro-smith/DaisySP
git submodule update --init --recursive
```

If cloning an existing repo that already has submodules:
```bash
git clone --recursive <repo-url>
# or, if already cloned:
git submodule update --init --recursive
```

### 2. Build the libraries (first time only)

```bash
cd libDaisy && make && cd ..
cd DaisySP  && make && cd ..
```

This produces the static libraries that your project Makefile links against. You only need to redo this if you update the submodules.

### 3. Create your project folder and Makefile

```bash
mkdir MyProject && cd MyProject
```

Minimal `Makefile`:
```makefile
# Target binary name
TARGET = MyProject

# Your source files
CPP_SOURCES = MyProject.cpp

# Library locations (relative to this Makefile)
LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR  = ../DaisySP

# Optimization
OPT = -O2 -g

# Include the Daisy build system
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
```

The included `Makefile` from libDaisy handles all the ARM cross-compilation flags, linker script, and flash targets automatically.

### 4. Minimal main file

```cpp
#include "daisy_patch.h"
using namespace daisy;

DaisyPatch patch;

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    for(size_t i = 0; i < size; i++)
    {
        out[0][i] = in[0][i]; // passthrough
        out[1][i] = in[1][i];
    }
}

int main()
{
    patch.Init();
    patch.StartAudio(AudioCallback);

    while(true)
    {
        patch.ProcessAnalogControls();
        patch.ProcessDigitalControls();
    }
}
```

### 5. VSCode configuration

Create `.vscode/` inside your project folder with these files:

**`tasks.json`** — build and flash tasks:
```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "build",
      "type": "shell",
      "command": "make clean; make",
      "group": { "kind": "build", "isDefault": true },
      "options": { "cwd": "${workspaceFolder}" },
      "problemMatcher": ["$gcc"]
    },
    {
      "label": "build_and_program",
      "type": "shell",
      "command": "make clean; make; make program",
      "options": { "cwd": "${workspaceFolder}" },
      "problemMatcher": ["$gcc"]
    },
    {
      "label": "build_libdaisy",
      "type": "shell",
      "command": "make",
      "options": { "cwd": "${workspaceFolder}/../libDaisy" },
      "problemMatcher": ["$gcc"]
    },
    {
      "label": "build_daisysp",
      "type": "shell",
      "command": "make",
      "options": { "cwd": "${workspaceFolder}/../DaisySP" },
      "problemMatcher": ["$gcc"]
    }
  ]
}
```

**`c_cpp_properties.json`** — IntelliSense paths:
```json
{
  "version": 4,
  "configurations": [
    {
      "name": "macOS",
      "compilerPath": "/usr/local/bin/arm-none-eabi-g++",
      "cStandard": "c11",
      "cppStandard": "c++17",
      "intelliSenseMode": "gcc-arm",
      "includePath": [
        "${workspaceFolder}/**",
        "${workspaceFolder}/../libDaisy/**",
        "${workspaceFolder}/../DaisySP/**"
      ]
    }
  ]
}
```

**`launch.json`** — Cortex-Debug (optional, for step debugging via ST-Link):
```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Cortex Debug",
      "type": "cortex-debug",
      "request": "launch",
      "servertype": "openocd",
      "interface": "swd",
      "configFiles": ["interface/stlink.cfg", "target/stm32h7x.cfg"],
      "executable": "${workspaceRoot}/build/MyProject.elf",
      "svdFile": "${workspaceRoot}/.vscode/STM32H750x.svd",
      "preLaunchTask": "build",
      "runToMain": true
    }
  ]
}
```

The `.svd` file (STM32H750x.svd) enables peripheral register inspection in the debug panel. Copy it from `murmur/.vscode/STM32H750x.svd`.

### 6. .gitignore

```gitignore
# Build artifacts
build/
*.bin
*.elf
*.hex
*.map
*.o
*.d
*.lst

# IDE
.vscode/settings.json
*.swp

# macOS
.DS_Store
```

### 7. Build and flash

```bash
# From your project folder:
make clean && make       # compile
make program             # flash via ST-Link (debugger probe must be connected)
```

> `make program-dfu` (USB DFU bootloader) is an alternative but requires the Daisy bootloader to already be flashed. Use the ST-Link probe path above for reliable first-time flashing.

### Final directory structure

```
MyDaisyProject/
├── libDaisy/           # git submodule
├── DaisySP/            # git submodule
├── MyProject/
│   ├── MyProject.cpp   # main application
│   ├── Makefile
│   └── .vscode/
│       ├── tasks.json
│       ├── c_cpp_properties.json
│       ├── launch.json
│       └── STM32H750x.svd
├── .gitmodules
└── .gitignore
```

---

## Acknowledgments

- [Electro-Smith](https://www.electro-smith.com/) for the Daisy platform
- Craig Reynolds for the original [boids algorithm](https://www.red3d.com/cw/boids/)
