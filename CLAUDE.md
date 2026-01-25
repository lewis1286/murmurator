# Murmur Boids - Granular Synthesis Controlled by Flocking Algorithm

A Daisy Patch module where a flock of boids controls granular synthesis parameters. Each boid represents a grain voice - its 2D position and velocity map to playback position, pitch, grain size, and stereo pan.

## Current Status: COMPLETE ✓

All 5 phases implemented and building successfully:
- ✓ Phase 1: Audio Infrastructure (circular buffer, delay)
- ✓ Phase 2: Single Grain Engine (Hann envelope, pitch shifting)
- ✓ Phase 3: Boids Simulation (flocking with spatial partitioning, OLED/LED viz)
- ✓ Phase 4: Multi-Grain Polyphony (16 voices with voice stealing)
- ✓ Phase 5: Integration (boids control grains via scheduler)

## Quick Start

```bash
# Build
cd murmur
make clean && make

# Flash (put Daisy in DFU mode: hold BOOT, press RESET)
make program-dfu
```

## Project Structure

```
murmurator/
├── libDaisy/                      # Git submodule
├── DaisySP/                       # Git submodule
├── murmur/                        # Main project
│   ├── MurmurBoids.cpp           # Main application
│   ├── Makefile                  # Build configuration
│   ├── audio/
│   │   ├── circular_buffer.h/.cpp # 4-second ring buffer in SDRAM
│   │   ├── grain_voice.h/.cpp     # Single grain with Hann envelope
│   │   └── grain_pool.h/.cpp      # 16-voice polyphony with stealing
│   ├── boids/
│   │   ├── vec2.h                 # 2D vector math
│   │   ├── boids.h/.cpp           # Flock simulation
│   │   └── scheduler.h/.cpp       # Maps boids to grain triggers
│   ├── ui/
│   │   ├── display.h/.cpp         # OLED rendering (3 pages)
│   │   └── led_grid.h/.cpp        # LED visualization
│   └── .vscode/                   # VSCode configuration
├── .gitmodules
└── .gitignore
```

## Controls

| Knob | Parameter | Description |
|------|-----------|-------------|
| CTRL_1 | Separation (0-2) | Spreads boids apart → wider spectral spread |
| CTRL_2 | Cohesion (0-2) | Pulls boids together → focused, clustered sound |
| CTRL_3 | Grain Density (1-50 Hz) | Base trigger rate for grains |
| CTRL_4 | Pitch Range (0-24 st) | Y-axis maps to ± this many semitones |

| Gate | Function |
|------|----------|
| GATE_1 | Freeze/unfreeze buffer (toggle recording) |
| GATE_2 | Scatter flock (randomize all boid positions) |

| Encoder | Function |
|---------|----------|
| Rotate | Change number of boids (4-16) |
| Press | Cycle display pages |
| Long press | Toggle recording |

## Display Pages

1. **Flock View** - Visual boid simulation on OLED
2. **Parameters** - Current parameter values
3. **Waveform** - Audio buffer visualization

## How It Works

### Boid → Grain Mapping
| Boid Property | Grain Parameter |
|---------------|-----------------|
| position.x | Buffer playback position (0-100%) |
| position.y | Pitch (±pitch_range semitones) |
| velocity magnitude | Grain size (faster = smaller grains) |
| velocity angle | Stereo pan (-1 to +1) |

### Audio Signal Flow
1. Stereo input mixed to mono → written to 4-second circular buffer
2. Boids simulation runs at 60fps
3. Each boid has its own trigger timer (faster boids trigger more often)
4. When triggered, boid state maps to GrainParams
5. 16-voice grain pool plays grains with Hann envelope
6. Output mixed 50/50 dry/wet

## Memory Usage

| Region | Used | Total | % |
|--------|------|-------|---|
| FLASH | 105KB | 128KB | 80.7% |
| SRAM | 57KB | 512KB | 11% |
| SDRAM | 750KB | 64MB | 1.1% |

## Testing Checklist

### Basic Functionality
- [ ] Audio passes through (dry signal audible)
- [ ] Grains triggered (hear granular texture)
- [ ] OLED displays boid animation
- [ ] Knobs respond smoothly

### Boid Behavior
- [ ] Separation knob spreads boids apart visually
- [ ] Cohesion knob clusters boids together
- [ ] Encoder changes boid count (visible on OLED)
- [ ] GATE_2 scatters flock (boids randomize positions)

### Audio Behavior
- [ ] Separation creates wider spectral spread
- [ ] Cohesion creates focused sound
- [ ] Density knob changes grain rate
- [ ] Pitch range affects pitch variation

### Recording
- [ ] GATE_1 freezes buffer (grains loop same material)
- [ ] Unfreezing resumes live recording

---

# Daisy Patch Project Setup Guide (Reference)

## Project Structure Overview

```
MyDaisyProject/                    # Your new repository root
├── libDaisy/                      # Git submodule
├── DaisySP/                       # Git submodule
├── HelloPatch/                    # Your actual program
│   ├── HelloPatch.cpp            # Main code
│   ├── Makefile                  # Build configuration
│   ├── README.md                 # Project description
│   └── .vscode/                  # VSCode configuration
│       ├── tasks.json            # Build & flash tasks
│       ├── c_cpp_properties.json # IntelliSense config
│       ├── launch.json           # Debugging config (optional)
│       └── STM32H750x.svd        # Chip peripheral definitions
├── .gitmodules                   # Submodule configuration
├── .gitignore                    # Ignore build artifacts
└── README.md                     # Repository overview
```

## Step-by-Step Implementation Plan

### Phase 1: Repository Setup

1. **Create new repository**
   - Create a new directory for your project
   - Initialize git repository: `git init`
   - Create basic README.md

2. **Add library dependencies as git submodules**
   - Add libDaisy: `git submodule add https://github.com/electro-smith/libDaisy`
   - Add DaisySP: `git submodule add https://github.com/electro-smith/DaisySP`
   - Create `.gitmodules` file (automatically created by submodule commands)

3. **Build the libraries**
   - Build libDaisy: `cd libDaisy && make`
   - Build DaisySP: `cd DaisySP && make`

### Phase 2: Create Your Project Directory

4. **Create project folder**
   - Create `HelloPatch/` directory at repository root
   - This is where your actual program lives

5. **Create the Makefile**
   - Model after DaisyExamples `patch/lfo/Makefile` as template
   - Key settings needed:
     - `TARGET = HelloPatch` (your program name)
     - `CPP_SOURCES = HelloPatch.cpp` (your source file)
     - `LIBDAISY_DIR = ../libDaisy` (path from HelloPatch/ to libDaisy/)
     - `DAISYSP_DIR = ../DaisySP` (path from HelloPatch/ to DaisySP/)
     - `SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core`
     - `include $(SYSTEM_FILES_DIR)/Makefile` (pulls in all build rules)

### Phase 3: Create Your Code

6. **Write HelloPatch.cpp**
   - Model after `patch/lfo/lfo.cpp`
   - Minimal "hello world" features:
     - Initialize the hardware
     - Blink an LED or light up the LED grid
     - Display "Hello World" on the OLED screen
     - Pass audio through (input to output)

   Basic structure:
   ```cpp
   #include "daisysp.h"
   #include "daisy_patch.h"

   using namespace daisy;
   using namespace daisysp;

   DaisyPatch patch;

   static void AudioCallback(AudioHandle::InputBuffer  in,
                             AudioHandle::OutputBuffer out,
                             size_t                    size)
   {
       // Process audio
   }

   int main(void)
   {
       patch.Init();
       patch.StartAdc();
       patch.StartAudio(AudioCallback);

       while(1)
       {
           // Update display, etc.
       }
   }
   ```

### Phase 4: VSCode Configuration

7. **Create `.vscode/` directory** inside `HelloPatch/`

8. **Create `tasks.json`**
   - Copy from DaisyExamples `patch/lfo/.vscode/tasks.json`
   - Update library paths: change `../../libDaisy` to `../libDaisy` (since you're one level shallower)
   - Update library paths: change `../../DaisySP` to `../DaisySP`
   - Key tasks you need:
     - `build` - Compile the project
     - `build_and_program_dfu` - Build and flash via USB (DFU mode)
     - `build_libdaisy` - Build the library
     - `build_daisysp` - Build the DSP library

9. **Create `c_cpp_properties.json`**
   - Copy from DaisyExamples `patch/lfo/.vscode/c_cpp_properties.json`
   - Update include paths to `../libDaisy/**` and `../DaisySP/**`
   - This helps VSCode IntelliSense find the header files

10. **Copy `STM32H750x.svd`**
    - This file provides hardware register definitions for debugging
    - Copy from any example project's `.vscode/` folder (e.g., `patch/lfo/.vscode/STM32H750x.svd`)

11. **Create `launch.json`** (optional, for debugging)
    - Only needed if you plan to use a debugger (ST-Link, J-Link)
    - Can skip for initial development

### Phase 5: Build & Flash

12. **Test the build**
    - From `HelloPatch/` directory: `make clean && make`
    - Should produce `build/HelloPatch.bin` file

13. **Flash to hardware**
    - Put Daisy Patch in DFU mode (hold BOOT button, press RESET)
    - In VSCode: Run task "build_and_program_dfu" (Cmd+Shift+P → "Tasks: Run Task")
    - Or from terminal: `make program-dfu`

### Phase 6: Version Control

14. **Create `.gitignore`**
    - Ignore `build/` directories
    - Ignore `*.bin`, `*.elf`, `*.hex` files
    - Ignore other build artifacts

    Example:
    ```
    build/
    *.bin
    *.elf
    *.hex
    *.map
    *.o
    *.d
    .DS_Store
    ```

15. **Initial commit**
    - Commit your code, Makefile, and VSCode configs
    - The submodules (libDaisy, DaisySP) are tracked by reference only

## Key Differences from DaisyExamples Structure

| DaisyExamples | Your Standalone Project |
|---------------|------------------------|
| `patch/lfo/` is 2 levels deep | `HelloPatch/` is 1 level deep |
| `../../libDaisy` in Makefile | `../libDaisy` in Makefile |
| Many example projects | Single focused project |
| Examples repo structure | Clean standalone repo |

## Hardware Flashing Methods

- **DFU (USB)**: `make program-dfu` - Requires BOOT button hold
- **ST-Link/J-Link**: `make program` - Requires hardware debugger
- DFU is easier for beginners, no extra hardware needed

## Critical Files Summary

**Must have:**
- `HelloPatch.cpp` - Your code
- `Makefile` - Build instructions
- `libDaisy/` and `DaisySP/` - Libraries (as git submodules)

**Strongly recommended:**
- `.vscode/tasks.json` - Build/flash commands
- `.vscode/c_cpp_properties.json` - IDE support
- `.gitignore` - Keep repo clean

**Optional:**
- `.vscode/launch.json` - For debugging
- `.vscode/STM32H750x.svd` - For debugging

## Daisy Patch Hardware Reference

### Available I/O:
- 4x Audio Inputs
- 4x Audio Outputs
- 4x Knobs (CV-capable)
- 4x CV inputs
- 2x Gate Inputs
- 1x Gate Output
- TRS MIDI In and Out
- 2x buttons
- 1x toggle
- 1x encoder (with push button)
- USB Mini
- MicroSD Port
- Grid of 16 LEDs
- OLED Display (128x64)

### Reading Hardware:
```cpp
// Update controls
patch.ProcessAnalogControls();
patch.ProcessDigitalControls();

// Read knobs (0-1 range)
float knob_value = patch.GetKnobValue(DaisyPatch::CTRL_1);

// Read gates
bool gate = patch.gate_input[0].Trig();

// Read encoder
int32_t increment = patch.encoder.Increment();
bool button_pressed = patch.encoder.FallingEdge();
```

### Audio Callback:
```cpp
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    for(size_t i = 0; i < size; i++)
    {
        // Read from in[0][i] through in[3][i]
        // Write to out[0][i] through out[3][i]
    }
}
```

## Next Steps

Once you've copied this file to your new project, we can work through each phase step by step.
