# Murmur Boids - 3D Flocking Oscillator Synth

A Daisy Patch module where a flock of 3D boids controls oscillator voices. Each boid is an always-on sine oscillator - its 3D position maps directly to audio parameters: **x = pan, y = frequency, z = amplitude**.

## Current Status

- ✓ 3D boids simulation (Vec3, flocking with spatial partitioning)
- ✓ Oscillator voices (DaisySP Oscillator, one per boid, equal-power panning)
- ✓ OLED visualization (triangle size varies with z/amplitude)
- ✓ Full and UI-only builds working

Previous granular synthesis engine files (circular_buffer, grain_voice, grain_pool, scheduler) kept in repo for reference but removed from build.

## Quick Start

```bash
# Full build (audio + UI)
cd murmur
make clean && make

# UI-only build (no audio, for testing OLED/LEDs/knobs/encoder)
make clean && make ui-only

# Flash (put Daisy in DFU mode: hold BOOT, press RESET)
make program-dfu

# Flash using debugger probe
make program
```

### UI-Only Mode

Build with `make ui-only` to skip audio initialization (oscillator voices, `StartAudio`). Uses `MURMUR_UI_ONLY` preprocessor flag. Always `make clean` when switching between full and UI-only builds.

## Project Structure

```
murmurator/
├── libDaisy/                      # Git submodule
├── DaisySP/                       # Git submodule
├── murmur/                        # Main project
│   ├── MurmurBoids.cpp           # Main application
│   ├── Makefile                  # Build configuration
│   ├── audio/
│   │   ├── osc_voice.h           # Oscillator voice (sine, equal-power pan)
│   │   ├── circular_buffer.h/.cpp # (legacy, not in build)
│   │   ├── grain_voice.h/.cpp     # (legacy, not in build)
│   │   └── grain_pool.h/.cpp      # (legacy, not in build)
│   ├── boids/
│   │   ├── vec3.h                 # 3D vector math
│   │   ├── vec2.h                 # (legacy, kept for reference)
│   │   ├── boids.h/.cpp           # 3D flock simulation
│   │   └── scheduler.h/.cpp       # (legacy, not in build)
│   ├── ui/
│   │   ├── display.h/.cpp         # OLED rendering (3 pages)
│   │   └── led_grid.h/.cpp        # LED visualization
│   └── .vscode/                   # VSCode configuration
├── .gitmodules
└── .gitignore
```

## Controls

| Knob | Parameter | Range | Description |
|------|-----------|-------|-------------|
| CTRL_1 | Separation | 0-2 | Spreads boids apart → wider frequency/pan spread |
| CTRL_2 | Cohesion | 0-2 | Pulls boids together → focused, unison-like sound |
| CTRL_3 | Freq Range | 50-800 Hz | Controls frequency spread of oscillators |
| CTRL_4 | Alignment | 0-2 | Velocity alignment → synchronized movement |

| Gate | Function |
|------|----------|
| GATE_1 | (reserved) |
| GATE_2 | Scatter flock (randomize all boid positions) |

| Encoder | Function |
|---------|----------|
| Rotate | Change number of boids/voices (4-16) |
| Press | Cycle display pages |

## Display Pages

1. **Flock View** - Boid triangles on OLED; triangle size varies with z (amplitude)
2. **Parameters** - Current control values and mapping info
3. **Waveform** - Real-time summed oscillator output

## How It Works

### 3D Boid → Oscillator Mapping
| Boid Axis | Audio Param | Range | Formula |
|-----------|------------|-------|---------|
| x (0-1) | Stereo pan | L to R | `pan = x * 2 - 1` |
| y (0-1) | Frequency | 200 + y * freq_range Hz | `freq = FREQ_MIN + y * freq_range` |
| z (0-1) | Amplitude | 0 to max_amp/num_boids | `amp = z * MAX_AMP / num_boids` |

### Audio Signal Flow
1. Boids simulation runs at ~60fps (16ms tick)
2. Each boid's 3D position maps to its oscillator's freq/amp/pan
3. Parameters smoothed via one-pole filter to avoid clicks
4. Audio callback sums all active oscillators per sample
5. Equal-power panning: `gain_l = cos(pan * PI/2) * amp`, `gain_r = sin(pan * PI/2) * amp`

## Memory Usage

| Region | Used | Total | % |
|--------|------|-------|---|
| FLASH | 105KB | 128KB | 80.3% |
| SRAM | 57KB | 512KB | 10.9% |
| SDRAM | 0 | 64MB | 0% |

## Build Modes

| Mode | Command | What runs |
|------|---------|-----------|
| Full | `make clean && make` | Audio callback + boids + UI |
| UI-only | `make clean && make ui-only` | Boids + UI only (no audio) |

## Bug Fixes

### Edge Freeze Fix (WrapPosition)

`BoidsFlock::WrapPosition` in `boids/boids.cpp` guards against `Inf`/`NaN` positions using `std::isfinite()` on all three axes. Resets non-finite positions to center `(0.5, 0.5, 0.5)`.

### DrawLine Unsigned Overflow

libDaisy's `DrawLine` uses `uint_fast8_t` (= `uint32_t` on ARM). Negative coordinates wrap to ~4 billion, causing Bresenham to loop forever. All triangle vertices are clamped before drawing.

## Hardware Testing

### Test 1: UI-Only Smoke Test
Flash: `make clean && make ui-only && make program`

1. **OLED boots** - "MURMUR BOIDS" title visible, boid triangles moving
2. **Variable triangle size** - triangles should be different sizes (z-axis controls size, range 2-6px). Watch for a few seconds to confirm sizes change as boids move
3. **Encoder rotate** - turn encoder, boid count (shown top-right) changes between 4-16. Visible boids should match the count
4. **Encoder press** - cycles through 3 pages: Flock View → Parameters → Waveform → back to Flock View
5. **CTRL_1 (Separation)** - turn full CW, boids should spread apart on screen. Turn full CCW, boids cluster
6. **CTRL_2 (Cohesion)** - turn full CW, boids should cluster tightly. Turn full CCW, loose flock
7. **CTRL_4 (Alignment)** - turn full CW, boids align velocities (move in same direction). Turn full CCW, more chaotic movement
8. **GATE_2 trigger** - send a gate pulse, all boids should jump to random positions
9. **Edge stability** - let it run for 2+ minutes. No freezes, no stuck boids, triangles stay within the border rectangle
10. **Parameters page** - shows Sep/Coh/Frq/Ali values, boid count, and "x:pan y:freq z:amp" mapping label

### Test 2: Full Audio Build
Flash: `make clean && make && make program-dfu`

1. **Startup drone** - immediately hear a chord-like drone from outputs 1+2 (8 sine oscillators at different frequencies). No input signal needed
2. **Stereo field** - plug in headphones or monitor L+R. Sound should have width - not mono. As boids move on x-axis, individual tones drift L/R
3. **CTRL_3 (Freq Range)** - turn full CCW (50Hz range): tight cluster of frequencies, nearly unison. Turn full CW (800Hz range): wide spread, dissonant chord
4. **CTRL_1 (Separation) + listen** - high separation → boids spread → wider frequency intervals, more dissonant. Low separation → frequencies converge
5. **CTRL_2 (Cohesion) + listen** - high cohesion → boids cluster → near-unison, beating/chorusing effect. Low cohesion → looser, more varied
6. **CTRL_4 (Alignment) + listen** - high alignment → boids move together → more stable drone. Low alignment → more chaotic parameter changes
7. **Encoder boid count** - rotate to change 4→16 voices. Fewer voices = sparser, individual tones audible. More voices = denser, richer texture. Listen for clicks during count changes (should be smooth due to amplitude fade)
8. **GATE_2 scatter** - trigger gate, hear all frequencies/pans/amplitudes jump to new random values. Should be a sudden dramatic shift in the sound
9. **Amplitude variation** - z-axis controls volume per voice. Watch OLED: larger triangles = louder voices. The overall level should stay reasonable (MAX_AMP_TOTAL = 0.8 divided across all voices)
10. **Waveform page** - press encoder to page 3. Should show a live waveform of the summed oscillator output (not a flat line)

### Test 3: Stability and Edge Cases
1. **Rapid encoder spinning** - quickly spin encoder between 4-16 and back. No audio glitches, no freezes
2. **All knobs at extremes** - set all 4 knobs fully CW, then all fully CCW. No crashes or audio blowups
3. **Rapid gate triggers** - send fast gate pulses to GATE_2 (~10Hz). Boids scatter repeatedly without freezing
4. **Long run** - leave running for 10+ minutes with knobs at moderate positions. Verify no drift, no freeze, no audio degradation
5. **Outputs 3+4** - should pass through inputs 3+4 unchanged (audio passthrough on channels 3-4)

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
patch.ProcessAnalogControls();
patch.ProcessDigitalControls();
float knob_value = patch.GetKnobValue(DaisyPatch::CTRL_1);
bool gate = patch.gate_input[0].Trig();
int32_t increment = patch.encoder.Increment();
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
