# Murmur Boids - 3D Flocking Oscillator Synth

A Daisy Patch module where a flock of 3D boids controls oscillator voices. Each boid is an always-on sine oscillator - its 3D position maps directly to audio parameters: **x = pan, y = frequency, z = amplitude**.

## Current Status

- ✓ 3D boids simulation (Vec3, single-pass flocking with brute-force neighbors)
- ✓ Oscillator voices (DaisySP Oscillator, one per boid, linear panning)
- ✓ OLED visualization (triangle size varies with z/amplitude, low freq at bottom)
- ✓ Full, UI-only, and debug builds working
- ✓ 500Hz boids tick rate with FastInvSqrt optimization

## Future Ideas (Backlog)

1. **Rethink Y-axis parameter** — frequency may be too simple. Candidates: wave-folding, filter cutoff, vibrato/FM depth, reverb send, detune. Also explore quantizing Y to a scale/chord (user picks root + mode, boids snap to scale degrees).
2. **Granular synthesis revival** — old engine (circular_buffer, grain_voice, grain_pool, scheduler) is still in repo. Could map 3D boid axes to grain params (playback position, size, pitch, pan) instead of or alongside sine oscillators.
3. **Assignable X/Y/Z menu** — encoder-driven settings page (4th display page) where user assigns each boid axis to any audio parameter (freq, amp, pan, filter, detune, reverb, wave-fold, etc.) without reflashing.
4. **Single "Flock Density" knob** — merge CTRL_1 (separation) and CTRL_2 (cohesion) into one bipolar control. CCW = max separation, CW = max cohesion. Frees a knob for a new parameter.

Previous granular synthesis engine files (circular_buffer, grain_voice, grain_pool, scheduler) kept in repo for reference but removed from build.

## Quick Start

```bash
# Full build (audio + UI)
cd murmur
make clean && make

# UI-only build (no audio, for testing OLED/LEDs/knobs/encoder)
make clean && make ui-only

# Flash using debugger probe
make program
```

### Build Variants

- `make ui-only` — skip audio initialization (oscillator voices, `StartAudio`). Uses `MURMUR_UI_ONLY` preprocessor flag.
- `make debug` — full build with `-Og` optimization for stepping through code.
- `make debug-ui-only` — debug + UI-only combined.

Always `make clean` when switching between build variants.

## Project Structure

```
murmurator/
├── libDaisy/                      # Git submodule
├── DaisySP/                       # Git submodule
├── murmur/                        # Main project
│   ├── MurmurBoids.cpp           # Main application
│   ├── Makefile                  # Build configuration
│   ├── audio/
│   │   ├── osc_voice.h           # Oscillator voice (sine, linear pan)
│   │   ├── circular_buffer.h/.cpp # (legacy, not in build)
│   │   ├── grain_voice.h/.cpp     # (legacy, not in build)
│   │   └── grain_pool.h/.cpp      # (legacy, not in build)
│   ├── boids/
│   │   ├── vec3.h                 # 3D vector math + FastInvSqrt
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
| CTRL_1 | Density | 0-1 | CCW = min separation (cluster), CW = max separation (spread) |
| CTRL_2 | (reserved) | — | — |
| CTRL_3 | Freq Range / Span | 50-800 Hz / 1-4 oct | Hz spread (scale=OFF) or octave span (scale active) |
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

1. **Flock View** - Boid triangles on OLED; triangle size varies with z (amplitude); low freq at bottom, high freq at top
2. **Parameters** - Current control values (integer-formatted for newlib-nano compatibility) and mapping info
3. **Waveform** - Real-time summed oscillator output

## How It Works

### 3D Boid → Oscillator Mapping
| Boid Axis | Audio Param | Range | Formula |
|-----------|------------|-------|---------|
| x (0-1) | Stereo pan | L to R | `pan = x * 2 - 1` |
| y (0-1) | Frequency | 200 + y * freq_range Hz | `freq = FREQ_MIN + y * freq_range` |
| z (0-1) | Amplitude | 0 to max_amp/num_boids | `amp = z * MAX_AMP / num_boids` |

### Audio Signal Flow
1. Boids simulation runs at 500Hz (2ms tick)
2. Each boid's 3D position maps to its oscillator's freq/amp/pan
3. Parameters smoothed via one-pole filter (coeff=0.006) to avoid clicks
4. Audio callback sums all active oscillators per sample
5. Linear panning: `gain_l = (1 - pan_norm) * amp`, `gain_r = pan_norm * amp`

## Memory Usage

| Region | Used | Total | % |
|--------|------|-------|---|
| FLASH | 103KB | 128KB | 78.7% |
| SRAM | 56KB | 512KB | 10.7% |
| SDRAM | 0 | 64MB | 0% |

## Build Modes

| Mode | Command | What runs |
|------|---------|-----------|
| Full | `make clean && make` | Audio callback + boids + UI |
| UI-only | `make clean && make ui-only` | Boids + UI only (no audio) |
| Debug | `make clean && make debug` | Full build with `-Og` for debugger |
| Debug UI-only | `make clean && make debug-ui-only` | UI-only with `-Og` |

## Performance Optimizations

- **-O2 compiler optimization** (was `-Og`), ~2-5x overall speedup
- **Single-pass flocking**: `ApplyFlockingForces()` computes separation, alignment, and cohesion in one neighbor traversal (was 3 separate passes with 3 `GetNeighbors()` calls each)
- **Brute-force neighbors**: Removed 4x4 spatial grid. With max 16 boids, direct O(N^2) loop with squared distance is faster and gives correct 3D neighbor detection
- **Squared distances**: All neighbor comparisons use `DistanceSquared()`, eliminating sqrtf from the hot path
- **FastInvSqrt**: Quake-style fast inverse square root for steering normalization (~1-2% error, fine for flocking)
- **floorf wrap**: `WrapPosition` uses `pos -= floorf(pos)` instead of while-loops (branchless on Cortex-M7 FPU)
- **500Hz tick rate** (was 62.5Hz / 16ms)

## Bug Fixes & Known Issues

### Edge Freeze Fix (WrapPosition)

`BoidsFlock::WrapPosition` in `boids/boids.cpp` guards against `Inf`/`NaN` positions using `std::isfinite()` on all three axes. Resets non-finite positions to center `(0.5, 0.5, 0.5)`.

### DrawLine Unsigned Overflow

libDaisy's `DrawLine` uses `uint_fast8_t` (= `uint32_t` on ARM). Negative coordinates wrap to ~4 billion, causing Bresenham to loop forever. All triangle vertices are clamped before drawing.

### newlib-nano Float Printf

`--specs=nano.specs` strips float printf support. Parameters page uses integer-based formatting (`%d.%02d`) instead of `%.2f`.

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
Flash: `make clean && make && make program`

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
