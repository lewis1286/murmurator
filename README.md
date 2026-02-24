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

## Acknowledgments

- [Electro-Smith](https://www.electro-smith.com/) for the Daisy platform
- Craig Reynolds for the original [boids algorithm](https://www.red3d.com/cw/boids/)
