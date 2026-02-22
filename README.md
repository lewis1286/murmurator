# Murmur Boids

A granular synthesis module for [Daisy Patch](https://www.electro-smith.com/daisy/patch) where a flock of boids controls grain parameters. Each boid represents a grain voice—its 2D position and velocity map to playback position, pitch, grain size, and stereo pan.


# Ideas & Future Exploration

## 1. Rethink the Y-axis parameter
Frequency is a natural but maybe uninteresting choice for `y`. Alternatives to explore:
- **Wave-folding** — y controls fold amount, adds harmonic richness without pitch shift
- **Filter cutoff** — y sweeps a lowpass/bandpass; flock height becomes brightness
- **Vibrato/FM depth** — y controls modulation index; tightly clustered boids = subtle, spread = intense
- **Reverb send** — y controls wet/dry; higher boids sound more spacious
- **Detune** — y offsets from a fixed root pitch; flock movement creates subtle chorusing

### Chord/scale quantization
Instead of a continuous frequency range, quantize `y` to a user-selectable scale/chord. Ideas:
- User picks root note + mode (major, minor, pentatonic, etc.)
- Boids snap to the nearest scale degree as they move
- Could expose as a menu option alongside free-running mode

## 2. Bring back granular synthesis
The old granular engine (circular_buffer, grain_voice, grain_pool, scheduler) is still in the repo. Possible hybrid:
- Boids control grain parameters (playback position, grain size, pitch, pan) instead of — or alongside — the sine oscillators
- Keep the 3D boid sim; map axes to grain params the same way as current osc params

## 3. Assignable X/Y/Z menu
Add an in-module menu (encoder-driven) so the user can assign any boid axis to any audio parameter:
- X, Y, Z each get a slot: frequency, amplitude, pan, filter cutoff, detune, reverb send, wave-fold, etc.
- Lets users discover their own mappings without reflashing
- Could be a fourth display page (settings page)

## 4. Merge separation + cohesion into a single "flock density" control
Currently CTRL_1 (separation) and CTRL_2 (cohesion) are separate, which is redundant — they pull in opposite directions. Proposal:
- Single **Density** knob: CCW = max separation (boids repel, spread out), center = balanced, CW = max cohesion (boids cluster)
- Internally: `separation = map(density, 0, 1, 2, 0)`, `cohesion = map(density, 0, 1, 0, 2)`
- Frees up a knob — could be repurposed for a new parameter (e.g. a fifth axis like filter cutoff)

# Demo

Feed audio into the module, and watch as the boids flock across the OLED display. Their movement creates evolving granular textures—separation spreads the sound across the stereo field and pitch range, while cohesion creates focused, clustered tones.

## Features

- **4-second live audio buffer** in SDRAM for granular playback
- **16-voice polyphony** with voice stealing
- **Boids flocking simulation** with separation, alignment, and cohesion
- **Spatial partitioning** for efficient neighbor queries (O(n) vs O(n²))
- **OLED visualization** of flock behavior
- **Gate-triggered buffer freeze** and flock scatter

## Controls

| Knob | Parameter | Description |
|------|-----------|-------------|
| CTRL_1 | Separation | Spreads boids apart → wider spectral spread |
| CTRL_2 | Cohesion | Pulls boids together → focused, clustered sound |
| CTRL_3 | Grain Density | Base trigger rate (1-50 Hz) |
| CTRL_4 | Pitch Range | Y-axis pitch mapping (0-24 semitones) |

| Gate | Function |
|------|----------|
| GATE_1 | Freeze/unfreeze buffer |
| GATE_2 | Scatter flock (randomize positions) |

| Encoder | Function |
|---------|----------|
| Rotate | Number of boids (4-16) |
| Press | Cycle display pages |

## Boid → Grain Mapping

| Boid Property | Grain Parameter |
|---------------|-----------------|
| position.x | Buffer playback position |
| position.y | Pitch (±pitch_range semitones) |
| velocity magnitude | Grain size (faster = smaller) |
| velocity angle | Stereo pan |

## Building

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Make
- [dfu-util](http://dfu-util.sourceforge.net/) for flashing

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

# UI-only build (no audio, for testing display/knobs/boids)
make clean && make ui-only

# Put Daisy in DFU mode: hold BOOT, press RESET
make program-dfu
```

> **Note:** Always run `make clean` when switching between full and UI-only builds.

## Project Structure

```
murmurator/
├── libDaisy/                 # Hardware abstraction (submodule)
├── DaisySP/                  # DSP library (submodule)
└── murmur/
    ├── MurmurBoids.cpp       # Main application
    ├── audio/
    │   ├── circular_buffer   # 4-second ring buffer
    │   ├── grain_voice       # Single grain with Hann envelope
    │   └── grain_pool        # 16-voice polyphony
    ├── boids/
    │   ├── vec2.h            # 2D vector math
    │   ├── boids             # Flock simulation
    │   └── scheduler         # Boid → grain triggering
    └── ui/
        ├── display           # OLED rendering
        └── led_grid          # LED visualization
```

## License

MIT

## Acknowledgments

- [Electro-Smith](https://www.electro-smith.com/) for the Daisy platform
- Craig Reynolds for the original [boids algorithm](https://www.red3d.com/cw/boids/)
- Inspired by Mutable Instruments Clouds
