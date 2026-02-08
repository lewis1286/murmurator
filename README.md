# Murmur Boids

A granular synthesis module for [Daisy Patch](https://www.electro-smith.com/daisy/patch) where a flock of boids controls grain parameters. Each boid represents a grain voice—its 2D position and velocity map to playback position, pitch, grain size, and stereo pan.

## Demo

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
