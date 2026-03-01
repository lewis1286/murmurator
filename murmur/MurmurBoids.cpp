#include "daisysp.h"
#include "daisy_patch.h"
#include "audio/osc_voice.h"
#include "audio/simple_reverb.h"
#include "audio/scale_quantizer.h"
#include "audio/chord_progression.h"
#include "audio/axis_mapping.h"
#include "boids/boids.h"
#include "ui/display.h"
#include "ui/led_grid.h"
#include <cmath>

using namespace daisy;
using namespace daisysp;

// Hardware
DaisyPatch patch;

// Oscillator voices (one per boid)
murmur::OscVoice voices[murmur::MAX_BOIDS];

// Shared reverb bus for z-axis distance simulation (mono in, mono out)
murmur::SimpleReverb reverb;
constexpr float REVERB_LEVEL = 0.4f;

// Boids
murmur::BoidsFlock flock;
murmur::BoidsParams boids_params;

// UI
murmur::Display display;
murmur::LedGrid led_grid;

// Control parameters
float density          = 0.5f;   // CTRL_1: CCW=max separation, CW=max cohesion
float alignment_weight = 1.0f;   // CTRL_2
// CTRL_3: speed (handled inline in UpdateControls)
float morph            = 1.0f;   // CTRL_4: waveform morph (0=sine, 1=tri, 2=square)

// Frequency range — fixed (no longer knob-controlled)
float freq_range = 400.0f;

// Scale quantizer (default: root=A, OFF, octave=3)
murmur::ScaleQuantizer scale_quantizer;
murmur::ChordProgression chord_prog;
murmur::AxisMapping axis_mapping;  // default: x=pan, y=freq, z=amp
int settings_cursor = 0;  // 0=root, 1=scale, 2=base_octave, 3=chord_prog
int span_octaves    = 3;  // octave span when scale mode is active

// Audio parameters
constexpr float FREQ_MIN = 200.0f;
constexpr float FREQ_MAX = 800.0f;
constexpr float MAX_AMP_TOTAL = 0.8f;  // Total max amplitude across all voices

// State
int num_boids = 8;
float sample_rate = 48000.0f;

// Timing
uint32_t last_display_update = 0;
uint32_t last_boids_update = 0;
constexpr uint32_t DISPLAY_UPDATE_MS = 33;
constexpr uint32_t BOIDS_UPDATE_MS = 2;

void UpdateControls();
void UpdateDisplay();
void UpdateVoicesFromBoids();

#ifndef MURMUR_UI_ONLY
static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size) {
    for (size_t i = 0; i < size; i++) {
        float sum_l  = 0.0f;
        float sum_r  = 0.0f;
        float rev_in = 0.0f;

        for (size_t v = 0; v < static_cast<size_t>(num_boids); v++) {
            sum_l  += voices[v].ProcessLeft();
            sum_r  += voices[v].ProcessRight();
            rev_in += voices[v].GetReverbSend();
        }

        // Mix reverb tail into output — adds spatial depth for far (low-z) boids
        float rev_out = reverb.Process(rev_in);
        out[0][i] = sum_l + rev_out * REVERB_LEVEL;
        out[1][i] = sum_r + rev_out * REVERB_LEVEL;
        out[2][i] = in[2][i];
        out[3][i] = in[3][i];
    }
}
#endif

int main(void) {
    patch.Init();
    sample_rate = patch.AudioSampleRate();

    // Initialize oscillator voices and reverb
#ifndef MURMUR_UI_ONLY
    for (size_t i = 0; i < murmur::MAX_BOIDS; i++) {
        voices[i].Init(sample_rate);
    }
    reverb.Init(sample_rate);
#endif

    // Initialize boids
    flock.Init(num_boids);
    boids_params.separation_weight = density * 2.0f;
    boids_params.cohesion_weight   = (1.0f - density) * 2.0f;
    boids_params.alignment_weight  = alignment_weight;
    boids_params.perception_radius = 0.25f;
    boids_params.max_speed = 0.3f;
    boids_params.max_force = 0.3f * 0.5f;  // coupled: force scales with speed

    // Initialize UI
    display.Init(&patch);
    led_grid.Init(&patch);

    // Activate initial voices
#ifndef MURMUR_UI_ONLY
    for (int i = 0; i < num_boids; i++) {
        voices[i].SetActive(true);
    }
#endif

    patch.StartAdc();
#ifndef MURMUR_UI_ONLY
    patch.StartAudio(AudioCallback);
#endif

    while (1) {
        UpdateControls();

        uint32_t now = System::GetNow();

        chord_prog.Update(now, scale_quantizer);

        // Update boids simulation
        if (now - last_boids_update >= BOIDS_UPDATE_MS) {
            float dt = static_cast<float>(now - last_boids_update) / 1000.0f;
            flock.Update(dt, boids_params);

#ifndef MURMUR_UI_ONLY
            UpdateVoicesFromBoids();
#endif

            last_boids_update = now;
        }

        // Update display and LEDs (visual rate)
        if (now - last_display_update >= DISPLAY_UPDATE_MS) {
            led_grid.UpdateFromFlock(flock);
            UpdateDisplay();
            last_display_update = now;
        }
    }
}

void UpdateVoicesFromBoids() {
    murmur::MappingContext ctx = {
        scale_quantizer,
        FREQ_MIN,
        freq_range,
        span_octaves,
        MAX_AMP_TOTAL / static_cast<float>(num_boids)
    };

    for (int i = 0; i < num_boids; i++) {
        const murmur::Boid& boid = flock.GetBoid(i);
        murmur::VoiceParams vp = MapBoidToVoice(boid.position, axis_mapping, ctx);

        // boid.position.z is passed as depth hint regardless of axis assignment —
        // OscVoice uses it for filter brightness and reverb send scaling.
        voices[i].SetParams(vp.freq, vp.amp, vp.pan, boid.position.z);
        voices[i].SetMorph(morph);
        // In scale mode, snap freq immediately so boids land on discrete notes
        // rather than gliding through them (amp/pan still smooth normally).
        if (scale_quantizer.GetScale() != murmur::ScaleType::OFF) {
            voices[i].SnapFreq(vp.freq);
        }
        voices[i].UpdateSmoothing();
    }
}

void UpdateControls() {
    patch.ProcessAnalogControls();
    patch.ProcessDigitalControls();

    // === KNOBS ===
    // CTRL_1: Density — CCW = min separation (cluster), CW = max separation (spread)
    density = patch.GetKnobValue(DaisyPatch::CTRL_1);
    boids_params.separation_weight = density * 2.0f;
    boids_params.cohesion_weight   = (1.0f - density) * 2.0f;

    // CTRL_2: Alignment weight (0-2)
    alignment_weight = patch.GetKnobValue(DaisyPatch::CTRL_2) * 2.0f;
    boids_params.alignment_weight = alignment_weight;

    // CTRL_3: Speed (max_speed 0.05-1.5); max_force coupled so boids can reach target speed
    boids_params.max_speed = 0.05f + patch.GetKnobValue(DaisyPatch::CTRL_3) * 1.45f;
    boids_params.max_force = boids_params.max_speed * 0.5f;

    // CTRL_4: Waveform morph (0=sine, 1=triangle, 2=square)
    morph = patch.GetKnobValue(DaisyPatch::CTRL_4) * 2.0f;

    // === ENCODER ===
    int inc = patch.encoder.Increment();

    if (display.GetPage() == murmur::DisplayPage::SCALE_SETTINGS) {
        // On Scale Settings page: encoder navigates/edits settings.
        if (inc != 0) {
            switch (settings_cursor) {
                case 0: {
                    // Root: wrap 0-11
                    int r = ((scale_quantizer.GetRoot() + inc) % 12 + 12) % 12;
                    scale_quantizer.SetRoot(r);
                    break;
                }
                case 1: {
                    // Scale type: wrap 0 to COUNT-1
                    int s = ((static_cast<int>(scale_quantizer.GetScale()) + inc)
                             % static_cast<int>(murmur::ScaleType::COUNT)
                             + static_cast<int>(murmur::ScaleType::COUNT))
                            % static_cast<int>(murmur::ScaleType::COUNT);
                    scale_quantizer.SetScale(static_cast<murmur::ScaleType>(s));
                    break;
                }
                case 2:
                    // Base octave: clamp 1-5
                    scale_quantizer.SetBaseOctave(scale_quantizer.GetBaseOctave() + inc);
                    break;
                case 3:
                    chord_prog.Increment(inc, System::GetNow(), scale_quantizer);
                    break;
                default:
                    break;
            }
        }

        // Press: advance cursor; after chord prog row exit back to Flock View
        if (patch.encoder.RisingEdge()) {
            if (settings_cursor < 3) {
                settings_cursor++;
            } else {
                settings_cursor = 0;
                display.NextPage();  // exits SCALE_SETTINGS → FLOCK_VIEW
            }
        }
    } else {
        // All other pages: encoder changes boid count + cycles page.
        if (inc != 0) {
#ifndef MURMUR_UI_ONLY
            int old_num = num_boids;
#endif
            num_boids += inc;
            if (num_boids < 4)  num_boids = 4;
            if (num_boids > 16) num_boids = 16;
            flock.SetNumBoids(num_boids);

#ifndef MURMUR_UI_ONLY
            if (num_boids > old_num) {
                for (int i = old_num; i < num_boids; i++) {
                    voices[i].SetActive(true);
                }
            } else {
                for (int i = num_boids; i < old_num; i++) {
                    voices[i].SetActive(false);
                }
            }
#endif
        }

        // Press: cycle display page
        if (patch.encoder.RisingEdge()) {
            display.NextPage();
        }
    }

    // === GATE INPUTS ===
    // GATE_1: Reserved (was buffer freeze, no longer needed)

    // GATE_2: Scatter flock (randomize positions)
    if (patch.gate_input[1].Trig()) {
        flock.Scatter();
    }
}

void UpdateDisplay() {
    switch (display.GetPage()) {
        case murmur::DisplayPage::FLOCK_VIEW: {
            static char chord_str[8];
            const char* chord_label = chord_prog.BuildLabel(scale_quantizer, chord_str, sizeof(chord_str))
                                      ? chord_str : nullptr;
            display.DrawFlockView(flock, boids_params, chord_label);
            break;
        }

        case murmur::DisplayPage::PARAMETERS:
            display.DrawParameters(boids_params, num_boids, morph);
            break;

        case murmur::DisplayPage::SCALE_SETTINGS:
            display.DrawScaleSettings(
                scale_quantizer.GetRoot(),
                static_cast<int>(scale_quantizer.GetScale()),
                scale_quantizer.GetBaseOctave(),
                settings_cursor,
                span_octaves,
                freq_range,
                chord_prog.GetMode(),
                chord_prog.GetIndex());
            break;

        default:
            break;
    }

    led_grid.Update();
}
