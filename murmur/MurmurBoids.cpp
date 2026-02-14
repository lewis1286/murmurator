#include "daisysp.h"
#include "daisy_patch.h"
#include "audio/osc_voice.h"
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

// Boids
murmur::BoidsFlock flock;
murmur::BoidsParams boids_params;

// UI
murmur::Display display;
murmur::LedGrid led_grid;

// Control parameters
float separation_weight = 1.0f;   // CTRL_1
float cohesion_weight = 1.0f;     // CTRL_2
float freq_range = 600.0f;        // CTRL_3: frequency spread in Hz
float alignment_weight = 1.0f;    // CTRL_4

// Audio parameters
constexpr float FREQ_MIN = 200.0f;
constexpr float FREQ_MAX = 800.0f;
constexpr float MAX_AMP_TOTAL = 0.8f;  // Total max amplitude across all voices

// State
int num_boids = 8;
float sample_rate = 48000.0f;

// Waveform display buffer (captured from audio output)
constexpr size_t WAVE_DISPLAY_SIZE = 128;
float wave_display[WAVE_DISPLAY_SIZE];
size_t wave_display_pos = 0;
size_t wave_display_decimation = 0;

// Timing
uint32_t last_display_update = 0;
uint32_t last_boids_update = 0;
constexpr uint32_t DISPLAY_UPDATE_MS = 33;
constexpr uint32_t BOIDS_UPDATE_MS = 16;

void UpdateControls();
void UpdateDisplay();
void UpdateVoicesFromBoids();

#ifndef MURMUR_UI_ONLY
static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size) {
    for (size_t i = 0; i < size; i++) {
        float sum_l = 0.0f;
        float sum_r = 0.0f;

        for (size_t v = 0; v < static_cast<size_t>(num_boids); v++) {
            sum_l += voices[v].ProcessLeft();
            sum_r += voices[v].ProcessRight();
        }

        out[0][i] = sum_l;
        out[1][i] = sum_r;
        out[2][i] = in[2][i];
        out[3][i] = in[3][i];

        // Capture samples for waveform display (decimated)
        wave_display_decimation++;
        if (wave_display_decimation >= 8) {
            wave_display_decimation = 0;
            wave_display[wave_display_pos] = (sum_l + sum_r) * 0.5f;
            wave_display_pos = (wave_display_pos + 1) % WAVE_DISPLAY_SIZE;
        }
    }
}
#endif

int main(void) {
    patch.Init();
    sample_rate = patch.AudioSampleRate();

    // Initialize oscillator voices
#ifndef MURMUR_UI_ONLY
    for (size_t i = 0; i < murmur::MAX_BOIDS; i++) {
        voices[i].Init(sample_rate);
    }
#endif

    // Initialize boids
    flock.Init(num_boids);
    boids_params.separation_weight = separation_weight;
    boids_params.alignment_weight = alignment_weight;
    boids_params.cohesion_weight = cohesion_weight;
    boids_params.perception_radius = 0.25f;
    boids_params.max_speed = 0.1f;
    boids_params.max_force = 0.005f;

    // Initialize UI
    display.Init(&patch);
    led_grid.Init(&patch);

    // Clear waveform display buffer
    for (size_t i = 0; i < WAVE_DISPLAY_SIZE; i++) {
        wave_display[i] = 0.0f;
    }

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

        // Update boids simulation
        if (now - last_boids_update >= BOIDS_UPDATE_MS) {
            float dt = static_cast<float>(now - last_boids_update) / 1000.0f;
            flock.Update(dt, boids_params);
            led_grid.UpdateFromFlock(flock);

#ifndef MURMUR_UI_ONLY
            UpdateVoicesFromBoids();
#endif

            last_boids_update = now;
        }

        // Update display
        if (now - last_display_update >= DISPLAY_UPDATE_MS) {
            UpdateDisplay();
            last_display_update = now;
        }
    }
}

void UpdateVoicesFromBoids() {
    float max_amp_per_voice = MAX_AMP_TOTAL / static_cast<float>(num_boids);

    for (int i = 0; i < num_boids; i++) {
        const murmur::Boid& boid = flock.GetBoid(i);

        // y -> frequency
        float freq = FREQ_MIN + boid.position.y * freq_range;

        // z -> amplitude
        float amp = boid.position.z * max_amp_per_voice;

        // x -> pan (-1 to +1)
        float pan = boid.position.x * 2.0f - 1.0f;

        voices[i].SetParams(freq, amp, pan);
        voices[i].UpdateSmoothing();
    }
}

void UpdateControls() {
    patch.ProcessAnalogControls();
    patch.ProcessDigitalControls();

    // === KNOBS ===
    // CTRL_1: Separation weight (0-2)
    separation_weight = patch.GetKnobValue(DaisyPatch::CTRL_1) * 2.0f;
    boids_params.separation_weight = separation_weight;

    // CTRL_2: Cohesion weight (0-2)
    cohesion_weight = patch.GetKnobValue(DaisyPatch::CTRL_2) * 2.0f;
    boids_params.cohesion_weight = cohesion_weight;

    // CTRL_3: Frequency range (50-800 Hz spread)
    freq_range = 50.0f + patch.GetKnobValue(DaisyPatch::CTRL_3) * 750.0f;

    // CTRL_4: Alignment weight (0-2)
    alignment_weight = patch.GetKnobValue(DaisyPatch::CTRL_4) * 2.0f;
    boids_params.alignment_weight = alignment_weight;

    // === ENCODER ===
    // Rotation: Number of boids (4-16)
    int inc = patch.encoder.Increment();
    if (inc != 0) {
#ifndef MURMUR_UI_ONLY
        int old_num = num_boids;
#endif
        num_boids += inc;
        if (num_boids < 4) num_boids = 4;
        if (num_boids > 16) num_boids = 16;
        flock.SetNumBoids(num_boids);

#ifndef MURMUR_UI_ONLY
        // Activate/deactivate voices as needed
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

    // Press: Cycle display page
    if (patch.encoder.RisingEdge()) {
        display.NextPage();
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
        case murmur::DisplayPage::FLOCK_VIEW:
            display.DrawFlockView(flock, boids_params);
            break;

        case murmur::DisplayPage::PARAMETERS:
            display.DrawParameters(boids_params, num_boids, freq_range);
            break;

        case murmur::DisplayPage::WAVEFORM:
            display.DrawWaveform(wave_display, WAVE_DISPLAY_SIZE);
            break;

        default:
            break;
    }

    led_grid.Update();
}
