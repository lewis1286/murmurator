#include "daisysp.h"
#include "daisy_patch.h"
#include "audio/circular_buffer.h"
#include "audio/grain_pool.h"
#include "boids/boids.h"
#include "boids/scheduler.h"
#include "ui/display.h"
#include "ui/led_grid.h"
#include <string>

using namespace daisy;
using namespace daisysp;

// Hardware
DaisyPatch patch;

// Audio
murmur::CircularBuffer buffer;
murmur::GrainPool grain_pool;

// Boids
murmur::BoidsFlock flock;
murmur::BoidsParams boids_params;
murmur::BoidScheduler scheduler;
murmur::SchedulerParams sched_params;

// UI
murmur::Display display;
murmur::LedGrid led_grid;

// Control parameters (knobs)
float separation_weight = 1.0f;   // CTRL_1
float cohesion_weight = 1.0f;     // CTRL_2
float grain_density = 10.0f;      // CTRL_3
float pitch_range = 12.0f;        // CTRL_4

// CV inputs
float cv_position_offset = 0.0f;  // CV_1
float cv_energy = 1.0f;           // CV_2
float cv_grain_size = 100.0f;     // CV_3
float cv_pitch_offset = 0.0f;     // CV_4

// State
int num_boids = 8;
float sample_rate = 48000.0f;

// Timing
uint32_t last_display_update = 0;
uint32_t last_boids_update = 0;
constexpr uint32_t DISPLAY_UPDATE_MS = 33;
constexpr uint32_t BOIDS_UPDATE_MS = 16;

// Audio processing buffers
float grain_out_l[48];  // Max block size
float grain_out_r[48];

void UpdateControls();
void UpdateDisplay();

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size) {
    // Update scheduler parameters from controls
    sched_params.base_density = grain_density;
    sched_params.pitch_range = pitch_range;
    sched_params.position_offset = cv_position_offset;
    sched_params.pitch_offset = cv_pitch_offset;
    sched_params.size_base_ms = cv_grain_size;
    sched_params.energy = cv_energy;
    scheduler.SetParams(sched_params);

    for (size_t i = 0; i < size; i++) {
        // Mix input to mono
        float dry_in = (in[0][i] + in[1][i]) * 0.5f;

        // Write to buffer
        buffer.Write(dry_in);

        // Process scheduler (triggers grains based on boid states)
        scheduler.Process(flock, grain_pool, buffer);

        // Process grain pool (single sample)
        float gl = 0.0f, gr = 0.0f;
        grain_pool.Process(murmur::audio_buffer, murmur::BUFFER_SIZE,
                          &gl, &gr, 1);

        // Mix dry/wet
        float mix = 0.5f;
        out[0][i] = dry_in * (1.0f - mix) + gl * mix;
        out[1][i] = dry_in * (1.0f - mix) + gr * mix;
        out[2][i] = in[2][i];
        out[3][i] = in[3][i];
    }
}

int main(void) {
    patch.Init();
    buffer.Init();
    sample_rate = patch.AudioSampleRate();

    // Initialize audio
    grain_pool.Init();

    // Initialize boids
    flock.Init(num_boids);
    boids_params.separation_weight = separation_weight;
    boids_params.alignment_weight = 1.0f;  // Fixed at 1.0 for natural behavior
    boids_params.cohesion_weight = cohesion_weight;
    boids_params.perception_radius = 0.25f;
    boids_params.max_speed = 0.1f;
    boids_params.max_force = 0.005f;

    // Initialize scheduler
    scheduler.Init(sample_rate);

    // Initialize UI
    display.Init(&patch);
    led_grid.Init(&patch);

    patch.StartAdc();
    patch.StartAudio(AudioCallback);

    while (1) {
        UpdateControls();

        uint32_t now = System::GetNow();

        // Update boids simulation
        if (now - last_boids_update >= BOIDS_UPDATE_MS) {
            float dt = static_cast<float>(now - last_boids_update) / 1000.0f;
            flock.Update(dt, boids_params);
            led_grid.UpdateFromFlock(flock);
            last_boids_update = now;
        }

        // Update display
        if (now - last_display_update >= DISPLAY_UPDATE_MS) {
            UpdateDisplay();
            last_display_update = now;
        }
    }
}

void UpdateControls() {
    patch.ProcessAnalogControls();
    patch.ProcessDigitalControls();

    // === KNOBS ===
    // CTRL_1: Separation weight (0-2)
    // Spreads grains across buffer/pitch
    separation_weight = patch.GetKnobValue(DaisyPatch::CTRL_1) * 2.0f;
    boids_params.separation_weight = separation_weight;

    // CTRL_2: Cohesion weight (0-2)
    // Clusters grains together
    cohesion_weight = patch.GetKnobValue(DaisyPatch::CTRL_2) * 2.0f;
    boids_params.cohesion_weight = cohesion_weight;

    // CTRL_3: Grain density (1-50 Hz base trigger rate)
    grain_density = 1.0f + patch.GetKnobValue(DaisyPatch::CTRL_3) * 49.0f;

    // CTRL_4: Pitch range (0-24 semitones)
    pitch_range = patch.GetKnobValue(DaisyPatch::CTRL_4) * 24.0f;

    // Note: CV inputs on Daisy Patch are normalled to knobs
    // Additional CV modulation can be added via the controls array
    // For now, set reasonable defaults for scheduler params not on knobs
    cv_position_offset = 0.0f;  // Could add CV modulation here
    cv_energy = 1.0f;           // Fixed energy for now
    cv_grain_size = 80.0f;      // Fixed grain size base
    cv_pitch_offset = 0.0f;     // No pitch offset

    // === ENCODER ===
    // Rotation: Number of boids (4-16)
    int inc = patch.encoder.Increment();
    if (inc != 0) {
        num_boids += inc;
        if (num_boids < 4) num_boids = 4;
        if (num_boids > 16) num_boids = 16;
        flock.SetNumBoids(num_boids);
    }

    // Press: Cycle display page OR toggle recording
    if (patch.encoder.RisingEdge()) {
        display.NextPage();
    }

    // Long press (held): Toggle recording
    if (patch.encoder.TimeHeldMs() > 500 && patch.encoder.FallingEdge()) {
        buffer.SetRecording(!buffer.IsRecording());
    }

    // === GATE INPUTS ===
    // GATE_1: Freeze buffer (toggle recording)
    if (patch.gate_input[0].Trig()) {
        buffer.SetRecording(!buffer.IsRecording());
    }

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
            display.DrawParameters(boids_params, num_boids, buffer.IsRecording());
            break;

        case murmur::DisplayPage::WAVEFORM:
            display.DrawWaveform(murmur::audio_buffer, murmur::BUFFER_SIZE,
                                buffer.GetWritePosition());
            break;

        default:
            break;
    }

    led_grid.Update();
}
