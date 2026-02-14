#pragma once
#ifndef OSC_VOICE_H
#define OSC_VOICE_H

#include "daisysp.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace murmur {

struct OscVoice {
    daisysp::Oscillator osc;
    float gain_l;         // Left channel gain (panning * amplitude)
    float gain_r;         // Right channel gain (panning * amplitude)
    float target_freq;    // Target frequency for smoothing
    float target_amp;     // Target amplitude for smoothing
    float target_pan;     // Target pan (-1 to +1) for smoothing
    float current_freq;
    float current_amp;
    float current_pan;
    float last_sample;    // Cached for stereo processing
    bool active;

    void Init(float sample_rate) {
        osc.Init(sample_rate);
        osc.SetWaveform(daisysp::Oscillator::WAVE_SIN);
        osc.SetFreq(440.0f);
        osc.SetAmp(1.0f);  // We handle amplitude externally via gains
        gain_l = 0.0f;
        gain_r = 0.0f;
        target_freq = 440.0f;
        target_amp = 0.0f;
        target_pan = 0.0f;
        current_freq = 440.0f;
        current_amp = 0.0f;
        current_pan = 0.0f;
        last_sample = 0.0f;
        active = false;
    }

    void SetParams(float freq, float amp, float pan) {
        target_freq = freq;
        target_amp = amp;
        target_pan = pan;
    }

    void SetActive(bool a) {
        active = a;
        if (!a) {
            target_amp = 0.0f;
        }
    }

    // Call once per audio block to smooth parameters (control rate)
    void UpdateSmoothing() {
        // One-pole smoothing (scaled for 500Hz boids update rate)
        constexpr float coeff = 0.006f;

        current_freq += (target_freq - current_freq) * coeff;
        current_amp += (target_amp - current_amp) * coeff;
        current_pan += (target_pan - current_pan) * coeff;

        osc.SetFreq(current_freq);

        // Linear panning (wider stereo field than equal-power)
        float pan_norm = (current_pan + 1.0f) * 0.5f;  // 0-1 range
        gain_l = (1.0f - pan_norm) * current_amp;
        gain_r = pan_norm * current_amp;
    }

    // Process one sample, outputs left channel
    float ProcessLeft() {
        if (!active && current_amp < 0.001f) {
            last_sample = 0.0f;
            return 0.0f;
        }
        last_sample = osc.Process();
        return last_sample * gain_l;
    }

    // Uses cached sample for right channel
    float ProcessRight() {
        return last_sample * gain_r;
    }
};

} // namespace murmur

#endif // OSC_VOICE_H
