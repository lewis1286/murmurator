#pragma once
#ifndef OSC_VOICE_H
#define OSC_VOICE_H

#include "daisysp.h"
#include <cmath>

namespace murmur {

struct OscVoice {
    daisysp::Svf filter;

    float phase_;       // 0-1 phase accumulator
    float phase_inc_;   // frequency / sample_rate (phase advance per sample)
    float sample_rate_;
    float morph_;       // 0=sine, 1=triangle, 2=square (continuous blend)

    float gain_l;
    float gain_r;
    float target_freq;
    float target_amp;
    float target_pan;
    float target_z;
    float current_freq;
    float current_amp;
    float current_pan;
    float current_z;
    float current_reverb_send;
    float last_sample;
    bool active;

    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        phase_       = 0.0f;
        phase_inc_   = 440.0f / sample_rate;
        morph_       = 1.0f;  // default: triangle

        filter.Init(sample_rate);
        filter.SetRes(0.1f);
        filter.SetDrive(0.0f);

        gain_l = 0.0f;
        gain_r = 0.0f;
        target_freq  = 440.0f;
        target_amp   = 0.0f;
        target_pan   = 0.0f;
        target_z     = 0.5f;
        current_freq = 440.0f;
        current_amp  = 0.0f;
        current_pan  = 0.0f;
        current_z    = 0.5f;
        current_reverb_send = 0.0f;
        last_sample  = 0.0f;
        active = false;
    }

    void SetParams(float freq, float amp, float pan, float z) {
        target_freq = freq;
        target_amp  = amp;
        target_pan  = pan;
        target_z    = z;
    }

    // Bypass freq smoothing: jump immediately to target (for scale-quantized mode).
    void SnapFreq(float freq) {
        target_freq  = freq;
        current_freq = freq;
    }

    void SetActive(bool a) {
        active = a;
        if (!a) target_amp = 0.0f;
    }

    // morph: 0=sine, 1=triangle, 2=square. Continuous blend between adjacent shapes.
    void SetMorph(float morph) {
        morph_ = morph < 0.0f ? 0.0f : (morph > 2.0f ? 2.0f : morph);
    }

    // Call once per boid tick (500 Hz) to smooth parameters and update DSP state.
    void UpdateSmoothing() {
        constexpr float coeff_freq = 0.006f;
        constexpr float coeff_amp  = 0.05f;
        constexpr float coeff_pan  = 0.006f;
        constexpr float coeff_z    = 0.05f;

        current_freq += (target_freq - current_freq) * coeff_freq;
        current_amp  += (target_amp  - current_amp)  * coeff_amp;
        current_pan  += (target_pan  - current_pan)  * coeff_pan;
        current_z    += (target_z    - current_z)    * coeff_z;

        phase_inc_ = current_freq / sample_rate_;

        // LPF cutoff: opens up from dark (2x fundamental at z=0) to bright (7kHz+ at z=1).
        // Keeps the fundamental intact; only filters harmonics — effective for tri and square.
        float cutoff = current_freq * 2.0f + current_z * 7000.0f;
        filter.SetFreq(cutoff);

        float pan_norm = (current_pan + 1.0f) * 0.5f;
        gain_l = (1.0f - pan_norm) * current_amp;
        gain_r = pan_norm           * current_amp;

        current_reverb_send = current_amp;
    }

    // Process one sample. Caches result for ProcessRight() and GetReverbSend().
    // Call ProcessLeft() before ProcessRight().
    float ProcessLeft() {
        if (!active && current_amp < 0.001f) {
            last_sample = 0.0f;
            return 0.0f;
        }

        // Advance phase accumulator
        phase_ += phase_inc_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;

        // All three waveforms computed from the same phase — guaranteed phase-lock.
        // sine: pure fundamental; tri: odd harmonics at 1/n²; square: odd harmonics at 1/n
        float sine = sinf(phase_ * 6.28318530f);
        float tri  = 1.0f - 4.0f * fabsf(phase_ - 0.5f);
        float sq   = phase_ < 0.5f ? 1.0f : -1.0f;

        // Morph: 0-1 blends sine→triangle, 1-2 blends triangle→square
        float raw;
        if (morph_ <= 1.0f) {
            raw = sine * (1.0f - morph_) + tri * morph_;
        } else {
            float t = morph_ - 1.0f;
            raw = tri * (1.0f - t) + sq * t;
        }

        filter.Process(raw);
        last_sample = filter.Low();
        return last_sample * gain_l;
    }

    // Uses cached (filtered) sample — must be called after ProcessLeft().
    float ProcessRight() {
        return last_sample * gain_r;
    }

    // Returns this voice's contribution to the shared reverb bus.
    // Must be called after ProcessLeft().
    float GetReverbSend() const {
        return last_sample * current_reverb_send;
    }
};

} // namespace murmur

#endif // OSC_VOICE_H
