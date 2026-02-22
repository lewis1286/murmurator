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
    daisysp::Svf        filter;

    float gain_l;         // Left channel gain (panning * amplitude)
    float gain_r;         // Right channel gain (panning * amplitude)
    float target_freq;    // Target frequency for smoothing
    float target_amp;     // Target amplitude for smoothing
    float target_pan;     // Target pan (-1 to +1) for smoothing
    float target_z;       // Target z position (0=far, 1=close) for smoothing
    float current_freq;
    float current_amp;
    float current_pan;
    float current_z;
    float current_reverb_send;  // = current_amp: reverb follows voice amplitude, not z
    float last_sample;    // Filtered sample, cached for right channel and reverb send
    bool active;

    void Init(float sample_rate) {
        osc.Init(sample_rate);
        // Triangle wave: has odd harmonics (3rd, 5th, …) so the LPF has an
        // audible effect, unlike sinusoids which are harmonic-free.
        osc.SetWaveform(daisysp::Oscillator::WAVE_TRI);
        osc.SetFreq(440.0f);
        osc.SetAmp(1.0f);  // Amplitude handled externally via gains

        filter.Init(sample_rate);
        filter.SetRes(0.1f);    // Subtle resonance — no peak, just a smooth roll-off
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
        if (!a) {
            target_amp = 0.0f;
        }
    }

    // Call once per boid tick (500 Hz) to smooth parameters and update DSP state.
    void UpdateSmoothing() {
        constexpr float coeff_freq = 0.006f;  // slow — preserves glide in linear mode
        constexpr float coeff_amp  = 0.05f;   // faster — amplitude tracks z movement
        constexpr float coeff_pan  = 0.006f;  // slow — smooth stereo drift
        constexpr float coeff_z    = 0.05f;   // matches amp

        current_freq += (target_freq - current_freq) * coeff_freq;
        current_amp  += (target_amp  - current_amp)  * coeff_amp;
        current_pan  += (target_pan  - current_pan)  * coeff_pan;
        current_z    += (target_z    - current_z)    * coeff_z;

        osc.SetFreq(current_freq);

        // LPF cutoff: 2× the boid's fundamental at z=0 (far → dark, only fundamental
        // passes through), opening up by 7 kHz as z→1 (close → bright, full harmonics).
        // This ensures the fundamental is never filtered, only harmonics are attenuated.
        float cutoff = current_freq * 2.0f + current_z * 7000.0f;
        filter.SetFreq(cutoff);

        // Linear panning (wider stereo field than equal-power)
        float pan_norm = (current_pan + 1.0f) * 0.5f;  // 0-1 range
        gain_l = (1.0f - pan_norm) * current_amp;
        gain_r = pan_norm           * current_amp;

        // Reverb send proportional to voice amplitude (not z).
        // Far boids go quiet in reverb too — preserving full z amplitude range.
        // Budget: sum(current_amp) <= MAX_AMP_TOTAL, so reverb input stays bounded.
        current_reverb_send = current_amp;
    }

    // Process one sample — filters oscillator output. Caches result for right channel
    // and reverb send. Call ProcessLeft() before ProcessRight() and GetReverbSend().
    float ProcessLeft() {
        if (!active && current_amp < 0.001f) {
            last_sample = 0.0f;
            return 0.0f;
        }
        float raw = osc.Process();
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
