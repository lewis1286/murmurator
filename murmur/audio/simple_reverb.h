#pragma once
#ifndef SIMPLE_REVERB_H
#define SIMPLE_REVERB_H

#include <cstring>

namespace murmur {

// Schroeder reverb: 4 parallel comb filters + 2 series allpass filters (mono).
//
// Used as a shared reverb bus for z-axis distance simulation.
// Far boids (z≈0) contribute more to the reverb send; close boids (z≈1) less.
// Delay lengths chosen to be mutually prime in samples to avoid resonant peaks.
//
// Memory: ~17 KB in SRAM for the delay buffers.
class SimpleReverb {
public:
    void Init(float /*sample_rate*/) {
        memset(comb_buf_, 0, sizeof(comb_buf_));
        memset(ap_buf_,   0, sizeof(ap_buf_));
        for(int i = 0; i < 4; i++) comb_pos_[i] = 0;
        for(int i = 0; i < 2; i++) ap_pos_[i]   = 0;
    }

    // Process one mono sample; returns reverb output.
    float Process(float in) {
        float out = 0.0f;

        // 4 parallel comb filters (density / build-up)
        for(int i = 0; i < 4; i++) {
            float delayed = comb_buf_[i][comb_pos_[i]];
            comb_buf_[i][comb_pos_[i]] = in + delayed * kCombFb;
            if(++comb_pos_[i] >= kCombSize(i)) comb_pos_[i] = 0;
            out += delayed;
        }
        out *= 0.25f;  // normalize sum of 4 combs

        // 2 series allpass filters (diffusion / smearing)
        for(int i = 0; i < 2; i++) {
            float buf = ap_buf_[i][ap_pos_[i]];
            float y   = buf - kApGain * out;
            ap_buf_[i][ap_pos_[i]] = out + kApGain * buf;
            if(++ap_pos_[i] >= kApSize(i)) ap_pos_[i] = 0;
            out = y;
        }

        return out;
    }

private:
    // Feedback gain for comb filters (~0.82 gives a short, tight room)
    static constexpr float kCombFb = 0.82f;
    // Allpass coefficient (0.5 = classic Schroeder diffusion)
    static constexpr float kApGain = 0.5f;

    // Delay lengths in samples at 48 kHz (mutually prime, ~15–19 ms)
    // Using constexpr functions avoids the C++14 ODR issue with static constexpr arrays.
    static constexpr int kCombSize(int i) {
        return i == 0 ? 743 : i == 1 ? 811 : i == 2 ? 863 : 919;
    }
    // Allpass delay lengths (~4–6 ms)
    static constexpr int kApSize(int i) {
        return i == 0 ? 211 : 293;
    }

    float comb_buf_[4][919];  // ~14.7 KB
    float ap_buf_[2][293];    //  ~2.3 KB
    int   comb_pos_[4];
    int   ap_pos_[2];
};

} // namespace murmur

#endif // SIMPLE_REVERB_H
