#pragma once
#ifndef GRAIN_VOICE_H
#define GRAIN_VOICE_H

#include <cstdint>
#include <cstddef>
#include <cmath>

namespace murmur {

// Precomputed Hann window lookup table
constexpr size_t WINDOW_LUT_SIZE = 512;

struct GrainParams {
    float position;      // 0-1 position in buffer
    float size_samples;  // grain size in samples
    float pitch_ratio;   // 1.0 = original pitch, 2.0 = octave up
    float pan;           // -1 to 1 (left to right)
    float amplitude;     // 0-1
};

class GrainVoice {
public:
    GrainVoice() : active_(false), phase_(0), envelope_phase_(0) {}
    ~GrainVoice() {}

    static void InitLUT();

    void Trigger(const GrainParams& params, size_t buffer_start_idx, size_t buffer_size);

    void Process(const float* buffer, size_t buffer_size,
                 float* out_l, float* out_r);

    bool IsActive() const { return active_; }
    float GetProgress() const {
        return active_ ? envelope_phase_ / static_cast<float>(grain_size_samples_) : 0.0f;
    }

private:
    float GetEnvelopeValue(float phase) const;
    float ReadBufferInterpolated(const float* buffer, size_t buffer_size, float index) const;

    bool active_;

    // Playback state
    float phase_;           // Current position in buffer (fractional samples)
    float phase_increment_; // How much to advance per sample (pitch ratio)
    float envelope_phase_;  // Current position in envelope (0 to grain_size_samples)

    // Grain parameters
    size_t start_index_;         // Starting index in buffer
    size_t grain_size_samples_;  // Total grain duration in samples
    float amplitude_;
    float gain_l_;
    float gain_r_;

    // LUT for Hann window
    static float hann_lut_[WINDOW_LUT_SIZE];
    static bool lut_initialized_;
};

} // namespace murmur

#endif // GRAIN_VOICE_H
