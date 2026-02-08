#include "grain_voice.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace murmur {

float GrainVoice::hann_lut_[WINDOW_LUT_SIZE] = {0};
bool GrainVoice::lut_initialized_ = false;

void GrainVoice::InitLUT() {
    if (lut_initialized_) return;

    for (size_t i = 0; i < WINDOW_LUT_SIZE; i++) {
        // Hann window: 0.5 * (1 - cos(2 * pi * n / N))
        float phase = static_cast<float>(i) / static_cast<float>(WINDOW_LUT_SIZE - 1);
        hann_lut_[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * phase));
    }

    lut_initialized_ = true;
}

void GrainVoice::Trigger(const GrainParams& params, size_t buffer_start_idx, size_t buffer_size) {
    active_ = true;

    // Calculate starting position in buffer
    float buffer_pos = params.position * static_cast<float>(buffer_size);
    start_index_ = static_cast<size_t>(buffer_pos) % buffer_size;

    // Initialize playback state
    phase_ = 0.0f;
    phase_increment_ = params.pitch_ratio;
    envelope_phase_ = 0.0f;

    // Store grain parameters
    grain_size_samples_ = static_cast<size_t>(params.size_samples);
    if (grain_size_samples_ < 2) grain_size_samples_ = 2;

    amplitude_ = params.amplitude;

    // Calculate stereo gains from pan (-1 to 1)
    // Equal power panning
    float pan_normalized = (params.pan + 1.0f) * 0.5f; // 0 to 1
    gain_l_ = cosf(pan_normalized * M_PI * 0.5f) * amplitude_;
    gain_r_ = sinf(pan_normalized * M_PI * 0.5f) * amplitude_;
}

void GrainVoice::Process(const float* buffer, size_t buffer_size,
                         float* out_l, float* out_r) {
    if (!active_) {
        *out_l = 0.0f;
        *out_r = 0.0f;
        return;
    }

    // Get envelope value
    float env_phase_normalized = envelope_phase_ / static_cast<float>(grain_size_samples_);
    float envelope = GetEnvelopeValue(env_phase_normalized);

    // Read sample with interpolation
    float read_idx = static_cast<float>(start_index_) + phase_;
    float sample = ReadBufferInterpolated(buffer, buffer_size, read_idx);

    // Apply envelope and output
    float output = sample * envelope;
    *out_l = output * gain_l_;
    *out_r = output * gain_r_;

    // Advance playback position
    phase_ += phase_increment_;
    envelope_phase_ += 1.0f;

    // Check if grain is complete
    if (envelope_phase_ >= static_cast<float>(grain_size_samples_)) {
        active_ = false;
    }
}

float GrainVoice::GetEnvelopeValue(float phase) const {
    if (phase < 0.0f || phase >= 1.0f) return 0.0f;

    // Interpolate from LUT
    float idx_f = phase * static_cast<float>(WINDOW_LUT_SIZE - 1);
    size_t idx0 = static_cast<size_t>(idx_f);
    size_t idx1 = (idx0 + 1) % WINDOW_LUT_SIZE;
    float frac = idx_f - static_cast<float>(idx0);

    return hann_lut_[idx0] * (1.0f - frac) + hann_lut_[idx1] * frac;
}

float GrainVoice::ReadBufferInterpolated(const float* buffer, size_t buffer_size,
                                          float index) const {
    // Guard against non-finite values to avoid infinite loops
    if (!std::isfinite(index)) {
        return 0.0f;
    }

    // Wrap index to buffer size
    while (index < 0.0f) index += static_cast<float>(buffer_size);
    while (index >= static_cast<float>(buffer_size)) index -= static_cast<float>(buffer_size);

    size_t idx0 = static_cast<size_t>(index) % buffer_size;
    size_t idx1 = (idx0 + 1) % buffer_size;
    float frac = index - static_cast<float>(static_cast<size_t>(index));

    // Linear interpolation
    return buffer[idx0] * (1.0f - frac) + buffer[idx1] * frac;
}

} // namespace murmur
