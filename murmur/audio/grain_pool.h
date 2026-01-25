#pragma once
#ifndef GRAIN_POOL_H
#define GRAIN_POOL_H

#include "grain_voice.h"
#include <cstddef>

namespace murmur {

constexpr size_t MAX_GRAINS = 16;

class GrainPool {
public:
    GrainPool() : active_count_(0), next_voice_(0) {}
    ~GrainPool() {}

    void Init();

    // Trigger a new grain with given parameters
    // Returns the voice index used, or -1 if failed
    int TriggerGrain(const GrainParams& params, size_t buffer_write_pos, size_t buffer_size);

    // Process all active grains and output to stereo buffers
    void Process(const float* buffer, size_t buffer_size,
                 float* out_l, float* out_r, size_t num_samples);

    // Get number of currently active grains
    size_t GetActiveCount() const { return active_count_; }

    // Check if a specific voice is active
    bool IsVoiceActive(size_t voice_idx) const {
        if (voice_idx >= MAX_GRAINS) return false;
        return voices_[voice_idx].IsActive();
    }

    // Get voice progress for visualization
    float GetVoiceProgress(size_t voice_idx) const {
        if (voice_idx >= MAX_GRAINS) return 0.0f;
        return voices_[voice_idx].GetProgress();
    }

private:
    // Find the best voice to use (oldest active or first inactive)
    size_t FindVoice();

    GrainVoice voices_[MAX_GRAINS];
    size_t active_count_;
    size_t next_voice_;  // Round-robin index

    // Age tracking for voice stealing
    uint32_t voice_ages_[MAX_GRAINS];
    uint32_t global_age_;
};

} // namespace murmur

#endif // GRAIN_POOL_H
