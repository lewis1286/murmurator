#include "grain_pool.h"

namespace murmur {

void GrainPool::Init() {
    GrainVoice::InitLUT();

    active_count_ = 0;
    next_voice_ = 0;
    global_age_ = 0;

    for (size_t i = 0; i < MAX_GRAINS; i++) {
        voice_ages_[i] = 0;
    }
}

size_t GrainPool::FindVoice() {
    // First, try to find an inactive voice using round-robin
    for (size_t i = 0; i < MAX_GRAINS; i++) {
        size_t idx = (next_voice_ + i) % MAX_GRAINS;
        if (!voices_[idx].IsActive()) {
            next_voice_ = (idx + 1) % MAX_GRAINS;
            return idx;
        }
    }

    // All voices are active - steal the oldest one
    size_t oldest_idx = 0;
    uint32_t oldest_age = global_age_ - voice_ages_[0];

    for (size_t i = 1; i < MAX_GRAINS; i++) {
        uint32_t age = global_age_ - voice_ages_[i];
        if (age > oldest_age) {
            oldest_age = age;
            oldest_idx = i;
        }
    }

    next_voice_ = (oldest_idx + 1) % MAX_GRAINS;
    return oldest_idx;
}

int GrainPool::TriggerGrain(const GrainParams& params, size_t buffer_write_pos, size_t buffer_size) {
    size_t voice_idx = FindVoice();

    voices_[voice_idx].Trigger(params, buffer_write_pos, buffer_size);
    voice_ages_[voice_idx] = global_age_++;

    return static_cast<int>(voice_idx);
}

void GrainPool::Process(const float* buffer, size_t buffer_size,
                        float* out_l, float* out_r, size_t num_samples) {
    // Clear output buffers
    for (size_t i = 0; i < num_samples; i++) {
        out_l[i] = 0.0f;
        out_r[i] = 0.0f;
    }

    // Process each sample
    active_count_ = 0;

    for (size_t i = 0; i < num_samples; i++) {
        float sample_l = 0.0f;
        float sample_r = 0.0f;

        // Sum output from all active voices
        for (size_t v = 0; v < MAX_GRAINS; v++) {
            float gl = 0.0f, gr = 0.0f;
            voices_[v].Process(buffer, buffer_size, &gl, &gr);
            sample_l += gl;
            sample_r += gr;
        }

        out_l[i] = sample_l;
        out_r[i] = sample_r;
    }

    // Count active voices
    for (size_t v = 0; v < MAX_GRAINS; v++) {
        if (voices_[v].IsActive()) {
            active_count_++;
        }
    }
}

} // namespace murmur
