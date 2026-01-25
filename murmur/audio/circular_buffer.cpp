#include "circular_buffer.h"
#include "daisy_patch.h"

namespace murmur {

// Allocate 4-second buffer in SDRAM
float DSY_SDRAM_BSS audio_buffer[BUFFER_SIZE];

void CircularBuffer::Init() {
    write_pos_ = 0;
    buffer_filled_ = false;
    recording_ = true;

    // Clear buffer
    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        audio_buffer[i] = 0.0f;
    }
}

void CircularBuffer::Write(float sample) {
    if (!recording_) return;

    audio_buffer[write_pos_] = sample;
    write_pos_++;

    if (write_pos_ >= BUFFER_SIZE) {
        write_pos_ = 0;
        buffer_filled_ = true;
    }
}

float CircularBuffer::ReadLinear(float position) const {
    // Position is 0.0 to 1.0, representing position in buffer
    // We read relative to current write position (looking back in time)

    float samples_back = position * static_cast<float>(BUFFER_SIZE - 1);
    int32_t base_idx = static_cast<int32_t>(write_pos_) - static_cast<int32_t>(samples_back) - 1;
    float frac = samples_back - static_cast<int32_t>(samples_back);

    size_t idx0 = WrapIndex(base_idx);
    size_t idx1 = WrapIndex(base_idx - 1);

    // Linear interpolation
    return audio_buffer[idx0] * (1.0f - frac) + audio_buffer[idx1] * frac;
}

float CircularBuffer::ReadNearest(size_t position) const {
    return audio_buffer[position % BUFFER_SIZE];
}

size_t CircularBuffer::WrapIndex(int32_t index) const {
    while (index < 0) {
        index += BUFFER_SIZE;
    }
    return static_cast<size_t>(index) % BUFFER_SIZE;
}

} // namespace murmur
