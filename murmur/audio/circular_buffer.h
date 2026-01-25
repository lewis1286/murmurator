#pragma once
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <cstdint>
#include <cstddef>

namespace murmur {

// 4 seconds at 48kHz = 192000 samples
constexpr size_t BUFFER_SIZE = 192000;

class CircularBuffer {
public:
    CircularBuffer() : write_pos_(0), buffer_filled_(false) {}
    ~CircularBuffer() {}

    void Init();
    void Write(float sample);
    float ReadLinear(float position) const;
    float ReadNearest(size_t position) const;

    size_t GetWritePosition() const { return write_pos_; }
    size_t GetSize() const { return BUFFER_SIZE; }
    bool IsFilled() const { return buffer_filled_; }

    void SetRecording(bool recording) { recording_ = recording; }
    bool IsRecording() const { return recording_; }

private:
    size_t WrapIndex(int32_t index) const;

    size_t write_pos_;
    bool buffer_filled_;
    bool recording_;
};

// Declare the buffer (defined in cpp file, allocated in SDRAM)
extern float audio_buffer[BUFFER_SIZE];

} // namespace murmur

#endif // CIRCULAR_BUFFER_H
