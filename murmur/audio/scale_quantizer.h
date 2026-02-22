#pragma once
#ifndef SCALE_QUANTIZER_H
#define SCALE_QUANTIZER_H

#include <cmath>
#include <cstdint>

namespace murmur {

enum class ScaleType : uint8_t {
    OFF            = 0,
    MAJOR          = 1,
    NATURAL_MINOR  = 2,
    DORIAN         = 3,
    PENTATONIC_MAJ = 4,
    PENTATONIC_MIN = 5,
    LYDIAN         = 6,
    MIXOLYDIAN     = 7,
    COUNT          = 8
};

class ScaleQuantizer {
public:
    // Default: root=A (9), scale=OFF, octave=3
    ScaleQuantizer() : root_(9), scale_(ScaleType::OFF), base_octave_(3) {}

    void SetRoot(int root) {
        if (root < 0)  root = 0;
        if (root > 11) root = 11;
        root_ = root;
    }

    void SetScale(ScaleType scale) { scale_ = scale; }

    void SetBaseOctave(int oct) {
        if (oct < 1) oct = 1;
        if (oct > 5) oct = 5;
        base_octave_ = oct;
    }

    int       GetRoot()       const { return root_; }
    ScaleType GetScale()      const { return scale_; }
    int       GetBaseOctave() const { return base_octave_; }

    // Quantize y (0-1) to a frequency.
    // ScaleType::OFF: linear mapping (current behaviour).
    // Otherwise: snap to nearest scale degree across span_octaves octaves.
    float Quantize(float y, float freq_min, float freq_range, int span_octaves) const {
        if (scale_ == ScaleType::OFF) {
            return freq_min + y * freq_range;
        }

        // Clamp y
        if (y < 0.0f) y = 0.0f;
        if (y > 1.0f) y = 1.0f;

        int n_notes = 0;
        const int* intervals = GetIntervals(scale_, n_notes);

        int total_notes = n_notes * span_octaves;

        int degree = static_cast<int>(y * static_cast<float>(total_notes));
        if (degree >= total_notes) degree = total_notes - 1;

        int octave_offset = degree / n_notes;
        int scale_degree  = degree % n_notes;

        // MIDI note: C0=12, C1=24, C2=36, C3=48, C4=60, A4=69
        int midi = 12 + 12 * base_octave_ + root_
                 + octave_offset * 12 + intervals[scale_degree];

        return 440.0f * powf(2.0f, static_cast<float>(midi - 69) / 12.0f);
    }

private:
    int       root_;
    ScaleType scale_;
    int       base_octave_;

    // Function-local statics: defined once per TU, no ODR issues in header.
    static const int* GetIntervals(ScaleType scale, int& n_notes) {
        static const int major[]      = {0, 2, 4, 5, 7, 9, 11};
        static const int nat_minor[]  = {0, 2, 3, 5, 7, 8, 10};
        static const int dorian[]     = {0, 2, 3, 5, 7, 9, 10};
        static const int pent_maj[]   = {0, 2, 4, 7, 9};
        static const int pent_min[]   = {0, 3, 5, 7, 10};
        static const int lydian[]     = {0, 2, 4, 6, 7, 9, 11};
        static const int mixolydian[] = {0, 2, 4, 5, 7, 9, 10};

        switch (scale) {
            case ScaleType::MAJOR:          n_notes = 7; return major;
            case ScaleType::NATURAL_MINOR:  n_notes = 7; return nat_minor;
            case ScaleType::DORIAN:         n_notes = 7; return dorian;
            case ScaleType::PENTATONIC_MAJ: n_notes = 5; return pent_maj;
            case ScaleType::PENTATONIC_MIN: n_notes = 5; return pent_min;
            case ScaleType::LYDIAN:         n_notes = 7; return lydian;
            case ScaleType::MIXOLYDIAN:     n_notes = 7; return mixolydian;
            default:                        n_notes = 7; return major;
        }
    }
};

} // namespace murmur

#endif // SCALE_QUANTIZER_H
