#pragma once
#ifndef CHORD_PROGRESSION_H
#define CHORD_PROGRESSION_H

#include "scale_quantizer.h"
#include <cstdint>
#include <cstddef>
#include <cstdio>

namespace murmur {

// Manages the I→IV→V→I chord progression timer and chord offset state.
// Owns the chord data tables, mode/index state, and timing logic that
// was previously scattered as globals in MurmurBoids.cpp.
class ChordProgression {
public:
    ChordProgression() : mode_(0), index_(0), last_change_ms_(0) {}

    // Call once per main-loop tick.
    // Advances the chord index on timer expiry and updates sq's chord offset.
    void Update(uint32_t now, ScaleQuantizer& sq) {
        if (mode_ == 0 || sq.GetScale() == ScaleType::OFF) return;
        constexpr uint32_t kIntervals[3] = {0, 10000, 15000};
        if (now - last_change_ms_ >= kIntervals[mode_]) {
            index_ = (index_ + 1) % 4;
            sq.SetChordOffset(Offset(index_));
            last_change_ms_ = now;
        }
    }

    // Encoder delta: cycles mode OFF → 10s → 15s → OFF.
    // Resets chord to I and clears the offset when turned off.
    void Increment(int delta, uint32_t now, ScaleQuantizer& sq) {
        mode_ = ((mode_ + delta) % 3 + 3) % 3;
        if (mode_ == 0) {
            index_ = 0;
            sq.SetChordOffset(0);
        } else {
            last_change_ms_ = now;
        }
    }

    // Builds the flock-view label (e.g. "IV:D#") into buf.
    // Returns true and fills buf when the progression is active and scale != OFF.
    // Returns false when inactive — caller should treat label as nullptr.
    bool BuildLabel(const ScaleQuantizer& sq, char* buf, size_t len) const {
        if (mode_ == 0 || sq.GetScale() == ScaleType::OFF) return false;
        static const char* const kNoteNames[12] = {
            "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
        int root = (sq.GetRoot() + Offset(index_)) % 12;
        snprintf(buf, len, "%s:%s", Name(index_), kNoteNames[root]);
        return true;
    }

    int GetMode()  const { return mode_; }
    int GetIndex() const { return index_; }

private:
    int      mode_;           // 0=OFF, 1=10s interval, 2=15s interval
    int      index_;          // position in the I/IV/V/I cycle (0-3)
    uint32_t last_change_ms_;

    static int Offset(int i) {
        constexpr int kOffsets[4] = {0, 5, 7, 0};
        return kOffsets[i];
    }

    static const char* Name(int i) {
        static const char* const kNames[4] = {"I", "IV", "V", "I"};
        return kNames[i];
    }
};

} // namespace murmur

#endif // CHORD_PROGRESSION_H
