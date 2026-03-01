#pragma once
#ifndef AXIS_MAPPING_H
#define AXIS_MAPPING_H

#include "scale_quantizer.h"
#include "../boids/vec3.h"
#include <cstdint>

namespace murmur {

// Which audio parameter a boid axis is assigned to.
enum class Param : uint8_t { FREQ, AMP, PAN };

// Assigns each boid axis (x, y, z) to an audio parameter.
// Default matches the original hardcoded mapping: x=pan, y=freq, z=amp.
struct AxisMapping {
    Param x = Param::PAN;
    Param y = Param::FREQ;
    Param z = Param::AMP;
};

// Context data required to evaluate the per-axis formulas.
struct MappingContext {
    const ScaleQuantizer& scale;
    float freq_min;
    float freq_range;
    int   span_octaves;
    float max_amp_per_voice;
};

// Computed audio parameters for one voice.
struct VoiceParams {
    float freq;
    float amp;
    float pan;
};

// Maps a single axis value (0-1) to a specific audio parameter.
// AMP uses an amplitude floor so voices never fully silence (z=1 stays audible).
inline float MapAxisValue(float value, Param param, const MappingContext& ctx) {
    constexpr float AMP_FLOOR = 0.2f;
    switch (param) {
        case Param::FREQ:
            return ctx.scale.Quantize(value, ctx.freq_min, ctx.freq_range, ctx.span_octaves);
        case Param::AMP:
            return (AMP_FLOOR + (1.0f - value) * (1.0f - AMP_FLOOR)) * ctx.max_amp_per_voice;
        case Param::PAN:
            return value * 2.0f - 1.0f;
        default:
            return 0.0f;
    }
}

// Maps a boid position to voice parameters using the given axis assignment.
// If two axes are assigned to the same parameter, the last one (z > y > x) wins.
inline VoiceParams MapBoidToVoice(const Vec3& pos, const AxisMapping& mapping,
                                   const MappingContext& ctx) {
    VoiceParams out = {0.0f, 0.0f, 0.0f};

    auto apply = [&](float axis_val, Param p) {
        float v = MapAxisValue(axis_val, p, ctx);
        switch (p) {
            case Param::FREQ: out.freq = v; break;
            case Param::AMP:  out.amp  = v; break;
            case Param::PAN:  out.pan  = v; break;
            default: break;
        }
    };

    apply(pos.x, mapping.x);
    apply(pos.y, mapping.y);
    apply(pos.z, mapping.z);
    return out;
}

} // namespace murmur

#endif // AXIS_MAPPING_H
