#pragma once
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "boids.h"
#include "../audio/grain_pool.h"
#include "../audio/circular_buffer.h"

namespace murmur {

struct SchedulerParams {
    float base_density;     // Base trigger rate in Hz (1-50)
    float pitch_range;      // Y-axis pitch range in semitones (0-24)
    float position_offset;  // CV-controllable position offset (0-1)
    float pitch_offset;     // CV-controllable pitch offset in semitones
    float size_base_ms;     // Base grain size in ms
    float energy;           // Flock energy/turbulence multiplier (0-2)
};

class BoidScheduler {
public:
    BoidScheduler() : sample_rate_(48000.0f) {}
    ~BoidScheduler() {}

    void Init(float sample_rate);
    void SetParams(const SchedulerParams& params) { params_ = params; }

    // Process one audio sample - checks timers and triggers grains
    void Process(const BoidsFlock& flock, GrainPool& pool,
                 const CircularBuffer& buffer);

    // Get trigger activity for visualization
    bool WasTriggered(size_t boid_idx) const {
        if (boid_idx >= MAX_BOIDS) return false;
        return triggered_[boid_idx];
    }

private:
    // Map a boid's state to grain parameters
    GrainParams MapBoidToGrain(const Boid& boid) const;

    float sample_rate_;
    SchedulerParams params_;

    // Per-boid trigger timers (in samples)
    float trigger_timers_[MAX_BOIDS];
    float trigger_intervals_[MAX_BOIDS];

    // Trigger state for visualization
    bool triggered_[MAX_BOIDS];
};

} // namespace murmur

#endif // SCHEDULER_H
