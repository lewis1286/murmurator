#include "scheduler.h"
#include <cmath>

namespace murmur {

void BoidScheduler::Init(float sample_rate) {
    sample_rate_ = sample_rate;

    // Initialize default params
    params_.base_density = 10.0f;
    params_.pitch_range = 12.0f;
    params_.position_offset = 0.0f;
    params_.pitch_offset = 0.0f;
    params_.size_base_ms = 100.0f;
    params_.energy = 1.0f;

    // Initialize timers
    for (size_t i = 0; i < MAX_BOIDS; i++) {
        trigger_timers_[i] = 0.0f;
        trigger_intervals_[i] = sample_rate_ / params_.base_density;
        triggered_[i] = false;
    }
}

GrainParams BoidScheduler::MapBoidToGrain(const Boid& boid) const {
    GrainParams params;

    // X position -> buffer playback position (with offset)
    params.position = boid.position.x + params_.position_offset;
    // Wrap position to 0-1 range
    while (params.position < 0.0f) params.position += 1.0f;
    while (params.position >= 1.0f) params.position -= 1.0f;

    // Y position -> pitch (±pitch_range semitones, plus offset)
    float pitch_semitones = (boid.position.y - 0.5f) * 2.0f * params_.pitch_range;
    pitch_semitones += params_.pitch_offset;
    params.pitch_ratio = powf(2.0f, pitch_semitones / 12.0f);

    // Velocity magnitude -> grain size (inverse relationship)
    // Faster boids = smaller grains for more frenetic sound
    float speed = boid.velocity.Magnitude();
    float speed_factor = 1.0f - (speed * params_.energy * 10.0f);
    if (speed_factor < 0.1f) speed_factor = 0.1f;
    if (speed_factor > 1.0f) speed_factor = 1.0f;

    float size_ms = params_.size_base_ms * speed_factor;
    if (size_ms < 10.0f) size_ms = 10.0f;
    if (size_ms > 500.0f) size_ms = 500.0f;

    params.size_samples = size_ms * sample_rate_ / 1000.0f;

    // Velocity angle -> stereo pan
    float angle = boid.velocity.Angle();
    params.pan = sinf(angle);  // -1 (left) to 1 (right)

    // Amplitude based on speed (faster = quieter to prevent harshness)
    params.amplitude = 0.5f + (1.0f - speed * 5.0f) * 0.3f;
    if (params.amplitude < 0.3f) params.amplitude = 0.3f;
    if (params.amplitude > 0.8f) params.amplitude = 0.8f;

    return params;
}

void BoidScheduler::Process(const BoidsFlock& flock, GrainPool& pool,
                            const CircularBuffer& buffer) {
    size_t num_boids = flock.GetNumBoids();

    for (size_t i = 0; i < num_boids; i++) {
        const Boid& boid = flock.GetBoid(i);
        triggered_[i] = false;

        // Update trigger interval based on boid speed
        // Faster boids trigger more frequently
        float speed = boid.velocity.Magnitude();
        float speed_multiplier = 1.0f + speed * params_.energy * 20.0f;
        float rate = params_.base_density * speed_multiplier;
        if (rate < 0.5f) rate = 0.5f;
        if (rate > 100.0f) rate = 100.0f;

        trigger_intervals_[i] = sample_rate_ / rate;

        // Check timer
        trigger_timers_[i] += 1.0f;

        if (trigger_timers_[i] >= trigger_intervals_[i]) {
            trigger_timers_[i] -= trigger_intervals_[i];

            // Map boid to grain and trigger
            GrainParams grain_params = MapBoidToGrain(boid);
            pool.TriggerGrain(grain_params, buffer.GetWritePosition(), buffer.GetSize());

            triggered_[i] = true;
        }
    }

    // Clear triggered flags for inactive boids
    for (size_t i = num_boids; i < MAX_BOIDS; i++) {
        triggered_[i] = false;
    }
}

} // namespace murmur
