#pragma once
#ifndef BOIDS_H
#define BOIDS_H

#include "vec3.h"
#include <cstdint>
#include <cstddef>

namespace murmur {

constexpr size_t MAX_BOIDS = 16;
constexpr size_t LED_GRID_DIM = 4;  // 4x4 LED grid for density visualization

// Boundary avoidance constants
constexpr float BOUNDARY_MARGIN_XY  = 0.2f;   // 10% margin on x and y edges
constexpr float BOUNDARY_MARGIN_Z_LO = 0.10f;  // 5% margin at z=0 (allow near-silence)
constexpr float BOUNDARY_MARGIN_Z_HI = 0.15f;  // 15% margin at z=1 (avoid sustained max amp)
constexpr float BOUNDARY_FORCE_XY   = 0.16f;   // 4x max_force for x/y boundaries
constexpr float BOUNDARY_FORCE_Z    = 0.08f;   // 2x max_force for z boundaries

struct Boid {
    Vec3 position;      // 0-1 range for all axes (x=pan, y=freq, z=amp)
    Vec3 velocity;
    Vec3 acceleration;

    void ApplyForce(const Vec3& force) {
        acceleration += force;
    }
};

struct BoidsParams {
    float separation_weight;  // 0-2
    float alignment_weight;   // 0-2
    float cohesion_weight;    // 0-2
    float perception_radius;  // 0.05-0.5
    float max_speed;          // Maximum velocity magnitude
    float max_force;          // Maximum steering force
};

class BoidsFlock {
public:
    BoidsFlock() : num_boids_(0), initialized_(false) {}
    ~BoidsFlock() {}

    void Init(size_t num_boids);
    void Update(float dt, const BoidsParams& params);
    void Scatter();  // Randomize positions

    void SetNumBoids(size_t num);
    size_t GetNumBoids() const { return num_boids_; }
    const Boid& GetBoid(size_t index) const { return boids_[index]; }

    // Get boid density in a grid cell (for LED visualization, x-y projection)
    // Computes directly from boid positions (no spatial grid needed)
    int GetCellDensity(size_t grid_x, size_t grid_y) const;

private:
    // Single-pass flocking: computes separation + alignment + cohesion in one neighbor traversal
    Vec3 ApplyFlockingForces(size_t boid_idx, const BoidsParams& params);
    Vec3 ComputeBoundaryForce(const Vec3& pos);
    void ClampPosition(Vec3& pos);

    Boid boids_[MAX_BOIDS];
    size_t num_boids_;
    bool initialized_;

    // Simple random number generator (LCG)
    uint32_t rng_state_;
    float Random01();
};

} // namespace murmur

#endif // BOIDS_H
