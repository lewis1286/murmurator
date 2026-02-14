#pragma once
#ifndef BOIDS_H
#define BOIDS_H

#include "vec3.h"
#include <cstdint>
#include <cstddef>

namespace murmur {

constexpr size_t MAX_BOIDS = 16;
constexpr size_t GRID_SIZE = 4;  // 4x4 spatial partitioning grid (x-y plane)
constexpr size_t MAX_BOIDS_PER_CELL = MAX_BOIDS;

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
    int GetCellDensity(size_t grid_x, size_t grid_y) const;

private:
    void UpdateSpatialGrid();
    Vec3 Separation(size_t boid_idx, const BoidsParams& params);
    Vec3 Alignment(size_t boid_idx, const BoidsParams& params);
    Vec3 Cohesion(size_t boid_idx, const BoidsParams& params);
    void WrapPosition(Vec3& pos);
    void GetNeighbors(size_t boid_idx, float radius, size_t* neighbors, size_t& count);

    Boid boids_[MAX_BOIDS];
    size_t num_boids_;
    bool initialized_;

    // Spatial partitioning grid (x-y plane only; z proximity handled by 3D distance)
    size_t grid_[GRID_SIZE][GRID_SIZE][MAX_BOIDS_PER_CELL];
    size_t grid_counts_[GRID_SIZE][GRID_SIZE];

    // Simple random number generator (LCG)
    uint32_t rng_state_;
    float Random01();
};

} // namespace murmur

#endif // BOIDS_H
