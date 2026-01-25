#pragma once
#ifndef BOIDS_H
#define BOIDS_H

#include "vec2.h"
#include <cstdint>
#include <cstddef>

namespace murmur {

constexpr size_t MAX_BOIDS = 16;
constexpr size_t GRID_SIZE = 4;  // 4x4 spatial partitioning grid
constexpr size_t MAX_BOIDS_PER_CELL = MAX_BOIDS;

struct Boid {
    Vec2 position;      // 0-1 range for both axes
    Vec2 velocity;
    Vec2 acceleration;

    void ApplyForce(const Vec2& force) {
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

    // Get boid density in a grid cell (for LED visualization)
    int GetCellDensity(size_t grid_x, size_t grid_y) const;

private:
    void UpdateSpatialGrid();
    Vec2 Separation(size_t boid_idx, const BoidsParams& params);
    Vec2 Alignment(size_t boid_idx, const BoidsParams& params);
    Vec2 Cohesion(size_t boid_idx, const BoidsParams& params);
    void WrapPosition(Vec2& pos);
    void GetNeighbors(size_t boid_idx, float radius, size_t* neighbors, size_t& count);

    Boid boids_[MAX_BOIDS];
    size_t num_boids_;
    bool initialized_;

    // Spatial partitioning grid
    size_t grid_[GRID_SIZE][GRID_SIZE][MAX_BOIDS_PER_CELL];
    size_t grid_counts_[GRID_SIZE][GRID_SIZE];

    // Simple random number generator (LCG)
    uint32_t rng_state_;
    float Random01();
};

} // namespace murmur

#endif // BOIDS_H
