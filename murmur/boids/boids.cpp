#include "boids.h"
#include <cmath>

namespace murmur {

float BoidsFlock::Random01() {
    // Linear congruential generator
    rng_state_ = rng_state_ * 1103515245 + 12345;
    return static_cast<float>((rng_state_ >> 16) & 0x7FFF) / 32767.0f;
}

void BoidsFlock::Init(size_t num_boids) {
    rng_state_ = 12345;  // Seed

    num_boids_ = (num_boids > MAX_BOIDS) ? MAX_BOIDS : num_boids;

    // Initialize boids with random positions inside safe zone (within margins)
    for (size_t i = 0; i < num_boids_; i++) {
        boids_[i].position = Vec3(
            BOUNDARY_MARGIN_XY + Random01() * (1.0f - 2.0f * BOUNDARY_MARGIN_XY),
            BOUNDARY_MARGIN_XY + Random01() * (1.0f - 2.0f * BOUNDARY_MARGIN_XY),
            0.3f + Random01() * 0.4f  // z starts in 0.3-0.7 range (within margins)
        );
        boids_[i].velocity = Vec3(
            (Random01() - 0.5f) * 0.02f,
            (Random01() - 0.5f) * 0.02f,
            (Random01() - 0.5f) * 0.01f  // Slower z movement
        );
        boids_[i].acceleration = Vec3(0.0f, 0.0f, 0.0f);
    }

    initialized_ = true;
}

void BoidsFlock::SetNumBoids(size_t num) {
    size_t new_num = (num > MAX_BOIDS) ? MAX_BOIDS : num;

    // Initialize any new boids inside safe zone
    for (size_t i = num_boids_; i < new_num; i++) {
        boids_[i].position = Vec3(
            BOUNDARY_MARGIN_XY + Random01() * (1.0f - 2.0f * BOUNDARY_MARGIN_XY),
            BOUNDARY_MARGIN_XY + Random01() * (1.0f - 2.0f * BOUNDARY_MARGIN_XY),
            0.3f + Random01() * 0.4f
        );
        boids_[i].velocity = Vec3(
            (Random01() - 0.5f) * 0.02f,
            (Random01() - 0.5f) * 0.02f,
            (Random01() - 0.5f) * 0.01f
        );
        boids_[i].acceleration = Vec3(0.0f, 0.0f, 0.0f);
    }

    num_boids_ = new_num;
}

void BoidsFlock::Scatter() {
    for (size_t i = 0; i < num_boids_; i++) {
        boids_[i].position = Vec3(
            BOUNDARY_MARGIN_XY + Random01() * (1.0f - 2.0f * BOUNDARY_MARGIN_XY),
            BOUNDARY_MARGIN_XY + Random01() * (1.0f - 2.0f * BOUNDARY_MARGIN_XY),
            BOUNDARY_MARGIN_Z_LO + Random01() * (1.0f - BOUNDARY_MARGIN_Z_LO - BOUNDARY_MARGIN_Z_HI)
        );
        boids_[i].velocity = Vec3(
            (Random01() - 0.5f) * 0.05f,
            (Random01() - 0.5f) * 0.05f,
            (Random01() - 0.5f) * 0.02f
        );
    }
}

Vec3 BoidsFlock::ApplyFlockingForces(size_t boid_idx, const BoidsParams& params) {
    const Vec3& pos = boids_[boid_idx].position;
    const Vec3& vel = boids_[boid_idx].velocity;
    float radius_sq = params.perception_radius * params.perception_radius;

    // Accumulators for the three forces
    Vec3 sep_sum(0.0f, 0.0f, 0.0f);
    Vec3 ali_sum(0.0f, 0.0f, 0.0f);
    Vec3 coh_sum(0.0f, 0.0f, 0.0f);
    size_t count = 0;

    // Single brute-force pass over all other boids
    for (size_t i = 0; i < num_boids_; i++) {
        if (i == boid_idx) continue;

        float dist_sq = Vec3::DistanceSquared(pos, boids_[i].position);
        if (dist_sq >= radius_sq || dist_sq < 0.00000001f) continue;

        count++;

        // Separation: accumulate diff weighted by inverse squared distance
        Vec3 diff = pos - boids_[i].position;
        sep_sum += diff * (1.0f / dist_sq);

        // Alignment: accumulate neighbor velocities
        ali_sum += boids_[i].velocity;

        // Cohesion: accumulate neighbor positions
        coh_sum += boids_[i].position;
    }

    if (count == 0) return Vec3(0.0f, 0.0f, 0.0f);

    float inv_count = 1.0f / static_cast<float>(count);
    Vec3 force(0.0f, 0.0f, 0.0f);

    // Separation steering
    sep_sum *= inv_count;
    if (sep_sum.MagnitudeSquared() > 0.0f) {
        sep_sum.SetMagnitude(params.max_speed);
        sep_sum = sep_sum - vel;
        sep_sum.Limit(params.max_force);
        force += sep_sum * params.separation_weight;
    }

    // Alignment steering
    ali_sum *= inv_count;
    ali_sum.SetMagnitude(params.max_speed);
    Vec3 ali_steer = ali_sum - vel;
    ali_steer.Limit(params.max_force);
    force += ali_steer * params.alignment_weight;

    // Cohesion steering
    coh_sum *= inv_count;
    Vec3 desired = coh_sum - pos;
    desired.SetMagnitude(params.max_speed);
    Vec3 coh_steer = desired - vel;
    coh_steer.Limit(params.max_force);
    force += coh_steer * params.cohesion_weight;

    return force;
}

Vec3 BoidsFlock::ComputeBoundaryForce(const Vec3& pos) {
    Vec3 force(0.0f, 0.0f, 0.0f);
    float t;

    // X axis (symmetric margins)
    if (pos.x < BOUNDARY_MARGIN_XY) {
        t = (BOUNDARY_MARGIN_XY - pos.x) / BOUNDARY_MARGIN_XY;
        force.x = BOUNDARY_FORCE_XY * t * t;
    } else if (pos.x > 1.0f - BOUNDARY_MARGIN_XY) {
        t = (pos.x - (1.0f - BOUNDARY_MARGIN_XY)) / BOUNDARY_MARGIN_XY;
        force.x = -BOUNDARY_FORCE_XY * t * t;
    }

    // Y axis (symmetric margins)
    if (pos.y < BOUNDARY_MARGIN_XY) {
        t = (BOUNDARY_MARGIN_XY - pos.y) / BOUNDARY_MARGIN_XY;
        force.y = BOUNDARY_FORCE_XY * t * t;
    } else if (pos.y > 1.0f - BOUNDARY_MARGIN_XY) {
        t = (pos.y - (1.0f - BOUNDARY_MARGIN_XY)) / BOUNDARY_MARGIN_XY;
        force.y = -BOUNDARY_FORCE_XY * t * t;
    }

    // Z axis (asymmetric margins)
    if (pos.z < BOUNDARY_MARGIN_Z_LO) {
        t = (BOUNDARY_MARGIN_Z_LO - pos.z) / BOUNDARY_MARGIN_Z_LO;
        force.z = BOUNDARY_FORCE_Z * t * t;
    } else if (pos.z > 1.0f - BOUNDARY_MARGIN_Z_HI) {
        t = (pos.z - (1.0f - BOUNDARY_MARGIN_Z_HI)) / BOUNDARY_MARGIN_Z_HI;
        force.z = -BOUNDARY_FORCE_Z * t * t;
    }

    return force;
}

void BoidsFlock::ClampPosition(Vec3& pos) {
    // Guard against non-finite values
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) {
        pos.x = 0.5f;
        pos.y = 0.5f;
        pos.z = 0.5f;
        return;
    }

    // Hard clamp to [0, 1] as safety net
    if (pos.x < 0.0f) pos.x = 0.0f;
    else if (pos.x > 1.0f) pos.x = 1.0f;
    if (pos.y < 0.0f) pos.y = 0.0f;
    else if (pos.y > 1.0f) pos.y = 1.0f;
    if (pos.z < 0.0f) pos.z = 0.0f;
    else if (pos.z > 1.0f) pos.z = 1.0f;
}

void BoidsFlock::Update(float dt, const BoidsParams& params) {
    if (!initialized_ || num_boids_ == 0) return;

    // Apply flocking + boundary forces
    for (size_t i = 0; i < num_boids_; i++) {
        Vec3 flocking = ApplyFlockingForces(i, params);
        boids_[i].ApplyForce(flocking);
        Vec3 boundary = ComputeBoundaryForce(boids_[i].position);
        boids_[i].ApplyForce(boundary);
    }

    // Update physics
    for (size_t i = 0; i < num_boids_; i++) {
        boids_[i].velocity += boids_[i].acceleration * dt;
        boids_[i].velocity.Limit(params.max_speed);
        boids_[i].position += boids_[i].velocity * dt;
        boids_[i].acceleration = Vec3(0.0f, 0.0f, 0.0f);

        ClampPosition(boids_[i].position);
    }
}

int BoidsFlock::GetCellDensity(size_t grid_x, size_t grid_y) const {
    if (grid_x >= LED_GRID_DIM || grid_y >= LED_GRID_DIM) return 0;

    int count = 0;
    for (size_t i = 0; i < num_boids_; i++) {
        int gx = static_cast<int>(boids_[i].position.x * LED_GRID_DIM);
        int gy = static_cast<int>(boids_[i].position.y * LED_GRID_DIM);
        // Clamp to valid range
        if (gx < 0) gx = 0;
        if (gx >= static_cast<int>(LED_GRID_DIM)) gx = static_cast<int>(LED_GRID_DIM) - 1;
        if (gy < 0) gy = 0;
        if (gy >= static_cast<int>(LED_GRID_DIM)) gy = static_cast<int>(LED_GRID_DIM) - 1;

        if (static_cast<size_t>(gx) == grid_x && static_cast<size_t>(gy) == grid_y) {
            count++;
        }
    }
    return count;
}

} // namespace murmur
