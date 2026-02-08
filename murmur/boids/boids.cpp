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

    // Initialize boids with random positions and velocities
    for (size_t i = 0; i < num_boids_; i++) {
        boids_[i].position = Vec2(Random01(), Random01());
        boids_[i].velocity = Vec2(
            (Random01() - 0.5f) * 0.02f,
            (Random01() - 0.5f) * 0.02f
        );
        boids_[i].acceleration = Vec2(0.0f, 0.0f);
    }

    // Clear grid
    for (size_t x = 0; x < GRID_SIZE; x++) {
        for (size_t y = 0; y < GRID_SIZE; y++) {
            grid_counts_[x][y] = 0;
        }
    }

    initialized_ = true;
}

void BoidsFlock::SetNumBoids(size_t num) {
    size_t new_num = (num > MAX_BOIDS) ? MAX_BOIDS : num;

    // Initialize any new boids
    for (size_t i = num_boids_; i < new_num; i++) {
        boids_[i].position = Vec2(Random01(), Random01());
        boids_[i].velocity = Vec2(
            (Random01() - 0.5f) * 0.02f,
            (Random01() - 0.5f) * 0.02f
        );
        boids_[i].acceleration = Vec2(0.0f, 0.0f);
    }

    num_boids_ = new_num;
}

void BoidsFlock::Scatter() {
    for (size_t i = 0; i < num_boids_; i++) {
        boids_[i].position = Vec2(Random01(), Random01());
        boids_[i].velocity = Vec2(
            (Random01() - 0.5f) * 0.05f,
            (Random01() - 0.5f) * 0.05f
        );
    }
}

void BoidsFlock::UpdateSpatialGrid() {
    // Clear grid
    for (size_t x = 0; x < GRID_SIZE; x++) {
        for (size_t y = 0; y < GRID_SIZE; y++) {
            grid_counts_[x][y] = 0;
        }
    }

    // Populate grid
    for (size_t i = 0; i < num_boids_; i++) {
        int gx = static_cast<int>(boids_[i].position.x * GRID_SIZE);
        int gy = static_cast<int>(boids_[i].position.y * GRID_SIZE);

        // Clamp to valid grid range [0, GRID_SIZE-1].
        // C/C++ '%' on negative ints preserves the sign (-2 % 4 == -2, not 2),
        // so a slightly negative position (from float drift) would produce a
        // negative index. Casting that to size_t wraps to a huge value,
        // causing an out-of-bounds array access and a hard fault / freeze.
        if (gx < 0) gx = 0;
        if (gx >= static_cast<int>(GRID_SIZE)) gx = static_cast<int>(GRID_SIZE) - 1;
        if (gy < 0) gy = 0;
        if (gy >= static_cast<int>(GRID_SIZE)) gy = static_cast<int>(GRID_SIZE) - 1;

        size_t& count = grid_counts_[gx][gy];
        if (count < MAX_BOIDS_PER_CELL) {
            grid_[gx][gy][count] = i;
            count++;
        }
    }
}

void BoidsFlock::GetNeighbors(size_t boid_idx, float radius,
                               size_t* neighbors, size_t& count) {
    count = 0;
    const Vec2& pos = boids_[boid_idx].position;

    // Determine which grid cells to check.
    // Clamp base cell the same way as UpdateSpatialGrid to avoid
    // negative indices from float edge cases.
    int gx = static_cast<int>(pos.x * GRID_SIZE);
    int gy = static_cast<int>(pos.y * GRID_SIZE);
    if (gx < 0) gx = 0;
    if (gx >= static_cast<int>(GRID_SIZE)) gx = static_cast<int>(GRID_SIZE) - 1;
    if (gy < 0) gy = 0;
    if (gy >= static_cast<int>(GRID_SIZE)) gy = static_cast<int>(GRID_SIZE) - 1;

    // Check 3x3 neighborhood of cells
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            int cx = gx + dx;
            int cy = gy + dy;

            // Wrap neighborhood offsets into valid range
            if (cx < 0) cx += GRID_SIZE;
            if (cx >= static_cast<int>(GRID_SIZE)) cx -= GRID_SIZE;
            if (cy < 0) cy += GRID_SIZE;
            if (cy >= static_cast<int>(GRID_SIZE)) cy -= GRID_SIZE;

            // Check all boids in this cell
            for (size_t i = 0; i < grid_counts_[cx][cy]; i++) {
                size_t other_idx = grid_[cx][cy][i];
                if (other_idx == boid_idx) continue;

                float dist = Vec2::Distance(pos, boids_[other_idx].position);
                if (dist < radius && count < MAX_BOIDS) {
                    neighbors[count++] = other_idx;
                }
            }
        }
    }
}

Vec2 BoidsFlock::Separation(size_t boid_idx, const BoidsParams& params) {
    Vec2 steering(0.0f, 0.0f);
    size_t neighbors[MAX_BOIDS];
    size_t count = 0;

    GetNeighbors(boid_idx, params.perception_radius, neighbors, count);

    if (count == 0) return steering;

    const Vec2& pos = boids_[boid_idx].position;

    for (size_t i = 0; i < count; i++) {
        Vec2 diff = pos - boids_[neighbors[i]].position;
        float dist = diff.Magnitude();
        if (dist > 0.0001f) {
            diff = diff / (dist * dist);  // Weight by inverse square distance
            steering += diff;
        }
    }

    steering = steering / static_cast<float>(count);

    if (steering.MagnitudeSquared() > 0.0f) {
        steering.SetMagnitude(params.max_speed);
        steering = steering - boids_[boid_idx].velocity;
        steering.Limit(params.max_force);
    }

    return steering * params.separation_weight;
}

Vec2 BoidsFlock::Alignment(size_t boid_idx, const BoidsParams& params) {
    Vec2 steering(0.0f, 0.0f);
    size_t neighbors[MAX_BOIDS];
    size_t count = 0;

    GetNeighbors(boid_idx, params.perception_radius, neighbors, count);

    if (count == 0) return steering;

    Vec2 avg_velocity(0.0f, 0.0f);

    for (size_t i = 0; i < count; i++) {
        avg_velocity += boids_[neighbors[i]].velocity;
    }

    avg_velocity = avg_velocity / static_cast<float>(count);
    avg_velocity.SetMagnitude(params.max_speed);
    steering = avg_velocity - boids_[boid_idx].velocity;
    steering.Limit(params.max_force);

    return steering * params.alignment_weight;
}

Vec2 BoidsFlock::Cohesion(size_t boid_idx, const BoidsParams& params) {
    Vec2 steering(0.0f, 0.0f);
    size_t neighbors[MAX_BOIDS];
    size_t count = 0;

    GetNeighbors(boid_idx, params.perception_radius, neighbors, count);

    if (count == 0) return steering;

    Vec2 center(0.0f, 0.0f);

    for (size_t i = 0; i < count; i++) {
        center += boids_[neighbors[i]].position;
    }

    center = center / static_cast<float>(count);

    // Desired velocity towards center
    Vec2 desired = center - boids_[boid_idx].position;
    desired.SetMagnitude(params.max_speed);
    steering = desired - boids_[boid_idx].velocity;
    steering.Limit(params.max_force);

    return steering * params.cohesion_weight;
}

// Debug: volatile so debugger can always read these at -O2
volatile float dbg_pos_x, dbg_pos_y;

void BoidsFlock::WrapPosition(Vec2& pos) {
    dbg_pos_x = pos.x;
    dbg_pos_y = pos.y;

    // Guard against non-finite values to avoid infinite loops
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y)) {
        pos.x = 0.5f;
        pos.y = 0.5f;
        return;
    }

    // Use while loops to handle any position value
    while (pos.x < 0.0f) pos.x += 1.0f;
    while (pos.x >= 1.0f) pos.x -= 1.0f;
    while (pos.y < 0.0f) pos.y += 1.0f;
    while (pos.y >= 1.0f) pos.y -= 1.0f;
}

void BoidsFlock::Update(float dt, const BoidsParams& params) {
    if (!initialized_ || num_boids_ == 0) return;

    // Update spatial grid
    UpdateSpatialGrid();

    // Apply flocking forces
    for (size_t i = 0; i < num_boids_; i++) {
        Vec2 sep = Separation(i, params);
        Vec2 ali = Alignment(i, params);
        Vec2 coh = Cohesion(i, params);

        boids_[i].ApplyForce(sep);
        boids_[i].ApplyForce(ali);
        boids_[i].ApplyForce(coh);
    }

    // Update physics
    for (size_t i = 0; i < num_boids_; i++) {
        boids_[i].velocity += boids_[i].acceleration * dt;
        boids_[i].velocity.Limit(params.max_speed);
        boids_[i].position += boids_[i].velocity * dt;
        boids_[i].acceleration = Vec2(0.0f, 0.0f);

        WrapPosition(boids_[i].position);
    }
}

int BoidsFlock::GetCellDensity(size_t grid_x, size_t grid_y) const {
    if (grid_x >= GRID_SIZE || grid_y >= GRID_SIZE) return 0;
    return static_cast<int>(grid_counts_[grid_x][grid_y]);
}

} // namespace murmur
