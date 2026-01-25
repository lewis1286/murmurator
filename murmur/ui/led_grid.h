#pragma once
#ifndef LED_GRID_H
#define LED_GRID_H

#include "daisy_patch.h"
#include "../boids/boids.h"

namespace murmur {

// Daisy Patch has a 4x4 LED grid (accent LEDs via shift register)
// Note: The actual LED control might need adjustment based on hardware
constexpr size_t LED_GRID_WIDTH = 4;
constexpr size_t LED_GRID_HEIGHT = 4;

class LedGrid {
public:
    LedGrid() : patch_(nullptr) {}
    ~LedGrid() {}

    void Init(daisy::DaisyPatch* patch);

    // Update LED brightness based on boid density in each cell
    void UpdateFromFlock(const BoidsFlock& flock);

    // Set individual LED brightness (0-1)
    void SetLed(size_t x, size_t y, float brightness);

    // Clear all LEDs
    void Clear();

    // Apply the LED state to hardware
    void Update();

private:
    daisy::DaisyPatch* patch_;
    float brightness_[LED_GRID_WIDTH][LED_GRID_HEIGHT];
};

} // namespace murmur

#endif // LED_GRID_H
