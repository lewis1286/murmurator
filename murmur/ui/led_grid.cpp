#include "led_grid.h"

namespace murmur {

void LedGrid::Init(daisy::DaisyPatch* patch) {
    patch_ = patch;
    Clear();
}

void LedGrid::Clear() {
    for (size_t x = 0; x < LED_GRID_WIDTH; x++) {
        for (size_t y = 0; y < LED_GRID_HEIGHT; y++) {
            brightness_[x][y] = 0.0f;
        }
    }
}

void LedGrid::SetLed(size_t x, size_t y, float brightness) {
    if (x >= LED_GRID_WIDTH || y >= LED_GRID_HEIGHT) return;

    // Clamp brightness
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;

    brightness_[x][y] = brightness;
}

void LedGrid::UpdateFromFlock(const BoidsFlock& flock) {
    // Map boid density to LED brightness
    // The grid cells in BoidsFlock match our LED grid (4x4)

    for (size_t x = 0; x < LED_GRID_WIDTH; x++) {
        for (size_t y = 0; y < LED_GRID_HEIGHT; y++) {
            int density = flock.GetCellDensity(x, y);

            // Map density to brightness
            // Max reasonable density is about 4 boids per cell
            float brightness = static_cast<float>(density) / 4.0f;
            if (brightness > 1.0f) brightness = 1.0f;

            brightness_[x][y] = brightness;
        }
    }
}

void LedGrid::Update() {
    if (!patch_) return;

    // Daisy Patch has 16 LEDs accessible via the patch's LED driver
    // The exact method depends on the hardware revision
    // For the Daisy Patch (not Patch SM), we access via the 74HC595 shift register

    // Convert 2D grid to linear LED indices
    // The mapping may need adjustment based on physical LED layout
    for (size_t y = 0; y < LED_GRID_HEIGHT; y++) {
        for (size_t x = 0; x < LED_GRID_WIDTH; x++) {
            // Calculate linear index (row-major order)
            size_t led_idx = y * LED_GRID_WIDTH + x;

            // Set LED using the Patch's LED interface
            // Note: DaisyPatch uses SetLed() for basic on/off
            // For PWM brightness, we'd need to use a different approach

            // Simple threshold for on/off (since Patch LEDs may not support PWM directly)
            bool led_on = brightness_[x][y] > 0.1f;

            // The Daisy Patch accent LEDs are typically controlled
            // through direct GPIO or shift register
            // This is a simplified implementation
            (void)led_idx;
            (void)led_on;
            // patch_->SetLed(led_idx, led_on);
        }
    }

    // For now, we'll use the OLED to show LED states since
    // direct LED control varies by Patch hardware version
}

} // namespace murmur
