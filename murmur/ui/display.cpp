#include "display.h"
#include <cstdio>
#include <cmath>

namespace murmur {

void Display::Init(daisy::DaisyPatch* patch) {
    patch_ = patch;
    current_page_ = DisplayPage::FLOCK_VIEW;
}

void Display::NextPage() {
    int page = static_cast<int>(current_page_);
    page = (page + 1) % static_cast<int>(DisplayPage::NUM_PAGES);
    current_page_ = static_cast<DisplayPage>(page);
}

void Display::Clear() {
    if (patch_) {
        patch_->display.Fill(false);
    }
}

void Display::Update() {
    if (patch_) {
        patch_->display.Update();
    }
}

void Display::DrawTitle(const char* title) {
    patch_->display.SetCursor(0, 0);
    patch_->display.WriteString(title, Font_6x8, true);
}

void Display::DrawBoid(const Boid& boid, bool highlight) {
    // Map x-y position to display coordinates
    // OLED is 128x64, reserve top 10 pixels for title
    int x = static_cast<int>(boid.position.x * 127);
    int y = 63 - static_cast<int>(boid.position.y * 53);

    // Clamp to display bounds
    if (x < 0) x = 0;
    if (x > 127) x = 127;
    if (y < 10) y = 10;
    if (y > 63) y = 63;

    // Calculate heading angle from x-y velocity
    // Negate vy because screen-y is inverted (y increases downward on OLED)
    float angle = atan2f(-boid.velocity.y, boid.velocity.x);

    // Triangle size varies with z (amplitude): louder = bigger
    // z ranges 0-1, map to triangle size 2-6 pixels
    float size = 2.0f + boid.position.z * 4.0f;
    if (highlight) size += 1.0f;

    // Front point
    int x1 = x + static_cast<int>(cosf(angle) * size);
    int y1 = y + static_cast<int>(sinf(angle) * size);

    // Back left point
    int x2 = x + static_cast<int>(cosf(angle + 2.5f) * size * 0.7f);
    int y2 = y + static_cast<int>(sinf(angle + 2.5f) * size * 0.7f);

    // Back right point
    int x3 = x + static_cast<int>(cosf(angle - 2.5f) * size * 0.7f);
    int y3 = y + static_cast<int>(sinf(angle - 2.5f) * size * 0.7f);

    // Clamp triangle vertices to display bounds.
    // DrawLine uses uint_fast8_t params -- negative values wrap to
    // huge unsigned values, causing Bresenham to loop ~forever.
    auto clampX = [](int v) { return v < 0 ? 0 : (v > 127 ? 127 : v); };
    auto clampY = [](int v) { return v < 10 ? 10 : (v > 63 ? 63 : v); };
    x1 = clampX(x1); y1 = clampY(y1);
    x2 = clampX(x2); y2 = clampY(y2);
    x3 = clampX(x3); y3 = clampY(y3);

    // Draw triangle (as three lines)
    patch_->display.DrawLine(x1, y1, x2, y2, true);
    patch_->display.DrawLine(x2, y2, x3, y3, true);
    patch_->display.DrawLine(x3, y3, x1, y1, true);

    if (highlight) {
        // Draw center point for highlighted boid
        patch_->display.DrawPixel(x, y, true);
    }
}

void Display::DrawFlockView(const BoidsFlock& flock, const BoidsParams& params) {
    Clear();
    DrawTitle("MURMUR BOIDS");

    // Draw border for flock area
    patch_->display.DrawRect(0, 10, 127, 63, true, false);

    // Draw all boids
    for (size_t i = 0; i < flock.GetNumBoids(); i++) {
        DrawBoid(flock.GetBoid(i), i == 0);
    }

    // Show boid count in corner
    char str[16];
    snprintf(str, sizeof(str), "%d", static_cast<int>(flock.GetNumBoids()));
    patch_->display.SetCursor(110, 2);
    patch_->display.WriteString(str, Font_6x8, true);

    Update();
}

void Display::DrawParameters(const BoidsParams& params, size_t num_boids,
                              float freq_range) {
    Clear();
    DrawTitle("MURMUR PARAMS");

    char str[32];

    // Separation
    patch_->display.SetCursor(0, 12);
    snprintf(str, sizeof(str), "Sep: %.2f", static_cast<double>(params.separation_weight));
    patch_->display.WriteString(str, Font_6x8, true);

    // Cohesion
    patch_->display.SetCursor(64, 12);
    snprintf(str, sizeof(str), "Coh: %.2f", static_cast<double>(params.cohesion_weight));
    patch_->display.WriteString(str, Font_6x8, true);

    // Freq Range
    patch_->display.SetCursor(0, 22);
    snprintf(str, sizeof(str), "Frq: %.0fHz", static_cast<double>(freq_range));
    patch_->display.WriteString(str, Font_6x8, true);

    // Alignment
    patch_->display.SetCursor(64, 22);
    snprintf(str, sizeof(str), "Ali: %.2f", static_cast<double>(params.alignment_weight));
    patch_->display.WriteString(str, Font_6x8, true);

    // Number of boids
    patch_->display.SetCursor(0, 36);
    snprintf(str, sizeof(str), "Boids: %d", static_cast<int>(num_boids));
    patch_->display.WriteString(str, Font_6x8, true);

    // Mapping info
    patch_->display.SetCursor(0, 46);
    patch_->display.WriteString("x:pan y:freq z:amp", Font_6x8, true);

    // Page indicator
    patch_->display.SetCursor(0, 54);
    patch_->display.WriteString("[2/3] Params", Font_6x8, true);

    Update();
}

void Display::DrawWaveform(const float* buffer, size_t size) {
    Clear();
    DrawTitle("MURMUR WAVE");

    // Draw waveform from display buffer (captured from audio output)
    int prev_y = 37;

    for (size_t x = 0; x < size && x < 128; x++) {
        float sample = buffer[x];
        int y = 37 + static_cast<int>(sample * 26);  // 37 is center, +/-26 pixels

        // Clamp
        if (y < 10) y = 10;
        if (y > 63) y = 63;

        if (x > 0) {
            patch_->display.DrawLine(x - 1, prev_y, x, y, true);
        }
        prev_y = y;
    }

    // Center line
    patch_->display.DrawLine(0, 37, 127, 37, true);

    // Page indicator
    patch_->display.SetCursor(0, 2);
    patch_->display.WriteString("[3/3] Wave", Font_6x8, true);

    Update();
}

} // namespace murmur
