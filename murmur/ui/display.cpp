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
    // Map 0-1 position to display coordinates
    // OLED is 128x64, reserve top 10 pixels for title
    int x = static_cast<int>(boid.position.x * 127);
    int y = 10 + static_cast<int>(boid.position.y * 53);

    // Clamp to display bounds
    if (x < 0) x = 0;
    if (x > 127) x = 127;
    if (y < 10) y = 10;
    if (y > 63) y = 63;

    // Calculate heading angle
    float angle = boid.velocity.Angle();

    // Draw as small triangle pointing in direction of travel
    // Triangle size
    float size = highlight ? 4.0f : 3.0f;

    // Front point
    int x1 = x + static_cast<int>(cosf(angle) * size);
    int y1 = y + static_cast<int>(sinf(angle) * size);

    // Back left point
    int x2 = x + static_cast<int>(cosf(angle + 2.5f) * size * 0.7f);
    int y2 = y + static_cast<int>(sinf(angle + 2.5f) * size * 0.7f);

    // Back right point
    int x3 = x + static_cast<int>(cosf(angle - 2.5f) * size * 0.7f);
    int y3 = y + static_cast<int>(sinf(angle - 2.5f) * size * 0.7f);

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

void Display::DrawParameters(const BoidsParams& params, size_t num_boids, bool recording) {
    Clear();
    DrawTitle("MURMUR PARAMS");

    char str[32];

    // Separation
    patch_->display.SetCursor(0, 12);
    snprintf(str, sizeof(str), "Sep: %.2f", static_cast<double>(params.separation_weight));
    patch_->display.WriteString(str, Font_6x8, true);

    // Alignment
    patch_->display.SetCursor(64, 12);
    snprintf(str, sizeof(str), "Ali: %.2f", static_cast<double>(params.alignment_weight));
    patch_->display.WriteString(str, Font_6x8, true);

    // Cohesion
    patch_->display.SetCursor(0, 22);
    snprintf(str, sizeof(str), "Coh: %.2f", static_cast<double>(params.cohesion_weight));
    patch_->display.WriteString(str, Font_6x8, true);

    // Perception
    patch_->display.SetCursor(64, 22);
    snprintf(str, sizeof(str), "Rad: %.2f", static_cast<double>(params.perception_radius));
    patch_->display.WriteString(str, Font_6x8, true);

    // Number of boids
    patch_->display.SetCursor(0, 36);
    snprintf(str, sizeof(str), "Boids: %d", static_cast<int>(num_boids));
    patch_->display.WriteString(str, Font_6x8, true);

    // Recording status
    patch_->display.SetCursor(64, 36);
    if (recording) {
        patch_->display.WriteString("REC", Font_6x8, true);
    } else {
        patch_->display.WriteString("FROZEN", Font_6x8, true);
    }

    // Page indicator
    patch_->display.SetCursor(0, 54);
    patch_->display.WriteString("[2/3] Params", Font_6x8, true);

    Update();
}

void Display::DrawWaveform(const float* buffer, size_t size, size_t read_pos) {
    Clear();
    DrawTitle("MURMUR WAVE");

    // Draw waveform from buffer
    // Sample 128 points across the display
    int prev_y = 37;

    for (int x = 0; x < 128; x++) {
        // Map x to buffer position (look back from read_pos)
        size_t samples_back = (size * x) / 128;
        size_t idx = (read_pos + size - samples_back) % size;

        // Map sample to y coordinate
        float sample = buffer[idx];
        int y = 37 + static_cast<int>(sample * 26);  // 37 is center, ±26 pixels

        // Clamp
        if (y < 10) y = 10;
        if (y > 63) y = 63;

        // Draw line from previous point
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
