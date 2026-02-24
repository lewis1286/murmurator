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

    // Triangle size varies with z (amplitude): louder = bigger.
    // z=0 is closest/loudest (large triangle), z=1 is farthest/quietest (small).
    float size = 2.0f + (1.0f - boid.position.z) * 4.0f;
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

void Display::DrawFlockView(const BoidsFlock& flock, const BoidsParams& params,
                             const char* chord_label) {
    Clear();
    DrawTitle("MURMUR BOIDS");

    // Draw border for flock area
    patch_->display.DrawRect(0, 10, 127, 63, true, false);

    // Draw all boids
    for (size_t i = 0; i < flock.GetNumBoids(); i++) {
        DrawBoid(flock.GetBoid(i), i == 0);
    }

    char str[16];

    // Show active chord + root note (e.g. "I:A", "IV:D#") when chord prog is running.
    // Positioned at x=74 so even the widest label ("IV:D#" = 30px) stays left of
    // the boid count at x=110.
    if (chord_label && chord_label[0] != '\0') {
        patch_->display.SetCursor(74, 2);
        patch_->display.WriteString(chord_label, Font_6x8, true);
    }

    // Show boid count in corner
    snprintf(str, sizeof(str), "%d", static_cast<int>(flock.GetNumBoids()));
    patch_->display.SetCursor(110, 2);
    patch_->display.WriteString(str, Font_6x8, true);

    Update();
}

void Display::DrawParameters(const BoidsParams& params, size_t num_boids,
                              float morph) {
    Clear();
    DrawTitle("MURMUR PARAMS");

    char str[32];

    // Density (derived: cohesion_weight = density * 2)
    float density = params.cohesion_weight * 0.5f;
    int den_i = static_cast<int>(density * 100);
    patch_->display.SetCursor(0, 12);
    snprintf(str, sizeof(str), "Den: %d%%", den_i);
    patch_->display.WriteString(str, Font_6x8, true);

    // Alignment
    int ali_i = static_cast<int>(params.alignment_weight);
    int ali_f = static_cast<int>((params.alignment_weight - ali_i) * 100);
    if (ali_f < 0) ali_f = -ali_f;
    patch_->display.SetCursor(64, 12);
    snprintf(str, sizeof(str), "Ali: %d.%02d", ali_i, ali_f);
    patch_->display.WriteString(str, Font_6x8, true);

    // Speed
    int spd_i = static_cast<int>(params.max_speed * 100);
    patch_->display.SetCursor(0, 22);
    snprintf(str, sizeof(str), "Spd: %d%%", spd_i);
    patch_->display.WriteString(str, Font_6x8, true);

    // Wave morph: show label for current blend position
    const char* wave_label;
    if (morph < 0.15f)       wave_label = "SINE";
    else if (morph < 0.85f)  wave_label = "SIN>TRI";
    else if (morph < 1.15f)  wave_label = "TRI";
    else if (morph < 1.85f)  wave_label = "TRI>SQR";
    else                     wave_label = "SQR";
    patch_->display.SetCursor(64, 22);
    snprintf(str, sizeof(str), "Wave:%s", wave_label);
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

void Display::DrawScaleSettings(int root, int scale_idx, int base_oct,
                                 int cursor, int span_oct, float freq_range,
                                 int chord_prog_mode, int chord_index) {
    Clear();
    DrawTitle("SCALE SETTINGS");

    // Local statics — same TU only, no ODR issues.
    static const char* root_names[]  = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    static const char* scale_names[] = {"Linear","Major","Nat.Minor","Dorian",
                                         "Pent.Maj","Pent.Min","Lydian","Mixo"};
    static const char* chord_names[] = {"I", "IV", "V", "I"};
    static const char* prog_labels[] = {"OFF", "10s", "15s"};

    char str[32];

    // Root row (cursor 0)
    patch_->display.SetCursor(0, 12);
    snprintf(str, sizeof(str), "%cRoot: %s",
             cursor == 0 ? '>' : ' ', root_names[root]);
    patch_->display.WriteString(str, Font_6x8, true);

    // Scale row (cursor 1)
    patch_->display.SetCursor(0, 22);
    snprintf(str, sizeof(str), "%cScale: %s",
             cursor == 1 ? '>' : ' ', scale_names[scale_idx]);
    patch_->display.WriteString(str, Font_6x8, true);

    // Octave row (cursor 2)
    patch_->display.SetCursor(0, 32);
    snprintf(str, sizeof(str), "%cOctave: %d",
             cursor == 2 ? '>' : ' ', base_oct);
    patch_->display.WriteString(str, Font_6x8, true);

    // Chord progression row (cursor 3) — only meaningful when scale != Linear
    patch_->display.SetCursor(0, 42);
    if (chord_prog_mode == 0 || scale_idx == 0) {
        snprintf(str, sizeof(str), "%cChProg: %s",
                 cursor == 3 ? '>' : ' ', prog_labels[chord_prog_mode]);
    } else {
        // Show interval and the currently active chord degree
        snprintf(str, sizeof(str), "%cChProg:%s [%s]",
                 cursor == 3 ? '>' : ' ',
                 prog_labels[chord_prog_mode],
                 chord_names[chord_index]);
    }
    patch_->display.WriteString(str, Font_6x8, true);

    // Navigation hint
    patch_->display.SetCursor(0, 54);
    patch_->display.WriteString(" enc>next  [3/3]", Font_6x8, true);

    Update();
}

} // namespace murmur
