#pragma once
#ifndef DISPLAY_H
#define DISPLAY_H

#include "daisy_patch.h"
#include "../boids/boids.h"

namespace murmur {

enum class DisplayPage {
    FLOCK_VIEW,      // Boid visualization
    PARAMETERS,      // Parameter values
    WAVEFORM,        // Audio waveform
    NUM_PAGES
};

class Display {
public:
    Display() : current_page_(DisplayPage::FLOCK_VIEW), patch_(nullptr) {}
    ~Display() {}

    void Init(daisy::DaisyPatch* patch);
    void SetPage(DisplayPage page) { current_page_ = page; }
    void NextPage();
    DisplayPage GetPage() const { return current_page_; }

    void DrawFlockView(const BoidsFlock& flock, const BoidsParams& params);
    void DrawParameters(const BoidsParams& params, size_t num_boids, bool recording);
    void DrawWaveform(const float* buffer, size_t size, size_t read_pos);

    void Clear();
    void Update();

private:
    void DrawBoid(const Boid& boid, bool highlight = false);
    void DrawTitle(const char* title);

    DisplayPage current_page_;
    daisy::DaisyPatch* patch_;
};

} // namespace murmur

#endif // DISPLAY_H
