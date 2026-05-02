// murmur_core.cpp — all murmur application logic.
// Include exactly once per build via MurmurBoids.cpp or algorithms/murmur.cpp.
// Never add to CPP_SOURCES — always pulled in by the #include in the entry file.
#include "daisysp.h"
#include "daisy_patch.h"
#include "audio/osc_voice.h"
#include "audio/simple_reverb.h"
#include "audio/scale_quantizer.h"
#include "audio/chord_progression.h"
#include "audio/axis_mapping.h"
#include "boids/boids.h"
#include "ui/display.h"
#include "ui/led_grid.h"
#include <cmath>

using namespace daisy;
using namespace daisysp;

// ─── Hardware ────────────────────────────────────────────────────────────────
static DaisyPatch* s_patch = nullptr;

// ─── Audio objects ───────────────────────────────────────────────────────────
static murmur::OscVoice      voices[murmur::MAX_BOIDS];
static murmur::SimpleReverb  reverb;
static constexpr float       REVERB_LEVEL = 0.4f;

// ─── Boids ───────────────────────────────────────────────────────────────────
static murmur::BoidsFlock  flock;
static murmur::BoidsParams boids_params;

// ─── UI ──────────────────────────────────────────────────────────────────────
static murmur::Display display;
static murmur::LedGrid led_grid;

// ─── Control state ───────────────────────────────────────────────────────────
static float density          = 0.5f;
static float alignment_weight = 1.0f;
static float morph            = 1.0f;
static float freq_range       = 400.0f;

// ─── Scale/chord state ───────────────────────────────────────────────────────
static murmur::ScaleQuantizer  scale_quantizer;
static murmur::ChordProgression chord_prog;
static murmur::AxisMapping     axis_mapping;
static int settings_cursor = 0;
static int span_octaves    = 3;

// ─── Audio parameters ────────────────────────────────────────────────────────
static constexpr float FREQ_MIN      = 200.0f;
static constexpr float MAX_AMP_TOTAL = 0.8f;

// ─── Runtime state ───────────────────────────────────────────────────────────
static int      num_boids   = 8;
static float    sample_rate = 48000.0f;

// ─── Timing ──────────────────────────────────────────────────────────────────
static uint32_t           last_display_update = 0;
static uint32_t           last_boids_update   = 0;
static constexpr uint32_t DISPLAY_UPDATE_MS   = 33;
static constexpr uint32_t BOIDS_UPDATE_MS     = 2;

// ─── Internal helpers ────────────────────────────────────────────────────────
static void UpdateVoicesFromBoids()
{
    murmur::MappingContext ctx = {
        scale_quantizer,
        FREQ_MIN,
        freq_range,
        span_octaves,
        MAX_AMP_TOTAL / static_cast<float>(num_boids)
    };

    for (int i = 0; i < num_boids; i++) {
        const murmur::Boid& boid = flock.GetBoid(i);
        murmur::VoiceParams vp = MapBoidToVoice(boid.position, axis_mapping, ctx);

        voices[i].SetParams(vp.freq, vp.amp, vp.pan, boid.position.z);
        voices[i].SetMorph(morph);
        if (scale_quantizer.GetScale() != murmur::ScaleType::OFF)
            voices[i].SnapFreq(vp.freq);
        voices[i].UpdateSmoothing();
    }
}

static void UpdateMurmurControls()
{
    // ProcessAnalogControls/ProcessDigitalControls are called by the entry point
    // (AudioCallback in standalone, meta AudioCallback in meta build) — do not call here.

    density = s_patch->GetKnobValue(DaisyPatch::CTRL_1);
    boids_params.separation_weight = density * 2.0f;
    boids_params.cohesion_weight   = (1.0f - density) * 2.0f;

    alignment_weight = s_patch->GetKnobValue(DaisyPatch::CTRL_2) * 2.0f;
    boids_params.alignment_weight = alignment_weight;

    boids_params.max_speed = 0.05f + s_patch->GetKnobValue(DaisyPatch::CTRL_3) * 1.45f;
    boids_params.max_force = boids_params.max_speed * 0.5f;

    morph = s_patch->GetKnobValue(DaisyPatch::CTRL_4) * 2.0f;

    if (s_patch->gate_input[1].Trig())
        flock.Scatter();
}

static void UpdateMurmurDisplay()
{
    switch (display.GetPage()) {
        case murmur::DisplayPage::FLOCK_VIEW: {
            static char chord_str[8];
            const char* chord_label = chord_prog.BuildLabel(scale_quantizer, chord_str, sizeof(chord_str))
                                      ? chord_str : nullptr;
            display.DrawFlockView(flock, boids_params, chord_label);
            break;
        }
        case murmur::DisplayPage::PARAMETERS:
            display.DrawParameters(boids_params, num_boids, morph);
            break;
        case murmur::DisplayPage::SCALE_SETTINGS:
            display.DrawScaleSettings(
                scale_quantizer.GetRoot(),
                static_cast<int>(scale_quantizer.GetScale()),
                scale_quantizer.GetBaseOctave(),
                settings_cursor,
                span_octaves,
                freq_range,
                chord_prog.GetMode(),
                chord_prog.GetIndex());
            break;
        default:
            break;
    }
    led_grid.Update();
}

// ─── Public API ───────────────────────────────────────────────────────────────

void MurmurInit(DaisyPatch* p)
{
    s_patch     = p;
    sample_rate = p->AudioSampleRate();

#ifndef MURMUR_UI_ONLY
    for (size_t i = 0; i < murmur::MAX_BOIDS; i++)
        voices[i].Init(sample_rate);
    reverb.Init(sample_rate);
#endif

    flock.Init(num_boids);
    boids_params.separation_weight = density * 2.0f;
    boids_params.cohesion_weight   = (1.0f - density) * 2.0f;
    boids_params.alignment_weight  = alignment_weight;
    boids_params.perception_radius = 0.25f;
    boids_params.max_speed         = 0.3f;
    boids_params.max_force         = 0.3f * 0.5f;

    display.Init(p);
    led_grid.Init(p);

#ifndef MURMUR_UI_ONLY
    for (int i = 0; i < num_boids; i++)
        voices[i].SetActive(true);
#endif
}

void MurmurOnActivate()
{
    last_boids_update   = System::GetNow();
    last_display_update = System::GetNow();
}

void MurmurOnDeactivate() {}

void MurmurProcessBlock(AudioHandle::InputBuffer  in,
                        AudioHandle::OutputBuffer out,
                        size_t size)
{
    for (size_t i = 0; i < size; i++) {
        float sum_l  = 0.0f;
        float sum_r  = 0.0f;
        float rev_in = 0.0f;

        for (size_t v = 0; v < static_cast<size_t>(num_boids); v++) {
            sum_l  += voices[v].ProcessLeft();
            sum_r  += voices[v].ProcessRight();
            rev_in += voices[v].GetReverbSend();
        }

        float rev_out = reverb.Process(rev_in);
        out[0][i] = sum_l + rev_out * REVERB_LEVEL;
        out[1][i] = sum_r + rev_out * REVERB_LEVEL;
        out[2][i] = in[2][i];
        out[3][i] = in[3][i];
    }
}

void MurmurOnEncoderShortPress()
{
    if (display.GetPage() == murmur::DisplayPage::SCALE_SETTINGS) {
        if (settings_cursor < 3) {
            settings_cursor++;
        } else {
            settings_cursor = 0;
            display.NextPage();
        }
    } else {
        display.NextPage();
    }
}

void MurmurOnEncoderIncrement(int32_t delta)
{
    if (delta == 0) return;

    if (display.GetPage() == murmur::DisplayPage::SCALE_SETTINGS) {
        switch (settings_cursor) {
            case 0: {
                int r = ((scale_quantizer.GetRoot() + delta) % 12 + 12) % 12;
                scale_quantizer.SetRoot(r);
                break;
            }
            case 1: {
                int s = ((static_cast<int>(scale_quantizer.GetScale()) + delta)
                         % static_cast<int>(murmur::ScaleType::COUNT)
                         + static_cast<int>(murmur::ScaleType::COUNT))
                        % static_cast<int>(murmur::ScaleType::COUNT);
                scale_quantizer.SetScale(static_cast<murmur::ScaleType>(s));
                break;
            }
            case 2:
                scale_quantizer.SetBaseOctave(scale_quantizer.GetBaseOctave() + delta);
                break;
            case 3:
                chord_prog.Increment(delta, System::GetNow(), scale_quantizer);
                break;
            default:
                break;
        }
    } else {
#ifndef MURMUR_UI_ONLY
        const int old_num = num_boids;
#endif
        num_boids += delta;
        if (num_boids < 4)  num_boids = 4;
        if (num_boids > 16) num_boids = 16;
        flock.SetNumBoids(num_boids);

#ifndef MURMUR_UI_ONLY
        if (num_boids > old_num) {
            for (int i = old_num; i < num_boids; i++)
                voices[i].SetActive(true);
        } else {
            for (int i = num_boids; i < old_num; i++)
                voices[i].SetActive(false);
        }
#endif
    }
}

void MurmurUpdateUI(DaisyPatch& /*patch*/)
{
    UpdateMurmurControls();

    const uint32_t now = System::GetNow();

    chord_prog.Update(now, scale_quantizer);

    if (now - last_boids_update >= BOIDS_UPDATE_MS) {
        float dt = static_cast<float>(now - last_boids_update) / 1000.0f;
        flock.Update(dt, boids_params);
#ifndef MURMUR_UI_ONLY
        UpdateVoicesFromBoids();
#endif
        last_boids_update = now;
    }

    if (now - last_display_update >= DISPLAY_UPDATE_MS) {
        led_grid.UpdateFromFlock(flock);
        UpdateMurmurDisplay();
        last_display_update = now;
    }
}
