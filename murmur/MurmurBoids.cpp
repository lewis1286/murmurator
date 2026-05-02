#include "daisysp.h"
#include "daisy_patch.h"
#include "murmur_core.cpp"

using namespace daisy;
using namespace daisysp;

static DaisyPatch       patch;
static volatile int32_t g_enc_delta   = 0;
static volatile bool    g_enc_pressed = false;

#ifndef MURMUR_UI_ONLY
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    patch.ProcessAnalogControls();
    patch.ProcessDigitalControls();
    g_enc_delta += patch.encoder.Increment();
    if (patch.encoder.RisingEdge()) g_enc_pressed = true;
    MurmurProcessBlock(in, out, size);
}
#endif

int main(void)
{
    patch.Init();
    MurmurInit(&patch);
    patch.StartAdc();
#ifndef MURMUR_UI_ONLY
    patch.StartAudio(AudioCallback);
#endif

    while (1) {
        const int32_t enc = g_enc_delta; g_enc_delta = 0;
        if (enc != 0)     MurmurOnEncoderIncrement(enc);
        if (g_enc_pressed) { g_enc_pressed = false; MurmurOnEncoderShortPress(); }
        MurmurUpdateUI(patch);
    }
}
