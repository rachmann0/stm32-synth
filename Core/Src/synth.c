#include "synth.h"
#include <math.h>

// SINE TABLE
#define SINE_TABLE_SIZE 1024
static const float TWO_PI = 6.28318530718f;
float sine_table[SINE_TABLE_SIZE];
void sine_table_init(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++)
    {
        float angle = TWO_PI * i / SINE_TABLE_SIZE;
        sine_table[i] = sinf(angle);
    }
};

// OSCILLATOR

#define CLOCK_SPEED 84000000
#define PERIOD 1905 // 44.1 kHz sample rate, 84 MHz clock speed, 84e6 / 44100 = 1904.76

// #define SAMPLE_RATE 44100.0f
// static const float SAMPLE_RATE = (float)CLOCK_SPEED / (float)PERIOD;
#define SAMPLE_RATE 44094.49f

static Oscillator osc1;
static Oscillator osc2;

void oscillator_init(Oscillator *osc, float freq, float amp)
{
    osc->phase = 0;

    osc->phase_increment =
        (uint32_t)((freq * 4294967296.0) / SAMPLE_RATE); // 4294967296.0 is 2^32, the maximum value of a uint32_t

    osc->amplitude = amp;
}

#define TABLE_BITS 10 // for 1024 sine table entries, 2^10 = 1024
#define TABLE_SIZE (1 << TABLE_BITS)

const float PHASE_SCALE = 1.0f / 4294967296.0f;

float oscillator_process(Oscillator *osc)
{
    osc->phase += osc->phase_increment;

    float wave = 0.0f;
    switch (osc->waveform){
        case SINE:
            uint16_t index = osc->phase >> (32 - TABLE_BITS);
            wave = sine_table[index];
            break;
        case SAWTOOTH:
            wave = 2.0f * (osc->phase * PHASE_SCALE) - 1.0f; // phase is a uint32_t, so it wraps around at 2^32, which is 4294967296
            break;
        case SQUARE: 
            wave = (osc->phase >= 2147483648U) ? 1.0f : -1.0f; // 2147483648U is 2^31, the midpoint of the uint32_t range
            break;
        case TRIANGLE: 
            // wave = (osc->phase > 2147483648U) ? 2.0f * (osc->phase / 2147483648.0f) - 1.0f : - 2.0f * ((osc->phase - 2147483648.0f) / 2147483648.0f) + 1.0f;
            // wave = (osc->phase > 2147483648U) ? 2.0f * (osc->phase / 2147483648.0f) - 1.0f : - 2.0f * ((osc->phase - 2147483648.0f) / 2147483648.0f) + 1.0f;
            wave = -2*fabsf(2.0f * (osc->phase * PHASE_SCALE) - 1.0f)+1;
            break;
        default:
            break;
    }

    // return sine_table[index] * osc->amplitude;
    return wave * osc->amplitude;
}

// void turn_off_oscillator(Oscillator *osc)
void toggle_OSC2(void)
{
    osc2.amplitude = (osc2.amplitude == 0.0f) ? 0.5f : 0.0f;
}

void iter_OSC1_waveform()
{
    osc1.waveform = (osc1.waveform + 1) % 4; // cycle through waveforms
    // osc1.waveform = (osc1.waveform + 1) % 2; // cycle through waveforms
}

// SYNTH
void synth_init(void)
{
    sine_table_init(); // initialize the sine table
    oscillator_init(&osc1, 440.0f, 0.5f);
    oscillator_init(&osc2, 442.0f, 0.5f);
}

float synth_process(void)
{
    float sample = 0.0f;

    sample += oscillator_process(&osc1);
    // sample += oscillator_process(&osc2);

    return sample;
}