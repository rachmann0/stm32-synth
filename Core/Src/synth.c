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

// music notes
#define MAJOR_THIRD_RATIO 1.259921f
#define FIFTH_RATIO       1.498307f
#define MAJOR_SEVENTH_RATIO 1.88775f
#define MAJOR_NINTH_RATIO 2.24492f

static Oscillator osc1;
static Oscillator osc2;

void oscillator_init(Oscillator *osc, float freq, float amp)
{
    osc->phase = 0;
    osc->phase2 = 0;
    osc->phase3 = 0;
    osc->phase4 = 0;
    osc->phase5 = 0;

    osc->phase_increment =
        (uint32_t)((freq * 4294967296.0) / SAMPLE_RATE); // 4294967296.0 is 2^32

    osc->amplitude = amp;

    // osc->waveform = SQUARE;
    // osc->waveform = SAWTOOTH;
    osc->waveform = SINE;
    // osc->waveform = TRIANGLE;
    osc->chord = 0;
}

#define TABLE_BITS 10 // for 1024 sine table entries, 2^10 = 1024
#define TABLE_SIZE (1 << TABLE_BITS)

// use .0f instead of .0 to avoid implicit conversion from double to float, which can cause performance issues on some microcontrollers
#define PHASE_SCALE (1.0f / 4294967296.0f)

static inline float phase_to_sine(uint32_t phase)
{
    uint16_t index = phase >> (32 - TABLE_BITS);
    return sine_table[index];
}
static inline float phase_to_triangle(uint32_t phase)
{
    return -2*fabsf(2.0f * (phase * PHASE_SCALE) - 1.0f)+1;
}
static inline float phase_to_square(uint32_t phase)
{
    return (phase >= 2147483648U) ? 1.0f : -1.0f; // 2147483648U is 2^31, the midpoint of the uint32_t range break;
}
static inline float phase_to_saw(uint32_t phase)
{
    return 2.0f * (phase * PHASE_SCALE) - 1.0f; // phase is a uint32_t, so it wraps around at 2^32, which is 4294967296
}
static inline float compute_sample_value(Waveform waveform, uint32_t phase){
    float wave = 0.0f;
    switch (waveform){
        case SINE:
            wave = phase_to_sine(phase);
            break;
        case TRIANGLE: 
            wave = phase_to_triangle(phase);
            break;
        case SQUARE: 
            wave = phase_to_square(phase);
            break;
        case SAWTOOTH:
            wave = phase_to_saw(phase);
            break;
        default:
            break;
    }
    return wave;
}

float oscillator_process(Oscillator *osc)
{
    osc->phase += osc->phase_increment;
    // need to explicitly cast to avoid wierd bug
    osc->phase2 += (uint32_t)(osc->phase_increment * MAJOR_THIRD_RATIO); 
    osc->phase3 += (uint32_t)(osc->phase_increment * FIFTH_RATIO);
    osc->phase4 += (uint32_t)(osc->phase_increment * MAJOR_SEVENTH_RATIO);
    osc->phase5 += (uint32_t)(osc->phase_increment * MAJOR_NINTH_RATIO);

    float wave = compute_sample_value(osc->waveform, osc->phase);
    // switch (osc->waveform){
    //     case SINE:
    //         wave = phase_to_sine(osc->phase);
    //         break;
    //     case TRIANGLE: 
    //         wave = phase_to_triangle(osc->phase);
    //         break;
    //     case SQUARE: 
    //         wave = phase_to_square(osc->phase);
    //         break;
    //     case SAWTOOTH:
    //         wave = phase_to_saw(osc->phase);
    //         break;
    //     default:
    //         break;
    // }

    // MAJOR CHORD TEST
    float sample = 0.0f;
    // NO sqrtf() IT CAN CAUSE real-time deadline misses, because it is a slow function, and we are in an interrupt context
    // float gain = 1.0f/sqrtf(osc->chord+1);
    // float gain = 1.0f/(osc->chord+1);
    float gain = 0.0f;
    switch(osc1.chord)
    {
        case 0:
            gain = 1.0f;
            break;
        case 1:
            gain = 0.707f;
            break;
        case 2:
            gain = 0.577f;
            break;
        case 3:
            gain = 0.5f;
            break;
        case 4:
            gain = 0.447f;
            break;
    }

    // root note
    sample += wave*gain;
    // major third
    if (osc->chord>0) {
        // sample += phase_to_saw(osc->phase2)*gain;
        sample += compute_sample_value(osc->waveform, osc->phase2)*gain; // add osc2 to the major third, to create a detuned effect
    }
    // perfect fifth
    if (osc->chord>1) {
        sample += compute_sample_value(osc->waveform, osc->phase3)*gain;
    }
    // major seventh
    if (osc->chord>2) {
        sample += compute_sample_value(osc->waveform, osc->phase4)*gain;
    }
    // major ninth
    if (osc->chord>3) {
        sample += compute_sample_value(osc->waveform, osc->phase5)*gain;
    }


    // return wave * osc->amplitude;
    return sample * osc->amplitude;
}

// void turn_off_oscillator(Oscillator *osc)
void toggle_OSC2(void)
{
    osc2.amplitude = (osc2.amplitude == 0.0f) ? 0.5f : 0.0f;
}

void iter_OSC1_waveform()
{
    osc1.waveform = (osc1.waveform + 1) % 4; // cycle through waveforms
}

void iter_chord()
{
    osc1.chord = (osc1.chord + 1) % 5;
}

// SYNTH
void synth_init(void)
{
    sine_table_init(); // initialize the sine table
    oscillator_init(&osc1, 440.0f, 0.5f);
    oscillator_init(&osc2, 442.0f, 0.5f);
}

void set_osc1_freq(float freq)
{
    osc1.phase_increment = (uint32_t)((freq * 4294967296.0) / SAMPLE_RATE);
}

float synth_process(void)
{
    float sample = 0.0f;

    sample += oscillator_process(&osc1);
    // sample += oscillator_process(&osc2);

    return sample;
}