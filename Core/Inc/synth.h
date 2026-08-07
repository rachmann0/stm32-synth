#ifndef OSCILLATOR_H
#define OSCILLATOR_H

#include <stdint.h>

// SINE TABLE
void sine_table_init(void);

// OSCILLATOR

typedef enum{
    SINE,
    TRIANGLE,
    SQUARE,
    SAWTOOTH,
} Waveform;

// one way to think about this is as an oscillator state
typedef struct {
    // use 32bit phase accumulator
    uint32_t phase;
    uint32_t phase2;
    uint32_t phase3;
    uint32_t phase4;
    uint32_t phase5;

    uint32_t phase_increment;
    float amplitude;
    Waveform waveform;
    uint32_t chord;

} Oscillator;

void oscillator_init(Oscillator *osc, float freq, float amp);
float oscillator_process(Oscillator *osc);
void toggle_OSC2(void);
void iter_OSC1_waveform();
void iter_chord();

// SYNTH

void synth_init(void);
float synth_process(void);

#endif