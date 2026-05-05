
#pragma once

#include "common.h"

// ---------------------------------------------------------------------------
// RC receiver — interrupt-driven PWM capture for 6 channels
// ---------------------------------------------------------------------------

#define NUM_CHANNELS 6

// GPIO pins assigned to each channel
extern const uint8_t channel_pins[NUM_CHANNELS];

// Latest captured pulse widths (us). Use getReceiverValues() to read safely.
extern volatile uint16_t ReceiverValue[NUM_CHANNELS];

// Attach interrupts to all channel pins. Call once inside setup().
void rx_config(void);

// Copy ReceiverValue[] into ch[] with interrupts disabled.
void getReceiverValues(uint16_t *ch);