
#include "common.h"

#define NUM_CHANNELS 6

// ======================= PIN CONFIG =======================
const uint8_t channel_pins[NUM_CHANNELS] = {37, 38, 39, 40, 41, 42};

// ======================= DATA =======================
volatile uint32_t timer_ch[NUM_CHANNELS];
volatile uint16_t ReceiverValue[NUM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000};

// ======================= ISR CORE =======================
void IRAM_ATTR handle_channel(uint8_t ch)
{
    uint32_t t = micros();
    uint8_t pin = channel_pins[ch];
    uint8_t state;

    if (pin < 32)
        state = (GPIO.in >> pin) & 0x1;
    else
        state = (GPIO.in1.val >> (pin - 32)) & 0x1;

    if (state)
    {
        timer_ch[ch] = t;
    }
    else
    {
        uint32_t width = t - timer_ch[ch];
        if (width > 900 && width < 2100)
            ReceiverValue[ch] = (uint16_t)width;
    }
}

// ======================= ISR WRAPPERS =======================
void IRAM_ATTR ch1_ISR() { handle_channel(0); }
void IRAM_ATTR ch2_ISR() { handle_channel(1); }
void IRAM_ATTR ch3_ISR() { handle_channel(2); }
void IRAM_ATTR ch4_ISR() { handle_channel(3); }
void IRAM_ATTR ch5_ISR() { handle_channel(4); }
void IRAM_ATTR ch6_ISR() { handle_channel(5); }

// ======================= INIT =======================
void rx_config(void)
{
    for (int i = 0; i < NUM_CHANNELS; i++)
        pinMode(channel_pins[i], INPUT);

    attachInterrupt(channel_pins[0], ch1_ISR, CHANGE);
    attachInterrupt(channel_pins[1], ch2_ISR, CHANGE);
    attachInterrupt(channel_pins[2], ch3_ISR, CHANGE);
    attachInterrupt(channel_pins[3], ch4_ISR, CHANGE);
    attachInterrupt(channel_pins[4], ch5_ISR, CHANGE);
    attachInterrupt(channel_pins[5], ch6_ISR, CHANGE);
}

// ======================= SAFE READ =======================
void getReceiverValues(uint16_t *ch)
{
    noInterrupts();
    for (int i = 0; i < NUM_CHANNELS; i++)
        ch[i] = ReceiverValue[i];
    interrupts();
}
