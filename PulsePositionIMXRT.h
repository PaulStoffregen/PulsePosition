/* PulsePosition Library for Teensy 3.1
 * High resolution input and output of PPM encoded signals
 * http://www.pjrc.com/teensy/td_libs_PulsePosition.html
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy
 * boards.  Please support PJRC's efforts to develop open source software by
 * purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __PULSE_POSITION_IMXRT_H__
#define __PULSE_POSITION_IMXRT_H__
 
#if defined(__IMXRT1062__)

#include <Arduino.h>

#define PULSEPOSITION_MAXCHANNELS 16

class PulsePositionBase
{
public:

protected:
	static PulsePositionBase *list[10];
	virtual void isr() = 0;

	typedef struct {
		uint8_t 		pin;
		uint8_t			channel;
		volatile IMXRT_TMR_t* tmr;
		volatile uint32_t *clock_gate_register;
		uint32_t 		clock_gate_mask;
		IRQ_NUMBER_t	interrupt;
		void     		(*isr)();
		volatile uint32_t	*select_input_register; // Which register controls the selection
		const uint32_t		select_val;	// Value for that selection
	} TMR_Hardware_t;

	static const TMR_Hardware_t hardware[];
	static const uint8_t _hardware_count;

	// static class functions

	static void isrTimer1();
	static void isrTimer2();
	static void isrTimer3();
	static void isrTimer4();
	static inline void checkAndProcessTimerCHInPending(uint8_t index, volatile IMXRT_TMR_CH_t *tmr_ch);
};


class PulsePositionOutput : public PulsePositionBase
{
public:
	PulsePositionOutput(void);
	PulsePositionOutput(int polarity);
	bool begin(uint8_t txPin); // txPin can be 6,9,10,11,12,13,14,15,18,19
	bool begin(uint8_t txPin, uint32_t _framePin);
	bool write(uint8_t channel, float microseconds);

private:
	uint8_t outPolarity = 1;  // Polarity rising
	uint8_t inPolarity = 1;
	
	volatile uint8_t framePinMask;
	uint32_t state, total_channels, total_channels_buffer, pulse_remaining,
			 current_channel, framePin = 255;
	volatile uint32_t ticks;
	
	uint32_t pulse_width[PULSEPOSITION_MAXCHANNELS + 1];
	uint32_t pulse_buffer[PULSEPOSITION_MAXCHANNELS + 1];

	// member variables...
	uint16_t idx_channel;
	virtual void isr();
};


class PulsePositionInput : public PulsePositionBase
{
public:
	PulsePositionInput(void);
	PulsePositionInput(int polarity);
	bool begin(uint8_t rxPin); // rxPin can be 6,9,10,11,12,13,14,15,18,19
	int available(void);
	float read(uint8_t channel);
	
private:
	uint32_t pulse_width[PULSEPOSITION_MAXCHANNELS+1];
	uint32_t pulse_buffer[PULSEPOSITION_MAXCHANNELS+1];
	uint32_t prev;
	uint8_t write_index;
	uint8_t total_channels;
	uint8_t outPolarity = 1;  // Polarity rising
	uint8_t inPolarity = 1;
	volatile uint32_t ticks, overflow_count;
	volatile bool overflow_inc, available_flag;
	static uint8_t channelmask;
	// member variables...
	uint16_t idx_channel;
	virtual void isr();
};

#endif
#endif