/* PulsePosition Library for Teensy 3.x, LC, and 4.0
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

#if defined(__IMXRT1062__)

#include "Arduino.h"
#include "PulsePositionIMXRT.h"

// Some debug defines 
//#define DEBUG_IO_PINS
//#define DEBUG_OUTPUT

#ifdef DEBUG_IO_PINS
#define DBGdigitalWriteFast(pin, state) digitalWriteFast(pin, state);
#else
    inline void  DBGdigitalWriteFast(uint8_t pin, uint8_t state) {};
#endif



// Timing parameters, in microseconds.


// The shortest time allowed between any 2 rising edges.  This should be at
// least double TX_PULSE_WIDTH.
#define TX_MINIMUM_SIGNAL   300.0

// The longest time allowed between any 2 rising edges for a normal signal.
#define TX_MAXIMUM_SIGNAL  2500.0

// The default signal to send if nothing has been written.
#define TX_DEFAULT_SIGNAL  1500.0

// When transmitting with a single pin, the minimum space signal that marks
// the end of a frame.  Single wire receivers recognize the end of a frame
// by looking for a gap longer than the maximum data size.  When viewing the
// waveform on an oscilloscope, set the trigger "holdoff" time to slightly
// less than TX_MINIMUM_SPACE, for the most reliable display.  This parameter
// is not used when transmitting with 2 pins.
#define TX_MINIMUM_SPACE   5000.0

// The minimum total frame size.  Some servo motors or other devices may not
// work with pulses the repeat more often than 50 Hz.  To allow transmission
// as fast as possible, set this to the same as TX_MINIMUM_SIGNAL.
#define TX_MINIMUM_FRAME  20000.0

// The length of all transmitted pulses.  This must be longer than the worst
// case interrupt latency, which depends on how long any other library may
// disable interrupts.  This must also be no more than half TX_MINIMUM_SIGNAL.
// Most libraries disable interrupts for no more than a few microseconds.
// The OneWire library is a notable exception, so this may need to be lengthened
// if a library that imposes unusual interrupt latency is in use.
#define TX_PULSE_WIDTH      100.0

// When receiving, any time between rising edges longer than this will be
// treated as the end-of-frame marker.
#define RX_MINIMUM_SPACE   3500.0

// convert from microseconds to I/O clock ticks
#define CLOCKS_PER_MICROSECOND (150./4)  // pcs 8+2
#define TX_MINIMUM_SPACE_CLOCKS   (uint32_t)(TX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_FRAME_CLOCKS   (uint32_t)(TX_MINIMUM_FRAME * CLOCKS_PER_MICROSECOND)
#define TX_PULSE_WIDTH_CLOCKS     (uint32_t)(TX_PULSE_WIDTH * CLOCKS_PER_MICROSECOND)
#define TX_DEFAULT_SIGNAL_CLOCKS  (uint32_t)(TX_DEFAULT_SIGNAL * CLOCKS_PER_MICROSECOND)
#define RX_MINIMUM_SPACE_CLOCKS   (uint32_t)(RX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)

//Leave Frame stuff for now
//
//
// Output 
#define CTRL_SET TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_LENGTH |TMR_CTRL_OUTMODE(2)
#define CTRL_CLEAR TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_LENGTH |TMR_CTRL_OUTMODE(1)
#define PULSEPOSITION_MAXCHANNELS 16

///////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// PulsePositionBase - defines and code. 
//-----------------------------------------------------------------------------
PulsePositionBase * PulsePositionBase::list[10];

const PulsePositionBase::TMR_Hardware_t PulsePositionBase::hardware[] = {
	{ 6,1, &IMXRT_TMR4, &CCM_CCGR6, CCM_CCGR6_QTIMER4(CCM_CCGR_ON), IRQ_QTIMER4, &PulsePositionInput::isrTimer4, nullptr, 0},
	{ 9,2, &IMXRT_TMR4, &CCM_CCGR6, CCM_CCGR6_QTIMER4(CCM_CCGR_ON), IRQ_QTIMER4, &PulsePositionInput::isrTimer4, nullptr, 0},
    {10,0, &IMXRT_TMR1, &CCM_CCGR6, CCM_CCGR6_QTIMER1(CCM_CCGR_ON), IRQ_QTIMER1, &PulsePositionInput::isrTimer1, nullptr, 0},
    {11,2, &IMXRT_TMR1, &CCM_CCGR6, CCM_CCGR6_QTIMER1(CCM_CCGR_ON), IRQ_QTIMER1, &PulsePositionInput::isrTimer1, nullptr, 0},
    {12,1, &IMXRT_TMR1, &CCM_CCGR6, CCM_CCGR6_QTIMER1(CCM_CCGR_ON), IRQ_QTIMER1, &PulsePositionInput::isrTimer1, nullptr, 0},
    {13,0, &IMXRT_TMR2, &CCM_CCGR6, CCM_CCGR6_QTIMER2(CCM_CCGR_ON), IRQ_QTIMER2, &PulsePositionInput::isrTimer2, &IOMUXC_QTIMER2_TIMER0_SELECT_INPUT, 1 },
    {14,2, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &PulsePositionInput::isrTimer3, &IOMUXC_QTIMER3_TIMER2_SELECT_INPUT, 1 },
    {15,3, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &PulsePositionInput::isrTimer3, &IOMUXC_QTIMER3_TIMER3_SELECT_INPUT, 1 },
    {18,1, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &PulsePositionInput::isrTimer3, &IOMUXC_QTIMER3_TIMER1_SELECT_INPUT, 0 },
    {19,0, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &PulsePositionInput::isrTimer3, &IOMUXC_QTIMER3_TIMER0_SELECT_INPUT, 1 }
};

const uint8_t PulsePositionBase::_hardware_count =  (sizeof(PulsePositionBase::hardware)/sizeof(PulsePositionBase::hardware[0]));

inline void PulsePositionBase::checkAndProcessTimerCHInPending(uint8_t index, volatile IMXRT_TMR_CH_t *tmr_ch) {
 	if (((tmr_ch->CSCTRL & (TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF1EN)) == (TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF1EN)) 
			|| ((tmr_ch->SCTRL & (TMR_SCTRL_IEF | TMR_SCTRL_IEFIE)) == (TMR_SCTRL_IEF | TMR_SCTRL_IEFIE))) {
 		// Interrupt condtion is set. 
 		if 	(list[index]) {
 			list[index]->isr();
 		} else {
 			// no one registered to process clear out the conditions.
 			tmr_ch->CSCTRL &= ~TMR_CSCTRL_TCF1;
			tmr_ch->SCTRL &= ~TMR_SCTRL_IEF;
 		}
 	}
}


void PulsePositionBase::isrTimer1()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(2, &IMXRT_TMR1.CH[0]);
	checkAndProcessTimerCHInPending(3, &IMXRT_TMR1.CH[2]);
	checkAndProcessTimerCHInPending(4, &IMXRT_TMR1.CH[1]);
	asm volatile ("dsb");  // wait for clear  memory barrier
	DBGdigitalWriteFast(2, LOW);
}

void PulsePositionBase::isrTimer2()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(5, &IMXRT_TMR2.CH[0]);
	asm volatile ("dsb");  // wait for clear  memory barrier
	DBGdigitalWriteFast(2, LOW);

}

void PulsePositionBase::isrTimer3()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(6, &IMXRT_TMR3.CH[2]);
	checkAndProcessTimerCHInPending(7, &IMXRT_TMR3.CH[3]);
	checkAndProcessTimerCHInPending(8, &IMXRT_TMR3.CH[1]);
	checkAndProcessTimerCHInPending(9, &IMXRT_TMR3.CH[0]);
	asm volatile ("dsb");  // wait for clear  memory barrier
	DBGdigitalWriteFast(2, LOW);
}
void PulsePositionBase::isrTimer4()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(0, &IMXRT_TMR4.CH[1]);
	checkAndProcessTimerCHInPending(1, &IMXRT_TMR4.CH[2]);
	asm volatile ("dsb");  // wait for clear  memory barrier
	DBGdigitalWriteFast(2, LOW);
}
//-----------------------------------------------------------------------------
// PulsePositionOutput 
//-----------------------------------------------------------------------------

PulsePositionOutput::PulsePositionOutput(void)
{
	pulse_width[0] = TX_MINIMUM_FRAME_CLOCKS;
	for (int i=1; i <= PULSEPOSITION_MAXCHANNELS; i++) {
		pulse_width[i] = TX_DEFAULT_SIGNAL_CLOCKS;
	}
}

PulsePositionOutput::PulsePositionOutput(int polarity)
{
	pulse_width[0] = TX_MINIMUM_FRAME_CLOCKS;
	for (int i=1; i <= PULSEPOSITION_MAXCHANNELS; i++) {
		pulse_width[i] = TX_DEFAULT_SIGNAL_CLOCKS;
	}
	if (polarity == FALLING) {
		outPolarity = 0;
	} else {
		outPolarity = 1;
	}
}

bool PulsePositionOutput::begin(uint8_t txPin)
{
	return begin(txPin, 255);
}

bool PulsePositionOutput::begin(uint8_t txPin, uint32_t _framePin)
{
#ifdef DEBUG_OUTPUT
	Serial.println(txPin);
#endif
	for (idx_channel = 0; idx_channel < _hardware_count; idx_channel++) {
		if (hardware[idx_channel].pin == txPin) break; 
	}
	if (idx_channel == _hardware_count) return false;

	// make sure the appropriate clock gate is enabled.
	*hardware[idx_channel].clock_gate_register |= hardware[idx_channel].clock_gate_mask;

	uint8_t channel =  hardware[idx_channel].channel;
	volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];
	
	tmr_ch->CTRL = 0; // stop
	tmr_ch->CNTR = 0;
	tmr_ch->LOAD = 0;

  //framePin = 2;   // optional select a framePin
	if (_framePin < NUM_DIGITAL_PINS) {
		framePin = _framePin;
		pinMode(framePin,OUTPUT);
		digitalWriteFast(framePin,HIGH);
	}
	
	tmr_ch->COMP1 = 200;  // first time
	state = 0;
	tmr_ch->CMPLD1 = TX_PULSE_WIDTH_CLOCKS;
	//TMR1_SCTRL0 = TMR_SCTRL_OEN | TMR_SCTRL_OPS to make falling
	if(outPolarity == 0){
	  tmr_ch->SCTRL = TMR_SCTRL_OEN | TMR_SCTRL_OPS;
	} else {
	  tmr_ch->SCTRL = TMR_SCTRL_OEN ;
	}

	tmr_ch->CSCTRL = TMR_CSCTRL_CL1(1);
	attachInterruptVector(hardware[idx_channel].interrupt, hardware[idx_channel].isr);
	tmr_ch->CSCTRL &= ~(TMR_CSCTRL_TCF1);  // clear
	tmr_ch->CSCTRL |= TMR_CSCTRL_TCF1EN;  // enable interrupt
	NVIC_SET_PRIORITY(hardware[idx_channel].interrupt, 32);
	NVIC_ENABLE_IRQ(hardware[idx_channel].interrupt);
	tmr_ch->CTRL =  CTRL_SET;

	list[idx_channel] = this;
	
	//set Mux for Tx Pin - all timers on ALT1
	*(portConfigRegister(txPin)) = 1;

	return true;
}

bool PulsePositionOutput::write(uint8_t channel, float microseconds)
{
  uint32_t i, sum, space, clocks, num_channels;

  if (channel < 1 || channel > PULSEPOSITION_MAXCHANNELS) return false;
  if (microseconds < TX_MINIMUM_SIGNAL || microseconds > TX_MAXIMUM_SIGNAL) return false;
  clocks = microseconds * CLOCKS_PER_MICROSECOND;
  num_channels = total_channels;
  if (channel > num_channels) num_channels = channel;
  sum = clocks;
  for (i = 1; i < channel; i++) sum += pulse_width[i];
  for (i = channel + 1; i <= num_channels; i++) sum += pulse_width[i];
  if (sum < TX_MINIMUM_FRAME_CLOCKS - TX_MINIMUM_SPACE_CLOCKS) {
    space = TX_MINIMUM_FRAME_CLOCKS - sum;
  } else {
    if (framePin < NUM_DIGITAL_PINS) {
      space = TX_PULSE_WIDTH_CLOCKS;
    } else {
      space = TX_MINIMUM_SPACE_CLOCKS;
    }
  }
  __disable_irq();
  pulse_width[0] = space;
  pulse_width[channel] = clocks;
  total_channels = num_channels;
  __enable_irq();
  return true;
}


void PulsePositionOutput::isr() 
{
	DBGdigitalWriteFast(3, HIGH);
  uint8_t channel = hardware[idx_channel].channel;
  volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];
  ticks++;

  // Clear out the Compare match interrupt. 
  tmr_ch->CSCTRL &= ~(TMR_CSCTRL_TCF1);
  
  if (state == 0) {
    // pin was just set high, schedule it to go low
	
   tmr_ch->COMP1 = tmr_ch->CMPLD1 = TX_PULSE_WIDTH_CLOCKS;
    tmr_ch->CTRL =  CTRL_CLEAR;
    state = 1;
  } else {
    // pin just went low
    uint32_t width, channel;
    if (state == 1) {
      channel = current_channel;
      if (channel == 0) {
        total_channels_buffer = total_channels;
        for (uint32_t i = 0; i <= total_channels_buffer; i++) {
          pulse_buffer[i] = pulse_width[i];
        }
      }
      width = pulse_buffer[channel] - TX_PULSE_WIDTH_CLOCKS;
      if (++channel > total_channels_buffer) {
        channel = 0;
      }
      if (framePin < NUM_DIGITAL_PINS) {
        if (channel == 1) {
          digitalWriteFast(framePin,HIGH);
        } else {
          digitalWriteFast(framePin,LOW);
        }
      }
      current_channel = channel;
    } else {
      width = pulse_remaining;
    }
    if (width <= 60000) {
      tmr_ch->COMP1 = tmr_ch->CMPLD1 = width;
      tmr_ch->CTRL =  CTRL_SET; // set on compare match & interrupt
      state = 0;
    } else {
      tmr_ch->COMP1 =tmr_ch->CMPLD1 = 58000;
      tmr_ch->CTRL =  CTRL_CLEAR; // clear on compare match & interrupt
      pulse_remaining = width - 58000;
      state = 2;
    }
  }

	DBGdigitalWriteFast(3, LOW);
}


//-----------------------------------------------------------------------------
// PulsePositionOutput 
//-----------------------------------------------------------------------------

PulsePositionInput::PulsePositionInput(void)
{
	outPolarity = 1;
}

PulsePositionInput::PulsePositionInput(int polarity)
{
	if (polarity == FALLING) {
		outPolarity = 0;
	} else {
		outPolarity = 1;
	}
}

bool PulsePositionInput::begin(uint8_t rxPin)
{
	for (idx_channel = 0; idx_channel < _hardware_count; idx_channel++) {
		if (hardware[idx_channel].pin == rxPin) break; 
	}
	if (idx_channel == _hardware_count) return false;

	// make sure the appropriate clock gate is enabled.
	*hardware[idx_channel].clock_gate_register |= hardware[idx_channel].clock_gate_mask;

	uint8_t channel =  hardware[idx_channel].channel;
	volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];

	tmr_ch->CTRL = 0; // stop
	tmr_ch->CNTR = 0;
	tmr_ch->LOAD = 0;
	tmr_ch->CSCTRL = 0;
	tmr_ch->LOAD = 0;  // start val after compare
	tmr_ch->COMP1 = 0xffff;  // count up to this val, interrupt,  and start again
	tmr_ch->CMPLD1 = 0xffff;

	if(outPolarity == 0){
	  tmr_ch->SCTRL = TMR_SCTRL_CAPTURE_MODE(1) | TMR_SCTRL_IPS;  //falling
	} else {
		tmr_ch->SCTRL = TMR_SCTRL_CAPTURE_MODE(1);  //rising
	}

	attachInterruptVector(hardware[idx_channel].interrupt, hardware[idx_channel].isr);
#if 1
	// uses timer match condition. 
	tmr_ch->SCTRL |= TMR_SCTRL_IEFIE;  // enable compare interrupt as well as overflow
	tmr_ch->CSCTRL = TMR_CSCTRL_TCF1EN;  // enable capture interrupt
#else	
	// tries to use overflow condition
	tmr_ch->SCTRL |= TMR_SCTRL_IEFIE | TMR_SCTRL_TOFIE;  // enable compare interrupt as well as overflow
	// tmr_ch->CSCTRL = TMR_CSCTRL_TCF1EN;  // enable capture interrupt
#endif
	NVIC_SET_PRIORITY(hardware[idx_channel].interrupt, 32);
	NVIC_ENABLE_IRQ(hardware[idx_channel].interrupt);
	
	tmr_ch->CTRL =  TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_SCS(channel) | TMR_CTRL_LENGTH ; // prescale
	//tmr_ch->CTRL =  TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_SCS(channel) ; // prescale
	
	list[idx_channel] = this;
	
#ifdef DEBUG_OUTPUT
	Serial.printf("PulsePositionInput::begin pin:%d idx: %d CH:%d SC:%x CSC:%x\n", rxPin, idx_channel, channel, tmr_ch->SCTRL, tmr_ch->CSCTRL); Serial.flush();
	//set Mux for Tx Pin - all timers on ALT1
	Serial.printf("Select Input Regster: %x %d\n", (uint32_t)hardware[idx_channel].select_input_register, hardware[idx_channel].select_val); Serial.flush();
#endif
	if (hardware[idx_channel].select_input_register) {
		*hardware[idx_channel].select_input_register = hardware[idx_channel].select_val;
#ifdef DEBUG_OUTPUT
		Serial.println("Select Input completed");Serial.flush();
#endif
	}
	*(portConfigRegister(rxPin)) = 1 | 0x10;

#ifdef DEBUG_OUTPUT
	Serial.printf("  CP1:%x CP2:%x CAPT:%x LOAD:%x \n", (uint16_t)tmr_ch->COMP1, (uint16_t)tmr_ch->COMP2,
		(uint16_t)tmr_ch->CAPT, (uint16_t)tmr_ch->LOAD); 
	Serial.flush();
	Serial.printf("  HOLD:%x CNTR:%x CTRL:%x  SCTRL:%x\n", (uint16_t)tmr_ch->HOLD, (uint16_t)tmr_ch->CNTR, (uint16_t)tmr_ch->CTRL, (uint32_t)tmr_ch->SCTRL); 
	Serial.flush();

	Serial.printf(" CMPLD1:%x, CMPLD2:%x, FILT:%x DMA:%x ENBL:%x\n",tmr_ch->CMPLD1,
		tmr_ch->CMPLD2, tmr_ch->FILT, tmr_ch->DMA, tmr_ch->ENBL);
	Serial.flush();
#endif


	
	return true;
}

int PulsePositionInput::available(void)
{
	uint32_t total;
	bool flag;

	__disable_irq();
	flag = available_flag;
	total = total_channels;
	__enable_irq();
	if (flag) return total;
	return -1;
}

float PulsePositionInput::read(uint8_t channel)
{
	uint32_t total, index, value=0;

	if (channel == 0) return 0.0;
	index = channel - 1;
	__disable_irq();
	total = total_channels;
	if (index < total) value = pulse_buffer[index];
	if (channel >= total) available_flag = false;
	__enable_irq();
	return (float)value / (float)CLOCKS_PER_MICROSECOND;
}


void PulsePositionInput::isr() {  // capture and compare
  DBGdigitalWriteFast(4, HIGH);
  uint8_t channel = hardware[idx_channel].channel;
  volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];
  
#if 1
  // uses match
  // tries to use overflow 
  if (tmr_ch->CSCTRL & TMR_CSCTRL_TCF1) { // compare rollover
    tmr_ch->CSCTRL &= ~(TMR_CSCTRL_TCF1);  // clear
    overflow_count++;
    overflow_inc = true;
  }
#else 
  // tries to use overflow 
  if (tmr_ch->SCTRL & TMR_SCTRL_TOF) { // compare rollover
    tmr_ch->SCTRL &= ~(TMR_SCTRL_TOF);  // clear
    overflow_count++;
    overflow_inc = true;
  }
#endif  
  if (tmr_ch->SCTRL & TMR_SCTRL_IEF) { // capture
    uint32_t val, count;
    tmr_ch->SCTRL &= ~(TMR_SCTRL_IEF);  // clear
    val = tmr_ch->CAPT;
    count = overflow_count;
    if (val > 0xE000 && overflow_inc) count--;
    val |= (count << 16);
    count = val - prev;
    prev = val;
    if (count >= RX_MINIMUM_SPACE_CLOCKS) {
      if (write_index < 255) {
        for (int i = 0; i < write_index; i++) {
          pulse_buffer[i] = pulse_width[i];
        }
        total_channels = write_index;
        available_flag = true;
      }
      write_index = 0;
    } else {
      if (write_index < PULSEPOSITION_MAXCHANNELS) {
        pulse_width[write_index++] = count;
      }
    }
  }
  ticks++;
  asm volatile ("dsb");  // wait for clear  memory barrier
  overflow_inc = false;
  DBGdigitalWriteFast(4, LOW);
}

#endif

