/*
 * LCD Library for Stellaris/Tiva launchpads
 * All rights reserved.
 * Distributed under the BSD License
 * Copyright (c) 2015, Manolis Kiagias
 *
 * Based on ideas and code of the MSP430 Launchpad LCD Library
 * published in Co-Random thoughts blog:
 * http://cacheattack.blogspot.gr/2011/06/quick-overview-on-interfacing-msp430.html
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************/

#include "heart_rate.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//Initialize heart_rate

void HeartRateInit(void) {

// Enable the GPIO port that is used for the on-board LED.
//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

//
// Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
// enable the GPIO pin for digital function.
//
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}


void sendDataToProcessing(char symbol, int data) {
	UARTprintf("%c", symbol); // symbol prefix tells Processing what type of data is coming
	UARTprintf("%4d\n", data);
}

void ADC0Setup() {
	// The ADC0 peripheral must be enabled for use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	//
	// For this example ADC0 is used with AIN0 on port E7.
	// The actual port and pins used may be different on your part, consult
	// the data sheet for more information.  GPIO port E needs to be enabled
	// so these pins can be used.
	// TODO: change this to whichever GPIO port you are using.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//
	// Select the analog ADC function for these pins.
	// Consult the data sheet to see which functions are allocated per pin.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

	//
	// Enable sample sequence 3 with a processor signal trigger.  Sequence 3
	// will do a single sample when the processor sends a signal to start the
	// conversion.  Each ADC module has 4 programmable sequences, sequence 0
	// to sequence 3.  This example is arbitrarily using sequence 3.
	//
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

	//
	// Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
	// single-ended mode (default) and configure the interrupt flag
	// (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
	// that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
	// 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
	// sequence 0 has 8 programmable steps.  Since we are only doing a single
	// conversion using sequence 3 we will only configure step 0.  For more
	// information on the ADC sequences and steps, reference the datasheet.
	//
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
	ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

	//
	// Since sample sequence 3 is now configured, it must be enabled.
	//
	ADCSequenceEnable(ADC0_BASE, 3);

	//
	// Clear the interrupt status flag.  This is done to make sure the
	// interrupt flag is cleared before we sample.
	//
	ADCIntClear(ADC0_BASE, 3);
}

void ResultCal() {
	int i;
	sampleCounter += 2;       // keep track of the time in mS with this variable
	int N = sampleCounter - lastBeatTime; // monitor the time since the last beat to avoid noise

	//  find the peak and trough of the pulse wave
	if (Signal < thresh && N > (IBI / 5) * 3) { // avoid dichrotic noise by waiting 3/5 of last IBI
		if (Signal < T) {                        // T is the trough
			T = Signal;             // keep track of lowest point in pulse wave
		}
	}

	if (Signal > thresh && Signal > P) {   // thresh condition helps avoid noise
		P = Signal;                             // P is the peak
	}                               // keep track of highest point in pulse wave

	//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	// signal surges up in value every time there is a pulse
	if (N > 250) {                                 // avoid high frequency noise
		if ((Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3)) {
			Pulse = true;   // set the Pulse flag when we think there is a pulse
//			digitalWrite(blinkPin, HIGH);                // turn on pin 13 LED
			IBI = sampleCounter - lastBeatTime; // measure time between beats in mS
			lastBeatTime = sampleCounter;   // keep track of time for next pulse

			if (secondBeat) { // if this is the second beat, if secondBeat == TRUE
				secondBeat = false;                  // clear secondBeat flag
				for ( i = 0; i <= 9; i++) { // seed the running total to get a realisitic BPM at startup
					rate[i] = IBI;
				}
			}

			if (firstBeat) { // if it's the first time we found a beat, if firstBeat == TRUE
				firstBeat = false;                   // clear firstBeat flag
				secondBeat = true;                   // set the second beat flag
//				sei();                               // enable interrupts again
				return;                 // IBI value is unreliable so discard it
			}

			// keep a running total of the last 10 IBI values
			int runningTotal = 0;        // clear the runningTotal variable

			for ( i = 0; i <= 8; i++) {       // shift data in the rate array
				rate[i] = rate[i + 1];         // and drop the oldest IBI value
				runningTotal += rate[i];       // add up the 9 oldest IBI values
			}

			rate[9] = IBI;               // add the latest IBI to the rate array
			runningTotal += rate[9];       // add the latest IBI to runningTotal
			runningTotal /= 10;               // average the last 10 IBI values
			BPM = 60000 / runningTotal; // how many beats can fit into a minute? that's BPM!
			QS = true;                              // set Quantified Self flag
			// QS FLAG IS NOT CLEARED INSIDE THIS ISR
		}
	}

	if (Signal < thresh && Pulse == true) { // when the values are going down, the beat is over
//		digitalWrite(blinkPin, LOW);            // turn off pin 13 LED
		Pulse = false;             // reset the Pulse flag so we can do it again
		amp = P - T;                          // get amplitude of the pulse wave
		thresh = amp / 2 + T;              // set thresh at 50% of the amplitude
		P = thresh;                            // reset these for next time
		T = thresh;
	}

	if (N > 2500) {                       // if 2.5 seconds go by without a beat
		thresh = 2048;                          // set thresh default
		P = 2048;                               // set P default
		T = 2048;                               // set T default
		lastBeatTime = sampleCounter; // bring the lastBeatTime up to date
		firstBeat = true;                      // set these to avoid noise
		secondBeat = false;                    // when we get the heartbeat back
	}
}

