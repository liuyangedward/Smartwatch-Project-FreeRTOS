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
#include <stdbool.h>

#ifndef LIB_HEART_RATE_H_
#define LIB_HEART_RATE_H_

//  VARIABLES
int pulsePin;          // Pulse Sensor purple wire connected to analog pin 0
int blinkPin;                // pin to blink led at each beat
int fadePin;             // pin to do fancy classy fading blink at each beat
int fadeRate;                 // used to fade LED on with PWM on fadePin


int test_num[1];


// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI; // holds the time between beats, the Inter-Beat Interval
volatile bool Pulse; // true when pulse wave is high, false when it's low
volatile bool QS;        // becomes true when Arduoino finds a beat.

volatile int rate[10];                    // used to hold last ten IBI values
volatile unsigned long sampleCounter;      // used to determine pulse timing
volatile unsigned long lastBeatTime; // used to find the inter beat interval
volatile int P;                      // used to find peak in pulse wave
volatile int T;                     // used to find trough in pulse wave
volatile int thresh;          // used to find instant moment of heart beat
volatile int amp;              // used to hold amplitude of pulse waveform
volatile bool firstBeat; // used to seed rate array so we startup with reasonable BPM
volatile bool secondBeat; // used to seed rate array so we startup with reasonable BPM

extern void ledFadeToBeat(void);
extern void sendDataToProcessing(char, int);
extern void HeartRateInit(void);
extern void ADC0Setup(void);
extern void ResultCal(void);

#endif /* LIB_DISPLAY_H_ */
