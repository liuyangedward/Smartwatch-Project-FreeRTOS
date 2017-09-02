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

#ifndef LIB_DISPLAY_H_
#define LIB_DISPLAY_H_

#define RS GPIO_PIN_4 // Pin 5
#define EN GPIO_PIN_5 // Pin 6
#define D4 GPIO_PIN_0 // Pin 23
#define D5 GPIO_PIN_1 // Pin 24
#define D6 GPIO_PIN_2 // Pin 25
#define D7 GPIO_PIN_3 // Pin 26
#define ALLDATAPINS  D7 | D6 | D5 | D4
#define ALLCONTROLPINS RS | EN

#define DATA_PORT_BASE GPIO_PORTD_BASE
#define CMD_PORT_BASE GPIO_PORTE_BASE
#define DATA_PERIPH SYSCTL_PERIPH_GPIOD
#define CMD_PERIPH SYSCTL_PERIPH_GPIOE

extern void pulseLCD(void);
extern void sendByte(char, int);
extern void setCursorPositionLCD(char, char);
extern void clearLCD(void);
extern void initLCD(void);
extern void printLCD(char*);
extern void setBlockCursorLCD(void);
extern void setLineCursorLCD(void);
extern void cursorOnLCD(void);
extern void cursorOffLCD(void);
extern void displayOffLCD(void);
extern void displayOnLCD(void);
extern void homeLCD(void);

#define TRUE 1
#define FALSE 0

#endif /* LIB_DISPLAY_H_ */
