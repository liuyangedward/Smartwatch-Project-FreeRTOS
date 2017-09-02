//*****************************************************************************
//
// bsp.c - Driver for the RTC module
//
// Copyright (c) 2006-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.3.156 of the Tiva Peripheral Driver Library.
//
//*****************************************************************************
#include <stdio.h>

#include "bsp.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "display.h"

//initialize I2C module 0
void InitI2C0(void) {
	//enable I2C module 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	//reset module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
	//enable GPIO peripheral that contains I2C 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	// Select the I2C function for these pins.
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	// Enable and initialize the I2C0 master module. Use the system clock for
	// the I2C0 module. The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.
	I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
	//clear I2C FIFOs
	HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...) {
	// Tell the master module what address it will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
	//stores list of variable number of arguments
	va_list vargs;
	//specifies the va_list to "open" and the last fixed argument
	//so vargs knows where to start looking
	va_start(vargs, num_of_args);
	//put data to be sent into FIFO
	I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
	//if there is only one argument, we only need to use the
	//single send I2C function
	if (num_of_args == 1) {
		//Initiate send of data from the MCU
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		// Wait until MCU is done transferring.
		while (I2CMasterBusy(I2C0_BASE))
			;
		//"close" variable argument list
		va_end(vargs);
	}
	//otherwise, we start transmission of multiple bytes on the
	//I2C bus
	else {
		//Initiate send of data from the MCU
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		// Wait until MCU is done transferring.
		while (I2CMasterBusy(I2C0_BASE))
			;
		//send num_of_args-2 pieces of data, using the
		//BURST_SEND_CONT command of the I2C module
		unsigned char i;
		for (i = 1; i < (num_of_args - 1); i++) {
			//put next piece of data into I2C FIFO
			I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
			//send next data that was just placed into FIFO
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
			// Wait until MCU is done transferring.
			while (I2CMasterBusy(I2C0_BASE))
				;
		}
		//put last piece of data into I2C FIFO
		I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
		//send next data that was just placed into FIFO
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		// Wait until MCU is done transferring.
		while (I2CMasterBusy(I2C0_BASE))
			;
		//"close" variable args list
		va_end(vargs);
	}
}

//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg) {
	//specify that we are writing (a register address) to the
	//slave device
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
	//specify register to be read
	I2CMasterDataPut(I2C0_BASE, reg);
	//send control byte and register address byte to slave device
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	//wait for MCU to finish transaction
	while (I2CMasterBusy(I2C0_BASE))
		;
	//specify that we are going to read from slave device
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
	//send control byte and read from the register we
	//specified
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	//wait for MCU to finish transaction
	while (I2CMasterBusy(I2C0_BASE))
		;
	//return data pulled from the specified register
	return I2CMasterDataGet(I2C0_BASE);
}

// convert decimal to binary
unsigned char dec2bcd(unsigned char val) {
	return (((val / 10) << 4) | (val % 10));
}

// convert binary to decimal
unsigned char bcd2dec(unsigned char val) {
	return (((val & 0xF0) >> 4) * 10) + (val & 0x0F);
}

//Set Time and Date
void SetTimeDate(unsigned char sec, unsigned char min, unsigned char hour,
		unsigned char day, unsigned char date, unsigned char month,
		unsigned char year) {
	I2CSend(SLAVE_ADDRESS, 8, SEC, dec2bcd(sec), dec2bcd(min), dec2bcd(hour),
			dec2bcd(day), dec2bcd(date), dec2bcd(month), dec2bcd(year));
}

//Get Time and Date
//uint32_t GetClock(unsigned char reg) {
//	return I2CReceive(SLAVE_ADDRESS, reg);
//}

unsigned char GetClock(unsigned char reg) {
	unsigned char clockData = I2CReceive(SLAVE_ADDRESS, reg);
	return bcd2dec(clockData);
}

//Initialize UART
void ConfigureUART(void) {
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//
	// Enable UART0
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	//
	// Configure GPIO Pins for UART mode.
	//
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	//
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);
}

// integer to string conversion
char *itoa(int value, char *result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ =
				"zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35
						+ (tmp_value - value * base)];
	} while (value);

	// Apply negative sign
	if (tmp_value < 0)
		*ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

//my strlen function
uint8_t strlen(const char *s) {
	uint8_t i;
	for (i = 0; s[i] != '\0'; i++)
		;
	return i;
}

//zero-padding
void zero_padding(void) {
	if (strlen(hour_c) < 2) {
		hour_c[1] = hour_c[0];
		hour_c[0] = '0';
	}
	if (strlen(min_c) < 2) {
		min_c[1] = min_c[0];
		min_c[0] = '0';
	}
	if (strlen(sec_c) < 2) {
		sec_c[1] = sec_c[0];
		sec_c[0] = '0';
	}
	if (strlen(date_c) < 2) {
			date_c[1] = date_c[0];
			date_c[0] = '0';
		}
	if (strlen(month_c) < 2) {
			month_c[1] = month_c[0];
			month_c[0] = '0';
		}
	if (strlen(year_c) < 2) {
			year_c[1] = year_c[0];
			year_c[0] = '0';
		}
}

