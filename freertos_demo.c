//*****************************************************************************
//
// freertos_demo.c - The FreeRTOS version of the Smart watch project
//
//*****************************************************************************

//*****************************************************************************
//
//Include files from the standard header files and the TI driver library
//
//*****************************************************************************
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

//*****************************************************************************
//
//Include files from the TI driver library
//
//*****************************************************************************
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"

//*****************************************************************************
//
//Include files for FreeRTOS
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "watch_task.h"
#include "switch_task.h"
#include "heart_rate_task.h"
//*****************************************************************************
//
//Include IMU (MPU6050) driver
//
//*****************************************************************************
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu6050.h"
#include "sensorlib/comp_dcm.h"
#include "drivers/rgb.h"

//*****************************************************************************
//
//Include RTC (DS1307) driver
//
//*****************************************************************************
#include "bsp.h"

//*****************************************************************************
//
//Include LCD (HD44780) driver
//
//*****************************************************************************
#include "display.h"

//*****************************************************************************
//
//Include Pulse Sensor driver
//
//*****************************************************************************
#include "heart_rate.h"

//*****************************************************************************
//
//Include my_timer.h
//
//*****************************************************************************
#include "my_timer.h"

//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName) {
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while (1) {
	}
}



//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int main(void) {
	// Initialize the I2C between the RTC and MCU
	InitI2C0();

	//initialize LCD
	initLCD();

	// Initialize the UART and configure it for 115,200, 8-N-1 operation.
	ConfigureUART();

	//Initialize GPIO pins for pulse sensor
	HeartRateInit();

	//Setup the ADC0 SS3 for pulse sensor
	ADC0Setup();

	//Send the initial date and time to the RTC
	SetTimeDate(50, 41, 22, 2, 29, 11, 16);

	//Create a mutex to guard the UART.
	g_pUARTSemaphore = xSemaphoreCreateMutex();

	xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
	UARTprintf("Hi\n");
	xSemaphoreGive(g_pUARTSemaphore);

// Create the watch task.
	if (WatchTaskInit() != 0) {
		while (1) {
		}
	}

// Create the switch task.
	if (SwitchTaskInit() != 0) {
		while (1) {
		}
	}

// Start the scheduler.  This should not return.
	vTaskStartScheduler();

// In case the scheduler returns for some reason, print an error and loop
// forever.

	while (1) {
	}
}
