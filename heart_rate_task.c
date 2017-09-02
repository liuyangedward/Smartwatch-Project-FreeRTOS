//*****************************************************************************
//
// switch_task.c - A simple switch task to process the buttons.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "heart_rate_task.h"

#include "driverlib/uart.h"

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "heart_rate.h"
#include "driverlib/adc.h"

#include "display.h"

#include "bsp.h"
#include "timers.h"
#include "my_timer.h"

#include "switch_task.h"


//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define HEARTRATETASKSTACKSIZE        128         // Stack size in words
//*****************************************************************************
//
// Default heart rate delay  (1000ms).
//
//*****************************************************************************

void HeartRateTask(void *pvParameters) {

	//initial the values
	sampleCounter = 0;          // used to determine pulse timing
	lastBeatTime = 0;           // used to find the inter beat interval
	P = 2048;                      // used to find peak in pulse wave
	T = 2048;                     // used to find trough in pulse wave
	thresh = 2048;                // used to find instant moment of heart beat
	amp = 100;                   // used to hold amplitude of pulse waveform
	firstBeat = true; // used to seed rate array so we startup with reasonable BPM
	secondBeat = false; // used to seed rate array so we startup with reasonable BPM

	IBI = 600;          // holds the time between beats, the Inter-Beat Interval
	Pulse = false;     // true when pulse wave is high, false when it's low
	QS = false;        // becomes true when Arduoino finds a beat.

	//Create a timer
	TimerInit();

	if (xAutoReloadTimer1 != NULL) {
		xTimer1Started = xTimerStart(xAutoReloadTimer1, 0);
	}

	while (1) {


	}

}

//*****************************************************************************
//
// Initializes the switch task.
//
//*****************************************************************************
uint32_t HeartRateTaskInit(void) {

	if (xTaskCreate(HeartRateTask, (const portCHAR *)"HeartRate",
			HEARTRATETASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
			PRIORITY_HEART_RATE_TASK , &HeartRateTaskHandle) != pdTRUE) {
		return (1);
	}

	return (0);
}
