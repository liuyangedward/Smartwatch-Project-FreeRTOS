//*****************************************************************************
//
// WATCH_task.c - A simple flashing WATCH task.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/rgb.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "watch_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "heart_rate_task.h"

//*****************************************************************************
//
//Include BSP
//
//*****************************************************************************
#include "bsp.h"

//*****************************************************************************
//
//Include LCD driver
//
//*****************************************************************************
#include "display.h"

//*****************************************************************************
//
// The stack size for the WATCH toggle task.
//
//*****************************************************************************
#define WATCHTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// The item size and queue size for the WATCH message queue.
//
//*****************************************************************************
#define WATCH_ITEM_SIZE           sizeof(uint8_t)
#define WATCH_QUEUE_SIZE          5

//*****************************************************************************
//
// Default LCD refresh rate.
//
//*****************************************************************************
#define WATCH_TOGGLE_DELAY        1000

//*****************************************************************************
//
// The queue that holds messages sent to the WATCH task.
//
//*****************************************************************************
xQueueHandle g_pWatchQueue;

extern xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// This task toggles the user selected WATCH at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void WatchTask(void *pvParameters) {
	portTickType ui32WakeTime;
	uint32_t ui32WatchToggleDelay;
	uint8_t i8Message;

	//
	// Initialize the WATCH Toggle Delay to default value.
	//
	ui32WatchToggleDelay = WATCH_TOGGLE_DELAY;

	//
	// Get the current tick count.
	//
	ui32WakeTime = xTaskGetTickCount();

	//
	// Loop forever.
	//
	while (1) {
		//
		// Read the next message, if available on queue.
		//
		if (xQueueReceive(g_pWatchQueue, &i8Message, 0) == pdPASS) {
			//
			// If left button, update to next WATCH.
			//
			if (i8Message == LEFT_BUTTON) {

			}

			//
			// If right button, update delay time between toggles of WATCH.
			//
			if (i8Message == RIGHT_BUTTON) {

			}
		}

		//obtain the date and time from the RTC
		sec = GetClock(SEC);
		min = GetClock(MIN);
		hour = GetClock(HRS);
		day = GetClock(DAY);
		date = GetClock(DATE);
		month = GetClock(MONTH);
		year = GetClock(YEAR);

		//convert the char to decimal
		itoa(hour, hour_c, 10);
		itoa(min, min_c, 10);
		itoa(sec, sec_c, 10);
		itoa(date, date_c, 10);
		itoa(month, month_c, 10);
		itoa(year, year_c, 10);

		//	UARTprintf("%02d:%02d:%02d \n%02d/%02d/%02d\n", hour, min, sec,
		//			date, month, year);

		//zero-padding for the display
		zero_padding();

		//select the cursor position and display the date and time
		setCursorPositionLCD(0, 0);
		printLCD("TIME");
		setCursorPositionLCD(0, 5);
		printLCD(hour_c);
		setCursorPositionLCD(0, 7);
		printLCD(":");
		setCursorPositionLCD(0, 8);
		printLCD(min_c);
		setCursorPositionLCD(0, 10);
		printLCD(":");
		setCursorPositionLCD(0, 11);
		printLCD(sec_c);
		setCursorPositionLCD(1, 0);
		printLCD("DATE");
		setCursorPositionLCD(1, 5);
		printLCD(date_c);
		setCursorPositionLCD(1, 7);
		printLCD("/");
		setCursorPositionLCD(1, 8);
		printLCD(month_c);
		setCursorPositionLCD(1, 10);
		printLCD("/");
		setCursorPositionLCD(1, 11);
		printLCD(year_c);

		//enter the blocked state and become ready every 1000ms
		vTaskDelayUntil(&ui32WakeTime, ui32WatchToggleDelay / portTICK_RATE_MS);
	}
}

//*****************************************************************************
//
// Initializes the WATCH task.
//
//*****************************************************************************
uint32_t WatchTaskInit(void) {

	//
	// Create a queue for sending messages to the WATCH task.
	//
	g_pWatchQueue = xQueueCreate(WATCH_QUEUE_SIZE, WATCH_ITEM_SIZE);

	//
	// Create the WATCH task.
	//
	if (xTaskCreate(WatchTask, (const portCHAR *)"WATCH", WATCHTASKSTACKSIZE, NULL,
			tskIDLE_PRIORITY + PRIORITY_WATCH_TASK, NULL) != pdTRUE) {
		return (1);
	}

	//
	// Success.
	//
	return (0);
}
