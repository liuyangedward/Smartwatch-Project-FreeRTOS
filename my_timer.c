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
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "heart_rate.h"
#include "driverlib/adc.h"

#include "display.h"

#include "bsp.h"
#include "my_timer.h"

void prvAutoReloadTimerCallback(TimerHandle_t xTimer) {

	// Trigger the ADC conversion.
	//
	ADCProcessorTrigger(ADC0_BASE, 3);

	//
	// Wait for conversion to be completed.
	//
	while (!ADCIntStatus(ADC0_BASE, 3, false)) {
	}

	//
	// Clear the ADC interrupt flag.
	//
	ADCIntClear(ADC0_BASE, 3);

	//
	// Read ADC Value.
	//
	//	ADCSequenceDataGet(ADC0_BASE, 3, (uint32_t*) Signal);

	ADCSequenceDataGet(ADC0_BASE, 3, &Signal);

	//Process the ADC value
	ResultCal();

	UARTprintf("AIN0 = %4d    BPM = %4d\r ", Signal, BPM);

	setCursorPositionLCD(1, 0);

	char signal_c[5];
	char bpm_c[5];
	itoa(BPM, bpm_c, 10);

	printLCD(bpm_c);


}

//Create a timer

void TimerInit(void) {
	xAutoReloadTimer1 = xTimerCreate("AutoReload", mainAUTO_RELOAD_TIMER_PERIOD,
	pdTRUE, 0, prvAutoReloadTimerCallback);
}

