//*****************************************************************************
//
// my_timer.h - Prototypes for the my_timer task.
//
//*****************************************************************************

#ifndef __MY_TIMER_H__
#define __MY_TIMER_H__

#include "priorities.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS.h"

#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 2 )  //2ms
//Timer global variable
TimerHandle_t xAutoReloadTimer1;
BaseType_t xTimer1Started;

static int count = 0;

//*****************************************************************************
//
// Prototypes for the switch task.
//
//*****************************************************************************
extern void TimerInit(void);
extern void prvAutoReloadTimerCallback(TimerHandle_t);

#endif // __SWITCH_TASK_H__
