//*****************************************************************************
//
// switch_task.h - Prototypes for the switch task.
//
//*****************************************************************************

#ifndef __HEART_RATE_TASK_H__
#define __HEART_RATE_TASK_H__

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

TaskHandle_t HeartRateTaskHandle;

//*****************************************************************************
//
// Prototypes for the heart_rate task.
//
//*****************************************************************************
extern uint32_t HeartRateTaskInit(void);
extern void HeartRateTask(void *);

#endif // __SWITCH_TASK_H__
