//*****************************************************************************
//
// STEP_task.h - Prototypes for the step task.
//
//*****************************************************************************

#ifndef __STEP_TASK_H__
#define __STEP_TASK_H__

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

TaskHandle_t StepTaskHandle;

//*****************************************************************************
//
// Prototypes for the STEP task.
//
//*****************************************************************************
extern uint32_t StepTaskInit(void);
extern void StepTask(void *);

#endif 
