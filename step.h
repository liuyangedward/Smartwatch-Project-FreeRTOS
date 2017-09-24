#include <stdbool.h>
#include <stdint.h>

#ifndef LIB_STEP_H_
#define LIB_STEP_H_


//*****************************************************************************
//
// Define MPU6050 I2C Address.
//
//*****************************************************************************
#define MPU6050_I2C_ADDRESS     0x69 //68

//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU6050 g_sMPU6050Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global flags to alert main that MPU6050 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU6050 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU6050 data is ready to be retrieved.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        10

uint32_t g_ui32PrintSkipCounter;


extern void MPU6050AppCallback(void, uint_fast8_t);
extern void IntGPIOb(void);
extern void MPU6050I2CIntHandler(void);
extern void MPU6050AppErrorHandler(char*, uint_fast32_t);
extern void MPU6050AppI2CWait(char *, uint_fast32_t);

#endif /* LIB_DISPLAY_H_ */



