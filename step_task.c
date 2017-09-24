//*****************************************************************************
//
// step_task.c
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
#include "driverlib/adc.h"
#include "display.h"
#include "bsp.h"
#include "timers.h"
#include "my_timer.h"
#include "switch_task.h"

#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu6050.h"
#include "sensorlib/comp_dcm.h"
#include "step.h"


//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define STEPSTACKSIZE        128         // Stack size in words
//*****************************************************************************
//
// Default heart rate delay  (1000ms).
//
//*****************************************************************************

void StepTask(void *pvParameters) {
		int_fast32_t i32IPart[16], i32FPart[16];
	    uint_fast32_t ui32Idx, ui32CompDCMStarted;
	    float pfData[16];
	    float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;

	    //
	    // Initialize convenience pointers that clean up and clarify the code
	    // meaning. We want all the data in a single contiguous array so that
	    // we can make our pretty printing easier later.
	    //
	    pfAccel = pfData;
	    pfGyro = pfData + 3;
	    pfMag = pfData + 6;
	    pfEulers = pfData + 9;
	    pfQuaternion = pfData + 12;

	    //
	    // Enable port F used for motion interrupt.
	    //
	    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	    //
	    // The I2C1 (I2C3) peripheral must be enabled before use.
	    //
	    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	    //
	    // Configure the pin muxing for I2C1 functions on port D0 and D1.
	    //
	    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);

	    //
	    // Select the I2C function for these pins.  This function will also
	    // configure the GPIO pins pins for I2C operation, setting them to
	    // open-drain operation with weak pull-ups.  Consult the data sheet
	    // to see which functions are allocated per pin.
	    //
	    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
	    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

	    //
	    // Configure and Enable the GPIO interrupt. Used for INT signal from the
	    // MPU6050
	    //
	    ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
	    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
	    ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
	    ROM_IntEnable(INT_GPIOF);

	    //
	    // Keep only some parts of the systems running while in sleep mode.
	    // GPIOB is for the MPU6050 interrupt pin.
	    // UART0 is the virtual serial port
	    // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
	    // I2C3 is the I2C interface to the ISL29023
	    //
	    ROM_SysCtlPeripheralClockGating(true);
	    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
	    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
	    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
	    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
	    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C1);
	    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

	    //
	    // Initialize I2C3 peripheral.
	    //
	    I2CMInit(&g_sI2CInst, I2C1_BASE, INT_I2C1, 0xff, 0xff,
	             ROM_SysCtlClockGet());

	    //
	    // Initialize the MPU6050 Driver.
	    //
	    MPU6050Init(&g_sMPU6050Inst, &g_sI2CInst, MPU6050_I2C_ADDRESS,
	                MPU6050AppCallback, &g_sMPU6050Inst);

	    //
	    // Wait for transaction to complete
	    //
	    MPU6050AppI2CWait(__FILE__, __LINE__);

	    //
	    // Write application specifice sensor configuration such as filter settings
	    // and sensor range settings.
	    //
	    g_sMPU6050Inst.pui8Data[0] = MPU6050_CONFIG_DLPF_CFG_94_98;
	    g_sMPU6050Inst.pui8Data[1] = MPU6050_GYRO_CONFIG_FS_SEL_250;
	    g_sMPU6050Inst.pui8Data[2] = MPU6050_ACCEL_CONFIG_AFS_SEL_2G;
	    MPU6050Write(&g_sMPU6050Inst, MPU6050_O_CONFIG, g_sMPU6050Inst.pui8Data, 3,
	                 MPU6050AppCallback, &g_sMPU6050Inst);

	    //
	    // Wait for transaction to complete
	    //
	    MPU6050AppI2CWait(__FILE__, __LINE__);

	    //
	    // Configure the data ready interrupt pin output of the MPU6050.
	    //
	    g_sMPU6050Inst.pui8Data[0] = MPU6050_INT_PIN_CFG_INT_LEVEL |
	                                    MPU6050_INT_PIN_CFG_INT_RD_CLEAR |
	                                    MPU6050_INT_PIN_CFG_LATCH_INT_EN;
	    g_sMPU6050Inst.pui8Data[1] = MPU6050_INT_ENABLE_DATA_RDY_EN;
	    MPU6050Write(&g_sMPU6050Inst, MPU6050_O_INT_PIN_CFG,
	                 g_sMPU6050Inst.pui8Data, 2, MPU6050AppCallback,
	                 &g_sMPU6050Inst);

	    //
	    // Wait for transaction to complete
	    //
	    MPU6050AppI2CWait(__FILE__, __LINE__);

	    //
	    // Initialize the DCM system. 50 hz sample rate.
	    // accel weight = .2, gyro weight = .8, mag weight = .2
	    //
	    CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.0f);

	    UARTprintf("\033[2J\033[H");
	    UARTprintf("MPU6050 6-Axis Simple Data Application Example\n\n");
	    UARTprintf("\033[20GX\033[31G|\033[43GY\033[54G|\033[66GZ\n\n");
	    UARTprintf("Accel\033[8G|\033[31G|\033[54G|\n\n");
	    UARTprintf("Gyro\033[8G|\033[31G|\033[54G|\n\n");
	    UARTprintf("Mag\033[8G|\033[31G|\033[54G|\n\n");
	    UARTprintf("\n\033[20GRoll\033[31G|\033[43GPitch\033[54G|\033[66GYaw\n\n");
	    UARTprintf("Eulers\033[8G|\033[31G|\033[54G|\n\n");

	    UARTprintf("\n\033[17GQ1\033[26G|\033[35GQ2\033[44G|\033[53GQ3\033[62G|"
	               "\033[71GQ4\n\n");
	    UARTprintf("Q\033[8G|\033[26G|\033[44G|\033[62G|\n\n");


	    ui32CompDCMStarted = 0;

	    while(1)
	    {
	        //
	        // Go to sleep mode while waiting for data ready.
	        //
	        while(!g_vui8I2CDoneFlag)
	        {
	            ROM_SysCtlSleep();
	        }

	        //
	        // Clear the flag
	        //
	        g_vui8I2CDoneFlag = 0;

	        //
	        // Get floating point version of the Accel Data in m/s^2.
	        //
	        MPU6050DataAccelGetFloat(&g_sMPU6050Inst, pfAccel, pfAccel + 1,
	                                 pfAccel + 2);

	        //
	        // Get floating point version of angular velocities in rad/sec
	        //
	        MPU6050DataGyroGetFloat(&g_sMPU6050Inst, pfGyro, pfGyro + 1,
	                                pfGyro + 2);

	        //
	        // Get floating point version of magnetic fields strength in tesla
	        //
	//        MPU9150DataMagnetoGetFloat(&g_sMPU6050Inst, pfMag, pfMag + 1,
	//                                  pfMag + 2);

	        //
	        // Check if this is our first data ever.
	        //
	        if(ui32CompDCMStarted == 0)
	        {
	            //
	            // Set flag indicating that DCM is started.
	            // Perform the seeding of the DCM with the first data set.
	            //
	            ui32CompDCMStarted = 1;
	            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
	                                 pfMag[2]);
	            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
	                               pfAccel[2]);
	            CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
	                              pfGyro[2]);
	            CompDCMStart(&g_sCompDCMInst);
	        }
	        else
	        {
	            //
	            // DCM Is already started.  Perform the incremental update.
	            //
	            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
	                                 pfMag[2]);
	            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
	                               pfAccel[2]);
	            CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyro[0], -pfGyro[1],
	                              -pfGyro[2]);
	            CompDCMUpdate(&g_sCompDCMInst);
	        }

	        //
	        // Increment the skip counter.  Skip counter is used so we do not
	        // overflow the UART with data.
	        //
	        g_ui32PrintSkipCounter++;
	        if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
	        {
	            //
	            // Reset skip counter.
	            //
	            g_ui32PrintSkipCounter = 0;

	            //
	            // Get Euler data. (Roll Pitch Yaw)
	            //
	            CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
	                                 pfEulers + 2);

	            //
	            // Get Quaternions.
	            //
	            CompDCMComputeQuaternion(&g_sCompDCMInst, pfQuaternion);

	            //
	            // convert mag data to micro-tesla for better human interpretation.
	            //
	            pfMag[0] *= 1e6;
	            pfMag[1] *= 1e6;
	            pfMag[2] *= 1e6;

	            //
	            // Convert Eulers to degrees. 180/PI = 57.29...
	            // Convert Yaw to 0 to 360 to approximate compass headings.
	            //
	            pfEulers[0] *= 57.295779513082320876798154814105f;
	            pfEulers[1] *= 57.295779513082320876798154814105f;
	            pfEulers[2] *= 57.295779513082320876798154814105f;
	            if(pfEulers[2] < 0)
	            {
	                pfEulers[2] += 360.0f;
	            }

	            //
	            // Now drop back to using the data as a single array for the
	            // purpose of decomposing the float into a integer part and a
	            // fraction (decimal) part.
	            //
	            for(ui32Idx = 0; ui32Idx < 16; ui32Idx++)
	            {
	                //
	                // Conver float value to a integer truncating the decimal part.
	                //
	                i32IPart[ui32Idx] = (int32_t) pfData[ui32Idx];

	                //
	                // Multiply by 1000 to preserve first three decimal values.
	                // Truncates at the 3rd decimal place.
	                //
	                i32FPart[ui32Idx] = (int32_t) (pfData[ui32Idx] * 1000.0f);

	                //
	                // Subtract off the integer part from this newly formed decimal
	                // part.
	                //
	                i32FPart[ui32Idx] = i32FPart[ui32Idx] -
	                                    (i32IPart[ui32Idx] * 1000);

	                //
	                // make the decimal part a positive number for display.
	                //
	                if(i32FPart[ui32Idx] < 0)
	                {
	                    i32FPart[ui32Idx] *= -1;
	                }
	            }

	            //
	            // Print the acceleration numbers in the table.
	            //
	            UARTprintf("\033[5;17H%3d.%03d", i32IPart[0], i32FPart[0]);
	            UARTprintf("\033[5;40H%3d.%03d", i32IPart[1], i32FPart[1]);
	            UARTprintf("\033[5;63H%3d.%03d", i32IPart[2], i32FPart[2]);

	            //
	            // Print the angular velocities in the table.
	            //
	            UARTprintf("\033[7;17H%3d.%03d", i32IPart[3], i32FPart[3]);
	            UARTprintf("\033[7;40H%3d.%03d", i32IPart[4], i32FPart[4]);
	            UARTprintf("\033[7;63H%3d.%03d", i32IPart[5], i32FPart[5]);

	            //
	            // Print the magnetic data in the table.
	            //
	            UARTprintf("\033[9;17H%3d.%03d", i32IPart[6], i32FPart[6]);
	            UARTprintf("\033[9;40H%3d.%03d", i32IPart[7], i32FPart[7]);
	            UARTprintf("\033[9;63H%3d.%03d", i32IPart[8], i32FPart[8]);

	            //
	            // Print the Eulers in a table.
	            //
	            UARTprintf("\033[14;17H%3d.%03d", i32IPart[9], i32FPart[9]);
	            UARTprintf("\033[14;40H%3d.%03d", i32IPart[10], i32FPart[10]);
	            UARTprintf("\033[14;63H%3d.%03d", i32IPart[11], i32FPart[11]);

	            //
	            // Print the quaternions in a table format.
	            //
	            UARTprintf("\033[19;14H%3d.%03d", i32IPart[12], i32FPart[12]);
	            UARTprintf("\033[19;32H%3d.%03d", i32IPart[13], i32FPart[13]);
	            UARTprintf("\033[19;50H%3d.%03d", i32IPart[14], i32FPart[14]);
	            UARTprintf("\033[19;68H%3d.%03d", i32IPart[15], i32FPart[15]);

	        }
	    }

}

//*****************************************************************************
//
// Initializes the switch task.
//
//*****************************************************************************
uint32_t StepTaskInit(void) {

	if (xTaskCreate(StepTask, (const portCHAR *)"Step",
			STEPSTACKSIZE, NULL, tskIDLE_PRIORITY +
			PRIORITY_STEP_TASK , &StepTaskHandle) != pdTRUE) {
		return (1);
	}

	return (0);
}
