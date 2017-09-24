# Smartwatch-Project-FreeRTOS
This is the FreeRTOS version. Another bare-metal version (with less accessories) can be found in a different repository.

Microprocessor: ARM Cortex-M4F  (Tiva-C Luanchpad)

Key accessories: an LCD (HD44780), an external RTC (DS1307), a heart-rate sensor (from pulsesensor.com) and an inertia measurement unit(MPU6050).  

By default, the LCD shows the current date&time. If you press left-button on the Tiva-C launchpad, the smart-watch will start measuring your heart rate. The heart-rate is calculated using a 15-secdons running average of you heart beat. If you press right-button, the smart-watch will tell you how many steps you've walked and an estimation of the total distance.
