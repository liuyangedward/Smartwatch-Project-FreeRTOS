//*****************************************************************************
//
// bsp.h - Defines and Macros for the RTC module
//
//*****************************************************************************

#ifndef __BSP_H__
#define __BSP_H__

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

/* Board Support Package for the EK-TM4C123GXL board */
//Defines for DS1307
#define SLAVE_ADDRESS 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
#define SYS_CLOCK_HZ 16000000
//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint32_t sec, min, hour, day, date, month, year;
char sec_c[2];
char min_c[2];
char hour_c[2];
char day_c[2];
char date_c[2];
char month_c[2];
char year_c[2];
//*****************************************************************************
//
// RTC API function prototypes
//
//*****************************************************************************
extern void InitI2C0(void);
extern void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
extern uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);
extern unsigned char dec2bcd(unsigned char val);
extern unsigned char bcd2dec(unsigned char val);
extern void SetTimeDate(unsigned char sec, unsigned char min,
		unsigned char hour, unsigned char day, unsigned char date,
		unsigned char month, unsigned char year);
extern unsigned char GetClock(unsigned char reg);
extern void ConfigureUART(void);
extern char *itoa(int value, char *result, int base);
extern uint8_t strlen(const char *s);
extern void zero_padding(void);


#endif // __BSP_H__
