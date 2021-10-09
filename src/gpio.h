/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define LED0_port gpioPortF // change to correct ports and pins
#define LED0_pin   4
#define LED1_port gpioPortF
#define LED1_pin   5
#define SENSOR_PORT     gpioPortD
#define SENSOR_ENABLE   15
#define DISP_PORT     gpioPortF
#define DISP_EXTCOMIN   18



// Function prototypes
void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

//For toggling the LED
void gpioToggleLED0();

//void gpioSensorEnSetOn();

/* function     : gpio_I2C
 * params       : int on_off
 * brief        : Enables/Disables I2C sensor
 * return_type  : void
 * */
void gpio_I2C(int on_off);

void gpioSetDisplayExtcomin(bool value);


#endif /* SRC_GPIO_H_ */
