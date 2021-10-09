/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

 */




#include "gpio.h"

// Set GPIO drive strengths and modes of operation
//NOTE: gpioInit has been removed from the app_init function anyways.
void gpioInit()
{

  // Student Edit:

  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

  GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
  GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);



} // gpioInit()


void gpioLed0SetOn()
{
  GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
  GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
  GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
  GPIO_PinOutClear(LED1_port,LED1_pin);
}

void gpioToggleLED0()
{
  static  bool on=false;
  if (on == false) {
    on = true;
    GPIO_PinOutSet(LED0_port,LED0_pin);
  } else {
    on = false;
    GPIO_PinOutClear(LED0_port,LED0_pin);
  }
} // gpioToggleLED0(0


//void gpioSensorEnSetOn()
//{
//  GPIO_PinModeSet(SENSOR_PORT, SENSOR_ENABLE, gpioModePushPull, on_off);
//}

//Enables/Disables I2C sensor
void gpio_I2C(int on_off){
  GPIO_PinModeSet(SENSOR_PORT, SENSOR_ENABLE, gpioModePushPull, on_off);

}

//toggle the EXTCOMIN pin for operation of LCD
void gpioSetDisplayExtcomin(bool value)
{
  GPIO_PinModeSet(DISP_PORT, DISP_EXTCOMIN, gpioModePushPull, value);
}





