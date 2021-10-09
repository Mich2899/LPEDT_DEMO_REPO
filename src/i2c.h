/*
 * i2c.h
 * This file contains function prototypes used for i2c operations.Check .c file for function definition.
 *  Created on: Sep 15, 2021
 *      Author: mich1576
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

//includes
#include "sl_i2cspm.h"
#include "timers.h"

//Defining the device address and the port for enabling the sensor
#define SI7021_DEVICE_ADDR 0x40

//Defining the port used for I2C0 and the pins used for SCL and SDA
#define I2C_port            gpioPortC
#define SCL_pin             10
#define SDA_pin             11
#define SCL_PORT_LOCATION   14
#define SDA_PORT_LOCATION   16

//function prototypes

/* function     : I2C_init
 * params       : none
 * brief        : Initializes the I2CSPM_Init_TypeDef structure
 * return_type  : void
 * */
void I2C_init();

/* function     : I2C_write
 * params       : none
 * brief        : Used to perform I2C write.
 *                Sends start bit, slave address, send write command
 *                wait for acknowledgment, send measure command,
 *                wait for acknowledgment
 * return_type  : void
 * */
void I2C_write();

/* function     : I2C_read
 * params       : none
 * brief        : Used to perform I2C read.
 *                Sends repeated start bit, slave address, send read command
 *                read MS and LS byte
 * return_type  : void
 * */
void I2C_read();

/* function     : warmup()
 * params       : none
 * brief        : Enable the sensor
 * return_type  : void
 * */
void warmup();

/* function     : turnoff
 * params       : none
 * brief        : Disable the sensor
 * return_type  : void
 * */
void turnoff();

/* function     : store
 * params       : none
 * brief        : Convert and log the temperature
 * return_type  : float
 * */
float store();

#endif /* SRC_I2C_H_ */
