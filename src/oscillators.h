/*
 * oscillators.h
 * This file contains function prototype for oscillator initialization.Check .c file for function definition.
 *  Created on: Sep 9, 2021
 *      Author: mich1576
 */

#ifndef SRC_OSCILLATORS_H_
#define SRC_OSCILLATORS_H_

#include "em_cmu.h"
#include "app.h"
#define PRESCALAR_VALUE_LFXO    2                      //Prescalar value for LFXO
#define PRESCALAR_VALUE_ULFRCO  1                      //Prescalar value for ULFRCO

#define FREQUENCY_LFXO          32768                  //Frequency of LFXO clock
#define FREQUENCY_ULFRCO        1000                   //Frequency of ULFRCO clock

#define US_EACH_TICK_LFXO           1000000/(FREQUENCY_LFXO/PRESCALAR_VALUE_LFXO)
#define US_EACH_TICK_ULFRCO         1000000/(FREQUENCY_ULFRCO/PRESCALAR_VALUE_ULFRCO)

#define ACTUAL_CLK_FREQ_LFXO        (FREQUENCY_LFXO/PRESCALAR_VALUE_LFXO)         //clock frequency after pre-scaling for LFXO
#define ACTUAL_CLK_FREQ_ULFRCO      (FREQUENCY_ULFRCO/PRESCALAR_VALUE_ULFRCO)     //clock frequency after pre-scaling for ULFRCO

#define VALUE_TO_LOAD_LFXO_COMP0    ((ACTUAL_CLK_FREQ_LFXO *LETIMER_PERIOD_MS)/1000)  //value to load in comp0 when LFXO is used
//#define VALUE_TO_LOAD_LFXO_COMP1    (ACTUAL_CLK_FREQ_LFXO *(LETIMER_PERIOD_MS-LETIMER_ON_TIME_MS))/1000 //value to load in comp1 when LFXO is used

#define VALUE_TO_LOAD_ULFRCO_COMP0  ((ACTUAL_CLK_FREQ_ULFRCO *LETIMER_PERIOD_MS)/1000)       //value to load in comp0 when ULFRCO is used
//#define VALUE_TO_LOAD_ULFRCO_COMP1  (ACTUAL_CLK_FREQ_ULFRCO *(LETIMER_PERIOD_MS-LETIMER_ON_TIME_MS))/1000 //value to load in comp1 when ULFRCO is used

#define MILLIECONDS_PER_TICK_LFXO   (((VALUE_TO_LOAD_LFXO_COMP0 - LETIMER_CounterGet(LETIMER0))* ACTUAL_CLK_FREQ_LFXO)/1000)            //calculates milliseconds per tick for LFXO clock
#define MILLIECONDS_PER_TICK_ULFRCO (((VALUE_TO_LOAD_ULFRCO_COMP0 - LETIMER_CounterGet(LETIMER0))*ACTUAL_CLK_FREQ_ULFRCO)/1000)         //calculates milliseconds per tick for ULFRCO clock



//function prototypes
/* function       : init_oscillators
 * params         : void
 * brief          : initializes the clock base on the energy mode setting and initializes the clock for use of LETIMER0
 * return type    : void
 */
void init_oscillators();

#endif /* SRC_OSCILLATORS_H_ */
