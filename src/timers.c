/*
 * timers.c
 * This file contains function prototype for timer initialization and interrupt settings.
 *  Created on: Sep 9, 2021
 *      Author: mich1576
 */

/*********************************************************INCLUDES****************************************************************/
#include "timers.h"

/*******************************************************GLOBAL VARIABLES*********************************************************/
//Logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

/****************************************************FUNCTION DEFINITION**********************************************************/
//initialize LETIMER0, set the compare register values based on the energy mode settings and set the correct settings for interrupt
void initLETIMER0 (void){
  LETIMER_Init_TypeDef timer_instance=
      {
          .enable         = true,                                               //Don't start counting when initialization completes

          .debugRun       = true,                                               //Counter will not keep running during debug halt

          .comp0Top       = true,                                               //Load COMP0 register into CNT when counter underflows

          .bufTop         = false,                                              //Don't load COMP1 into COMP0 when REP0 reaches 0

          .out0Pol        = 0,                                                  //Idle value for output 0
          .out1Pol        = 0,                                                  //Idle value for output 1
          .ufoa0          = letimerUFOANone,                                    //No output action
          .ufoa1          = letimerUFOANone,                                    //No output action
          .repMode        = letimerRepeatFree,                                  //Repeat until stopped

          .topValue   = 0,                                                      //Use default top Value
        };

  LETIMER_Init(LETIMER0, &timer_instance);

#if ((LOWEST_ENERGY_MODE == 0) || (LOWEST_ENERGY_MODE == 1) || (LOWEST_ENERGY_MODE == 2))
    LETIMER_CompareSet(LETIMER0, 0, VALUE_TO_LOAD_LFXO_COMP0);
    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);                                  //clear the underflow flag
    LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);                                //enable underflow in interruot enable register

#elif (LOWEST_ENERGY_MODE == 3)
    LETIMER_CompareSet(LETIMER0, 0, VALUE_TO_LOAD_ULFRCO_COMP0);
  LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);                                    //clear the underflow flag
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);                                  //enable underflow in interruot enable register

#endif

}

//Takes input in microseconds and provides required amount of delay
void timerWaitUs_polled(uint32_t us_wait){

  uint32_t difference, current, store;

#if ((LOWEST_ENERGY_MODE == 0) || (LOWEST_ENERGY_MODE == 1) || (LOWEST_ENERGY_MODE == 2))
    uint32_t us_each_tick   = US_EACH_TICK_LFXO ;
    uint32_t wait_for_ticks = (us_wait/us_each_tick);

    store = VALUE_TO_LOAD_LFXO_COMP0;
    if(wait_for_ticks> VALUE_TO_LOAD_LFXO_COMP0){
        LOG_ERROR("Invalid wait input\n\r");
    }
#elif (LOWEST_ENERGY_MODE == 3)

    uint32_t us_each_tick   = US_EACH_TICK_ULFRCO ;                             //Using the ULFRCO and tick for that
    uint32_t wait_for_ticks = (us_wait/us_each_tick);                           //calculate the number of ticks required for the same

    store = VALUE_TO_LOAD_ULFRCO_COMP0;
    if(wait_for_ticks> VALUE_TO_LOAD_ULFRCO_COMP0){                             //if the value for ticks exceeds the range provide error message
        LOG_ERROR("Invalid wait input\n\r");
    }
#endif

    else{
            current = LETIMER_CounterGet (LETIMER0);                            //take the current value of the timer
            if( current > wait_for_ticks ){                                     //check if the required number of ticks are greater than current counter value
                difference = current - wait_for_ticks;                          //take the difference between current timer counter value and the required ticks
                while(current>=difference){                                     //poll until required ticks are obtained
                   current = LETIMER_CounterGet (LETIMER0);
                }
            }
            else{

                difference = store - (wait_for_ticks - current);                //count until the counter reaches zero and calculate remaining number of ticks
                 while(current != difference){
                     current = LETIMER_CounterGet (LETIMER0);
                 }
            }
        }
}

//Takes input in microseconds and provides required amount of delay
void timerWaitUs_irq(uint32_t us_wait){

  uint32_t difference, current, store;

#if ((LOWEST_ENERGY_MODE == 0) || (LOWEST_ENERGY_MODE == 1) || (LOWEST_ENERGY_MODE == 2))

    uint32_t us_each_tick   = US_EACH_TICK_LFXO ;
    uint32_t wait_for_ticks = (us_wait/us_each_tick);


    store = VALUE_TO_LOAD_LFXO_COMP0;                                           //store the overflow value
    if(wait_for_ticks> VALUE_TO_LOAD_LFXO_COMP0){                               //if the value for ticks exceeds the range provide error message
        LOG_ERROR("Invalid wait input\n\r");
    }

#elif (LOWEST_ENERGY_MODE == 3)

    uint32_t us_each_tick   = US_EACH_TICK_ULFRCO ;                             //Using the ULFRCO and tick for that
    uint32_t wait_for_ticks = (us_wait/us_each_tick);                           //calculate the number of ticks required for the same

    store = VALUE_TO_LOAD_ULFRCO_COMP0;                                         //store the overflow value
    if(wait_for_ticks> VALUE_TO_LOAD_ULFRCO_COMP0){                             //if the value for ticks exceeds the range provide error message
        LOG_ERROR("Invalid wait input\n\r");
    }
#endif

    else{
            current = LETIMER_CounterGet (LETIMER0);                            //take the current value of the timer
            if( current > wait_for_ticks ){                                     //check if the required number of ticks are greater than current counter value
                difference= current - wait_for_ticks;                           //take the difference between current timer counter value and the required ticks
            }
            else{
                LOG_ERROR("rollover");
                difference = store - (wait_for_ticks - current);                //count until the counter reaches zero and calculate remaining number of ticks
            }
            LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);                      //to avoid any timer related issues caused by BLE external signal
            LETIMER_CompareSet(LETIMER0, 1, difference);                        //Set COMP1 register to calculated value
            LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);                     //Enable COMP1 interrupt
    }
}
