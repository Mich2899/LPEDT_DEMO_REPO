/*
 * irq.c
 * This file contains function prototype defining the functionality in interrupt handler
 *  Created on: Sep 9, 2021
 *      Author: mich1576
 */

/*********************************************************INCLUDES****************************************************************/
#include "irq.h"

// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "log.h"


uint32_t rollover_count=0, milliseconds=0;
I2C_TransferReturn_TypeDef transferStatus;

/****************************************************FUNCTION DEFINITION*********************************************************/
//Check for any interrupts, turn the LED on and off accordingly and reset the interrupt flags.
void LETIMER0_IRQHandler (void) {

    uint32_t flags;
        // determine source of IRQ
        flags = LETIMER_IntGetEnabled(LETIMER0);
        // clear source of IRQ set in step 3
        LETIMER_IntClear(LETIMER0, flags);

        if(flags & LETIMER_IF_UF){
            //call scheduler to read temperature data from SI7021
            scheduler_evtUF ();
            rollover_count++;
        }

        if(flags & LETIMER_IF_COMP1){
            //call scheduler to read temperature data from SI7021
            scheduler_evtCOMP1 ();
        }

} // LETIMER0_IRQHandler()

void I2C0_IRQHandler(void) {

        // This shepherds the IC2 transfer along,
        // it’s a state machine! see em_i2c.c
        // It accesses global variables :
        // transferSequence
        // cmd_data
        // read_data
        transferStatus = I2C_Transfer(I2C0);
        if (transferStatus == i2cTransferDone) {
            scheduler_evtI2C ();
        }
        if(transferStatus < 0) {
            LOG_ERROR("%d", transferStatus);
        }
} // I2C0_IRQHandler()

uint32_t letimerMilliseconds(){
#if ((LOWEST_ENERGY_MODE == 0) || (LOWEST_ENERGY_MODE == 1) || (LOWEST_ENERGY_MODE == 2))
    milliseconds = rollover_count*LETIMER_PERIOD_MS + MILLIECONDS_PER_TICK_LFXO;
#elif (LOWEST_ENERGY_MODE == 3)
    milliseconds = rollover_count*LETIMER_PERIOD_MS + MILLIECONDS_PER_TICK_ULFRCO;
#endif

    return milliseconds;
}
