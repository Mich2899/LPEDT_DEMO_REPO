/*
 * scheduler.h
 * Contains function prototypes for scheduling temperature measurement event. Check .c file for function definition.
 *  Created on: Sep 15, 2021
 *      Author: mich1576
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdint.h>
#include "em_core.h"
#include "i2c.h"

//enum to determine which state/event to be in
typedef enum {
evtNoEvent = 0,
evtUF = 1,
evtCOMP1 = 2,
evtI2C = 4,
}event_t;

typedef enum{
Idle = 0,
Warmup = 1,
write = 2,
read = 3,
timerwait = 4,
}state_t;

//function prototypes
/*  function          : scheduler_evtUF
 *  params            : none
 *  brief             : set the myevents global variable to underflow event
 *                      Makes sure that the variable is assigned value in critical section
 *                      Based on the previous state of the state machine, which would be idle
 *
 *  return-type       : void
 * */
void scheduler_evtUF ();

/*  function          : scheduler_evtCOMP1
 *  params            : none
 *  brief             : set the myevents global variable to COMP1 event
 *                      Makes sure that the variable is assigned value in critical section
 *  return-type       : void
 * */
void scheduler_evtCOMP1 ();

/*  function          : scheduler_evtI2C
 *  params            : none
 *  brief             : set the myevents global variable to I2C event
 *                      Makes sure that the variable is assigned value in critical section
 *  return-type       : void
 * */
void scheduler_evtI2C ();

/*  function          : getNextEvent
 *  params            : none
 *  brief             : save the current event in a variable return that value and
 *                      set the myEvents variable to noEvent
 *  return-type       : uint32_t
 * */
uint32_t getNextEvent();

/*  function          : temperature_state_machine
 *  params            : uint32_t event
 *  brief             : Takes event as input and based on the previous state updates the next state
 *  return-type       : void
 * */
void temperature_state_machine (uint32_t event);

#endif /* SRC_SCHEDULER_H_ */
