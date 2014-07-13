/*
 * uiBase.h
 *
 *  Created on: Jul 13, 2014
 *      Author: dmolnar
 *
 *      Helper routines for the 3x7 segment diplay block
 */

#ifndef UIBASE_H_
#define UIBASE_H_

#include "Base.h"

// define the number of input lines
#define NR_OF_INPUTS 3

// define input line pins
#define PLUS_INPUT_PIN P1_3
#define MINUS_INPUT_PIN P1_4
#define SET_INPUT_PIN P1_5
#define MASK_START_VALUE 0x08

// define input events
#define NO_INPUT_EVENT 0
#define PLUS_INPUT_PRESSED 1
#define PLUS_INPUT_RELEASED 2
#define MINUS_INPUT_PRESSED 3
#define MINUS_INPUT_RELEASED 4
#define SET_INPUT_PRESSED 5
#define SET_INPUT_RELEASED 6

// UI initializer
void init_ui(void);

// The UI event generator function
unsigned char get_ui_input_event(void);


#endif /* UIBASE_H_ */
