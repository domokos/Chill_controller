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

/*
 * INPUT related definitions
 */

// define the number of input lines
#define NR_OF_INPUTS 3

// Event buffer STRUCT
struct input_events
{
  unsigned char   is_handled;
  unsigned char   counter;
};

// define input line pins
#define PLUS_INPUT_PIN P1_3
#define MINUS_INPUT_PIN P1_4
#define SET_INPUT_PIN P1_5
#define MASK_START_VALUE 0x08

// define input events
#define NO_INPUT_EVENT 0
#define PLUS_INPUT_PRESSED 1
#define MINUS_INPUT_PRESSED 2
#define SET_INPUT_PRESSED 3

// input event counter threshold
#define INPUT_EVENT_COUNT_THRESHOLD 20

/*
 * DISPLAY related definitions
 */

/*
 * segment buffer specific declarations and defines.
 * For segment buffer 74HC595, 8-bit parallell or serial out shift registers are used.
 */

#define DS_PIN P3_0
#define SHCP_PIN P3_1
#define NMR_PIN P3_2
#define STCP_PIN P3_3
#define NOE_PIN P3_4

// digit encoder - encodes digits to segment bits for the buffer
extern const static unsigned char digit_encoder[];
extern static unsigned char segment_buffer[];
extern static unsigned char display_index;
extern static bool blink, is_blinking, display_off;

// Digit specific definitions
#define NR_OF_DIGITS 3
#define BLINK_PERIOD_MS 600

// Digit selector pins
#define DIGIT_1 P3_5
#define DIGIT_2 P3_7
#define DIGIT_3 P1_6

// additional segment display values
#define CHAR_L 0x1c
#define CHAR_H 0x6e
#define CHAR_MINUS 0x02
#define CHAR_SPACE 0x00
#define DOT_MASK 0x01

// UI initializer
void init_ui(void);

// The UI event generator and ui operator function
unsigned char do_ui(void);
// Sets the temperature value to be displayed into the display buffer. Takes 100 times the value to be displayed
void set_display_temp(signed int value);
void set_blink(bool blink_request);

// The output display handler
static void display_output(void);

// Handle segments
static void reset_segment_output(void);
static void write_segment_output(unsigned char index);



#endif /* UIBASE_H_ */
