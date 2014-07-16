/*
 * uiBase.c
 *
 *  Created on: Jul 13, 2014
 *      Author: dmolnar
 *
 *      Helper routines for the 3x7 segment diplay block
 */

#include "uiBase.h"

// The state of input lines
static struct input_events input_line_event_buffer[NR_OF_INPUTS];

// The content of the display buffer
static unsigned char segment_buffer[NR_OF_DIGITS];

// The index to be used in the actual display cycle
static unsigned char display_index;

// Blink flags
static bool blink, is_blinking, display_off;

// 7-segment display encoder lookup table
__code const static unsigned char digit_encoder[] =
    {
        0xfc,0x60,0xda,0xf2,0x66,0xb6,0xbe,0xe0,0xfe,0xf6
    };

/*
 * INPUT handlers
 */

void
init_ui(void)
{
  unsigned char input_mask, i;
  input_mask = MASK_START_VALUE;

  for(i=0;i<NR_OF_INPUTS;i++)
    {
    // Clear input event buffer
    input_line_event_buffer[i].counter = 0;
    input_line_event_buffer[i].is_handled = FALSE;

    // Set Inputs high so key press events can be read
    P1 |= input_mask;
    input_mask = input_mask << 1;
    }
  reset_segment_output();
}

unsigned char
do_ui(void)
{
  unsigned char input_mask, i, return_event;

  return_event = NO_INPUT_EVENT;
  input_mask = MASK_START_VALUE;

  // Loop through all inputs
  for(i=0;i<NR_OF_INPUTS;i++)
    {
      if(!(P1 & input_mask)){
          if(input_line_event_buffer[i].counter < INPUT_EVENT_COUNT_THRESHOLD && !input_line_event_buffer[i].is_handled)
            {
            input_line_event_buffer[i].counter++;
            if(input_line_event_buffer[i].counter == INPUT_EVENT_COUNT_THRESHOLD)
              {
                // If the threshold is reached then return the press event of the corresponding input and set the handled flag to true
                input_line_event_buffer[i].is_handled = TRUE;
                return_event = NO_INPUT_EVENT + 1 + i;
              }
            }
      }else{
          if(input_line_event_buffer[i].counter > 0)
            {
              input_line_event_buffer[i].counter--;
              // If enough time has elapsed since the release of the input then clear the handled flag
              if (input_line_event_buffer[i].counter == 0)
                {
                  input_line_event_buffer[i].is_handled = FALSE;
                }
            }
      }
      input_mask = input_mask << 1;
    }

  display_output();
  return return_event;
}


/*
 * DISPLAY handlers
 */

// Reset the display
static void
reset_segment_output(void)
{
  NOE_PIN = 1;
  DS_PIN = 0;
  SHCP_PIN = 0;
  NMR_PIN = 0;

  STCP_PIN = 0;
  STCP_PIN = 1;
  STCP_PIN = 0;

  NMR_PIN = 1;
  NOE_PIN = 0;

  display_index = 0;
  DIGIT_1 = 0;
  DIGIT_2 = 0;
  DIGIT_3 = 0;
  blink = FALSE;
  is_blinking = FALSE;
  display_off = FALSE;
}

// Set the output to the values in the buffer position marked by index
static void
write_segment_output(unsigned char index)
{
  unsigned char mask;

  // Reset the shift registers
  NMR_PIN = 0;
  NMR_PIN = 1;

  mask = 0x80;

  // Put all bits to the serial output
  while (mask)
    {
      DS_PIN = (segment_buffer[index] & mask) > 0;
      SHCP_PIN = 1;
      SHCP_PIN = 0;

      mask >>= 1;
    }

  STCP_PIN = 1;
  STCP_PIN = 0;
}

static void
display_output(void)
{
  if(display_off)
    {
      // Turn off all digits
      DIGIT_1 = 0;
      DIGIT_2 = 0;
      DIGIT_3 = 0;

    }else{
     // Perform normal operation

      // Extinguish the currently displayed digit
      // Write the actual display buffer to the output
      // Display the actual digit and
      // move the index to the next digit for the next cycle
      switch(display_index)
      {
      case 0:
        DIGIT_3 = 0;
        write_segment_output(display_index);
        DIGIT_1 = 1;
        display_index = 1;
        break;
      case 1:
        DIGIT_1 = 0;
        write_segment_output(display_index);
        DIGIT_2 = 1;
        display_index = 2;
        break;
      case 2:
        DIGIT_2 = 0;
        write_segment_output(display_index);
        DIGIT_3 = 1;
        display_index = 0;
        break;
      }
    }

  // Take care of blinking
  if(blink)
    {
      if(is_blinking)
        {
          // Blinking active - check blink timer timeout and toggle display off/reset timer if needed
          if (timeout_occured(BLINK_TIMER, TIMER_MS, BLINK_PERIOD_MS))
            {
              display_off = !display_off;
              reset_timeout(BLINK_TIMER, TIMER_MS);
            }
        }else{
           // Blinking requested but not active - start blinking and reset timer
            display_off = TRUE;
            is_blinking = TRUE;
            reset_timeout(BLINK_TIMER, TIMER_MS);
        }
    } else {
        if(is_blinking)
          {
          // Blinking active - stop blinking
            display_off = FALSE;
            is_blinking = FALSE;
          }
    }
}

// Sets the temperature value to be displayed into the display buffer. Takes 100 times the value to be displayed
void
set_display_temp(signed int value)
{
  unsigned int uns_value;

  if(value < -100)
    {
        // display " L0"
        segment_buffer[0] = CHAR_SPACE;
        segment_buffer[1] = CHAR_L;
        segment_buffer[2] = digit_encoder[0];
    }else if(value < 0) {
        // display "-x.x"
        uns_value = value * -1;
        segment_buffer[0] = CHAR_MINUS;
        segment_buffer[1] = digit_encoder[(unsigned char) (uns_value / 10)];
        segment_buffer[1] |= DOT_MASK;
        segment_buffer[2] = digit_encoder[(unsigned char) (uns_value % 10)];
    }else if(value < 100) {
        // display x.x
        uns_value = value;
        segment_buffer[0] = CHAR_SPACE;
        segment_buffer[1] = digit_encoder[(unsigned char) (uns_value / 10)];
        segment_buffer[1] |= DOT_MASK;
        segment_buffer[2] = digit_encoder[(unsigned char) (uns_value % 10)];
    }else if(value < 1000) {
        // display xx.x
        uns_value = value;
        segment_buffer[0] = digit_encoder[(unsigned char) (uns_value / 100)];
        uns_value = uns_value - (uns_value / 100)*100;
        segment_buffer[1] = digit_encoder[(unsigned char) (uns_value / 10)];
        segment_buffer[1] |= DOT_MASK;
        segment_buffer[2] = digit_encoder[(unsigned char) (uns_value % 10)];
    }else{
        // dsiplay HI
        segment_buffer[0] = CHAR_SPACE;
        segment_buffer[1] = CHAR_H;
        segment_buffer[2] = digit_encoder[1];
    }
}

void
set_blink(bool blink_request)
{
  blink = blink_request;
}
