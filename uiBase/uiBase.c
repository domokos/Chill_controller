/*
 * uiBase.c
 *
 *  Created on: Jul 13, 2014
 *      Author: dmolnar
 *
 *      Helper routines for the 3x7 segment diplay block
 */

#include "uiBase.h"

static unsigned char input_line_event_buffer[NR_OF_INPUTS];

void
init_ui(void)
{
  unsigned char i;

  for(i=0;i<NR_OF_INPUTS;i++)
    input_line_event_buffer[i] = 0;
}

unsigned char
get_ui_input_event(void)
{
  unsigned char mask, i;

  mask = MASK_START_VALUE;

  for(i=0;i<NR_OF_INPUTS;i++)
    {
      if(P1 & mask){
          if(input_line_event_buffer[i]<255)
            input_line_event_buffer[i]++;
      }else{
          if(input_line_event_buffer[i]>0)
            input_line_event_buffer[i]--;
      }
      mask = mask << 1;
    }


  return NO_INPUT_EVENT;
}
