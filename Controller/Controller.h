/*
 * Controller.h
 *
 *  Created on: Jul 23, 2014
 *      Author: dmolnar
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_


#include "Base.h"
#include "Onewire.h"
#include "uiBase.h"

#define NR_OF_TEMP_SENSORS 2
#define NR_OF_OW_BUSES 2
#define PWM_PIN P1_2
#define FILTER_BUFFER_LENGTH 5
#define TEMP_MEASUREMENT_PERIOD_SEC 60

// Define the indexes of each sensor in the temperatures_buffer array
typedef enum {
  RADIATOR_SENSOR=0,
  ROOM_SENSOR=1 } sensor_type;

// Set initial target temperature to 5.0 C
#define INITIAL_TARGET_TEMP 50

#define MIN_TARGET_TEMP 5 // 0.5 C
#define MAX_TARGET_TEMP 150 // 15.0 C

// PWM state type
typedef enum {PWM_OFF, PWM_ON} pwm_states;

// Define the states of the user interface
typedef enum {
  ACTUAL_TEMP_DISPLAY,
  SETTING_TARGET_TEMP
} ui_state_type;

// define the UI timer to return to basic state
#define UI_STATE_RESET_TIME_MS 5000

#endif /* CONTROLLER_H_ */
