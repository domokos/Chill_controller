/*
 * Chiller controller.c
 *
 *  Created on: Jul 27, 2014
 *      Author: dmolnar
 */

#include "Controller.h"

/*
 * Onewire specific declarations and defines
 */
// Map sensors to onewire buses Sensor1 is on P1_0, Sensor2 is on P1_1
__code const unsigned char sensor_pinmask_map[2] =
  { 0x40, // P1_6
    0x80 }; // P1_7

bool conv_complete, bus0_conv_initiated, bus1_conv_initiated, reevaluate_chill_logic;

// Buffer to store Temperatures and teir index
int temperatures_buffer[NR_OF_TEMP_SENSORS][FILTER_BUFFER_LENGTH];
unsigned char temp_buffer_index[NR_OF_TEMP_SENSORS];

// The target temperature of the controller
int target_temp;
unsigned int icing_condition_counter;
chiller_state_type chiller_state;

// Variable to store the state of the UI
ui_state_type ui_state;

// Sensors are read in a circular manner. On e cycle completes in time equal to the conversion
// time. This variable holds the id of the sensor to be addressed next during the cycle.
unsigned char bus_to_address;

/*
 * PWM specific variables
 */

// Variables holding PWM times and state
unsigned int pwm_on_time, pwm_off_time, pwm_wait_time;
pwm_states pwm_state;

// Variables holding PWM modification related flags
bool pwm_active, inactive_pwm_pin_value;

/*
 * Functions of the device
 *
 * ONEWIRE devices specific section
 */

bool
set_temp_resolution(sensor_type sensor_id, unsigned char resolution)
{
  unsigned char pinmask = sensor_pinmask_map[sensor_id];

  if (onewire_reset(pinmask))
    {
      onewire_write_byte(CMD_SKIP_ROM, pinmask);
      onewire_write_byte(CMD_WRITE_SCRATCHPAD, pinmask);
      onewire_write_byte(0, pinmask);
      onewire_write_byte(0, pinmask);
      onewire_write_byte(resolution, pinmask);
      return TRUE;
    }
  else
    {
      return FALSE;
    }
}

int
convert_to_decimal_temp(int measured_bits)
{
  int integer_part;
  int fractional_part;

  integer_part = measured_bits >> 4;

  integer_part = integer_part * 10;

  if (measured_bits >= 0)
    fractional_part = measured_bits & 0x000f;
  else
    fractional_part = (measured_bits & 0x000f) | 0xfff0;

  fractional_part = (fractional_part * 100) / 16;

  if (measured_bits >= 0)
    {
      if (fractional_part % 10 < 5)
        fractional_part = fractional_part / 10;
      else
        fractional_part = (fractional_part / 10) + 1;
    }else{
      if (fractional_part % 10 < -5)
        fractional_part = (fractional_part / 10) - 1;
      else
        fractional_part = (fractional_part / 10);
    }

  return integer_part + fractional_part;
}

// Read a DS18B20 sensor
void
read_DS18B20(sensor_type sensor_id)
{
  unsigned char i, pinmask = sensor_pinmask_map[sensor_id];
  bool measurement_succesful;

  // Reset and read the temperature
  if (onewire_reset(pinmask))
    {
      onewire_write_byte(CMD_SKIP_ROM, pinmask);

      onewire_write_byte(CMD_READ_SCRATCHPAD, pinmask);

      for (i = 0; i < 9; i++)
        ow_buf[i] = onewire_read_byte(pinmask);

      measurement_succesful = (ow_buf[8] == calculate_onewire_crc(ow_buf, 8)) && (ow_buf[7] == 0x10);

      // Increment measurement index if the buffer should be filled and fill the buffer with the result of the current measurement
      if (timeout_occured(TEMP_MEASUREMENT_TIMER, TIMER_SEC, TEMP_MEASUREMENT_PERIOD_SEC))
        {
        if(measurement_succesful)
          {
            reset_timeout(TEMP_MEASUREMENT_TIMER, TIMER_SEC);

            if (temp_buffer_index[sensor_id] < FILTER_BUFFER_LENGTH-1)
              temp_buffer_index[sensor_id]++;
            else
              temp_buffer_index[sensor_id] = 0;

            temperatures_buffer[sensor_id][temp_buffer_index[sensor_id]] = convert_to_decimal_temp(ow_buf[0] | (ow_buf[1] << 8));
            // Set flag for reevaluating the chilling logic
            reevaluate_chill_logic = TRUE;
          }
        }
    }
}

// Return if conversion command is sent succesfully
// It takes a reference to a specific device but issues convert on the entire bus
bool
issue_convert_on_bus(sensor_type sensor_id)
{
  unsigned char pinmask = sensor_pinmask_map[sensor_id];

  if (onewire_reset(pinmask))
    {
      onewire_write_byte(CMD_SKIP_ROM, pinmask);
      onewire_write_byte(CMD_CONVERT_T, pinmask);
      return TRUE;
    }
  return FALSE;
}

// Keep conversions going on for each sensor on each onewire bus
void
operate_onewire_temp_measurement(void)
{
  if (conv_complete)
    {
      switch (bus_to_address)
        {
      case 0:
        // Only read if conversoin was succesfully initiated
        if (bus0_conv_initiated)
          {
            read_DS18B20(RADIATOR_SENSOR);
          }
        bus0_conv_initiated = issue_convert_on_bus(0);
        bus_to_address = 1;
        break;

      case 1:
        // Only read if conversoin was succesfully initiated
        if (bus1_conv_initiated)
          {
            read_DS18B20(ROOM_SENSOR);
          }
        bus1_conv_initiated = issue_convert_on_bus(1);
        bus_to_address = 0;
        break;
        }

      // Reset the conversion timer and set the complete flag so we
      // can wait for conversion time expiry on the next bus
      reset_timeout(TEMP_CONV_TIMER, TIMER_MS);
      conv_complete = FALSE;
    }
  else
    {
      conv_complete = timeout_occured(TEMP_CONV_TIMER, TIMER_MS,
          DS18x20_CONV_TIME / NR_OF_OW_BUSES);
    }
}


// Calculate the filtered average temperature
//  m_k = m_{k-1} + (x_k - m_{k-1}) / k
int get_filtered_mean_temp(sensor_type sensor_id)
{
  unsigned char i;
  int average;

  average = (temperatures_buffer[sensor_id][0] + temperatures_buffer[sensor_id][1]) / 2;

  for(i = 2; i < FILTER_BUFFER_LENGTH; i++)
    average = average + (temperatures_buffer[sensor_id][i] - average) / (i+1);

  return average;
}

// Must be called periodically to take care of PWM outputs
void
operate_PWM(void)
{
  if (timeout_occured(PWM_TIMER, TIMER_SEC, pwm_wait_time))
    {
      if(!pwm_active)
        {
          PWM_PIN = inactive_pwm_pin_value;
          pwm_state = PWM_OFF;
          return;
        }
      if (pwm_state == PWM_OFF)
        {
          pwm_wait_time = pwm_on_time;
          PWM_PIN = PWM_OUTPUT_ON;
          pwm_state = PWM_ON;
        } else {  // If pwm_state == PWM_ON
          pwm_wait_time = pwm_off_time;
          PWM_PIN = PWM_OUTPUT_OFF;
          pwm_state = PWM_OFF;
        }
      reset_timeout(PWM_TIMER, TIMER_SEC);
    }
}

void
calculate_PWM_times(int actual_temp)
{
  char required_cooling_power;
  signed int error;

  error = actual_temp - target_temp;

  if (error > 2)
    required_cooling_power = 100;
  else if (error > -3)
    required_cooling_power = error*TEMP_COEFF_A + TEMP_COEFF_B;
  else { required_cooling_power = 0;}

  if (required_cooling_power == 100)
    {
      pwm_on_time = 0;
      pwm_off_time = 0;
      pwm_active = FALSE;
      inactive_pwm_pin_value = PWM_OUTPUT_ON;
    }
  else if (required_cooling_power > 0)
    {
      pwm_on_time =  required_cooling_power * POWER_COEFF_A + POWER_COEFF_B;
      pwm_off_time = 1800 - pwm_on_time;
      pwm_active = TRUE;
    }
  else {
      pwm_on_time = 0;
      pwm_off_time = 0;
      pwm_active = FALSE;
      inactive_pwm_pin_value = PWM_OUTPUT_OFF;
  }
}

void
operate_chilling_logic(void)
{

  int radiator_temp, room_temp;

  if(reevaluate_chill_logic)
    {
      // reset need for evaluation
      reevaluate_chill_logic = FALSE;

      radiator_temp = get_filtered_mean_temp(RADIATOR_SENSOR);
      room_temp = get_filtered_mean_temp(ROOM_SENSOR);

      if (radiator_temp < 0 && icing_condition_counter < 0xffff)
          icing_condition_counter++;
        else if (icing_condition_counter > 0)
          icing_condition_counter--;

      switch (chiller_state)
      {
      case COOLING:
        if (icing_condition_counter > ICING_CONDITION_THRESHOLD)
          {
            chiller_state = DEICING;
            // Turn of cooling
            pwm_on_time = 0;
            pwm_off_time = 0;
            pwm_active = FALSE;
            inactive_pwm_pin_value = PWM_OUTPUT_OFF;
            reset_timeout(DEICING_TIMER, TIMER_SEC);
            break;
          }
        calculate_PWM_times(room_temp);
        break;

      case DEICING:
        if(timeout_occured(PWM_TIMER, TIMER_SEC, DEICING_TIME_SEC))
            {
            // Exiting DEICING state
            chiller_state = COOLING;
            icing_condition_counter = 0;
            calculate_PWM_times(room_temp);
            }
        break;
      }
    }
}

void handle_ui(void)
{
  unsigned char input_event;
  input_event = do_ui();

  switch (input_event)
  {
  case NO_INPUT_EVENT:
    if (ui_state == SETTING_TARGET_TEMP && timeout_occured( UI_STATE_TIMER, TIMER_MS, UI_STATE_RESET_TIME_MS))
      ui_state = ACTUAL_TEMP_DISPLAY;
    break;

  case PLUS_INPUT_PRESSED:
    if (ui_state == ACTUAL_TEMP_DISPLAY)
      ui_state = SETTING_TARGET_TEMP;

      // ui_state == SETTING_TARGET_TEMP
      else if (target_temp < MAX_TARGET_TEMP)
          target_temp++;

    reset_timeout( UI_STATE_TIMER , TIMER_MS);
    break;

  case MINUS_INPUT_PRESSED:
    if (ui_state == ACTUAL_TEMP_DISPLAY)
        ui_state = SETTING_TARGET_TEMP;

      // ui_state == SETTING_TARGET_TEMP
      else if (target_temp > MIN_TARGET_TEMP)
         target_temp--;

    reset_timeout( UI_STATE_TIMER , TIMER_MS);
    break;
  }

  if (ui_state == ACTUAL_TEMP_DISPLAY)
    {
      set_display_temp(get_filtered_mean_temp(ROOM_SENSOR));
      set_display_blink(FALSE);

    // ui_state == SETTING_TARGET_TEMP
    } else {
       set_display_temp(target_temp);
       set_display_blink(TRUE);
    }
}

void
init_device(void)
{
  unsigned char i,j;

  i = NR_OF_TEMP_SENSORS;
  while (i--)
    {
    j = FILTER_BUFFER_LENGTH;
    while (j--)
      temperatures_buffer[i][j] = -199;
    }
  temp_buffer_index[RADIATOR_SENSOR] = 0;
  temp_buffer_index[ROOM_SENSOR] = 0;

  reset_timeout(TEMP_MEASUREMENT_TIMER, TIMER_SEC);

  // Set initial resolutions to 12 bit
  set_temp_resolution(0, TEMP_RESOLUTION_12BIT);
  set_temp_resolution(1, TEMP_RESOLUTION_12BIT);

  // We need to start a new conversion so it is complete on init
  conv_complete = TRUE;

  bus0_conv_initiated = bus1_conv_initiated = FALSE;
  bus_to_address = 0;

  // Reset conversion timers and distribute conversion across the 2 sensors
  reset_timeout(TEMP_CONV_TIMER, TIMER_MS);

  // Reset PWM
  pwm_on_time = 0;
  pwm_off_time = 1;
  pwm_state = PWM_OFF;
  pwm_active = FALSE;
  inactive_pwm_pin_value = PWM_OUTPUT_OFF;

  //Set the initial target temp

  target_temp = INITIAL_TARGET_TEMP;

  init_ui();
  set_display_temp(get_filtered_mean_temp(ROOM_SENSOR));

  // Init chill logic
  reevaluate_chill_logic = FALSE;
  icing_condition_counter = 0;
  chiller_state = COOLING;

  // Reset UI state
  ui_state = ACTUAL_TEMP_DISPLAY;
}

void
main(void)
{
// Enable interrupts and initialize timer
  EA = 1;
  init_timer();
// Initialize the device
  init_device();

// Start the main execution loop
  while (TRUE)
    {
      // Operate main device functions and sets reevaluate_chill_logic flag if change is detected
      operate_onewire_temp_measurement();
      handle_ui();

      operate_chilling_logic();

      operate_PWM();

    }
}
