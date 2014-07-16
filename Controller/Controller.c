/*
 * Furnace_temp_device.c
 *
 *  Created on: Oct 22, 2012
 *      Author: dmolnar
 */

#include "Base.h"
#include "Onewire.h"
#include "uiBase.h"

#define NR_OF_TEMP_SENSORS 2
#define NR_OF_OW_BUSES 2
#define PWM_PIN P1_2
#define FILTER_BUFFER_LENGTH 5
#define TEMP_MEASUREMENT_PERIOD_SEC 60

// Define the indexes of each sensor in the temperatures_buffer array
#define RADIATOR_SENSOR 0
#define ROOM_SENSOR 1

/*
 * Onewire specific declarations and defines
 */
// Map sensors to onewire buses Sensor1 is on P1_0, Sensor2 is on P1_1
__code const unsigned char sensor_pinmask_map[2] =
  { 0x01, 0x02 };

bool conv_complete, bus0_conv_initiated, bus1_conv_initiated;

// Buffer to store Temperatures and its index
int temperatures_buffer[NR_OF_TEMP_SENSORS][FILTER_BUFFER_LENGTH];
int prompt_temp[NR_OF_TEMP_SENSORS];
unsigned char temp_buffer_index[NR_OF_TEMP_SENSORS];

// Sensors are read in a circular manner. On e cycle completes in time equal to the conversion
// time. This variable holds the id of the sensor to be addressed next during the cycle.
unsigned char bus_to_address;

/*
 * PWM specific defines and variables
 */
// States and requested states
#define PWM_OFF 0
#define PWM_ON 1

// Variables holding PWM times and state
unsigned char pwm_on_time, pwm_off_time, pwm_wait_time, pwm_state;

// Variables holding PWM modification related flags
bool pwm_active;

/*
 * Functions of the device
 *
 * ONEWIRE devices specific section
 */

bool
set_temp_resolution(unsigned char sensor_id, unsigned char resolution)
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

void
scale_DS18B20_result(unsigned char sensor_id)
{
  temperatures_buffer[sensor_id][temp_buffer_index[sensor_id]] *= 8;
  temperatures_buffer[sensor_id][temp_buffer_index[sensor_id]] &= 0xFFF0;
  temperatures_buffer[sensor_id][temp_buffer_index[sensor_id]] += 12;
  temperatures_buffer[sensor_id][temp_buffer_index[sensor_id]] -= ow_buf[6];
}

// Read a DS18xx sensor
void
read_DS18xxx(unsigned char sensor_id)
{
  unsigned char i, pinmask = sensor_pinmask_map[sensor_id];

  // Reset and read the temperature
  if (onewire_reset(pinmask))
    {
      onewire_write_byte(CMD_SKIP_ROM, pinmask);

      onewire_write_byte(CMD_READ_SCRATCHPAD, pinmask);

      for (i = 0; i < 9; i++)
        ow_buf[i] = onewire_read_byte(pinmask);

      if (ow_buf[8] == calculate_onewire_crc(ow_buf, 8) && ow_buf[7] == 0x10)
        {
          prompt_temp[sensor_id] = ow_buf[0] | (ow_buf[1] << 8);
          // If result needs scaling up then scale it up
//          if (sensor_identification[sensor_id][SCALE_POSITION] == SCALE_TEMP)
//            scale_DS18B20_result(sensor_id);
        }

      // Increment measurement index if the buffer should be filled and fill the buffer with the result of the current measurement
      if (timeout_occured(TEMP_MEASUREMENT_TIMER, TIMER_SEC, TEMP_MEASUREMENT_PERIOD_SEC))
        {
        reset_timeout(TEMP_MEASUREMENT_TIMER, TIMER_SEC);

        if (temp_buffer_index[sensor_id] < FILTER_BUFFER_LENGTH-1)
          temp_buffer_index[sensor_id]++;
        else
          temp_buffer_index[sensor_id] = 0;

        temperatures_buffer[sensor_id][temp_buffer_index[sensor_id]] = prompt_temp[sensor_id];
        }
    }
}

// Return if conversion command is sent succesfully
// It takes a reference to a specific device but issues convert on the entire bus
bool
issue_convert_on_bus(unsigned char sensor_id)
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
        // Evaluate side effect: Only read until read is succesful
        if (bus0_conv_initiated)
          {
            read_DS18xxx(0);
          }
        bus0_conv_initiated = issue_convert_on_bus(0);
        bus_to_address = 1;
        break;

      case 1:
        // Evaluate side effect: Only read until read is succesful
        if (bus1_conv_initiated)
          {
            read_DS18xxx(1);
          }
        bus1_conv_initiated = issue_convert_on_bus(1);
        bus_to_address = 0;
        break;
        }

      // Reset the conversion timer and set the complete flag so we
      // can wait for conversion time expiry on the next bus
      reset_timeout(TIMER_MS, TEMP_CONV_TIMER);
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
int get_filtered_mean_temp(unsigned char sensor_id)
{
  unsigned char i;
  int average;

  average = (temperatures_buffer[sensor_id][0] + temperatures_buffer[sensor_id][1]) / 2;

  for(i = 2; i < FILTER_BUFFER_LENGTH; i++)
    average = average + (temperatures_buffer[sensor_id][i] - average) / (i+1);

  return average;
}

// Activate the PWM output values on the extender outputs and reset PWM timer
void
activate_pwm_state(unsigned char next_pwm_state)
{
  pwm_state = next_pwm_state;
  switch (pwm_state)
    {
  case PWM_OFF:
      PWM_PIN = 0;
    break;
   case PWM_ON:
      PWM_PIN = 1;
    break;
    }
}

// Must be called periodically to take care of PWM outputs
void
operate_PWM(void)
{
  if (timeout_occured(PWM_TIMER, TIMER_SEC, pwm_wait_time))
    {
      if(!pwm_active)
        {
          PWM_PIN = 0;
          pwm_state = PWM_OFF;
          return;
        }
      if (pwm_state == PWM_OFF)
        {
          pwm_wait_time = pwm_on_time;
          PWM_PIN = 1;
          pwm_state = PWM_ON;
        } else {  // If pwm_state == PWM_ON
          pwm_wait_time = pwm_off_time;
          PWM_PIN = 0;
          pwm_state = PWM_OFF;
        }
      reset_timeout(PWM_TIMER, TIMER_SEC);
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
      temperatures_buffer[i-1][j-1] = 0;
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

  init_ui();
}

void
operate_chilling_logoic(void)
{

  // Setup new pwm parameters
  pwm_on_time = 0;
  pwm_off_time = 0;

}

void handle_ui(void)
{
  unsigned char input_event;
  input_event = do_ui();

  switch (input_event)
  {
  case NO_INPUT_EVENT:
    break;
  case PLUS_INPUT_PRESSED:
    break;
  case MINUS_INPUT_PRESSED:
    break;
  case SET_INPUT_PRESSED:
    break;
  }

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
      // Operate main device functions
      operate_onewire_temp_measurement();

      handle_ui();

      operate_chilling_logoic();

      operate_PWM();
    }
}
