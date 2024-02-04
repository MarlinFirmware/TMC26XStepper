/**
 * TMC26XStepper.cpp - - TMC26X Stepper library for Wiring/Arduino
 *
 * Based on the stepper library by Tom Igoe, et. al.
 *
 * Copyright (c) 2011, Interactive Matter, Marcus Nowotny
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <SPI.h>
#include "TMC26XStepper.h"

// Some default values used in initialization
#define DEFAULT_MICROSTEPPING_VALUE 32

// TMC26X register definitions
#define DRIVER_CONTROL_REGISTER 0x0UL
#define CHOPPER_CONFIG_REGISTER 0x80000UL
#define COOL_STEP_REGISTER  0xA0000UL
#define STALL_GUARD2_LOAD_MEASURE_REGISTER 0xC0000UL
#define DRIVER_CONFIG_REGISTER 0xE0000UL

#define REGISTER_BIT_PATTERN 0xFFFFFUL

// Definitions for the driver control register
#define MICROSTEPPING_PATTERN 0xFUL
#define STEP_INTERPOLATION 0x200UL
#define DOUBLE_EDGE_STEP 0x100UL
#define VSENSE 0x40UL
#define READ_MICROSTEP_POSTION 0x0UL
#define READ_STALL_GUARD_READING 0x10UL
#define READ_STALL_GUARD_AND_COOL_STEP 0x20UL
#define READ_SELECTION_PATTERN 0x30UL

// Definitions for the chopper config register
#define CHOPPER_MODE_STANDARD 0x0UL
#define CHOPPER_MODE_T_OFF_FAST_DECAY 0x4000UL
#define T_OFF_PATTERN 0xFUL
#define RANDOM_TOFF_TIME 0x2000UL
#define BLANK_TIMING_PATTERN 0x18000UL
#define BLANK_TIMING_SHIFT 15
#define HYSTERESIS_DECREMENT_PATTERN 0x1800UL
#define HYSTERESIS_DECREMENT_SHIFT 11
#define HYSTERESIS_LOW_VALUE_PATTERN 0x780UL
#define HYSTERESIS_LOW_SHIFT 7
#define HYSTERESIS_START_VALUE_PATTERN 0x78UL
#define HYSTERESIS_START_VALUE_SHIFT 4
#define T_OFF_TIMING_PATERN 0xFUL

// Definitions for cool step register
#define MINIMUM_CURRENT_FOURTH 0x8000UL
#define CURRENT_DOWN_STEP_SPEED_PATTERN 0x6000UL
#define SE_MAX_PATTERN 0xF00UL
#define SE_CURRENT_STEP_WIDTH_PATTERN 0x60UL
#define SE_MIN_PATTERN 0xFUL

// Definitions for stall guard2 current register
#define STALL_GUARD_FILTER_ENABLED 0x10000UL
#define STALL_GUARD_TRESHHOLD_VALUE_PATTERN 0x17F00UL
#define CURRENT_SCALING_PATTERN 0x1FUL
#define STALL_GUARD_CONFIG_PATTERN 0x17F00UL
#define STALL_GUARD_VALUE_PATTERN 0x7F00UL

// Definitions for the input from the TCM260
#define STATUS_STALL_GUARD_STATUS 0x1UL
#define STATUS_OVER_TEMPERATURE_SHUTDOWN 0x2UL
#define STATUS_OVER_TEMPERATURE_WARNING 0x4UL
#define STATUS_SHORT_TO_GROUND_A 0x8UL
#define STATUS_SHORT_TO_GROUND_B 0x10UL
#define STATUS_OPEN_LOAD_A 0x20UL
#define STATUS_OPEN_LOAD_B 0x40UL
#define STATUS_STAND_STILL 0x80UL
#define READOUT_VALUE_PATTERN 0xFFC00UL

// Default values
#define INITIAL_MICROSTEPPING 0x3UL // 32th microstepping

// Debuging output
// #define DEBUG

/**
 * Constructor
 * number_of_steps - the steps per rotation
 * cs_pin - the SPI client select pin
 * dir_pin - the pin where the direction pin is connected
 * step_pin - the pin where the step pin is connected
 */
TMC26XStepper::TMC26XStepper(int number_of_steps, int cs_pin, int dir_pin, int step_pin, unsigned int current, unsigned int resistor) {
  // We are not started yet
  started = false;
  // By default cool step is not enabled
  cool_step_enabled = false;

  // Save the pins for later use
  this->cs_pin = cs_pin;
  this->dir_pin = dir_pin;
  this->step_pin = step_pin;

  // Store the current sense resistor value for later use
  this->resistor = resistor;

  // Initizalize our status values
  this->steps_left = 0;
  this->direction = 0;

  // Initialize register values
  driver_control_register_value = DRIVER_CONTROL_REGISTER | INITIAL_MICROSTEPPING;
  chopper_config_register = CHOPPER_CONFIG_REGISTER;

  // Setting the default register values
  driver_control_register_value = DRIVER_CONTROL_REGISTER | INITIAL_MICROSTEPPING;
  microsteps = (1 << INITIAL_MICROSTEPPING);
  chopper_config_register = CHOPPER_CONFIG_REGISTER;
  cool_step_register_value = COOL_STEP_REGISTER;
  stall_guard2_current_register_value = STALL_GUARD2_LOAD_MEASURE_REGISTER;
  driver_configuration_register_value = DRIVER_CONFIG_REGISTER | READ_STALL_GUARD_READING;

  // Set the current
  setCurrent(current);
  // Set to a conservative start value
  setConstantOffTimeChopper(7, 54, 13, 12, 1);
  // Set a nice microstepping value
  setMicrosteps(DEFAULT_MICROSTEPPING_VALUE);
  // Save the number of steps
  this->number_of_steps =   number_of_steps;
}

/**
 * start & configure the stepper driver
 * just must be called.
 */
void TMC26XStepper::start() {
  #ifdef DEBUG
    Serial.println("TMC26X stepper library");
    Serial.print("CS pin: ");
    Serial.println(cs_pin);
    Serial.print("DIR pin: ");
    Serial.println(dir_pin);
    Serial.print("STEP pin: ");
    Serial.println(step_pin);
    Serial.print("current scaling: ");
    Serial.println(current_scaling, DEC);
  #endif

  // Set the pins as output & its initial value
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(cs_pin, OUTPUT);
  digitalWrite(step_pin, LOW);
  digitalWrite(dir_pin, LOW);
  digitalWrite(cs_pin, HIGH);

  // Configure the SPI interface
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  // Todo this does not work reliably - find a way to foolprof set it (e.g. while communicating
  // SPI.setDataMode(SPI_MODE3);
  SPI.begin();

  // Set the initial values
  send262(driver_control_register_value);
  send262(chopper_config_register);
  send262(cool_step_register_value);
  send262(stall_guard2_current_register_value);
  send262(driver_configuration_register_value);

  // Save that we are in running mode
  started = true;
}

/**
 * Mark the driver as unstarted to be able to start it again
 */
void TMC26XStepper::un_start() {
  started = false;
}

/**
 * Set the speed in revs per minute
 */
void TMC26XStepper::setSpeed(unsigned int whatSpeed) {
  this->speed = whatSpeed;
  this->step_delay = (60UL * 1000UL * 1000UL) / ((unsigned long)this->number_of_steps * (unsigned long)whatSpeed * (unsigned long)this->microsteps);
  #ifdef DEBUG
    Serial.print("Step delay in micros: ");
    Serial.println(this->step_delay);
  #endif
  // Update the next step time
  this->next_step_time = this->last_step_time + this->step_delay;

}

unsigned int TMC26XStepper::getSpeed(void) {
  return this->speed;
}

/**
 * Move the motor steps_to_move steps. If the number is negative,
 *  the motor moves in the reverse direction.
 */
char TMC26XStepper::step(int steps_to_move) {
  if (this->steps_left == 0) {
    this->steps_left = abs(steps_to_move);  // How many steps to take

    // Determine direction based on whether steps_to_mode is + or -:
    if (steps_to_move > 0)
      this->direction = 1;
    else if (steps_to_move < 0)
      this->direction = 0;
    return 0;
  }
  else {
    return -1;
  }
}

char TMC26XStepper::move(void) {
  // Decrement the number of steps, moving one step each time:
  if (this->steps_left > 0) {
    unsigned long time = micros();
    // Move only if the appropriate delay has passed:
    if (time >= this->next_step_time) {
      // Increment or decrement the step number,
      // Depending on direction:
      if (this->direction == 1) {
        digitalWrite(step_pin, HIGH);
      }
      else {
        digitalWrite(dir_pin, HIGH);
        digitalWrite(step_pin, HIGH);
      }
      // Get the timeStamp of when you stepped:
      this->last_step_time = time;
      this->next_step_time = time + this->step_delay;
      // Decrement the steps left:
      steps_left--;
      // Disable the step & dir pins
      digitalWrite(step_pin, LOW);
      digitalWrite(dir_pin, LOW);
    }
    return -1;
  }
  return 0;
}

char TMC26XStepper::isMoving(void) {
  return (this->steps_left > 0);
}

unsigned int TMC26XStepper::getStepsLeft(void) {
  return this->steps_left;
}

char TMC26XStepper::stop(void) {
  // Note to self if the motor is currently moving
  char state = isMoving();
  // Stop the motor
  this->steps_left = 0;
  this->direction = 0;
  // Return if it was moving
  return state;
}

void TMC26XStepper::setCurrent(unsigned int current) {
  unsigned char current_scaling = 0;
  // Calculate the current scaling from the max current setting (in mA)
  double mASetting = (double)current;
  double resistor_value = (double) this->resistor;
  // Remove vesense flag
  this->driver_configuration_register_value &= ~(VSENSE);
  // This is derrived from I=(cs+1)/32*(Vsense/Rsense)
  // Leading to cs = CS = 32*R*I/V (with V = 0,31V oder 0,165V  and I = 1000*current)
  // With Rsense=0,15
  // For vsense = 0,310V (VSENSE not set)
  // Or vsense = 0,165V (VSENSE set)
  current_scaling = (byte)((resistor_value * mASetting * 32.0 / (0.31 * 1000.0 * 1000.0)) - 0.5); // Theoretically - 1.0 for better rounding it is 0.5

  // Check if the current scalingis too low
  if (current_scaling < 16) {
    // Set the csense bit to get a use half the sense voltage (to support lower motor currents)
    this->driver_configuration_register_value |= VSENSE;
    // And recalculate the current setting
    current_scaling = (byte)((resistor_value * mASetting * 32.0 / (0.165 * 1000.0 * 1000.0)) - 0.5); // Theoretically - 1.0 for better rounding it is 0.5
    #ifdef DEBUG
      Serial.print("CS (Vsense=1): ");
      Serial.println(current_scaling);
  }
  else {
    Serial.print("CS: ");
    Serial.println(current_scaling);
    #endif
  }

  // Do some sanity checks
  if (current_scaling > 31)
    current_scaling = 31;
  // Delete the old value
  stall_guard2_current_register_value &= ~(CURRENT_SCALING_PATTERN);
  // Set the new current scaling
  stall_guard2_current_register_value |= current_scaling;
  // If started we directly send it to the motor
  if (started) {
    send262(driver_configuration_register_value);
    send262(stall_guard2_current_register_value);
  }
}

unsigned int TMC26XStepper::getCurrent(void) {
  // We calculate the current according to the datasheet to be on the safe side
  // This is not the fastest but the most accurate and illustrative way
  double result = (double)(stall_guard2_current_register_value & CURRENT_SCALING_PATTERN);
  double resistor_value = (double)this->resistor;
  double voltage = (driver_configuration_register_value & VSENSE) ? 0.165 : 0.31;
  result = (result + 1.0) / 32.0 * voltage / resistor_value * 1000.0 * 1000.0;
  return (unsigned int)result;
}

void TMC26XStepper::setStallGuardThreshold(char stall_guard_threshold, char stall_guard_filter_enabled) {
  if (stall_guard_threshold < -64)
    stall_guard_threshold = -64;
  // We just have 5 bits
  else if (stall_guard_threshold > 63)
    stall_guard_threshold = 63;
  // Add trim down to 7 bits
  stall_guard_threshold &= 0x7F;
  // Delete old stall guard settings
  stall_guard2_current_register_value &= ~(STALL_GUARD_CONFIG_PATTERN);
  if (stall_guard_filter_enabled)
    stall_guard2_current_register_value |= STALL_GUARD_FILTER_ENABLED;
  // Set the new stall guard threshold
  stall_guard2_current_register_value |= (((unsigned long)stall_guard_threshold << 8) & STALL_GUARD_CONFIG_PATTERN);
  // If started we directly send it to the motor
  if (started)
    send262(stall_guard2_current_register_value);
}

char TMC26XStepper::getStallGuardThreshold(void) {
  unsigned long stall_guard_threshold = stall_guard2_current_register_value & STALL_GUARD_VALUE_PATTERN;
  // Shift it down to bit 0
  stall_guard_threshold >>= 8;
  // Convert the value to an int to correctly handle the negative numbers
  char result = stall_guard_threshold;
  // Check if it is negative and fill it up with leading 1 for proper negative number representation
  if (result & _BV(6)) result |= 0xC0;
  return result;
}

char TMC26XStepper::getStallGuardFilter(void) {
  return (stall_guard2_current_register_value & STALL_GUARD_FILTER_ENABLED) ? -1 : 0;
}

/**
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
void TMC26XStepper::setMicrosteps(int number_of_steps) {
  long setting_pattern;
  // Poor mans log
  if (number_of_steps >= 256) {
    setting_pattern = 0;
    microsteps = 256;
  }
  else if (number_of_steps >= 128) {
    setting_pattern = 1;
    microsteps = 128;
  }
  else if (number_of_steps >= 64) {
    setting_pattern = 2;
    microsteps = 64;
  }
  else if (number_of_steps >= 32) {
    setting_pattern = 3;
    microsteps = 32;
  }
  else if (number_of_steps >= 16) {
    setting_pattern = 4;
    microsteps = 16;
  }
  else if (number_of_steps >= 8) {
    setting_pattern = 5;
    microsteps = 8;
  }
  else if (number_of_steps >= 4) {
    setting_pattern = 6;
    microsteps = 4;
  }
  else if (number_of_steps >= 2) {
    setting_pattern = 7;
    microsteps = 2;
    // 1 and 0 lead to full step
  }
  else if (number_of_steps <= 1) {
    setting_pattern = 8;
    microsteps = 1;
  }
  #ifdef DEBUG
    Serial.print("Microstepping: ");
    Serial.println(microsteps);
  #endif
  // Delete the old value
  this->driver_control_register_value &= 0xFFFF0UL;
  // Set the new value
  this->driver_control_register_value |= setting_pattern;

  // If started we directly send it to the motor
  if (started)
    send262(driver_control_register_value);
  // Recalculate the stepping delay by simply setting the speed again
  this->setSpeed(this->speed);
}

/**
 * returns the effective number of microsteps at the moment
 */
int TMC26XStepper::getMicrosteps(void) {
  return microsteps;
}

/**
 * constant_off_time: The off time setting controls the minimum chopper frequency.
 * For most applications an off time within the range of 5μs to 20μs will fit.
 *    2...15: off time setting
 *
 * blank_time: Selects the comparator blank time. This time needs to safely cover the switching event and the
 * duration of the ringing on the sense resistor. For
 *    0: min. setting 3: max. setting
 *
 * fast_decay_time_setting: Fast decay time setting. With CHM=1, these bits control the portion of fast decay for each chopper cycle.
 *    0: slow decay only
 *    1...15: duration of fast decay phase
 *
 * sine_wave_offset: Sine wave offset. With CHM=1, these bits control the sine wave offset.
 * A positive offset corrects for zero crossing error.
 *    -3..-1: negative offset 0: no offset 1...12: positive offset
 *
 * use_current_comparator: Selects usage of the current comparator for termination of the fast decay cycle.
 * If current comparator is enabled, it terminates the fast decay cycle in case the current
 * reaches a higher negative value than the actual positive value.
 *    1: enable comparator termination of fast decay cycle
 *    0: end by time only
 */
void TMC26XStepper::setConstantOffTimeChopper(char constant_off_time, char blank_time, char fast_decay_time_setting, char sine_wave_offset, unsigned char use_current_comparator) {
  // Perform some sanity checks
  if (constant_off_time < 2)
    constant_off_time = 2;
  else if (constant_off_time > 15)
    constant_off_time = 15;
  // Save the constant off time
  this->constant_off_time = constant_off_time;
  char blank_value;
  // Calculate the value acc to the clock cycles
  if (blank_time >= 54)
    blank_value = 3;
  else if (blank_time >= 36)
    blank_value = 2;
  else if (blank_time >= 24)
    blank_value = 1;
  else
    blank_value = 0;
  if (fast_decay_time_setting < 0)
    fast_decay_time_setting = 0;
  else if (fast_decay_time_setting > 15)
    fast_decay_time_setting = 15;
  if (sine_wave_offset < -3)
    sine_wave_offset = -3;
  else if (sine_wave_offset > 12)
    sine_wave_offset = 12;
  // Shift the sine_wave_offset
  sine_wave_offset += 3;

  // Calculate the register setting
  // First of all delete all the values for this
  chopper_config_register &= ~((1 << 12) | BLANK_TIMING_PATTERN | HYSTERESIS_DECREMENT_PATTERN | HYSTERESIS_LOW_VALUE_PATTERN | HYSTERESIS_START_VALUE_PATTERN | T_OFF_TIMING_PATERN);
  // Set the constant off pattern
  chopper_config_register |= CHOPPER_MODE_T_OFF_FAST_DECAY;
  // Set the blank timing value
  chopper_config_register |= ((unsigned long)blank_value) << BLANK_TIMING_SHIFT;
  // Setting the constant off time
  chopper_config_register |= constant_off_time;
  // Set the fast decay time
  // Set msb
  chopper_config_register |= (((unsigned long)(fast_decay_time_setting & 0x8)) << HYSTERESIS_DECREMENT_SHIFT);
  // Other bits
  chopper_config_register |= (((unsigned long)(fast_decay_time_setting & 0x7)) << HYSTERESIS_START_VALUE_SHIFT);
  // Set the sine wave offset
  chopper_config_register |= (unsigned long)sine_wave_offset << HYSTERESIS_LOW_SHIFT;
  // Using the current comparator?
  if (!use_current_comparator)
    chopper_config_register |= (1 << 12);
  // If started we directly send it to the motor
  if (started)
    send262(driver_control_register_value);
}

/**
 * constant_off_time: The off time setting controls the minimum chopper frequency.
 * For most applications an off time within the range of 5μs to 20μs will fit.
 *    2...15: off time setting
 *
 * blank_time: Selects the comparator blank time. This time needs to safely cover the switching event and the
 * duration of the ringing on the sense resistor. For
 *    0: min. setting 3: max. setting
 *
 * hysteresis_start: Hysteresis start setting. Please remark, that this value is an offset to the hysteresis end value HEND.
 *    1...8
 *
 * hysteresis_end: Hysteresis end setting. Sets the hysteresis end value after a number of decrements. Decrement interval time is controlled by HDEC.
 * The sum HSTRT+HEND must be <16. At a current setting CS of max. 30 (amplitude reduced to 240), the sum is not limited.
 *    -3..-1: negative HEND 0: zero HEND 1...12: positive HEND
 *
 * hysteresis_decrement: Hysteresis decrement setting. This setting determines the slope of the hysteresis during on time and during fast decay time.
 *    0: fast decrement 3: very slow decrement
 */

void TMC26XStepper::setSpreadCycleChopper(char constant_off_time, char blank_time, char hysteresis_start, char hysteresis_end, char hysteresis_decrement) {
  // Perform some sanity checks
  if (constant_off_time < 2)
    constant_off_time = 2;
  else if (constant_off_time > 15)
    constant_off_time = 15;
  // Save the constant off time
  this->constant_off_time = constant_off_time;
  char blank_value;
  // Calculate the value acc to the clock cycles
  if (blank_time >= 54)
    blank_value = 3;
  else if (blank_time >= 36)
    blank_value = 2;
  else if (blank_time >= 24)
    blank_value = 1;
  else
    blank_value = 0;
  if (hysteresis_start < 1)
    hysteresis_start = 1;
  else if (hysteresis_start > 8)
    hysteresis_start = 8;
  hysteresis_start--;

  if (hysteresis_end < -3)
    hysteresis_end = -3;
  else if (hysteresis_end > 12)
    hysteresis_end = 12;
  // Shift the hysteresis_end
  hysteresis_end += 3;

  if (hysteresis_decrement < 0)
    hysteresis_decrement = 0;
  else if (hysteresis_decrement > 3)
    hysteresis_decrement = 3;

  // First of all delete all the values for this
  chopper_config_register &= ~(CHOPPER_MODE_T_OFF_FAST_DECAY | BLANK_TIMING_PATTERN | HYSTERESIS_DECREMENT_PATTERN | HYSTERESIS_LOW_VALUE_PATTERN | HYSTERESIS_START_VALUE_PATTERN | T_OFF_TIMING_PATERN);

  // Set the blank timing value
  chopper_config_register |= ((unsigned long)blank_value) << BLANK_TIMING_SHIFT;
  // Setting the constant off time
  chopper_config_register |= constant_off_time;
  // Set the hysteresis_start
  chopper_config_register |= ((unsigned long)hysteresis_start) << HYSTERESIS_START_VALUE_SHIFT;
  // Set the hysteresis end
  chopper_config_register |= ((unsigned long)hysteresis_end) << HYSTERESIS_LOW_SHIFT;
  // Set the hystereis decrement
  chopper_config_register |= ((unsigned long)blank_value) << BLANK_TIMING_SHIFT;
  // If started we directly send it to the motor
  if (started)
    send262(driver_control_register_value);
}

/**
 * In a constant off time chopper scheme both coil choppers run freely, i.e. are not synchronized.
 * The frequency of each chopper mainly depends on the coil current and the position dependant motor coil inductivity, thus it depends on the microstep position.
 * With some motors a slightly audible beat can occur between the chopper frequencies, especially when they are near to each other. This typically occurs at a
 * few microstep positions within each quarter wave. This effect normally is not audible when compared to mechanical noise generated by ball bearings, etc.
 * Further factors which can cause a similar effect are a poor layout of sense resistor GND connection.
 * Hint: A common factor, which can cause motor noise, is a bad PCB layout causing coupling of both sense resistor voltages
 * (please refer to sense resistor layout hint in chapter 8.1).
 * In order to minimize the effect of a beat between both chopper frequencies, an internal random generator is provided.
 * It modulates the slow decay time setting when switched on by the RNDTF bit. The RNDTF feature further spreads the chopper spectrum,
 * reducing electromagnetic emission on single frequencies.
 */
void TMC26XStepper::setRandomOffTime(char value) {
  if (value)
    chopper_config_register |= RANDOM_TOFF_TIME;
  else
    chopper_config_register &= ~(RANDOM_TOFF_TIME);
  // If started we directly send it to the motor
  if (started)
    send262(driver_control_register_value);
}

void TMC26XStepper::setCoolStepConfiguration(unsigned int lower_SG_threshold, unsigned int SG_hysteresis, unsigned char current_decrement_step_size,
  unsigned char current_increment_step_size, unsigned char lower_current_limit) {
  // Sanitize the input values
  if (lower_SG_threshold > 480)
    lower_SG_threshold = 480;
  // Divide by 32
  lower_SG_threshold >>= 5;
  if (SG_hysteresis > 480)
    SG_hysteresis = 480;
  // Divide by 32
  SG_hysteresis >>= 5;
  if (current_decrement_step_size > 3)
    current_decrement_step_size = 3;
  if (current_increment_step_size > 3)
    current_increment_step_size = 3;
  if (lower_current_limit > 1)
    lower_current_limit = 1;
  // Store the lower level in order to enable/disable the cool step
  this->cool_step_lower_threshold = lower_SG_threshold;
  // If cool step is not enabled we delete the lower value to keep it disabled
  if (!this->cool_step_enabled)
    lower_SG_threshold = 0;
  // The good news is that we can start with a complete new cool step register value
  // And simply set the values in the register
  cool_step_register_value = ((unsigned long)lower_SG_threshold) | (((unsigned long)SG_hysteresis) << 8) | (((unsigned long)current_decrement_step_size) << 5)
    | (((unsigned long)current_increment_step_size) << 13) | (((unsigned long)lower_current_limit) << 15)
    // And of course we have to include the signature of the register
    | COOL_STEP_REGISTER;
  // Serial.println(cool_step_register_value,HEX);
  if (started)
    send262(cool_step_register_value);
}

void TMC26XStepper::setCoolStepEnabled(boolean enabled) {
  // Simply delete the lower limit to disable the cool step
  cool_step_register_value &= ~SE_MIN_PATTERN;
  // And set it to the proper value if cool step is to be enabled
  if (enabled)
    cool_step_register_value |= this->cool_step_lower_threshold;
  // And save the enabled status
  this->cool_step_enabled = enabled;
  // Save the register value
  if (started)
    send262(cool_step_register_value);
}

boolean TMC26XStepper::isCoolStepEnabled(void) {
  return this->cool_step_enabled;
}

unsigned int TMC26XStepper::getCoolStepLowerSgThreshold() {
  // We return our internally stored value - in order to provide the correct setting even if cool step is not enabled
  return this->cool_step_lower_threshold << 5;
}

unsigned int TMC26XStepper::getCoolStepUpperSgThreshold() {
  return (unsigned char)((cool_step_register_value & SE_MAX_PATTERN) >> 8) << 5;
}

unsigned char TMC26XStepper::getCoolStepCurrentIncrementSize() {
  return (unsigned char)((cool_step_register_value & CURRENT_DOWN_STEP_SPEED_PATTERN) >> 13);
}

unsigned char TMC26XStepper::getCoolStepNumberOfSGReadings() {
  return (unsigned char)((cool_step_register_value & SE_CURRENT_STEP_WIDTH_PATTERN) >> 5);
}

unsigned char TMC26XStepper::getCoolStepLowerCurrentLimit() {
  return (unsigned char)((cool_step_register_value & MINIMUM_CURRENT_FOURTH) >> 15);
}

void TMC26XStepper::setEnabled(boolean enabled) {
  // Delete the t_off in the chopper config to get sure
  chopper_config_register &= ~(T_OFF_PATTERN);
  if (enabled)
    // And set the t_off time
    chopper_config_register |= this->constant_off_time;
  // If not enabled we don't have to do anything since we already delete t_off from the register
  if (started)
    send262(chopper_config_register);
}

boolean TMC26XStepper::isEnabled() {
  return (boolean)(chopper_config_register & T_OFF_PATTERN);
}

/**
 * reads a value from the TMC26X status register. The value is not obtained directly but can then
 * be read by the various status routines.
 *
 */
void TMC26XStepper::readStatus(char read_value) {
  unsigned long old_driver_configuration_register_value = driver_configuration_register_value;
  // Reset the readout configuration
  driver_configuration_register_value &= ~(READ_SELECTION_PATTERN);
  // This now equals TMC26X_READOUT_POSITION - so we just have to check the other two options
  if (read_value == TMC26X_READOUT_STALLGUARD)
    driver_configuration_register_value |= READ_STALL_GUARD_READING;
  else if (read_value == TMC26X_READOUT_CURRENT)
    driver_configuration_register_value |= READ_STALL_GUARD_AND_COOL_STEP;
  // All other cases are ignored to prevent funny values
  // Check if the readout is configured for the value we are interested in
  if (driver_configuration_register_value != old_driver_configuration_register_value)
    // Because then we need to write the value twice - one time for configuring, second time to get the value, see below
    send262(driver_configuration_register_value);
  // Write the configuration to get the last status
  send262(driver_configuration_register_value);
}

int TMC26XStepper::getMotorPosition(void) {
  // We read it out even if we are not started yet - perhaps it is useful information for somebody
  readStatus(TMC26X_READOUT_POSITION);
  return getReadoutValue();
}

// Reads the stall guard setting from last status
// Returns -1 if stallguard information is not present
int TMC26XStepper::getCurrentStallGuardReading(void) {
  // If we don't yet started there cannot be a stall guard value
  if (!started)
    return -1;
  // Not time optimal, but solution optiomal:
  // First read out the stall guard value
  readStatus(TMC26X_READOUT_STALLGUARD);
  return getReadoutValue();
}

unsigned char TMC26XStepper::getCurrentCSReading(void) {
  // If we don't yet started there cannot be a stall guard value
  if (!started)
    return 0;
  // Not time optimal, but solution optiomal:
  // First read out the stall guard value
  readStatus(TMC26X_READOUT_CURRENT);
  return (getReadoutValue() & 0x1F);
}

unsigned int TMC26XStepper::getCurrentCurrent(void) {
  double result = (double)getCurrentCSReading();
  double resistor_value = (double)this->resistor;
  double voltage = (driver_configuration_register_value & VSENSE) ? 0.165 : 0.31;
  result = (result + 1.0) / 32.0 * voltage / resistor_value * 1000.0 * 1000.0;
  return (unsigned int)result;
}

/**
 return true if the stallguard threshold has been reached
*/
boolean TMC26XStepper::isStallGuardOverThreshold(void) {
  if (!this->started) return false;
  return (driver_status_result & STATUS_STALL_GUARD_STATUS);
}

/**
 returns if there is any over temperature condition:
 OVER_TEMPERATURE_PREWARING if pre warning level has been reached
 OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
 Any of those levels are not too good.
*/
char TMC26XStepper::getOverTemperature(void) {
  if (!this->started)
    return 0;
  if (driver_status_result & STATUS_OVER_TEMPERATURE_SHUTDOWN)
    return TMC26X_OVERTEMPERATURE_SHUTDOWN;
  if (driver_status_result & STATUS_OVER_TEMPERATURE_WARNING)
    return TMC26X_OVERTEMPERATURE_PREWARING;
  return 0;
}

// Is motor channel A shorted to ground
boolean TMC26XStepper::isShortToGroundA(void) {
  if (!this->started) return false;
  return (driver_status_result & STATUS_SHORT_TO_GROUND_A);
}

// Is motor channel B shorted to ground
boolean TMC26XStepper::isShortToGroundB(void) {
  if (!this->started) return false;
  return (driver_status_result & STATUS_SHORT_TO_GROUND_B);
}

// Is motor channel A connected
boolean TMC26XStepper::isOpenLoadA(void) {
  if (!this->started) return false;
  return (driver_status_result & STATUS_OPEN_LOAD_A);
}

// Is motor channel B connected
boolean TMC26XStepper::isOpenLoadB(void) {
  if (!this->started) return false;
  return (driver_status_result & STATUS_OPEN_LOAD_B);
}

// Is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
boolean TMC26XStepper::isStandStill(void) {
  if (!this->started) return false;
  return (driver_status_result & STATUS_STAND_STILL);
}

// Is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
boolean TMC26XStepper::isStallGuardReached(void) {
  if (!this->started) return false;
  return (driver_status_result & STATUS_STALL_GUARD_STATUS);
}

// Reads the stall guard setting from last status
// Returns -1 if stallguard inforamtion is not present
int TMC26XStepper::getReadoutValue(void) {
  return (int)(driver_status_result >> 10);
}

int TMC26XStepper::getResistor() {
  return this->resistor;
}

boolean TMC26XStepper::isCurrentScalingHalfed() {
  return (boolean)(this->driver_configuration_register_value & VSENSE);
}

/**
 version() returns the version of the library:
 */
int TMC26XStepper::version(void) {
  return 1;
}

void TMC26XStepper::debugLastStatus() {
  #ifdef DEBUG
    if (this->started) {
      if (this->getOverTemperature() & TMC26X_OVERTEMPERATURE_PREWARING)
        Serial.println("WARNING: Overtemperature Prewarning!");
      else if (this->getOverTemperature() & TMC26X_OVERTEMPERATURE_SHUTDOWN)
        Serial.println("ERROR: Overtemperature Shutdown!");
      if (this->isShortToGroundA())
        Serial.println("ERROR: SHORT to ground on channel A!");
      if (this->isShortToGroundB())
        Serial.println("ERROR: SHORT to ground on channel A!");
      if (this->isOpenLoadA())
        Serial.println("ERROR: Channel A seems to be unconnected!");
      if (this->isOpenLoadB())
        Serial.println("ERROR: Channel B seems to be unconnected!");
      if (this->isStallGuardReached())
        Serial.println("INFO: Stall Guard level reached!");
      if (this->isStandStill())
        Serial.println("INFO: Motor is standing still.");
      unsigned long readout_config = driver_configuration_register_value & READ_SELECTION_PATTERN;
      int value = getReadoutValue();
      if (readout_config == READ_MICROSTEP_POSTION) {
        Serial.print("Microstep postion phase A: ");
        Serial.println(value);
      }
      else if (readout_config == READ_STALL_GUARD_READING) {
        Serial.print("Stall Guard value:");
        Serial.println(value);
      }
      else if (readout_config == READ_STALL_GUARD_AND_COOL_STEP) {
        int stallGuard = value & 0xF;
        int current = value & 0x1F0;
        Serial.print("Approx Stall Guard: ");
        Serial.println(stallGuard);
        Serial.print("Current level");
        Serial.println(current);
      }
    }
  #endif // DEBUG
}

/**
 * send register settings to the stepper driver via SPI
 * returns the current status
 */
inline void TMC26XStepper::send262(unsigned long datagram) {
  unsigned long i_datagram;

  // Preserver the previous spi mode
  unsigned char oldMode =  SPCR & SPI_MODE_MASK;

  // If the mode is not correct set it to mode 3
  if (oldMode != SPI_MODE3)
    SPI.setDataMode(SPI_MODE3);

  // Select the TMC driver
  digitalWrite(cs_pin, LOW);

  // Ensure that only valid bist are set (0-19)
  // Datagram &=REGISTER_BIT_PATTERN;

  #ifdef DEBUG
    Serial.print("Sending ");
    Serial.println(datagram, HEX);
  #endif

  // Write/read the values
  i_datagram = SPI.transfer((datagram >> 16) & 0xFF);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >>  8) & 0xFF);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xFF);
  i_datagram >>= 4;

  #ifdef DEBUG
    Serial.print("Received ");
    Serial.println(i_datagram, HEX);
    debugLastStatus();
  #endif

  // Deselect the TMC chip
  digitalWrite(cs_pin, HIGH);

  // Restore the previous SPI mode if neccessary
  // If the mode is not correct set it to mode 3
  if (oldMode != SPI_MODE3)
    SPI.setDataMode(oldMode);

  // Store the datagram as status result
  driver_status_result = i_datagram;
}
