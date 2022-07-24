/*                               | \ | |               | |(_)             | |
  __      ____      ____      __ |  \| |  ___  _ __  __| | _  _   _     __| |  ___
  \ \ /\ / /\ \ /\ / /\ \ /\ / / | . ` | / _ \| '__|/ _` || || | | |   / _` | / _ \
   \ V  V /  \ V  V /  \ V  V /_ | |\  ||  __/| |  | (_| || || |_| | _| (_| ||  __/
    \_/\_/    \_/\_/    \_/\_/(_)|_| \_| \___||_|   \__,_||_| \__, |(_)\__,_| \___|
                                                               __/ |
                                                              |___/
    BKA30D stepper motor library by Fabian Steppat
    Contact: nerdiy.de

    This is an Arduino library to control the BKA30D or compatible (e.g VID28) six state stepper motor.
    The implementation is based on the VID28 library by Gijs Withagen (https://github.com/GewoonGijs/VID28). Most credits belong to him!

    ============================================

    This is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include "nerdiys_stepper_BKA30D.h"

Stepper_BKA30D::Stepper_BKA30D(uint16_t steps_per_rotation, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
  this->current_pin_state = 0;
  this->steps_per_rotation = steps_per_rotation;
  this->state_count = STATE_COUNT;
  this->pins[0] = pin1;
  this->pins[1] = pin2;
  this->pins[2] = pin3;
  this->pins[3] = pin4;
  for (int i = 0; i < STEPPER_PIN_COUNT; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
  this->dir = 0;
  this->stepper_stopped = true;
  this->current_position = 0;
  this->target_position = 0;
  this->step_since_start = 0;
  this->velocity_min_duration = VELOCITY_MINIMUM_DELAY;
  float factor = 1.0;
  for (int i = 0; i < ACCEL_STAGES; i++)
  {
    this->accel_table[i] = (ACCEL_STEPTIME * factor);
    factor *= ACCEL_FACTOR;
  }
}

void Stepper_BKA30D::writeIO()
{
  uint8_t mask = pin_state_map[current_pin_state];
  for (int i = 0; i < STEPPER_PIN_COUNT; i++)
  {
    digitalWrite(pins[i], mask & 0x1);
    mask >>= 1;
  }
}

// disable stepper motor
void Stepper_BKA30D::powerOff()
{
  for (int i = 0; i < STEPPER_PIN_COUNT; i++)
  {
    digitalWrite(pins[i], 0);
  }
}

/*
 *  Turn one step in CW direction
 */
void Stepper_BKA30D::stepCW()
{
  // current_position = (current_position + 1) % steps_per_rotation; // this is some kind of limitation. This way current_position will never be > steps_per_rotation
  this->current_position = this->current_position + 1;
  this->current_position = this->current_position > this->steps_per_rotation ? 0 : this->current_position;
#if defined(STEPPER_BKA30D_DEBUG)
  Serial.print(F(">u>current_position: "));
  Serial.println(current_position);
#endif
  this->current_pin_state = (this->current_pin_state + 1) % this->state_count;
  writeIO();
}

/*
 *  Turn one step in CCW direction
 */
void Stepper_BKA30D::stepCCW()
{
  // current_position = (current_position + steps_per_rotation - 1) % steps_per_rotation;
  this->current_position = (this->current_position - 1);
  this->current_position = this->current_position > this->steps_per_rotation ? this->steps_per_rotation : this->current_position;
#if defined(STEPPER_BKA30D_DEBUG)
  Serial.print(F(">d>current_position: "));
  Serial.println(current_position);
#endif
  this->current_pin_state = (this->current_pin_state + (this->state_count - 1)) % this->state_count;
  writeIO();
}

/*
 *  Perform movemement and handle acceleration if needed
 */
void Stepper_BKA30D::advance()
{
  uint16_t stepsToGo;

  // detect stepper_stopped state
  if (this->current_position == this->target_position)
  {
    this->stepper_stopped = true;
    this->dir = 0;
    this->time0 = micros();
#if defined(STEPPER_BKA30D_DEBUG)
    Serial.println(F("->advance finished."));
    Serial.print(F(">a>current_position: "));
    Serial.println(current_position);
#endif
    return;
  }

  if (this->dir > 0)
  {
    stepCW();
  }
  else
  {
    stepCCW();
  }

  if (this->step_since_start < (ACCEL_STAGES * ACCEL_STEPS))
  {
    this->step_duration = this->accel_table[this->step_since_start / ACCEL_STEPS];
    this->step_since_start++; // Note, only first 64 steps matter
  }

  // Maybe we need to decelarate
  if ((this->dir > 0) &&
      ((this->target_position - this->current_position) < (ACCEL_STAGES * ACCEL_STEPS)))
  {
    stepsToGo = this->target_position - this->current_position;
  }
  else if ((this->dir < 0) &&
           ((this->current_position - this->target_position) < (ACCEL_STAGES * ACCEL_STEPS)))
  {
    stepsToGo = this->current_position - this->target_position;
  }

  if (stepsToGo < this->step_since_start)
  {
    this->step_duration = this->accel_table[stepsToGo / ACCEL_STEPS];
  }

  if (this->step_duration < this->velocity_min_duration)
    this->step_duration = this->velocity_min_duration;
  this->time0 = micros();
}

void Stepper_BKA30D::moveRelative(int16_t steps_to_go)
{
  // uint16_t diff;
  //  pos is unsigned so don't need to check for <0

  // target_position = pos % steps; // this is done to avoid multiple rotations in case the entered value is higher than the steps per rotation value
  // steps_to_go = steps_to_go % steps_per_rotation; // this is done to avoid multiple rotations in case the entered value is higher than the steps per rotation value

#if defined(STEPPER_BKA30D_DEBUG)
  Serial.println();
  Serial.println();
  Serial.print(F("->steps_to_go untouched: "));
  Serial.println(steps_to_go);
  Serial.print(F("->(-steps_per_rotation): "));
  Serial.println((int32_t)(0 - (int32_t)steps_per_rotation));
#endif

  // this limits steps_to_go to the range 0 to steps_per_rotation or -steps_per_rotation to 0 (in case steps_to_go is negative)
  /*while (steps_to_go > 0 && steps_to_go > steps_per_rotation)
  {
    steps_to_go -= steps_per_rotation;
    Serial.println(F("blubs"));
  }

  while (steps_to_go < (int32_t)(0 - (int32_t)steps_per_rotation))
  {
    steps_to_go += steps_per_rotation;
    Serial.println(F("bla"));
  }*/

  // this is done to avoid multiple rotations in case the entered (absolute) value is higher than the steps per rotation value
  /*  if (steps_to_go > steps_per_rotation)
    {
      steps_to_go = steps_to_go - steps_per_rotation;
    }
    else if (steps_to_go < (0 - steps_per_rotation))
    {
      steps_to_go = steps_to_go + steps_per_rotation;
    }*/

  // int16_t diff = current_position + target_position;

  this->target_position = this->current_position + steps_to_go;

  if (steps_to_go < 0 && this->target_position > this->steps_per_rotation) // in case of steps_to_go is negative this could lead to a negative target_position. Since target_position is unsigned this results in a very high value which needs to be limited.
  {
    this->target_position = this->current_position + steps_to_go + this->steps_per_rotation; // this brings the target_position back in the range of 0 to steps_per_rotation
  }
  else if (steps_to_go > 0 && this->target_position > this->steps_per_rotation)
  {
    this->target_position = this->current_position + steps_to_go - this->steps_per_rotation; // this brings the target_position back in the range of 0 to steps_per_rotation
  }

  /*if (diff > (int)steps / 2)
    diff = diff - steps;

  if (diff < -(int)(steps / 2))
    diff = diff + steps;*/

  // dir = (diff > 0) ? 1 : -1;
  this->dir = (steps_to_go > 0) ? 1 : -1;

  if (this->stepper_stopped)
  {
    // reset the timer to avoid possible time overflow giving spurious deltas
    this->stepper_stopped = false;
    this->time0 = micros();
    this->step_duration = 0;
    this->step_since_start = 0;
  }

#if defined(STEPPER_BKA30D_DEBUG)
  Serial.print(F("->steps_per_rotation: "));
  Serial.println(steps_per_rotation);
  Serial.print(F("->steps_to_go: "));
  Serial.println(steps_to_go);
  Serial.print(F("->current_position: "));
  Serial.println(current_position);
  Serial.print(F("->target_position: "));
  Serial.println(target_position);
  Serial.println();
  Serial.println();
#endif
}

void Stepper_BKA30D::moveRelative(int16_t steps_to_go, uint16_t speed)
{
  this->setMaxSpeed(speed);
  this->moveRelative(steps_to_go);
}

void Stepper_BKA30D::moveRelativeDegree(int16_t degree)
{
  int16_t degree_as_steps = map(degree, 0, 360, 0, this->steps_per_rotation);
  this->moveRelative(degree_as_steps);
}

void Stepper_BKA30D::moveRelativeDegree(int16_t degree, uint16_t speed)
{
  int16_t degree_as_steps = map(degree, 0, 360, 0, this->steps_per_rotation);
  this->moveRelative(degree_as_steps, speed);
}

void Stepper_BKA30D::moveToAbsolute(uint16_t position_to_go, int8_t direction)
{

  // target_position = pos % steps; // this is done to avoid multiple rotations in case the entered value is higher than the steps per rotation value
  // steps_to_go = steps_to_go % steps_per_rotation; // this is done to avoid multiple rotations in case the entered value is higher than the steps per rotation value

#if defined(STEPPER_BKA30D_DEBUG)
  Serial.println();
  Serial.println();
  Serial.print(F("->position_to_go untouched: "));
  Serial.println(position_to_go);
  Serial.print(F("->direction untouched: "));
  Serial.println(direction);
  Serial.print(F("->current_position untouched: "));
  Serial.println(current_position);
#endif

  this->target_position = position_to_go % this->steps_per_rotation;

  this->dir = direction;

  if (this->stepper_stopped)
  {
    // reset the timer to avoid possible time overflow giving spurious deltas
    this->stepper_stopped = false;
    this->time0 = micros();
    this->step_duration = 0;
    this->step_since_start = 0;
  }

#if defined(STEPPER_BKA30D_DEBUG)
  Serial.print(F("->steps_per_rotation: "));
  Serial.println(steps_per_rotation);
  Serial.print(F("->current_position: "));
  Serial.println(current_position);
  Serial.print(F("->target_position: "));
  Serial.println(target_position);
  Serial.println();
  Serial.println();
#endif
}

void Stepper_BKA30D::moveToAbsolute(uint16_t position_to_go, int8_t direction, uint16_t speed)
{
  this->setMaxSpeed(speed);
  this->moveToAbsolute(position_to_go, direction);
}

void Stepper_BKA30D::moveToAbsoluteDegree(uint16_t degree, int8_t direction)
{
  uint16_t degree_as_steps = map(degree, 0, 360, 0, this->steps_per_rotation);
  this->moveToAbsolute(degree_as_steps, direction);
}

void Stepper_BKA30D::moveToAbsoluteDegree(uint16_t degree, int8_t direction, uint16_t speed)
{
  uint16_t degree_as_steps = map(degree, 0, 360, 0, this->steps_per_rotation);
  this->moveToAbsolute(degree_as_steps, direction, speed);
}

// handle planned motor movements
void Stepper_BKA30D::update()
{
  if (!stepper_stopped)
  {
#if defined(STEPPER_BKA30D_DEBUG)
    Serial.println(F("->stepper updated."));
#endif
    uint32_t delta = micros() - this->time0;
    if (delta >= this->step_duration)
    {
#if defined(STEPPER_BKA30D_DEBUG)
      Serial.println(F("->stepper advanced."));
#endif
      advance();
    }
  }
}

void Stepper_BKA30D::setZero()
{
  this->current_position = 0;
#if defined(STEPPER_BKA30D_DEBUG)
  Serial.println(F("->Current position set as Origin."));
  Serial.print(F("->current_position: "));
  Serial.println(current_position);
  Serial.print(F("->current_position: "));
  Serial.println(this->getPosition());
#endif
}

uint16_t Stepper_BKA30D::getPosition()
{
  return this->current_position;
}

uint16_t Stepper_BKA30D::getPositionDegree()
{
  return map(this->current_position, 0, this->steps_per_rotation, 0, 360);
}

void Stepper_BKA30D::setMaxSpeed(uint16_t velocity)
{
  this->velocity_min_duration = (velocity > 0) ? (1000000 / velocity) : VELOCITY_MINIMUM_DELAY;
}

boolean Stepper_BKA30D::isRunning()
{
  return !this->stepper_stopped;
}