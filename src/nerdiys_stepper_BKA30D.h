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

#ifndef Stepper_BKA30D_h
#define Stepper_BKA30D_h

#include <Arduino.h>

//#define STEPPER_BKA30D_DEBUG // uncomment this to activate additional debug output

#define VELOCITY_MINIMUM_DELAY 500 // Minimum delay in microsecs.
#define STEPPER_PIN_COUNT 4
#define STATE_COUNT 6

#define ACCEL_STAGES 32
#define ACCEL_STEPS 8
#define ACCEL_STEPTIME 2400 // Starting with 2400 microsecs between steps, gives
#define ACCEL_FACTOR 0.95   // a minimum of 489 microsecs at step 31

#define NRDY_STEP_BKA30D_DIR_CW 1
#define NRDY_STEP_BKA30D_DIR_CCW 1

class Stepper_BKA30D
{
public:
  Stepper_BKA30D(uint16_t steps, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4);
  void stepCW();                                                                  // this performs one step in CW direction
  void stepCCW();                                                                 // this performs one step in CCW direction
  void update();                                                                  // this performs any planned steps
  void moveRelative(int16_t steps_to_go);                                         // this plans a relative movement
  void moveRelative(int16_t steps_to_go, uint16_t speed);                         // this plans a relative movement and sets the speed
  void moveRelativeDegree(int16_t degree);                                        // this plans a relative movement as degree
  void moveRelativeDegree(int16_t degree, uint16_t speed);                        // this plans a relative movement as degree and sets the speed
  void moveToAbsolute(uint16_t position_to_go, int8_t direction);                 // this plans a movement to an absolute position. Position is handled as a fracture of steps_per_rotation
  void moveToAbsolute(uint16_t position_to_go, int8_t direction, uint16_t speed); // this plans a movement to an absolute position and sets the speed. Position is handled as degree
  void moveToAbsoluteDegree(uint16_t degree, int8_t direction);                   // this plans a movement to an absolute position. Position is handled as degree
  void moveToAbsoluteDegree(uint16_t degree, int8_t direction, uint16_t speed);   // this plans a movement to an absolute position and sets the speed. Position is handled as degree
  uint16_t getPosition();                                                         // returns the current position measured in steps
  uint16_t getPositionDegree();                                                   // returns the current position as degree
  boolean isRunning();                                                            // returns true if a planned movement is ongoing
  void powerOff();                                                                // deactivates all gpios of the stepper. The stepper can rotate freely.
  void setZero();                                                                 // sets the current stepper position to zero. Can be used the use the current position as reference
  void setMaxSpeed(uint16_t velocity);                                            // sets the maximum speed for future movements

private:
  void advance(); // checks if planned movements need an update
  void writeIO(); // write next steps to gpios

  uint8_t state_count = STATE_COUNT;     // 6 states
  uint8_t pins[STEPPER_PIN_COUNT] = {0}; // gpios of the used stepper motor
  uint8_t current_pin_state = 0;         // current state in the pin cycle: Needed to maintain the proper movement of the stepper motor
  uint16_t current_position = 0;         // position we are currently at in steps
  uint16_t target_position = 0;          // target position we are plan to move to in steps
  uint16_t step_since_start = 0;         // steps since last start: used for accel/decel
  uint16_t steps_per_rotation = 0;       // Steps per revolution: Is set during setup and stepper motor specific
  uint32_t time0 = 0;                    // time when we entered this state
  uint16_t step_duration = 0;            // microsecs until next state
  uint16_t velocity_min_duration = 0;    // Minimum time between steps set by setMaxSpeed()
  int8_t dir = 0;                        // direction -1,0,1
  uint8_t stepper_stopped = true;        // true if stepper_stopped
  uint8_t accel_table[ACCEL_STAGES];     // holds velocity values to handle acceleration/decelaration
  // This should act as an accelerating / decelarating ramp
  // Probably is 32 stages of 8 steps enough
  // When starting, count number of steps / 8, as index in Array
  // When almost there (if steps_to_go < 32*8) Stepstogo / 8 as index in Array

  const uint8_t pin_state_map[6] = {0x9, 0x1, 0x7, 0x6, 0xE, 0x8};
  // Pins   1 2 3 4   Value
  // 0      1 0 0 1   0x9
  // 1      0 0 0 1   0x1
  // 2      0 1 1 1   0x7
  // 3      0 1 1 0   0x6
  // 4      1 1 1 0   0xE
  // 5      1 0 0 0   0x8
};

#endif
