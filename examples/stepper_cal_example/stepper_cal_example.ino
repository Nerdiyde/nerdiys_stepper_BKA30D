
/*                               | \ | |               | |(_)             | |
  __      ____      ____      __ |  \| |  ___  _ __  __| | _  _   _     __| |  ___
  \ \ /\ / /\ \ /\ / /\ \ /\ / / | . ` | / _ \| '__|/ _` || || | | |   / _` | / _ \
   \ V  V /  \ V  V /  \ V  V /_ | |\  ||  __/| |  | (_| || || |_| | _| (_| ||  __/
    \_/\_/    \_/\_/    \_/\_/(_)|_| \_| \___||_|   \__,_||_| \__, |(_)\__,_| \___|
                                                               __/ |
                                                              |___/
    BKA30D stepper motor library by Fabian Steppat
    Contact: nerdiy.de

    This is an example for the Arduino library to control the BKA30D or compatible (e.g VID28) six state stepper motor. More info below.
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

/*
 * This example uses a hall sensor and little magnets (3x1 mm) at the hands
 * to make calibration possible.
 */

#include <Arduino.h>
#include "nerdiys_stepper_BKA30D.h"

#define STEPS_PER_FULL_ROTATION 1080 // for wathever reason the stepper runs more accurate when this is set to 1080. Actually 1091 should be correct. Probly needs som investigation if this is an issue in the stepper library //BKA30D=0.33 degree per step

#define MOTOR_A1_PIN 6 // PD5  //This pin is reverse with MOTOR_A4_PIN to let the motor spin in the same direction like motor B
#define MOTOR_A2_PIN 8 // PB0
#define MOTOR_A3_PIN 8 // PB0
#define MOTOR_A4_PIN 5 // PD5 //This pin is reverse with MOTOR_A1_PIN to let the motor spin in the same direction like motor B

#define MOTOR_B1_PIN 9  // PB1
#define MOTOR_B2_PIN 10 // PB2
#define MOTOR_B3_PIN 10 // PB2
#define MOTOR_B4_PIN 4  // PD4

Stepper_BKA30D motor(STEPS_PER_FULL_ROTATION, MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_A3_PIN, MOTOR_A4_PIN);
Stepper_BKA30D motor2(STEPS_PER_FULL_ROTATION, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_B3_PIN, MOTOR_B4_PIN);

void calibrate(Stepper_BKA30D motor)
{
  unsigned long now;
  unsigned int val, zero_position = 0, max = 0;
  unsigned int start_max = 4319, end_max = 0;

  max = 0;
  start_max = 4319;
  end_max = 0;
  for (int i = 0; i < 4320; i++)
  {
    val = analogRead(A3);
    Serial.println(val);
    if (val > max)
    {
      max = val;
      start_max = motor.getPosition();
    }
    if (val == max)
    {
      end_max = motor.getPosition();
    }
    motor.stepCW();
    now = micros();
    while ((micros() - now) < 2200)
    {
    }
  }
  unsigned int diff;
  diff = end_max - start_max;
  if (diff > 4320 / 2)
  {
    diff = diff + 4320;
  }
  zero_position = ((start_max + diff / 2) % 4320);
  Serial.print("Zero Position: ");
  Serial.print(zero_position);
  Serial.print("(");
  Serial.print(start_max);
  Serial.print(" - ");
  Serial.print(end_max);
  Serial.println(")");

  motor.moveToAbsolute(zero_position, NRDY_STEP_BKA30D_DIR_CW);
  while (motor.isRunning())
    motor.update();
  motor.setZero();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Enter a step position from 0 through ");
  Serial.print(STEPS_PER_FULL_ROTATION);
  Serial.println(".");

  calibrate(motor);
  calibrate(motor2);
}

static int nextPos = 0;
void loop(void)
{
  // the motor only moves when you call update()
  motor.update();
  motor2.update();

  if (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      motor.moveToAbsolute(nextPos, NRDY_STEP_BKA30D_DIR_CW);
      motor2.moveToAbsolute((nextPos + 2160), NRDY_STEP_BKA30D_DIR_CW);
      nextPos = 0;
    }
    else if (c >= '0' && c <= '9')
    {
      nextPos = 10 * nextPos + (c - '0');
    }
    else if (c == 'z')
    {
      motor.moveToAbsolute(0, NRDY_STEP_BKA30D_DIR_CW);
      motor2.moveToAbsolute(0, NRDY_STEP_BKA30D_DIR_CW);
    }
    else if (c == 'r')
    {
      calibrate(motor);
      calibrate(motor2);
    }
    else if (c == 's')
    {
      motor.setMaxSpeed(500);
      motor2.setMaxSpeed(1000);
    }
    else if (c == 'S')
    {
      motor.setMaxSpeed(250);
      motor2.setMaxSpeed(500);
    }
    else if (c == 'f')
    {
      motor.setMaxSpeed(2000);
      motor2.setMaxSpeed(2000);
    }
  }
}
