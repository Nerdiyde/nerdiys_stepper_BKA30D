# Nerdiys BKA30D Stepper library

Please note: This is still in development!

This is an Arduino library to control the BKA30D or compatible (e.g VID28) six state stepper motor.
Please see the example in examples\stepper_cal_example to see an implementation to control the steppers and perform a simple positional calibration to create a reference for absolute positioning.

It is used in my "Clockception" project which will be published on my blog Nerdiy.de soon.
 
### Planned features
 - Implement direction auto detect based on "shortest way" to target position
 - Functionality that makes it possible to go to a specific target position after a specific number of steps/degree was performed

### License
Unless otherwise stated, all works presented here and on Nerdiy.de that are not based on software/code are subject to the CC BY-NC-SA 4.0 license (attribution - non-commercial - dissemination under the same conditions 4.0 international).
You can find additional infos here: https://nerdiy.de/en/lizenz/

### Credits:
 - The implementation is inspired by the VID28 library of Gijs Withagen (https://github.com/GewoonGijs/VID28)