# adc_llc
Auto Driving Car Low Level Control

## Description
This package create a ROS node that subscribe to "cmd_vel" topic.

It is capable of translating the cmd_vel.linear.x and cmd_vel.angular.z to the differential drive Left and Right RPM values and computing the PWM values required for both motors.

## Contributor
Zonghao Huang (kyle0923@qq.com)

## Key Word
ROS, differential drive, PWM, H bridge, DC motor
