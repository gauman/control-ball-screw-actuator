# control-ball-screw-actuator
control ball screw long actuator using stepper motor driver A4988 driver.


Implemented an extent, shrink, and homing function to control ball screw long stage actuator by 
STM32 microcontroller.
Components:
1. STM32 Microcontroller.
2. Ball screw long actuator.
3. A4988 stepper motor driver

![Actuator](https://github.com/gauman/control-ball-screw-actuator/assets/78633686/18c51159-711b-4d0c-9c44-ea8b5cb9cf0f)


The homing function determines the actuator's mid value by calculating stepper motor steps.
Homing algorithm steps:
1. Initially move the actuator to the sw_limit1 endpoint.
2. Set the Total_step to ‘0’ to measure the steps between sw_limit1 and sw_limit2.
3. Move the actuator to sw_limit2 to count the steps between endpoints.
4. Calculate the midpoint by dividing the total_step.
5. Move the actuator from sw_limit2 to calculate mid_step.

6. The code is verified using the logical analyzer as shown below.
![Screenshot 2023-06-10 191652](https://github.com/gauman/control-ball-screw-actuator/assets/78633686/7389175f-5f52-4e3e-b780-7df51d56fcda)
7. ![Screenshot 2023-06-10 192015](https://github.com/gauman/control-ball-screw-actuator/assets/78633686/3f83f15b-af71-4bd7-97b2-efb6e6d7f991)
![Screenshot 2023-06-10 192133](https://github.com/gauman/control-ball-screw-actuator/assets/78633686/95c80d3b-eb4c-4869-a3be-1a3a20e03556)
