# Samuko Motor Controller (**`smc`**) Project
The parent repo of the Samuko Motor Controller (**`smc`**) Project.
<br/>
<br/>
One of the problems faced by students, learners, researchers, hobbyists, and even some startups in Mobile Robotics is `precise velocity control of DC Motors`. 
<br/>
Traditionally, DC motors are controlled using PWM signals via an H-Bridge driver module which does not provide good velocity control of the connected motor(s) with feedback.
<br/>
This generally makes it hard to design a good and precise `motor velocity controller` for the interfaced motor, not to mention implementing good motion planning algorithms on the mobile robot. 
<br/>
Because of this, most mobile robotics projects revolves around (or are usually limited to) Telemetry operation, obstacle avoidance, line following, Opencv lane tracking/following (sometimes with AI), and so on or such that does not require any advanced motion planning and navigation algorithms like SLAM and so on.
<br/>
<br/>
The AIM of this project is to design and make available a motor driver system `(i.e a physical motor driver module, software and libraries)` that uses the PID control algorithm to help setup `precise velocity control for geared dc motors that uses quadrature encoders` and provide means for easy integration into any desired mobile robotics project.

>The motor driver system will allow the user to:
> - easily connect/interface any geared dc motor with a quadrature encoder to it.
> - easily setup the motor encoder and configure a velocity PID control for the connected/interfaced motor.
> - contol the motor directly with angular velocity commands (not PWM).
> - get the motor’s actual angular position and angular velocity as feedback.
> - easily integrate it into your microcontroller-based (Arduino) project.
> - easily integrate it into your microcomputer-based (Raspberry Pi, etc.) project.
> - easily integrate it into your ROS2-based project in wheeled mobile robotics.

> NOTE: `It can also be used in other projects that requires precise motor velocity control`

<br/>
The `smc` project consist of the following sub-parts:

- **`smc_l298n_pid_driver module`**: It has a controller that handles the PID control. here's the link to the repo of the control driver code -> [smc_l298n_pid_driver_code](https://github.com/samuko-things-company/smc_l298n_pid_driver_code). The module provides a USB serial communication interface using the FTDI programmer to connect with a PC or micro-computer during Encoder and PID control parameter setup, microcomputer-based (Raspberry Pi, etc.) projects or ROS2-based projects. It also provides an I2C communication interface for microcontroller-based (Arduino) projects.

![smc_img](./docs/smc_img2.jpeg)

- **`smc_app`**: to help setup the Encoder and PID control parameters for the **geared DC motor with a quadrature encoder** connected to the **smc_l298n_pid_driver module**. Here's the lik to the repo -> [smc_app](https://github.com/samuko-things-company/smc_app)
- **`smc_pyserial_lib`**: library that helps communicate with the already setup motor control in you PC or microcomputer-based python projects. here's a link to the repo -> [smc_pyserial_lib](https://github.com/samuko-things-company/smc_pyserial_lib) 
- **`smc_cppserial_lib`**: library that helps communicate with the already setup motor control in you PC or microcomputer-based cpp projects. here's a link to the repo -> [smc_cppserial_lib](https://github.com/samuko-things-company/smc_cppserial_lib) 
- **`smc_ros2_hw_plugin`**: ros2 harware interface plugin package to help communicate with the already setup motor control in your ROS2-based mobile robotics project **(currently implemented in ROS2-humble LTS)**. here's a link to the repo -> [smc_ros2_hw_plugin](https://github.com/samuko-things-company/smc_ros2_hw_plugin) 
- **`smc_i2c_lib`**: arduino library that helps communicate with the already setup motor control via I2C in your arduino-based project (e.g Arduino nano, UNO, MEGA, ESP32, e.t.c). here's the link to the repo -> [smc_i2c_lib](https://github.com/samuko-things-company/smc_i2c_lib) 
<br/>
<br/>

![smc sub part](./docs/samuko_motor_controller_diagram1.drawio%20(1).png)

## HOW TO USE THE **`smc`** IN YOUR PROJECT
- First of all get the `smc_l298n_pid_driver module` (it will come preloaded with the control code and also with an FTDI serial programmer for USB serial communication) then interface/connect your motor(s) with quadrature encoder to it and power it up.

- Download and install the `smc_app` on you PC (or clone the repo into your PC and run the application code). 

- Connect the `smc_l298n_pid_driver module` to your PC via the FTDI and start the `smc_app`. Setup your motor encoder and PID parameter.
  > **NOTE:** the parameter values you set are automatically saved to the microcontroller's memory (i.e it remembers the parameter values)

- After successfully setingup the PID controller, close the application, disconnect the driver module from the PC and hit the reset button on the `smc_l298n_pid_driver module`.

- connect it to your prefered project using any of the API library - `smc_pyserial_lib`, `smc_cppserial_lib`, `smc_ros2_hw_plugin`, or `smc_i2c_lib`.

## RESOURCES USED AND THEIR REFERENCES

|S/N|AUTHOR(S)|INFO/LINKS|WHERE/HOW IT WAS USED|
|--|--|--|:--:|
|01|Kevin Lynch</br>(Northwestern Robotics)|[PID Feedback Control Series](https://www.youtube.com/playlist?list=PLggLP4f-rq02ecj0q0VB9jEbfua8lOc_E)|It formed the basis of the PID algorithm and process used in the **`smc_l298n_pid_driver_code`**. </br>It also formed the process of how the PID parameters is tuned experimentally.|
|02|Subcooled Mind|[Anti-windup for intergrtor](https://www.youtube.com/watch?v=UMit8mVCJ_I&list=PLhfTXXGugELP3ZbLftFk_5UjMq5sIsR15&index=11)|Improved the PID algorithm with Anti windup for the integrator in the **`smc_l298n_pid_driver_code`**.|
|03|Brian Douglas</br>(Matlab Tech Talks)|[PID Control Tutorial Series](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)|Explained PID control concepts which aided in the development of the **`smc`** project.|
|04|Curio Res|[DC Motor Speed Control](https://www.youtube.com/watch?v=HRaZLCBFVDE&list=PLhfTXXGugELP3ZbLftFk_5UjMq5sIsR15&index=15)|It formed the basis of the algorithm of the motor speed calculation with encoders in the **`smc_l298n_pid_driver_code`**</br>It also provided an understanding of filtering velocity reading and feedback control in the driver code.|
|05|Curio Res|[Digital FIlter on Arduino 1](https://www.youtube.com/watch?v=HJ-C4Incgpw&list=PLhfTXXGugELP3ZbLftFk_5UjMq5sIsR15&index=12&pp=gAQBiAQB)</br>[Digital FIlter on Arduino 2](https://www.youtube.com/watch?v=eM4VHtettGg&list=PLhfTXXGugELP3ZbLftFk_5UjMq5sIsR15&index=13&pp=gAQBiAQB)|Provided practical understanding of digital filters and how it is used on Arduino.</br>It provided the arduino code on which the filter library (header file) used in the **`smc_l298n_pid_driver_code`** was built.</br>It helped in filtering noise in the computed motor speed from the encoders|
|06|Josh Newans</br>(Articulated Robotics)|[Using Harware With ros2_control - Video](https://www.youtube.com/watch?v=J02jEKawE5U&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=21&pp=iAQB)</br>[Using Hardware With ros2_control - Article](https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/)</br>[Sample diffdrive hardware - Github](https://github.com/joshnewans/diffdrive_arduino)|Understanding How to interface hardware with ros2 for motor controls</br>Adapted his diffdrive ros2 hardware interfaces code in developing the **`smc_ros2_interface`** plugin and the **`smc_cppserial_lib`**</br>Understanding how to build a real mobile robot in ROS|
|07|Muhammad Rafay Khan, et al.|[Speed Control of DC Motor under Varying Load Using PID Controller - Journal](https://www.cscjournals.org/manuscript/Journals/IJE/Volume9/Issue3/IJE-485.pdf)|Explained and showed how effective the PID controller is under varied loading.|
|08|Obiagba Samuel|[Serial Communication between Arduino and PC - Github](https://github.com/samuko-things/serial_comm_pyserial_and_arduino)|Formed the basis of the communication algorithm used in the **`smc_pyserial_lib`** and the driver code|