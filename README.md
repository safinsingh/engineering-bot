# Honors Engineering Robot

This project was developed as part of an Honors Engineering class and involves the implementation of a PID (Proportional-Integral-Derivative) controller to control the movement of a robot. The code utilizes quadrature encoders and bump sensors to control the robot's position and velocity accurately.

## Functionality Overview

The code implements various procedures for driving the robot forward, backward, and making turns. It also incorporates a PID controller for precise control over the robot's movements. Here's a brief overview of the main functionalities:

1. **Drive Forward**: The `driveForward` function allows the robot to move forward by a specified distance in meters. It utilizes quadrature encoders to measure the robot's angular displacement and implements a PID controller to control the movement of the right motor.

2. **Drive Backward with Bump Detection**: The `driveBackwardBump` function enables the robot to move backward until both bump sensors are pressed. This is useful when the robot needs to detect and react to obstacles.

3. **Turn**: The `turn` function enables the robot to make turns by a specified number of degrees. It calculates the required encoder displacement for the turn and uses a PID controller to control the movement of the appropriate motor.

4. **PID Controller Implementation**: The code includes an implementation of a PID controller as a separate structure (`pid_t`). The controller receives a set point and adjusts the motor power accordingly to reach the desired position or velocity. The PID controller accounts for error accumulation and implements clamps on the output to ensure the motor operates within safe limits.

## Code Segments

To demonstrate the PID controller implementation, here are a couple of code segments:

```c
// PID Controller Initialization
pid_t r_control;
// kp = 0.05, ki = 0.1, kd = 0.00025
// Output clamps: 40, 90
pid_init(&r_control, totalDegrees, 0.05, 0.1, 0.00025, 40, 90);
```

The code above initializes a PID controller `r_control` with specific constants and output clamps. It sets the set point as `totalDegrees`, which is the desired angular displacement for the motor.

```c
// PID Controller Correction
float right_power = pid_correct(&r_control, SensorValue[rightEncoder]);
startMotor(rightMotor, right_power);
```

In this code segment, the PID controller calculates the corrected power value based on the current position reading (`SensorValue[rightEncoder]`). The corrected power is then used to control the `rightMotor` using the `startMotor` function.

## How to Use

To use this code for your own project, follow these steps:

1. Set up the hardware components and ensure the motors, sensors, and encoders are properly connected to the microcontroller.
2. Copy the code and ensure it is uploaded to the microcontroller or development board compatible with the VEX Robotics system.
3. Adjust the PID controller constants (`kP`, `kI`, `kD`) and output clamps (`output_clamp_min`, `output_clamp_max`) to fit your specific robot and application requirements.
4. Modify the main logic to suit your specific task or project. The pseudocode provided in the comments can serve as a guide for structuring your project.
5. Compile and run the code on the microcontroller, monitoring the debug output (if available) for any additional information or troubleshooting.

**Note:** It's essential to understand the hardware and firmware capabilities of your specific microcontroller or development board to ensure compatibility and successful execution of the code.

## Conclusion

This project demonstrates the use of a PID controller for precise control over a robot's movements. By

 leveraging quadrature encoders and bump sensors, the code enables accurate positioning and velocity control, allowing the robot to navigate through a predefined course. The provided code segments and instructions serve as a starting point for building your own robot control system based on PID principles.
