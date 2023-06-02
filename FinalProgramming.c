#pragma config(Sensor, in1, rightFollower, sensorLineFollower)
#pragma config(Sensor, in2, leftFollower, sensorLineFollower)
#pragma config(Sensor, dgtl4, rightEncoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl6, leftEncoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl8, rightBumper, sensorTouch)
#pragma config(Sensor, dgtl9, leftBumper, sensorTouch)
#pragma config(Motor, port1, rightMotor, tmotorVex393_HBridge, openLoop)
#pragma config(Motor, port3, armMotor, tmotorVex393_MC29, openLoop)
#pragma config(Motor, port4, clawMotor, tmotorVex393_MC29, openLoop)
#pragma config(Motor, port10, leftMotor, tmotorVex393_HBridge, openLoop)
#pragma DebuggerWindows("DebugStream")

/*
	Sensors used: 2 bump sensors, 2 quadrature encoders
	Pseudocode: See below
	Author: Safin Singh
*/

// measurements in meters
static const float wheel_radius = 0.054;
static const float robot_width = 0.295;

// due to play in motor and gear ratio
static const float encoder_epsilon = 30.0;

// conversion factors
static const float radians_to_degrees = 180 / PI;
static const float in_to_meters = 0.0254;
static const float gear_ratio = 30.0 / 64.0;
static const float dt = 0.01;
static const float correction_factor = 0.04;

///////// PID CONTROLLER IMPL /////////

// define PID controller structure
typedef struct {
	// target set point
	float set_point;
	// clamps on controller output
	float output_clamp_min;
	float output_clamp_max;

	// controller constants
	float kP;
	float kI;
	float kD;
	float dt;

	// error accumulation
	float previous_error;
	float sum_error;
} pid_t;

// initialize PID controller struct
void pid_init(pid_t* pid, float set_point, float kP, float kI, float kD, float output_clamp_min, float output_clamp_max) {
	pid->set_point = set_point;
	pid->output_clamp_min = output_clamp_min;
	pid->output_clamp_max = output_clamp_max;

	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->dt = dt;

	pid->previous_error = 0;
	pid->sum_error = 0;
};

// calculate corrected power based on observed point and controller
float pid_correct(pid_t* pid, float observed_point) {
	float error = pid->set_point - observed_point;
	float d_error = (error - pid->previous_error) / dt;

	float i_error = error * pid->dt;
	// prospective corrected value
	float out = pid->kP * error + pid->kI * i_error + pid->kD * d_error;

	// invert clamps if output is negative
	float c_max = pid->output_clamp_max;
	float c_min = pid->output_clamp_min;
	if (out < 0) {
		c_max = -1 * pid->output_clamp_min;
		c_min = -1 * pid->output_clamp_max;
	}

	// clamp output based on controller settings
	if (out > c_max) {
		out = c_max;
		} else if (out < c_min) {
		out = c_min;
		} else {
		pid->sum_error += i_error;
	}

	pid->previous_error = error;
	return out;
}

///////// DRIVE PROCEDURES /////////

// drive `distance` meters
void driveForward(float distance) {
	// use s = r * theta to calculate encoder angular displacement
	float totalDegrees = (distance / wheel_radius) * radians_to_degrees / gear_ratio;

	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;

	// right motor acts as a "driver"
	pid_t r_control;
	// kp = 0.15, ki = 0.2, kd = 0.0005
	// clamp(25, 90)
	pid_init(&r_control, totalDegrees, 0.05, 0.1, 0.00025, 40, 90);

	// left motor follows right (corrects inherent motor error)
	float right_encoder_prev = 0;
	float left_encoder_prev = 0;
	pid_t l_velo_control;
	// control error in velocity between motors
	pid_init(&l_velo_control, 0, 0.3, 0.0005, 0, 0, 90);
	float left_power = 70;

	// closed loop: until right encoder records an approximately correct value, ...
	while (fabs(SensorValue[rightEncoder] - totalDegrees) > encoder_epsilon) {
		// correct right power
		float right_power = pid_correct(&r_control, SensorValue[rightEncoder]);
		startMotor(rightMotor, right_power);

		// left encoder is inverted
		float left_velocity = -1 * (SensorValue[leftEncoder] - left_encoder_prev) / dt;
		float right_velocity = (SensorValue[rightEncoder] - right_encoder_prev) / dt;
		float velocity_error = left_velocity - right_velocity;
		// calculate left power based on right velocity
		float left_power_diff = pid_correct(&l_velo_control, velocity_error);
		left_power += left_power_diff * correction_factor;
		startMotor(leftMotor, left_power);

		left_encoder_prev = SensorValue[leftEncoder];
		right_encoder_prev = SensorValue[rightEncoder];
		wait1Msec(10);
	}

	stopMotor(leftMotor);
	stopMotor(rightMotor);
}

void driveBackwardBump() {
	while (SensorValue[rightBumper] != 1 && SensorValue[leftBumper] != 1) {
		startMotor(leftMotor, -90);
		startMotor(rightMotor, -90);
	}

	stopMotor(leftMotor);
	stopMotor(rightMotor);
}

///////// TURNING PROCEDURES /////////

enum turn_t {
	right,
	left
};

// drive `distance` meters
void turn(turn_t direction, float degrees) {
	// encoder = (width/2)(degrees)(pi/180)(1/2pi*wheel radius)(360)(1/gear ratio)
	// encoder = wd/2rg
	float totalDegrees = robot_width * degrees / (gear_ratio * wheel_radius);

	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;

	// only one motor is controlled in a swing turn
	pid_t m_control;
	// kp = 0.15, ki = 0.2, kd = 0.0005
	// clamp(25, 50)
	pid_init(&m_control, totalDegrees, 0.05, 0.1, 0.00025, 70, 100);

	int encoder = direction == left ? rightEncoder : leftEncoder;
	int mot = direction == left ? rightMotor : leftMotor;
	float mult = direction == left ? 0.75 : -1;
	// control encoder position error
	while (fabs(mult * SensorValue[encoder] - totalDegrees) > encoder_epsilon) {
		writeDebugStreamLine("encoder: %f, target: %f", mult * SensorValue[encoder], totalDegrees);
		float power = pid_correct(&m_control, mult * SensorValue[encoder]);
		startMotor(mot, power);
		wait1Msec(10);
	}

	stopMotor(mot);
}

///////// MAIN LOGIC /////////

/*

Pseudocode:
1. Drive forward 3 meters and turn 90 degrees to the right
2. Drive forward 4 meters and turn 90 degrees to the right
3. Drive forward 3 meters and turn 90 degrees to the left
4. Drive forward 4 meters and turn 90 degrees to the left
5. Drive forward 3 meters and turn 90 degrees to the left
6. Drive forward 4 meters and turn 90 degrees to the right
7. Drive forward 3 meters and turn 90 degrees to the right
8. Drive forward 4 meters and turn 90 degrees to the left
9. Drive forward 3 meters and turn 90 degrees to the left
10. Drive forward 7 meters and turn 90 degrees to the left
11. Back up until both bump sensors are pressed (the robot has reached the desk)
12. Drive forward 12 inches and stop

*/

task main() {
	clearDebugStream();

	// complete classroom course
	driveForward(1.5);
	turn(right, 90);
	wait(1);

	driveForward(1.85);
	turn(left, 90);
	wait(1);

	driveForward(1.5);
	turn(left, 90);
	wait(1);

	driveForward(1.85);
	turn(right, 90);
	wait(1);

	driveForward(1.5);
	turn(right, 90);
	wait(1);

	driveForward(1.85);
	turn(left, 90);
	wait(1);

	driveForward(1.5);
	turn(right, 90);
	wait(1);

	driveForward(1.85);
	turn(left, 90);
	wait(1);

	driveForward(1.5);
	turn(left, 90);
	wait(1);

	// approach table and back up into it

	driveForward(4);
	turn(left, 90);
	wait(1);

	// drive backwards into desk and move forwards

	wait(3);

	driveBackwardBump();
	driveForward(12 * in_to_meters);
}
