#include <stdint.h>
#include "sam3xa.h"
#include <asf.h>
#include <math.h>
#include "sam3xa.h"
#include <stdint.h>
#include <stdbool.h>

// PID constants
int PID(int current_error);
int update_pid(void);
void processLineFollowing(void);
int update_pid_decelaration(void);
float error;

//Line Following Constants
float Kp_a = 15; //acceleration
float Kp_d =25;  //deceleration
float Ki = 0.0;
float Kd = 0;
int last_error = 0;
int integral = 0;

int PID(int current_error)
{
	integral += current_error;
	int derivative = current_error - last_error;
	int output = (Kp_d * current_error) + (Ki * integral) + (Kd * derivative);
	last_error = current_error;
	return output;
}


int update_pid(void){
	// === Read sensor values ===
	uint8_t sensors[4];
	sensors[0] = (PIOD->PIO_PDSR & (1 << 6)) != 0;
	sensors[1] = (PIOD->PIO_PDSR & (1 << 5)) != 0;
	sensors[2] = (PIOD->PIO_PDSR & (1 << 2)) != 0;
	sensors[3] = (PIOD->PIO_PDSR & (1 << 1)) != 0;
	

	// === Weights ===
	int8_t weights[4] = {-3, -1, 1, 3};

	// === Error calculation ===
	int16_t sum = 0;
	uint8_t active_count = 0; //# of sensors that are high

	for (int i = 0; i < 4; i++) {
		if (sensors[i]) {
			sum += weights[i];
			active_count++;
		}
	}

	int16_t Error = 0;
	if (active_count > 0) {
		Error = sum / active_count;
	}
	int output = PID(Error);
	return output;
}

void processLineFollowing_deceleration(void) {
	error = update_pid_decelaration();
	float error_mini = (error * 0.2);
	if (frequency < 600) {
		leftMotorSpeed = 0;
		rightMotorSpeed = 0;
	}
	
	else {
		leftMotorSpeed = frequency - (40 * error_mini);
		rightMotorSpeed = frequency + (40 * error_mini);
	}
	mdrive(leftMotorSpeed, rightMotorSpeed);
	delay_ms(20);
}

int update_pid_decelaration(void) {
	// === Read sensor values ===
	uint8_t sensors[4];
	sensors[0] = (PIOD->PIO_PDSR & (1 << 6)) != 0;
	sensors[1] = (PIOD->PIO_PDSR & (1 << 5)) != 0;
	sensors[2] = (PIOD->PIO_PDSR & (1 << 2)) != 0;
	sensors[3] = (PIOD->PIO_PDSR & (1 << 1)) != 0;
	
	int8_t weights[4] = {-3,-1, 1, 3};
	int16_t sum = 0;
	uint8_t active_count = 0;
	
	for (int i = 0; i < 4; i++) {
		if (sensors[i]) {
			sum += weights[i];
			active_count++;
		}
	}
	
	int16_t Error = 0;
	if (active_count > 0) {
		Error = sum / active_count;
	}
	int output = PID(Error);
	return output;
}

void processLineFollowing(void){
	error=update_pid();
	float error_mini=(error*0.2);

	leftMotorSpeed=frequency-(40*error_mini);
	rightMotorSpeed=frequency+(40*error_mini);
	
	mdrive(leftMotorSpeed,rightMotorSpeed);
	delay_ms(20);
}
