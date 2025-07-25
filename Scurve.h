﻿#include <stdint.h>
#include "sam3xa.h"
#include <asf.h>
#include <math.h>
#include "sam3xa.h"
#include <stdint.h>
#include <stdbool.h>
#include <asf.h>
#include <math.h>
#include "sam3xa.h"
#include "system_sam3x.h"
#include <stdint.h>
#include <stdbool.h>

//S curve variables
#define VMAX 75.0f
#define K 1.5f
#define X0 4.5f

//motors
float leftMotorSpeed;
float rightMotorSpeed;

int junction_count=0;
int Done=0;

//turns
uint32_t right_turn ;
uint32_t left_turn ;

//smooth turns
void accelerationScurve_right_turn1(void);
void accelerationScurve_right_turn2(void);
void accelerationScurve_left_turn1(void);
void accelerationScurve_left_turn2(void);
void decelerationScurve_left_turn1(void);
void decelerationScurve_left_turn2(void);
void decelerationScurve_right_turn1(void);
void decelerationScurve_right_turn2(void);

//S curve
void accelerationScurve(float start,float end ,int Delay);
void decelerationScurve(float start);
float exp_approx(float x);
float calculate_s_curve_velocity_acceleration(float t);
float calculate_s_curve_velocity_decelaration(float t);
float calculate_t_from_velocity_decelaration(float v);
float calculate_t_from_velocity_acceleration(float v);

//S-curve
// VMAX=50.0 , K=1.5 , X0=4.5
float calculate_s_curve_velocity_acceleration(float t) {
	float exponent = -K * (t - X0);  
	float exp_value = expf(exponent);
	float v = VMAX / (1.0f + exp_value);

	return v;
}

//to calculate the velocity for the deceleration curve
float calculate_s_curve_velocity_decelaration(float t) {
	float exponent = K * (t - X0);
	float exp_value = expf(exponent);
	float v = VMAX / (1.0f + exp_value);
	return v;
}

//calculate the t for a given velocity
float calculate_time_from_velocity_deceleration(int velocity) {
	if (velocity <= 0 || velocity >= (int)VMAX) {
		// Velocity out of valid range
		return -1.0f;  // Return error or sentinel value
	}

	float v = (float)velocity;
	float ratio = (VMAX / v) - 1.0f;
	float time = X0 + (1.0f / K) * logf(ratio);
	return time;
}

///Acceleration S Curve ///
void accelerationScurve(float start,float end ,int Delay){
	int velocity;
	for (float t = start; t <= end; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(40*velocity,40*velocity);
		delay_ms(Delay);
	}
	mdrive((40*50),(40*50));
	
}

/// Acceleration S curves for right turns ///
void accelerationScurve_right_turn1(void){
	int velocity;
	for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(40*velocity,0);
		delay_us(15000);
	}
	mdrive((40*50),0);
	
}

void accelerationScurve_right_turn2(void){
	int velocity;
	for (float t = 3.5f; t <= 6.0f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(0,40*velocity);
		delay_us(17500);
	}
	mdrive(0,(40*50));
}

/// Acceleration S curves for left turn ///
void accelerationScurve_left_turn1(void){
	int velocity;
	for (float t = 3.5f; t <= 7.0f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(0,40*velocity);
		delay_us(17500);
	}
	mdrive(0,(40*50));
}

void accelerationScurve_left_turn2(void){
	int velocity;
	for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(40*velocity,0);
		delay_us(17500);
	}
	mdrive((40*50),0);
}


void decelerationScurve(float start) {
	int velocity;
	for (float t = start; t <= 6.0f; t += 0.1f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency = 40 * velocity;
		processLineFollowing_deceleration();
		delay_ms(5);
		
		if (((readLeftSensor() == 1) || (readRightSensor() == 1)) && (junction_count==0)){
			junction_count=junction_count+1;
			Done=1;
		}
	}
	mdrive(0, 0);
}

/// Deceleration S curves for right turn ///
void decelerationScurve_right_turn1(void){
	int velocity;
	for (float t = 1.6f ; t <= 6.5f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(40*velocity,0);
		delay_us(18000);
	}
	mdrive(0,0);
}

void decelerationScurve_right_turn2(void){
	int velocity;
	for (float t = 1.5f ; t <= 6.6f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(0,40*velocity);
		delay_us(18000);
	}
	mdrive(0,0);
}


/// Deceleration S curves for left turn ///

void decelerationScurve_left_turn1(void){
	int velocity;
	for (float t = 2.0f ; t <= 8.0f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(0,40*velocity);
		delay_us(17500);
	}
	mdrive(0,0);
}

void decelerationScurve_left_turn2(void){
	int velocity;
	for (float t = 2.0f ; t <= 6.0f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(40*velocity,0);
		delay_us(17500);
	}
	mdrive(0,0);
}

//turning towards the table 1
void rightTurn1(void) {
	enablePWMPeripheral();
	accelerationScurve_right_turn1();
	decelerationScurve_right_turn1();
}

//turning towards the table 2
void leftTurn1(void) {
	enablePWMPeripheral();
	accelerationScurve_left_turn1();
	decelerationScurve_left_turn1();
}

//turn after serving the customer
void rightTurn2(void) {
	// left motor
	PIOC->PIO_OER |= (1<<7);     // Enable Output
	PIOC->PIO_PER |= (1<<7);     // Enable PIO control
	PIOC->PIO_CODR = (1<<7);
	// right motor
	PIOC->PIO_OER |= (1<<6);     // Enable Output
	PIOC->PIO_PER |= (1<<6);     // Enable PIO control
	PIOC->PIO_SODR = (1<<6);
	enablePWMPeripheral();
	accelerationScurve_right_turn2();
	decelerationScurve_right_turn2();
	
	// left motor
	PIOC->PIO_OER |= (1<<7);     // Enable Output
	PIOC->PIO_PER |= (1<<7);     // Enable PIO control
	PIOC->PIO_SODR = (1<<7);
	// right motor
	PIOC->PIO_OER |= (1<<6);     // Enable Output
	PIOC->PIO_PER |= (1<<6);     // Enable PIO control
	PIOC->PIO_CODR = (1<<6);
}

//turn after serving the customer
void leftTurn2(void) {
	// left motor
	PIOC->PIO_OER |= (1<<7);     // Enable Output
	PIOC->PIO_PER |= (1<<7);     // Enable PIO control
	PIOC->PIO_CODR = (1<<7);
	// right motor
	PIOC->PIO_OER |= (1<<6);     // Enable Output
	PIOC->PIO_PER |= (1<<6);     // Enable PIO control
	PIOC->PIO_SODR = (1<<6);
	
	enablePWMPeripheral();
	accelerationScurve_left_turn2();
	decelerationScurve_left_turn2();

}