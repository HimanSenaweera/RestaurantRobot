#include <stdint.h>
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
#define VMAX 50.0f
#define K 1.5f
#define X0 4.5f



//motors
float leftMotorSpeed;
float rightMotorSpeed;
uint32_t frequency=40*50;

//left motor
uint32_t desiredFrequencyLeft = 50;

//right motor
uint32_t desiredFrequencyRight = 50;

//turns
uint32_t right_turn ;
uint32_t left_turn ;

//turns
void rightTurn_after_serving(void);
void leftTurn_to_serve(void);
void leftTurn_after_serving(void);
void rightTurn_to_serve(void);

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
	float exponent = -K * (t - X0);  // Notice the minus here
	float exp_value = expf(exponent);
	float v = VMAX / (1.0f + exp_value);

	return v;
}

float calculate_s_curve_velocity_decelaration(float t) {
	float exponent = K * (t - X0);
	float exp_value = expf(exponent);  
	float v = VMAX / (1.0f + exp_value);
	return v;
}



void accelerationScurve_right_turn1(void){
	int velocity;
	for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(40*velocity,0);
		delay_us(22500);
	}
	mdrive((40*50),0);
	//at the end of this we get a velocity of 0.1ms^(-1) constant speed
}

void accelerationScurve_left_turn1(void){
	int velocity;
	for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(0,40*velocity);
		delay_us(22500);
	}
	mdrive(0,(40*50));
}

void accelerationScurve_right_turn2(void){
	int velocity;
	for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(0,40*velocity);
		delay_us(23000);
	}
	mdrive(0,(40*50));
}

void accelerationScurve_left_turn2(void){
	int velocity;
	for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
		velocity = calculate_s_curve_velocity_acceleration(t);
		mdrive(40*velocity,0);
		delay_us(23000);
	}
	mdrive((40*50),0);
}

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


void decelerationScurve(float start) {
	int velocity;
 	for (float t = start; t <= 5.0f; t += 0.01f) {
 		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency = 40 * velocity;
  		processLineFollowing_deceleration();
  		delay_ms(25);
		
		if (((readLeftSensor() == 1) || (readRightSensor() == 1)) && (junction_count==0)){
			junction_count=junction_count+1;
			Done=1;
		}
  	}
 	mdrive(0, 0);
}

void decelerationScurve_right_turn1(void){
	int velocity;
	for (float t = 1.6f ; t <= 9.0f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(40*velocity,0);
		delay_us(22500);
	}
	mdrive(0,0);
}

void decelerationScurve_left_turn1(void){
	int velocity;
	for (float t = 1.6f ; t <= 9.0f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(0,40*velocity);
		delay_us(22500);
	}
	mdrive(0,0);
}

void decelerationScurve_right_turn2(void){
	int velocity;
	for (float t = 1.6f ; t <= 9.0f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(0,40*velocity);
		delay_us(23000);
	}
	mdrive(0,0);
}

void decelerationScurve_left_turn2(void){
	int velocity;
	for (float t = 1.6f ; t <= 9.0f ; t += 0.01f) {
		velocity = calculate_s_curve_velocity_decelaration(t);
		frequency=40*velocity;
		mdrive(40*velocity,0);
		delay_us(23000);
	}
	mdrive(0,0);
}
