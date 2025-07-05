#include <asf.h>
#include <math.h>
#include "sam3xa.h"
#include "system_sam3x.h"
#include <stdint.h>
#include <stdbool.h>

#include <motor.h>
#include <SensorArray.h>
#include <PID.h>
#include <Ultrasonic.h>
#include <Scurve.h>

//other
int junction_count=0;
int Velocity;
int Done=0;
void enableSwitches(void);
void table1_notdone(void);
void table1(void);

int main(void) {
	// Initialize the SAM system
	sysclk_init();	
	board_init();	
	SystemInit();
	SystemCoreClockUpdate();
	init_systick();
	init_ultrasonic();

	//Enable the PIOA peripherals
	PMC->PMC_PCER0 = (1<<11);
	PMC->PMC_PCER0 = (1<<12);
	PMC->PMC_PCER0 = (1<<13);


	//left motor
 	leftMotorPIOEnable();
 	PIOC->PIO_CODR = (1 << LEFT_MOTOR_DIR); // Set Direction //Clear Output Data Register
	
	//Right motor
	rightMotorPIOEnable();
	PIOC->PIO_SODR = (1 << RIGHT_MOTOR_DIR); // Set Direction //Set Output Data Register
	
	enablePWMPeripheral();
	enableSensorArray();
	PIOB->PIO_PER |= (1 << 27); // Enable PIO control
	PIOB->PIO_OER |= (1 << 27); // Enable as output	
 	
 	/*for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
 		Velocity = calculate_s_curve_velocity_acceleration(t);
 		frequency = 40 * Velocity;
 		processLineFollowing();
 		delay_ms(50);
 	}
	decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
	*/
}
	
				
void enableSwitches(void){
	//switches
	//table1
	PIOD->PIO_PER |= (1 << 6); // Enable PIO control
	PIOD->PIO_ODR |= (1 << 6);
	
	//done serving
	PIOD->PIO_PER |= (1 << 3); // Enable PIO control
	PIOD->PIO_ODR |= (1 << 3);
	
	//table2
	PIOD->PIO_PER |= (1 << 2); // Enable PIO control
	PIOD->PIO_ODR |= (1 << 2);
}
 	

void table1(void){
	for (float t = 3.5f; t <= 7.4f; t += 0.01f) {
		Velocity = calculate_s_curve_velocity_acceleration(t);
		frequency = 40 * Velocity;
		processLineFollowing();
		delay_ms(50);

		if (((readLeftSensor() == 1) || (readRightSensor() == 1)) && (Done==0)){
			junction_count=junction_count+1;
			Done=1;
			decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
			delay_ms(6000);
			break;
			
		}

		if ((obstacle_flag) && (Done==0)){
			decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
			while(obstacle_flag){
				
			}
			break;
		}
	}
	
	table1_notdone();
	
}

void table1_notdone(void){
	if((Done==0) && (Velocity>(VMAX/2))){
		frequency = 40 * Velocity;
		processLineFollowing();
		
		if ((obstacle_flag) && (Done==0)){
			decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
			while(obstacle_flag){
			}
			if(Done==0){
				table1();
			}
		}
		
		else if (((readLeftSensor() == 1) || (readRightSensor() == 1)) && (Done==0)){
			junction_count=junction_count+1;
			Done=1;
		}
		decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
		
	}
	
	else if((Done==0) && (Velocity<(VMAX/2))){
		table1();
	}
}
