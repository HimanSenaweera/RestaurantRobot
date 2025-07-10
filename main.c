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

int temp_2=0;
int temp=0;
int Velocity;
void table_acceleration(void);
void table_acceleration_2(void);

void leftTurn(void);
void rightTurn(void);
void table_acceleration_return_2(void);
void table_acceleration_return(void);

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

	//Enable the clock for PIOC (Peripheral I/O Controller B)
	PMC->PMC_PCER0 |= (1 << ID_PIOC);
	
	enablePWMPeripheral();
	enableSensorArray();
	
 	// left motor
	PIOC->PIO_OER |= (1<<7);     // Enable Output
	PIOC->PIO_PER |= (1<<7);     // Enable PIO control
	PIOC->PIO_SODR = (1<<7);  
	// right motor
	PIOC->PIO_OER |= (1<<6);     // Enable Output
	PIOC->PIO_PER |= (1<<6);     // Enable PIO control
	PIOC->PIO_CODR = (1<<6);  
	PIOC->PIO_OER |= (1<<17);     // Enable Output
	PIOC->PIO_PER |= (1<<17);     // Enable PIO control

	table_acceleration();
	leftTurn();
	Done=0;
	junction_count=0;
	table_acceleration_return();
}
 
 void table_acceleration(void){
	 temp=0;
	 temp_2=0;
	 for (float t = 3.0f; t <= 8.0f; t += 0.1f) {
		 Velocity = calculate_s_curve_velocity_acceleration(t);
		 frequency = 40 * Velocity;
		 processLineFollowing();
		 //mdrive((40*Velocity),(40*Velocity));
		 delay_ms(25);
		 
		 if (((readLeftSensor() == 1) && (readRightSensor() == 1)) && (Done==0)){
			 junction_count=junction_count+1;
			 if(junction_count==2){
				 decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
				 Done=1;
			 }
			 
			 delay_ms(3000);
		 }
		 
	 }
	 table_acceleration_2();
 }

 void table_acceleration_2(void){
	 while ((temp==0) && (Done == 0) &&  ((readLeftSensor() == 0) && (readRightSensor() == 0))) {
		 frequency = 40 * Velocity;
		 processLineFollowing();
		 
		 if ((obstacle_flag) && (Done==0)){
			 decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
			 
			 while(temp_2==0){
				 for (int i = 0; i <= 56000000; i++) {
					 if(i==56000000){
						 if(!(obstacle_flag)){
							 temp_2=1;
						 }
						 
					 }
				 }
			 }
			 temp=1;
		 }
		 if (((readLeftSensor() == 1) || (readRightSensor() == 1)) && (Done==0)){
			 junction_count=junction_count+1;
			 if(junction_count==2){
				 
				 decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
				 temp=1;
				 Done=1;
			 }
			 
			 delay_ms(3000);
			 
			 
		 }
	 }
	 if(Done==0){
		 table_acceleration();
	 }
 }
 
 void table_acceleration_return(void){
	 temp=0;
	 temp_2=0;
	 for (float t = 3.0f; t <= 8.0f; t += 0.1f) {
		 Velocity = calculate_s_curve_velocity_acceleration(t);
		 frequency = 40 * Velocity;
		 processLineFollowing();
		 //mdrive((40*Velocity),(40*Velocity));
		 delay_ms(25);
		 
		 if (((readLeftSensor() == 1) && (readRightSensor() == 1)) && (Done==0)){
			 junction_count=junction_count+1;
			 if(junction_count==3){
				 decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
				 Done=1;
			 }
			 
			 delay_ms(2000);
		 }
		 
	 }
	 table_acceleration_return_2();
 }
 
 void table_acceleration_return_2(void){
	 while ((temp==0) && (Done == 0) &&  ((readLeftSensor() == 0) && (readRightSensor() == 0))) {
		 frequency = 40 * Velocity;
		 processLineFollowing();
		 
		 if ((obstacle_flag) && (Done==0)){
			 decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
			 
			 while(temp_2==0){
				 for (int i = 0; i <= 56000000; i++) {
					 if(i==56000000){
						 if(!(obstacle_flag)){
							 temp_2=1;
						 }
						 
					 }
				 }
			 }
			 temp=1;
		 }
		 if (((readLeftSensor() == 1) || (readRightSensor() == 1)) && (Done==0)){
			 junction_count=junction_count+1;
			 if(junction_count==3){
				 
				decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
				temp=1;
				Done=1;
			 }
			 
			 delay_ms(2000);
			 
			 
		 } 
	 }
	 if(Done==0){
		 table_acceleration_return();
	 }
 }
 
 void leftTurn(void){
	 rightTurn1();
	 rightTurn2();
 }

 void rightTurn(void){
	 leftTurn1();
	 leftTurn2();
 }