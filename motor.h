#include <stdint.h>
#include "sam3xa.h"         
#include <asf.h>
#include <math.h>
#include "sam3xa.h"
#include <stdint.h>
#include <stdbool.h>



#define  LEFT_MOTOR_DIR 25				//PC25 (5)
#define  LEFT_MOTOR_PU 2				//PC2 (34)
#define  LEFT_MOTOR_PWM_CHANNEL 0

//right motor
#define  RIGHT_MOTOR_DIR 28				//PC28 (3)
#define  RIGHT_MOTOR_PU 4				//PC4 (36)
#define  RIGHT_MOTOR_PWM_CHANNEL 1

//motors
float leftMotorSpeed;
float rightMotorSpeed;
uint32_t frequency=40*50;

//left motor
uint32_t desiredFrequencyLeft = 50;

//right motor
uint32_t desiredFrequencyRight = 50;


void enablePWMPeripheral(void);

//Enable/Disable Peripherals
void leftMotorPIOEnable(void);
void rightMotorPIOEnable(void);

//line following
void setup_pwm(void);
void mdrive(uint32_t desired_left, uint32_t desired_right);


void leftMotorPIOEnable(void){
	PIOC->PIO_PER |= (1 << LEFT_MOTOR_DIR); // Enable PIO control
	PIOC->PIO_OER |= (1 << LEFT_MOTOR_DIR); // Enable as output
}

void rightMotorPIOEnable(void){
	PIOC->PIO_PER |= (1 << RIGHT_MOTOR_DIR); // Enable PIO control
	PIOC->PIO_OER |= (1 << RIGHT_MOTOR_DIR); // Enable as output
}

void enablePWMPeripheral(void) { 
	
	// Enable PWM peripheral clock using PMC (Power Management Controller)
	PMC->PMC_PCER1 = (1 << (36 - 32)); // Enabling PWM peripheral
	
	PIOC->PIO_PDR = (1 << LEFT_MOTOR_PU); // Disable PIO control for PC2
	PIOC->PIO_ABSR |= (1 << LEFT_MOTOR_PU); // Select peripheral B for PC2 (PWM)
	
	
	PIOC->PIO_PDR = (1 << RIGHT_MOTOR_PU); // Disable PIO control for PC4
	PIOC->PIO_ABSR |= (1 << RIGHT_MOTOR_PU); // Select peripheral B for PC4 (PWM)
	
	// Configure PWM clock
	PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(84); // 84MHz / 84 = 1MHz  //base freq
	
	// Configure PWM Channel 0 LEFT MOTOR
	PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_CLKA; // Use Clock A
	
	// Configure PWM Channel 1 RIGHT MOTOR
	PWM->PWM_CH_NUM[1].PWM_CMR = PWM_CMR_CPRE_CLKA; // Use Clock A
}

void mdrive(uint32_t desired_left, uint32_t desired_right) {
	
	// Calculate period ticks for 1MHz clock
	uint32_t periodTicks_left = 1000000 / desired_left;
	uint32_t periodTicks_right = 1000000 / desired_right;
	
	
	PWM->PWM_CH_NUM[0].PWM_CPRD = periodTicks_left;     // we divide the base freq by this to determine the PWM freq
	PWM->PWM_CH_NUM[0].PWM_CDTY = periodTicks_left / 2; // Set duty cycle to 50% for LEFT motor
	
	
	PWM->PWM_CH_NUM[1].PWM_CPRD = periodTicks_right; // Set period for RIGHT motor (Channel 1)
	PWM->PWM_CH_NUM[1].PWM_CDTY = periodTicks_right / 2; // Set duty cycle to 50% for RIGHT motor
	
	if(desired_left==0 && desired_right==0){
		PWM->PWM_DIS = (1 << 0);
		PWM ->PWM_DIS = (1 << 1);

	}
	else if (desired_left==0 ){
		PWM->PWM_DIS = (1 << 0);
		PWM->PWM_ENA = (1 << 1); 
	}
	else if (desired_right==0){
		PWM->PWM_ENA = (1 << 0); // Enable Channel 0
		PWM ->PWM_DIS = (1 << 1);
	}
	else{
		PWM->PWM_ENA = (1 << 0); // Enable Channel 0
		PWM->PWM_ENA = (1 << 1); // Enable Channel 1
	}	
}
