#include "sam3xa.h"         
#include <asf.h>
#include <math.h>
#include "sam3xa.h"
#include <stdint.h>
#include <stdbool.h>

//sensor array
int readRightSensor(void);
int readLeftSensor(void);

void enableSensorArray(void);

void enableSensorArray(void){
	///sensor array///
	/// left(6) 18(5) 17(4) 16(3) 15(2) right(1) ///
	
	// right inductive proximity sensor
	PIOD->PIO_PER |= (1 << 4);	//Enable PIO control
	PIOD->PIO_ODR |= (1 << 4);	//Disable output register --> Enbale as Input
	
	// left inductive proximity sensor
	PIOA->PIO_PER |= (1 << 10);
	PIOA->PIO_ODR |= (1 << 10);//Disable output register --> Enbale as Input
	
	//other sensors
	
	//digital pin 18
	PIOA->PIO_PER |= (1 << 11); // Enable PIO control
	PIOA->PIO_ODR |= (1 << 11); // Disable output  --> Enbale as Input
	
	//digital pin 17
	PIOA->PIO_PER |= (1 << 12);// Enable PIO Control
	PIOA->PIO_ODR |= (1 << 12);// Disable output  --> Enbale as Input
	
	//digital pin 16
	PIOA->PIO_PER |= (1 << 13);//Enable PIO Control
	PIOA->PIO_ODR |= (1 << 13);//Disable output  --> Enbale as Input
	
	//digital pin 15
	PIOD->PIO_PER |= (1 << 5);//Enable PIO Control
	PIOD->PIO_ODR |= (1 << 5);//Disable output register --> Enbale as Input
}
