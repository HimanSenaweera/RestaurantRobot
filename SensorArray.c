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
	/// left(20) 19 18 15 14 right(13) ///   
	/// PD7 PD6 PD5 PD2 PD1 PD0 ///   
	
	PMC->PMC_PCER0 |= (1 << ID_PIOD);
	
	//13
	PIOD->PIO_PER |= (1 << 0);	//Enable PIO control
	PIOD->PIO_ODR |= (1 << 0);	//Disable output register --> Enbale as Input
	
	//14
	PIOD->PIO_PER |= (1 << 1);
	PIOD->PIO_ODR |= (1 << 1); //Disable output register --> Enbale as Input
	
	//15
	PIOD->PIO_PER |= (1 << 2); // Enable PIO control
	PIOD->PIO_ODR |= (1 << 2); // Disable output  --> Enbale as Input
	
	//18
	PIOD->PIO_PER |= (1 << 5); // Enable PIO Control
	PIOD->PIO_ODR |= (1 << 5); // Disable output  --> Enbale as Input
	
	//19
	PIOD->PIO_PER |= (1 << 6); // Enable PIO Control
	PIOD->PIO_ODR |= (1 << 6); // Disable output  --> Enbale as Input
	
	//20
	PIOD->PIO_PER |= (1 << 7);// Enable PIO Control
	PIOD->PIO_ODR |= (1 << 7);// Disable output register --> Enbale as Input
	
}

int readRightSensor(void){
	return ((PIOD->PIO_PDSR & (1 << 0))!=0);
}

int readLeftSensor(void){
	return ((PIOD->PIO_PDSR & (1 << 7))!=0);
}
