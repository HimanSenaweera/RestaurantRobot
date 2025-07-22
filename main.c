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

		"""Following Pins used for testing purposes """
// Onboard LED pin definitions (PB27)
#define LED_PIO        PIOB
#define LED_PIN_MASK   PIO_PB27
#define LED_PIN_ID     ID_PIOB

// LED for '2' → Pin 46 = PC17
#define LEDB_PIO        PIOC
#define LEDB_PIN_MASK   (1u << 17)
#define LEDB_PIN_ID     ID_PIOC

// LED for '3' → Pin 48 = PC15
#define LEDC_PIO        PIOC
#define LEDC_PIN_MASK   (1u << 15)
#define LEDC_PIN_ID     ID_PIOC

// USART1 pin definitions (TX = PA13, RX = PA12) → Peripheral A
#define UART_PIO         PIOB
#define UART_ID          ID_USART2
#define UART_RX_PIN      PIO_PB21
#define UART_TX_PIN      PIO_PB20
#define UART_PERIPHERAL  USART2



//other
int temp_2=0;
int temp=0;
int Velocity;
int Table=0;
int Return=0;
int x=1;
int Temp=0;

void table_acceleration(void);
void table_acceleration_2(void);

void leftTurn(void);
void rightTurn(void);
void table_acceleration_return_2(void);
void table_acceleration_return(void);

void communicate(void);
void blink_led(Pio *pio, uint32_t pin_mask);

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
	
	//communication
	// Initialize system clock and delay
	sysclk_init();
	delay_init(sysclk_get_cpu_hz());

	// Enable peripheral clocks
	pmc_enable_periph_clk(UART_ID);         // USART1
	pmc_enable_periph_clk(ID_PIOA);         // PIOA for UART1 pins
	pmc_enable_periph_clk(LED_PIN_ID);      // PIOB for onboard LED
	pmc_enable_periph_clk(LEDB_PIN_ID);     // PIOC for LEDB
	pmc_enable_periph_clk(LEDC_PIN_ID);     // PIOC for LEDC

	// Configure USART1 RX/TX pins (Peripheral A)
	pio_configure(UART_PIO, PIO_PERIPH_B, UART_RX_PIN | UART_TX_PIN, PIO_DEFAULT);

	// Configure USART1 options
	sam_usart_opt_t usart_settings = {
		.baudrate     = 9600,
		.char_length  = US_MR_CHRL_8_BIT,
		.parity_type  = US_MR_PAR_NO,
		.stop_bits    = US_MR_NBSTOP_1_BIT,
		.channel_mode = US_MR_CHMODE_NORMAL
	};

	// Initialize USART1
	usart_init_rs232(UART_PERIPHERAL, &usart_settings, sysclk_get_peripheral_hz());

	// Enable RX and TX
	usart_enable_tx(UART_PERIPHERAL);
	usart_enable_rx(UART_PERIPHERAL);

	// Configure LEDs as output
	pio_set_output(LED_PIO, LED_PIN_MASK, LOW, DISABLE, ENABLE);
	pio_set_output(LEDB_PIO, LEDB_PIN_MASK, LOW, DISABLE, ENABLE);
	pio_set_output(LEDC_PIO, LEDC_PIN_MASK, LOW, DISABLE, ENABLE);

	PIOC->PIO_OER |= (1<<17);     // Enable Output
	PIOC->PIO_PER |= (1<<17);     // Enable PIO control

	
	delay_ms(2000);
	while(Table==0){
		communicate();

	}
	
	//Go to the junction after table is selected
	if(Temp==0){
		table_acceleration();
		if(Table==1){
			leftTurn1();
			Temp=1;
			
		}
		else if (Table==2) {
			PIOC->PIO_CODR=(1<<17);
			rightTurn1();
			Temp=1;
		}
		
	}
	
	
	//wait for the customer input to confirm that the food was taken
	if(Temp==1){ 
		Return=0;
		delay_ms(1000);
		while(Return==0){
			
			communicate();
			
		}
		
		Temp=2;
		
		//If the selected table is 1 turn left
		if(Table==1 && Temp == 2 && Return == 1){
			leftTurn2();
			Temp = 3;
		}
		
		//If the selected Table is 2 turn right
		if (Table==2 && Temp == 2 && Return == 1){
			rightTurn2();
			// left motor
			PIOC->PIO_OER |= (1<<7);     // Enable Output
			PIOC->PIO_PER |= (1<<7);     // Enable PIO control
			PIOC->PIO_SODR = (1<<7);

			// right motor
			PIOC->PIO_OER |= (1<<6);     // Enable Output
			PIOC->PIO_PER |= (1<<6);     // Enable PIO control
			PIOC->PIO_CODR = (1<<6);
			Temp = 3;
		}
		
		
		
		Done=0;
		junction_count=0;
		
		//return to the destination
		table_acceleration_return();
		Temp=4;
		PIOC->PIO_CODR=(1<<17);
	}
	
}
 
 /// Reaching the junction where the table exists ///
void table_acceleration(void){
	temp=0;
	temp_2=0;

	//acceleration S curve
	for (float t = 3.0f; t <= 8.0f; t += 0.1f) {
		 Velocity = calculate_s_curve_velocity_acceleration(t);
		 frequency = 40 * Velocity;
		 processLineFollowing();
		 delay_ms(25);
		 
        //check for any junction
		if (((readLeftSensor() == 1) && (readRightSensor() == 1)) && (Done==0)){
			junction_count=junction_count+1;
			if(junction_count==2){
				decelerationScurve(calculate_time_from_velocity_deceleration(Velocity));
				Done=1;
			}
			 
			delay_ms(2000);
		}
		 
	 }

	//Function to reach the table once the Scurve accceleration is done
	table_acceleration_2();
}

void table_acceleration_2(void){

	//Going from the constant speed reached at the end of Scurve
	while ((temp==0) && (Done == 0) &&  ((readLeftSensor() == 0) && (readRightSensor() == 0))) {
		frequency = 40 * Velocity;
		processLineFollowing();
		
		//check for any obstacle
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

		//check for the junction and increment the variable when found
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
 
///Returning to the starting point ///
void table_acceleration_return(void){
	temp=0;
	temp_2=0;

	//Acceleration Scurve	
	for (float t = 3.0f; t <= 8.0f; t += 0.1f) {
		Velocity = calculate_s_curve_velocity_acceleration(t);
		frequency = 40 * Velocity;
		processLineFollowing(); 
		delay_ms(25);
		 
		// check for any junction
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

	// Going from the constant speed reached at the end of Scurve
	while ((temp==0) && (Done == 0) &&  ((readLeftSensor() == 0) && (readRightSensor() == 0))) {
		frequency = 40 * Velocity;
		processLineFollowing();
		 
		// check for any obstacle
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
		
		// check for the junction and increment the variable when found
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
	
	//call the function again if we have not reached the destination yet
	if(Done==0){
		table_acceleration_return();
	}
}
 
 //Turn to table1
void leftTurn(void){
	rightTurn1();
	rightTurn2();
}

//Turn to table2
void rightTurn(void){
	leftTurn1();
	leftTurn2();
	
}
 
//for USART communication via RS232
void communicate(void){
	 
	uint32_t received_char;
	if (usart_read(UART_PERIPHERAL, &received_char) == 0) {
		switch ((char)received_char) {
			case '1':
				Table=1;
			case '2':
				//PIOC->PIO_SODR=(1<<17);
				Table=2;
				PIOC->PIO_SODR=(1<<17);
			case '3':
				//PIOC->PIO_SODR=(1<<17);
				Return=1;
		}
	}
	 

 }
 4