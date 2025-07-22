#include "sam3xa.h"
#include "system_sam3x.h"
#include <stdint.h>
#include <stdbool.h>

// Pin definitions
#define LED1_PORT   PIOC
#define LED1_PIN    (1u << 17)   // onboard LED, D13

#define TRIG_PIO    PIOC
#define TRIG_PIN    (1u << 14)   // ultrasonic trigger, D49

#define ECHO_PIO    PIOA
#define ECHO_PIN    (1u <<  2)   // ultrasonic echo, D61 (TIOA1)


// Timer channel & ID for echo capture
#define TC_ECHO_CH   1
#define TC_ECHO_ID   ID_TC1


// Obstacle threshold
#define THRESHOLD_CM 70.0f

// SysTick & ping timing
static volatile uint32_t msTicks;
static volatile bool obstacle_flag = false;

#define PING_INTERVAL_MS  60    // ping every 60 ms (~16 Hz)
#define PULSE_WIDTH_MS     1    // 1 ms low pulse
static volatile uint32_t last_ping = 0;
static volatile bool in_pulse = false;

void SysTick_Handler(void) {
	msTicks++;
	// start new pulse
	if ((msTicks - last_ping) >= PING_INTERVAL_MS) {
		last_ping = msTicks;
		in_pulse  = true;
		TRIG_PIO->PIO_CODR = TRIG_PIN;  // drive low
	}
	// end pulse
	else if (in_pulse && (msTicks - last_ping) >= PULSE_WIDTH_MS) {
		TRIG_PIO->PIO_SODR = TRIG_PIN;  // drive high
		in_pulse = false;
	}
}

static void init_systick(void) {
	SysTick->LOAD = SystemCoreClock/1000 - 1;  // 1 ms
	SysTick->VAL  = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
	| SysTick_CTRL_TICKINT_Msk
	| SysTick_CTRL_ENABLE_Msk;
}

static inline float ticks_to_cm(uint32_t ticks) {
	float us = (float)ticks * (1.0f/42e6f) * 1e6f;
	return us / 57.5f;
}


// TC1 Handler: capture echo, set obstacle_flag
void TC1_Handler(void) {
	static uint32_t ra;
	uint32_t sr = TC0->TC_CHANNEL[TC_ECHO_CH].TC_SR;
	if (sr & TC_SR_LDRAS) {
		ra = TC0->TC_CHANNEL[TC_ECHO_CH].TC_RA;
	}
	if (sr & TC_SR_LDRBS) {
		uint32_t rb = TC0->TC_CHANNEL[TC_ECHO_CH].TC_RB;
		uint32_t dt = (rb >= ra) ? (rb - ra) : 0;
		float cm = ticks_to_cm(dt);
		obstacle_flag = (cm > 0 && cm < THRESHOLD_CM);
	}
}

static void init_ultrasonic(void) {
	// Enable PIOC, PIOA, TC1 clocks
	PMC->PMC_PCER0 |= (1u << ID_PIOC)
	| (1u << ID_PIOA)
	| (1u << TC_ECHO_ID);

	// Trigger pin output, idle high
	TRIG_PIO->PIO_PER = TRIG_PIN;
	TRIG_PIO->PIO_OER = TRIG_PIN;
	TRIG_PIO->PIO_SODR = TRIG_PIN;

	// Onboard LED output off
	LED1_PORT->PIO_PER  = LED1_PIN;
	LED1_PORT->PIO_OER  = LED1_PIN;
	LED1_PORT->PIO_CODR = LED1_PIN;

	// Echo pin as peripheral B
	ECHO_PIO->PIO_PDR  = ECHO_PIN;
	ECHO_PIO->PIO_ABSR = ECHO_PIN;

	// Capture rising & falling, clock MCK/2
	TC0->TC_CHANNEL[TC_ECHO_CH].TC_CMR =
	TC_CMR_TCCLKS_TIMER_CLOCK1
	| TC_CMR_ABETRG
	| TC_CMR_LDRA_RISING
	| TC_CMR_LDRB_FALLING;
	TC0->TC_CHANNEL[TC_ECHO_CH].TC_IER = TC_IER_LDRAS | TC_IER_LDRBS;
	NVIC_EnableIRQ(TC1_IRQn);
	TC0->TC_CHANNEL[TC_ECHO_CH].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}
