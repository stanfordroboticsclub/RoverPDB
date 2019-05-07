// RoverPDB Firmware, written for ATmega328PB.

extern "C" {
#include <avr/sleep.h>
#include <avr/wdt.h>
}

#include <Arduino.h>

//helper macros
#define SET(x,y) (x |= (1<<y))
#define FLP(x,y) (x ^= (1<<y))
#define CLR(x,y) (x &= (~(1<<y)))
#define CONCAT(a, b)   	a ## b
#define CC(a, b) 		CONCAT(a, b)

// IO defines
#define PORT_LED_OFF 	E
#define PIN_LED_OFF 	2
#define PORT_LED_MCU 	B
#define PIN_LED_MCU 	5

// gate defines
#define PORT_GATE		D
#define PIN_GATE_A 		3
#define PIN_GATE_B		2

// pdb controls (active low)
#define PORT_CONTROLS	C
#define PIN_START 		3
#define PIN_ESTOP 		4

enum PDBState {READY, LIVE, ESTOP, FAULT};
PDBState state_pdb;


volatile uint32_t wdt_ticks = 0;
ISR(WDT_vect) {
	wdt_ticks++;
}


void lp_delay_16(uint16_t periods) {
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	WDTCSR = _BV(WDIE) | _BV(WDP1) | _BV(WDP0);	// WDT INT mode, 128ms period
	uint8_t sleep_stride = 8;

	while(periods > 0) {
		if (periods < sleep_stride)
		{
			WDTCSR = _BV(WDIE);		// WDT INT mode, 16ms period
			sleep_stride = 1;
		}

		wdt_reset();
		wdt_ticks = 0;

		cli();
		sleep_enable();
		sei();
		sleep_cpu();			//go to sleep
		sleep_disable();	//resume execution after ISR

		if(wdt_ticks >= 1) {
			periods -= sleep_stride;
		}
	}
}

void update_state(PDBState new_state) {
	if(new_state == ESTOP) {
		// turn gates off
		CLR(CC(PORT, PORT_GATE), PIN_GATE_A);
		CLR(CC(PORT, PORT_GATE), PIN_GATE_B);

		// light OFF led
		SET(CC(PORT, PORT_LED_OFF), PIN_LED_OFF);
		// CLR(CC(PORT, PORT_LED_MCU), PIN_LED_MCU);
	}

	if(new_state == LIVE) {
		// clear OFF led
		CLR(CC(PORT, PORT_LED_OFF), PIN_LED_OFF);
		// SET(CC(PORT, PORT_LED_MCU), PIN_LED_MCU);

		// turn gates ON
		SET(CC(PORT, PORT_GATE), PIN_GATE_A);
		SET(CC(PORT, PORT_GATE), PIN_GATE_B);
		
	} else {
		SET(CC(PORT, PORT_LED_OFF), PIN_LED_OFF);
	}

	state_pdb = new_state;
}

void poll_buttons() {
	uint8_t control_pins = ~(CC(PIN, PORT_CONTROLS));

	bool pressed_estop = control_pins & _BV(PIN_ESTOP);
	bool pressed_start = control_pins & _BV(PIN_START);

	for(uint8_t i = 0; i < 10; i++) {
		control_pins = ~(CC(PIN, PORT_CONTROLS));
		
		pressed_estop = pressed_estop && (control_pins & _BV(PIN_ESTOP));
		pressed_start = pressed_start && (control_pins & _BV(PIN_START));
		_delay_ms(1);
	}

	if(pressed_estop) {
		update_state(ESTOP);
	} else if(pressed_start) {
		update_state(LIVE);
	}

}


void blink(uint8_t blink_seconds) {
    for(uint8_t iter_blink = 0; iter_blink < (blink_seconds * 5); iter_blink++) {
		SET(CC(PORT, PORT_LED_MCU), PIN_LED_MCU);
		_delay_ms(100);
		CLR(CC(PORT, PORT_LED_MCU), PIN_LED_MCU);
		_delay_ms(100);
    }
}


int main() {
	// drive gates off
	SET(CC(DDR, PORT_GATE), PIN_GATE_A);
	SET(CC(DDR, PORT_GATE), PIN_GATE_B);

	update_state(READY);

	// flash initialization pattern
	SET(CC(DDR, PORT_LED_MCU), PIN_LED_MCU);
	SET(CC(DDR, PORT_LED_OFF), PIN_LED_OFF);

	const uint8_t blink_seconds = 2;
	blink(2);

	// Serial1.begin(115200);
	// Serial1.println("hello");

	//enable WDT interrupt, 16ms period
	WDTCSR = _BV(WDIE);
	wdt_reset();
	wdt_ticks = 0;

	set_sleep_mode(SLEEP_MODE_IDLE);


	while(1) {
		poll_buttons();
	}
}

