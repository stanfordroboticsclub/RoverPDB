// RoverPDB Firmware, written for ATmega328PB.
// Fuses: 8MHz IntRC, 4.1ms start, 2.7V BOD
// avrdude -p m328pb -c usbasp -U lfuse:w:0xd2:m -U hfuse:w:0xd9:m -U efuse:w:0xfd:m 

extern "C" {
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
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
#define PIN_GATE_COMM 	1

#define PORT_GVCC 		C
#define PIN_GVCC_SHDN	5

// pdb controls (active low)
#define PORT_CONTROLS	C
#define PIN_START 		3 //pcint11
#define PIN_ESTOP 		4 //pcint12
volatile bool btn_ready = false;
// PCINT1 covers pins [14:8]

// analog read pins
#define PORT_ADC		C
#define PIN_ADC_VBAT_A  0
#define PIN_ADC_VBAT_B	1 
#define PIN_ADC_VSYS  	2
const uint8_t ADC_SCANLIST[] = {PIN_ADC_VBAT_A, PIN_ADC_VBAT_B, PIN_ADC_VSYS};
uint16_t ADC_RESULTS[sizeof(ADC_SCANLIST)] = {0};
volatile bool adc_result_ready = false;

#define MV_PER_LSB 		53.71
#define A_PER_LSB 		24  // = MVLSB / (~2.2mOhm)
#define V_OCPCATCH 		521 // 28volts
#define V_RECOVER 		298 // 16volts 
#define I_OCPPDB		4   // ~100A
#define MAX_DELTA_VBAT 	6	// do not connect batteries more than 322mV apart  

enum PDBState {READY, LIVE, ESTOP, FAULT};
PDBState state_pdb;



volatile uint32_t wdt_ticks = 0;
ISR(WDT_vect) {
	wdt_ticks++;
	//periodically force poll estop button, in case an interrupt missed
	btn_ready = true;	
}

ISR(ADC_vect) {
	adc_result_ready = true;
}

ISR(PCINT1_vect) {
	btn_ready = true;
}

void init_io() {

	// drive gates off
	SET(CC(DDR, PORT_GATE), PIN_GATE_A);
	SET(CC(DDR, PORT_GATE), PIN_GATE_B);
	SET(CC(DDR, PORT_GATE), PIN_GATE_COMM);
	SET(CC(DDR, PORT_GVCC), PIN_GVCC_SHDN);

	// power up comms system
	SET(CC(PORT, PORT_GATE), PIN_GATE_COMM);

	// disable input buffer on ADC pins
	DIDR0 = _BV(PIN_ADC_VBAT_A) + _BV(PIN_ADC_VBAT_B) + _BV(PIN_ADC_VSYS);

	// setup indicator lights
	SET(CC(DDR, PORT_LED_MCU), PIN_LED_MCU);
	SET(CC(DDR, PORT_LED_OFF), PIN_LED_OFF);

	// install interrupt handlers, pc3/4
	PCMSK1 = _BV(PCINT11) | _BV(PCINT12);
	SET(PCICR, PCIE1);

}


void init_adc() {
	// enable adc, set prescaler to 32x (250k @ 8M clk)
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);

	// fire a single conversion (first conversion)
	SET(ADCSRA, ADSC);
	// block until completion
	_delay_us(200);
	// extended (first) conversion does not clear ADSC
	CLR(ADCSRA, ADSC);

	// enable ADC interrupt
	SET(ADCSRA, ADIE);
}


void lp_delay_128(uint8_t periods) {
	// disable and turn off ADC
	CLR(ADCSRA, ADEN);
	SET(PRR0, PRADC);

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	

	wdt_reset();
	uint32_t wdt_last = wdt_ticks;
	for (uint8_t elapsed = 0; elapsed < periods; elapsed++) {
		while(wdt_ticks <= wdt_last) {
			sleep_mode();
		}
		wdt_last = wdt_ticks;
	}

	// reenable ADC
	CLR(PRR0, PRADC);
	init_adc();
}

// poll buttons over 50ms. this gets called in response to a pin change interrupt,
// and gives us more confidence in avoiding spurious estop fires
void poll_buttons() {
	uint8_t control_pins = ~(CC(PIN, PORT_CONTROLS));

	uint8_t votes_estop = 0;
	uint8_t votes_start = 0;

	for(uint8_t i = 0; i < 50; i++) {
		control_pins = ~(CC(PIN, PORT_CONTROLS));
		votes_estop += (control_pins & _BV(PIN_ESTOP)) ? 1 : 0;
		votes_start += (control_pins & _BV(PIN_START)) ? 1 : 0;
		_delay_ms(1);
	}

	btn_ready = false;

	bool pressed_estop = votes_estop > 40;
	bool pressed_start = votes_start > 40;

	if(pressed_estop) {
		update_state(ESTOP);
		return;
	} 

	if(pressed_start && !pressed_estop) {
		update_state(LIVE);
		return;
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


void read_adc() {
	set_sleep_mode(SLEEP_MODE_ADC);

	for(uint8_t idx_ch = 0; idx_ch < sizeof(ADC_SCANLIST); idx_ch++) {
		// select target channel
		ADMUX = ADC_SCANLIST[idx_ch];

		// fire a conversion
		SET(ADCSRA, ADSC);
		// wait for adc result in lp mode
		uint8_t pending_interrupts = 0;
		while(!adc_result_ready && pending_interrupts < 5) {
			sleep_mode();
			pending_interrupts++;
		}
		adc_result_ready = false;

		// store adc read result
		uint16_t adc_hi = ADCH;
		uint16_t adc_lo = ADCL;
		uint16_t adc = (adc_hi << 8) + adc_lo;
		ADC_RESULTS[idx_ch] = adc;
	}
}

void print_adc() {
	Serial1.print("VBAT A: ");
	Serial1.println(ADC_RESULTS[0]);
	Serial1.print("VBAT B: ");
	Serial1.println(ADC_RESULTS[1]);
	Serial1.print("VSYS: ");
	Serial1.println(ADC_RESULTS[2]);
}


void soft_start() {

	for(uint8_t t = 1; t < 10; t++) {
		SET(CC(PORT, PORT_GATE), PIN_GATE_A);
		SET(CC(PORT, PORT_GATE), PIN_GATE_B);

		delayMicroseconds(t*100);

		CLR(CC(PORT, PORT_GATE), PIN_GATE_A);
		CLR(CC(PORT, PORT_GATE), PIN_GATE_A);
		_delay_us(1000);
	}
	SET(CC(PORT, PORT_GATE), PIN_GATE_A);
	SET(CC(PORT, PORT_GATE), PIN_GATE_B);
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

		// power up gates
		soft_start();
		
	} else {
		SET(CC(PORT, PORT_LED_OFF), PIN_LED_OFF);
	}

	state_pdb = new_state;
}

void reset_power() {
	update_state(ESTOP);
	CLR(CC(PORT, PORT_LED_OFF), PIN_LED_OFF);

	// turn off comm power
	CLR(CC(PORT, PORT_GATE), PIN_GATE_COMM);
	// turn off vcc rail
	SET(CC(PORT, PORT_GVCC), PIN_GVCC_SHDN);
	// wait 256ms in minimum power state
	lp_delay_128(2);

	// turn on vcc rail, comm power
	CLR(CC(PORT, PORT_GVCC), PIN_GVCC_SHDN);
	SET(CC(PORT, PORT_GATE), PIN_GATE_COMM);
	// turn on system power
	update_state(LIVE);
}


int main() {
	// Critical ASAP initialization.
	init_io();
	update_state(ESTOP);

	// Setup remaining peripherals.
	init_adc();
	
	// disable all peripherals except Serial1 and ADC
	PRR0 = ~(
		// _BV(PRUSART1) | 
		_BV(PRADC)
	);
	PRR1 = 0xFF;

	// test initial voltage to see if we were previously live
	// just in case we were killed by a brownout on the 5V rail
	// read_adc();
	// if(ADC_RESULTS[2] > V_RECOVER) {
	// 	update_state(LIVE);
	// }

	// // flash initialization pattern
	// const uint8_t blink_seconds = 2;
	// blink(2);

	// Serial1.begin(115200);
	// Serial1.println("hello");

	//enable WDT in interrupt mode, 128ms period
	WDTCSR = _BV(WDIE) | _BV(WDP0) | _BV(WDP1);
	wdt_reset();
	wdt_ticks = 0;

	sei();

	uint8_t ocp_watch = 0;
	uint8_t ocp_pdb_count = 0;

	while(1) {
		read_adc();

		// Try to detect and recover from tripped batteries (voltage below 28V)
		if(ADC_RESULTS[2] < V_OCPCATCH) {
			ocp_watch += 10;
		} else if(ocp_watch > 0) {
			ocp_watch--;
		}

		if(ocp_watch > 18) {
			reset_power();
		}


		// Limit current per battery to 100A.
		uint16_t isense_a = ADC_RESULTS[2] - ADC_RESULTS[0];
		uint16_t isense_b = ADC_RESULTS[2] - ADC_RESULTS[1];
		if(isense_a > I_OCPPDB || isense_b > I_OCPPDB) {
			ocp_pdb_count += 10;
		} else if(ocp_pdb_count > 0) {
			ocp_pdb_count--;
		}


		if(ocp_pdb_count > 18) {
			reset_power();
		}


		// Process button events
		if(btn_ready) {
			poll_buttons();
		}
	}
}

