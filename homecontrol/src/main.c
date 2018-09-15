/**
 * Project: HomeControl
 * File:    control.c
 * Author:  Stefan Hepp <stefan@stefant.org>
 *
 * Main control code for HomeControl.
 *
 * Pinout:
 * - PortB:
 *   - PB0: DIn0
 *   - PB1: DIn1
 *   - PB2: DIn2
 *   - PB3: DIn3
 *   - PB4: DIn4
 *   - PB5: DIn5
 *   - PB6: DIn6
 *   - PB7: DIn7
 * - PortC:
 *   - PC0: DOut0
 *   - PC1: DOut1
 *   - PC2: Sense In 0
 *   - PC3: Sense In 1
 *   - PC4: SDA
 *   - PC5: SCL
 *   - PC6: Reset In
 * - PortD:
 *   - PD0: RXD
 *   - PD1: TXD
 *   - PD2: DOut2
 *   - PD3: DOut3
 *   - PD4: DOut4
 *   - PD5: DOut5
 *   - PD6: DOut6
 *   - PD7: DOut7
 **/

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


#define F_CPU 1000000
#define BAUD  9600

// baud rate calculation
#include <util/setbaud.h>

int main() {
    
    // *** Disable interrupts ***
    cli();

    // *** Setup ports ***
    // Port B: Input, external pull-down
    DDRB = 0x00;
    PORTB = 0x00;

    // Port C: 0,1: Output; 2-6: Input, pull-up
    DDRC = 0xFC;
    PORTC = 0xFC;
    
    // Port D: 0: Input; 1-7: Output
    DDRD = 0x01;
    PORTD = 0x01;

    // *** Configure UART ***
    // Set baud rate
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif
    // Enable RX and TX
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
    // Set frame format: 8N1
    UCSR0C = (3 << UCSZ00);

    // *** Configure timer ***


    // *** Enable interrupts ***
    sei();

    while(1) {
	
    }

    return 0;
}
