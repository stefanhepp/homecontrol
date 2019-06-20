/**
 * Project:   HomeControl
 * File:      main.c
 * Author:    Stefan Hepp <stefan@stefant.org>
 *
 * Copyright 2018 Stefan Hepp
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Main control code for HomeControl.
 *
 * Pinout:
 * - PortB:
 *   - PB0: DIn0 - Livingroom
 *   - PB1: DIn1 - Staircase
 *   - PB2: DIn2 - Stair-Light
 *   - PB3: DIn3 - Stair-Staircase
 *   - PB4: DIn4
 *   - PB5: DIn5
 *   - PB6: DIn6
 *   - PB7: DIn7
 * - PortC:
 *   - PC0: DOut0 - Livingroom
 *   - PC1: DOut1 - Staircase
 *   - PC2: Sense In 0 - Stair Sense
 *   - PC3: Sense In 1
 *   - PC4: SDA
 *   - PC5: SCL
 *   - PC6: Reset In
 * - PortD:
 *   - PD0: RXD
 *   - PD1: TXD
 *   - PD2: DOut2 - Stair-Light
 *   - PD3: DOut3 - Stair-Sense
 *   - PD4: DOut4
 *   - PD5: DOut5
 *   - PD6: DOut6
 *   - PD7: DOut7
 **/

// Define CPU clock speed and UART baud rate
#define F_CPU 1000000
#define BAUD  9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// Baud rate calculation, delays
#include <util/setbaud.h>
#include <util/delay.h>

#define SET_BIT(port,pin,value) \
    if (value) { \
	port |= _BV(pin);  \
    } else { \
	port &= ~_BV(pin); \
    }

/**
 * UART command codes
 **/
// Command header
#define UART_CMD_HEADER        0x70

// Command Opcodes:
// - Living room light. Value = 0: Off, 1: Light on, 2: Long press
#define UART_CMD_LIVINGROOM    0x01
// - Stair case light. Value = 0: Off, 1: Light on, 2: Stair long press
#define UART_CMD_STAIRCASE     0x02
// - Stair light. Value = 000000 | Stair Sense | Stair Light
#define UART_CMD_STAIR         0x03
// - All off. Value = 1: All off
#define UART_CMD_ALL_OFF       0x04
// - Status. Value = 000 | Sense | Stair Sense | Stair Light | Staircase | Livingroom
#define UART_CMD_STATUS        0x05
// - Sense notification. Value = 0: Motion Timeout, 1: Motion detected
#define UART_CMD_SENSE         0x06

#define P_OFF     0
// Short press event
#define P_SHORT   1
// Long press event
#define P_LONG    2
// pressed but not yet released
#define P_PRESSED 3

typedef struct {
    // current button pin state
    unsigned char  state;
    // button event state
    unsigned char  pressed;
    // countdown for long button press in 100ms steps. If non-zero, countdown is running
    unsigned short time_longpress;
} switch_t;

// Input switches
switch_t SW_Livingroom;
switch_t SW_Staircase;
switch_t SW_Stair_Light;
switch_t SW_Stair_Staircase;

switch_t SW_Stair_Sense;

// Output states
unsigned char DO_Livingroom = 0;
unsigned char DO_Staircase  = 0;
unsigned char DO_Stair_Light = 0;
unsigned char DO_Stair_Sense = 0;

unsigned char Last_input = 0;

#define UART_BUF_SZ  8

typedef struct {
    unsigned char start;
    unsigned char len;
    unsigned char buf[UART_BUF_SZ];
} uart_buf_t;

uart_buf_t UART_TX_Buf;
uart_buf_t UART_RX_Buf;

void sleep(void)
{
    cli();

    // Only sleep it there is no input change since last read
    if (Last_input == PINB) {
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	sleep_disable();
    } else {
	sei();
    }
}

/**
 * Start the long-press timer for that switch
 */
void start_timer(switch_t *sw)
{
    // Start timer with 1.8s
    sw->time_longpress = 18;
    // Enable timer
    TCCR1B = 0x1D;
}

void init_timer(void)
{
    // Fast PWM mode, OCR1A top (mode 15), prescaler clk/1024
    TCCR1A = 0x03;
    TCCR1B = 0x18;

    // Set timer to 100ms
    OCR1A = ((F_CPU/1024)/10);

    // Enable interrupts
    TIMSK1 = _BV(OCIE1A);
}

/**
 * Update switch status on 100ms timer events.
 *
 * \return 1 if switch requires another timer event, else 0.
 */
char update_timer(switch_t *sw)
{
    if (sw->pressed != P_PRESSED) {
	// Disable timer if switch is released (set to SHORT in the meantime)
	sw->time_longpress = 0;
	return 0;
    }
    if (sw->time_longpress) {
	sw->time_longpress--;
	if (sw->time_longpress > 0) {
	    return 1;
	} else {
	    // Set to LONG press event when countdown finishes, even if button is still pressed
	    sw->pressed = P_LONG;
	    return 0;
	}
    }
    return 0;
}

ISR(TIMER1_COMPA_vect)
{
    unsigned char cnt = 0;

    cnt += update_timer(&SW_Staircase);
    cnt += update_timer(&SW_Stair_Light);
    cnt += update_timer(&SW_Stair_Staircase);

    if (cnt == 0) {
	// Stop timer
	TCCR1B = 0x18;
    }
}

ISR(PCINT0_vect)
{
    // Register interrupt handler to wakeup CPU
}

void init_switch(switch_t *sw)
{
    sw->state = 0;
    sw->pressed = P_OFF;
    sw->time_longpress = 0;
}

/**
 * Update the status of a toggle switch from an pin input.
 */
void update_switch(switch_t *sw, unsigned char value)
{
    if (sw->state != value) {
	sw->pressed = P_SHORT;
    }
    sw->state = value;
}

/**
 * Update the status of a push button from an pin input.
 */
void update_button(switch_t *sw, unsigned char value)
{
    if (!sw->state && value) {
	start_timer(sw);
	sw->pressed = P_PRESSED;
    }
    if (sw->state && !value) {
	// If the timer runs out, this gets set to LONG
	if (sw->pressed == P_PRESSED) {
	    sw->pressed = P_SHORT;
	}
    }
    sw->state = value;
}

/**
 * Add a value to a FIFO buffer.
 */
void buf_push(uart_buf_t *buf, unsigned char value)
{
    if (buf->len < UART_BUF_SZ) {
	buf->buf[ (buf->start + buf->len) % UART_BUF_SZ ] = value;
	buf->len++;
    }
}

/**
 * Remove a value from a FIFO buffer.
 * \return the buffer, or 0 if the buffer is empty
 */
unsigned char buf_pop(uart_buf_t *buf)
{
    unsigned char ret = 0;

    if (buf->len > 0) {
	ret = buf->buf[buf->start];
	buf->start = (buf->start + 1) % UART_BUF_SZ;
	buf->len--;
    }
    return ret;
}

/**
 * Handle receiving a character from UART.
 */
ISR(USART_RX_vect)
{
    buf_push( &UART_RX_Buf, UDR0 );
}

/**
 * Send the next character when the send buffer is empty.
 */
ISR(USART_UDRE_vect)
{
    if (UART_TX_Buf.len > 0) {
	// Send the next character from the buffer
	UDR0 = buf_pop( &UART_TX_Buf );
    } else {
	// Stop transmitting
	UCSR0B &= ~_BV( UDRIE0 );
    }
}

/**
 * Init UART.
 **/
void init_uart(void)
{
    // Set baud rate
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif
    // Enable RX and TX, RX interrupts
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
    // Set frame format: 8N1
    UCSR0C = (3 << UCSZ00);
}

/**
 * Send a command via UART
 */
void send_command(unsigned char cmd, unsigned char value)
{
    cli();
    buf_push( &UART_TX_Buf, UART_CMD_HEADER | cmd );
    buf_push( &UART_TX_Buf, value );
    sei();

    // Start transmitting (enable UDRE interrupt on empty buffer)
    UCSR0B |= _BV( UDRIE0 );
}

/**
 * Read a command received from UART.
 * \return 1 if a command is received, else 0.
 */
char read_command(unsigned char *cmd, unsigned char *value)
{
    // We need at least 2 characters in the buffer for a valid message
    cli();
    while (UART_RX_Buf.len > 1) {
	*cmd = buf_pop(&UART_RX_Buf);
	if ((*cmd & UART_CMD_HEADER) == UART_CMD_HEADER) {
	    if (UART_RX_Buf.len > 0) {
		*cmd &= ~(UART_CMD_HEADER);
		*value = buf_pop(&UART_RX_Buf);
		sei();
		return 1;
	    } else {
		// Not yet enough data
		buf_push(&UART_RX_Buf, *cmd);
		sei();
		return 0;
	    }
	}
    }
    sei();
    return 0;
}

void send_status(void)
{
    unsigned char status = 0;

    status |= SW_Stair_Sense.state << 4;
    status |= DO_Stair_Sense << 3;
    status |= DO_Stair_Light << 2;
    status |= DO_Staircase   << 1;
    status |= DO_Livingroom;

    send_command(UART_CMD_STATUS, status);
}

/**
 * Turn all lights off
 */
void cmd_all_off(void)
{
    DO_Livingroom = 0;
    DO_Staircase = 0;
    if (DO_Stair_Light) {
	DO_Stair_Sense = 1;
    }
    DO_Stair_Light = 0;
}

/**
 * Read all input pins and update switch status.
 */
void read_inputs(void)
{
    Last_input = PINB;

    update_switch(&SW_Livingroom,      Last_input & (1 << PB0) ? 1 : 0);
    update_button(&SW_Staircase,       Last_input & (1 << PB1) ? 1 : 0);
    update_button(&SW_Stair_Light,     Last_input & (1 << PB2) ? 1 : 0);
    update_button(&SW_Stair_Staircase, Last_input & (1 << PB3) ? 1 : 0);

    update_switch(&SW_Stair_Sense,     bit_is_set(PINC, PC2) ? 1 : 0);
}

/**
 * Handle all UART commands.
 */
void process_uart(void)
{
    unsigned char cmd;
    unsigned char value;

    while (read_command(&cmd, &value)) {
	switch (cmd) {
	    case UART_CMD_LIVINGROOM:
		DO_Livingroom = value & 0x01;
		break;
	    case UART_CMD_STAIRCASE:
		DO_Staircase = value & 0x01;
		break;
	    case UART_CMD_STAIR:
		DO_Stair_Sense = (value >> 1) & 0x01;
		DO_Stair_Light =  value       & 0x01;
		break;
	    case UART_CMD_ALL_OFF:
		cmd_all_off();
		break;
	    case UART_CMD_STATUS:
		send_status();
		break;
	    case UART_CMD_SENSE:
		send_command(UART_CMD_SENSE, SW_Stair_Sense.state);
		break;
	}
    }
}

/**
 * Update outputs based on input switches.
 */
void process_inputs(void)
{
    // Living room switch
    if (SW_Livingroom.pressed == P_SHORT) {
	// Toggle living room
	DO_Livingroom ^= 1;
	send_command(UART_CMD_LIVINGROOM, DO_Livingroom);
    }
    if (SW_Livingroom.pressed == P_LONG) {
	// Send UART command
	send_command(UART_CMD_LIVINGROOM, 2);
    }

    // Staircase switch
    if (SW_Staircase.pressed == P_SHORT) {
	// Toggle staircase
	DO_Staircase ^= 1;
	send_command(UART_CMD_STAIRCASE, DO_Staircase);
    }
    if (SW_Staircase.pressed == P_LONG) {
	// All off
	cmd_all_off();
	send_command(UART_CMD_ALL_OFF, 1);
	send_status();
    }

    // Stair light switch
    if (SW_Stair_Light.pressed == P_SHORT) {
	// Toggle On > Sensor <> Off
	if (DO_Stair_Light) {
	    DO_Stair_Light = 0;
	    DO_Stair_Sense = 1;
	} else {
	    DO_Stair_Sense ^= 1;
	}
	send_command(UART_CMD_STAIR, (DO_Stair_Sense << 1) | DO_Stair_Light);
    }
    if (SW_Stair_Light.pressed == P_LONG) {
	// Toggle light on/off, sensor off
	DO_Stair_Light ^= 1;
        DO_Stair_Sense = 0;
	send_command(UART_CMD_STAIR, (DO_Stair_Sense << 1) | DO_Stair_Light);
    }

    // Outside stair staircase switch
    if (SW_Stair_Staircase.pressed == P_SHORT) {
	// Toggle staircase
	DO_Staircase ^= 1;
	send_command(UART_CMD_STAIRCASE, DO_Staircase);
    }
    if (SW_Stair_Staircase.pressed == P_LONG) {
	// Send UART command
	send_command(UART_CMD_STAIRCASE, 2);
    }

    // Motion sense input
    if (SW_Stair_Sense.pressed != P_OFF) {
	send_command(UART_CMD_SENSE, SW_Stair_Sense.state);
    }

    // All new inputs processed, clear status
    if (SW_Livingroom.pressed != P_PRESSED) {
	SW_Livingroom.pressed = P_OFF;
    }
    if (SW_Staircase.pressed != P_PRESSED) {
	SW_Staircase.pressed = P_OFF;
    }
    if (SW_Stair_Light.pressed != P_PRESSED) {
	SW_Stair_Light.pressed = P_OFF;
    }
    if (SW_Stair_Staircase.pressed != P_PRESSED) {
        SW_Stair_Staircase.pressed = P_OFF;
    }
    SW_Stair_Sense.pressed = P_OFF;
}

void write_outputs(void)
{
    SET_BIT(PORTC, PC0, DO_Livingroom);
    SET_BIT(PORTC, PC1, DO_Staircase);
    SET_BIT(PORTD, PD2, DO_Stair_Light);
    SET_BIT(PORTD, PD3, DO_Stair_Sense);
}

/**
 * Init IO ports.
 */
void init_io(void)
{
    // Port B: Input, external pull-down
    DDRB = 0x00;
    PORTB = 0x00;

    // Port C: 0,1: Output; 2-3: Input, pull-down; 4-6: Input pull-up
    DDRC = 0x03;
    PORTC = 0xF0;
    
    // Port D: 0: Input, pull-up; 1-7: Output
    DDRD = 0xFE;
    PORTD = 0x01;

    // Enable external interrupts on inputs
    PCMSK0 = 0xFF;
    PCICR = _BV(PCIE0);
}

int main()
{    
    // Disable interrupts
    cli();

    // Setup ports 
    init_io();

    // Configure UART
    init_uart();

    // Configure timer
    init_timer();

    init_switch(&SW_Livingroom);
    init_switch(&SW_Staircase);
    init_switch(&SW_Stair_Light);
    init_switch(&SW_Stair_Staircase);

    // Enable interrupts
    sei();

    while(1) {
	// Wait for any inputs
	//sleep();

	read_inputs();

	process_inputs();

	process_uart();

	write_outputs();

	// Debounce inputs
	_delay_ms(25);
    }

    return 0;
}
