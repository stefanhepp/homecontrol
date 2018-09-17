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

// Define CPU clock speed and UART baud rate
#define F_CPU 1000000
#define BAUD  9600

// Baud rate calculation
#include <util/setbaud.h>

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
// - Sense notification. Value = 1: Motion detected
#define UART_CMD_SENSE         0x06

#define P_OFF   0
#define P_SHORT 1
#define P_LONG  2

typedef struct {
    unsigned char  state;
    unsigned char  pressed;
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
    // Only sleep it there is no input change since last read
    if (Last_input == PINB) {
	sleep_enable();
	//sleep_bod_disable();
	sei();
	sleep_cpu();
	sleep_disable();
    }
}

void init_switch(switch_t *sw)
{
    sw->state = 0;
    sw->pressed = P_OFF;
    sw->time_longpress = 0;
}

void update_switch(switch_t *sw, unsigned char value)
{
    
}

void buf_push(uart_buf_t *buf, unsigned char value)
{
    if (buf->len < UART_BUF_SZ) {
	buf->buf[ (buf->start + buf->len) % UART_BUF_SZ ] = value;
	buf->len++;
    }
}

unsigned char buf_pop(uart_buf_t *buf)
{
    unsigned char ret = 0;

    if (buf->len > 0) {
	ret = buf->buf[buf->start];
	buf->start++;
	buf->len--;
    }
    return ret;
}

ISR(USART_RX_vect)
{
    buf_push( &UART_RX_Buf, UDR0 );
}

ISR(USART_UDRE_vect)
{
    if (UART_TX_Buf.len > 0) {
	UDR0 = buf_pop( &UART_TX_Buf );
    } else {
	// Stop transmitting
	UCSR0B &= ~_BV( UDRIE0 );
    }
}

void send_command(unsigned char cmd, unsigned char value)
{
    buf_push( &UART_TX_Buf, UART_CMD_HEADER | cmd );
    buf_push( &UART_TX_Buf, value );

    // Start transmitting
    UCSR0B |= _BV( UDRIE0 );
}

char read_command(unsigned char *cmd, unsigned char *value)
{
    // We need at least 2 characters in the buffer for a valid message
    while (UART_RX_Buf.len > 1) {
	*cmd = buf_pop(&UART_RX_Buf);
	if ((*cmd & UART_CMD_HEADER) == UART_CMD_HEADER) {
	    if (UART_RX_Buf.len > 0) {
		*value = buf_pop(&UART_RX_Buf);
	    } else {
		// Not yet enough data
		buf_push(&UART_RX_Buf, *cmd);
		return 0;
	    }
	}
    }
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

    update_switch(&SW_Livingroom,      Last_input & (1 << PB0));
    update_switch(&SW_Staircase,       Last_input & (1 << PB1));
    update_switch(&SW_Stair_Light,     Last_input & (1 << PB2));
    update_switch(&SW_Stair_Staircase, Last_input & (1 << PB3));

    update_switch(&SW_Stair_Sense,     bit_is_set(PINC, PC2));
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
	// Toggle sensor on/off
	if (DO_Stair_Light) {
	    DO_Stair_Light = 0;
	    DO_Stair_Sense = 1;
	} else {
	    DO_Stair_Sense ^= 1;
	}
	send_command(UART_CMD_STAIR, (DO_Stair_Sense << 1) | DO_Stair_Light);
    }
    if (SW_Stair_Light.pressed == P_LONG) {
	// Toggle light on/off
	DO_Stair_Light ^= 1;
        DO_Stair_Sense = 0;
	send_command(UART_CMD_STAIR, (DO_Stair_Sense << 1) | DO_Stair_Light);
    }

    // Outside stair staircase switch
    if (SW_Stair_Staircase.pressed == P_SHORT) {
	DO_Staircase ^= 1;
	send_command(UART_CMD_STAIRCASE, DO_Staircase);
    }
    if (SW_Stair_Staircase.pressed == P_LONG) {
	send_command(UART_CMD_STAIRCASE, 2);
    }

    // Motion sense input
    if (SW_Stair_Sense.pressed != P_OFF) {
	send_command(UART_CMD_SENSE, 1);
    }

    // All new inputs processed, clear status
    SW_Livingroom.pressed = P_OFF;
    SW_Staircase.pressed = P_OFF;
    SW_Stair_Light.pressed = P_OFF;
    SW_Stair_Staircase.pressed = P_OFF;
    SW_Stair_Sense.pressed = P_OFF;
}

void write_outputs(void)
{
    SET_BIT(PORTC, PC0, DO_Livingroom);
    SET_BIT(PORTC, PC1, DO_Staircase);
    SET_BIT(PORTD, PD2, DO_Stair_Light);
    SET_BIT(PORTD, PD3, DO_Stair_Sense);
}

int main()
{    
    // *** 
    // Disable interrupts
    // ***
    cli();

    // ***
    // Setup ports 
    // ***
    // Port B: Input, external pull-down
    DDRB = 0x00;
    PORTB = 0x00;

    // Port C: 0,1: Output; 2-6: Input, pull-up
    DDRC = 0xFC;
    PORTC = 0xFC;
    
    // Port D: 0: Input; 1-7: Output
    DDRD = 0x01;
    PORTD = 0x01;

    // ***
    // Configure UART
    // ***
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

    // ***
    // Configure timer
    // ***

    init_switch(&SW_Livingroom);
    init_switch(&SW_Staircase);
    init_switch(&SW_Stair_Light);
    init_switch(&SW_Stair_Staircase);


    // ***
    // Enable interrupts
    // ***
    sei();

    while(1) {
	// Wait for any inputs
	sleep();

	read_inputs();

	process_inputs();

	process_uart();

	write_outputs();
    }

    return 0;
}
