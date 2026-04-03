#ifdef __AVR_ATmega328P__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#define F_CPU 16000000UL
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NORMAL_MODE_VALUE(timer_bit, n_seconds, prescaler) ((int)(((1UL) << (timer_bit)) - ((n_seconds) * ((F_CPU) / (prescaler)))))
#define CTC_MODE_VALUE(n_seconds, prescaler) ((int)(((n_seconds) * ((F_CPU) / (prescaler))) - (1UL)))

/* 
https://github.com/arduino/ArduinoCore-avr/blob/87faf934a742fd6aa9fc269c99de5d529363f204/bootloaders/atmega/ATmegaBOOT_168.c#L375C1-L384C7

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#ifdef DOUBLE_SPEED
	UCSR0A = (1<<U2X0); //!!!!!!! Double speed mode USART0
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*8L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*8L)-1) >> 8;
#else
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
#endif
*/

#define UBRR_VALUE_LOW_SPEED(UART_BAUDRATE) ((unsigned char)(((F_CPU)/((UART_BAUDRATE) * (16UL)))-((double)(1UL))))
#define UBRR_VALUE_DOUBLE_SPEED(UART_BAUDRATE) ((unsigned char)(((F_CPU)/((UART_BAUDRATE) * (8L)))-((double)(1UL))))

#include <avr/io.h>

void Timer_1_Delay();		// Prototype for Delay Function

void usart_init()
{
	UCSR0B = (1<<TXEN0) /*enable TX*/ | (1<<RXEN0) /* enable RX */ /*| (1<<UDRIE0)  Register Empty Interrupt */ /*| (1<<RXCIE0)  Complete Interrupt Enable */;
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);  // no parity, 1 stop bit, 8-bit data
	// UBRR0 = UBRR_VALUE_LOW_SPEED(9600);

	UCSR0A = (1<<U2X0); //Double speed mode USART0
	UBRR0 = UBRR_VALUE_DOUBLE_SPEED(115200);

	// UBRR0L = (uint8_t)(F_CPU/(115200*16L)-1);
	// UBRR0H = (F_CPU/(115200*16L)-1) >> 8;
} 
 
#define OCRA0_VALUE(TARGET_FREQ, PRESCALER) (((unsigned char)((unsigned int)((unsigned int)((F_CPU) / (TARGET_FREQ)) / (PRESCALER)))) - 1UL)

#define MY_OCRA0_VALUE ((unsigned char) OCRA0_VALUE(500, 256))

// wave frequency to 500 Hz. The duty cycle should be 50%.
void Timer_0(volatile unsigned char on_ticks_requested) {
	//TCCR0A =
	//(1 << COM0A1) | // 7
	//(1 << COM0A0) | // 6
	//(1 << COM0B1) | // 5
	//(1 << COM0B0) | // 4, nothing in 3 and 2
	//
	//(1 << WGM01) |  // 1
	//(1 << WGM00);   // 0
//
	//TCCR0B =
	//(1 << FOC0A) |  // 7
	//(1 << FOC0B) |  // 6
	//(1 << WGM02) |  // 3
	//(1 << CS02) |   // 2
	//(1 << CS01) |   // 1
	//(1 << CS00);    // 0

	if (on_ticks_requested == 0) {
		TCCR0B = (0 << CS02) | (0 << CS01) | (0 << CS00); // close the timer
		return;
	}
	
	// timer mode 7: 
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | // Clear OC0A on Compare Match when up-counting. Set OC0A on Compare Match when down-counting.
		(1 << COM0B1) | (0 << COM0B0) | // Clear OC0B on Compare Match
		(1 << WGM01) | (1 << WGM00); // Fast PWM
		
	TCCR0B = (1 << WGM02) | // OCRA as TOP
		(1 << CS02) | (0 << CS01) | (0 << CS00); // prescaler 256
		
	
	OCR0A = (MY_OCRA0_VALUE); //ocr0a_value; //64kHz,  ((F_CPU) / (64000)) - 1
	OCR0B = on_ticks_requested; //(unsigned char)((MY_OCRA0_VALUE) * ((unsigned char)(pulse_width / 100))); //20% duty cycle, 249 * 0.2
	DDRD = 0b00100000; // PD5 (OC0B), have to set as output
}

char tx_buffer[80] = { 0 };

size_t rx_buffer_index = 0;
char rx_buffer[80] = { 0 };


void Capture(unsigned int* pulse_width_ticks_measured) {
	PORTB = 0xFF; //pullup enable
	DDRB &= ~(1 << 0); // PB0 (ICP1) as input

	TCCR1A = 0; // Mode = Normal

	TCCR1B = (1 <<ICES1) | 
		(1 << CS12) | (0 << CS11) | (0 << CS10);

	TIFR1 = (1<<ICF1); //clear ICF1 flag

	while ((TIFR1&(1<<ICF1)) == 0);

	*pulse_width_ticks_measured = ICR1; //first edge value

	TIFR1 = (1<<ICF1); //clear ICF1 flag

	TCCR1B = (0 <<ICES1) | 
		(1 << CS12) | (0 << CS11) | (0 << CS10);

	while ((TIFR1&(1<<ICF1)) == 0);

	*pulse_width_ticks_measured = ICR1 - *pulse_width_ticks_measured;

	TIFR1 = (1<<ICF1); //clear ICF1 flag
}

void usart_send_char(const char ch)
{
	while(!(UCSR0A &(1<<UDRE0)));
	UDR0 = ch;
}

void usart_read_char(unsigned char* ptr_ch)
{
	while(!(UCSR0A & (1<<RXC0)));
	*ptr_ch = UDR0;
}

void usart_send_string(const char* str, size_t str_length)
{
	for (size_t i = 0; i < str_length; i++) {
		if (str[i] != '\0') {
			usart_send_char(str[i]);
		} else {
			break;
		}
	}
}

void delay(void) {
	volatile unsigned long i;
	for (i = 0; i < 625000; i++);
}

int main(void)
{
	cli();

	memset(tx_buffer, '\0', sizeof(tx_buffer));
	memset(rx_buffer, '\0', sizeof(rx_buffer));

	unsigned char rx_char = '\0';

	usart_init();

	unsigned int pulse_width_requested = 0;
	unsigned int pulse_width_ticks_measured = 0;
	unsigned int previous_pulse_width_requested = 0;

    while (1)
	{
		snprintf((char*)tx_buffer, sizeof(tx_buffer), "[atmega328p]\n%d ticks per period, %luHz per period, Prescaler is %d\n", MY_OCRA0_VALUE, (F_CPU / (256 * (MY_OCRA0_VALUE + 1))), 256);
		usart_send_string((char*)tx_buffer, sizeof(tx_buffer));

		memset(tx_buffer,'\0', sizeof(tx_buffer));
		snprintf((char*)tx_buffer, sizeof(tx_buffer), "New pulse width (%%) ('n' to read external PWM):\n");
		usart_send_string((char*)tx_buffer, sizeof(tx_buffer));

		rx_buffer_index = 0;
		do {
			usart_read_char(&rx_char);
			rx_buffer[rx_buffer_index++] = rx_char;
			usart_send_char(rx_char);
			if (rx_char == '\n') {
				break;
			}
		} while (1);

		memset(tx_buffer,'\0', sizeof(tx_buffer));
		if (rx_buffer[0] != 'n')
		{
			snprintf((char*)tx_buffer, sizeof(tx_buffer), "Setting PWM...\n");
			usart_send_string((char*)tx_buffer, sizeof(tx_buffer));

			sscanf((char*)rx_buffer, "%u\n" , &pulse_width_requested);
			// HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		} else {
			pulse_width_ticks_measured = 0;
			pulse_width_requested = 0;
			Timer_0((volatile unsigned char)0); // stop the timer
		}

		if (pulse_width_requested == 0 || previous_pulse_width_requested != pulse_width_requested) {
			memset(tx_buffer,'\0', sizeof(tx_buffer));

			if (pulse_width_requested > 0) {
				unsigned char on_ticks_requested = (pulse_width_requested * MY_OCRA0_VALUE) / 100;
				snprintf((char*)tx_buffer, sizeof(tx_buffer), "Pulse width requested %u%%, equal to %u HIGH ticks\n", pulse_width_requested, on_ticks_requested);
				Timer_0((volatile unsigned char)on_ticks_requested);
			} else {
				snprintf((char*)tx_buffer, sizeof(tx_buffer), "Reading external PWM source\n");
			}

			usart_send_string((char*)tx_buffer, sizeof(tx_buffer));

			delay();

			previous_pulse_width_requested = pulse_width_requested;
		}
		
		pulse_width_ticks_measured = 0;
		Capture(&pulse_width_ticks_measured);

		memset(tx_buffer, '\0', sizeof(tx_buffer));
		snprintf((char*)tx_buffer, sizeof(tx_buffer), "Pulse width measured %u%%, equal to %u HIGH ticks\n=======\n", (pulse_width_ticks_measured * 100) / MY_OCRA0_VALUE, pulse_width_ticks_measured);
		usart_send_string((char*)tx_buffer, sizeof(tx_buffer));
	}
}

#else
#include "gtest/gtest.h"
using ::testing::InitGoogleTest;

// Demonstrate some basic assertions.
TEST(MyTest, BasicAssertions) {
	EXPECT_EQ(UBRR_VALUE_LOW_SPEED(9600), 103);
	EXPECT_EQ(UBRR_VALUE_LOW_SPEED(4800), 207);
	EXPECT_EQ(UBRR_VALUE_DOUBLE_SPEED(115200), 16);
}


int main(int argc, char** argv) {
  	InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif
