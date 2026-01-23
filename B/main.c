#ifdef __AVR_ATmega328P__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#define F_CPU 16000000UL

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
 
// wave frequency to 500 Hz. The duty cycle should be 50%.
void Timer_0() {
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
	
	// timer mode 7: 
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | //00
		(1 << COM0B1) | (0 << COM0B0) |
		(1 << WGM01) | (1 << WGM00);
		
	TCCR0B = (1 << WGM02) |
		(0 << CS02) | (0 << CS01) | (1 << CS00);
		
	OCR0A = 249; //64kHz,  ((F_CPU) / (64000)) - 1
	OCR0B = 49; //20% duty cycle, 249 * 0.2
	DDRD = 0b00100000; // PD5 (OC0B), have to set as output
}

int main(void)
{
	DDRD = 0xFF;			// PB0 as Output
	
    while (1)				// INF Loop
    {
		//Timer_1_Delay();	// Call 1 s Delay
		// PORTB ^= (1<<0);	// Toggle
		// PORTB = 0xFF;
		Timer_0();
    }
}

void Timer_1_Delay()
{
// Timer 1 16 bit timer with prescaler = 1024
// 1 * 16M / 1024 = 15625
	OCR1AH = 0x3D;		// CTC OCR
	OCR1AL = 0x08;		// 15625 - 1 = 0x3D08
	TCCR1A = 0x00;		// CTC Mode
	TCCR1B = 0x0D;		// CTC Mode, Prescaler = 1024
	while ((TIFR1 &(1<<OCF1A))==0);	// Continue Until Overflow
	TCCR1B = 0x00;		// Stop Timer 1
	TIFR1 = (1<<OCF1A);	// Reset OCR1A
}

#else
#include "gtest/gtest.h"
using ::testing::InitGoogleTest;

// Demonstrate some basic assertions.
TEST(MyTest, BasicAssertions) {
//   // Expect two strings not to be equal.
//   EXPECT_STRNE("hello", "world");
//   // Expect equality.
//   EXPECT_EQ(7 * 6, 42);
	EXPECT_EQ(UBRR_VALUE(9600), 103);
	EXPECT_EQ(UBRR_VALUE(4800), 207);
}


int main(int argc, char** argv) {
  	InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif

