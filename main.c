//Martin Bildhjerd

#include "Config.h"
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h> 
#include <avr/interrupt.h>
#include <util/delay.h>

void LCD_INIT();
void LCD_WRITE_COMMAND(unsigned char command);
void LCD_PRINT(unsigned char data);
void LCD_WRITE_STRING(char *str);
void LCD_GO_TO_XY(unsigned char x, unsigned char y); 
void USART_INIT(void);
void PWM_INIT(void);
void PWM_update(uint16_t i);
void PWM_SWEEP(void);
void SERVO_SET(uint16_t degrees, uint16_t max_degrees);

volatile static uint8_t PWM_update_rdy = 0;
unsigned int READ_LIDAR_DISTANCE(void);

int main(void)
{
	sei();
	DDRB = 0XFF;
	DDRD &= ~(1 << LIDAR);
	
	char tmp[16]; //MESSAGE CARRIER
	unsigned int lidar_distance;
	
	LCD_INIT(); 
	USART_INIT(); 
	PWM_INIT(); 
	
	SERVO_SET(0, 180); 
	int16_t i = 0; 
	
	for(i = 0; i <= 180; i++)
	{
		SERVO_SET(i, 180); 
		_delay_ms(40); //ROTATION
		lidar_distance = READ_LIDAR_DISTANCE();
		LCD_GO_TO_XY(0, 0);
		
			if (lidar_distance <= DISTANCE_THRESHOLD)
			{
				LCD_WRITE_STRING("Object detected!");
			}
			else
			{
				LCD_WRITE_STRING("                "); // Clear the line 0,0
				_delay_ms(LCD_DELAY_TWO);
				LCD_GO_TO_XY(0, 1);
				LCD_WRITE_STRING("                "); // Clear the line 0,1
				_delay_ms(LCD_DELAY_TWO);
			}

		LCD_GO_TO_XY(0, 1);
		sprintf(tmp, "Distance: %u cm", lidar_distance);
		LCD_WRITE_STRING(tmp);
		
		_delay_ms(LCD_DELAY_ONE);
	}
	
	for(i = 180; i >= 0; i--)
	{
		SERVO_SET(i, 180); 
		_delay_ms(40); //ROTATION
		lidar_distance = READ_LIDAR_DISTANCE();
		LCD_GO_TO_XY(0, 0);
		
			if (lidar_distance <= DISTANCE_THRESHOLD)
			{
				LCD_WRITE_STRING("Object detected!");
			}
			else
			{
				LCD_WRITE_STRING("                "); // Clear the line
			}

		LCD_GO_TO_XY(0, 1);
		sprintf(tmp, "Distance: %u cm", lidar_distance);
		LCD_WRITE_STRING(tmp);
		
		_delay_ms(LCD_DELAY_ONE);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// PWM CONF /////////////////////////////////////////////////////
ISR(TIMER1_OVF_vect)
{
	PWM_update_rdy = 0;
}

ISR(TIMER1_COMPA_vect)
{
	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// PWM INIT //////////////////////////////////////////////////////
void PWM_INIT(void)
{
	DDRB |= (1 << DDB3);
	
	TIMSK1 = (1 << TOIE1) | (1 << OCIE1A);
	
	ICR1H =(PWM_TOP && 0XFF00) >> 8;
	ICR1L = (PWM_TOP && 0X00FF);
	
	OCR1AH = (SERVO_MIN & 0XFF00) >> 8;
	OCR1AL = (SERVO_MIN & 0X00FF);
	
	TCCR1A = (0b10 << COM1A0) | (0b00 << COM1B0) | (0b10 << WGM10);
	TCCR1B = (0b11 << WGM12) | (0b010 << CS10);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// PWM UPD ////////////////////////////////////////////////////////
void PWM_update(uint16_t i)
{
	PWM_update_rdy = 1;
	while (PWM_update_rdy != 0);
	
	OCR1AH = (i & 0XFF00) >> 8;
	OCR1AL = (i & 0X00FF);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// PWM SWEEP///////////////////////////////////////////////////////
void PWM_SWEEP(void)
{
	uint16_t i = 0;
	
	for(i = SERVO_MIN; i <= SERVO_MAX; i += 50)
	{
		PWM_update(i);
		_delay_ms(40);
	}
	
	for(i = SERVO_MAX; i >= SERVO_MIN; i -= 50)
	{
		PWM_update(i);
		_delay_ms(40);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// SET SERVO //////////////////////////////////////////////////////
void SERVO_SET(uint16_t degrees, uint16_t max_degrees)
{
	float set = (float)degrees / (float)max_degrees;
	
	set = (((float)SERVO_MAX-(float)SERVO_MIN)*set) + (float)SERVO_MIN;
	
	uint16_t pointer  = (uint16_t)set;
	
	PWM_update(pointer);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////UART INIT//////////////////////////////////////////////////////
void USART_INIT(void) {
	// Calculate the UBRR value based on the desired baud rate
	uint16_t ubrr_value = F_CPU / (16 * BAUD_RATE) - 1;

	// Set the UBRR registers
	UBRR0H = (uint8_t)(ubrr_value >> 8);
	UBRR0L = (uint8_t)ubrr_value;

	// Enable receiver and transmitter
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	// Set frame format: 8 data bits, 1 stop bit, no parity
	UCSR0C = (1 << UCSZ01) | (3 << UCSZ00);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////// LIDAR ////////////////////////////////////////////////////
unsigned int READ_LIDAR_DISTANCE(void)
{
	unsigned char data[9];
	unsigned int distance;

	while (1)
	{
		// Wait for the start of the frame
		while (UDR0 != 0x59)
		;

		// Read 9 bytes of data
		for (int i = 0; i < 9; ++i)
		{
			while (!(UCSR0A & (1 << RXC0)))
			;
			data[i] = UDR0;
		}

		// Check if the data is valid
		if (data[0] == 0x59 && data[1] == 0x59)
		{
			// Calculate distance (in centimeters)
			distance = (data[2] | (data[3] << 8));
			return distance;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// LCD WRITE STRING //////////////////////////////////////////////////
void LCD_WRITE_STRING(char *str)
{
	int i;
	for(i = 0; str[i] != 0; i++)
	{
		LCD_PRINT(str[i]);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// LCD INIT //////////////////////////////////////////////////////
void LCD_INIT()
{
	_delay_ms(40); 
	LCD_WRITE_COMMAND(0X02); //HOME
	LCD_WRITE_COMMAND(0X28); //FUNCTION SET
	LCD_WRITE_COMMAND(0X0C); //DISPLAY ON/OFF
	LCD_WRITE_COMMAND(0x06); //ENTRY MODE
	LCD_WRITE_COMMAND(0X01); //CLEAR DISPLAY
	_delay_ms(2); 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// LCD WRITE COMMMAND /////////////////////////////////////////////////
void LCD_WRITE_COMMAND(unsigned char command)
{
	//UPPER NIBBLE:
	PORTB = (PORTB & 0X0F) | (command & 0XF0);
	PORTB &= ~(1<<LCD_RS); //RS -> 0
	PORTB |= (1<<LCD_EN); //EN -> 1
	_delay_ms(1);
	PORTB &= ~(1<<LCD_EN); //EN -> 0
	_delay_ms(1);
	
	//LOWER NIBBLE:
	PORTB = (PORTB & 0X0F) | (command << 4);
	PORTB |= (1<<LCD_EN); //EN -> 1
	_delay_ms(1);
	PORTB &= ~(1<<LCD_EN); //EN -> 0
	_delay_ms(1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// LCD PRINT //////////////////////////////////////////////////////////////
void LCD_PRINT(unsigned char data)
{
	//UPPER NIBBLE:
	PORTB = (PORTB & 0X0F) | (data & 0XF0);
	PORTB |= ~(1<<LCD_RS); //RS -> 1
	PORTB |= (1<<LCD_EN); //EN -> 1
	_delay_ms(1);
	PORTB &= ~(1<<LCD_EN); //EN -> 0
	_delay_ms(1);
	
	//LOWER NIBBLE:
	PORTB = (PORTB & 0X0F) | (data << 4);
	PORTB |= (1<<LCD_EN); //EN -> 1
	_delay_ms(1);
	PORTB &= ~(1<<LCD_EN); //EN -> 0
	_delay_ms(1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// GO TO POS ////////////////////////////////////////////////////////////
void LCD_GO_TO_XY(unsigned char x, unsigned char y)
{
	if(y == 0) //Line 0
	{
		LCD_WRITE_COMMAND(0x00 + x); //Pos 0, line 0
	}
	else if(y == 1) //Line 1
	{
		LCD_WRITE_COMMAND(0x40 + x); //Pos 0, line 1
	}
	else if(y == 2) //Line 2
	{
		LCD_WRITE_COMMAND(0x10 + x); //Pos 0, line 2
	}
	else //Line 3
	{
		LCD_WRITE_COMMAND(0x50 + x); //Pos 0, line 3
	}
	
	_delay_us(10); 
	
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
