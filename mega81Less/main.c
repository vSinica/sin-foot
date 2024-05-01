
#define F_CPU 8000000UL
#define BAUD 9600
#define UBRR ((F_CPU / (BAUD * 16UL)) - 1 )

#define R0C0lampPortB (1<<6)
#define R0C1lampPortB (1<<7)
#define R0C2lampPortC (1<<0)

#define R0C0SensorPortB (1<<0)
#define R0C1SensorPortB (1<<1)
#define R0C2SensorPortB (1<<2)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
//#include "usart0.h"


volatile char txBuff[4];
volatile char rxBuff[4];
volatile int tx_index = 0;
volatile int rx_index = 0;
volatile int process_data = 0;

char let = ' ';
volatile uint8_t USART_ReceiveBuffer;

ISR(USART_RX_vect){
	cli();
	
	if (rx_index < 4){
	  rxBuff[rx_index] = UDR0;
	  rx_index++;}
	else{
		//UCSR0B &= ~(1 << RXCIE0);
		//UCSR0B |= (1 << TXCIE0);
		UCSR0B |= (1 << UDRIE0) | (1 << TXCIE0);
		rx_index = 0;
		process_data = 1;
	}
	
	sei();
}

ISR (USART_TX_vect)
{
	PORTD &= ~0b00000100; // Отпускаем ножку направления PD2 по прерыванию завершения отправки данных
	UCSR0B &= ~(1 << TXCIE0);
	UCSR0B |= (1 << RXCIE0);
}

ISR(USART_UDRE_vect)
{
	cli();
	PORTD |= 0b00000100; // Поднимаем ножку направления PD2 по прерыванию начала отправки данных
	if(tx_index < 4){ 	// Вывели весь буффер?
		while (!(UCSR0A & (1<<UDRE0)));
		UDR0 = txBuff[tx_index];	// Берем данные из буффера.
		tx_index++;
	}
	else{
		tx_index = 0;
		UCSR0B &= ~(1 << UDRIE0);
		//UCSR0B &= ~(1 << TXCIE0);
		//UCSR0B |= (1 << RXCIE0);
	}
	sei();
}

void init_port(void)
{
	DDRB |= (1<<0) | (1<<1) | (1<<2);
	DDRB |= (1<<6) | (1<<7);
	DDRC |= (1<<0); 
	
	PORTB = 0b00111111;
}

void init_usart(void)
{
	//Set baud
	UBRR0H = (uint8_t)(UBRR >> 8);
	UBRR0L = (uint8_t)UBRR;
	//Enable rx/tx  
	UCSR0B = (1<<TXCIE0) | (1<<RXCIE0) |(1 << RXEN0) | (1 << TXEN0);
	//Set frame format 8N1
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
	sei();
}

// void tx_usart(uint8_t c)
// {
// 	//while(!(UCSR0A & ( 1 << UDRE0 ))); //waits until something was received
// 	UDR0 = c;
// }
void tx_uart_string(char str[]){
	//int len = strlen(str);
	//for(int i=0;i<len;i++){
	//	tx_usart(str[i]);
	//}
	//memcpy(txBuff, str, sizeof(str));
	for (int i = 0; i < 5; i ++){           
		txBuff[i] = str[i];
	}
	tx_index = 0;
	UDR0 = 'S';		// Отправляем первый байт
	UCSR0B|=(1<<UDRIE0);
}

int waitGoal = 0;
int main(void)
{
    init_port();
	init_usart();
	sei();
	///tx_uart_string("pot");
    while (1)
    {
		if(waitGoal = 0){
			int currentSegment = rand()%3;
			set_all_lamp(currentSegment);
			waitGoal = 1;
		}
		wait_goal();
		if (process_data == 1) {
			tx_uart_string(rxBuff);
			process_data = 0;
		}
    }
    return 0;
}

void wait_goal(void){
// 		 if(!(PINB & R0C1SensorPortB)){tx_uart_string("r0c1");waitGoal=0;_delay_ms(300);}
// 	     if(!(PINB & R0C2SensorPortB)){tx_uart_string("r0c2");waitGoal=0;_delay_ms(300);}
// 		 if(!(PINB & R0C0SensorPortB)){tx_uart_string("r0c0");waitGoal=0;_delay_ms(300);}
		if(!(PINB & R0C1SensorPortB)){tx_uart_string("r0c1");waitGoal=0;_delay_ms(400);}	
		if(!(PINB & R0C2SensorPortB)){tx_uart_string("r0c2");waitGoal=0;_delay_ms(400);}
		if(!(PINB & R0C0SensorPortB)){tx_uart_string("r0c0");waitGoal=0;_delay_ms(400);}
	    
}

void set_all_lamp(int currentSegment){
	PORTB = 0b00111111;
	PORTC = 0b00000000;
	
	if(currentSegment==0) PORTB |= R0C0lampPortB;
	if(currentSegment==1) PORTB |= R0C1lampPortB;
	if(currentSegment==2) PORTC |= R0C2lampPortC;
}
