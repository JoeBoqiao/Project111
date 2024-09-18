/*
 * main.c
 */

#include <avr/io.h>
#include <math.h>
#include "avr/interrupt.h"
#define BAUD 9600 //���ò�����
#define F_CPU 8000000UL//����CPUʱ��
#define MYUBRR F_CPU/(16*BAUD)-1 //�Ĵ�����ֵ����
#include "util/setbaud.h"
#include "avr/sfr_defs.h"//����loop_until_bit_is_set����
// Standard Input/Output functions
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/delay.h>
#include <stdlib.h>
#include "util/delay.h"
#include "string.h"
#include "./twi.h"
#include "./si5351.h"
#include "screen_cmds.h"
#include "i2c.h"

//�嵵����ÿ����λ��Ӧ��Ƶ��
#define Fre1  100
#define Fre2  1000
#define Fre3  10000
#define Fre4  100000
#define Fre5  1000000
uint32_t FA = 14000000; //ȱʡƵ��12MHz
uint32_t FA1 = 1000;

#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)
// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 16
char rx_buffer0[RX_BUFFER_SIZE0];
volatile unsigned char Rx_flag=0;


#if RX_BUFFER_SIZE0 < 256
volatile unsigned char rx_counter0=0;
#else
volatile unsigned int rx_counter0=0;
#endif

//TXEN�˿����
#define TX_EN PORTB	&= 0b11111110
#define RX_EN PORTB	|= 0b00000001
// This flag is set on USART0 Receiver buffer overflow
char rx_buffer_overflow0;
uint8_t mode = 1;//1:TX 0:RX
char str[100];
int checkOn = 0;

void Init_TXRX(void)//TXEN���ų�ʼ��
{
	//  DTR���ŷ�������
	DDRB &= 0b11111101;//PB1���ó����롣��0������������ó�����
	//  TXEN���ŷ�������
	DDRB |= 1 << DDB0;//PB0���ó��������1������������ó����
	//  DISP_RST���ŷ�������
	DDRB |= 1 << DDB3;//PB0���ó��������1������������ó����
	PORTB&=0b11111110;//PB0���0, /TXEN���0����Ĭ�ϴ���TX״̬
	PORTB|=0b00001000;//PB3���1, ��DISP_RST�øߣ�������Reset LCD ����EA DOGS164-A datasheet֪����LCD��ʾ���͸�λ��
}
//uart0���͵��ֽ�����
void putchar0(unsigned char c)
{
	while (!( UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

//uart0���յ��ֽ�����
//char getchar0(char c)
//{
//    while ( !(UCSR0A & (1<<RXC0)) )
//    return UDR0;
//}

//uart0�����ַ�������
void puts0(char *s)
{
	while(*s)
	{
		putchar0(*s);
		s++;
	}
}


// USART0 Receiver interrupt service routine����0�����ж�
ISR(USART0_RX_vect)
{
    static volatile unsigned char status;
    volatile char data;
    status = UCSR0A;
	while ( !(UCSR0A & (1<<RXC0)) );
    data = UDR0;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {
       rx_buffer0[rx_counter0++]=data;
       if (rx_counter0 > 0&&(rx_buffer0[rx_counter0]==0))
      {
      //  rx_counter0=0;
           rx_buffer_overflow0=1;
      }
// 	  else if(rx_counter0 > =16){
// 		   rx_counter0=0;
// 	  }
    }
}

// ���ڳ�ʼ��
void USART0_Init(void)
{
    // USART0 initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART0 Receiver: On
    // USART0 Transmitter: On
    // USART0 Mode: Asynchronous
    // USART0 Baud Rate: 9600
    cli();//���ж�
    UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
    /*Enable receiver and transmitter */
    UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
    /* Set frame format: 8data, 1stop bit */
    UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
    /*Set baud rate */
       UBRR0H=0;
       UBRR0L=0x33;//9600
//      UBRR0H = (unsigned char)(MYUBRR>>8);
//      UBRR0L = (unsigned char)MYUBRR;


    // Ensure that the USART0 is enabled
     PRR0&= ~(1<<PRUSART0);

	/* Set baud rate */
// 	UBRR0H = (unsigned char)(MYUBRR>>8);
// 	UBRR0L = (unsigned char)MYUBRR;
// 	/* Enable receiver and transmitter */
// 	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
// 	/* Set frame format: 8data, 2stop bit */
// 	UCSR0C = (1<<USBS0)|(3<<UCSZ00);

    // Globally enable interrupts
    sei();

}
//ADC��ʼ�� δʹ�ã����嵵���������
void ADC_Init()
{
	/* Make ADC port as input */
	DDRA = 0x00;
	/* Enable ADC, fr/128  */
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	/* Vref: AREF, ADC channel: 0 */
	ADMUX = 0b01000000;
}
//ADC��ȡ δʹ�ã����嵵���������
int ADC_Read(char channel)
{
	volatile uint8_t Ain=0,AinLow=0;
	
	/* TODO:Set input channel to read (ADMUX)*/
	ADMUX = 0x40 | (channel & 0x70);
	/* TODO:Start conversion (ADCSRA)*/
	ADCSRA |= (1<<ADSC);
	/* TODO: wait until end of conversion interrupt is low (ADSRA) */
	while (!(ADCSRA & (1<<ADIF)));
	ADCSRA |= (1<<ADIF);
	_delay_us(10);
	/* TODO: Read lower byte (ADCL)*/
	AinLow = ADCL;
	/* TODO: Read higher 2 bits and 
	Multiply with weight (ADCH)*/
	Ain = ADCH;
	// Combine upper and lower bits
	Ain = Ain + AinLow;				
	return(Ain);			/* Return digital value*/
}


void screen_init(void)
{
	// TODO: Initialize screen
	I2Csendcmd(SCREEN_ADDR,COMMAND_8BIT_4LINES_NORMAL_RE1_IS0);//Function Set
	I2Csendcmd(SCREEN_ADDR,COMMAND_NW);//Extended function set
	I2Csendcmd(SCREEN_ADDR,COMMAND_SEGMENT_BOTTOM_VIEW);//Entry mode set
	I2Csendcmd(SCREEN_ADDR,COMMAND_BS1_1);//Bais setting
	I2Csendcmd(SCREEN_ADDR,COMMAND_8BIT_4LINES_RE0_IS1);//Function Set
	I2Csendcmd(SCREEN_ADDR,COMMAND_BS0_1);//Internal OSC
	I2Csendcmd(SCREEN_ADDR,COMMAND_FOLLOWER_CONTROL);//Follower control
	I2Csendcmd(SCREEN_ADDR,COMMAND_POWER_BOOSTER_CONTRAST);//Power control
	I2Csendcmd(SCREEN_ADDR,COMMAND_SET_CONTRAST_1010);//Contrast Set
	I2Csendcmd(SCREEN_ADDR,COMMAND_8BIT_4LINES_RE0_IS0);//Function Set
	I2Csendcmd(SCREEN_ADDR,COMMAND_DISPLAY_ON_CURSOR_ON_BLINK_ON);//Display On
	I2Csendcmd(SCREEN_ADDR,COMMAND_CLEAR_DISPLAY);
 }

void Key5_Init(void)
{
	DDRA = 0x00;//input
	PORTA = 0xff;//pud	
}

void Key5_Scan(void)//PA2 PA3 PA4 PA5 PA6
{
	uint8_t i = 0x00;
	i = PINA;
	i &= 0b11111000;//������˿���Ϊ�嵵���ص�IO��ǰ��λ������
	switch(i)
	{
		case 0b11110000:FA1 = Fre1;break;
		case 0b11101000:FA1 = Fre2;break;
		case 0b11011000:FA1 = Fre3;break;
		case 0b10111000:FA1 = Fre4;break;
		case 0b01111000:FA1 = Fre5;break;
		default:FA1 = Fre1;
	}
}

void screen_write_string(char string_to_write[])
{
	int letter=0;
	
	I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
	I2Csendcmd(SCREEN_ADDR, COMMAND_SET_CURSOR_LINE_1);
	int current_line = COMMAND_SET_CURSOR_LINE_1;
	
	while(string_to_write[letter]!='\0')
	{
		if ((letter != 0) && (letter % LINE_LENGTH == 0))
		{
			if (current_line == COMMAND_SET_CURSOR_LINE_4){
				// We've gone past the end of the screen, go back to top
				current_line = COMMAND_SET_CURSOR_LINE_1;
				// Clear the screen 
				I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
			}
			else {
				current_line = current_line+0x20;
			}
			// We've gone past the end of the line, go to the next one
			I2Csendcmd(SCREEN_ADDR, current_line); 
		}
		
		I2Csenddatum(SCREEN_ADDR, string_to_write[letter]);
		letter++;
	}
}
//��ȡ�շ����Ĺ���״̬��TX ʱΪ1  RX ʱΪ0��
uint8_t get_state(void)
{
		uint8_t i;
		i = PINA;//��PA��8������״̬�ֽ�
		i &= 0x04;//ȡ�����λ
		if (i==0)//���������£����ӵ���
		{
			_delay_ms(10);//�ӳ�10ms, �ٴ��жϣ�ȥ����Ҫ���ֶ�����
			i = PINA;//�ٴζ�PA��8������״̬�ֽ�
			i &= 0x04;//ȡ�����λ
			if (i==0)//������ȷ�ϰ��£�ȷ��������������
			{
				mode=!mode;//��תȫ�ֱ���mode��״̬��line 55��mode���塪��1:TX 0:RX
			}
		}

}

//��ȡƵ�����Ӱ�ť��״̬���Ƿ�������100Hz������
void get_freqadd(void)
{
	uint8_t i;
	i = PINA;//��PA��8������״̬�ֽ�
	i &= 0x01;//ȡ��PA1λ
	if (i==0)//���������£����ӵ���
	{
		_delay_ms(10);//�ӳ�10ms, �ٴ��жϣ�ȥ����Ҫ���ֶ�����
		i = PINA;//�ٴζ�PA��8������״̬�ֽ�
		i &= 0x01;//ȡ��PA1λ
		if (i==0)//������ȷ�ϰ��£�ȷ��������������
		{
			if (FA>=1000000&&FA<=375000000)//ȫ�ֱ���FAƵ������1MHz��375MHz֮�䣬����FA���䡣FA��line 31����
			{
				FA+=FA1;//�������£�FA����100Hz
				checkOn = 1;
			}
		}
	}
	
}

//��ȡƵ�����Ӱ�ť��״̬���Ƿ��Ǽ���100Hz������
void get_freqsub(void)
{
	uint8_t i;
	i = PINA;//��PA��8������״̬�ֽ�
	i &= 0x02;//ȡ��PA2λ
	if (i==0)//���������£����ӵ���
	{
		_delay_ms(10);//�ӳ�10ms, �ٴ��жϣ�ȥ����Ҫ���ֶ�����
		i = PINA;//�ٴζ�PA��8������״̬�ֽ�
		i &= 0x02;//ȡ��PA2λ
		if (i==0)//������ȷ�ϰ��£�ȷ��������������
		{
			if (FA>=1000000&&FA<=375000000)//ȫ�ֱ���FAƵ������1MHz��375MHz֮�䣬����FA���䡣FA��line 31���塣
			{
				FA-=FA1;//�������£�FA����100Hz
				checkOn = 1;
			}
		}
	}
}

int _power0(int base){
	int answer = 1;
	for(int i = 0; i < base ; i++){
		answer *= 10;
	}
	return answer;
}

void USART_Init( unsigned int baud )
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = 'a';
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}

// void USART_Flush( void )
// {
// 	unsigned char dummy;
// 	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
// }

unsigned int findNum(unsigned char hold){
	unsigned int answer;
	for(int i = 0; i < 16; i++){
		answer += (rx_buffer0[i]-48) * pow(10, i);
	}
	return answer;
}


int main(void)
{
    
    CLKPR=(1<<CLKPCE);
	/*CLKPR = 0;*/
     CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);//ʱ�ӳ�ʼ��
	Init_TXRX();
	
	bool enabled = true;
	uint32_t freq_temp = 12000000; //��ȡPC�˴����·����ݵ��м��������ʼ����12MHz

	const int STR_LEN = 40;//������Ļ����ĳ���
	const float VREF = 3.3; // Measure this with a voltmeter
	volatile char string_to_write[STR_LEN];//������Ļ����
	
	_delay_ms(5);
	PORTB &= ~(1<<PINB3); // turn off
	_delay_ms(200);
	PORTB |= (1<<PINB3); // turn on display
	_delay_ms(5);
// 	
// 	//Set up I2C
// 	I2Cinit(); // Done
// 	
	//Initialize display

	

	
	/* bunch of initializations */
  	Key5_Init();
  	twi_init();
	screen_init(); // TO
  	si5351_init();
  	setup_PLL(SI5351_PLL_A, 28, 0, 1);//����fvcΪ700MHz
  	set_LO_freq(FA);
  	enable_clocks(enabled);//����ʱ��
       USART0_Init();
	   /*USART_Init(9600);*/
       _delay_ms(10);
	
	Key5_Scan();
     while (1)
     {
		 
		// ��ȡRX/TX��ת�����Ƿ���
		get_state();//����10ms�ӳ�
		
		// ��ȡƵ�����Ӱ����Ƿ���
		get_freqadd();//����10ms�ӳ�
		
		// ��ȡƵ�ʼ��ٰ����Ƿ���
		get_freqsub();//����10ms�ӳ�

		if(checkOn){
			set_LO_freq(FA);//�������Ƶ��
			checkOn = 0;
		}
		
		//�˷�����λ��������Ϣ��Ƶ�����嵵���ؾ���
		if (rx_buffer_overflow0)//ȷ���жϺ����Ƿ���Ч
		{
			rx_counter0 =0;
			rx_buffer_overflow0 =0;
			if(rx_buffer0[0] >= 48 && rx_buffer0[0] <= 57)//�����λ���������֣��򴮿ڴ�ӡƵ�ʡ�����0��ASCII����48������9��ASCII����57�����յ��ĵ�һλ�ǡ��ַ����֡�
			{
				//freq_temp=*rx_buffer0;
				freq_temp = strtol(rx_buffer0,NULL,0);//*rx_buffer0 ��ȡ�յ����ַ���������strtol�������ַ���ת����32λ�޷������֡�strtol�������Խ�һ��char*���͵��ַ���ת��Ϊһ��long int���͵���������һ��������Ҫת�����ַ������ڶ���������һ��ָ�룬�����洢ת������ַ�����ʣ�ಿ��,������������ʾҪת�����ַ����Ľ�����ʮ�����ơ���0�Ļ���Ĭ����ʮ����
				/*freq_temp = findNum(rx_buffer0);*/
				if(freq_temp <= 350000000 && freq_temp >= 1000000)//���Ƶ����1M��350M֮��ú���������Ӧ
				{
					FA=freq_temp; //�����Ƶ��ָ���FA�ı䣻���򣬲������
					/*puts0(rx_buffer0);*/
				}
			}
			//����ģʽ����
			if(rx_buffer0[0] == 'T')//�����λ������TX1/TX0���򴮿ڴ�ӡ״̬
			{
				if(rx_buffer0[2] == '1')
				{
					mode = 1;
// 					strcpy(str, "the state is TX, Fre:");
// 					sprintf(str + strlen(str), "%ld", FA);
// 					puts0(str);
				}
				else if(rx_buffer0[2] == '0')
				{
					mode = 0;
// 					strcpy(str, "the state is RX, Fre:");
// 					sprintf(str + strlen(str), "%ld", FA);
// 					puts0(str);
				}
				else if(rx_buffer0[1] == 'X')
				{
					if(mode == 1){
						puts0("TX1");
					}
					else if(mode == 0){
						puts0("TX0");
					}
				}
			}
			if(rx_buffer0[0] == 'I')//�����λ������TX1/TX0���򴮿ڴ�ӡ״̬
			{
				if(rx_buffer0[1] == 'F')
				{
					strcpy(str, "IF000");
					sprintf(str + strlen(str), "%ld", FA);
					strcat(str, "000");
 					puts0(str);
				}
			}
			
			
			
			
			memset(rx_buffer0,0,sizeof(rx_buffer0));//���մ������ݵ��������㡣
		}
		switch(mode)
		{
			case 0:
			RX_EN;// TXEN�˿���� RX״̬
			sprintf(string_to_write,"mode:RX,  Fre: %ld Hz",FA);//����Ļ����ʾ�ַ�д������
			screen_write_string(string_to_write);
//  			strcpy(str, "the state is RX, Fre:");
//  			sprintf(str + strlen(str), "%ld", FA);
//  			puts0(str);
			/*USART_Transmit("R");*/
			/*USART_Flush();*/
			/* putchar0('T');*/
			break;//RX
			case 1:
			TX_EN;// TXEN�˿���� TX״̬
			sprintf(string_to_write,"mode:TX,  Fre: %ld Hz",FA);//����Ļ����ʾ�ַ�д������
			screen_write_string(string_to_write);
// 			strcpy(str, "the state is TX, Fre:");
// 			sprintf(str + strlen(str), "%ld", FA);
//  			puts0(str);
			/*USART_Transmit("T");*/
			/*USART_Flush();*/
			  /*putchar0(49);*/
			break;//TX
		}
		

  
	    _delay_ms(100);
    }
}




