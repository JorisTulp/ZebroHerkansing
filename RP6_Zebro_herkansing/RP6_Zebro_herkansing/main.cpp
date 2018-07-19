/*
* RP6_Zebro_herkansing.cpp
*
* Created: 3/19/2016 2:14:44 PM
* Author : Joris & Tim
*/

#define F_CPU 16000000

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "TWIDefines.h"
#include "vRegisterDefines.h"


#define TRUE 0xFF
#define FALSE 0
#define SCL_frequentie 100000
#define BAUD 9600
#define DEBUG_ON TRUE

#define DEFAULT_AFSTAND 30
#define DEFAULT_AFSTAND_CALCULATED 2*DEFAULT_AFSTAND/0.0343
#define DEFAULT_SNELHEID 100
#define GAIN 5
#define MARGE 1
#define I2C_SLAVEADDRESS_SENSOR 0x70
#define I2C_SLAVEADDRESS_RP6 0x08
#define I2C_REG_COMMAND 0x00
#define I2C_REG_CENTIMETERS 0x51
#define I2C_REG_PING 0x52
#define I2C_REG_ECHO_2 0x02

/* Function Declaration */
void init_master();
void ontvangen(uint8_t ad,uint8_t b[],uint8_t max);
void verzenden(uint8_t ad,uint8_t b[],uint8_t length);
void initUSART();
void writeChar(char ch);
void writeString(char *string);
void writeInteger(int16_t number, uint8_t base);
uint16_t afstand();
void stuurAfstand(int16_t);
void afstandTest();
void processData();

/* Variable Declaration */
char out[100]; //May be used in combination with sprintf to make a string to send to usart
int gewensteAfstand = 0;
char lastOption;
char buffer[3];
uint8_t iter;

int main(void)
{
	//void bestuurRP6(int motorL, int motorR, int dirL, int dirR, int snelheidL, int snelheidR);
	initUSART();
	init_master();
	sei();

	gewensteAfstand = DEFAULT_AFSTAND_CALCULATED;
 	// // afstandTest(); //Only uncomment this if you want the Arduino to continuously print distance data. RP6 will do nothing else.
	
	writeString("\n\nWelkom bij de Wall-Follow Robot.\nKies een optie:\n");
	writeString("- a: Afstand tot de muur aanpassen (25-50 cm)\n");
	writeString("- s: Default snelheid aanpassen (10-150 units)\n");
	writeString("- p: P-gain aanpassen (0-255)\n");
	writeString("- d: D-gain aanpassen (0-255)\n");
	while (1) {
		//doe dingen
		stuurAfstand((afstand() - (gewensteAfstand)));
		
	}

}


void init_master() {
	TWSR = 0;
	// Set bit rate
	TWBR = ( ( F_CPU / SCL_frequentie ) - 16) / 2;
	TWCR = (1<<TWEN);
}

void ontvangen(uint8_t ad,uint8_t b[],uint8_t max) {
	uint8_t op[15];
	
	TWCR |= (1<<TWSTA);
	while(!(TWCR & (1<<TWINT)));
	op[0] = TWSR;

	TWDR=(ad<<1)+1;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	op[1] = TWSR;
	b[0]=TWDR;
	
	uint8_t tel=0;
	do {
		if(tel == max-1) {
			TWCR=(1<<TWINT)|(1<<TWEN);
		} else {
			TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
		}
		while(!(TWCR & (1<<TWINT)));
		op[tel] = TWSR;
		b[tel] = TWDR;
	} while (op[tel++] == TW_MR_DATA_ACK);

	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);

	//   for(uint8_t i=0;i<tel;++i) {
	//	 writeString("\n\r");writeInteger(op[i],16);
	//	 writeString(" data ");writeInteger(b[i],10);
	//   }

}

void verzenden(uint8_t ad,uint8_t b[],uint8_t length) {
	//  uint8_t op[5];

	TWCR |= (1<<TWSTA);
	while(!(TWCR & (1<<TWINT)));
	TWDR=(ad<<1);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	for(int i = 0;i<length;i++){
		TWDR=b[i];
		TWCR=(1<<TWINT)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
	}
	TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void initUSART() {
	UBRR1 = F_CPU/((long)16 * BAUD) -1;
	UCSR1A = 0x00;
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
	UCSR1B = (1 << TXEN1) | (1 << RXEN1) | (1<<RXCIE1);
	if (DEBUG_ON) {
		writeString("Usart enabled");
	}
}


ISR(USART1_RX_vect) {
	char recData = UDR1;
	switch(recData) {
		case 'a':
			lastOption = recData;
			iter = 0;
			break;
		case 's':
			lastOption = recData;
			iter = 0;
			break;
		case 'p':
			lastOption = recData;
			iter = 0;
			break;
		case 'd':
			lastOption = recData;
			iter = 0;
			break;
		case '\r':
			processData();
			break;
		default:
			if(iter<3)
				buffer[iter++] = recData;
			break;
			
	}
	writeChar(recData);
}

void processData() {
	int valueToSend;
	sscanf(buffer, "%d", &valueToSend);
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;
	uint8_t array[2] = {0,(uint8_t)valueToSend};
	switch(lastOption) {
		case 'a':
			gewensteAfstand = 2*valueToSend/0.0343;
			if(DEBUG_ON) {
				sprintf(out, "De nieuwe gewenste afstand is: %d\n",gewensteAfstand);
				writeString(out);
			}
			break;
		case 's':
			array[0] = DEF_SNELHEID;
			if(DEBUG_ON) {
				sprintf(out, "De verzonden data is: %d naar: %d\n",array[1],array[0]);
				writeString(out);
			}
			verzenden(I2C_SLAVEADDRESS_RP6,array,sizeof(array)/sizeof(uint8_t));
			break;
		case 'p':
			array[0] = GAIN_P;
			if(DEBUG_ON) {
				sprintf(out, "De verzonden data is: %d naar: %d\n",array[1],array[0]);
				writeString(out);
			}
			verzenden(I2C_SLAVEADDRESS_RP6,array,sizeof(array)/sizeof(uint8_t));
			break;
		case 'd':
			array[0] = GAIN_D;
			if(DEBUG_ON) {
				sprintf(out, "De verzonden data is: %d naar: %d\n",array[1],array[0]);
				writeString(out);
			}
			verzenden(I2C_SLAVEADDRESS_RP6,array,sizeof(array)/sizeof(uint8_t));
			break;
	}
	
}

void writeChar(char ch)
{
	while (!(UCSR1A & (1<<UDRE1)));
	UDR1 = (uint8_t)ch;
}

void writeString(char *string)
{
	while(*string)
	writeChar(*string++);
}

void writeInteger(int16_t number, uint8_t base)
{
	char buffer[17];
	itoa(number, &buffer[0], base);
	writeString(&buffer[0]);
}

uint16_t afstand() {
	//uint8_t vraagCM[2] = {I2C_REG_COMMAND,I2C_REG_CENTIMETERS};
	uint8_t vraagCM[2] = {I2C_REG_COMMAND,I2C_REG_PING};	
	uint8_t echoRegister[1] = {I2C_REG_ECHO_2};
	uint8_t ontvangenWaardes[2];
	
	verzenden(I2C_SLAVEADDRESS_SENSOR, vraagCM, sizeof(vraagCM)/sizeof(uint8_t));
	_delay_ms(100);
	verzenden(I2C_SLAVEADDRESS_SENSOR,echoRegister,sizeof(echoRegister)/sizeof(uint8_t));
	ontvangen(I2C_SLAVEADDRESS_SENSOR,ontvangenWaardes,2);
	
	return ((ontvangenWaardes[0] << 8) | ontvangenWaardes[1]);
}


void stuurAfstand(int16_t value) {
	uint8_t tempValue[3];
	tempValue[0] = AFSTAND_H;
	tempValue[1] = value >> 8;
	tempValue[2] = value;

//  	sprintf(out, "De 16bit error is: %d\n\n", value);
//  	writeString(out);
	if(DEBUG_ON) {
		sprintf(out, "Er wordt verzonden: %d ; %d\n\n", tempValue[0], ((tempValue[1] << 8) | tempValue[2]));
		writeString(out);
	}
	verzenden(I2C_SLAVEADDRESS_RP6,tempValue,sizeof(tempValue)/sizeof(uint8_t));
}

void afstandTest() {
	while(1) {
		sprintf(out, "De ontvangen afstand is: %d\n\n", afstand());
		writeString(out);
	}
}