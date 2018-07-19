/*
* i2cw.c
*
* Created: 3/19/2016 2:14:44 PM
* Author : john
*/


#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/twi.h>
#include "rp6aansluitingen.h"
#include "VirtualRegister.h"
#include "vRegisterDefines.h"
#include "TWIDefines.h"



#define SCL_frequentie 100000

#define BAUDRATE		38400
#define UBRR_BAUD	(((long)F_CPU/((long)16 * BAUDRATE))-1)
#define resetData()  for(uint8_t i=0;i<20;++i) data[i]=0

#define TRUE 0xFF;
#define FALSE 0;

#define DEFAULT_SNELHEID 100
#define GAINP 5
#define GAIND 2.3

uint8_t data_ont[20]; //max 20
volatile uint8_t data_flag = FALSE;
volatile uint8_t databyte = 0x33;

void ontvangData(uint8_t[], uint8_t);
uint8_t verzendByte();


void ontvangData(uint8_t data[], uint8_t tel);
void setup();
void changespeed(int L, int R);
void changespeedL(int L);
void changespeedR(int R);
void stop();
void move(int left, int right);
void changeDIR(int left, int right);
void bestuurRP6(int motorL, int motorR, int dirL, int dirR, int snelheidL, int snelheidR);


void (*ontfunc) (uint8_t[],uint8_t);
uint8_t (*verfunc) ();

void init_i2c_slave(uint8_t ad);
void slaaftwi();
void initUSART();
void init_i2c_ontvang( void (*ontvanger) (uint8_t [],uint8_t));
void init_i2c_verzend( uint8_t (*verzender) ());
void writeChar(char ch);
void writeString(char *string);
void writeInteger(int16_t number, uint8_t base);

VirtualRegister vReg;

/*de interrupt routine van de i2c*/
ISR(TWI_vect) {

	slaaftwi();

}

int main(void)
{
	//void bestuurRP6(int motorL, int motorR, int dirL, int dirR, int snelheidL, int snelheidR);
	setup();



	initUSART();
	init_i2c_slave(8);

	/*ontvangData is de functie die uitgevoerd wordt
	wanneer een byte via de i2c bus ontvangen wordt
	*/
	init_i2c_ontvang(ontvangData);

	/*verzendByte is de functie die aangeroepen wordt
	wanneer de slave een byte naar de master verzend*/
	init_i2c_verzend(verzendByte);

	sei(); //De slave van i2c werkt met interrupt

	/* Replace with your application code */
	
	double afstandError=0;
	double prevAfstandError;
	
	vReg.writeRegister(DEF_SNELHEID, DEFAULT_SNELHEID);
	vReg.writeRegister(GAIN_P, (uint8_t) (GAINP*10));
	vReg.writeRegister(GAIN_D, (uint8_t) (GAIND*10));

	changespeed(DEFAULT_SNELHEID,DEFAULT_SNELHEID);

	while (1) {
		if(data_flag) {

			int16_t afstandMs = (vReg.readRegister(AFSTAND_H)<<8) | vReg.readRegister(AFSTAND_L);
			prevAfstandError = afstandError;
			afstandError = afstandMs*0.0343;//343m/s -> 0.000343m/us -> 0.0343cm/us

// 			writeInteger((int16_t)afstandError , 10);
// 			writeString("\n\r");

			uint8_t defaultSnelheid = vReg.readRegister(DEF_SNELHEID);
			double gainP = vReg.readRegister(GAIN_P)/10.0;
			double gainD = vReg.readRegister(GAIN_D)/10.0;

			if(afstandError > 0){
				changespeedL(afstandError*gainP + (afstandError-prevAfstandError)*gainD + defaultSnelheid); 
			}else if(afstandError < 0){
				changespeedR((-1*afstandError)*gainP + (afstandError-prevAfstandError)*gainD + defaultSnelheid); 
			}

			data_flag = FALSE;

		}

	}

}
/*slave heeft data ontvangen van de master
data[] een array waarin de ontvangen data staat
tel het aantal bytes dat ontvangen is*/

void ontvangData(uint8_t data[], uint8_t tel){

	uint8_t registerAddress = data[0];

 	for (int i = 1; i<tel; ++i)
		vReg.writeRegister(registerAddress++,data[i]);

	data_flag = TRUE;
//	writeString("o\n\r");
}

/* het byte dat de slave verzend naar de master
in dit voorbeeld een eenvoudige teller
*/

uint8_t verzendByte() {
	return databyte++;
}


//setup ports/pwm
void setup(){
	DDRB = 0xFF;
	DDRC = 0xFF;
	DDRD = 0xFF;

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);//pwm klok aanzetten.
	TCCR1B = (1 << CS10);//pwm klok aanzetten.
}

//verander snelheid van de robot
void changespeed(int L, int R){
	changespeedL(L);
	changespeedR(R);
}

//verander snelheid van de robot
void changespeedL(int L){

	if(L>200) L=200;

	OCR1AH = 0;//unused dus 0
	OCR1AL = vReg.readRegister(0x00);//verander snelheid (255 max)
	OCR1BH = 0;//unused dus 0
	OCR1BL = L;//verander snelheid (255 max)
}

//verander snelheid van de robot
void changespeedR(int R){

	if(R>200) R=200;

	OCR1AH = 0;//unused dus 0
	OCR1AL = R;//verander snelheid (255 max)
	OCR1BH = 0;//unused dus 0
	OCR1BL = vReg.readRegister(0x00);//verander snelheid (255 max)
}

// stop motor en reset direction en snelheid
void stop(){
	PORTD &= ~(MOTOR_R | MOTOR_L);
	changeDIR(0, 0);
	changespeed(0, 0);
}

//zet motor aan zonder aangegeven stoptijd
void move(int left, int right){
	if (left == 1){
		PORTD |= MOTOR_L;
	}
	if (right == 1){
		PORTD |= MOTOR_R;
	}
}


//verander draai richting van de wielen 1 = achteruit 0 = vooruit
void changeDIR(int left, int right){
	if (right == 1){
		PORTC |= DIR_R;
	}
	else{
		PORTC &= ~DIR_R;
	}
	if (left == 1){
		PORTC |= DIR_L;
	}
	else{
		PORTC &= ~DIR_L;
	}
}

//bestuurRP6(usemotorL?,usemotorR?,directionL,directionR,snelheidL,snelheidR);
void bestuurRP6(int motorL, int motorR, int dirL, int dirR, int snelheidL, int snelheidR){
	changeDIR(!dirL, !dirR);
	changespeed(snelheidL, snelheidR);
	move(motorL, motorR);
}

///////////////////////
void init_i2c_slave(uint8_t ad) {
	
	TWSR = 0;
	TWBR = ((F_CPU / SCL_frequentie) - 16) / 2;
	TWCR = (1 << TWIE) | (1 << TWEN) | (1 << TWEA);
	TWAR = ad<<1;
}

void slaaftwi() {
	static uint8_t data[40];
	static uint8_t teller=0;
	switch(TWSR) {
		case 0x10:
		case 0x08:
			break;
		case 0x60:
			teller=0;
			break;
		case 0x80:
			data[teller++] = TWDR;
			break;
		case 0xA0:
			ontfunc(data,teller);
			resetData();
			break;
		case 0xA8:
			teller=0;
			TWDR=verfunc();
			break;
		case 0xB8:
			TWDR=verfunc();
			break;
		default:
			break;
	}
	TWCR |= (1<<TWINT);    // Clear TWINT Flag
}

void initUSART() {

	UBRRH = UBRR_BAUD >> 8;
	UBRRL = (uint8_t) UBRR_BAUD;
	UCSRA = 0x00;
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	UCSRB = (1 << TXEN) | (1 << RXEN);
	//writeString("usart werkt nog\n\r");
}


void init_i2c_ontvang( void (*ontvanger) (uint8_t [],uint8_t)) {
	ontfunc=ontvanger;
}

void init_i2c_verzend( uint8_t (*verzender) ()) {
	verfunc=verzender;
}

void writeChar(char ch)
{
	while (!(UCSRA & (1<<UDRE)));
	UDR = (uint8_t)ch;
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


/*
void rotate(int dir, int x){
if(dir == 1){
changeDIR(0,1);
} else {
changeDIR(1,0);
}
move(1,1,x);
changeDIR(0,0);
}
*/