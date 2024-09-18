#include "./twi.h"

byte twi_error;

/* 
 * custom TWI library functions derived from datasheet
 * Mostly enabling the correct bits in a register to enable the functions
 */

 /* initialise twi registers to i2c on */
void twi_init()
{
	bool pullup_en = false;
	
	DDRC  |= (1 << TW_SDA_PIN) | (1 << TW_SCL_PIN);
	if (pullup_en)
	{
		PORTC |= (1 << TW_SDA_PIN) | (1 << TW_SCL_PIN);
	}
	else
	{
		PORTC &= ~((1 << TW_SDA_PIN) | (1 << TW_SCL_PIN));
	}
	DDRC  &= ~((1 << TW_SDA_PIN) | (1 << TW_SCL_PIN));
	
	
// 	TWSR = 0x01; 
// 	TWBR = 0x01;
	/* Set bit rate register 72 and prescaler to 1 resulting in
	SCL_freq = 16MHz/(16 + 2*72*1) = 100KHz	*/
	//TWBR = 72;
	TWBR = (8000000/100000-16)/2;//8MHz clock
	TWCR |= (1<<TWEN);
}

/* Enable correct registers for twi transmission process to begin */
void twi_start()
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
	
}

void twi_repeat_start()
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

/* Enable correct registers for twi transmission to stop */
void twi_stop()
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); 
}

/* Release control of i2c lines */
void twi_releaseBus()
{
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWEA) | (1<<TWINT);
}

/* slave device write: activate a slave device */
byte twi_MT_SLA_W(byte addr)
{
	uint8_t TWDR_to_write = (addr << 1) | TW_WRITE;
    TWDR = (addr << 1) | TW_WRITE;
    TWCR = (1<<TWINT)|(1<<TWEN);                       
    while (!(TWCR & (1<<TWINT)));
	
	uint8_t test = TWSR;
    return TWSR & ~0x1;
}

/* slave device read: activate a slave device */
byte twi_MR_SLA_R(byte addr)
{
    TWDR = (addr << 1) | TW_READ;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
	return TWSR;
}

/*  write to slave device as a master device */
byte twi_MT_write(byte data)
{
    TWDR = data;
    TWCR =  (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
	return TWSR;
}

/* read slave device expecting an acknoledgement */
byte twi_MR_read_ACK()
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

/* read slave device expecting no acknoledgement */
byte twi_MR_read_NACK()
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

void twi_sendcmd(uint8_t addr, uint8_t data)
{
	twi_start();		//start transmission
	twi_MT_write(addr); //display address
	twi_MT_write(0x40); //write data control bit
	twi_MT_write(data); //write the character
	twi_stop();		//stop transmission
}