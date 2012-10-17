/**** RFM 12 library for Atmel AVR Microcontrollers *******
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA.
 *
 * @author Peter Fuhrmann, Hans-Gert Dahmen, Soeren Heisrath
 */

/******************************************************
 *    THIS FILE IS BEING INCLUDED DIRECTLY		*
 *		(for performance reasons)		*
 ******************************************************/

//hardware spi helper macros
#define SS_ASSERT() PORT_SS &= ~(1<<BIT_SS)
#define SS_RELEASE() PORT_SS |= (1<<BIT_SS)


#if RFM12_SPI_SOFTWARE
/* @description Actual sending function to send raw data to the Module
 * @note do NOT call this function directly, unless you know what you're doing.
 */
static uint8_t spi_data(uint8_t c)
{
	uint8_t x, d=d;
	for(x=0;x<8;x++){
		if(c & 0x80){
			PORT_MOSI |= (1<<BIT_MOSI);
		}else{
			PORT_MOSI &= ~(1<<BIT_MOSI);
		}
		PORT_SCK |= (1<<BIT_SCK);
		d<<=1;
		if(PIN_MISO & (1<<BIT_MISO)){
			d|=1;
		}
		PORT_SCK &= ~(1<<BIT_SCK);
		c<<=1;
	}
	return d;
}
#endif


//non-inlined version of rfm12_data
//warning: without the attribute, gcc will inline this even if -Os is set
static void __attribute__ ((noinline)) rfm12_data(uint16_t d)
{
	SS_ASSERT();

	#if RFM12_SPI_USART
	uint8_t dummy;

	UDR0 = d>>8;
	while (!(UCSR0A & (1<<RXC0)));

	dummy = UDR0;
	UDR0 = d & 0xff;
	while (!(UCSR0A & (1<<RXC0)));

	dummy = UDR0;

	#elif !(RFM12_SPI_SOFTWARE)
	SPDR = d>>8;
	while(!(SPSR & (1<<SPIF)));

	SPDR = d & 0xff;
	while(!(SPSR & (1<<SPIF)));

	#else
	spi_data(d >> 8   );
	spi_data(d &  0xff);
	#endif
	SS_RELEASE();
}


//non-inlined version of rfm12_read
//warning: without the attribute, gcc will inline this even if -Os is set
static uint16_t __attribute__ ((noinline)) rfm12_read(uint16_t c)
{
	uint16_t retval;
	SS_ASSERT();

	#if RFM12_SPI_USART
	UDR0 = c>>8;
	while (!(UCSR0A & (1<<RXC0)));

	retval = UDR0<<8;
	UDR0 = c & 0xff;
	while (!(UCSR0A & (1<<RXC0)));

	retval |= UDR0;

	#elif !(RFM12_SPI_SOFTWARE)
	SPDR = c>>8;
	while(!(SPSR & (1<<SPIF)));
	retval = SPDR<<8;
	SPDR = c & 0xff;
	while(!(SPSR & (1<<SPIF)));
	retval |= SPDR;

	#else
	retval =  spi_data(c >> 8   );
	retval <<= 8;
	retval |= spi_data(c &  0xff);
	#endif
	SS_RELEASE();
	return retval;
}


/* @description reads the upper 8 bits of the status
 * register (the interrupt flags)
 */
static uint8_t rfm12_read_int_flags_inline(void)
{
	SS_ASSERT();

	#if RFM12_SPI_USART
	UDR0 = 0;
	while (!(UCSR0A & (1<<RXC0)));

	SS_RELEASE();
	return UDR0;

	#elif !(RFM12_SPI_SOFTWARE)
	SPDR = 0;
	while(!(SPSR & (1<<SPIF)));
	SS_RELEASE();
	return SPDR;

	#else
	unsigned char x, d=d;
	PORT_MOSI &= ~(1<<BIT_MOSI);
	for(x=0;x<8;x++){
		PORT_SCK |= (1<<BIT_SCK);
		d<<=1;
		if(PIN_MISO & (1<<BIT_MISO)){
			d|=1;
		}
		PORT_SCK &= ~(1<<BIT_SCK);
	}
	SS_RELEASE();
	return d;
	#endif
}


/* @description inline version of rfm12_data for use in interrupt
 */
static void rfm12_data_inline(uint8_t cmd, uint8_t d)
{
	SS_ASSERT();

	#if RFM12_SPI_USART
	uint8_t dummy;

	UDR0 = cmd;
	while (!(UCSR0A & (1<<RXC0)));

	dummy = UDR0;
	UDR0 = d;
	while (!(UCSR0A & (1<<RXC0)));

	dummy = UDR0;

	#elif !(RFM12_SPI_SOFTWARE)
	SPDR = cmd;
	while(!(SPSR & (1<<SPIF)));

	SPDR = d;
	while(!(SPSR & (1<<SPIF)));

	#else
	spi_data( cmd );
	spi_data( d   );
	#endif
	SS_RELEASE();
}


/* @description inline function for reading the fifo
 */
static uint8_t rfm12_read_fifo_inline(void)
{
	SS_ASSERT();

	#if RFM12_SPI_USART
	uint8_t dummy;

	UDR0 = ( RFM12_CMD_READ >> 8 );
	while (!(UCSR0A & (1<<RXC0)));

	dummy = UDR0; // read and discard the first byte shifted in
	UDR0 = 0;
	while (!(UCSR0A & (1<<RXC0)));

	SS_RELEASE();
	return UDR0;	

	#elif !(RFM12_SPI_SOFTWARE)
	SPDR =  ( RFM12_CMD_READ >> 8 );
	while(!(SPSR & (1<<SPIF)));

	SPDR = 0;
	while(!(SPSR & (1<<SPIF)));

	SS_RELEASE();
	return SPDR;

	#else
	uint8_t retval;
	spi_data( RFM12_CMD_READ >> 8 );
	retval = spi_data( 0   );

	SS_RELEASE();
	return retval;
	#endif
}

static void spi_init(void)
{
	#if RFM12_SPI_USART
	/* AVR317 app note: To ensure that the XCK line is initialized correctly
	 * according to the SPI mode settings, it is important that the UBRR is
	 * set to zero at the time the transmitter is enabled. After enabling the
         * transmitter, the correct value can be set.
	 */
	UBRR0 = 0;

	/* XCK0 as output to configure as master clk */
	DDRD |= (1<<DDD4);

	/* Master SPI (MSPIM), MSB, SPI mode 0 */
	UCSR0C = (1<<UMSEL01) | (1<<UMSEL00) | (0<<UDORD0) | (0<<UCPHA0) | (0<<UCPOL0);

	/* Enable USART Tx and Rx */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);

	/* Set clock rate */
	UBRR0H = (uint8_t)(USART_BAUD_PRESCALE>>8);
	UBRR0L = (uint8_t)(USART_BAUD_PRESCALE);

	#elif !(RFM12_SPI_SOFTWARE)
	DDR_MOSI   |= (_BV(BIT_MOSI));
	DDR_SCK    |= (_BV(BIT_SCK));
	PORT_SPI   |= (_BV(BIT_SPI_SS));
	DDR_SPI    |= (_BV(BIT_SPI_SS));

	DDR_MISO   &= ~(_BV(BIT_MISO));

	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);//SPI Master, clk/16

	#else
	DDR_MOSI   |= (_BV(BIT_MOSI));
	DDR_SCK    |= (_BV(BIT_SCK));
	PORT_SPI   |= (_BV(BIT_SPI_SS));
	DDR_SPI    |= (_BV(BIT_SPI_SS));

	DDR_MISO   &= ~(_BV(BIT_MISO));
	#endif
}
