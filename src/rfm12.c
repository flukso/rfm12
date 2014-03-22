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


/** \file rfm12.c
 * \brief rfm12 library main file
 * \author Hans-Gert Dahmen
 * \author Peter Fuhrmann
 * \author Soeren Heisrath
 * \version 0.9.0
 * \date 08.09.09
 *
 * All core functionality is implemented within this file.
 */


/************************
 * standard includes
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <string.h>


/************************
 * library internal includes
 * the order in which they are included is important
*/
#include "include/rfm12_hw.h"
#include "include/rfm12_core.h"
#include "rfm12.h"

//for uart debugging
#if RFM12_UART_DEBUG
	#include "test-m8/uart.h"
#endif


/************************
 * library internal globals
*/

//! Buffer and status for packet transmission.
rf_tx_buffer_t rf_tx_buffer;

//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
	//! Buffers and status to receive packets.
	rf_rx_buffer_t rf_rx_buffers[2];
#endif /* RFM12_USE_WAKEUP_TIMER */

//! Global control and status.
rfm12_control_t ctrl;


/************************
 * load other core and external components
 * (putting them directly into here allows GCC to optimize better)
*/

/* include spi functions into here */
#include "include/rfm12_spi.c"

/*
 * include control / init functions into here
 * all of the stuff in there is optional, so there's no code-bloat..
*/
#include "include/rfm12_ctrl.c"

/*
 * include extra features here
 * all of the stuff in there is optional, so there's no code-bloat..
*/
#include "include/rfm12_extra.c"


/************************
 * Begin of library
*/


//! Interrupt handler to handle all transmit and receive data transfers to the rfm12.
/** The receiver will generate an interrupt request (IT) for the
* microcontroller - by pulling the nIRQ pin low - on the following events:
* - The TX register is ready to receive the next byte (RGIT)
* - The FIFO has received the preprogrammed amount of bits (FFIT)
* - Power-on reset (POR)
* - FIFO overflow (FFOV) / TX register underrun (RGUR)
* - Wake-up timer timeout (WKUP)
* - Negative pulse on the interrupt input pin nINT (EXT)
* - Supply voltage below the preprogrammed value is detected (LBD)
*
* The rfm12 status register is read to determine which event has occured.
* Reading the status register will clear the event flags.
*
* The interrupt handles the RGIT and FFIT events by default.
* Upon specific configuration of the library the WKUP and LBD events
* are handled additionally.
*
* \see rfm12_control_t, rf_rx_buffer_t and rf_tx_buffer_t
*/
//if polling is used, do not define an interrupt handler, but a polling function
#if (RFM12_USE_POLLING)
void rfm12_poll(void)
#else
ISR(RFM12_INT_VECT, ISR_NOBLOCK)
#endif
{
	RFM12_INT_OFF();
	uint8_t status;

	//if receive mode is not disabled (default)
	#if !(RFM12_TRANSMIT_ONLY)
		uint8_t i, data;
		static uint16_t checksum; //static local variables produce smaller code size than globals
	#endif /* !(RFM12_TRANSMIT_ONLY) */

	//first we read the first byte of the status register
	//to get the interrupt flags
	status = rfm12_read_int_flags_inline();

	//low battery detector feature
	#if RFM12_LOW_BATT_DETECTOR
	if(status & (RFM12_STATUS_LBD>>8))
	{
		//debug
		#if RFM12_UART_DEBUG >= 2
			uart_putc('L');
		#endif

		//set status variable to low battery
		ctrl.low_batt = RFM12_BATT_LOW;
	}
	#endif /* RFM12_LOW_BATT_DETECTOR */

	//wakeup timer feature
	#if RFM12_USE_WAKEUP_TIMER
	if(status & (RFM12_STATUS_WKUP>>8))
	{
		//debug
		#if RFM12_UART_DEBUG >= 2
			uart_putc('W');
		#endif

		//restart the wakeup timer by toggling the bit on and off
		rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
		rfm12_data(ctrl.pwrmgt_shadow);
	}
	#endif /* RFM12_USE_WAKEUP_TIMER */

	//check if the fifo interrupt occurred
	if(!(status & (RFM12_STATUS_FFIT>>8)))
		goto END;

	//see what we have to do (start rx, rx or tx)
	switch(ctrl.rfm12_state)
	{
		case STATE_RX_IDLE:
			//if receive mode is not disabled (default)
			#if !(RFM12_TRANSMIT_ONLY)
				data = rfm12_read_fifo_inline();

				if(ctrl.rf_buffer_in->status == STATUS_FREE)
				{
					//init the bytecounter - we're adding the GRP byte to the packet head as well!
					ctrl.bytecount = 2;

					//store the GRP byte at the head of the buffer
					//TODO hardcoded right now to 0xD4 -> make this configurable!
					ctrl.rf_buffer_in->buffer[0] = SYNC_LSB;
					checksum = _crc16_update(~0, SYNC_LSB);

					//store the HDR byte
					ctrl.rf_buffer_in->buffer[1] = data;
					checksum = _crc16_update(checksum, data);

					//debug
					#if RFM12_UART_DEBUG >= 2
						uart_putc('I');
						uart_putc(checksum);
					#endif

					ctrl.rfm12_state = STATE_RX_LEN;

					//end the interrupt without resetting the fifo
					goto END;
				}

				/* if we're here, the buffer is full, so we ignore this transmission by resetting the fifo (at the end of the function)  */
			#endif /* !(RFM12_TRANSMIT_ONLY) */
			break;

		case STATE_RX_LEN:
			//if receive mode is not disabled (default)
			#if !(RFM12_TRANSMIT_ONLY)
				//store the LEN byte
				data = rfm12_read_fifo_inline();

				// reset fifo when length byte is out of spec
				if (data > RFM12_MAXDATA) break;

				ctrl.rf_buffer_in->buffer[2] = data;
				checksum = _crc16_update(checksum, data);

				ctrl.num_bytes = data + PACKET_OVERHEAD;

				//debug
				#if RFM12_UART_DEBUG >= 2
					uart_putc('L');
					uart_putc(checksum);
				#endif

				ctrl.bytecount++;
				ctrl.rfm12_state = STATE_RX_ACTIVE;

				//end the interrupt without resetting the fifo
				goto END;
	
			#endif /* !(RFM12_TRANSMIT_ONLY) */
			break;

		case STATE_RX_ACTIVE:
			//if receive mode is not disabled (default)
			#if !(RFM12_TRANSMIT_ONLY)
				//check if transmission is complete
				if(ctrl.bytecount < ctrl.num_bytes)
				{
					data = rfm12_read_fifo_inline();

					//debug
					#if RFM12_UART_DEBUG >= 2
						uart_putc('R');
						uart_putc(data);
					#endif

					//put next byte into buffer, if there is enough space
					if(ctrl.bytecount < RFM12_RX_BUFFER_SIZE)
					{
						ctrl.rf_buffer_in->buffer[ctrl.bytecount] = data;
						checksum = _crc16_update(checksum, data);
					}

					ctrl.bytecount++;

					//end the interrupt without resetting the fifo
					goto END;
				}

				/* if we're here, receiving is done */
				/* the fifo will be reset at the end of the function */

				//debug
				#if RFM12_UART_DEBUG >= 2
					uart_putc('D');
				#endif

				if (checksum != 0)
				{
					//if the checksum does not match, reset the fifo
					break;
				}

				//check whether the sensor node requested an ack 
				if (ctrl.rf_buffer_in->buffer[1] & HDR_ACK)
				{
					rf_tx_buffer.ack[0] = SYNC_MSB;
					rf_tx_buffer.ack[1] = SYNC_LSB;
					rf_tx_buffer.ack[2] = HDR_CTL | (ctrl.rf_buffer_in->buffer[1] & HDR_NODE_ID);
                    // we reply differently to unicast or broadcast packets
					rf_tx_buffer.ack[2] |= (ctrl.rf_buffer_in->buffer[1] & HDR_DST) ? 0 : HDR_DST;
					// we reply differently to OAM or non-OAM packets
					rf_tx_buffer.ack[2] |= (ctrl.rf_buffer_in->buffer[1] & HDR_CTL) ? HDR_ACK : 0;
					rf_tx_buffer.ack[3] = 0x00;

                    //CRC-16 is calculated over entire packet except SYN(1)
					checksum = ~0;

					for (i=1; i<4; i++) {
						checksum = _crc16_update(checksum, rf_tx_buffer.ack[i]);
					}

					rf_tx_buffer.ack[4] = checksum;
					rf_tx_buffer.ack[5] = checksum >> 8;                 

					//set up transciever for immediate tx of ack packet

					//disable rx
					rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);

					//sending an extra dummy byte
					ctrl.num_bytes = RFM12_TX_ACK_SIZE + 1;
					ctrl.bytecount = 0;

					ctrl.rfm12_state = STATE_TX_ACK;

					//fill data register with preamble
					rfm12_data(RFM12_CMD_TX | PREAMBLE);
					rfm12_data(RFM12_CMD_TX | PREAMBLE);

					//enable tx
					rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET);

					//complete rx buffer processing before jumping to END
				}

				//indicate that the buffer is ready to be used
				ctrl.rf_buffer_in->status = STATUS_COMPLETE;

				//switch to other buffer
				ctrl.buffer_in_num = (ctrl.buffer_in_num + 1) % 2;
				ctrl.rf_buffer_in = &rf_rx_buffers[ctrl.buffer_in_num];

				if (ctrl.rfm12_state == STATE_TX_ACK)
					goto END;
	
			#endif /* !(RFM12_TRANSMIT_ONLY) */
			break;

		case STATE_TX:
			//debug
			#if RFM12_UART_DEBUG >= 2
				uart_putc('T');
			#endif

			if(ctrl.bytecount < ctrl.num_bytes)
			{
				//load the next byte from our buffer struct.
				rfm12_data_inline( (RFM12_CMD_TX>>8), rf_tx_buffer.buffer[ctrl.bytecount++]);

				//end the interrupt without resetting the fifo
				goto END;
			}

			/* if we're here, we're finished transmitting the bytes */
			/* the fifo will be reset at the end of the function */

			//flag the buffer as free again
			ctrl.txstate = STATUS_FREE;

			//set the length of the tx buffer to 0
			rf_tx_buffer.len = 0;

			//wakeup timer feature
			#if RFM12_USE_WAKEUP_TIMER
				//clear wakeup timer once
				rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
				//set shadow register to default receive state
				//the define correctly handles the transmit only mode
				ctrl.pwrmgt_shadow = (RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);
			#endif /* RFM12_USE_WAKEUP_TIMER */

			//turn off the transmitter and enable receiver
			//the receiver is not enabled in transmit only mode
			//if the wakeup timer is used, this will re-enable the wakeup timer bit
			//the magic is done via defines
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);

			//load a dummy byte to clear int status
			rfm12_data_inline( (RFM12_CMD_TX>>8), 0xaa);
			break;

		case STATE_TX_ACK:
			//debug
			#if RFM12_UART_DEBUG >= 2
				uart_putc('A');
			#endif

			if(ctrl.bytecount < ctrl.num_bytes)
			{
				//load the next byte from our buffer struct.
				rfm12_data_inline( (RFM12_CMD_TX>>8), rf_tx_buffer.ack[ctrl.bytecount++]);

				//end the interrupt without resetting the fifo
				goto END;
			}

			/* if we're here, we're finished transmitting the bytes */
			/* the fifo will be reset at the end of the function */

			//wakeup timer feature
			#if RFM12_USE_WAKEUP_TIMER
				//clear wakeup timer once
				rfm12_data(ctrl.pwrmgt_shadow & ~RFM12_PWRMGT_EW);
				//set shadow register to default receive state
				//the define correctly handles the transmit only mode
				ctrl.pwrmgt_shadow = (RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);
			#endif /* RFM12_USE_WAKEUP_TIMER */

			//turn off the transmitter and enable receiver
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);

			//load a dummy byte to clear int status
			rfm12_data_inline( (RFM12_CMD_TX>>8), 0xaa);
			break;

	}

	//set the state machine to idle
	ctrl.rfm12_state = STATE_RX_IDLE;

	//reset the receiver fifo, if receive mode is not disabled (default)
	#if !(RFM12_TRANSMIT_ONLY)
		rfm12_data_inline(RFM12_CMD_FIFORESET>>8, CLEAR_FIFO_INLINE);
		rfm12_data_inline(RFM12_CMD_FIFORESET>>8, ACCEPT_DATA_INLINE);
	#endif /* !(RFM12_TRANSMIT_ONLY) */

	END:
	//turn the int back on
	RFM12_INT_ON();
}


//! The tick function implements collision avoidance and initiates transmissions.
/** This function has to be called periodically.
* It will read the rfm12 status register to check if a carrier is being received,
* which would indicate activity on the chosen radio channel. \n
* If there has been no activity for long enough, the channel is believed to be free.
*
* When there is a packet waiting for transmission and the collision avoidance
* algorithm indicates that the air is free, then the interrupt control variables are
* setup for packet transmission and the rfm12 is switched to transmit mode.
* This function also fills the rfm12 tx fifo with a preamble.
*
* \warning Warning, if you do not call this function periodically, then no packet will get transmitted.
* \see rfm12_tx() and rfm12_start_tx()
*/
void rfm12_tick(void)
{
	//collision detection is enabled by default
	#if !(RFM12_NOCOLLISIONDETECTION)
		uint16_t status;

		//start with a channel free count of 16, this is necessary for the ASK receive feature to work
		static uint8_t channel_free_count = 16; //static local variables produce smaller code size than globals
	#endif

	//debug
	#if RFM12_UART_DEBUG
		static uint8_t oldstate;
		uint8_t state = ctrl.rfm12_state;
		if (oldstate != state)
		{
			uart_putstr ("mode change: ");
			switch (state)
			{
				case STATE_RX_IDLE:
					uart_putc ('i');
					break;
				case STATE_RX_ACTIVE:
					uart_putc ('r');
					break;
				case STATE_TX:
					uart_putc ('t');
					break;
				default:
					uart_putc ('?');
			}
			uart_putstr ("\r\n");
			oldstate = state;
		}
	#endif

	//don't disturb RFM12 if transmitting or receiving
	if(ctrl.rfm12_state != STATE_RX_IDLE)
	{
		return;
	}

	//collision detection is enabled by default
	#if !(RFM12_NOCOLLISIONDETECTION)
		//disable the interrupt (as we're working directly with the transceiver now)
		//hint: we could be losing an interrupt here
		//solutions: check status flag if int is set, launch int and exit ... OR implement packet retransmission
		RFM12_INT_OFF();
		status = rfm12_read(RFM12_CMD_STATUS);
		RFM12_INT_ON();

		//check if we see a carrier
		if(status & RFM12_STATUS_RSSI)
		{
			//yes: reset free counter and return
			channel_free_count = CHANNEL_FREE_TIME;
			return;
		}

		//no: decrement counter
		channel_free_count--;

		//is the channel free long enough ?
		if(channel_free_count != 0)
		{
			return;
		}

		//reset the channel free count for the next decrement (during the next call..)
		channel_free_count = 1;
	#endif

	//do we have something to transmit?
	if(ctrl.txstate == STATUS_COMPLETE)
	{ //yes: start transmitting
		//disable the interrupt (as we're working directly with the transceiver now)
		//hint: we could be losing an interrupt here, too
		//we could also disturb an ongoing reception,
		//if it just started some cpu cycles ago
		//(as the check for this case is some lines (cpu cycles) above)
		//anyhow, we MUST transmit at some point...
		RFM12_INT_OFF();

		//disable receiver - if you don't do this, tx packets will get lost
		//as the fifo seems to be in use by the receiver
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);

		//calculate number of bytes to be sent by ISR
		//sync + header + body + crc is already accounted for + 1 dummy byte
		ctrl.num_bytes = rf_tx_buffer.len + 1;

		//reset byte sent counter
		ctrl.bytecount = 0;

		//set mode for interrupt handler
		ctrl.rfm12_state = STATE_TX;

		//wakeup timer feature
		#if RFM12_USE_WAKEUP_TIMER
			ctrl.pwrmgt_shadow = (RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET);
		#endif /* RFM12_USE_WAKEUP_TIMER */

		//fill 2byte 0xAA preamble into data register
		//the preamble helps the receivers AFC circuit to lock onto the exact frequency
		//(hint: the tx FIFO [if el is enabled] is two staged, so we can safely write 2 bytes before starting)
		rfm12_data(RFM12_CMD_TX | PREAMBLE);
		rfm12_data(RFM12_CMD_TX | PREAMBLE);

		//set ET in power register to enable transmission (hint: TX starts now)
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET);

		//enable the interrupt to continue the transmission
		RFM12_INT_ON();
	}
}

//if receive mode is not disabled (default)
#if !(RFM12_TRANSMIT_ONLY)
	//! Function to clear buffer complete/occupied status.
	/** This function will set the current receive buffer status to free and switch
	* to the other buffer, which can then be read using rfm12_rx_buffer().
	*
	* \see rfm12_rx_status(), rfm12_rx_len(), rfm12_rx_type(), rfm12_rx_buffer() and rf_rx_buffers
	*/
	//warning: without the attribute, gcc will inline this even if -Os is set
	void __attribute__((noinline)) rfm12_rx_clear(void)
	{
			//mark the current buffer as empty
			ctrl.rf_buffer_out->status = STATUS_FREE;

			//switch to the other buffer
			ctrl.buffer_out_num ^= 1;
			ctrl.rf_buffer_out = &rf_rx_buffers[ctrl.buffer_out_num];

	}
#endif /* !(RFM12_TRANSMIT_ONLY) */


//! This is the main library initialization function
/**This function takes care of all module initialization, including:
* - Setup of the used frequency band and external capacitor
* - Setting the exact frequency (channel)
* - Setting the transmission data rate
* - Configuring various module related rx parameters, including the amplification
* - Enabling the digital data filter
* - Enabling the use of the modules fifo, as well as enabling sync pattern detection
* - Configuring the automatic frequency correction
* - Setting the transmit power
*
* This initialization function also sets up various library internal configuration structs and
* puts the module into receive mode before returning.
*
* \note Please note that the transmit power and receive amplification values are currently hard coded.
* Have a look into rfm12_hw.h for possible settings.
*/
void rfm12_init(void)
{
	//initialize spi
	SS_RELEASE();
	DDR_SS |= (1<<BIT_SS);
	spi_init();

	//enable internal data register and fifo
	//setup selected band
	rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_12PF);

	//set power default state (usually disable clock output)
	//do not write the power register two times in a short time
	//as it seems to need some recovery
	rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);

	//set frequency
	rfm12_data(RFM12_CMD_FREQUENCY | RFM12_FREQUENCY_CALC(FREQ) );

	//set data rate
	rfm12_data(RFM12_CMD_DATARATE | DATARATE_VALUE );

	//set rx parameters: int-in/vdi-out pin is vdi-out,
	//Bandwith, LNA, RSSI
	rfm12_data(RFM12_CMD_RXCTRL | RFM12_RXCTRL_P16_VDI
			| RFM12_RXCTRL_VDI_FAST | RFM12_RXCTRL_BW_134 | RFM12_RXCTRL_LNA_0
			| RFM12_RXCTRL_RSSI_91 );

	//automatic clock lock control(AL), digital Filter(!S),
	//Data quality detector value 4, slow clock recovery lock
	rfm12_data(RFM12_CMD_DATAFILTER | RFM12_DATAFILTER_AL | 4);

	//2 Byte Sync Pattern, Start fifo fill when sychron pattern received,
	//disable sensitive reset, Fifo filled interrupt at 8 bits
	rfm12_data(RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4));

	//set AFC to keep the offset during rx, no Limit, fine mode off, active and enabled
	rfm12_data(RFM12_CMD_AFC | RFM12_AFC_AUTO_VDI | RFM12_AFC_LIMIT_OFF
				| RFM12_AFC_OE | RFM12_AFC_EN);

	//set TX Power to -0dB, frequency shift = +-90kHz
	rfm12_data(RFM12_CMD_TXCONF | RFM12_TXCONF_POWER_0 | RFM12_TXCONF_FS_CALC(90000) );

    //PPL settings left to POR defaults

	//disable low dutycycle mode
	rfm12_data(RFM12_CMD_DUTYCYCLE);

	//disable wakeup timer
	rfm12_data(RFM12_CMD_WAKEUP);

    //low batt detection and uC clk division not used

	//if receive mode is not disabled (default)
	#if !(RFM12_TRANSMIT_ONLY)
		//init buffer pointers
		ctrl.rf_buffer_out = &rf_rx_buffers[0];
		ctrl.rf_buffer_in  = &rf_rx_buffers[0];
		//ctrl.buffer_in_num = 0;
		//ctrl.buffer_out_num = 0;
	#endif /* !(RFM12_TRANSMIT_ONLY) */

	//low battery detector feature initialization
	#if RFM12_LOW_BATT_DETECTOR
		ctrl.low_batt = RFM12_BATT_OKAY;
	#endif /* RFM12_LOW_BATT_DETECTOR */

	//enable rf receiver chain, if receiving is not disabled (default)
	//the magic is done via defines
	rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);

	//wakeup timer feature setup
	#if RFM12_USE_WAKEUP_TIMER
		//set power management shadow register to receiver chain enabled or disabled
		//the define correctly handles the transmit only mode
		ctrl.pwrmgt_shadow = (RFM12_CMD_PWRMGT | PWRMGT_RECEIVE);
	#endif /* RFM12_USE_WAKEUP_TIMER */

	//ASK receive mode feature initialization
	#if RFM12_RECEIVE_ASK
		adc_init();
	#endif

	//setup interrupt for falling edge trigger
	RFM12_INT_SETUP();

	//clear int flag
	rfm12_read(RFM12_CMD_STATUS);
	RFM12_INT_FLAG |= (1<<RFM12_FLAG_BIT);

	//init receiver fifo, we now begin receiving.
	rfm12_data(CLEAR_FIFO);
	rfm12_data(ACCEPT_DATA);

	//activate the interrupt
	RFM12_INT_ON();
}

