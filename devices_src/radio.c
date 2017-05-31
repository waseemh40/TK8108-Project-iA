/*
 * radio.c
 *
 *  Created on: Apr 11, 2017
 *      Author: waseemh
 */

#include "radio.h"

/*
 * private variables
 */
/*
 * public variables
 */
const 			unsigned char  		rs232_tx_buf[64];

/*
 * private functions
 */
 void write_fifo(unsigned char *data, uint8_t size){
	 int 		loop_var=0;
	 uint8_t	RFM_Tx_Location;

	 //Set Payload length
	 write_reg(REG_LR_PAYLOADLENGTH,size);

	 //Get location of Tx part of FiFo
	 RFM_Tx_Location = read_reg(REG_LR_FIFOTXBASEADDR);

	 //Set SPI pointer to start of Tx part in FiFo
	 write_reg(REG_LR_FIFOADDRPTR,RFM_Tx_Location);

	 //Write Payload to FiFo
	 for (loop_var = 0;loop_var < size; loop_var++)
	  {
		write_reg(REG_LR_FIFO,data[loop_var]);
	 }
	 return;
 }
 void read_fifo(uint8_t size, unsigned char *data){
 	uint8_t 			loop_var=0;
 	uint8_t				RFM_Package_Location=0;

	 //Get start location of Rx package.
 	 RFM_Package_Location = read_reg(REG_LR_FIFORXCURRENTADDR);

 	 //Set SPI pointer to recieved package location
 	 write_reg(REG_LR_FIFOADDRPTR, RFM_Package_Location);

 	 	 //Read FIFO and write into the buffer
 	 for(loop_var=0;loop_var<size;loop_var++){
 		 data[loop_var]=(unsigned char)read_reg(REG_LR_FIFO);
 	 }
 	 return;
  }
 void write_reg(uint8_t addr,uint8_t cmd){
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)(addr | WNR));
	 spi_write_byte((uint8_t)cmd);
	 spi_cs_set(radio);
	 delay_ms(1);
	 return;
 }
 uint8_t read_reg(uint8_t addr){
	 uint8_t rx_data=0;
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)addr);
	 rx_data=spi_read_byte();
	 spi_cs_set(radio);
	 delay_ms(1);
	 return rx_data;
 }

 uint8_t get_package(unsigned char *RFM_Rx_Package)
 {
   unsigned char RFM_Package_Length        = 0x0000;

   RFM_Package_Length   = read_reg(REG_LR_RXNBBYTES); /*Read length of received package*/
   read_fifo(RFM_Package_Length,RFM_Rx_Package);
   return RFM_Package_Length;
 }
/*
 * public functions
 */
uint8_t RFM_Init(void)
{
	GPIO_PinModeSet(PWR_EN_PORT,RADIO_PWR_EN,gpioModePushPull,0);
	GPIO_PinModeSet(RADIO_IO_0345_PORT,RADIO_IO_0,gpioModeInput,0);
	GPIO_PinModeSet(RADIO_IO_12_PORT,RADIO_IO_1,gpioModeInput,0);
	RFM_on();
	spi_cs_set(radio);
	delay_ms(7);
		//Switch RFM to sleep + LoRa mode
	write_reg(REG_LR_OPMODE,SLEEP_MODE);

	//Set carrier frequency
	// 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
	write_reg(REG_LR_FRFMSB,0xD9);
	write_reg(REG_LR_FRFMID,0x06);
	write_reg(REG_LR_FRFLSB,0x8B);

	//PA pin (maximal power)
	write_reg(REG_LR_PACONFIG,0xFF);

	//BW = 125 kHz, Coding rate 4/5, Explicit header mode
	write_reg(REG_LR_MODEMCONFIG1,COFNFIG_SETTINGS_1);

	//Spreading factor 7, PayloadCRC On
	write_reg(REG_LR_MODEMCONFIG2,COFNFIG_SETTINGS_2);

	//Low DataRate optimization off AGC auto on
	write_reg(REG_LR_MODEMCONFIG3,COFNFIG_SETTINGS_3);

	//Rx Timeout set to 37 symbols
	write_reg(REG_LR_SYMBTIMEOUTLSB,RX_TIMEOUT);

	//Pre-amble length set to 8 symbols
	//0x0008 + 4 = 12
	write_reg(REG_LR_PREAMBLEMSB,0x00);
	write_reg(REG_LR_PREAMBLELSB,PREAMBLE_LENGTH);


	//Set LoRa sync word
	write_reg(REG_LR_SYNCWORD,SYNCH_WORD);

	//Set IQ to normal values
	write_reg(REG_LR_INVERTIQ,0x27);
	write_reg(REG_LR_INVERTIQ2,0x1D);

	//Enable INTs
	write_reg( REG_LR_IRQFLAGSMASK,0x00);

		//Set FIFO pointers
	//TX base address
	write_reg(REG_LR_FIFOTXBASEADDR,TX_BASE_ADDRESS);
	//Rx base address
	write_reg(REG_LR_FIFORXBASEADDR,RX_BASE_ADDRESS);

	//change_mode(radio_mode);
	return read_reg(REG_LR_OPMODE);
}
void RFM_Send_Package(uint8_t *RFM_Tx_Package, uint8_t Package_Length)
{
	   //Set RFM in Standby mode wait on mode ready
	   write_reg(REG_LR_OPMODE,STDBY_MODE);

	   //Switch DIO0 to TxDone
	   write_reg(REG_LR_DIOMAPPING1,DIO_TX_MAPPING);

	   //write to FIFO
	   write_fifo(RFM_Tx_Package,Package_Length);

	   //Switch RFM to Tx
	   write_reg(REG_LR_OPMODE,TX_MODE);

	   //Wait for TxDone
	   while(!GPIO_PinInGet(RADIO_IO_0345_PORT,RADIO_IO_0))
	   {
			delay_ms(7);
	   }

	   //Clear interrupt register
	   write_reg(REG_LR_IRQFLAGS,0x01);

	   //Switch RFM to sleep mode
	   write_reg(REG_LR_OPMODE,SLEEP_MODE);

	   return;
}
uint8_t RFM_Receive(unsigned char  *msg)
{

  uint8_t 		RFM_Interrupt=0;
  uint8_t 		msg_length=0;

  //Set RFM in Standby mode wait on mode ready
  write_reg(REG_LR_OPMODE,STDBY_MODE);

  //Switch DIO0 to TxDone
  write_reg(REG_LR_DIOMAPPING1,DIO_RX_MAPPING);

  //Invert IQ
  //write_reg(REG_LR_INVERTIQ,0x67);
  //write_reg(REG_LR_INVERTIQ2,0x19);
  //write_reg(REG_LR_TEST36,0x02);
  //write_reg(REG_LR_TEST3A,0x64);

  //Switch RFM to Single reception
  write_reg(REG_LR_OPMODE,RX_MODE);

  //Wait until RxDone or Timeout

  while((GPIO_PinInGet(RADIO_IO_0345_PORT,RADIO_IO_0) == 0) && (GPIO_PinInGet(RADIO_IO_12_PORT,RADIO_IO_1) == 0))
  {
	   delay_ms(7);
  }

  //Get interrupt register
  RFM_Interrupt = read_reg(REG_LR_IRQFLAGS);

  //Check for Timeout
  if(GPIO_PinInGet(RADIO_IO_12_PORT,RADIO_IO_1) == 1)
  {
		sprintf((char *)rs232_tx_buf,"\tRX TIMEOUT INT\n");
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		delay_ms(7);
  }

  //Check for RxDone
  if(GPIO_PinInGet(RADIO_IO_0345_PORT,RADIO_IO_0) == 1)
  {
    //Check CRC
    if((RFM_Interrupt & 0x20) != 0x20)
    {
		write_reg(REG_LR_IRQFLAGS,RFM_Interrupt);
		msg_length=get_package(msg);
    }
    else
    {
		sprintf((char *)rs232_tx_buf,"\tCRC Error\n");
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		delay_ms(7);
    }
  }

  //Clear interrupt register
  write_reg(REG_LR_IRQFLAGS,0xE0);

  //Switch RFM to sleep mode
  write_reg(REG_LR_OPMODE,SLEEP_MODE);

  return msg_length;
}

void RFM_on(void){
	GPIO_PinOutSet(PWR_EN_PORT,RADIO_PWR_EN);
	spi_cs_set(radio);
	return;
}

void RFM_off(void){
	GPIO_PinOutClear(PWR_EN_PORT,RADIO_PWR_EN);
	return;
}
