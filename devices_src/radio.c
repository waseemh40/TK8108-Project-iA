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
	   sprintf((char *)rs232_tx_buf,"\t Write Fifo => Loop=%d char=%c Addr=%2x\n",loop_var,data[loop_var],read_reg(REG_LR_FIFOADDRPTR));
	   rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	 }
	 return;
 }
 unsigned char* read_fifo(uint8_t size){
	 int 		loop_var=0;
	 static 	 unsigned char	data[128];
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)(REG_LR_FIFOADDRPTR | WNR));
	 spi_write_byte((uint8_t)RX_BASE_ADDRESS);
	 delay_ms(1);
	 for(loop_var=0;loop_var<size;loop_var++){
		 spi_write_byte((uint8_t)REG_LR_FIFO);
		 data[loop_var]=(unsigned char)spi_read_byte();
		 //delay_ms(0);
	 }
	 spi_cs_set(radio);
	 return data;
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
   unsigned char RFM_Package_Location      = 0x0000;

   //RFM_Package_Location = RFM_Read(REG_LR_FIFORXCURRENTADDR); /*Read start position of received package*/
   RFM_Package_Length   = read_reg(REG_LR_RXNBBYTES); /*Read length of received package*/

   write_reg(REG_LR_FIFOADDRPTR,RFM_Package_Location); /*Set SPI pointer to start of package*/
   sprintf((char *)rs232_tx_buf,"\tAddr=%2x\tLength=%2x\n",RFM_Package_Location,RFM_Package_Length);
   rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
   RFM_Rx_Package=read_fifo(RFM_Package_Length);
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
 void RFM_Send_Package(unsigned char  *RFM_Tx_Package, uint8_t Package_Length)
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
		sprintf((char *)rs232_tx_buf,"\tTX WAITING on INT C1=%2x\tC2=%2x\tC3=%2x\n",read_reg(REG_LR_MODEMCONFIG1),read_reg(REG_LR_MODEMCONFIG2),read_reg(REG_LR_MODEMCONFIG3));
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		delay_ms(7);
   }
	sprintf((char *)rs232_tx_buf,"\tTX DONE\n");
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));

 }
 uint8_t* RFM_Receive_Package(void)
  {

    unsigned char RFM_Interrupt;
    unsigned char	msg[20];

    //Set RFM in Standby mode wait on mode ready
    RFM_Write(REG_LR_OPMODE,STDBY_MODE);

    //Switch DIO0 to TxDone
    RFM_Write(REG_LR_DIOMAPPING1,DIO_RX_MAPPING);

    //Switch RFM to Single reception
    RFM_Write(REG_LR_OPMODE,RX_MODE);

    //Wait until RxDone or Timeout
    //Wait until timeout or RxDone interrupt
    while((GPIO_PinInGet(RADIO_IO_0345_PORT,RADIO_IO_0) == 0) && (GPIO_PinInGet(RADIO_IO_12_PORT,RADIO_IO_1) == 0))
    {
 		sprintf((char *)rs232_tx_buf,"\tRX WAITING on INT \tIRQ=%2x \tMode=%2x \tRSSI=%3d\tC1=%2x\tC2=%2x\tC3=%2x\n",RFM_Read(REG_LR_IRQFLAGS),RFM_Read(REG_LR_MODEMSTAT),RFM_Read(REG_LR_RSSIVALUE),RFM_Read(REG_LR_MODEMCONFIG1),RFM_Read(REG_LR_MODEMCONFIG2),RFM_Read(REG_LR_MODEMCONFIG3));
 		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
 		if(RFM_Read(REG_LR_IRQFLAGS)==0x50){
 			sprintf((char *)rs232_tx_buf,"\tValid Packet Rxvd\n");
 			rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
 			break;}
 		delay_ms(10);
    }
 	sprintf((char *)rs232_tx_buf,"\tRX INT Done\n");
 	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
    //Get interrupt register
    RFM_Interrupt = RFM_Read(REG_LR_IRQFLAGS);

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
  		sprintf((char *)rs232_tx_buf,"\tRX OK INT Flags=%2x\n",RFM_Interrupt);
  		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
  		delay_ms(7);
  		RFM_Interrupt=get_package(msg);
 		sprintf((char *)rs232_tx_buf,"\tMSG=%d %d %d\n",msg[2], msg[3], msg[4]);
  		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
 		sprintf((char *)rs232_tx_buf,"\tMSG=%s\n",msg);
  		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));

      }
      else
      {
  		sprintf((char *)rs232_tx_buf,"\tRX OK WRONG CRC\n");
  		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
  		delay_ms(7);
      }
    }

    //Clear interrupt register
    RFM_Write(REG_LR_IRQFLAGS,0xE0);

    //Switch rfm to standby
    RFM_Write(REG_LR_OPMODE,SLEEP_MODE);

    return msg;
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
