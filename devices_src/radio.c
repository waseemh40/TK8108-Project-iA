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
 void write_fifo(uint8_t *data, uint8_t size){
	 int 		loop_var=0;
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)(REG_LR_FIFOADDRPTR | WNR));
	 spi_write_byte((uint8_t)TX_BASE_ADDRESS);
	 delay_ms(1);
	 for(loop_var=0;loop_var<size;loop_var++){
		 spi_write_byte((uint8_t)(REG_LR_FIFO | WNR));
		 spi_write_byte((uint8_t)data[loop_var]);
		 //delay_ms(0);
	 }
	 spi_cs_set(radio);
	 return;
 }
 uint8_t* read_fifo(uint8_t size){
	 int 		loop_var=0;
	 static uint8_t	data[128];
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)(REG_LR_FIFOADDRPTR | WNR));
	 spi_write_byte((uint8_t)RX_BASE_ADDRESS);
	 delay_ms(1);
	 for(loop_var=0;loop_var<size;loop_var++){
		 spi_write_byte((uint8_t)REG_LR_FIFO);
		 data[loop_var]=spi_read_byte();
		 //delay_ms(0);
	 }
	 spi_cs_set(radio);
	 return data;
 }
 void write_cmd(uint8_t addr,uint8_t cmd){
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)(addr | WNR));
	 spi_write_byte((uint8_t)cmd);
	 spi_cs_set(radio);
	 delay_ms(1);
	 return;
 }
 uint8_t read_cmd(uint8_t addr){
	 uint8_t rx_data=0;
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)addr);
	 rx_data=spi_read_byte();
	 spi_cs_set(radio);
	 delay_ms(1);
	 return rx_data;
 }
 void change_mode(radio_mode_t radio_mode){
 	if(radio_mode==radio_tx_mode){
 		write_cmd(REG_LR_FIFOTXBASEADDR,TX_BASE_ADDRESS);
 		write_cmd(REG_LR_DIOMAPPING1,DIO_TX_MAPPING);
 		write_cmd(REG_LR_OPMODE,TX_MODE);
 	}
 	else if(radio_mode==radio_rx_mode){
 		write_cmd(REG_LR_FIFORXBASEADDR,RX_BASE_ADDRESS);
 		write_cmd(REG_LR_DIOMAPPING1,DIO_RX_MAPPING);
 		write_cmd(REG_LR_OPMODE,RX_MODE);
 	}
 	else if(radio_mode==radio_standby_mode){
 		write_cmd(REG_LR_OPMODE,STDBY_MODE);
 	}
 	else {
 		write_cmd(REG_LR_OPMODE,SLEEP_MODE);
 	}
 	return;
 }
/*
 * public functions
 */
 void RFM_Write(uint8_t addr, uint8_t data){
	 write_cmd(addr,data);
 }
uint8_t RFM_Read(uint8_t addr){
	 return read_cmd(addr);
 }
uint8_t radio_init(radio_mode_t radio_mode)
{
		////////////////////////////////////////////////////
		spi_init();
		///////////////////////////////////////////////////
		GPIO_PinModeSet(PWR_EN_PORT,RADIO_PWR_EN,gpioModePushPull,0);
		GPIO_PinModeSet(RADIO_IO_0345_PORT,RADIO_IO_0,gpioModeInput,0);
		GPIO_PinModeSet(RADIO_IO_12_PORT,RADIO_IO_1,gpioModeInput,0);
		radio_on();
		spi_cs_set(radio);
		delay_ms(7);
	 //Switch RFM to sleep + LoRa mode
  RFM_Write(REG_LR_OPMODE,SLEEP_MODE);

  //Set carrair frequency
  // 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
  RFM_Write(REG_LR_FRFMSB,0xD9);
  RFM_Write(REG_LR_FRFMID,0x06);
  RFM_Write(REG_LR_FRFLSB,0x8B);

  //PA pin (maximal power)
  RFM_Write(REG_LR_PACONFIG,0xFF);

  //BW = 125 kHz, Coding rate 4/5, Explicit header mode
  RFM_Write(REG_LR_MODEMCONFIG1,COFNFIG_SETTINGS_1);

  //Spreading factor 7, PayloadCRC On
  RFM_Write(REG_LR_MODEMCONFIG2,COFNFIG_SETTINGS_2);

  //Low datarate optimization off AGC auto on
  RFM_Write(REG_LR_MODEMCONFIG3,COFNFIG_SETTINGS_3);
  //Rx Timeout set to 37 symbols
  RFM_Write(REG_LR_SYMBTIMEOUTLSB,RX_TIMEOUT);

  //Preamble length set to 8 symbols
  //0x0008 + 4 = 12
  RFM_Write(REG_LR_PREAMBLEMSB,0x00);
  RFM_Write(REG_LR_PREAMBLELSB,PREAMBLE_LENGTH);


  //Set LoRa sync word
  RFM_Write(REG_LR_SYNCWORD,SYNCH_WORD);

  //Set IQ to normal values
  RFM_Write(REG_LR_INVERTIQ,0x27);
  RFM_Write(REG_LR_INVERTIQ2,0x1D);

  //Enable INTs
  RFM_Write( REG_LR_IRQFLAGSMASK,0x00);

  //Set FIFO pointers
  //TX base adress
  RFM_Write(REG_LR_FIFOTXBASEADDR,TX_BASE_ADDRESS);
  //Rx base adress
  RFM_Write(REG_LR_FIFORXBASEADDR,RX_BASE_ADDRESS);
  //change_mode(radio_mode);
  return RFM_Read(REG_LR_OPMODE);
}
 void RFM_Send_Package(uint8_t *RFM_Tx_Package, uint8_t Package_Length)
 {
   unsigned char i;
   unsigned char RFM_Tx_Location = 0x00;

   //Set RFM in Standby mode wait on mode ready
   RFM_Write(REG_LR_OPMODE,SLEEP_MODE);

   //Switch DIO0 to TxDone
   RFM_Write(REG_LR_DIOMAPPING1,DIO_TX_MAPPING);

 /*  //Set carrier frequency
   // 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
   RFM_Write(0x06,0xD9);
   RFM_Write(0x07,0x06);
   RFM_Write(0x08,0x8B);

   //Switch to SF7 CRC on payload on
   RFM_Write(0x1E,0xB4);

   //Set IQ to normal values
   RFM_Write(0x33,0x27);
   RFM_Write(0x3B,0x1D);
*/
   //Set payload length to the right length
   RFM_Write(REG_LR_PAYLOADLENGTH,Package_Length);

   //Get location of Tx part of FiFo
   RFM_Tx_Location = RFM_Read(REG_LR_FIFOTXBASEADDR);

   //Set SPI pointer to start of Tx part in FiFo
   RFM_Write(REG_LR_FIFOADDRPTR,RFM_Tx_Location);

   //Write Payload to FiFo
   for (i = 0;i < Package_Length; i++)
   {
     RFM_Write(REG_LR_FIFO,*RFM_Tx_Package);
     RFM_Tx_Package++;
   }
   //Switch RFM to Tx
   RFM_Write(REG_LR_OPMODE,TX_MODE);

   //Wait for TxDone
   while(!GPIO_PinInGet(RADIO_IO_0345_PORT,RADIO_IO_0))
   {
		sprintf((char *)rs232_tx_buf,"\tTX WAITING on INT C1=%2x\tC2=%2x\tC3=%2x\n",RFM_Read(REG_LR_MODEMCONFIG1),RFM_Read(REG_LR_MODEMCONFIG2),RFM_Read(REG_LR_MODEMCONFIG3));
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		delay_ms(7);
   }
	sprintf((char *)rs232_tx_buf,"\tTX DONE\n");
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));

 }
uint8_t* RFM_Receive(void)
 {

   unsigned char RFM_Interrupt;
   unsigned char Timer = 0x00;
   uint8_t	msg[2];

   //Set carrier freqeuncy
   // 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
   RFM_Write(0x06,0xD9);
   RFM_Write(0x07,0x06);
   RFM_Write(0x08,0x8B);


   //Change DIO 0 to RxDone
   RFM_Write(0x40,0x00);

   //Switch to SF 9 payload on CRC on
   RFM_Write(0x1E,0x94);

   //Invert IQ
   RFM_Write(0x33,0x67);
   RFM_Write(0x3B,0x19);

   //Switch RFM to Single reception
   RFM_Write(0x01,0x86);


   //Wait until RxDone or Timeout
   //Wait until timeout or RxDone interrupt
   while((GPIO_PinInGet(RADIO_IO_0345_PORT,RADIO_IO_0) == 0) && (GPIO_PinInGet(RADIO_IO_12_PORT,RADIO_IO_1) == 0))
   {
		sprintf((char *)rs232_tx_buf,"\tRX WAITING on INT\n");
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		delay_ms(7);
   }
	sprintf((char *)rs232_tx_buf,"\tRX INT Done\n");
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
   //Get interrupt register
   RFM_Interrupt = RFM_Read(0x12);

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
 		sprintf((char *)rs232_tx_buf,"\tRX OK INT\n");
 		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
 		delay_ms(7);     }
     else
     {
 		sprintf((char *)rs232_tx_buf,"\tRX OK WRING CRC\n");
 		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
 		delay_ms(7);
     }
   }

   //Clear interrupt register
   RFM_Write(0x12,0xE0);

   //Switch rfm to standby
   RFM_Write(0x01,0x81);

   return msg;
 }
/*
uint8_t radio_init(radio_mode_t radio_mode){
	uint8_t		ret_val=0;
	uint64_t	F_rf=0;
	////////////////////////////////////////////////////
	spi_init();
	///////////////////////////////////////////////////
	GPIO_PinModeSet(PWR_EN_PORT,RADIO_PWR_EN,gpioModePushPull,0);
	GPIO_PinModeSet(RADIO_IO_0345_PORT,RADIO_IO_0,gpioModeInput,0);
	GPIO_PinModeSet(RADIO_IO_12_PORT,RADIO_IO_1,gpioModeInput,0);
	radio_on();
	spi_cs_set(radio);
	delay_ms(7);
		//Put in Sleep mode
	write_cmd(REG_LR_OPMODE,
			(read_cmd( REG_LR_OPMODE ) &
					RFLR_OPMODE_LONGRANGEMODE_MASK & RFLR_OPMODE_MASK) |
					(RFLR_OPMODE_LONGRANGEMODE_ON ) | ( RFLR_OPMODE_SLEEP ) );

		//Set center frequency
	F_rf = 0xD90667;//((uint64_t)Fc << 14);
	write_cmd(REG_LR_FRFMSB,RFLR_FRFMSB_868_1_MHZ);
	write_cmd(REG_LR_FRFMID,RFLR_FRFMID_868_1_MHZ);
	write_cmd(REG_LR_FRFLSB,RFLR_FRFLSB_868_1_MHZ);
		//LNA config.
	write_cmd(REG_LR_LNA,
			(read_cmd( REG_LR_LNA ) &
					RFLR_LNA_GAIN_MASK & RFLR_LNA_BOOST_HF_MASK) |
					(RFLR_LNA_GAIN_G1) | (RFLR_LNA_BOOST_HF_ON));
		//Configure Modem i.e. SF, BW & Rx timeout
	write_cmd( REG_LR_MODEMCONFIG1,
                 ( read_cmd( REG_LR_MODEMCONFIG1 ) &
                		 RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
						 ( RFLR_MODEMCONFIG1_BW_125_KHZ ) | ( RFLR_MODEMCONFIG1_CODINGRATE_4_5 ) | (RFLR_MODEMCONFIG1_IMPLICITHEADER_ON) );
	write_cmd( REG_LR_MODEMCONFIG2,
                 ( read_cmd( REG_LR_MODEMCONFIG2 ) &
                		 RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
						 ( RFLR_MODEMCONFIG2_SF_7 ) | ( RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF ) | (RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB) );
	//write_cmd( REG_LR_SYMBTIMEOUTLSB, RFLR_SYMBTIMEOUTLSB_SYMBTIMEOUT);
	write_cmd( REG_LR_MODEMCONFIG3,
                 ( read_cmd( REG_LR_MODEMCONFIG3 ) &
                		 RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK & RFLR_MODEMCONFIG3_AGCAUTO_MASK ) |
						 ( RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON ) | ( RFLR_MODEMCONFIG3_AGCAUTO_ON ) );

		//packet structure
	write_cmd(REG_LR_PAYLOADMAXLENGTH, RFLR_PAYLOADMAXLENGTH);
	write_cmd(REG_LR_PREAMBLEMSB,RFLR_PREAMBLELENGTHMSB);
	write_cmd(REG_LR_PREAMBLELSB,RFLR_PREAMBLELENGTHLSB);
	write_cmd(REG_LR_PAYLOADLENGTH,RFLR_PAYLOADLENGTH);
		//Synch word
	//write_cmd(REG_LR_SYNCWORD, 0x34);
		//Inversion of IQ
	write_cmd(REG_LR_INVERTIQ,0x67);
	write_cmd(REG_LR_INVERTIQ2,0x19);
		//Test Regs
	//write_cmd(REG_LR_TEST36,0x02);
	//write_cmd(REG_LR_TEST3A,0x64);
		//Enable IRQs
	write_cmd( REG_LR_IRQFLAGSMASK,//0x00);
    ( read_cmd( REG_LR_IRQFLAGSMASK ) &
   		 RFLR_IRQFLAGS_RXTIMEOUT_MASK & RFLR_IRQFLAGS_RXDONE_MASK & RFLR_IRQFLAGS_VALIDHEADER_MASK & RFLR_IRQFLAGS_CADDETECTED_MASK & RFLR_IRQFLAGS_CADDONE_MASK ) |
			 ( RFLR_IRQFLAGS_RXTIMEOUT ) | ( RFLR_IRQFLAGS_RXDONE ) | (RFLR_IRQFLAGS_VALIDHEADER) | (RFLR_IRQFLAGS_CADDETECTED) | (RFLR_IRQFLAGS_CADDONE) );
	////////////////////////////////////////
	//write_cmd( REG_LR_HOPPERIOD, 0xFF);
	////////////////////////////////////////
		//Set packet structure
//	write_cmd(PREAMBLE_MSB_W,(uint8_t)(PREAMBLE_LENGTH>>8));
//	write_cmd(PREAMBLE_LSB_W,PREAMBLE_LENGTH);
//	write_cmd(PAYLOAD_LENGTH_W,PAYLOAD_LENGTH);
//	write_cmd(MAX_PAYLOAD_LEN_W,MAX_PAYLOAD_LENGTH);
	//////////////////////////////////////////////
	ret_val=read_cmd(REG_LR_FRFMSB);
	sprintf((char *)rs232_tx_buf,"\tFrf_MSB=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_FRFMID);
	sprintf((char *)rs232_tx_buf,"\tFrf_MIB=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_FRFLSB);
	sprintf((char *)rs232_tx_buf,"\tFrf_LSB=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_SYNCWORD);
	sprintf((char *)rs232_tx_buf,"\tSynch Word=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_MODEMCONFIG1);
	sprintf((char *)rs232_tx_buf,"\tConfig_1=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_MODEMCONFIG2);
	sprintf((char *)rs232_tx_buf,"\tConfig_2=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_SYMBTIMEOUTLSB);
	sprintf((char *)rs232_tx_buf,"\tTimeOut=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_MODEMCONFIG3);
	sprintf((char *)rs232_tx_buf,"\tConfig_3=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_LNA);
	sprintf((char *)rs232_tx_buf,"\tLNA=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_IRQFLAGSMASK);
	sprintf((char *)rs232_tx_buf,"\tIRQ Mask=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_PREAMBLEMSB);
	sprintf((char *)rs232_tx_buf,"\tPreamble MSB=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_PREAMBLELSB);
	sprintf((char *)rs232_tx_buf,"\tPreamble LSB=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_PAYLOADLENGTH);
	sprintf((char *)rs232_tx_buf,"\tPayload=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_PAYLOADMAXLENGTH);
	sprintf((char *)rs232_tx_buf,"\tMax Payload=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_INVERTIQ);
	sprintf((char *)rs232_tx_buf,"\tIQ=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_INVERTIQ2);
	sprintf((char *)rs232_tx_buf,"\tIQ2=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_TEST36);
	sprintf((char *)rs232_tx_buf,"\ttest36=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	ret_val=read_cmd(REG_LR_TEST3A);
	sprintf((char *)rs232_tx_buf,"\tTEST3A=%2x\n",ret_val);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	delay_ms(7);
	///////////////////////////////////////////////
		//Finally put in desired mode
	change_mode(radio_mode);
	ret_val=read_cmd(REG_LR_OPMODE);

	return ret_val;
}
*/
void radio_on(void){
	GPIO_PinOutSet(PWR_EN_PORT,RADIO_PWR_EN);
	spi_cs_set(radio);
	return;
}

void radio_off(void){
	GPIO_PinOutClear(PWR_EN_PORT,RADIO_PWR_EN);
	return;
}/*
void radio_transmit_string(uint8_t *tx_buf, uint8_t size){
	change_mode(radio_standby_mode);
	write_fifo(tx_buf,size);
	change_mode(radio_tx_mode);
	delay_ms(1);
	return;
}

uint8_t radio_rx_string(void){
	uint8_t			radio_temp=0;
	uint8_t			radio_count=0;
	change_mode(radio_rx_mode);
	do{
		radio_temp=read_cmd(REG_LR_OPMODE);
		sprintf((char *)rs232_tx_buf,"\tCount =%d Radio Mode=%2x\n",radio_count,radio_temp);
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		delay_ms(7);
		radio_temp=read_cmd(REG_LR_IRQFLAGS);
		sprintf((char *)rs232_tx_buf,"\tIRQ Flags=%2x\n",radio_temp);
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		radio_count++;
		if((radio_temp & 0x02)){
			write_cmd(REG_LR_IRQFLAGS,0xFF);
			sprintf((char *)rs232_tx_buf,"\tBreaking on INT\n");
			rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
			break;
		}
		if(radio_count>100){break;}
	}while(1);
	change_mode(radio_sleep_mode);
	return radio_temp;
}
*/
