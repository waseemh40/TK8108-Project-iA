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

/*
 * private functions
 */
 void write_fifo(uint8_t *data, uint8_t size){
	 int 		loop_var=0;
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)FIFO_ADR_PTR_W);
	 spi_write_byte((uint8_t)TX_BASE_ADDRESS);
	 delay_ms(1);
	 for(loop_var=0;loop_var<size;loop_var++){
		 spi_write_byte((uint8_t)FIFO_IO_W);
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
	 spi_write_byte((uint8_t)FIFO_ADR_PTR_W);
	 spi_write_byte((uint8_t)RX_BASE_ADDRESS);
	 delay_ms(1);
	 for(loop_var=0;loop_var<size;loop_var++){
		 spi_write_byte((uint8_t)FIFO_IO_R);
		 data[loop_var]=spi_read_byte();
		 //delay_ms(0);
	 }
	 spi_cs_set(radio);
	 return data;
 }
 void write_cmd(uint8_t addr,uint8_t cmd){
	 spi_cs_clear(radio);
	 spi_write_byte((uint8_t)addr);
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
 		write_cmd(FIFO_TX_BASE_ADR_W,TX_BASE_ADDRESS);
 		write_cmd(DIO_MAPPING_1_W,DIO_TX_MAPPING);
 		write_cmd(OP_MODE_W,TX_MODE);
 	}
 	else if(radio_mode==radio_rx_mode){
 		write_cmd(FIFO_RX_BASE_ADR_W,RX_BASE_ADDRESS);
 		write_cmd(DIO_MAPPING_1_W,DIO_RX_MAPPING);
 		write_cmd(OP_MODE_W,RX_MODE);
 	}
 	else if(radio_mode==radio_standby_mode){
 		write_cmd(OP_MODE_W,STDBY_MODE);
 	}
 	else {
 		write_cmd(OP_MODE_W,SLEEP_MODE);
 	}
 	return;
 }
/*
 * public functions
 */

uint8_t radio_init(radio_mode_t radio_mode){
	uint8_t		ret_val=0;
	GPIO_PinModeSet(PWR_EN_PORT,RADIO_PWR_EN,gpioModePushPull,0);
	GPIO_PinModeSet(RADIO_IO_0345_PORT,RADIO_IO_0,gpioModeInput,0);
	GPIO_PinModeSet(RADIO_IO_12_PORT,RADIO_IO_1,gpioModeInput,0);
	radio_on();
		//Put in Sleep mode
	write_cmd(OP_MODE_W,SLEEP_MODE);
		//Set center frequency
	write_cmd(RF_FREQ_MSB_W,FC_MSB);
	write_cmd(RF_FREQ_MIB_W,FC_MIB);
	write_cmd(RF_FREQ_LSB_W,FC_LSB);
		//Set output power and ramp-up time
	write_cmd(PA_CONFIG_W,PA_7dBm);
	write_cmd(PA_RAMP_W,PA_RAMP_TIME);
		//Set LNA parameters
	write_cmd(LNA_W,LNA_GAIN_DEFAULT);
		//Enable IRQs
	write_cmd(IRQ_FLAG_MSK_W,IRQ_ENABLE);
		//Configure Modem i.e. SF, BW & Rx timeout
	write_cmd(MODEM_CONFIG1_W,COFNFIG_SETTINGS_1);
	write_cmd(MODEM_CONFIG2_W,COFNFIG_SETTINGS_1);
	write_cmd(SYM_TIMEOUT_W,RX_TIMEOUT);
		//Set packet structure
	write_cmd(PREAMBLE_MSB_W,(uint8_t)(PREAMBLE_LENGTH>>8));
	write_cmd(PREAMBLE_LSB_W,PREAMBLE_LENGTH);
	write_cmd(PAYLOAD_LENGTH_W,PAYLOAD_LENGTH);
	write_cmd(MAX_PAYLOAD_LEN_W,MAX_PAYLOAD_LENGTH);
		//Inversion of IQ
	write_cmd(INVERT_IQ_W,INVERT_IQ);
		//LoRaWAN type
	write_cmd(SYNCH_WORD_W,SYNCH_WORD);
		//Finally put in desired mode
	change_mode(radio_mode);
	ret_val=read_cmd(OP_MODE_R);
	return ret_val;
}

void radio_on(void){
	GPIO_PinOutSet(PWR_EN_PORT,RADIO_PWR_EN);
	spi_cs_set(radio);
	return;
}

void radio_off(void){
	GPIO_PinOutClear(PWR_EN_PORT,RADIO_PWR_EN);
	return;
}
void radio_transmit_string(uint8_t *tx_buf, uint8_t size){
	change_mode(radio_standby_mode);
	write_fifo(tx_buf,size);
	change_mode(radio_tx_mode);
	delay_ms(1);
	return;
}
