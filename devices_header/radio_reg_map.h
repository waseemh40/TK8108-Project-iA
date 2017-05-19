/*
 * radio_reg_map.h
 *
 *  Created on: Apr 11, 2017
 *      Author: waseemh
 */

#ifndef SRC_RADIO_REG_MAP_H_
#define SRC_RADIO_REG_MAP_H_

#define 	WNR							1<<7
/*
 * read mode addresses
 */
#define 	FIFO_IO_R					0x00
#define 	OP_MODE_R					0x01
#define 	RF_FREQ_MSB_R				0x06
#define 	RF_FREQ_MIB_R				0x07
#define 	RF_FREQ_LSB_R				0x08
#define 	PA_CONFIG_R					0x09
#define 	PA_RAMP_R					0x0A
#define 	LNA_R						0x0C
#define 	FIFO_ADR_PTR_R				0x0D
#define 	FIFO_TX_BASE_ADR_R			0x0E
#define 	FIFO_RX_BASE_ADR_R			0x0F
#define 	FIFO_RX_CUR_ADR_R			0x10
#define 	IRQ_FLAG_MSK_R				0x11
#define 	IRQ_FLAGS_R					0x12
#define 	RX_N_BYTES_R				0x13
#define 	MODEM_STATUS_R				0x18
#define 	PKT_SNR_VAL_R				0x19
#define 	RSSI_VAL_R					0x1B
#define 	MODEM_CONFIG1_R				0x1D
#define 	MODEM_CONFIG2_R				0x1E
#define 	SYM_TIMEOUT_R				0x1F
#define 	PREAMBLE_MSB_R				0x20
#define 	PREAMBLE_LSB_R				0x21
#define 	PAYLOAD_LENGTH_R			0x22
#define 	MAX_PAYLOAD_LEN_R			0x23
#define 	FREQ_ERR_MSB_R				0x28
#define 	FREQ_ERR_MIB_R				0x29
#define 	FREQ_ERR_LSB_R				0x2A
#define 	INVERT_IQ_R					0x33
#define 	SYNCH_WORD_R				0x39
#define		DIO_MAPPING_1_R				0x40

#define 	VERSION						0x42		//device version should be 0x22...

/*
 * write mode addresses
 */
#define 	FIFO_IO_W					(FIFO_IO_R | WNR)
#define 	OP_MODE_W					(OP_MODE_R | WNR)		//0x81
#define 	RF_FREQ_MSB_W				(RF_FREQ_MSB_R | WNR)
#define 	RF_FREQ_MIB_W				(RF_FREQ_MIB_R | WNR)
#define 	RF_FREQ_LSB_W				(RF_FREQ_LSB_R | WNR)
#define 	PA_CONFIG_W					(PA_CONFIG_R | WNR)
#define 	PA_RAMP_W					(PA_RAMP_R | WNR)		//0x8A
#define 	LNA_W						(LNA_R | WNR)
#define 	FIFO_ADR_PTR_W				(FIFO_ADR_PTR_R | WNR)
#define 	FIFO_TX_BASE_ADR_W			(FIFO_TX_BASE_ADR_R | WNR)
#define 	FIFO_RX_BASE_ADR_W			(FIFO_RX_BASE_ADR_R | WNR)
#define 	FIFO_RX_CUR_ADR_W			(FIFO_RX_CUR_ADR_R | WNR)
#define 	IRQ_FLAG_MSK_W				(IRQ_FLAG_MSK_R | WNR)
#define 	IRQ_FLAGS_W					(IRQ_FLAGS_R | WNR)
#define 	RX_N_BYTES_W				(RX_N_BYTES_R | WNR)
#define 	MODEM_STATUS_W				(MODEM_STATUS_R | WNR)
#define 	PKT_SNR_VAL_W				(PKT_SNR_VAL_R | WNR)
#define 	RSSI_VAL_W					(RSSI_VAL_R | WNR)
#define 	MODEM_CONFIG1_W				(MODEM_CONFIG1_R | WNR)	//0x9D
#define 	MODEM_CONFIG2_W				(MODEM_CONFIG2_R | WNR)	//0x9E
#define 	SYM_TIMEOUT_W				(SYM_TIMEOUT_R | WNR)
#define 	PREAMBLE_MSB_W				(PREAMBLE_MSB_R | WNR)
#define 	PREAMBLE_LSB_W				(PREAMBLE_LSB_R | WNR)
#define 	PAYLOAD_LENGTH_W			(PAYLOAD_LENGTH_R | WNR)
#define 	MAX_PAYLOAD_LEN_W			(MAX_PAYLOAD_LEN_R | WNR)
#define 	FREQ_ERR_MSB_W				(FREQ_ERR_MSB_R | WNR)
#define 	FREQ_ERR_MIB_W				(FREQ_ERR_MIB_R | WNR)
#define 	FREQ_ERR_LSB_W				(FREQ_ERR_LSB_R | WNR)
#define 	INVERT_IQ_W					(INVERT_IQ_R | WNR)
#define 	SYNCH_WORD_W				(SYNCH_WORD_R | WNR)			//0xB9
#define		DIO_MAPPING_1_W				(DIO_MAPPING_1_R | WNR)


#endif /* SRC_RADIO_REG_MAP_H_ */
