/*
 * app_manager.c
 *
 *  Created on: May 2, 2017
 *      Author: waseemh
 */


#include "app_manager.h"

		/*
		 * private variables
		 */
static			FATFS 				FatFs;
static 			nav_data_t			last_nav_data;
		/*
		 * public variables
		 */
const 			unsigned char  		rs232_tx_buf[64];

		/*
		 * private functions
		 */
bool file_sys_setup(uint16_t year,uint8_t month,uint8_t day, char buf[]){

	char 			filename[8]="00000000";
	bool			ret_flag=false;
	FIL  			f_pointer;
	FRESULT 		f_ret;

	/*
	 * FR_NOT_READY issue not solved with delay. Maybe send a command and wait...
	 * BUT only using f_open and not f_mount in this function works  :-)
	 * shutdown only once in init. NOT in this function
	 * Current Scenario: Current is 26mA on average but on write, goes to 46mA
	 * which is normal and as per logic. Function is almost done.
	 * Also works with remove and reinsert card
	 */
	sprintf(filename,"%4d%d%d.txt",year,month,day);
	f_ret = f_open(&f_pointer, filename, FA_WRITE | FA_OPEN_APPEND);
	if(f_ret==FR_OK){
    	f_ret = f_puts(buf,&f_pointer);
    	f_close(&f_pointer);
    	if(f_ret>0){
    		ret_flag=true;
    	}
    	else{
    		ret_flag=false;
    	}
    }
    else{
    	ret_flag=false;
		sprintf((char *)rs232_tx_buf,"f_open=%1d\n",f_ret);
		rs232_transmit_string((unsigned char *)rs232_tx_buf,strlen((const char *)rs232_tx_buf));
    	f_close(&f_pointer);
    }
    return ret_flag;
}
bool tbr_cmd_update_rgb_led(tbr_cmd_t tbr_cmd, time_t timestamp){
	bool 	ret_flag=false;

	ret_flag=tbr_send_cmd(tbr_cmd,timestamp);
	if(ret_flag){
		rgb_on(false,true,false);
		delay_ms(7);
	}
	else{
		rgb_on(true,false,false);
		delay_ms(7);
	}
	rgb_shutdown();
	return ret_flag;
}
void shift_buffer(unsigned char *buffer, uint8_t buffer_size, uint8_t shift_amount){
	int 			inner_loop_var=0;
	int 			outer_loop_var=0;

	for(outer_loop_var=0;outer_loop_var<shift_amount;outer_loop_var++){
		for (inner_loop_var = buffer_size; inner_loop_var > 0; inner_loop_var--){
			buffer[inner_loop_var]=buffer[inner_loop_var-1];
		}
	}
	return;
}
uint8_t	pack_radio_message(unsigned char *tbr_sn, uint8_t msg_type, uint8_t message_count, unsigned char *tx_buf){
	uint8_t			length=0;
	int 			inner_loop_var=0;
	length=strlen((char*)tx_buf);
		//shift buffer to add space in the beginning
	shift_buffer(tx_buf,240,4);
		//add SN and message type
	for (inner_loop_var = 0; inner_loop_var < 4; inner_loop_var++){
		tx_buf[inner_loop_var]=tbr_sn[inner_loop_var];
	}
	//strncat((char *)tx_buf, (char *) tbr_sn,6);
	//tx_buf[5]=(char)message_count;
	return (7+length);
}
void send_data_on_radio(unsigned char *tx_buf,uint8_t message_count){
	unsigned char	tbr_sn[7]="@01:";
	uint8_t			msg_type=0;
	uint8_t			msg_length_in_bytes=0;

	RFM_Init();
	msg_length_in_bytes=pack_radio_message(tbr_sn,msg_type,message_count,tx_buf);
	RFM_Send_Package(tx_buf,msg_length_in_bytes);
	RFM_off();
	return;
}
		/*
		 * public functions
		 */
bool app_manager_init(void){
	  const unsigned char  		rs232_tx_buf[64];
	  uint8_t					init_retry=0;
	  bool						temp_init_flag=false;

	  	  	  	  //basic initializations
	  rgb_init();
	  rs232_init();
	  rs232_enable();
	  delay_init();
	  	  	  	  //Turn on init. LED (blue)
	  rgb_on(false,false,true);
	  	  	  	  //GPS
	  init_retry=0;
	  do{
		  temp_init_flag= gps_init();
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"GPS INIT FAILED\n");
					rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
					rgb_shutdown();
					rgb_on(true,false,false);
					return 0;
			 }
	  	  }while(!temp_init_flag);
	sprintf((char *)rs232_tx_buf,"GPS Init. DONE\n");
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
				//LoRA
	init_retry=RFM_Init();
	RFM_off();
	sprintf((char *)rs232_tx_buf,"Radio is  init. in %2x mode\n",init_retry);
	rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
	init_retry=0;
				//SD card
	 do{
		 temp_init_flag=sd_card_init();
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"Init. SD Card Failed\n");
					rs232_transmit_string((unsigned char *)rs232_tx_buf,21);
					rgb_shutdown();
					rgb_on(true,false,false);
					return 0;
			 }
		 }while(!temp_init_flag);
	init_retry=0;
	 do{
		 temp_init_flag=(bool)f_mount(&FatFs,"", 0);
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"Mounting SD Card Failed\n");
					rs232_transmit_string((unsigned char *)rs232_tx_buf,24);
					sd_card_off();
					return false;
			 }
		 }while(temp_init_flag);
	sd_card_off();
				//TBR
	tbr_init();
	init_retry=0;
	 do{
		 temp_init_flag=	tbr_send_cmd(cmd_sn_req,(time_t) 0);
			 init_retry++;
			 if(init_retry>INIT_RETRIES){
					sprintf((char *)rs232_tx_buf,"TBR Missing....\n");
					rs232_transmit_string((unsigned char *)rs232_tx_buf,strlen((const char *)rs232_tx_buf));
					break;
			 }
		 }while(!temp_init_flag);
	 	 	 //Turn off init. LED
	 rgb_shutdown();
	 return true;
}

void app_manager_tbr_synch_msg(uint8_t  time_manager_cmd, nav_data_t nav_data){

	bool			temp_flag=false;
	int				tbr_msg_count=0;
	int				tbr_msg_length=0;
	char			tbr_msg_buf[ARRAY_MESSAGE_SIZE];
	////////////////Radio thing insh-A-ALLAH///////////////
	unsigned char	radio_buf[LORA_TX_BUFFER_SIZE]="insh A ALLAH basic msg";
	///////////////////////////////////////////////////////
	if(time_manager_cmd==0){
		temp_flag=tbr_cmd_update_rgb_led(cmd_basic_sync,(time_t)nav_data.gps_timestamp);
		sprintf((char *)rs232_tx_buf,"Basic Sync MSG:Flag=%d\t\n",temp_flag);
		rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		RFM_Send_Package(radio_buf,23);
	  }
	  else if (time_manager_cmd==1 && nav_data.valid==1 ){
		  temp_flag=tbr_cmd_update_rgb_led(cmd_advance_sync,(time_t)nav_data.gps_timestamp);
		  sprintf((char *)rs232_tx_buf,"Advance Synch MSG:Flag=%d\t TimeStamp=%ld\t",temp_flag,(time_t)nav_data.gps_timestamp);
		  rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		  tbr_msg_count=tbr_recv_msg((char *)tbr_msg_buf,&tbr_msg_length);
		  if(tbr_msg_count>0){
			temp_flag=file_sys_setup(nav_data.year,nav_data.month,nav_data.day,tbr_msg_buf);
			sprintf((char *)rs232_tx_buf,"\t\tFile Write Flag=%1d Length=%3d\t\n",temp_flag,tbr_msg_length);
			rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
			send_data_on_radio((unsigned char *)tbr_msg_buf,tbr_msg_count);
		  }
		  last_nav_data=nav_data;
	  }
	  else if (time_manager_cmd==1 && nav_data.valid==0){
		  sprintf((char *)rs232_tx_buf,"\tInvalid TimeStamp: Using Last TimeStamp\n");
		  rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		  temp_flag=tbr_cmd_update_rgb_led(cmd_advance_sync,(time_t)(last_nav_data.gps_timestamp+60));	//add exactly 60 seconds to last TimeStamp
		  sprintf((char *)rs232_tx_buf,"\tAdvance Synch MSG:Flag=%d",temp_flag);
		  rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		  tbr_msg_count=tbr_recv_msg((char *)tbr_msg_buf,&tbr_msg_length);
		  if(tbr_msg_count>0){
			  temp_flag=file_sys_setup(last_nav_data.year,last_nav_data.month,last_nav_data.day,tbr_msg_buf);
			  sprintf((char *)rs232_tx_buf,"\t\tFile Write Flag=%1d Length=%3d\t\n",temp_flag,tbr_msg_length);
			  rs232_transmit_string(rs232_tx_buf,strlen((const char *)rs232_tx_buf));
		  }
	  }
	  else{
		  ;
	  }
	return;
}

nav_data_t app_manager_get_nav_data(void){
	return gps_get_nav_data();
}
