/*
 * time_manager.h
 *
 *  Created on: Apr 7, 2017
 *      Author: waseemh
 */

#ifndef SRC_TIME_MANAGER_H_
#define SRC_TIME_MANAGER_H_

#include "pinmap.h"
#include "em_gpio.h"
#include "em_emu.h"

#define 	TIME_MANAGER_ISR			GPIO_EVEN_IRQHandler
#define 	TIME_MANAGER_IRQn			GPIO_EVEN_IRQn
#define		BASIC_SYNCH_SECONDS			3
#define		ADVANCE_SYNCH_SECONDS		12

typedef enum{
	basic_sync=0,
	advance_sync
}time_manager_cmd_t;
/*
 * public variables
 */
/*
 * private functions
 */
/*
 * public functions
 */

void 				time_manager_init(void);
time_manager_cmd_t 	time_manager_get_cmd(void);
unsigned long 		time_manager_unixTimestamp(int year, int month, int day,
              	  	  int hour, int min, int sec);

#endif /* SRC_TIME_MANAGER_H_ */
