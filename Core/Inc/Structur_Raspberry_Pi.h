/*
 * Structur_Raspberry_Pi.h
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#ifndef INC_STRUCTUR_RASPBERRY_PI_H_
#define INC_STRUCTUR_RASPBERRY_PI_H_

#include "stm32f4xx_hal.h"

typedef struct // Target_data
{
	uint8_t Rx_data[21];
	uint8_t Tx_data[13];

	volatile uint8_t transmitting;

	float Azimuth, Elevation;

	uint8_t Fire_mode;
} Target_data;

extern Target_data Target;

#endif /* INC_STRUCTUR_RASPBERRY_PI_H_ */
