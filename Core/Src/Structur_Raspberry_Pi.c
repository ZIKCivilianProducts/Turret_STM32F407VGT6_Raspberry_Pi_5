/*
 * Structur_Raspberry_Pi.c
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#include "Structur_Raspberry_Pi.h"

Target_data Target =
{
		.Azimuth = 0,
		.Elevation = 0,
		.Fire_mode = 0,
		.Rx_data = "Az+0000El-0000M0Fm\r\n\0",
		.Tx_data = "T0000000000\r\n",
		.transmitting = 0
};
