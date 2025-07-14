/*
 * Raspberry_Pi.h
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#ifndef INC_RASPBERRY_PI_H_
#define INC_RASPBERRY_PI_H_

#include "stm32f4xx_hal.h"
#include "Orientation_of_the_system.h"
#include "math.h"

typedef struct
{
  UART_HandleTypeDef *Chanal;
  unsigned char transmitting;

  unsigned char Rx_data[21];
  unsigned char Tx_data[13];

  float Azimuth, Elevation;
  unsigned char Fire_mode;
} Target_data;

extern Target_data Target;

void Data_parsing(void);
void Message_formation(void);

#endif /* INC_RASPBERRY_PI_H_ */
