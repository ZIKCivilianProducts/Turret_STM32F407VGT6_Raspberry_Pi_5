/*
 * Transfer_to_Raspberry_Pi.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Danil
 */

#include "Transfer_to_Raspberry_Pi.h"

void Transfer_to_raspberry_pi(UART_HandleTypeDef *huart, Target_data *Targit, Motor *Motor_az, Motor *Motor_el) {
  if (!Targit->transmitting) {
	size_t Size_Tx_UART = sizeof(Targit->Tx_data);

    uint16_t angular_az = (uint16_t)fabs(Motor_az->Status.Angular * 10);
    uint16_t angular_el = (uint16_t)fabs(Motor_el->Status.Angular * 10);

    Targit->Tx_data[1] = (Motor_az->Status.Angular >= 0) ? '1' : '0';
    Targit->Tx_data[2] = '0' + (angular_az / 1000) % 10;
    Targit->Tx_data[3] = '0' + (angular_az / 100) % 10;
    Targit->Tx_data[4] = '0' + (angular_az / 10) % 10;
    Targit->Tx_data[5] = '0' + (angular_az) % 10;

    Targit->Tx_data[6] = (Motor_el->Status.Angular >= 0) ? '1' : '0';
    Targit->Tx_data[7] = '0' +  (angular_el / 1000) % 10;
    Targit->Tx_data[8] = '0' +  (angular_el / 100) % 10;
    Targit->Tx_data[9] = '0' +  (angular_el / 10) % 10;
    Targit->Tx_data[10] = '0' + (angular_el) % 10;
    if (HAL_UART_Transmit_IT(huart, (uint8_t*)Targit->Tx_data, Size_Tx_UART) == HAL_OK)
    {
      Targit->transmitting = 1;
    };
  };
};
