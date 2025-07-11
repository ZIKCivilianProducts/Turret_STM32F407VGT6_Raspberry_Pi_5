/*
 * Transfer_to_Raspberry_Pi.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Danil
 */

#ifndef INC_TRANSFER_TO_RASPBERRY_PI_H_
#define INC_TRANSFER_TO_RASPBERRY_PI_H_

#include <Motor_step_driver.h>
#include <Raspberry_Pi.h>
#include <math.h>

void Transfer_to_raspberry_pi(UART_HandleTypeDef *huart, Target_data *Targit, Motor *Motor_az, Motor *Motor_el);

#endif /* INC_TRANSFER_TO_RASPBERRY_PI_H_ */
