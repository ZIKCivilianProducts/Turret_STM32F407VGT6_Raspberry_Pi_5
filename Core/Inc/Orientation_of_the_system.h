/*
 * Orientation_of_the_system.h
 *
 *  Created on: Jul 11, 2025
 *      Author: Danil
 */

#ifndef INC_ORIENTATION_OF_THE_SYSTEM_H_
#define INC_ORIENTATION_OF_THE_SYSTEM_H_

#include "stm32f4xx_hal.h"
#include "Ternary_macros.h"

typedef struct
{
  float Maximum_angular;
  float Minimum_angular;

  float Aiming_accuracy; // Точность_прицелывания
  float Off_limits; // Отступ_от_границ
} Angular_settings;

typedef struct // Status
{
  uint32_t Frequency;
  uint8_t Moving;

  float Angular;
} System_status;

typedef struct{
	Angular_settings Settings;
	System_status Status;
} Systema;

extern Systema Systema_AZ;
extern Systema Systema_EL;

HAL_StatusTypeDef Checking_the_workspace(Systema *Systema_xx);

#endif /* INC_ORIENTATION_OF_THE_SYSTEM_H_ */
