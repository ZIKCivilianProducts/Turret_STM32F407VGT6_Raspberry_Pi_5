/*
 * Orientation_of_the_system.c
 *
 *  Created on: Jul 11, 2025
 *      Author: Danil
 */

#include "Orientation_of_the_system.h"

Systema Systema_AZ ={
  .Settings ={
    .Maximum_angular = 270.0f,
    .Minimum_angular = -270.0f,
    .Aiming_accuracy = 5.0f,
    .Off_limits = 5.0f
  },
  .Status ={
    .Frequency = 0,
    .Moving = 0,
    .Angular = 0.0f
  }
};

Systema Systema_EL ={
  .Settings ={
    .Maximum_angular = 900.0f,
    .Minimum_angular = -20.0f,
    .Aiming_accuracy = 5.0f,
    .Off_limits = 5.0f
  },
  .Status ={
    .Frequency = 0,
    .Moving = 0,
    .Angular = 0.0f
  }
};

HAL_StatusTypeDef Checking_the_workspace(Systema *Systema_xx) {
  float lower_limit = Systema_xx->Settings.Minimum_angular + Systema_xx->Settings.Off_limits;
  float upper_limit = Systema_xx->Settings.Maximum_angular - Systema_xx->Settings.Off_limits;
  float current_angle = CLAMP(Systema_xx->Status.Angular, lower_limit, upper_limit);
  return ((current_angle != lower_limit) && (current_angle != upper_limit)) ? HAL_OK : HAL_ERROR;
};
