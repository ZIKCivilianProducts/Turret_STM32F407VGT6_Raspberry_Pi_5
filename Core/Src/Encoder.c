/*
 * Encoder.c
 *
 *  Created on: Jul 12, 2025
 *      Author: Danil
 */

#include "Encoder.h"

Encoder Positional_encoder_AZ = {.pulses_per_revolution = 8000}; // 2000 * 4
Encoder Positional_encoder_EL = {.pulses_per_revolution = 8000};

void Initialization_of_positioning_encoders(void) {
  HAL_StatusTypeDef Confirmation_of_initialization;
  do { // AZ
    Confirmation_of_initialization = HAL_TIM_Encoder_Start(Positional_encoder_AZ.Timer, TIM_CHANNEL_ALL);
    // Помигать светодиодом
  } while (!Confirmation_of_initialization);
  do { // EL
    Confirmation_of_initialization = HAL_TIM_Encoder_Start(Positional_encoder_EL.Timer, TIM_CHANNEL_ALL);
    // Помигать светодиодом
  } while (!Confirmation_of_initialization);
};

//void Reading_the_system_position(Encoder *Positional_encoder_xx, Systema *Systema_xx) {
//  int Counter_value = Positional_encoder_xx->Timer->Instance->CNT;
//  float Angular = (float)Counter_value * 360.0f / (float)Positional_encoder_xx->pulses_per_revolution;
//  Angular = CLAMP(Angular, Systema_xx->Settings.Minimum_angular, Systema_xx->Settings.Maximum_angular);
//  Systema_xx->Status.Angular = Angular;
//};

void Reading_the_system_position(Encoder *Positional_encoder_xx, Systema *Systema_xx) {
  // Использовать int32_t для обработки переполнения
  static uint32_t last_count_az = 0;
  static uint32_t last_count_el = 0;

  int32_t current_count = (int32_t)Positional_encoder_xx->Timer->Instance->CNT;

  float Angular = 0;

  // Обработка переполнения 32-битного счетчика
  if (Positional_encoder_xx->Timer == Positional_encoder_AZ.Timer) { // Для TIM5 (32-bit)
    static int32_t accumulated_az = 0;
    int32_t delta = current_count - last_count_az;

    // Коррекция при переполнении
    if (delta < -0x7FFFFFFF) delta += 0xFFFFFFFF;
    else if (delta > 0x7FFFFFFF) delta -= 0xFFFFFFFF;

    accumulated_az += delta;
    last_count_az = current_count;
    Angular = (float)accumulated_az * 360.0f / (float)Positional_encoder_xx->pulses_per_revolution;
  };
  if (Positional_encoder_xx->Timer == Positional_encoder_EL.Timer) { // Для TIM4 (16-bit)
    static int32_t accumulated_el = 0;
    int32_t delta = current_count - last_count_el;

    // Коррекция при переполнении
    if (delta < -0x7FFFFFFF) delta += 0xFFFFFFFF;
    else if (delta > 0x7FFFFFFF) delta -= 0xFFFFFFFF;

    accumulated_el += delta;
    last_count_el = current_count;
    Angular = (float)accumulated_el * 360.0f / (float)Positional_encoder_xx->pulses_per_revolution;
  };

  Systema_xx->Status.Angular = Angular;
};
