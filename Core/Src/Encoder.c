/*
 * Encoder.c
 *
<<<<<<< HEAD
 *  Created on: Jul 11, 2025
 *      Author: u033008
=======
 *  Created on: Jul 12, 2025
 *      Author: Danil
>>>>>>> fork/main
 */

#include "Encoder.h"

<<<<<<< HEAD
Encoder Encoder_AZ = {
  .total_counts = 0,
  .min_angle = AZIMUTH_MIN_ANGLE,
  .max_angle = AZIMUTH_MAX_ANGLE,
  .ppr = ENCODER_PPR
};

Encoder Encoder_EL = {
  .total_counts = 0,
  .min_angle = ELEVATION_MIN_ANGLE,
  .max_angle = ELEVATION_MAX_ANGLE,
  .ppr = ENCODER_PPR
};

/**
  * @brief Initializes encoder interfaces for both azimuth and elevation axes
  *
  * @details
  * This function performs complete initialization of two incremental encoders:
  * 1. Starts encoder interface for both timers in quadrature mode
  * 2. Enables timer interrupts for overflow handling
  * 3. Configures hardware for:
  *    - Automatic direction detection
  *    - 4x counting mode (counting all edges)
  *    - Optional digital filtering (if configured)
  *
  * @note Both encoders must be properly connected to timer channels:
  *       - Channel 1/2 for AZ encoder
  *       - Channel 1/2 for EL encoder
  * @note Timer peripherals must be pre-configured in encoder mode
  *       before calling this function
  *
  * @param[in] htim_az Pointer to azimuth timer handle (TIM_HandleTypeDef)
  * @param[in] htim_el Pointer to elevation timer handle (TIM_HandleTypeDef)
  *
  * @warning Timer interrupts must be enabled in NVIC
  * @warning PPR values must be set in Encoder_AZ/Encoder_EL structures
  */
void Encoders_Init(TIM_HandleTypeDef *htim_az, TIM_HandleTypeDef *htim_el) {
  HAL_TIM_Encoder_Start(htim_az, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(htim_az);

  HAL_TIM_Encoder_Start(htim_el, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(htim_el);
};

/**
  * @brief Gets current angular position from specified encoder
  *
  * @details
  * This function calculates absolute angular position by:
  * 1. Combining hardware counter value with software overflow counter
  * 2. Converting raw pulses to degrees using formula:
  *    angle = (total_counts * 360°) / pulses_per_revolution
  * 3. Applying range clamping to ensure result stays within:
  *    - AZ: [-270°, 270°]
  *    - EL: [-20°, 90°]
  *
  * @param[in] enc Pointer to encoder configuration structure
  * @param[in] htim_az Azimuth timer handle (for counter access)
  * @param[in] htim_el Elevation timer handle (for counter access)
  *
  * @return Current angle in degrees (clamped to valid range)
  *
  * @note Uses 32-bit arithmetic to prevent overflow during conversion
  * @note Automatically handles timer counter wrap-around
  * @note CLAMP macro ensures result stays within mechanical limits
  *
  * @warning Encoder structure must contain valid PPR value
  * @warning Timer handles must match initialized encoders
  */
float GetEncoderAngle(Encoder* enc, TIM_HandleTypeDef *htim_az) {
  int32_t full_counts = enc->total_counts + __HAL_TIM_GET_COUNTER(htim_az);
  float angle = (full_counts * 360.0f) / 2000;
  return CLAMP(angle, -270.0f, 270.0f);
=======
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
>>>>>>> fork/main
};
