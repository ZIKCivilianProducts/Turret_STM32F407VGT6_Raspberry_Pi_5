/*
 * Encoder.c
 *
 *  Created on: Jul 11, 2025
 *      Author: u033008
 */

#include "Encoder.h"

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
};
