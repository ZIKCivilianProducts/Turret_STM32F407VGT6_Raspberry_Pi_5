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

void Encoders_Init(TIM_HandleTypeDef *htim_az, TIM_HandleTypeDef *htim_el) {
  HAL_TIM_Encoder_Start(htim_az, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(htim_az);

  HAL_TIM_Encoder_Start(htim_el, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(htim_el);
};

float GetEncoderAngle(Encoder* enc, TIM_HandleTypeDef *htim_az, TIM_HandleTypeDef *htim_el) {
  int32_t full_counts = enc->total_counts + __HAL_TIM_GET_COUNTER(enc == &Encoder_AZ ? htim_az : htim_el);
  float angle = (full_counts * 360.0f) / enc->ppr;
  return CLAMP(angle, enc->min_angle, enc->max_angle);
}
