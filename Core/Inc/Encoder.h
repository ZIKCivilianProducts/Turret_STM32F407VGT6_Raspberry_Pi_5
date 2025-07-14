/*
 * Encoder.h
 *
 *  Created on: Jul 11, 2025
 *      Author: u033008
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

/*
 * Инкрементальный энкодер имеет два выхода: A (CLK) и B (DT).
 * */

#include "stm32f4xx_hal.h"
#include "Ternary_macros.h"

#define ENCODER_PPR 2000 // Импульсов на оборот (PPR - Pulses Per Revolution)

#define AZIMUTH_MIN_ANGLE -270.0f
#define AZIMUTH_MAX_ANGLE 270.0f
#define AZIMUTH_MAX_COUNTS (int32_t)((ENCODER_PPR * AZIMUTH_MAX_ANGLE) / 360.0f)

#define ELEVATION_MIN_ANGLE -20.0f
#define ELEVATION_MAX_ANGLE 90.0f
#define ELEVATION_MAX_COUNTS (int32_t)((ENCODER_PPR * ELEVATION_MAX_ANGLE) / 360.0f)

typedef struct {
    unsigned int total_counts;
    float min_angle;
    float max_angle;
    unsigned int ppr;
} Encoder;

extern Encoder Encoder_AZ;
extern Encoder Encoder_EL;

void Encoders_Init(TIM_HandleTypeDef *htim_az, TIM_HandleTypeDef *htim_el);
float GetEncoderAngle(Encoder* enc, TIM_HandleTypeDef *htim_az);

#endif /* INC_ENCODER_H_ */
