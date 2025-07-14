/*
 * Proportional_integral_differentiating_regulator.h
 *
 *  Created on: Jul 11, 2025
 *      Author: u033008
 */

#ifndef INC_PROPORTIONAL_INTEGRAL_DIFFERENTIATING_REGULATOR_H_
#define INC_PROPORTIONAL_INTEGRAL_DIFFERENTIATING_REGULATOR_H_

typedef struct{
	float Proportionality;
	float Integration;
	float Differentiation;
} Gain_factors;

typedef struct{
	Gain_factors Factors;
} Proportional_integral_differentiating_regulator;

extern Proportional_integral_differentiating_regulator PID_AZ;
extern Proportional_integral_differentiating_regulator PID_EL;

#endif /* INC_PROPORTIONAL_INTEGRAL_DIFFERENTIATING_REGULATOR_H_ */
