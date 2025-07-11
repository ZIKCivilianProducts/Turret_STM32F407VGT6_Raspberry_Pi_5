/*
 * Structur_STM.c
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#include <Motor_step_driver.h>

Motor Motor_AZ = {
  .Config = {
    .GPIO = {
      .DIR_port = GPIOE, .DIR_pin = GPIO_PIN_8,
      .ENA_port = GPIOE, .ENA_pin = GPIO_PIN_11
    },
    .PWM = {
      .Maximum_frequency = 200000,
      .Minimum_frequency = 20000,
      .Increment_frequency = 10000
    },
    .Angular = {
      .Maximum_angular = 270.0f,
      .Minimum_angular = -270.0f,
      .Deviation = 50.0f,
      .Guidance_accuracy = 5.0f
    },
    .Alfa = 0.01f
  },
  .Status = {
    .Angular = 0.0f,
    .Frequency = 5000,
    .Moving = 0
  }
};

Motor Motor_EL = {
  .Config = {
    .GPIO = {
      .DIR_port = GPIOE, .DIR_pin = GPIO_PIN_9,
      .ENA_port = GPIOE, .ENA_pin = GPIO_PIN_12
    },
    .PWM = {
      .Maximum_frequency = 200000,
      .Minimum_frequency = 20000,
      .Increment_frequency = 10000
    },
    .Angular = {
      .Maximum_angular = 90.0f,
      .Minimum_angular = -20.0f,
      .Deviation = 10.0f,
      .Guidance_accuracy = 5.0f
    },
    .Alfa = 0.01f
  },
  .Status = {
    .Angular = 0.0f,
    .Frequency = 5000,
    .Moving = 0
  }
};

/**
  * @brief First operating mode controller for motor movement
  * @param Motor_xx Pointer to Motor structure to control
  * @param Angular Target angular position/setpoint (determines direction)
  * @retval None
  *
  * @details
  * This function implements the basic motor control logic:
  * 1. Determines movement direction based on:
  *    - Timer association (AZ/EL motor identification)
  *    - Angular setpoint sign (positive/negative)
  *    - Predefined direction constants (Left/Right/Up/Down)
  * 2. Executes one of three control actions:
  *    - Starts motor if not moving (with proper direction)
  *    - Increases frequency if already moving
  *    - Stops motor if Angular setpoint is zero
  *
  * @note Direction logic uses ternary operators for compact AZ/EL differentiation
  * @note Start_motor() called only when motor is stationary
  * @note Up_fequency() used for gradual acceleration
  * @note Stop condition triggered by zero angular setpoint
  *
  * @warning Ensure Left/Right/Up/Down constants are properly defined as GPIO_PinState values
  * @warning Timer comparison assumes proper Motor_AZ global variable definition
  */
void First_mode (Motor *Motor_xx, float Angular) {
  GPIO_PinState roter =
    Motor_xx->Config.PWM.Timer == Motor_AZ.Config.PWM.Timer ?
    (Angular > 0 ? Left : Right) :
    (Angular > 0 ? Up : Down);
  if (Angular != 0) {
    if (!Motor_xx->Status.Moving) {
      Start_motor(Motor_xx, roter);
    }
    else {
      Up_fequency(Motor_xx);
    };
  }
  else {
    Stop_motor(Motor_xx);
  };
};

/**
  * @brief Execute border avoidance maneuver for motor
  * @param Motor_xx Pointer to the motor structure needing border avoidance
  * @param roter Direction to move away from border (GPIO_PIN_SET or GPIO_PIN_RESET)
  * @retval None
  *
  * @details
  * This safety function performs a controlled border avoidance sequence:
  * 1. Sets the specified movement direction
  * 2. Engages minimum PWM frequency for safe, slow movement
  * 3. Temporarily enables the AZ motor (100ms pulse) to create mechanical clearance
  * 4. After delay, disables the AZ motor
  * 5. Updates motor status flag (Moving = 0)
  *
  * @note Uses minimum PWM frequency to prevent mechanical stress
  * @warning The 100ms AZ motor pulse is critical for mechanical clearance
  * @note Work/Sleep states should be properly defined (typically GPIO_PIN_SET/RESET)
  * @note This is an emergency stop function, not for normal operation
  *
  * @sideeffect
  * - Generates a brief 100ms activation pulse on Motor_AZ
  * - Modifies the AZ motor enable state temporarily
  */
void Moving_away_from_borders(Motor *Motor_xx, GPIO_PinState roter) {
  HAL_GPIO_WritePin(Motor_xx->Config.GPIO.DIR_port, Motor_xx->Config.GPIO.DIR_pin, roter);
  Set_PWM_frequency(Motor_xx, Motor_xx->Config.PWM.Minimum_frequency);
  HAL_GPIO_WritePin(Motor_AZ.Config.GPIO.ENA_port, Motor_AZ.Config.GPIO.ENA_pin, Work);

  HAL_Delay(100);

  HAL_GPIO_WritePin(Motor_AZ.Config.GPIO.ENA_port, Motor_AZ.Config.GPIO.ENA_pin, Sleep);
  Motor_xx->Status.Moving = 0;
};

/**
  * @brief Second operating mode with position control and border safety
  * @param Angular Target angular position (setpoint)
  * @param Motor_xx Pointer to Motor structure to control
  * @retval None
  *
  * @details
  * This advanced control mode implements:
  * 1. Position error calculation (Difference = Setpoint - Current)
  * 2. Direction determination based on error sign (Left/Right)
  * 3. Three-tier control logic:
  *    a) If outside working area: Emergency border avoidance (reverse direction)
  *    b) If error > Guidance_accuracy:
  *       - Start motor if stationary
  *       - Increase speed if already moving
  *    c) If within accuracy: Stop motor
  *
  * @note Uses Working_area() for safety boundary checks
  * @note Implements deadband using Guidance_accuracy parameter
  * @note Automatically reverses direction for border escape
  * @note fabsf() used for absolute error comparison
  *
  * @warning Left/Right must be properly defined as GPIO_PinState values
  * @warning Guidance_accuracy should be set appropriately for application
  * @warning Border avoidance uses reverse direction (-roter)
  */
void Second_mode(float Angular, Motor *Motor_xx){
	float Difference = Angular - Motor_xx->Status.Angular;
	GPIO_PinState roter = Difference > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;

	if (Working_area(Motor_xx))	{
	  if (fabsf(Difference) > Motor_xx->Config.Angular.Guidance_accuracy) {
	    if (!Motor_xx->Status.Moving) {
	      Start_motor(Motor_xx, roter);
	    }
	    else {
	      Up_fequency(Motor_xx);
	    };
	  }
	  else {
	    Stop_motor(Motor_xx);
	  };
	}
	else {
	  Moving_away_from_borders(Motor_xx, -roter);
	};
};

/**
  * @brief Set PWM frequency for motor control
  * @param Motor_xx Pointer to Motor structure containing PWM configuration
  * @param freq Desired PWM frequency (Hz)
  * @retval None
  *
  * @details
  * This function configures the timer to generate a PWM signal with the specified frequency.
  * The duty cycle is set to 50% by default (CCR1 = ARR / 2).
  *
  * The function performs the following steps:
  * 1. Calculates the required timer period (ARR) and prescaler (PSC) based on the input frequency.
  * 2. Adjusts prescaler and period if the required period exceeds the 16-bit limit (0xFFFF).
  * 3. Stops the PWM generation before reconfiguration.
  * 4. Updates the timer registers (PSC, ARR, CCR1).
  * 5. Restarts PWM generation with the new settings.
  *
  * @note The function assumes the timer clock source is PCLK1 (APB1) and uses its frequency for calculations.
  * @note The duty cycle is fixed at 50%. Modify CCR1 manually if a different duty cycle is needed.
  */
void Set_PWM_frequency(Motor *Motor_xx, uint32_t freq)
{
  unsigned int prescaler = 0;
  unsigned int period = (HAL_RCC_GetPCLK1Freq() * 2 / freq) - 1;

  while (period > 0xFFFF) {
    prescaler++;
    period = (HAL_RCC_GetPCLK1Freq() * 2 / (freq * (prescaler + 1))) - 1;
  }

  HAL_TIM_PWM_Stop(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);
  Motor_xx->Config.PWM.Timer->Instance->PSC = prescaler;
  Motor_xx->Config.PWM.Timer->Instance->ARR = period;
  Motor_xx->Config.PWM.Timer->Instance->CCR1 = period / 2;
  HAL_TIM_PWM_Start(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);

  Motor_xx->Status.Frequency = freq;
};

/**
  * @brief Start motor with initial parameters
  * @param Motor_xx Pointer to Motor structure containing configuration and status
  * @param roter Direction of rotation (GPIO_PIN_SET or GPIO_PIN_RESET)
  * @retval None
  *
  * @details
  * This function performs the following operations to start the motor:
  * 1. Sets PWM frequency to minimum configured value (soft start)
  * 2. Applies specified direction signal through GPIO
  * 3. Enables motor driver by setting ENA pin to Work state
  * 4. Updates motor status flag (Moving = 1)
  *
  * @note The function uses minimum PWM frequency for safe motor start
  * @note Work state should be defined as GPIO_PIN_SET or GPIO_PIN_RESET
  * @note Motor direction depends on driver wiring and roter parameter interpretation
  */
void Start_motor(Motor *Motor_xx, GPIO_PinState roter)
{
  Set_PWM_frequency(Motor_xx, Motor_xx->Config.PWM.Minimum_frequency);
  HAL_GPIO_WritePin(Motor_xx->Config.GPIO.DIR_port, Motor_xx->Config.GPIO.DIR_pin, roter);
  HAL_GPIO_WritePin(Motor_xx->Config.GPIO.ENA_port, Motor_xx->Config.GPIO.ENA_pin, Work);
  Motor_xx->Status.Moving = 1;
};

/**
  * @brief Stop motor safely and put it in sleep mode
  * @param Motor_xx Pointer to Motor structure containing configuration and status
  * @retval None
  *
  * @details
  * This function performs a complete motor stop sequence:
  * 1. Stops PWM generation on timer channel
  * 2. Disables motor driver by setting ENA pin to Sleep state
  * 3. Updates motor status flag (Moving = 0)
  *
  * @note The function provides complete motor shutdown
  * @note Sleep state should be defined as GPIO_PIN_SET or GPIO_PIN_RESET
  * @note Setting frequency to 0 before stopping PWM ensures smooth deceleration
  * @warning Ensure proper ENA pin state configuration to avoid unexpected motor behavior
  */
void Stop_motor(Motor *Motor_xx)
{
  HAL_TIM_PWM_Stop(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(Motor_xx->Config.GPIO.ENA_port, Motor_xx->Config.GPIO.ENA_pin, Sleep);
  Motor_xx->Status.Moving = 0;
};

/**
  * @brief Increase motor PWM frequency by configured increment
  * @param Motor_xx Pointer to Motor structure containing PWM configuration and status
  * @retval None
  *
  * @details
  * This function safely increases the motor's PWM frequency by following steps:
  * 1. Checks if current frequency is below maximum allowed limit
  * 2. Calculates new frequency by adding configured increment value
  * 3. Ensures new frequency doesn't exceed maximum limit using MIN macro
  * 4. Applies new frequency using Set_PWM_Frequency() function
  *
  * @note Frequency change occurs only if current frequency is below maximum limit
  * @note Actual increment step is determined by Config.PWM.Increment_frequency
  * @note Function uses safe frequency clamping to prevent exceeding limits
  */
void Up_fequency(Motor *Motor_xx)
{
  if (Motor_xx->Status.Frequency < Motor_xx->Config.PWM.Maximum_frequency) {
    unsigned int freq = Motor_xx->Status.Frequency + Motor_xx->Config.PWM.Increment_frequency;
    freq = MIN(freq, Motor_xx->Config.PWM.Maximum_frequency);
    Set_PWM_frequency(Motor_xx, freq);
    HAL_Delay(10);
  };
};

/**
  * @brief Check if motor's current angle is within working area boundaries
  * @param Motor_xx Pointer to Motor structure containing angular configuration and status
  * @retval char Returns 1 (true) if angle is within working area, 0 (false) otherwise
  *
  * @details
  * This function checks whether the motor's current angular position is within the
  * defined working area. The working area is calculated as:
  *
  * [Minimum_angular + Deviation, Maximum_angular - Deviation]
  *
  * Where:
  * - Minimum_angular: Lower mechanical limit
  * - Maximum_angular: Upper mechanical limit
  * - Deviation: Safety margin from mechanical limits
  *
  * @note The function returns a boolean value as char type (1/0)
  * @note The comparison is exclusive (angle must be strictly between bounds)
  */
char Working_area(Motor *Motor_xx) {
  float current_angle = Motor_xx->Status.Angular;

  float lower_bound = Motor_xx->Config.Angular.Minimum_angular + Motor_xx->Config.Angular.Deviation;;
  float upper_bound = Motor_xx->Config.Angular.Maximum_angular - Motor_xx->Config.Angular.Deviation;;

  return (current_angle > lower_bound) && (current_angle < upper_bound);
};
