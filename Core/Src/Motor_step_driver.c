/*
 * Motor_step_driver.c
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#include <Motor_step_driver.h>

Motor Motor_AZ = {
<<<<<<< HEAD
  .Config = {
    .GPIO = {
      .DIR_port = GPIOE, .DIR_pin = GPIO_PIN_8,
      .ENA_port = GPIOE, .ENA_pin = GPIO_PIN_11
    },
    .PWM = {
      .Maximum_frequency = 50000,
      .Minimum_frequency = 5000,
      .Increment_frequency = 500
    },
    .Angular = {
      .Maximum_angular = 270.0f,
      .Minimum_angular = -270.0f,
      .Deviation = 1.0f,
      .Guidance_accuracy = 1.0f
    },
    .Alfa = 0.01f
  },
  .Status = {
    .Angular = 0.0f,
    .Frequency = 0,
    .Moving = 0
=======
  .Pins = {
    .DIR_port = GPIOE, .DIR_pin = GPIO_PIN_8,
    .ENA_port = GPIOE, .ENA_pin = GPIO_PIN_11
  },
  .PWM = {
    .Maximum_frequency = 200000,
    .Minimum_frequency = 10000,
    .Increment_frequency = 10000
>>>>>>> fork/main
  }
};

Motor Motor_EL = {
<<<<<<< HEAD
  .Config = {
    .GPIO = {
      .DIR_port = GPIOE, .DIR_pin = GPIO_PIN_9,
      .ENA_port = GPIOE, .ENA_pin = GPIO_PIN_12
    },
    .PWM = {
      .Maximum_frequency = 50000,
      .Minimum_frequency = 5000,
      .Increment_frequency = 500
    },
    .Angular = {
      .Maximum_angular = 90.0f,
      .Minimum_angular = -20.0f,
      .Deviation = 1.0f,
      .Guidance_accuracy = 1.0f
    },
    .Alfa = 0.01f
  },
  .Status = {
    .Angular = 0.0f,
    .Frequency = 0,
    .Moving = 0
=======
  .Pins = {
    .DIR_port = GPIOE, .DIR_pin = GPIO_PIN_9,
    .ENA_port = GPIOE, .ENA_pin = GPIO_PIN_12
  },
  .PWM = {
    .Maximum_frequency = 200000,
    .Minimum_frequency = 10000,
    .Increment_frequency = 10000
>>>>>>> fork/main
  }
};

/**
  * @brief Установка частоты ШИМ для управления двигателем
  * @param Motor_xx Указатель на структуру управления двигателем
  * @param frequency Желаемая частота ШИМ в герцах
  * @return None
  *
  * @details
  * Функция рассчитывает и устанавливает параметры ШИМ-сигнала:
  * 1. Вычисляет период и коэффициент деления для заданной частоты:
  *    - Использует частоту PCLK1 (APB1) в качестве базовой
  *    - Автоматически подбирает предделитель при переполнении периода
  * 2. Останавливает текущую генерацию ШИМ
  * 3. Настраивает регистры таймера:
  *    - PSC (предделитель)
  *    - ARR (период)
  *    - CCR1 (скважность 50%)
  * 4. Запускает ШИМ с новыми параметрами
  * 5. Обновляет статус частоты в соответствующей системе (AZ/EL)
  *
  * @note Автоматически обрабатывает случай слишком низких частот
  * @warning При слишком низкой частоте возможна потеря точности
  * @note Коэффициент 2 в расчетах учитывает удвоение частоты APB1
  * @note По умолчанию устанавливается 50% скважность (CCR1 = ARR/2)
  *
  * @sideeffect
  * - Модифицирует регистры таймера
  * - Обновляет статус системы
  * - Временно прерывает генерацию ШИМ
  */
void Setting_the_pulse_frequency(Motor *Motor_xx, unsigned int frequency) {
  unsigned int prescaler = 0;
  unsigned int period = (HAL_RCC_GetPCLK1Freq() * 2 / frequency) - 1;

  while (period > 0xFFFF) {
    prescaler++;
    period = (HAL_RCC_GetPCLK1Freq() * 2 / (frequency * (prescaler + 1))) - 1;
  }

  HAL_TIM_PWM_Stop(Motor_xx->PWM.Timer, Motor_xx->PWM.Channel);
  Motor_xx->PWM.Timer->Instance->PSC = prescaler;
  Motor_xx->PWM.Timer->Instance->ARR = period;
  Motor_xx->PWM.Timer->Instance->CCR1 = period / 2;
  HAL_TIM_PWM_Start(Motor_xx->PWM.Timer, Motor_xx->PWM.Channel);

  if (Motor_xx == &Motor_AZ) {
    Systema_AZ.Status.Frequency = frequency;
  }
  else {
    Systema_EL.Status.Frequency = frequency;
  };
};

/**
  * @brief Запуск двигателя с безопасными начальными параметрами
  * @param Motor_xx Указатель на структуру управления двигателем
  * @return None
  *
  * @details
  * Функция выполняет плавный запуск двигателя в следующей последовательности:
  * 1. Устанавливает минимальную частоту ШИМ из конфигурации двигателя
  * 2. Определяет начальное направление вращения на основе:
  *    - Типа двигателя (AZ/EL)
  *    - Текущего углового положения
  *    - Предопределенных направлений (Left/Right/Up/Down)
  * 3. Включает питание драйвера (перевод ENA в Work режим)
  *
  * @note Используется для безопасного старта двигателя
  * @warning Требует предварительной инициализации всех параметров
  * @note Направление выбирается автоматически в зависимости от положения
  * @note Всегда начинается с минимальной частоты для плавного пуска
  *
  * @sideeffect
  * - Активирует ШИМ-генерацию
  * - Устанавливает направление вращения
  * - Включает питание двигателя
  * - Может изменить текущее положение системы
  */
void Turning_on_the_engine(Motor *Motor_xx, GPIO_PinState direction_of_rotation) {
  Setting_the_pulse_frequency(Motor_xx, Motor_xx->PWM.Minimum_frequency);
  HAL_GPIO_WritePin(Motor_xx->Pins.DIR_port, Motor_xx->Pins.DIR_pin, direction_of_rotation);
  HAL_GPIO_WritePin(Motor_xx->Pins.ENA_port, Motor_xx->Pins.ENA_pin, Work);
  if (Motor_xx == &Motor_AZ) {Systema_AZ.Status.Moving = 1;}
  else {Systema_EL.Status.Moving = 1;};
};

/**
  * @brief Безопасное аварийное отключение двигателя
  * @param Motor_xx Указатель на структуру управления двигателем
  * @return None
  *
  * @details
  * Выполняет корректное отключение двигателя в защитной последовательности:
  * 1. Снижает частоту ШИМ до минимального значения (плавное торможение)
  * 2. Полностью останавливает ШИМ-генерацию на соответствующем канале таймера
  * 3. Отключает питание драйвера (перевод в Sleep режим)
  * 4. Обновляет статус движения в соответствующей системе (AZ/EL)
  *
  * @note Оптимальная последовательность для безопасного останова:
  *       Частота→ШИМ→Питание→Статус
  * @warning Требует корректной настройки Sleep режима (GPIO_PIN_SET/RESET)
  * @note Минимальная частота берется из конфигурации PWM структуры
  * @note Автоматически определяет систему (AZ/EL) через сравнение указателей
  *
  * @sideeffect
  * - Полностью останавливает двигатель
  * - Сбрасывает статус движения
  * - Освобождает ресурсы таймера
  * - Переводит драйвер в энергосберегающий режим
  */
void Engine_shutdown(Motor *Motor_xx) {
  Setting_the_pulse_frequency(Motor_xx, Motor_xx->PWM.Minimum_frequency);
  HAL_TIM_PWM_Stop(Motor_xx->PWM.Timer, Motor_xx->PWM.Channel);
  HAL_GPIO_WritePin(Motor_xx->Pins.ENA_port, Motor_xx->Pins.ENA_pin, Sleep);
  if (Motor_xx == &Motor_AZ) {Systema_AZ.Status.Moving = 0;}
  else {Systema_EL.Status.Moving = 0;};
};

/**
  * @brief Возврат системы в рабочую зону
  * @param None
  * @return None
  *
  * @details
  * Функция выполняет безопасный возврат двигателей в рабочую зону:
  * 1. Аварийно отключает оба двигателя (AZ и EL)
  * 2. Сбрасывает флаги движения для обеих осей
  * 3. Проверяет возможность восстановления рабочей зоны для оси AZ
  * 4. Если возможно - включает двигатель AZ:
  *    - Устанавливает направление от границы (в зависимости от текущего положения)
  *    - Дает 100мс импульс движения
  *    - Корректно отключает двигатель
  * 5. Если ось AZ недоступна - повторяет процедуру для оси EL
  *
  * @note Использует кратковременные управляющие импульсы (500мс)
  * @warning Направления Right/Left/Up/Down должны быть корректно определены
  * @note Автоматически определяет направление для выхода из граничного положения
  * @note Приоритет восстановления отдается оси AZ
  *
  * @sideeffect
  * - Может временно активировать один из двигателей
  * - Модифицирует состояние обеих систем управления
  * - Изменяет текущее положение двигателей
  */
void Return_to_the_workspace(void) {
  HAL_GPIO_WritePin(Motor_AZ.Pins.ENA_port, Motor_AZ.Pins.ENA_pin, Sleep);
  Systema_AZ.Status.Moving = 0;
  HAL_GPIO_WritePin(Motor_EL.Pins.ENA_port, Motor_EL.Pins.ENA_pin, Sleep);
  Systema_EL.Status.Moving = 0;

  while (Checking_the_workspace(&Systema_AZ) != HAL_OK || Checking_the_workspace(&Systema_EL) != HAL_OK) {
    if (Checking_the_workspace(&Systema_AZ) == HAL_OK) {
      Turning_on_the_engine(&Motor_AZ, Systema_AZ.Status.Angular > 0 ? Right : Left);
      HAL_Delay(250);
      Engine_shutdown(&Motor_AZ);
    };
    if (Checking_the_workspace(&Systema_EL) == HAL_OK) {
      Turning_on_the_engine(&Motor_EL, Systema_EL.Status.Angular > 0 ? Down : Up);
      HAL_Delay(250);
      Engine_shutdown(&Motor_EL);
    };
  };
};

/**
  * @brief Плавное увеличение частоты ШИМ для двигателя
  * @param Motor_xx Указатель на структуру управления двигателем
  * @return None
  *
  * @details
  * Функция выполняет безопасное увеличение частоты ШИМ-сигнала:
  * 1. Проверяет, что текущая частота ниже максимально допустимой
  * 2. Рассчитывает новую частоту:
  *    - Увеличивает текущее значение на заданный инкремент
  *    - Ограничивает максимальным значением через MIN()
  * 3. Устанавливает новую частоту через Setting_the_pulse_frequency()
  * 4. Выдерживает паузу 100 мс для стабилизации системы
  *
  * @note Автоматически определяет систему (AZ/EL) по указателю
  * @warning Частота не превысит Maximum_frequency из конфигурации PWM
  * @note Использует защищенное изменение частоты с задержкой
  * @note Величина шага задается в Increment_frequency
  *
  * @sideeffect
  * - Изменяет частоту ШИМ-генерации
  * - Вносит задержку в работу системы
  * - Обновляет статус частоты в соответствующей системе
  */
void Increasing_the_pulse_frequency(Motor *Motor_xx) {
  if ((Motor_xx == &Motor_AZ ? Systema_AZ.Status.Frequency : Systema_EL.Status.Frequency) < Motor_xx->PWM.Maximum_frequency) {
    unsigned int frequency = (Motor_xx == &Motor_AZ ? Systema_AZ.Status.Frequency : Systema_EL.Status.Frequency) + Motor_xx->PWM.Increment_frequency;
    frequency = MIN(frequency, Motor_xx->PWM.Maximum_frequency);
    Setting_the_pulse_frequency(Motor_xx ,frequency);
    HAL_Delay(100);
  }
};
