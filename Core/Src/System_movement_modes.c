/*
 * System_movement_modes.c
 *
 *  Created on: Jul 12, 2025
 *      Author: Danil
 */

#include "System_movement_modes.h"

/**
  * @brief Функция ручного управления двигателями
  * @param None
  * @return None
  *
  * @details
  * Осуществляет ручное управление двигателями на основе принятых команд:
  * 1. Для оси AZ (азимут):
  *    - Если Target.Rx_data[5] != '0' (команда движения):
  *      * Запускает двигатель, если он остановлен (Turning_on_the_engine)
  *      * Увеличивает частоту, если двигатель уже работает
  *    - Иначе останавливает двигатель (Engine_shutdown)
  * 2. Для оси EL (угол места):
  *    - Аналогичная логика по Target.Rx_data[12]
  *
  * @note Использует данные из буфера Target.Rx_data для управления
  * @warning Требует предварительного заполнения Rx_data
  * @note Для каждой оси обрабатывается отдельно
  * @note Поддерживает плавное увеличение скорости при непрерывном управлении
  *
  * @sideeffect
  * - Может запускать/останавливать двигатели
  * - Меняет частоту ШИМ
  * - Обновляет статусы движения в Systema_AZ/EL
  */
void Manual_operation(void) {
  if (Target.Rx_data[5] != '0') {
    if (!Systema_AZ.Status.Moving) {Turning_on_the_engine(&Motor_AZ, Target.Azimuth > 0 ? Left : Right);}
    else {Increasing_the_pulse_frequency(&Motor_AZ);};
  }
  else {Engine_shutdown(&Motor_AZ);};

  if (Target.Rx_data[12] != '0') {
    if (!Systema_EL.Status.Moving) {Turning_on_the_engine(&Motor_EL, Target.Elevation > 0 ? Up : Down);}
    else {Increasing_the_pulse_frequency(&Motor_EL);};
  }
  else {Engine_shutdown(&Motor_EL);};
};

/**
  * @brief Функция позиционирования по заданным углам
  * @param None
  * @return None
  *
  * @details
  * Осуществляет автоматическое позиционирование двигателей по целевым углам:
  * 1. Для каждой оси (AZ/EL) вычисляет разницу между текущим и целевым углом
  * 2. Если разница превышает минимальный порог (Minimum_angular):
  *    - При остановленном двигателе: запускает в нужном направлении
  *      (Left/Right для AZ, Up/Down для EL в зависимости от знака ошибки)
  *    - При работающем двигателе: увеличивает частоту ШИМ
  * 3. Если разница меньше порога - останавливает двигатель
  *
  * @note Использует разные направления для осей AZ и EL
  * @warning Требует предварительного задания Target.Azimuth/Elevation
  * @note Учитывает Minimum_angular как зону нечувствительности
  * @note Поддерживает плавное наращивание скорости при больших рассогласованиях
  *
  * @sideeffect
  * - Может запускать/останавливать двигатели
  * - Меняет частоту ШИМ
  * - Обновляет статусы движения
  * - Изменяет текущее положение системы
  */
void Pointing_at_an_angle(void) {
  float angle_difference_az = Systema_AZ.Status.Angular - Target.Azimuth;
  if (fabs(angle_difference_az) > Systema_AZ.Settings.Aiming_accuracy) {
    if (!Systema_AZ.Status.Moving) {Turning_on_the_engine(&Motor_AZ, angle_difference_az > 0 ? Left : Right);}
	else {Increasing_the_pulse_frequency(&Motor_AZ);};
  }
  else {Engine_shutdown(&Motor_AZ);};

  float angle_difference_el = Systema_EL.Status.Angular - Target.Elevation;
  if (fabs(angle_difference_el) > Systema_EL.Settings.Aiming_accuracy) {
    if (!Systema_EL.Status.Moving) {Turning_on_the_engine(&Motor_EL, angle_difference_el > 0 ? Up : Down);}
  	else {Increasing_the_pulse_frequency(&Motor_EL);};
  }
  else {Engine_shutdown(&Motor_EL);};
};
