/*
 * Raspberry_Pi.c
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#include <Raspberry_Pi.h>

Target_data Target =
{
  .Azimuth = 0,
  .Elevation = 0,
  .Fire_mode = 0,
  .Rx_data = "Az+0000El-0000M0Fm\r\n\0",
  .Tx_data = "T0000000000\r\n",
  .transmitting = 0
};

/**
  * @brief Разбор и преобразование данных целеуказания
  * @param None
  * @return None
  *
  * @details
  * Функция выполняет обработку принятых данных в формате ASCII:
  * 1. Добавляет нуль-терминатор в конец строки данных (позиция 20)
  * 2. Преобразует данные азимута из строки в число с плавающей точкой:
  *    - Позиции 3-5: целая часть (сотни, десятки, единицы)
  *    - Позиция 6: десятые доли
  *    - Позиция 2: знак '-' для отрицательных значений
  * 3. Аналогично преобразует данные угла места (элевации):
  *    - Позиции 10-12: целая часть
  *    - Позиция 13: десятые доли
  *    - Позиция 9: знак '-'
  *
  * @note Ожидает данные в строго определенном формате:
  *       [X][±][DDD][D][X][±][DDD][D]...
  * @warning Не выполняет проверку на корректность входных данных
  * @note Преобразует ASCII-символы в числовые значения вычитанием '0'
  * @note Поддерживает отрицательные значения через знак '-'
  *
  * @sideeffect
  * - Модифицирует структуру Target (поля Azimuth и Elevation)
  * - Добавляет нуль-терминатор в массив Rx_data
  */
void Data_parsing(void) {
  Target.Rx_data[20] = '\0';

  Target.Azimuth =
    (Target.Rx_data[3]  - '0') * 100.0f +
    (Target.Rx_data[4]  - '0') * 10.0f +
    (Target.Rx_data[5]  - '0') +
    (Target.Rx_data[6]  - '0') * 0.1f;
  if (Target.Rx_data[2] == '-') Target.Azimuth = -Target.Azimuth;

  Target.Elevation =
    (Target.Rx_data[10] - '0') * 100.0f +
    (Target.Rx_data[11] - '0') * 10.0f +
    (Target.Rx_data[12] - '0') +
    (Target.Rx_data[13] - '0') * 0.1f;
  if (Target.Rx_data[9] == '-') Target.Elevation = -Target.Elevation;
};

/**
  * @brief Формирование сообщения с текущими углами положения
  * @param None
  * @return None
  *
  * @details
  * Функция формирует сообщение в формате ASCII с текущими углами:
  * 1. Работает только если передача не активна (!Target.transmitting)
  * 2. Подготавливает данные:
  *    - Преобразует углы в абсолютные значения с точностью до 0.1°
  *    - Масштабирует значения (×10) для хранения как целых чисел
  * 3. Формирует сообщение в буфере Tx_data:
  *    - Позиция 1: знак азимута ('1' если ≥0, иначе '0')
  *    - Позиции 2-5: цифры азимута (тысячи, сотни, десятки, единицы)
  *    - Позиция 6: знак угла места (аналогично азимуту)
  *    - Позиции 7-10: цифры угла места (аналогично азимуту)
  *
  * @note Сообщение формируется только когда система не передает данные
  * @warning Требует предварительной проверки флага transmitting
  * @note Точность представления углов - 0.1 градуса
  * @note Используется ASCII-кодировка цифр (добавление '0')
  * @note Формат сообщения: [S1][D4][S2][D4] (знак + 4 цифры на каждый угол)
  *
  * @sideeffect
  * - Модифицирует буфер Tx_data в структуре Target
  * - Не изменяет исходные угловые значения
  */
void Message_formation() {
  if (!Target.transmitting) {
    uint16_t angular_az = (uint16_t)fabs(Systema_AZ.Status.Angular * 10);
    uint16_t angular_el = (uint16_t)fabs(Systema_EL.Status.Angular * 10);

    Target.Tx_data[1] = (Systema_AZ.Status.Angular >= 0) ? '1' : '0';
    Target.Tx_data[2] = '0' + (angular_az / 1000) % 10;
    Target.Tx_data[3] = '0' + (angular_az / 100) % 10;
    Target.Tx_data[4] = '0' + (angular_az / 10) % 10;
    Target.Tx_data[5] = '0' + (angular_az) % 10;

    Target.Tx_data[6] = (Systema_EL.Status.Angular >= 0) ? '1' : '0';
    Target.Tx_data[7] = '0' +  (angular_el / 1000) % 10;
    Target.Tx_data[8] = '0' +  (angular_el / 100) % 10;
    Target.Tx_data[9] = '0' +  (angular_el / 10) % 10;
    Target.Tx_data[10] = '0' + (angular_el) % 10;
  };
};
