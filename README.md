```markdown
# Turret Control System with STM32F407VGT6 and Raspberry Pi 5

Проект представляет собой систему управления турелью с микроконтроллером STM32F407VGT6 для низкоуровневого управления и Raspberry Pi 5 для обработки изображений и высокоуровневой логики.

## Основные возможности
- Управление сервоприводами через ШИМ
- Двусторонняя связь по UART между STM32 и RPi
- Распознавание целей с помощью камеры Raspberry Pi
- Режимы работы: ручное управление и автономное слежение
- Защита от перегрузок и аварийное отключение

## Требуемое оборудование
| Компонент | Количество |
|-----------|------------|
| Плата STM32F407VGT6 (Discovery) | 1 |
| Raspberry Pi 5 | 1 |
| Сервоприводы MG996R | 2 |
| Камера Raspberry Pi HQ | 1 |
| Лазерный модуль 650 нм | 1 |
| Блок питания 12V 5A | 1 |
| Преобразователь уровней 5V↔3.3V | 1 |

## Схема подключения
```
Raspberry Pi 5          STM32F407VGT6
=====================================
GPIO14 (TXD)  ------>  PA3 (USART2_RX)
GPIO15 (RXD)  <------  PA2 (USART2_TX)
GND           ------>  GND

STM32F407VGT6    Периферия
=====================================
PA0 (PWM)    ------>  Сервопривод Pan 
PA1 (PWM)    ------>  Сервопривод Tilt
PB0          ------>  Управление лазером
PC13         ------>  Аварийный стоп
```

## Установка и настройка

### Для STM32 (Keil uVision)
1. Клонируйте репозиторий
2. Откройте проект `STM32_Turret_Control.uvprojx`
3. Установите пакеты CMSIS и HAL
4. Соберите проект и прошейте через ST-Link

### Для Raspberry Pi
```bash
git clone https://github.com/GusevDanilRabota/Turret_STM32F407VGT6_Raspberry_Pi_5.git
cd Turret_STM32F407VGT6_Raspberry_Pi_5/RPi_Control

# Установите зависимости
sudo apt install python3-opencv python3-serial

# Запуск системы
python3 main.py --mode=auto --camera=0
```

## Использование
### Пример команд
```python
ser = serial.Serial('/dev/ttyS0', 115200)
ser.write(b'PAN:45\n')   # Поворот по горизонтали
ser.write(b'TILT:30\n')  # Поворот по вертикали
ser.write(b'LASER:1\n')  # Активация лазера
```

### Режимы работы
1. **Ручное управление** (веб-интерфейс):
   ```bash
   python3 control_app.py
   ```
   Доступ через: `http://raspberrypi.local:8080`

2. **Автономный режим**:
   ```bash
   python3 main.py --mode=auto --confidence=0.75
   ```

3. **Калибровка сервоприводов**:
   ```bash
   python3 calibration_tool.py
   ```

## Структура проекта
```
├── STM32_Turret_Control/   # Проект STM32
├── RPi_Control/            # Скрипты Raspberry Pi
│   ├── main.py             # Основной скрипт
│   ├── vision_processor.py # Обработка видео
│   └── serial_interface.py # Коммуникация с STM32
├── docs/                   # Документация
├── firmware/               # Прошивки STM32
└── LICENSE                 # Лицензия MIT
```

## Параметры по умолчанию
| Параметр          | Значение       |
|-------------------|---------------|
| Скорость UART     | 921600 бод    |
| Разрешение камеры | 1920x1080     |
| FPS обработки     | 30            |

## Лицензия
MIT License - подробности в файле [LICENSE](LICENSE)

## Контакты
**Автор**: Гусев Данил  
**Email**: your.email@example.com  
**Telegram**: @your_telegram  

> **Внимание**: При работе с лазером соблюдайте меры безопасности. Избегайте прямого попадания лазера в глаза.
```

Для использования:
1. Скопируйте весь текст
2. Вставьте в файл `README.md` в корне репозитория
3. Заполните контактную информацию
4. Добавьте изображения в папку `docs/`:
   - `system_architecture.png` - схема системы
   - `demo.gif` - пример работы турели
