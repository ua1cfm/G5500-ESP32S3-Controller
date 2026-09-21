# G5500 ESP32-S3 Controller

Контроллер поворотного устройства **Yaesu G-5500** на базе **ESP32-S3**.

Текущая версия прошивки: **v1.5.49 R4UAB DDE TCP**  
Copyright © UA1CFM

Проект предназначен для управления Yaesu G-5500 через USB и Wi‑Fi с поддержкой Web UI, Yaesu GS-232/GS-232B, Hamlib rotctld, Look4Sat, PSTROTATOR и R4UAB DDE Client.

## Возможности

- управление AZ + EL;
- одновременное движение по двум плоскостям;
- логика движения по принципу LVBTrack;
- GS-232 / GS-232B по USB/UART0;
- GS-232B по TCP/IP;
- Hamlib rotctld по TCP;
- Look4Sat по TCP;
- R4UAB DDE Client по TCP;
- Web UI;
- Wi‑Fi AP + подключение к домашней сети;
- многоточечная калибровка AZ и EL;
- измерение через `analogReadMilliVolts()`;
- усреднение ADC по 25 выборкам;
- LCD 16×2;
- RGB-индикация RX/TX/heartbeat;
- reset history;
- RSSI history;
- хранение параметров в NVS;
- раздельная отмена tracking по AZ и EL при ручном управлении;
- безопасный старт GPIO через предварительный LOW до `pinMode(OUTPUT)`.

BLE в текущей версии отключён.

## Текущая стабильная версия

```text
v1.5.49 R4UAB DDE TCP
```

Версия основана на v1.5.48 и сохраняет все её исправления.

### Что добавлено в v1.5.49

Добавлена поддержка формата чисел, который R4UAB DDE Client отправляет при русской локали Windows.

R4UAB может передавать:

```text
P 187,30 22,40
```

Контроллер нормализует строку в:

```text
P 187.30 22.40
```

После чего принимает:

```text
AZ = 187.3°
EL = 22.4°
```

и отвечает:

```text
RPRT 0
```

Пример реального обмена:

```text
[TCP RX] P 187,30 22,40
[LOOK4SAT TARGET] AZ=187.3 EL=22.4
[TCP TX] RPRT 0

[TCP RX] p
[TCP TX] 1.853933
0.715318
```

Метка `[LOOK4SAT TARGET]` в диагностике историческая; команда может приходить и от R4UAB DDE Client.

## Поддерживаемые протоколы

### Yaesu GS-232 / GS-232B по USB/UART0

```text
9600 baud
8 data bits
No parity
1 stop bit
```

Основные команды:

```text
R       Rotate Right
L       Rotate Left
U       Elevation Up
D       Elevation Down
A       Stop Azimuth
E       Stop Elevation
S       Stop all
C       Read Azimuth
B       Read Elevation
C2      Read Azimuth + Elevation
Maaa    Set Azimuth
Waaa eee
        Set Azimuth + Elevation
P36
P45
H
H2
H3
```

Подходит для PSTROTATOR.

### TCP/IP

Порт:

```text
4533
```

Поддерживаются:

- Hamlib rotctld;
- Look4Sat;
- GS-232B over TCP;
- R4UAB DDE Client.

## Hamlib / Look4Sat / R4UAB DDE Client

### Чтение текущего положения

Команда:

```text
p
```

Ответ:

```text
AZ
EL
```

Пример:

```text
187.300000
22.400000
```

### Установка положения

```text
P 187.30 22.40
\set_pos 187.30 22.40
set_pos 187.30 22.40
```

Для R4UAB DDE Client также поддерживаются десятичные запятые:

```text
P 187,30 22,40
```

### Остановка

```text
S
stop
\stop
```

Ответ:

```text
RPRT 0
```

## R4UAB DDE Client

Подключение выполняется по TCP к IP-адресу контроллера, порт:

```text
4533
```

При успешном соединении контроллер принимает команды вида:

```text
P 187,30 22,40
```

и отвечает:

```text
RPRT 0
```

Клиент также периодически запрашивает текущее положение командой:

```text
p
```

Контроллер возвращает AZ и EL отдельными строками.

## Web Interface

Web UI:

```text
http://<IP-адрес-контроллера>/
```

В Web UI доступны:

- текущее AZ/EL;
- заданные координаты;
- ручное управление;
- установка цели;
- калибровка;
- ADC diagnostics;
- Wi‑Fi settings;
- RSSI history;
- reset diagnostics;
- история перезагрузок.

## Wi‑Fi

SSID точки доступа:

```text
G5500-Rotator
```

При старте:

1. запускается собственный AP;
2. выполняется одна попытка подключения к сохранённой домашней сети;
3. если сеть недоступна, контроллер остаётся в AP-режиме.

## Аппаратная конфигурация ESP32-S3

### Аналоговые входы

| Функция | GPIO |
|---|---:|
| AZ position ADC | GPIO4 |
| EL position ADC | GPIO5 |

### Выходы управления

| Направление | GPIO |
|---|---:|
| LEFT | GPIO15 |
| RIGHT | GPIO16 |
| UP | GPIO17 |
| DOWN | GPIO18 |

Не подключайте реле и другие силовые нагрузки напрямую к GPIO ESP32-S3.

## LCD 16×2 HD44780

| LCD | GPIO |
|---|---:|
| RS | GPIO8 |
| E | GPIO9 |
| D4 | GPIO10 |
| D5 | GPIO11 |
| D6 | GPIO12 |
| D7 | GPIO13 |

`R/W` можно подключить к GND.

## RGB LED

По умолчанию:

```text
GPIO48
```

```text
GREEN  - RX
RED    - TX
BLUE   - heartbeat
```

## Калибровка

### Azimuth

```text
0°
90°
180°
270°
360°
450°
```

### Elevation

```text
0°
45°
90°
135°
180°
```

Рабочий тракт:

```text
analogReadMilliVolts()
        ↓
factory calibrated ESP32-S3 ADC
        ↓
25-sample averaging
        ↓
piecewise mechanical calibration
        ↓
AZ / EL angle
```

## Алгоритм движения


После новой цели контроллер:

1. считывает текущее положение;
2. определяет направление;
3. включает соответствующий выход;
4. продолжает движение;
5. отключает выход после достижения или пересечения цели.

Непрерывного подруливания вокруг цели нет.

`STOP` останавливает текущее движение, но следующая команда снова запускает движение.

Ручное управление одной осью отменяет tracking только этой оси.

## Исправления v1.5.48, сохранённые в v1.5.49

- ADC averaging: 25 samples;
- ручной AZ отменяет только AZ tracking;
- ручной EL отменяет только EL tracking;
- GPIO реле предварительно устанавливаются LOW до OUTPUT;
- float-сравнение текущего положения и цели;
- LVBTrack-style start/stop logic;
- RESET HISTORY;
- PSTROTATOR / Look4Sat / Hamlib compatibility.

## Питание и BROWNOUT

При одновременном движении AZ + EL был обнаружен reset:

```text
BROWNOUT
```

Причина оказалась в питании до стабилизатора 8 V → 5 V.

После установки дополнительного электролитического конденсатора перед 5-вольтовым стабилизатором проблема исчезла.

Рекомендуемая цепь:

```text
+8 V
  |
Schottky diode
  |
470...1000 uF
  |
5 V regulator
  |
output capacitor
  |
ESP32-S3
```

Дополнительно полезны:

```text
100 nF на входе стабилизатора
100 nF на выходе стабилизатора
```

## Последовательный Schottky по 8 V

Был установлен последовательный Schottky-диод в линии +8 V:

```text
+8 V source → ANODE |>| CATHODE → regulator input
```

Полоса на корпусе соответствует катоду.

После установки диода исчез резкий бросок стрелочных индикаторов G-5500 при выключении питания.

Подходящие силовые Schottky:

```text
SS34
SS54
1N5822
MBR340
MBR360
```

## Защита входов GPIO4 / GPIO5

Защита построена по идее LVB Tracker, адаптированной к 3.3 V ESP32-S3.

Для каждого входа:

```text
G-5500 AZ/EL signal
        |
       4.7 kOhm
        |
        +--------- GPIO4 / GPIO5
        |
       |>| BAT85
        |
       +3.3 V
```

Ориентация BAT85:

```text
ANODE   → GPIO
CATHODE → +3.3 V
```

BAT85 подходит для этой задачи как Schottky-диод.

Последовательный резистор 4.7 kOhm ограничивает ток при выбросе.

Дополнительно можно поставить:

```text
100 nF от GPIO4 к GND
100 nF от GPIO5 к GND
```

Если на EL уже стоит 4.7 kOhm к GND, его можно оставить.

## Защита реле

Если используются реле, желательно поставить обратные диоды непосредственно параллельно катушкам:

```text
cathode → +5 V
anode   → collector/drain
```

Подойдут:

```text
1N4001...1N4007
1N5819
```

## Транзисторные ключи

Возможны BC547 или logic-level N-MOSFET.

Пример для N-MOSFET:

```text
GPIO → 100...330 Ohm → Gate
Gate → 10 kOhm → GND
Source → GND
Drain → load
```

MOSFET должен уверенно открываться от 3.3 V.

## Совместимость с другими ротаторами Yaesu

Планируемые профили:

```text
Yaesu G-5500 / G-5500DC
Yaesu G-800DXA
Yaesu G-1000DXA
Yaesu G-2800DXA
Yaesu G-450ADC / G-450CDC
Custom / Other
```

### G-5500

```text
AZ + EL
AZ: 0...450°
EL: 0...180°
```

### G-800DXA / G-1000DXA / G-2800DXA

```text
AZ only
0...450°
CW / CCW
Analog position
Optional 0...5 V speed input
```

### G-450ADC / G-450CDC

Потребуется отдельная аппаратная адаптация, но архитектурно возможен AZ-only профиль.

В будущих версиях Web UI можно добавить:

```text
Rotator type:
[ G-5500 / G-5500DC ]
[ G-800DXA ]
[ G-1000DXA ]
[ G-2800DXA ]
[ G-450ADC / G-450CDC ]
[ Custom / Other ]
```

## Диагностика

Доступны:

- ADC TEST;
- reset reason;
- reset history;
- RSSI history;
- USB diagnostics;
- TCP RX/TX diagnostics.

Примеры:

```text
[TCP RX] P 187,30 22,40
[TCP TX] RPRT 0
```

```text
[RESET REASON] BROWNOUT
```

## Сборка в Arduino IDE

Откройте:

```text
G5500_ESP32S3_Controller_v1_5_49_R4UAB_DDE_TCP.ino
```

Выберите корректную ESP32-S3 и параметры:

- Flash size;
- Partition scheme;
- PSRAM;
- USB mode;
- CPU frequency;
- Upload mode.

Затем:

```text
Sketch → Verify/Compile
```

или:

```text
Sketch → Upload
```

## Экспорт готовой прошивки

```text
Sketch → Export Compiled Binary
```

Для одного полного файла можно объединить:

```text
bootloader
partition table
boot_app0
application
```

в:

```text
G5500_ESP32S3_v1.5.49_FULL.bin
```

через `esptool merge-bin`.

Адреса секций берите из verbose upload log Arduino IDE.

## Рекомендуемая структура GitHub

```text
G5500-ESP32S3-Controller/
├── G5500_ESP32S3_Controller_v1_5_49_R4UAB_DDE_TCP.ino
├── README.md
├── LICENSE
└── docs/
```

## Автор

**UA1CFM**

ESP32-S3 controller for Yaesu G-5500 antenna rotator.



