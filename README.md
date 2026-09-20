# G5500-ESP32S3-Controller

Контроллер поворотного устройства **Yaesu G-5500** на базе **ESP32-S3**.

Проект добавляет к штатному ротатору управление по USB и Wi‑Fi, совместимость с **Yaesu GS-232 / GS-232B**, **Hamlib rotctld**, **Look4Sat**, **PSTROTATOR**, а также встроенный Web UI для ручного управления, калибровки и диагностики.

Текущая версия прошивки: **v1.5.48 LVB MATCH TUNING**  
Copyright © UA1CFM

---

## Возможности

- управление азимутом и элевацией;
- совместная работа AZ + EL;
- совместимость с логикой оригинального проекта **LVBTrack**;
- Yaesu **GS-232 / GS-232B** по USB/UART;
- GS-232B по TCP/IP;
- **Hamlib rotctld / Look4Sat** по TCP;
- встроенный Web UI;
- работа в режиме Wi‑Fi Access Point;
- подключение к домашней Wi‑Fi сети;
- многоточечная калибровка AZ и EL;
- измерение положения через калиброванный ADC ESP32-S3;
- усреднение ADC по 25 измерениям;
- LCD 16×2;
- RGB-индикация обмена;
- журнал причин перезагрузки;
- история RSSI;
- сохранение настроек во Flash/NVS;
- защита от одновременного включения противоположных направлений одной оси.

BLE в текущей версии отключён.

---

## Поддерживаемые интерфейсы

### USB / UART0

Используется для управления из программ типа **PSTROTATOR**.

Параметры:

```text
9600 baud
8 data bits
No parity
1 stop bit
```

Поддерживаются команды семейства Yaesu GS-232 / GS-232B.

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

---

## TCP/IP

Порт:

```text
4533
```

Поддерживаются:

- Hamlib rotctld;
- Look4Sat;
- GS-232B over TCP.

---

## Web Interface

Web UI работает на:

```text
http://<IP-адрес-контроллера>/
```

В Web UI доступны:

- текущее положение AZ/EL;
- заданное положение;
- ручное управление;
- установка целевых координат;
- калибровка;
- ADC diagnostics;
- Wi‑Fi settings;
- RSSI history;
- reset diagnostics;
- история последних перезагрузок.

---

## Wi‑Fi

SSID точки доступа:

```text
G5500-Rotator
```

Кроме того, можно сохранить параметры домашней Wi‑Fi сети.

При старте контроллер:

1. запускает собственный Access Point;
2. один раз пытается подключиться к сохранённой домашней сети;
3. если сеть недоступна, продолжает работать через собственный AP.

---

## Аппаратная конфигурация ESP32-S3

### Аналоговые входы

| Функция | GPIO |
|---|---:|
| AZ position ADC | GPIO4 |
| EL position ADC | GPIO5 |

### Управление ротатором

| Направление | GPIO |
|---|---:|
| LEFT | GPIO15 |
| RIGHT | GPIO16 |
| UP | GPIO17 |
| DOWN | GPIO18 |

Выходы рассчитаны на управление внешними транзисторными ключами или другой согласующей схемой.

Не подключайте катушки реле или другие силовые нагрузки непосредственно к GPIO ESP32-S3.

### LCD 16×2 HD44780

| LCD | GPIO |
|---|---:|
| RS | GPIO8 |
| E | GPIO9 |
| D4 | GPIO10 |
| D5 | GPIO11 |
| D6 | GPIO12 |
| D7 | GPIO13 |

`R/W` LCD можно подключить к GND.

### RGB LED

По умолчанию используется:

```text
GPIO48
```

Индикация:

```text
GREEN  - RX command
RED    - TX response
BLUE   - heartbeat
```

---

## Калибровка

### Azimuth

Используются точки:

```text
0°
90°
180°
270°
360°
450°
```

### Elevation

Используются точки:

```text
0°
45°
90°
135°
180°
```

В рабочем тракте используется:

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

Калибровка сохраняется в NVS.

---

## Алгоритм движения

Логика движения максимально приближена к **LVBTrack.c**.

При получении новой цели контроллер:

1. считывает текущее положение;
2. определяет нужное направление;
3. включает соответствующий выход;
4. продолжает движение;
5. отключает выход после достижения или пересечения целевой координаты.

Контроллер не выполняет непрерывное автоматическое подруливание вокруг цели.

Команда `STOP` останавливает текущее движение, но не отключает возможность принять следующую команду позиционирования.

Ручное управление одной осью отменяет автоматическое движение только этой оси.

---

## Питание

Для стабильной работы ESP32-S3 особенно важно качественное питание.

В процессе разработки была обнаружена перезагрузка `BROWNOUT` при одновременном движении AZ + EL. Причиной оказалась недостаточная фильтрация питания перед линейным стабилизатором 8 V → 5 V.

После установки дополнительного электролитического конденсатора на входе 5-вольтового стабилизатора проблема исчезла.

Рекомендуется:

```text
8 V input
   |
470...1000 uF
   |
5 V regulator
   |
output capacitor
   |
ESP32-S3
```

Также полезны керамические конденсаторы около стабилизатора:

```text
100 nF input
100 nF output
```

---

## Сборка в Arduino IDE

Откройте:

```text
G5500_ESP32S3_Controller.ino
```

Выберите вашу ESP32-S3 плату и используйте те же параметры:

- Flash size;
- Partition scheme;
- PSRAM;
- USB mode;
- CPU frequency;
- Upload mode.

После этого:

```text
Sketch → Verify/Compile
```

или:

```text
Sketch → Upload
```

---

## Экспорт готовой прошивки

Для получения бинарного файла:

```text
Sketch → Export Compiled Binary
```

Arduino IDE создаст `.bin` файлы.

Для создания одного полного файла прошивки рекомендуется объединить bootloader, partition table, boot_app0 и application в один:

```text
G5500_ESP32S3_v1.5.48_FULL.bin
```

через `esptool merge-bin`.

Адреса бинарных секций нужно брать из реального verbose-лога Arduino IDE для конкретной платы и выбранной Partition Scheme.

---

## Совместимость с другими ротаторами Yaesu

Архитектура проекта может быть адаптирована и для других моделей, особенно:

```text
Yaesu G-800DXA
Yaesu G-1000DXA
Yaesu G-2800DXA
```

Для азимутальных ротаторов потребуется отключить EL-часть прошивки и адаптировать вход POSITION под допустимый диапазон ADC ESP32-S3.

---

## Диагностика

В прошивке доступны:

- ADC TEST;
- reset reason;
- reset history;
- RSSI history;
- USB diagnostic output.

Пример причины аппаратной перезагрузки:

```text
BROWNOUT
```

---

## Текущая версия

```text
v1.5.48 LVB MATCH TUNING
```

Основные изменения:

- ADC averaging: 25 samples;
- manual AZ cancels only AZ tracking;
- manual EL cancels only EL tracking;
- relay GPIO outputs preload LOW before switching to OUTPUT;
- LVBTrack-style target and stop logic retained.

---

## Автор

**UA1CFM**

Проект создан для управления антенной поворотной системой Yaesu G-5500 с использованием ESP32-S3.

---
