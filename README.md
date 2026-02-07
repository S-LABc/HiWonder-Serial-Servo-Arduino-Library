# HiWonder-Serial-Servo-Arduino-Library
Библиотека для Arduino в которой реализована подержка цифровых сервоприводов HiWonder (могут встречаться по брендами LOBOT, LewanSoul) с интерфейсом управления Half-Duplex UART

## Общиее
* Поддерживает HardwareSerial и SoftwareSerial
* Есть возможность отправлять данные приводу без обратной связи
* Проверена работа на платах Arduino UNO и Arduino MEGA2560
* Должна работать на платах STM32 и ESP32 (на данный момент находится в разработке)
* Использует встроенные библиотеки **HardwareSerial.h** и **SoftwareSerial.h**
* Можно наследовать класс **HiWonderSerialServo** и расширять или изменять методы

## Установка
* Скачать репозиторий: Кнопка **Code**, затем **Downloade ZIP**
* В Arduino IDE выбрать **Скетч**, **Подключить библиотеку**, **Добавить .ZIP библиотеку…** и выбрать скачанный архив

## Ссылки
* [Официальный сайт HiWonder](https://www.hiwonder.com/collections/bus-servo?page=1)
* [Документация к протоколу обмена данными](https://github.com/S-LABc/HiWonder-Serial-Servo-Arduino-Library/blob/main/docs/Hiwonder%20Bus%20Servo%20Communication%20Protocol.pdf)
* [Пример схемы подключения](https://github.com/S-LABc/HiWonder-Serial-Servo-Arduino-Library/blob/main/docs/half_full_uart_sch.png)
