/*
 * 0.9.VoltageLimit_AVR
 * 
 * Изменение пределов минимального и максимального рабочего напряжения сервопривода
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2026 / v0.1.0 / License MIT / Скляр Роман S-LAB
 */


// Подключение файла библиотеки
#include <HiWonderSerialServo.h>

// Адрес сервопривода
const uint8_t SERVO_ID = 1;

// Для чтения минимального и максимального напряжений
uint16_t min_lim_v = 0;
uint16_t max_lim_v = 0;

// Новые минимальное и максимальное напряжения
const uint16_t min_lim_v_new = 6000; // 4500 - 14000 мВ
const uint16_t max_lim_v_new = 9000; // 4500 - 14000 мВ

// Контакт для управление полудуплексным UART
const uint8_t HALF_DUPLEX_DIR_PIN = 24;

// Контакты для программного UART
const uint8_t SOFTWARE_SERIAL_RX_PIN = 10;
const uint8_t SOFTWARE_SERIAL_TX_PIN = 11;

/*
 * Создание объекта для работы с сервоприводами через HardwareSerial (Serial1, Serial2, Serial3 и т.д.)
*/
// С контактом управления полудуплексным UART
HiWonderSerialServo BusServo(&Serial3, HALF_DUPLEX_DIR_PIN);

// Без контакта управления полудуплексным UART. Можно только отправять данные, но не получать что либо в ответ
// Необходимо подключить TX непосредственно к линии данных сервопривода
//HiWonderSerialServo BusServo(&Serial3);

/*
 * Создание объекта для работы с сервоприводами через SoftwareSerial
*/
// Объект программаного UART
//SoftwareSerial SoftSerial(SOFTWARE_SERIAL_RX_PIN, SOFTWARE_SERIAL_TX_PIN);

// С контактом управления полудуплексным UART
//HiWonderSerialServo BusServo(&SoftSerial, HALF_DUPLEX_DIR_PIN);

// Без контакта управления полудуплексным UART. Можно только отправять данные, но не получать что либо в ответ
// Необходимо подключить TX непосредственно к линии данных сервопривода
//HiWonderSerialServo BusServo(&SoftSerial);

void setup()
{
  // Запуск отладочного порта
  Serial.begin(115200);
  // Инициализация библиотеки для сервопривода
  BusServo.begin();
  delay(1000);

  // Получение имеющихся пределов напряжения
  Serial.println("CURR");
  BusServo.getServoVoltageLimit(SERVO_ID, &min_lim_v, &max_lim_v);
  Serial.print("Min: ");
  Serial.print(min_lim_v);
  Serial.println(" mV");
  Serial.print("Max: ");
  Serial.print(max_lim_v);
  Serial.println(" mV");

  Serial.println();
  delay(1000);

  // Установка новых пределов напряжения
  Serial.println("NEW");
  Serial.print("Min: ");
  Serial.print(min_lim_v_new);
  Serial.println(" mV");
  Serial.print("Max: ");
  Serial.print(max_lim_v_new);
  Serial.println(" mV");
  BusServo.setServoVoltageLimit(SERVO_ID, min_lim_v_new, max_lim_v_new);
}

void loop()
{
  // Получение имеющихся пределов напряжения в цикле
  BusServo.getServoVoltageLimit(SERVO_ID, &min_lim_v, &max_lim_v);
  Serial.print("Min: ");
  Serial.print(min_lim_v);
  Serial.println(" mV");
  Serial.print("Max: ");
  Serial.print(max_lim_v);
  Serial.println(" mV");
  Serial.println();
  delay(2000);
}
