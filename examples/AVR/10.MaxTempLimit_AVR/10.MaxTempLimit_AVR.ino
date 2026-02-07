/*
 * 10.MaxTempLimit_AVR
 * 
 * Изменение верхнего предела темпаратуры сервопривода
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

// Новое максимальное значение предела температуры
const uint8_t max_temp_lim = 95; // 50 - 100 С

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

  // Получение имеющегося предела температуры
  Serial.println("CURR");
  Serial.print("Max Temp: ");
  Serial.print(BusServo.getServoMaximumTemperatureLimit(SERVO_ID));
  Serial.println(" C");

  Serial.println();
  delay(1000);

  // Установка нового предела температуры
  Serial.println("NEW");
  Serial.print("Max Temp: ");
  Serial.print(max_temp_lim);
  Serial.println(" C");
  BusServo.setServoMaximumTemperatureLimit(SERVO_ID, max_temp_lim);
  Serial.println();
  delay(100); // Пауза для выполнения команды сервоприводом
}

void loop()
{
  // Получение текущего предела температуры
  Serial.print("Max Temp: ");
  Serial.print(BusServo.getServoMaximumTemperatureLimit(SERVO_ID));
  Serial.println(" C");
  Serial.println();
  delay(2000);
}
