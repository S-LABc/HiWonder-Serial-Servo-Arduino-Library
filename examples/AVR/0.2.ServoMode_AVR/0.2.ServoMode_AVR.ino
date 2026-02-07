/*
 * 0.2.ServoMode_AVR
 * 
 * Управление сервоприводом в режиме сервопривода
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

// Начальное и конечное положения вала сервопривода
const uint16_t POS_1 = 100; // 0 - 1000
const uint16_t POS_2 = 800; // 0 - 1000

// Время за которое вал должен придти в заданное положение
const uint16_t TIME_POS_1 = 3000; // 0 - 30000
const uint16_t TIME_POS_2 = 3000; // 0 - 30000

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
  // Принудительное переключение сервопривода в режим сервопривода
  BusServo.setServoOrMotorMode(SERVO_ID, HIWONDER_SERVO_MODE);
}

void loop()
{
  BusServo.setServoMoveTime(SERVO_ID, POS_1, TIME_POS_1); // Поворот вала сервопривода в позицию 1
  delay(TIME_POS_1 + 100); // Важно не передавать новое положение раньше чем привод дойдет до этого положения. +100мс - для нормализации значений поположения
  
  Serial.print("Position 1: ");
  Serial.println(BusServo.getServoPosition(SERVO_ID)); // Запрос реального положения вала сервопривода в позиции 1
  
  BusServo.setServoMoveTime(SERVO_ID, POS_2, TIME_POS_2); // Поворот вала сервопривода в позицию 2
  delay(TIME_POS_2 + 100); // Важно не передавать новое положение раньше чем привод дойдет до этого положения. +100мс - для нормализации значений поположения

  Serial.print("Position 2: ");
  Serial.println(BusServo.getServoPosition(SERVO_ID));// Запрос реального положения вала сервопривода в позиции 2

  Serial.println();
}
