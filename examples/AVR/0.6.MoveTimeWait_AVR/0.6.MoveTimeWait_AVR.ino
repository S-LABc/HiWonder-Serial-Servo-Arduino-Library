/*
 * 0.6.MoveTimeWait_AVR
 * 
 * Управление сервоприводом в режиме сервопривода с указанием времени и положения
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

// Положение в которое вал должен переместиться
const uint16_t POS_1 = 100; // 0 - 1000 (для некоторых приводов 1500)
const uint16_t POS_2 = 1000; // 0 - 1000 (для некоторых приводов 1500)

// Время за которое вал должен переместиться
const uint16_t TIME_1 = 3000; // 0 - 30000 мс
const uint16_t TIME_2 = 1000; // 0 - 30000 мс

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
}

void loop()
{
  Serial.print("Position 1: ");
  BusServo.setServoMoveTimeWait(SERVO_ID, POS_1, TIME_1);
  delay(500); // Пауза для наглядного представления что метод с Wait не запускает действие сразу
  BusServo.setServoMoveStart(SERVO_ID);
  delay(TIME_1 + 500); // Пауза + задержка для уверенности
  BusServo.setServoMoveStop(SERVO_ID);
  Serial.println(BusServo.getServoPosition(SERVO_ID)); // Запрос реального положения вала сервопривода в позиции 1

  Serial.print("Position 2: ");
  BusServo.setServoMoveTimeWait(SERVO_ID, POS_2, TIME_2);
  delay(1000); // Пауза для наглядного представления что метод с Wait не запускает действие сразу
  BusServo.setServoMoveStart(SERVO_ID); 
  delay(TIME_2 + 500); // Пауза + задержка для уверенности
  BusServo.setServoMoveStop(SERVO_ID);
  Serial.println(BusServo.getServoPosition(SERVO_ID)); // Запрос реального положения вала сервопривода в позиции 1
}
