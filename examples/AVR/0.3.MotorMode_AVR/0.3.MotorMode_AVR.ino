/*
 * 0.3.MotorMode_AVR
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

// Скорости вращения вала сервопривода с указанием направления
const int16_t SPEED_1 = -200; // -1000 - 1000
const int16_t SPEED_2 = 500; // -1000 - 1000

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
  Serial.println("Dir 1");
  BusServo.setServoMoveStop(SERVO_ID); // Остановка вращения
  BusServo.setServoOrMotorMode(SERVO_ID, HIWONDER_MOTOR_MODE, SPEED_1); // Режим мотора со скоростью вращения 200 и направлением вращение в одну сторону
  BusServo.setServoMoveStart(SERVO_ID); // Запуск вращения

  checkModeAndSpeed();
  
  delay(1000); // Пауза

  Serial.println("Dir 2");
  BusServo.setServoMoveStop(SERVO_ID); // Остановка вращения
  BusServo.setServoOrMotorMode(SERVO_ID, HIWONDER_MOTOR_MODE, SPEED_2); // Режим мотора со скоростью вращения 500 и направлением вращение в другую сторону
  BusServo.setServoMoveStart(SERVO_ID); // Запуск вращения

  checkModeAndSpeed();

  delay(1000); // Пауза
}

void checkModeAndSpeed()
{
  // Переменные для хранения значений режима и скорости
  HiwonderServoOrMotorMode_t current_mode;
  int16_t current_speed;

  BusServo.getServoOrMotorMode(SERVO_ID, &current_mode, &current_speed); // Запрос текущего режима и текущей скорости

  Serial.print("Mode: ");
  if (current_mode == HIWONDER_SERVO_MODE)
  {
    Serial.println("SERVO");
  }
  else if (current_mode == HIWONDER_MOTOR_MODE)
  {
    Serial.println("MOTOR");
  }

  Serial.print("Speed: ");
  Serial.println(current_speed);

  Serial.println(); // Разграничение
}
