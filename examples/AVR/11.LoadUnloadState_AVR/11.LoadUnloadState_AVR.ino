/*
 * 11.LoadUnloadState_AVR
 * 
 * Настройка блокировки и разблокировки вала сервопривода
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

  // Получение имеющегося состояния
  Serial.print("CURR: ");
  HiwonderServoLoadOrUnloadState_t state;
  state = BusServo.getServoLoadOrUnloadState(SERVO_ID);
  if (state == HIWONDER_SERVO_LOAD_STATE)
  {
    Serial.println("Load");
  }
  else if (state == HIWONDER_SERVO_UNLOAD_STATE)
  {
    Serial.println("Unload");
  }

  Serial.println();
  delay(1000);

  // Установка нового состояния
  Serial.println("NEW: Load");
  BusServo.setServoLoadOrUnloadState(SERVO_ID, HIWONDER_SERVO_LOAD_STATE);
  
  Serial.println();
  delay(100); // Пауза для выполнения команды сервоприводом
}

void loop()
{
  // Получение текущего состояния в цикле
  Serial.print("CURR: ");
  HiwonderServoLoadOrUnloadState_t state;
  state = BusServo.getServoLoadOrUnloadState(SERVO_ID);
  if (state == HIWONDER_SERVO_LOAD_STATE)
  {
    Serial.println("Load");
  }
  else if (state == HIWONDER_SERVO_UNLOAD_STATE)
  {
    Serial.println("Unload");
  }
  delay(2000);
}
