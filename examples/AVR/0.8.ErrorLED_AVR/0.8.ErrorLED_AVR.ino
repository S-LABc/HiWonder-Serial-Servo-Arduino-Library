/*
 * 0.8.ErrorLED_AVR
 * 
 * Получение кода ошибки
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
}

void loop()
{
  checkErrorCode();
  Serial.println();
  delay(500);
}

void checkErrorCode()
{
  HiwonderServoErorCodeLED_t error; // Переменная для хранения кода ошибки

  error = BusServo.getServoErrorLED(SERVO_ID); // Получение кода ошибки

  if (error == HIWONDER_SERVO_ERROR_LED_NO_ALARM)
  {
    Serial.print("No Alarm "); // Нет ошибок
  }
  else if (error == HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE)
  {
    Serial.print("Over Temperature "); // Перегрев
  }
  else if (error == HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE)
  {
    Serial.print("Over Voltage "); // Перенапряжение
  }
  else if (error == HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_VOLTAGE)
  {
    Serial.print("Over Temperature and Voltage "); // Перегрев и перенапряжение
  }
  else if (error == HIWONDER_SERVO_ERROR_LED_LOKED_ROTOR)
  {
    Serial.print("Loked Rotor "); // Заблокирован вал
  }
  else if (error == HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_STALLED)
  {
    Serial.print("Over Temperature and Stalled "); // Перегрев и заклинило
  }
  else if (error == HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE_AND_STALLED)
  {
    Serial.print("Over Voltage and Stalled "); // Перенапряжение и заклинило
  }
  else if (error == HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_OVER_VOLTAGE_AND_STALLED)
  {
    Serial.print("Over Voltage and Over Temperature and Stalled "); // Перегрев, перенапряжение, заклинило
  }

  Serial.println(error); // Вывод числового кода ошибки
}
