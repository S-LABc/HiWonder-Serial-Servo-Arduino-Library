/* 
 * Класс для Arduino IDE реализующий множество методов
 * взаимодействия с программируемыми сервоприводами
 * 
 * Hiwonder https://www.hiwonder.com/
 * 
 * Псевдонимы: LOBOT, HiWonder, LewanSoul
 * 
 * Формат пакета отправляемого в сервопривод
 * Header1 | Header2 | ID number | Data Length | Command | Parameter | Checksum
 *   0x55      0x55    0x00~0xFD      Length       Cmd     Prm1~PrmN     CRC
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2026 / v0.0.1 / License MIT / Скляр Роман S-LAB
 */


#include "HiWonderSerialServo.h"

// ########## CONSTRUCTOR ##########
/*
 * @brief: конструктор с использованием HardwareSerial, контакта переключения приема/передачи
 * @param _h_serial: доступ к методам объекта класса HardwareSerial
 * @param _dir_pin: контакт перключения приема/передачи полудуплексного режима
 */
HiWonderSerialServo::HiWonderSerialServo(HardwareSerial *_h_serial, uint8_t _dir_pin)
{
  _use_single_pin_mode_ = false;
  _use_software_serial_ = false;
  _h_serial_ = _h_serial;
  _pin_direction_ = _dir_pin;
}

/*
 * @brief: конструктор с использованием только HardwareSerial
 * @param _h_serial: доступ к методам объекта класса HardwareSerial
 */
HiWonderSerialServo::HiWonderSerialServo(HardwareSerial *_h_serial)
{
  _use_single_pin_mode_ = true;
  _use_software_serial_ = false;
  _h_serial_ = _h_serial;
}

#if !defined(ESP32)
/*
 * @brief: конструктор с использованием SoftwareSerial, контакта переключения приема/передачи
 * @param _s_serial: доступ к методам объекта класса SoftwareSerial
 * @param _dir_pin: контакт перключения приема/передачи полудуплексного режима
 */
HiWonderSerialServo::HiWonderSerialServo(SoftwareSerial *_s_serial, uint8_t _dir_pin)
{
  _use_single_pin_mode_ = false;
  _use_software_serial_ = true;
  _s_serial_ = _s_serial;
  _pin_direction_ = _dir_pin;
}

/*
 * @brief: конструктор с использованием только SoftwareSerial
 * @param _s_serial: доступ к методам объекта класса SoftwareSerial
 */
HiWonderSerialServo::HiWonderSerialServo(SoftwareSerial *_s_serial)
{
  _use_single_pin_mode_ = true;
  _use_software_serial_ = true;
  _s_serial_ = _s_serial;
}
#endif


// ########## PROTECTED ##########
/*
 * @brief: передача пакета данных приводу через полудуплексный UART
 * @param _send_pack: массив байтов (пакет) для передачи приводу
 * @param _pack_length: длина массива (пакета)
 */
void HiWonderSerialServo::HW_SendPacket(unsigned char *_send_pack, unsigned int _pack_length)
{
  if (_use_software_serial_)
  { // Очистка входящего буфера
#if !defined(ESP32)
    while (_s_serial_->available())
    {
      _s_serial_->read();
    }
#endif
  }
  else
  {
    while (_h_serial_->available())
    {
      _h_serial_->read();
    }
  }

  if (_use_single_pin_mode_)
  {
#if defined(ARDUINO_ARCH_STM32)
    _h_serial_->write(_send_pack, _pack_length); // Переключение на передачу данных и отправка пакета побайтово
    _h_serial_->enableHalfDuplexRx(); // Переключение на прием данных (flush() вызывается внутри enableHalfDuplexRx())
#else
    if (_use_software_serial_)
    {
#if !defined(ESP32)
      _s_serial_->write(_send_pack, _pack_length); // Отправка пакета побайтово
      _s_serial_->flush(); // Очистка буфера приема
#endif
    }
    else
    {
      _h_serial_->write(_send_pack, _pack_length); // Отправка пакета побайтово
      _h_serial_->flush(); // Очистка буфера приема
    }
#endif
  }
  else
  {
    if (_use_software_serial_)
    {
#if !defined(ESP32)
      setHalfDuplexSerialModeSender();
      _s_serial_->write(_send_pack, _pack_length); // Отправка пакета побайтово
      _s_serial_->flush(); // Очистка буфера приема
      setHalfDuplexSerialModeReciever();
#endif
    }
    else
    {
      setHalfDuplexSerialModeSender();
      _h_serial_->write(_send_pack, _pack_length); // Отправка пакета побайтово
      _h_serial_->flush(); // Очистка буфера приема
      setHalfDuplexSerialModeReciever();
    }
  }
}
/*
 * @brief: прием пакета данных от привода через полудуплексный UART
 * @param _return_data: адрес массива для передачи полезных данных от привода
 * @param _answer_lenght: длина пакета данных от привода
 *   HIWONDER_SERVO_LENGHT_PACKET_LENGHT_BYTES_4 - 4 байта
 *   HIWONDER_SERVO_LENGHT_PACKET_LENGHT_BYTES_5 - 5 байт
 *   HIWONDER_SERVO_LENGHT_PACKET_LENGHT_BYTES_7 - 7 байт
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 */
void HiWonderSerialServo::HW_ReceivePacket(uint8_t *_return_data, uint8_t _answer_lenght, uint8_t _id)
{
  uint8_t attempts = 0;
  uint8_t receive_buffer[10]; // 10 - максимальное количество байт данных приходящих от привода одним пакетом

  if (_use_software_serial_)
  {
#if !defined(ESP32)
    // Ожидание минимального количества данных через таймер
    while ((_s_serial_->available() < HIWONDER_SERVO_RECIEVED_BYTE_BUFFER_MIN) & (attempts < HIWONDER_SERVO_ATTEMPTS_COUNT_MAX))
    {
		  attempts++;
		  delayMicroseconds(HIWONDER_SERVO_TIMEOUT_CHEKING_BUFFER_MAX);
    }

    while (_s_serial_->available() > 0)
    {
      receive_buffer[0] = _s_serial_->read(); // Первый байт заголовка
      if (receive_buffer[0] == HIWONDER_SERVO_PACKET_HEADER)
      {
        receive_buffer[1] = _s_serial_->read(); // Второй байт заголовка
        if (receive_buffer[1] == HIWONDER_SERVO_PACKET_HEADER)
        {
          receive_buffer[2] = _s_serial_->read(); // ID
          if (receive_buffer[2] == _id)
          {
            receive_buffer[3] = _s_serial_->read(); // Длина команды
            receive_buffer[4] = _s_serial_->read(); // Номер команды

            if (_answer_lenght == HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES)
            {
              receive_buffer[5] = _s_serial_->read(); // Запрошенное значение
              receive_buffer[6] = _s_serial_->read(); // Контрольная сумма
              // Вычисление контрольной суммы полученного пакета
              uint8_t calc_crc = (~(receive_buffer[2] + receive_buffer[3] + receive_buffer[4] + receive_buffer[5])) & 0xFF;
              if (receive_buffer[6] != calc_crc)
              { // Если ошибка
                _return_data[0] = 0;
              }
              else
              {
                _return_data[0] = receive_buffer[5];
              }
            }
            if (_answer_lenght == HIWONDER_SERVO_LENGHT_PACKET_FIVE_BYTES)
            {
              receive_buffer[5] = _s_serial_->read(); // Запрошенное значение (младший байт)
              receive_buffer[6] = _s_serial_->read(); // Запрошенное значение (старший байт)
              receive_buffer[7] = _s_serial_->read(); // Контрольная сумма
              // Вычисление контрольной суммы полученного пакета
              uint8_t calc_crc = (~(receive_buffer[2] + receive_buffer[3] + receive_buffer[4] + receive_buffer[5] + receive_buffer[6])) & 0xFF;
              if (receive_buffer[7] != calc_crc)
              { // Если ошибка
                _return_data[0] = 0;
                _return_data[1] = 0;
              }
              else
              {
                _return_data[0] = receive_buffer[6]; // Старший
                _return_data[1] = receive_buffer[5]; // Младший
              }
            }
            if (_answer_lenght == HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES)
            {
              receive_buffer[5] = _s_serial_->read(); // Запрошенное значение 1 (младший байт)
              receive_buffer[6] = _s_serial_->read(); // Запрошенное значение 1 (старший байт)
              receive_buffer[7] = _s_serial_->read(); // Запрошенное значение 2 (младший байт)
              receive_buffer[8] = _s_serial_->read(); // Запрошенное значение 2 (старший байт)
              receive_buffer[9] = _s_serial_->read(); // Контрольная сумма
              // Вычисление контрольной суммы полученного пакета
              uint8_t calc_crc = (~(receive_buffer[2] + receive_buffer[3] + receive_buffer[4] + receive_buffer[5] + receive_buffer[6] + receive_buffer[7] + receive_buffer[8])) & 0xFF;
              if (receive_buffer[9] != calc_crc)
              { // Если ошибка
                _return_data[0] = 0;
                _return_data[1] = 0;
                _return_data[2] = 0;
                _return_data[3] = 0;
              }
              else
              {
                _return_data[0] = receive_buffer[6]; // Старший 1
                _return_data[1] = receive_buffer[5]; // Младший 1
                _return_data[2] = receive_buffer[8]; // Старший 2
                _return_data[3] = receive_buffer[7]; // Младший 2
              }
            }
          }
        }
      }
    }
#endif
	}
  else
  {
    // Ожидание минимального количества данных через таймер
    while ((_h_serial_->available() < HIWONDER_SERVO_RECIEVED_BYTE_BUFFER_MIN) & (attempts < HIWONDER_SERVO_ATTEMPTS_COUNT_MAX))
    {
		  attempts++;
		  delayMicroseconds(HIWONDER_SERVO_TIMEOUT_CHEKING_BUFFER_MAX);
    }

    while (_h_serial_->available() > 0)
    {
      receive_buffer[0] = _h_serial_->read(); // Первый байт заголовка
      if (receive_buffer[0] == HIWONDER_SERVO_PACKET_HEADER)
      {
        receive_buffer[1] = _h_serial_->read(); // Второй байт заголовка
        if (receive_buffer[1] == HIWONDER_SERVO_PACKET_HEADER)
        {
          receive_buffer[2] = _h_serial_->read(); // ID
          if (receive_buffer[2] == _id)
          {
            receive_buffer[3] = _h_serial_->read(); // Длина команды
            receive_buffer[4] = _h_serial_->read(); // Номер команды

            if (_answer_lenght == HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES)
            {
              receive_buffer[5] = _h_serial_->read(); // Запрошенное значение
              receive_buffer[6] = _h_serial_->read(); // Контрольная сумма
              // Вычисление контрольной суммы полученного пакета
              uint8_t calc_crc = (~(receive_buffer[2] + receive_buffer[3] + receive_buffer[4] + receive_buffer[5])) & 0xFF;
              if (receive_buffer[6] != calc_crc)
              { // Если ошибка
                _return_data[0] = 0;
              }
              else
              {
                _return_data[0] = receive_buffer[5];
              }
            }
            if (_answer_lenght == HIWONDER_SERVO_LENGHT_PACKET_FIVE_BYTES)
            {
              receive_buffer[5] = _h_serial_->read(); // Запрошенное значение (младший байт)
              receive_buffer[6] = _h_serial_->read(); // Запрошенное значение (старший байт)
              receive_buffer[7] = _h_serial_->read(); // Контрольная сумма
              // Вычисление контрольной суммы полученного пакета
              uint8_t calc_crc = (~(receive_buffer[2] + receive_buffer[3] + receive_buffer[4] + receive_buffer[5] + receive_buffer[6])) & 0xFF;
              if (receive_buffer[7] != calc_crc)
              { // Если ошибка
                _return_data[0] = 0;
                _return_data[1] = 0;
              }
              else
              {
                _return_data[0] = receive_buffer[6]; // Старший
                _return_data[1] = receive_buffer[5]; // Младший
              }
            }
            if (_answer_lenght == HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES)
            {
              receive_buffer[5] = _h_serial_->read(); // Запрошенное значение 1 (младший байт)
              receive_buffer[6] = _h_serial_->read(); // Запрошенное значение 1 (старший байт)
              receive_buffer[7] = _h_serial_->read(); // Запрошенное значение 2 (младший байт)
              receive_buffer[8] = _h_serial_->read(); // Запрошенное значение 2 (старший байт)
              receive_buffer[9] = _h_serial_->read(); // Контрольная сумма
              // Вычисление контрольной суммы полученного пакета
              uint8_t calc_crc = (~(receive_buffer[2] + receive_buffer[3] + receive_buffer[4] + receive_buffer[5] + receive_buffer[6] + receive_buffer[7] + receive_buffer[8])) & 0xFF;
              if (receive_buffer[9] != calc_crc)
              { // Если ошибка
                _return_data[0] = 0;
                _return_data[1] = 0;
                _return_data[2] = 0;
                _return_data[3] = 0;
              }
              else
              {
                _return_data[0] = receive_buffer[6]; // Старший 1
                _return_data[1] = receive_buffer[5]; // Младший 1
                _return_data[2] = receive_buffer[8]; // Старший 2
                _return_data[3] = receive_buffer[7]; // Младший 2
              }
            }
          }
        }
      }
    }
  }
}


// ########## PUBLIC ##########
/* 
 * @brief: начать соединение с приводом со скоростью 115200
 * @note: по умолчанию включается прием данных от шины
 */
void HiWonderSerialServo::begin(void)
{
  if (_use_single_pin_mode_)
  {
    if (_use_software_serial_)
    {
#if !defined(ESP32)
      _s_serial_->begin(HIWONDER_SERVO_SERIAL_BAUDRATE);
#endif
    }
    else
    {
#if defined(ARDUINO_ARCH_STM32)
      _h_serial_->setHalfDuplex(); // Переключение в полудуплексный режим работы
#endif
      _h_serial_->begin(HIWONDER_SERVO_SERIAL_BAUDRATE);
    }
  }
  else
  {
    pinMode(_pin_direction_, OUTPUT); // Вывод направления настроен на выход

    if (_use_software_serial_)
    {
#if !defined(ESP32)
      digitalWrite(_pin_direction_, LOW); // Переключение на прием данных
      _s_serial_->begin(HIWONDER_SERVO_SERIAL_BAUDRATE);
      _s_serial_->flush(); // Очистка буфера приема
#endif
    }
    else
    {
      digitalWrite(_pin_direction_, LOW); // Переключение на прием данных
      _h_serial_->begin(HIWONDER_SERVO_SERIAL_BAUDRATE);
      _h_serial_->flush(); // Очистка буфера приема
    }
  }
}
/* 
 * @brief: завершить соединение с приводом
 */
void HiWonderSerialServo::end(void)
{
  _use_single_pin_mode_ = false;
  _use_software_serial_ = false;
  _pin_direction_ = 0;

  if (_use_software_serial_)
  {
#if !defined(ESP32)
    _s_serial_->end();
#endif
  }
  else
  {
    _h_serial_->end();
  }
}
/* 
 * @brief: включить режим приема/передачи по одному контакту
 * @note: пооддерживается только контроллерами со встроенным USART
 *   в остальных случаях можно только передавать, но не примать данные от привода
 */
void HiWonderSerialServo::enableSinglePinMode(void)
{
  _use_single_pin_mode_ = true;
}
/* 
 * @brief: выключить режим приема/передачи по одному контакту
 * @note: метод обратный предыдущему
 */
void HiWonderSerialServo::disableSinglePinMode(void)
{
  _use_single_pin_mode_ = false;
}
/* 
 * @brief: проверить режим приема/передачи по одному контакту
 * @return: режим работы
 *   true - включен
 *   false - выключен
 */
bool HiWonderSerialServo::isUsingSinglePinMode(void)
{
  return _use_single_pin_mode_;
}
/* 
 * @brief: проверить на использование HardwareSerial или SoftwareSerial
 * @return:
 *   true - SoftwareSerial
 *   false - HardwareSerial
 */
bool HiWonderSerialServo::isUsingSoftwareSerial(void)
{
  return _use_software_serial_;
}
/* 
 * @brief: установить другой контакт переключения приема/передачи полудуплексного режима
 * @param _dir_pin: контакт перключения приема/передачи полудуплексного режима
 */
void HiWonderSerialServo::setHalfDuplexDirectionPin(uint8_t _dir_pin)
{
  if (!_use_single_pin_mode_)
  {
    _pin_direction_ = _dir_pin;
    pinMode(_pin_direction_, OUTPUT); // Вывод направления настроен на выход
  }
}
/* 
 * @brief: получить номер контакта переключения приема/передачи полудуплексного режима
 * @return: 
 *   1 - 255 : номер контакта
 *   0 : не назначен
 */
uint8_t HiWonderSerialServo::getHalfDuplexDirectionPin(void)
{
  if (!_use_single_pin_mode_)
  {
    return _pin_direction_;
  }
  else
  {
    return 0;
  }
}
/* 
 * @brief: переключиться в режим ПЕРЕДАЧИ данных через полудуплексный UART
 */
void HiWonderSerialServo::setHalfDuplexSerialModeSender(void)
{
  if (!_use_single_pin_mode_)
  {
    digitalWrite(_pin_direction_, HIGH);
  }
}
/* 
 * @brief: переключиться в режим ПРИЕМА данных через полудуплексный UART
 */
void HiWonderSerialServo::setHalfDuplexSerialModeReciever(void)
{
  if (!_use_single_pin_mode_)
  {
    digitalWrite(_pin_direction_, LOW);
  }
}
/* 
 * @brief: получить режим работы полудуплексного UART
 * @return: режим работы
 *   true - передатчик
 *   false - приемник
 */
bool HiWonderSerialServo::getHalfDuplexSerialMode(void)
{
  if (!_use_single_pin_mode_)
  {
    return (bool)digitalRead(_pin_direction_);
  }
  else
  {
    return false;
  }
}
/*
 * @brief: поворот вала привода в желаемое положение за желаемое время
 *   Запускает привод сразу!
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _position: желаемое положенение вала
 *   0 - 1000 (1500)
 * @param _time: время за которое вал должен переместиться в _position
 *   0 - 30000мс (0 - 30сек)
 * @note:
 *   SERVO_MOVE_TIME_WRITE
 */
void HiWonderSerialServo::setServoMoveTime(uint8_t _id, uint16_t _position, uint16_t _time)
{
  // Фильтр максимального значения положения. 0 - 1000 (1500)
  if (_position > HIWONDER_SERVO_SHAFT_POSITION_MAX)
  {
    _position = HIWONDER_SERVO_SHAFT_POSITION_MAX;
  }
  // Фильтр максимального значения времени. 0 - 30сек
  if (_time > HIWONDER_SERVO_MOVE_TIME_VALUE_MAX)
  {
    _time = HIWONDER_SERVO_MOVE_TIME_VALUE_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_MOVE_TIME_WRITE;
  packet[5] = _position;
  packet[6] = (_position >> 8);
  packet[7] = _time;
  packet[8] = (_time >> 8);
  packet[9] = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение поворота вала привода в желаемое положение за желаемое время
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @param *_position: установленное желаемое положение вала
 *   0 - 1000 (1500)
 * @param *_time: установленное желаемое время перемещения
 *   0 - 30000мс (0 - 30сек)
 * @note: получение через ссылку
 *   SERVO_MOVE_TIME_READ
 */
void HiWonderSerialServo::getServoMoveTime(uint8_t _id, uint16_t *_position, uint16_t *_time)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_MOVE_TIME_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES, _id);

  *_position = (packet[0] << 8) | packet[1];
  *_time = (packet[2] << 8) | packet[3];
}
/*
 * @brief: поворот вала привода в желаемое положение за желаемое время
 *   Запускает привод не сразу! Связан с servoMoveStart и servoMoveStop!
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _position: желаемое положенение вала
 *   0 - 1000 (1500)
 * @param _time: время за которое вал должен переместиться в _position
 *   0 - 30000мс (0 - 30сек)
 * @note:
 *   SERVO_MOVE_TIME_WAIT_WRITE
 */
void HiWonderSerialServo::setServoMoveTimeWait(uint8_t _id, uint16_t _position, uint16_t _time)
{
  // Фильтр максимального значения положения. 0 - 1000 (1500)
  if (_position > HIWONDER_SERVO_SHAFT_POSITION_MAX)
  {
    _position = HIWONDER_SERVO_SHAFT_POSITION_MAX;
  }
  // Фильтр максимального значения времени. 0 - 30сек
  if (_time > HIWONDER_SERVO_MOVE_TIME_VALUE_MAX)
  {
    _time = HIWONDER_SERVO_MOVE_TIME_VALUE_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_MOVE_TIME_WAIT_WRITE;
  packet[5] = _position;
  packet[6] = (_position >> 8);
  packet[7] = _time;
  packet[8] = (_time >> 8);
  packet[9] = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение поворота вала привода в желаемое положение за желаемое время
 *   Значения из регистра ожидания в связке с методами servoMoveStart и servoMoveStop!
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @param *_position: установленное желаемое положение вала
 *   0 - 1000 (1500)
 * @param *_time: установленное желаемое время перемещения
 *   0 - 30000мс (0 - 30сек)
 * @note: получение через ссылку
 *   SERVO_MOVE_TIME_WAIT_READ
 */
void HiWonderSerialServo::getServoMoveTimeWait(uint8_t _id, uint16_t *_position, uint16_t *_time)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_MOVE_TIME_WAIT_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES, _id);

  *_position = (packet[0] << 8) | packet[1];
  *_time = (packet[2] << 8) | packet[3];
}
/*
 * @brief: разрешить приводу начать движение
 *   Связан с servoMoveTimeWait!
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @note:
 *   SERVO_MOVE_START
 */
void HiWonderSerialServo::setServoMoveStart(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_MOVE_START;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: зарпетить приводу продолжать движение
 *   Связан с servoMoveTimeWait!
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @note:
 *   SERVO_MOVE_STOP
 */
void HiWonderSerialServo::setServoMoveStop(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_MOVE_STOP;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: установка нового номера привода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _new_id: новый номер для выбранного привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @note:
 *   SERVO_ID_WRITE
 */
void HiWonderSerialServo::setServoID(uint8_t _id, uint8_t _new_id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_ID_WRITE;
  packet[5] = _new_id;
  packet[6] = (~(packet[2] + packet[3] + packet[4] + packet[5])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение номера привода на шине
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @return: номер привода
 * @note:
 *   SERVO_ID_READ
 */
uint8_t HiWonderSerialServo::getServoID(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_ID_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES, _id);

  return (packet[0]); // 0 - если ошибка
}
/*
 * @brief: подстройка угла смещения
 *   сбрасывается после отключения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _offset_adjust: значение подстройки угла смещения
 *   от -125 до 125
 * @note:
 *   SERVO_ANGLE_OFFSET_ADJUST
 */
void HiWonderSerialServo::setServoAngleOffsetAdjust(uint8_t _id, int8_t _offset_adjust)
{
  // Фильтр минимального и максимального значения подстройки угла смещения. +-125
  if (_offset_adjust < HIWONDER_SERVO_ANGLE_OFFSET_MIN)
  {
    _offset_adjust = HIWONDER_SERVO_ANGLE_OFFSET_MIN;
  }
  if (_offset_adjust > HIWONDER_SERVO_ANGLE_OFFSET_MAX)
  {
    _offset_adjust = HIWONDER_SERVO_ANGLE_OFFSET_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_ANGLE_OFFSET_ADJUST;
  packet[5] = _offset_adjust;
  packet[6] = (~(packet[2] + packet[3] + packet[4] + packet[5])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: подстройка угла смещения
 *   сохраняется после отключения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _offset: значение угла смещения
 *   от -125 до 125
 * @note:
 *   SERVO_ANGLE_OFFSET_WRITE
 */
void HiWonderSerialServo::setServoAngleOffset(uint8_t _id, int8_t _offset)
{
  // Фильтр минимального и максимального значения угла смещения. +-125
  if (_offset < HIWONDER_SERVO_ANGLE_OFFSET_MIN)
  {
    _offset = HIWONDER_SERVO_ANGLE_OFFSET_MIN;
  }
  if (_offset > HIWONDER_SERVO_ANGLE_OFFSET_MAX)
  {
    _offset = HIWONDER_SERVO_ANGLE_OFFSET_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_ANGLE_OFFSET_WRITE;
  packet[5] = _offset;
  packet[6] = (~(packet[2] + packet[3] + packet[4] + packet[5])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение угла смещения
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: угол смещения
 *   от -125 до 125
 * @note:
 *   SERVO_ANGLE_OFFSET_READ
 */
int8_t HiWonderSerialServo::getServoAngleOffset(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_ANGLE_OFFSET_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES, _id);

  return ((int8_t)packet[0]); // 0 - если ошибка или отсутствует смещение
}
/*
 * @brief: ограничение максимального и минимального угла поворота вала
 *   сохраняется после отключения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _angle_min_limit: минимальное значение угла
 *   0 - 1000 (1500)
 * @param _angle_max_limit: минимальное значение угла
 *   0 - 1000 (1500)
 * @note:
 *   SERVO_ANGLE_LIMIT_WRITE
 */
void HiWonderSerialServo::setServoAngleLimit(uint8_t _id, uint16_t _angle_min_limit, uint16_t _angle_max_limit)
{
  // Фильтр минимального и максимального угла поворота вала. 0 - 1000 (1500)
  if (_angle_min_limit > HIWONDER_SERVO_SHAFT_POSITION_MAX)
  {
    _angle_min_limit = HIWONDER_SERVO_SHAFT_POSITION_MAX;
  }
  if (_angle_max_limit > HIWONDER_SERVO_SHAFT_POSITION_MAX)
  {
    _angle_max_limit = HIWONDER_SERVO_SHAFT_POSITION_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_ANGLE_LIMIT_WRITE;
  packet[5] = _angle_min_limit;
  packet[6] = (_angle_min_limit >> 8);
  packet[7] = _angle_max_limit;
  packet[8] = (_angle_max_limit >> 8);
  packet[9] = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение ограничения максимального и минимального угла поворота вала
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @param *_angle_min_limit: значение минимального угла
 *   0 - 1000 (1500)
 * @param *_angle_max_limit: значение максимального угла
 *   0 - 1000 (1500)
 * @note: получение через ссылку
 *   SERVO_ANGLE_LIMIT_READ
 */
void HiWonderSerialServo::getServoAngleLimit(uint8_t _id, uint16_t *_angle_min_limit, uint16_t *_angle_max_limit)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_ANGLE_LIMIT_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES, _id);

  *_angle_min_limit = (packet[0] << 8) | packet[1];
  *_angle_max_limit = (packet[2] << 8) | packet[3];
}
/*
 * @brief: установка ограничения максимального и минимального значения напряжения питания
 *   сохраняется после отключения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _voltage_min_limit: минимальное значение напряжения
 *   4500 - 14000
 * @param _voltage_max_limit: максимальное значение напряжения
 *   4500 - 14000
 * @note:
 *   SERVO_VIN_LIMIT_WRITE
 */
void HiWonderSerialServo::setServoVoltageLimit(uint8_t _id, uint16_t _voltage_min_limit, uint16_t _voltage_max_limit)
{
  // Фильтр минимального и максимального значения напряжения питания. 4500 - 14000
  if (_voltage_min_limit < HIWONDER_SERVO_VIN_LIMIT_MIN)
  {
    _voltage_min_limit = HIWONDER_SERVO_VIN_LIMIT_MIN;
  }
  if (_voltage_min_limit > HIWONDER_SERVO_VIN_LIMIT_MAX)
  {
    _voltage_min_limit = HIWONDER_SERVO_VIN_LIMIT_MAX;
  }
  if (_voltage_max_limit < HIWONDER_SERVO_VIN_LIMIT_MIN)
  {
    _voltage_max_limit = HIWONDER_SERVO_VIN_LIMIT_MIN;
  }
  if (_voltage_max_limit > HIWONDER_SERVO_VIN_LIMIT_MAX)
  {
    _voltage_max_limit = HIWONDER_SERVO_VIN_LIMIT_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_VIN_LIMIT_WRITE;
  packet[5] = _voltage_min_limit;
  packet[6] = (_voltage_min_limit >> 8);
  packet[7] = _voltage_max_limit;
  packet[8] = (_voltage_max_limit >> 8);
  packet[9] = (~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение ограничения максимального и минимального значения напряжения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @param *_voltage_min_limit: значение минимального напряжения
 *   4500 - 14000
 * @param *_voltage_max_limit: значение максимального напряжения
 *   4500 - 14000
 * @note: получение через ссылку
 *   SERVO_VIN_LIMIT_READ
 */
void HiWonderSerialServo::getServoVoltageLimit(uint8_t _id, uint16_t *_voltage_min_limit, uint16_t *_voltage_max_limit)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_VIN_LIMIT_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES, _id);

  *_voltage_min_limit = (packet[0] << 8) | packet[1];
  *_voltage_max_limit = (packet[2] << 8) | packet[3];
}
/*
 * @brief: ограничение максимального значения температуры привода
 *   сохраняется после отключения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _temperature_max_limit: максимальное значение температуры
 *   50 - 100
 * @note:
 *   SERVO_TEMP_MAX_LIMIT_WRITE
 */
void HiWonderSerialServo::setServoMaximumTemperatureLimit(uint8_t _id, uint8_t _temperature_max_limit)
{
  // Фильтр минимального и максимального значения пороговой температуры. 50 - 100
  if (_temperature_max_limit < HIWONDER_SERVO_TEMPERATURE_LIMIT_MIN)
  {
    _temperature_max_limit = HIWONDER_SERVO_TEMPERATURE_LIMIT_MIN;
  }
  if (_temperature_max_limit > HIWONDER_SERVO_TEMPERATURE_LIMIT_MAX)
  {
    _temperature_max_limit = HIWONDER_SERVO_TEMPERATURE_LIMIT_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_TEMP_MAX_LIMIT_WRITE;
  packet[5] = _temperature_max_limit;
  packet[6] = (~(packet[2] + packet[3] + packet[4] + packet[5])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение ограничения максимальноой температуры привода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: максимальная температура привода в град. С
 * @note:
 *   SERVO_TEMP_MAX_LIMIT_READ
 */
uint8_t HiWonderSerialServo::getServoMaximumTemperatureLimit(uint8_t _id) {
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_TEMP_MAX_LIMIT_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES, _id);

  return (packet[0]); // 0 - если ошибка
}
/*
 * @brief: получение температуры привода в реальном времени
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: температура привода в град. С
 * @note:
 *   SERVO_TEMP_READ
 */
uint8_t HiWonderSerialServo::getServoRealTimeTemperature(uint8_t _id) {
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_TEMP_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES, _id);

  return (packet[0]); // 0 - если ошибка
}
/*
 * @brief: получение напряжения питания привода в реальном времени
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: напряжение питания привода в мВ
 * @note:
 *   SERVO_VIN_READ
 */
uint16_t HiWonderSerialServo::getServoInputVoltage(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_VIN_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FIVE_BYTES, _id);

  return ((packet[0] << 8) | packet[1]); // 0 - если ошибка
}
/*
 * @brief: получение положения вала привода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: положение вала привода
 * @note:
 *   SERVO_POS_READ
 */
uint16_t HiWonderSerialServo::getServoPosition(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_POS_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FIVE_BYTES, _id);

  return ((packet[0] << 8) | packet[1]); // 0 - если ошибка
}
/*
 * @brief: выбор режима работы привода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _servo_motor_mode: режим работы привода. Серва или Мотор
 *   HIWONDER_SERVO_MODE
 *   HIWONDER_MOTOR_MODE
 * @param _servo_motor_mode: скорость и направление врещения. Только в режиме МОТОР!
 *   от -1000 до 1000
 * @note:
 *   SERVO_OR_MOTOR_MODE_WRITE
 */
void HiWonderSerialServo::setServoOrMotorMode(uint8_t _id, HiwonderServoOrMotorMode_t _servo_motor_mode, int16_t _motor_speed)
{
  // Фильтр минимального и максимального значения скорости. +-1000 (+ прямое направление, - обратное направление)
  if (_motor_speed < HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_MIN)
  {
    _motor_speed = HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_MIN;
  }
  if (_motor_speed > HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_MAX)
  {
    _motor_speed = HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_MAX;
  }
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_OR_MOTOR_MODE_WRITE;
  packet[5] = _servo_motor_mode;
  packet[6] = 0; // NULL по документации
  packet[7] = _motor_speed;
  packet[8] = (_motor_speed >> 8);
  packet[9] = (~(packet[2] + packet[3] + packet[4] + packet[5] + 0 + packet[7] + packet[8])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение режима работы привода и скорости вращения в рижиме мотора
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @param *_servo_motor_mode: рижим работы привода
 *   HIWONDER_SERVO_MODE
 *   HIWONDER_MOTOR_MODE
 * @param *_motor_speed: скорость и направление вращения в рижиме мотора
 *   от -1000 до 1000
 * @note: получение через ссылку
 *   SERVO_OR_MOTOR_MODE_READ
 */
void HiWonderSerialServo::getServoOrMotorMode(uint8_t _id, HiwonderServoOrMotorMode_t *_servo_motor_mode, int16_t *_motor_speed)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_OR_MOTOR_MODE_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES, _id);

  *_servo_motor_mode = (HiwonderServoOrMotorMode_t)packet[1]; // Не packet[0] потому что перестановка есть порядка старший младший байты в методе HW_ReceivePacket
  *_motor_speed = (int16_t)((packet[2] << 8) | packet[3]);
}
/*
 * @brief: нагрузить или разгрузить вал при включении питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _load_state: вал нагружен\разгружен
 *   HIWONDER_SERVO_UNLOAD_STATE
 *   HIWONDER_SERVO_LOAD_STATE
 * @note:
 *   SERVO_LOAD_OR_UNLOAD_WRITE
 */
void HiWonderSerialServo::setServoLoadOrUnloadState(uint8_t _id, HiwonderServoLoadOrUnloadState_t _load_state)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_LOAD_OR_UNLOAD_WRITE;
  packet[5] = _load_state;
  packet[6] = (~(packet[2] + packet[3] + packet[4] + packet[5])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: узнать будет вал нагружен или разгрузужен при включении питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: состояние вала
 *   HIWONDER_SERVO_UNLOAD_STATE
 *   HIWONDER_SERVO_LOAD_STATE
 *   
 * @note:
 *   SERVO_LOAD_OR_UNLOAD_READ
 */
HiwonderServoLoadOrUnloadState_t HiWonderSerialServo::getServoLoadOrUnloadState(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_LOAD_OR_UNLOAD_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES, _id);

  return ((HiwonderServoLoadOrUnloadState_t)packet[0]);
}
/*
 * @brief: управление состоянием встроенного светодиода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _state_led: светодиод включен\выключен
 *   HIWONDER_SERVO_LED_ON
 *   HIWONDER_SERVO_LED_OFF
 * @note:
 *   SERVO_LED_CTRL_WRITE
 */
void HiWonderSerialServo::setServoControlLED(uint8_t _id, HiwonderServoControlLED_t _led_state)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_LED_CTRL_WRITE;
  packet[5] = _led_state;
  packet[6] = (~(packet[2] + packet[3] + packet[4] + packet[5])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение состояния встроенного светодиода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: состояние светодиода
 *   HIWONDER_SERVO_LED_ON
 *   HIWONDER_SERVO_LED_OFF
 * @note:
 *   SERVO_LED_CTRL_READ
 */
HiwonderServoControlLED_t HiWonderSerialServo::getServoControlLED(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_LED_CTRL_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES, _id);

  return ((HiwonderServoControlLED_t)packet[0]);
}
/*
 * @brief: настроить встроенный светодиод мигать по одному из заделарированных событий
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD), 0xFE широковещательный
 * @param _led_error_code: код события для мигания встроенного светодиода
 *   HIWONDER_SERVO_ERROR_LED_NO_ALARM
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE
 *   HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_VOLTAGE
 *   HIWONDER_SERVO_ERROR_LED_LOKED_ROTOR
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_STALLED
 *   HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE_AND_STALLED
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_OVER_VOLTAGE_AND_STALLED
 * @note:
 *   SERVO_LED_ERROR_WRITE
 */
void HiWonderSerialServo::setServoErrorLED(uint8_t _id, HiwonderServoErorCodeLED_t _led_error_code)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA;
  packet[4] = HIWONDER_CMD_SERVO_LED_ERROR_WRITE;
  packet[5] = _led_error_code;
  packet[6] = (~(packet[2] + packet[3] + packet[4] + packet[5])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
}
/*
 * @brief: получение кода ошибки привода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: код ошибки
 *   HIWONDER_SERVO_ERROR_LED_NO_ALARM
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE
 *   HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_VOLTAGE
 *   HIWONDER_SERVO_ERROR_LED_LOKED_ROTOR
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_STALLED
 *   HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE_AND_STALLED
 *   HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_OVER_VOLTAGE_AND_STALLED
 * @note:
 *   SERVO_LED_ERROR_READ
 */
HiwonderServoErorCodeLED_t HiWonderSerialServo::getServoErrorLED(uint8_t _id)
{
  // Создание массива пакета
  uint8_t packet[HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION];
  // Заполнение масива пакета данными
  packet[0] = HIWONDER_SERVO_PACKET_HEADER;
  packet[1] = HIWONDER_SERVO_PACKET_HEADER;
  packet[2] = _id;
  packet[3] = HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA;
  packet[4] = HIWONDER_CMD_SERVO_LED_ERROR_READ;
  packet[5] = (~(packet[2] + packet[3] + packet[4])) & 0xFF;
  // Передача пакета с данными приводу
  HW_SendPacket(packet, HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA + HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION);
  // Получение данных от привода с проверками
  HW_ReceivePacket(packet, HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES, _id);

  return ((HiwonderServoErorCodeLED_t)packet[0]);
}


/* Extend methods (custom section) */

/*
 * @brief: получение поворота вала привода в желаемое положение
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: установленное желаемое положение вала
 *   0 - 1000 (1500)
 * @note:
 *   SERVO_MOVE_TIME_READ
 */
uint16_t HiWonderSerialServo::getServoPositionFromMoveTime(uint8_t _id)
{
  uint16_t position;
  getServoMoveTime(_id, &position, nullptr);

  return position;
}
/*
 * @brief: получение желаемого времени поворота вала привода в желаемое положение
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: установленное желаемое время поворота
 *   0 - 30000мс (0 - 30сек)
 * @note:
 *   SERVO_MOVE_TIME_READ
 */
uint16_t HiWonderSerialServo::getServoTime(uint8_t _id)
{
  uint16_t time;
  getServoMoveTime(_id, nullptr, &time);

  return time;
}
/*
 * @brief: получение поворота вала привода в желаемое положение
 *   Связан с servoMoveTimeWait!
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: установленное желаемое положение вала
 *   0 - 1000 (1500)
 * @note:
 *   SERVO_MOVE_TIME_WAIT_READ
 */
uint16_t HiWonderSerialServo::getServoPositionWait(uint8_t _id)
{
  uint16_t position_wait;
  getServoMoveTimeWait(_id, &position_wait, nullptr);

  return position_wait;
}
/*
 * @brief: получение желаемого времени поворота вала привода в желаемое положение
 *   Связан с servoMoveTimeWait!
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: установленное желаемое время поворота
 *   0 - 30000мс (0 - 30сек)
 * @note:
 *   SERVO_MOVE_TIME_WAIT_READ
 */
uint16_t HiWonderSerialServo::getServoTimeWait(uint8_t _id)
{
  uint16_t time_wait;
  getServoMoveTimeWait(_id, nullptr, &time_wait);

  return time_wait;
}
/*
 * @brief: получение ограничения минимального угла поворота вала
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: значение минимального угла
 *   0 - 1000 (1500)
 * @note:
 *   SERVO_ANGLE_LIMIT_READ
 */
uint16_t HiWonderSerialServo::getServoAngleLimitMin(uint8_t _id)
{
  uint16_t angle_lim_min;
  getServoAngleLimit(_id, &angle_lim_min, nullptr);

  return angle_lim_min;
}
/*
 * @brief: получение ограничения максимального угла поворота вала
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: значение максимального угла
 *   0 - 1000 (1500)
 * @note:
 *   SERVO_ANGLE_LIMIT_READ
 */
uint16_t HiWonderSerialServo::getServoAngleLimitMax(uint8_t _id)
{
  uint16_t angle_lim_max;
  getServoAngleLimit(_id, nullptr, &angle_lim_max);

  return angle_lim_max;
}
/*
 * @brief: получение ограничения минимального значения напряжения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: значение минимального напряжения питания
 *   4500 - 14000
 * @note:
 *   SERVO_VIN_LIMIT_READ
 */
uint16_t HiWonderSerialServo::getServoVoltageLimitMin(uint8_t _id)
{
  uint16_t voltage_lim_min;
  getServoVoltageLimit(_id, &voltage_lim_min, nullptr);

  return voltage_lim_min;
}
/*
 * @brief: получение ограничения максимального значения напряжения питания
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: значение максимального напряжения питания
 *   4500 - 14000
 * @note:
 *   SERVO_VIN_LIMIT_READ
 */
uint16_t HiWonderSerialServo::getServoVoltageLimitMax(uint8_t _id)
{
  uint16_t voltage_lim_max;
  getServoVoltageLimit(_id, nullptr, &voltage_lim_max);

  return voltage_lim_max;
}
/*
 * @brief: получение режима работы привода
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: рижим работы привода
 *   HIWONDER_SERVO_MODE
 *   HIWONDER_MOTOR_MODE
 * @note:
 *   SERVO_OR_MOTOR_MODE_READ
 */
HiwonderServoOrMotorMode_t HiWonderSerialServo::getMode(uint8_t _id)
{
  HiwonderServoOrMotorMode_t current_mode;
  getServoOrMotorMode(_id, &current_mode, nullptr);

  return current_mode;
}
/*
 * @brief: получение скорости вращения в режиме мотора
 * @param _id: номер привода на шине
 *   0 - 253 (0x00 ~ 0xFD)
 * @return: скорость и направление вращения в рижиме мотора
 *   от -1000 до 1000
 * @note:
 *   SERVO_OR_MOTOR_MODE_READ
 */
int16_t HiWonderSerialServo::getSpeedInMotorMode(uint8_t _id)
{
  int16_t current_speed;
  getServoOrMotorMode(_id, nullptr, &current_speed);

  return current_speed;
}
