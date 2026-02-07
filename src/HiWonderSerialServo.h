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


#pragma once
#include "Arduino.h"
#include <HardwareSerial.h>

#if !defined(ESP32)
#include <SoftwareSerial.h>
#endif

#define HIWONDER_CLASS_LIBRARY_VERSION "0.0.1"
#define HIWONDER_CLASS_SERVO_MODEL_SUPPORT "LX-824, LX-824HV, LX-224, LX-224HV, LX-225, LX-1501, LX-15D, LX-16A,\nHX-06L, HX-12H, HX-20L, HX-35H, HX-35HM,\nHTS-16L, HTS-20L, HTS-20H, HTS-21H, HTS-30HS, HTS-35H,\nHTD-35H, HTD-455H"
#define HIWONDER_CLASS_SERVO_MANUFACTURER_ALIASES "HiWonder, LOBOT, LewanSoul"
#define HIWONDER_CLASS_SERVO_MANUFACTURER_SITE_LINK "https://www.hiwonder.com/collections/bus-servo?page=1"

// Скорость UART. Для этих приводов она не меняется
const uint32_t HIWONDER_SERVO_SERIAL_BAUDRATE = 115200;

// Широковещательный ID
const uint8_t HIWONDER_SERVO_BROADCAST_ID = 0xFE; // 254

// Заголовок пакета. Используется дважды
const uint8_t HIWONDER_SERVO_PACKET_HEADER = 0x55;

// Максимальное количество попыток проверки данных в применом буфере
const uint8_t HIWONDER_SERVO_ATTEMPTS_COUNT_MAX = 10;

// Максимальное время проверки буфера примема (микросекунды)
const uint16_t HIWONDER_SERVO_TIMEOUT_CHEKING_BUFFER_MAX = 1000;

// Минимальное количество байт в буфере приема
// Header1 + Header2 + ID + Length + Command + ONE Parameter + Checksum
const uint8_t HIWONDER_SERVO_RECIEVED_BYTE_BUFFER_MIN = 7;

// Коэффициент длинны пакета. Включает в себя 2 байта HEADER и 1 байт CRC
const uint8_t HIWONDER_PACKET_LENGHT_FACTOR_CORRECTION = 3;

// Максимальное положение вала в режиме серопривода (для сервоприводов с потенциометром - 1000)
const uint16_t HIWONDER_SERVO_SHAFT_POSITION_MAX = 1500;
// Максимальное время перемещения вала в рижиме сервопривода
const uint16_t HIWONDER_SERVO_MOVE_TIME_VALUE_MAX = 30000;
// Минимальное значение смещения угла
const int8_t HIWONDER_SERVO_ANGLE_OFFSET_MIN = -125;
// Максимальное значение смещения угла
const int8_t HIWONDER_SERVO_ANGLE_OFFSET_MAX = 125;
// Минимальное значение ограничения напряжения питания (4.5V)
const uint16_t HIWONDER_SERVO_VIN_LIMIT_MIN = 4500;
// Максимальное значение ограничения напряжения питания (14.0V)
const uint16_t HIWONDER_SERVO_VIN_LIMIT_MAX = 14000;
// Минимальное значение ограничения температуры привода
const uint8_t HIWONDER_SERVO_TEMPERATURE_LIMIT_MIN = 50;
// Максимальное значение ограничения температуры привода
const uint8_t HIWONDER_SERVO_TEMPERATURE_LIMIT_MAX = 100;

// Скорость по умолчанию для режимов серво и мотор
const int8_t HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_DEFAULT = 0;
// Минимальная скорость для режимов серво и мотор
const int16_t HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_MIN = -1000;
// Максимальная скорость для режимов серво и мотор
const int16_t HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_MAX = 1000;

// Значения команд
const uint8_t HIWONDER_CMD_SERVO_MOVE_TIME_WRITE = 1;
const uint8_t HIWONDER_CMD_SERVO_MOVE_TIME_READ = 2;
const uint8_t HIWONDER_CMD_SERVO_MOVE_TIME_WAIT_WRITE = 7;
const uint8_t HIWONDER_CMD_SERVO_MOVE_TIME_WAIT_READ = 8;
const uint8_t HIWONDER_CMD_SERVO_MOVE_START = 11;
const uint8_t HIWONDER_CMD_SERVO_MOVE_STOP = 12;
const uint8_t HIWONDER_CMD_SERVO_ID_WRITE = 13;
const uint8_t HIWONDER_CMD_SERVO_ID_READ = 14;
const uint8_t HIWONDER_CMD_SERVO_ANGLE_OFFSET_ADJUST = 17;
const uint8_t HIWONDER_CMD_SERVO_ANGLE_OFFSET_WRITE = 18;
const uint8_t HIWONDER_CMD_SERVO_ANGLE_OFFSET_READ = 19;
const uint8_t HIWONDER_CMD_SERVO_ANGLE_LIMIT_WRITE = 20;
const uint8_t HIWONDER_CMD_SERVO_ANGLE_LIMIT_READ = 21;
const uint8_t HIWONDER_CMD_SERVO_VIN_LIMIT_WRITE = 22;
const uint8_t HIWONDER_CMD_SERVO_VIN_LIMIT_READ = 23;
const uint8_t HIWONDER_CMD_SERVO_TEMP_MAX_LIMIT_WRITE = 24;
const uint8_t HIWONDER_CMD_SERVO_TEMP_MAX_LIMIT_READ = 25;
const uint8_t HIWONDER_CMD_SERVO_TEMP_READ = 26;
const uint8_t HIWONDER_CMD_SERVO_VIN_READ = 27;
const uint8_t HIWONDER_CMD_SERVO_POS_READ = 28;
const uint8_t HIWONDER_CMD_SERVO_OR_MOTOR_MODE_WRITE = 29;
const uint8_t HIWONDER_CMD_SERVO_OR_MOTOR_MODE_READ = 30;
const uint8_t HIWONDER_CMD_SERVO_LOAD_OR_UNLOAD_WRITE = 31;
const uint8_t HIWONDER_CMD_SERVO_LOAD_OR_UNLOAD_READ = 32;
const uint8_t HIWONDER_CMD_SERVO_LED_CTRL_WRITE = 33;
const uint8_t HIWONDER_CMD_SERVO_LED_CTRL_READ = 34;
const uint8_t HIWONDER_CMD_SERVO_LED_ERROR_WRITE = 35;
const uint8_t HIWONDER_CMD_SERVO_LED_ERROR_READ = 36;

// Значения длины команд отправляемого пакета данных
const uint8_t HIWONDER_CMD_LENGHT_SERVO_ONE_BYTE_DATA = 3;
const uint8_t HIWONDER_CMD_LENGHT_SERVO_TWO_BYTES_DATA = 4;
const uint8_t HIWONDER_CMD_LENGHT_SERVO_FOUR_BYTES_DATA = 7;

// Количество байтов запрашиваемых данных в принимаем пакете
const uint8_t HIWONDER_SERVO_LENGHT_PACKET_FOUR_BYTES = 4;
const uint8_t HIWONDER_SERVO_LENGHT_PACKET_FIVE_BYTES = 5;
const uint8_t HIWONDER_SERVO_LENGHT_PACKET_SEVEN_BYTES = 7;

// Режимы привода - Серва, Мотор
enum HiwonderServoOrMotorMode_t : uint8_t
{
  HIWONDER_SERVO_MODE,
  HIWONDER_MOTOR_MODE
};

// Нагрузить или разгрузить вал привода
enum HiwonderServoLoadOrUnloadState_t : uint8_t
{
  HIWONDER_SERVO_UNLOAD_STATE,
  HIWONDER_SERVO_LOAD_STATE
};

// Включение или выключение встроенного светодиода
enum HiwonderServoControlLED_t : uint8_t
{
  HIWONDER_SERVO_LED_ON,
  HIWONDER_SERVO_LED_OFF
};

// Коды ошибок привода
enum HiwonderServoErorCodeLED_t : uint8_t
{
  HIWONDER_SERVO_ERROR_LED_NO_ALARM,
  HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE,
  HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE,
  HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_VOLTAGE,
  HIWONDER_SERVO_ERROR_LED_LOKED_ROTOR,
  HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_STALLED,
  HIWONDER_SERVO_ERROR_LED_OVER_VOLTAGE_AND_STALLED,
  HIWONDER_SERVO_ERROR_LED_OVER_TEMPERATURE_AND_OVER_VOLTAGE_AND_STALLED
};


class HiWonderSerialServo
{
  private:
    HardwareSerial *_h_serial_ = NULL;
#if !defined(ESP32)
    SoftwareSerial *_s_serial_ = NULL;
#endif

    uint8_t _pin_direction_ = 0;
    bool _use_single_pin_mode_ = false;
    bool _use_software_serial_ = false;

  protected:
    virtual void HW_SendPacket(unsigned char *_packet, unsigned int _length);
    virtual void HW_ReceivePacket(uint8_t *_return_data, uint8_t _answer_lenght, uint8_t _id);

  public:
    HiWonderSerialServo(HardwareSerial *_h_serial, uint8_t _dir_pin);
    HiWonderSerialServo(HardwareSerial *_h_serial);
#if !defined(ESP32)
    HiWonderSerialServo(SoftwareSerial *_s_serial, uint8_t _dir_pin);
    HiWonderSerialServo(SoftwareSerial *_s_serial);
#endif

    virtual void begin(void);
    virtual void end(void);

    virtual void enableSinglePinMode(void);
    virtual void disableSinglePinMode(void);
    
    virtual bool isUsingSinglePinMode(void);
    virtual bool isUsingSoftwareSerial(void);

    virtual void setHalfDuplexDirectionPin(uint8_t _dir_pin);
    virtual uint8_t getHalfDuplexDirectionPin(void);

    virtual void setHalfDuplexSerialModeSender(void);
    virtual void setHalfDuplexSerialModeReciever(void);
    virtual bool getHalfDuplexSerialMode(void);

    // SERVO_MOVE_TIME_WRITE
    virtual void setServoMoveTime(uint8_t _id, uint16_t _position, uint16_t _time);
    // SERVO_MOVE_TIME_READ
    virtual void getServoMoveTime(uint8_t _id, uint16_t *_position, uint16_t *_time);

    // SERVO_MOVE_TIME_WAIT_WRITE
    virtual void setServoMoveTimeWait(uint8_t _id, uint16_t _position, uint16_t _time);
    // SERVO_MOVE_TIME_WAIT_READ
    virtual void getServoMoveTimeWait(uint8_t _id, uint16_t *_position, uint16_t *_time);

    // SERVO_MOVE_START
    virtual void setServoMoveStart(uint8_t _id);

    // SERVO_MOVE_STOP
    virtual void setServoMoveStop(uint8_t _id);

    // SERVO_ID_WRITE
    virtual void setServoID(uint8_t _id, uint8_t _new_id);
    // SERVO_ID_READ
    virtual uint8_t getServoID(uint8_t _id);

    // SERVO_ANGLE_OFFSET_ADJUST
    virtual void setServoAngleOffsetAdjust(uint8_t _id, int8_t _offset_adjust);

    // SERVO_ANGLE_OFFSET_WRITE
    virtual void setServoAngleOffset(uint8_t _id, int8_t _offset);
    // SERVO_ANGLE_OFFSET_READ
    virtual int8_t getServoAngleOffset(uint8_t _id);

    // SERVO_ANGLE_LIMIT_WRITE
    virtual void setServoAngleLimit(uint8_t _id, uint16_t _angle_min_limit, uint16_t _angle_max_limit);
    // SERVO_ANGLE_LIMIT_READ
    virtual void getServoAngleLimit(uint8_t _id, uint16_t *_angle_min_limit, uint16_t *_angle_max_limit);

    // SERVO_VIN_LIMIT_WRITE
    virtual void setServoVoltageLimit(uint8_t _id, uint16_t _voltage_min_limit, uint16_t _voltage_max_limit);
    // SERVO_VIN_LIMIT_READ
    virtual void getServoVoltageLimit(uint8_t _id, uint16_t *_voltage_min_limit, uint16_t *_voltage_max_limit);

    // SERVO_TEMP_MAX_LIMIT_WRITE
    virtual void setServoMaximumTemperatureLimit(uint8_t _id, uint8_t _temperature_max_limit);
    // SERVO_TEMP_MAX_LIMIT_READ
    virtual uint8_t getServoMaximumTemperatureLimit(uint8_t _id);

    // SERVO_TEMP_READ
    virtual uint8_t getServoRealTimeTemperature(uint8_t _id);
    // SERVO_VIN_READ
    virtual uint16_t getServoInputVoltage(uint8_t _id);
    // SERVO_POS_READ
    virtual uint16_t getServoPosition(uint8_t _id);

    // SERVO_OR_MOTOR_MODE_WRITE
    virtual void setServoOrMotorMode(uint8_t _id, HiwonderServoOrMotorMode_t _servo_motor_mode, int16_t _motor_speed = HIWONDER_SERVO_OR_MOTOR_MODE_SPEED_DEFAULT);
    // SERVO_OR_MOTOR_MODE_READ
    virtual void getServoOrMotorMode(uint8_t _id, HiwonderServoOrMotorMode_t *_servo_motor_mode, int16_t *_motor_speed);

    // SERVO_LOAD_OR_UNLOAD_WRITE
    virtual void setServoLoadOrUnloadState(uint8_t _id, HiwonderServoLoadOrUnloadState_t _load_state);
    // SERVO_LOAD_OR_UNLOAD_READ
    virtual HiwonderServoLoadOrUnloadState_t getServoLoadOrUnloadState(uint8_t _id);

    // SERVO_LED_CTRL_WRITE
    virtual void setServoControlLED(uint8_t _id, HiwonderServoControlLED_t _led_state);
    // SERVO_LED_CTRL_READ
    virtual HiwonderServoControlLED_t getServoControlLED(uint8_t _id);

    // SERVO_LED_ERROR_WRITE
    virtual void setServoErrorLED(uint8_t _id, HiwonderServoErorCodeLED_t _led_error_code);
    // SERVO_LED_ERROR_READ
    virtual HiwonderServoErorCodeLED_t getServoErrorLED(uint8_t _id);


    /* Extend methods (not original HiWonder protocol) */

    // SERVO_MOVE_TIME_READ
    virtual uint16_t getServoPositionFromMoveTime(uint8_t _id);
    virtual uint16_t getServoTime(uint8_t _id);

    // SERVO_MOVE_TIME_WAIT_READ
    virtual uint16_t getServoPositionWait(uint8_t _id);
    virtual uint16_t getServoTimeWait(uint8_t _id);
    
    // SERVO_ANGLE_LIMIT_READ
    virtual uint16_t getServoAngleLimitMin(uint8_t _id);
    virtual uint16_t getServoAngleLimitMax(uint8_t _id);

    // SERVO_VIN_LIMIT_READ
    virtual uint16_t getServoVoltageLimitMin(uint8_t _id);
    virtual uint16_t getServoVoltageLimitMax(uint8_t _id);

    // SERVO_OR_MOTOR_MODE_READ
    virtual HiwonderServoOrMotorMode_t getMode(uint8_t _id);
    virtual int16_t getSpeedInMotorMode(uint8_t _id);
};
