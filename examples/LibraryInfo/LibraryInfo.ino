/*
 * LibraryInfo
 * 
 * Получение информации о библиотеке 
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2026 / v0.1.0 / License MIT / Скляр Роман S-LAB
 */

 
#include "HiWonderSerialServo.h"

void setup() {
  Serial.begin(115200);
  while(!Serial); // Для МК с аппаратной поддержкой USB
  
  delay(1000); // Когда нет аппаратной поддержки USB

  Serial.print("Library Version: ");
  Serial.println(HIWONDER_CLASS_LIBRARY_VERSION);
  Serial.print("Support Models: ");
  Serial.println(HIWONDER_CLASS_SERVO_MODEL_SUPPORT);
  Serial.print("Manufacturer Alias: ");
  Serial.println(HIWONDER_CLASS_SERVO_MANUFACTURER_ALIASES);
  Serial.print("Manufacturer Site Link: ");
  Serial.println(HIWONDER_CLASS_SERVO_MANUFACTURER_SITE_LINK);
  Serial.println();
}

void loop() {
  // Ничего
}
