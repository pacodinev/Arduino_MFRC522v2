/* SPDX-License-Identifier: LGPL-2.1 */
#pragma once

#include <HardwareSerial.h>
#include <MFRC522Driver.h>

class MFRC522DriverHardwareSerial : public MFRC522Driver {
private: 
  static byte baud_rate_to_serialspeedreg_val (unsigned long &baud_rate);

public:
  //using PCD_Register = MFRC522Constants::PCD_Register;
  /////////////////////////////////////////////////////////////////////////////////////
  // Functions for setting up the Arduino.
  /////////////////////////////////////////////////////////////////////////////////////
  
  bool init() override;
  bool post_reset_init() override;
  
  /////////////////////////////////////////////////////////////////////////////////////
  // Basic interface functions for communicating with the MFRC522.
  /////////////////////////////////////////////////////////////////////////////////////
  void PCD_WriteRegister(const PCD_Register reg, const byte value) override;
  void PCD_WriteRegister(const PCD_Register reg, const byte count, byte *const values) override;
  byte PCD_ReadRegister(const PCD_Register reg) override;
  void PCD_ReadRegister(const PCD_Register reg, const byte count, byte *const values, const byte rxAlign = 0) override;
  
#ifdef ESP32
  MFRC522DriverHardwareSerial(HardwareSerial &serial = Serial, unsigned long baud_rate=9600, 
                              int8_t rxPin=-1, int8_t txPin=-1)
                  : MFRC522Driver(), _serial(serial), baud_rate(), rxPin(rxPin), txPin(txPin)
  {
        baud_rate_to_serialspeedreg_val(baud_rate);
        this->baud_rate = baud_rate;
  }
#else
  MFRC522DriverHardwareSerial(HardwareSerial &serial = Serial, unsigned long baud_rate=9600)
                  : MFRC522Driver(), _serial(serial), baud_rate(0)
  {
        baud_rate_to_serialspeedreg_val(baud_rate);
        this->baud_rate = baud_rate;
  }
#endif

protected:
  // Serial instance.
  HardwareSerial &_serial;
  unsigned long baud_rate;
#ifdef ESP32
  int8_t rxPin, txPin;
#endif


};
