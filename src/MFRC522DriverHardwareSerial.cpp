/* SPDX-License-Identifier: LGPL-2.1 */
#include "MFRC522DriverHardwareSerial.h"

byte MFRC522DriverHardwareSerial::baud_rate_to_serialspeedreg_val (unsigned long &baud_rate)
{
    if(baud_rate < 9600)
    {
        baud_rate = 7200;
        return 0xFA;
    }
    else if(baud_rate < 14400)
    {
        baud_rate = 9600;
        return 0xEB;
    }
    else if(baud_rate < 19200)
    {
        baud_rate = 14400;
        return 0xDA;
    }
    else if(baud_rate < 38400)
    {
        baud_rate = 19200;
        return 0xCB;
    }
    else if(baud_rate < 57600)
    {
        baud_rate = 38400;
        return 0xAB;
    }
    else if(baud_rate < 115200)
    {
        baud_rate = 57600;
        return 0x9A;
    }
    else if(baud_rate < 128000)
    {
        baud_rate = 115200;
        return 0x7A;
    }
    else if(baud_rate < 230400)
    {
        baud_rate = 128000;
        return 0x74;
    }
    else if(baud_rate < 460800)
    {
        baud_rate = 230400;
        return 0x5A;
    }
    else if(baud_rate < 921600)
    {
        baud_rate = 460800;
        return 0x3A;
    }
    else if(baud_rate < 1228800)
    {
        baud_rate = 921600;
        return 0x1C;
    }
    else
    {
        baud_rate = 1228800;
        return 0x15;
    }
}

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522DriverHardwareSerial
/////////////////////////////////////////////////////////////////////////////////////

bool MFRC522DriverHardwareSerial::init() {
  // TODO avoid double init.
#ifdef ESP32
  _serial.begin(9600, SERIAL_8N1, rxPin, txPin);
#else
  _serial.begin(9600, SERIAL_8N1);
#endif 

  // disable MX and DTRQ registers
  byte val = PCD_ReadRegister(PCD_Register::TestPinEnReg);
  val = val & 0x7F;
  PCD_WriteRegister(PCD_Register::TestPinEnReg, val);

  if(baud_rate != 9600)
  {
    byte speed_reg_val = baud_rate_to_serialspeedreg_val(baud_rate);
    PCD_WriteRegister(PCD_Register::SerialSpeedReg, speed_reg_val);
#ifdef ESP32
    _serial.begin(baud_rate, SERIAL_8N1, rxPin, txPin); 
#else
    _serial.begin(baud_rate, SERIAL_8N1); 
#endif 
  }

  return true;
}

bool MFRC522DriverHardwareSerial::post_reset_init()
{
    return init();
}

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4. 
 */
void MFRC522DriverHardwareSerial::PCD_WriteRegister(const PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte value           ///< The value to write.
                                        ) {
  while(_serial.available()>0)
    _serial.read();
  _serial.write(reg & 0x3F);
  while(_serial.available() == 0);
  _serial.read(); // discard address
  _serial.write(value);
} // End PCD_WriteRegister().

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4.
 */
void MFRC522DriverHardwareSerial::PCD_WriteRegister(const MFRC522Constants::PCD_Register reg,    ///< The register to write to. One of the PCD_Register enums.
                                         const byte count,        ///< The number of bytes to write to the register.
                                         byte *const values        ///< The values to write. Byte array.
                                        ) {
  // Sanity check.
  if(count == 0 || values == nullptr) {
    return;
  }

  for(byte i=0; i<count; i++)
  {
    PCD_WriteRegister(reg, values[i]);
  }
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.4.
 */
byte MFRC522DriverHardwareSerial::PCD_ReadRegister(const PCD_Register reg    ///< The register to read from. One of the PCD_Register enums.
                                       ) {
  byte value;

  while(_serial.available()>0)
    _serial.read();
  _serial.write((reg & 0x3F) | 0x80);
  while(_serial.available() == 0);
  value = _serial.read();
  
  return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverHardwareSerial::PCD_ReadRegister(const PCD_Register reg,    ///< The register to read from. One of the PCD_Register enums.
                                        const byte count,          ///< The number of bytes to read.
                                        byte *const values,        ///< Byte array to store the values in.
                                        const byte rxAlign         ///< Only bit positions rxAlign..7 in values[0] are updated.
                                       ) {
  // Sanity check.
  if(count <= 0 || values == nullptr) {
    return;
  }

  byte i=0;
  if(rxAlign != 0)
  {
    byte mask  = (byte)(0xFF << rxAlign) & 0xFF;

    // Read value and tell that we want to read the same address again.
    byte value = (byte)PCD_ReadRegister(reg); // returns int but only with uint8 content
      
    // Apply mask to both current value of values[0] and the new data in value.
    values[0] = (values[0] & ~mask) | (value & mask); 
    i++;
  }

  for(; i<count; i++)
  {
    values[i] = PCD_ReadRegister(reg);
  }
  
  // Fixme: flush wire if still data available?
} // End PCD_ReadRegister()
