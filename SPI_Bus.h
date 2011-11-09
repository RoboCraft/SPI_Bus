/* This is header file for SPI_Bus library which gives the programmer
 * more convinient way of working with SPI than the standard SPI library.
 * More strictly speaking, SPI_Bus is built on top of it.
 *
 * Copyright (C) 2011 Artem Borisovskiy (bytefu@gmail.com), http://robocraft.ru
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <inttypes.h>
#include "Arduino.h"
#include "SPI.h"
#include "LineDriver.h"

enum
{
  _8bit = 1,
  _16bit = 2,
  _24bit = 3,
  _32bit = 4,
  _64bit = 8,
};


class SPI_Bus: public LineDriver
{
public:
  enum Implementation { HARDWARE, SOFTWARE };
  enum SelectionPolicy { SELECT_NONE, SELECT_BEFORE, SELECT_AROUND, SELECT_AFTER };

  SPI_Bus(uint8_t bandwidth, uint8_t select_pin,
    uint8_t bit_order = MSBFIRST, LineDriver *pin_driver = 0);

  SPI_Bus(uint8_t bandwidth, uint8_t select_pin,
    uint8_t clock_pin, uint8_t mosi_pin, uint8_t miso_pin,
    uint8_t bit_order = MSBFIRST, LineDriver *pin_driver = 0);

  SPI_Bus(const SPI_Bus &prototype);

  void write(const void *data);
  void write(const SPI_Bus &right);
  void write(uint8_t data);
  void write(uint16_t data);
  void write(const uint32_t &data);
  void write(const uint64_t &data);

  uint8_t read8bits();
  uint16_t read16bits();
  uint32_t read32bits();
  uint64_t read64bits();
  const uint8_t* read();

  uint8_t fullDuplexTransfer(uint8_t);
  uint16_t fullDuplexTransfer(uint16_t);
  uint32_t fullDuplexTransfer(uint32_t);
  uint64_t fullDuplexTransfer(uint64_t);
  const uint8_t* fullDuplexTransfer(const uint8_t *data);
  
  uint8_t bandwidth() const;
  const uint8_t* getBuffer() const;

  void setBitOrder(uint8_t bit_order);
  void setClockDivider(uint8_t clock_divider);
  void setMode(uint8_t mode); // only supported for hardware SPI implementation
  void setImplementation(Implementation type);
  void setSelectionPolicy(SelectionPolicy policy);

  virtual void lineConfig(uint8_t pin, uint8_t mode);
  virtual void lineWrite(uint8_t bit, uint8_t value);
  virtual uint8_t lineRead(uint8_t pin);

private:
  typedef void (SPI_Bus::*Operation)();

  LineDriver *m_pins;
  uint8_t m_bandwidth;
  bool m_hardware_SPI;
  uint8_t m_clock_div; // hardware SPI
  uint8_t m_clock_pin; // software SPI
  uint8_t m_mosi_pin;  // software SPI
  uint8_t m_miso_pin;  // software SPI
  uint8_t m_select_pin;
  uint8_t m_bit_order;
  SelectionPolicy m_selection_policy;
  uint8_t *m_buffer;

  void init(LineDriver *pin_driver, uint8_t bandwidth, Implementation impl_type,
    uint8_t clock_div, uint8_t bit_order,
    uint8_t select_pin, uint8_t clock_pin, uint8_t mosi_pin, uint8_t miso_pin);

  void communicate(Operation op);
  void operationSendBuffer();
  void operationReceiveEntireBuffer();
  void operationFullDuplexTrasfer();
  
  uint8_t softwareRead();
  void softwareWrite(uint8_t data);
  uint8_t softwareTransfer(uint8_t data);
  void clearBufferFrom(uint8_t pos);
};

#endif // SPI_BUS_H_
