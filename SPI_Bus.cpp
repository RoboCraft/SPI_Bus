/* This is implementation of SPI_Bus library.
 *
 * (C) 2011 Artem Borisovskiy, bytefu@gmail.com
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

#include <inttypes.h>
#include <stdlib.h>
#include "SPI_Bus.h"
#include "Arduino.h"
#include "LineDriver.h"


void SPI_Bus::init(LineDriver *pin_driver, uint8_t bandwidth, Implementation impl_type,
  uint8_t clock_div, uint8_t bit_order,
  uint8_t select_pin, uint8_t clock_pin, uint8_t mosi_pin, uint8_t miso_pin)
{
  m_pins = (pin_driver ? pin_driver : DefaultLineDriver::getInstance());
  m_bandwidth = bandwidth;
  m_hardware_SPI = (impl_type == HARDWARE);
  m_clock_div = clock_div;
  m_bit_order = bit_order;
  m_select_pin = select_pin;
  m_clock_pin = clock_pin;
  m_mosi_pin = mosi_pin;
  m_miso_pin = miso_pin;
  m_selection_policy = SELECT_AROUND;
  m_buffer = reinterpret_cast<uint8_t*>(calloc(1, m_bandwidth));
  
  m_pins->lineConfig(m_select_pin, OUTPUT);
  m_pins->lineWrite(m_select_pin, HIGH);

  if (m_hardware_SPI)
    SPI.begin();
  else
  {
    m_pins->lineConfig(m_clock_pin, OUTPUT);
    m_pins->lineWrite(m_clock_pin, LOW);
    
    m_pins->lineConfig(m_mosi_pin, OUTPUT);
    m_pins->lineWrite(m_mosi_pin, LOW);
    
    m_pins->lineConfig(m_miso_pin, INPUT);
  }
}


void SPI_Bus::communicate(Operation op)
{
  if (!op || m_bandwidth < 1)
    return;
    
  if (m_hardware_SPI)
  {
    SPI.setClockDivider(m_clock_div);
    SPI.setBitOrder(m_bit_order);
  }

  if (m_selection_policy == SELECT_BEFORE || m_selection_policy == SELECT_AROUND)
    m_pins->lineWrite(m_select_pin, LOW);

  if (m_selection_policy == SELECT_BEFORE)
    m_pins->lineWrite(m_select_pin, HIGH);
  
  (this->*op)();

  if (m_selection_policy == SELECT_AFTER)
    m_pins->lineWrite(m_select_pin, LOW);
  
  if (m_selection_policy == SELECT_AROUND || m_selection_policy == SELECT_AFTER)
    m_pins->lineWrite(m_select_pin, HIGH);
}


void SPI_Bus::operationSendBuffer()
{
  const uint8_t *p_current_byte = (m_bit_order == MSBFIRST ? m_buffer + (m_bandwidth - 1) : m_buffer);
  
  while ((m_bit_order == MSBFIRST ? (p_current_byte >= m_buffer) : (p_current_byte - m_buffer < m_bandwidth)))
  {
    if (m_hardware_SPI)
      SPI.transfer(*p_current_byte);
    else
      softwareWrite(*p_current_byte);
    
    m_bit_order == MSBFIRST ? --p_current_byte : ++p_current_byte;
  }
}


void SPI_Bus::operationReceiveEntireBuffer()
{
  uint8_t *p_current_byte = (m_bit_order == MSBFIRST ? m_buffer + (m_bandwidth - 1) : m_buffer);
  
  while ((m_bit_order == MSBFIRST ? (p_current_byte >= m_buffer) : (p_current_byte - m_buffer < m_bandwidth)))
  {
    if (m_hardware_SPI)
      *p_current_byte = SPI.transfer(0);
    else
      *p_current_byte = softwareRead();
    
    m_bit_order == MSBFIRST ? --p_current_byte : ++p_current_byte;
  }
}


void SPI_Bus::operationFullDuplexTrasfer()
{
  uint8_t *p_current_byte = (m_bit_order == MSBFIRST ? m_buffer + (m_bandwidth - 1) : m_buffer);
  
  while ((m_bit_order == MSBFIRST ? (p_current_byte >= m_buffer) : (p_current_byte - m_buffer < m_bandwidth)))
  {
    if (m_hardware_SPI)
      *p_current_byte = SPI.transfer(*p_current_byte);
    else
      *p_current_byte = softwareTransfer(*p_current_byte);
    
    m_bit_order == MSBFIRST ? --p_current_byte : ++p_current_byte;
  }
}


uint8_t reverse8bits(uint8_t bits)
{
  uint8_t tmp;
  
  /* bits = (bits & 0x55) << 1 | (bits & 0xAA) >> 1; */
  asm volatile("mov     %0, %1"   : "=d"(tmp) : "r"(bits));
  asm volatile("andi    %0, %1"   : "=d"(bits) : "M"(0x55), "0"(bits));
  asm volatile("lsl     %0"       : "=d"(bits) : "0"(bits));
  asm volatile("andi    %0, %1"   : "=d"(tmp) : "M"(0xAA), "0"(tmp));
  asm volatile("lsr     %0"       : "=d"(tmp) : "0"(tmp));
  asm volatile("or      %0, %1"   : "=d"(bits): "r"(tmp), "0"(bits));
  
  /* bits = (bits & 0x33) << 2 | (bits & 0xCC) >> 2; */
  asm volatile("mov     %0, %1"   : "=d"(tmp) : "r"(bits));
  asm volatile("andi    %0, %1"   : "=d"(bits) : "M"(0x33), "0"(bits));
  asm volatile("lsl     %0"       : "=d"(bits) : "0"(bits));
  asm volatile("lsl     %0"       : "=d"(bits) : "0"(bits));
  asm volatile("andi    %0, %1"   : "=d"(tmp) : "M"(0xCC), "0"(tmp));
  asm volatile("lsr     %0"       : "=d"(tmp) : "0"(tmp));
  asm volatile("lsr     %0"       : "=d"(tmp) : "0"(tmp));
  asm volatile("or      %0, %1"   : "=d"(bits) : "r"(tmp), "0"(bits));
  
  /* bits = (bits & 0x0F) << 4 | (bits & 0xF0) >> 4; */
  asm volatile("swap    %0"       : "=d"(bits) : "0"(bits));

  return bits;
}


uint8_t SPI_Bus::softwareRead()
{
  uint8_t value = 0;

  for (uint8_t i = 0; i < 8; ++i)
  {
    m_pins->lineWrite(m_clock_pin, HIGH);
    
    value <<= 1;
    value |= (m_pins->lineRead(m_miso_pin) & 0x01);

    m_pins->lineWrite(m_clock_pin, LOW);	
  }

  if (m_bit_order == LSBFIRST)
    value = reverse8bits(value);
  
  return value;
}


void SPI_Bus::softwareWrite(uint8_t data)
{
  if (m_bit_order == MSBFIRST)
    data = reverse8bits(data);

  for (uint8_t i = 0; i < 8; i++)
  {
    m_pins->lineWrite(m_mosi_pin, data & 0x01);
    data >>= 1;
    
    m_pins->lineWrite(m_clock_pin, HIGH);
    m_pins->lineWrite(m_clock_pin, LOW);		
  }
}


uint8_t SPI_Bus::softwareTransfer(uint8_t data)
{
  uint8_t input = 0;

  if (m_bit_order == MSBFIRST)
    data = reverse8bits(data);

  for (uint8_t i = 0; i < 8; i++)
  {
    m_pins->lineWrite(m_mosi_pin, data & 0x01);
    data >>= 1;
    
    m_pins->lineWrite(m_clock_pin, HIGH);
    
    input <<= 1;
    input |= (m_pins->lineRead(m_miso_pin) & 0x01);
    
    m_pins->lineWrite(m_clock_pin, LOW);		
  }
  
  return input;
}


void SPI_Bus::clearBufferFrom(uint8_t pos)
{
  memset(m_buffer + pos, 0, m_bandwidth - pos);
}

  
SPI_Bus::SPI_Bus(uint8_t bandwidth, uint8_t select_pin,
  uint8_t bit_order, LineDriver *pin_driver)
{
  init(pin_driver, bandwidth, HARDWARE, SPI_CLOCK_DIV4, bit_order, select_pin, 0xFFu, 0xFFu, 0xFFu);
}


SPI_Bus::SPI_Bus(uint8_t bandwidth, uint8_t select_pin,
  uint8_t clock_pin, uint8_t mosi_pin, uint8_t miso_pin,
  uint8_t bit_order, LineDriver *pin_driver)
{
  init(pin_driver, bandwidth, SOFTWARE, SPI_CLOCK_DIV4, bit_order,
    select_pin, clock_pin, mosi_pin, miso_pin);
}


SPI_Bus::SPI_Bus(const SPI_Bus &prototype)
{
  init(prototype.m_pins, prototype.m_bandwidth, prototype.m_hardware_SPI ? HARDWARE : SOFTWARE,
    prototype.m_clock_div, prototype.m_bit_order, prototype.m_select_pin,
    prototype.m_clock_pin, prototype.m_mosi_pin, prototype.m_miso_pin);
}


void SPI_Bus::write(const void *data)
{
  memcpy(m_buffer, data, m_bandwidth);
  communicate(&SPI_Bus::operationSendBuffer);
}


void SPI_Bus::write(const SPI_Bus &right)
{
  memcpy(m_buffer, right.m_buffer, min(m_bandwidth, right.m_bandwidth));

  if (m_bandwidth > right.m_bandwidth)
    clearBufferFrom(m_bandwidth - right.m_bandwidth);

  communicate(&SPI_Bus::operationSendBuffer);
}


void SPI_Bus::write(uint8_t data)
{
  if (m_bandwidth >= 1)
  {
    *m_buffer = data;
    clearBufferFrom(1);
    communicate(&SPI_Bus::operationSendBuffer);
  }
}


void SPI_Bus::write(uint16_t data)
{
  if (m_bandwidth == 1)
  {
    if (m_bandwidth >= sizeof(data))
    {
      *reinterpret_cast<uint16_t*>(m_buffer) = data;
      clearBufferFrom(sizeof(data));
    }
    else
      *m_buffer = static_cast<uint8_t>(data & 0xFF);

    communicate(&SPI_Bus::operationSendBuffer);
  }
}


void SPI_Bus::write(const uint32_t &data)
{
  if (m_bandwidth == 1)
  {
    if (m_bandwidth >= sizeof(data))
    {
      *reinterpret_cast<uint32_t*>(m_buffer) = data;
      clearBufferFrom(sizeof(data));
    }
    else
      memcpy(m_buffer, &m_buffer, m_bandwidth);
    
    communicate(&SPI_Bus::operationSendBuffer);
  }
}


void SPI_Bus::write(const uint64_t &data)
{
  if (m_bandwidth == 1)
  {
    if (m_bandwidth >= sizeof(data))
    {
      *reinterpret_cast<uint64_t*>(m_buffer) = data;
      clearBufferFrom(sizeof(data));
    }
    else
      memcpy(m_buffer, &m_buffer, m_bandwidth);
  
    communicate(&SPI_Bus::operationSendBuffer);
  }
}


uint8_t SPI_Bus::read8bits()
{
  if (m_bandwidth >= 1)
  {
    communicate(&SPI_Bus::operationReceiveEntireBuffer);
    return *m_buffer;
  }

  return 0;
}


uint16_t SPI_Bus::read16bits()
{
  uint16_t data = 0;
  
  communicate(&SPI_Bus::operationReceiveEntireBuffer);
  
  if (m_bandwidth >= 2)
    data = *reinterpret_cast<uint16_t*>(m_buffer);
  else if (m_bandwidth == 1)
    data = *m_buffer;

  return 0;
}


uint32_t SPI_Bus::read32bits()
{
  uint32_t data = 0;

  communicate(&SPI_Bus::operationReceiveEntireBuffer);
  
  if (m_bandwidth >= sizeof(data))
    data = *reinterpret_cast<uint32_t*>(m_buffer);
  else
    memcpy(&data, m_buffer, m_bandwidth);

  return data;
}


uint64_t SPI_Bus::read64bits()
{
  uint64_t data = 0;

  communicate(&SPI_Bus::operationReceiveEntireBuffer);
  memcpy(&data, m_buffer, m_bandwidth);
  
  if (m_bandwidth < sizeof(data))
    memset(&data + m_bandwidth, 0, sizeof(data) - m_bandwidth);

  return data;
}


const uint8_t* SPI_Bus::read()
{
  communicate(&SPI_Bus::operationReceiveEntireBuffer);
  return m_buffer;
}


uint8_t SPI_Bus::bandwidth() const
{
  return m_bandwidth;
}


const uint8_t* SPI_Bus::getBuffer() const
{
  return m_buffer;
}


void SPI_Bus::setBitOrder(uint8_t bit_order)
{
  m_bit_order = bit_order;
}


void SPI_Bus::setClockDivider(uint8_t clock_divider)
{
  m_clock_div = clock_divider;
}


void SPI_Bus::setMode(uint8_t mode)
{
  if (m_hardware_SPI)
    SPI.setDataMode(mode);
}


void SPI_Bus::setImplementation(Implementation type)
{
  m_hardware_SPI = type != SOFTWARE;
}


void SPI_Bus::setSelectionPolicy(SelectionPolicy policy)
{
  switch (policy)
  {
    case SELECT_BEFORE:
    case SELECT_AROUND:
    case SELECT_AFTER:
      m_selection_policy = policy;
      break;

    default:
      m_selection_policy = SELECT_NONE;
  }
}


void SPI_Bus::lineConfig(uint8_t pin, uint8_t mode)
{
  /* There's nothing to do if you use either the parallel-out or parallel-load shift register.
   * Otherwise, you may need to implement this function.
   */
}


void SPI_Bus::lineWrite(uint8_t line_num, uint8_t value)
{
  if (line_num < m_bandwidth * 8)
  {
    const uint8_t byte_index = line_num / 8,
                  bit_index = line_num % 8;
    
    m_buffer[byte_index] &= ~(1 << bit_index);
    m_buffer[byte_index] |= (value == LOW ? 0 : 1) << bit_index;
    
    communicate(&SPI_Bus::operationSendBuffer);
  }
}


uint8_t SPI_Bus::lineRead(uint8_t line_num)
{
  if (line_num >= m_bandwidth * 8)
    return LOW;

  communicate(&SPI_Bus::operationReceiveEntireBuffer);

  const uint8_t byte_index = line_num / 8,
                bit_index = line_num % 8;

  return ((m_buffer[byte_index] >> bit_index) & 1) ? HIGH : LOW;
}
