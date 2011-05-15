#include "SPI_Bus.h"
#include <inttypes.h>
#include <stdlib.h>
#include "WProgram.h"
#include "LineDriver.h"


void SPI_Bus::init(LineDriver *pin_driver, uint8_t bandwidth, Implementation impl_type,
  uint8_t clock_div, uint8_t select_pin, uint8_t clock_pin, uint8_t data_pin, uint8_t bit_order)
{
  m_pins = (pin_driver ? pin_driver :DefaultLineDriver::getInstance());
  m_bandwidth = bandwidth;
  m_hardware_SPI = (impl_type == HARDWARE);
  m_clock_div = clock_div;
  m_clock_pin = clock_pin;
  m_data_pin = data_pin;
  m_select_pin = select_pin;
  m_bit_order = bit_order;
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
    
    m_pins->lineConfig(m_data_pin, OUTPUT);
    m_pins->lineWrite(m_data_pin, LOW);
  }
}


void SPI_Bus::operationSendBuffer()
{
  const uint8_t *p_current_byte = (m_bit_order == MSBFIRST ? m_buffer + m_bandwidth : m_buffer);
  
  while ((m_bit_order == MSBFIRST ? (p_current_byte >= m_buffer) : (p_current_byte - m_buffer < m_bandwidth)))
  {
    if (m_hardware_SPI)
      SPI.transfer(*p_current_byte);
    else
      softwareWrite(m_data_pin, m_clock_pin, m_bit_order, *p_current_byte);
    
    m_bit_order == MSBFIRST ? --p_current_byte : ++p_current_byte;
  }
}


void SPI_Bus::operationReceiveFullBuffer()
{
  uint8_t *p_current_byte = (m_bit_order == MSBFIRST ? m_buffer + m_bandwidth : m_buffer);
  
  while ((m_bit_order == MSBFIRST ? (p_current_byte >= m_buffer) : (p_current_byte - m_buffer < m_bandwidth)))
  {
    if (m_hardware_SPI)
      *p_current_byte = SPI.transfer(0);
    else
      *p_current_byte = softwareRead(m_data_pin, m_clock_pin, m_bit_order);
    
    m_bit_order == MSBFIRST ? --p_current_byte : ++p_current_byte;
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


uint8_t SPI_Bus::softwareRead(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder)
{
  uint8_t value = 0;
  uint8_t i;

  for (i = 0; i < 8; ++i)
  {
    m_pins->lineWrite(clockPin, HIGH);
    
    if (bitOrder == LSBFIRST)
      value |= m_pins->lineRead(dataPin) << i;
    else
      value |= m_pins->lineRead(dataPin) << (7 - i);
    
    m_pins->lineWrite(clockPin, LOW);
  }
  
  return value;
}


void SPI_Bus::softwareWrite(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
  uint8_t i;

  for (i = 0; i < 8; i++)
  {
    if (bitOrder == LSBFIRST)
      m_pins->lineWrite(dataPin, !!(val & (1 << i)));
    else
      m_pins->lineWrite(dataPin, !!(val & (1 << (7 - i))));
    
    m_pins->lineWrite(clockPin, HIGH);
    m_pins->lineWrite(clockPin, LOW);		
  }
}


void SPI_Bus::clearBufferFrom(uint8_t pos)
{
  memset(m_buffer + pos, 0, m_bandwidth - pos);
}

  
SPI_Bus::SPI_Bus(uint8_t bandwidth, uint8_t select_pin,
  uint8_t bit_order, LineDriver *pin_driver)
{
  init(pin_driver, bandwidth, HARDWARE, SPI_CLOCK_DIV4,
    select_pin, 0xFFu, 0xFFu, bit_order);
}


SPI_Bus::SPI_Bus(uint8_t bandwidth, uint8_t select_pin,
  uint8_t clock_pin, uint8_t data_pin,
  uint8_t bit_order, LineDriver *pin_driver)
{
  init(pin_driver, bandwidth, SOFTWARE, SPI_CLOCK_DIV4,
    select_pin, clock_pin, data_pin, bit_order);
}


SPI_Bus::SPI_Bus(const SPI_Bus &prototype)
{
  init(prototype.m_pins, prototype.m_bandwidth, prototype.m_hardware_SPI ? HARDWARE : SOFTWARE,
    prototype.m_clock_div, prototype.m_select_pin, prototype.m_clock_pin,
    prototype.m_data_pin, prototype.m_bit_order);
}


SPI_Bus& SPI_Bus::operator=(const void *data)
{
  memcpy(m_buffer, data, m_bandwidth);
  communicate(&SPI_Bus::operationSendBuffer);
  
  return *this;
}


SPI_Bus& SPI_Bus::operator=(const SPI_Bus &right)
{
  memcpy(m_buffer, right.m_buffer, min(m_bandwidth, right.m_bandwidth));

  if (m_bandwidth > right.m_bandwidth)
    clearBufferFrom(m_bandwidth - right.m_bandwidth);

  communicate(&SPI_Bus::operationSendBuffer);
  
  return *this;
}


SPI_Bus& SPI_Bus::operator=(uint8_t data)
{
  if (m_bandwidth >= 1)
  {
    *m_buffer = data;
    clearBufferFrom(1);
    communicate(&SPI_Bus::operationSendBuffer);
  }
  
  return *this;
}


SPI_Bus& SPI_Bus::operator=(uint16_t data)
{
  if (m_bandwidth >= sizeof(data))
  {
    *reinterpret_cast<uint16_t*>(m_buffer) = data;
    clearBufferFrom(sizeof(data));
    communicate(&SPI_Bus::operationSendBuffer);
  }
  else if (m_bandwidth == 1)
  {
    *m_buffer = static_cast<uint8_t>(data & 0xFF);
    communicate(&SPI_Bus::operationSendBuffer);
  }
  
  return *this;
}


SPI_Bus& SPI_Bus::operator=(const uint32_t &data)
{
  if (m_bandwidth >= sizeof(data))
  {
    *reinterpret_cast<uint32_t*>(m_buffer) = data;
    clearBufferFrom(sizeof(data));
    communicate(&SPI_Bus::operationSendBuffer);
  }
  else
    memcpy(m_buffer, &m_buffer, m_bandwidth);
  
  communicate(&SPI_Bus::operationSendBuffer);
  
  return *this;
}


SPI_Bus& SPI_Bus::operator=(const uint64_t &data)
{
  if (m_bandwidth >= sizeof(data))
  {
    *reinterpret_cast<uint64_t*>(m_buffer) = data;
    clearBufferFrom(sizeof(data));
    communicate(&SPI_Bus::operationSendBuffer);
  }
  else
    memcpy(m_buffer, &m_buffer, m_bandwidth);
  
  communicate(&SPI_Bus::operationSendBuffer);
  
  return *this;
}


uint8_t SPI_Bus::read8bits()
{
  if (m_bandwidth >= 1)
  {
    communicate(&SPI_Bus::operationReceiveFullBuffer);
    return *m_buffer;
  }

  return 0;
}


uint16_t SPI_Bus::read16bits()
{
  uint16_t data = 0;
  
  communicate(&SPI_Bus::operationReceiveFullBuffer);
  
  if (m_bandwidth >= 2)
    data = *reinterpret_cast<uint16_t*>(m_buffer);
  else if (m_bandwidth == 1)
    data = *m_buffer;

  return 0;
}


uint32_t SPI_Bus::read32bits()
{
  uint32_t data = 0;

  communicate(&SPI_Bus::operationReceiveFullBuffer);
  
  if (m_bandwidth >= sizeof(data))
    data = *reinterpret_cast<uint32_t*>(m_buffer);
  else
    memcpy(&data, m_buffer, m_bandwidth);

  return data;
}


uint64_t SPI_Bus::read64bits()
{
  uint64_t data = 0;

  communicate(&SPI_Bus::operationReceiveFullBuffer);
  
  if (m_bandwidth >= sizeof(data))
    data = *reinterpret_cast<uint64_t*>(m_buffer);
  else
    memcpy(&data, m_buffer, m_bandwidth);

  return data;
}


const uint8_t* SPI_Bus::read()
{
  communicate(&SPI_Bus::operationReceiveFullBuffer);
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


void SPI_Bus::lineConfig(uint8_t pin, uint8_t mode)
{
  /* There's nothing to do if you use either the parallel-out or parallel-load shift register.
   * Otherwise, you may need to implement this function.
   */
}


void SPI_Bus::lineWrite(uint8_t bit, uint8_t value)
{
  if (bit < m_bandwidth * 8)
  {
    const uint8_t byte_index = bit / 8, bit_index = bit % 8;
    
    m_buffer[byte_index] &= ~(1 << bit_index);
    m_buffer[byte_index] |= ((value == LOW ? 0 : 1) << bit_index);
    
    communicate(&SPI_Bus::operationSendBuffer);
  }
}


uint8_t SPI_Bus::lineRead(uint8_t pin)
{
  if (pin >= m_bandwidth * 8)
    return 0;

  return (m_buffer[pin / 8] >> pin % 8) & 0x01;
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
