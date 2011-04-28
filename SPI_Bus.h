#ifndef SPI_BUS_H
#define SPI_BUS_H

enum
{
  _8bit = 1,
  _16bit = 2,
  _24bit = 3,
  _32bit = 4,
  _64bit = 8,
  _128bit = 16
};

template <uint8_t bandwidth_>
class SPI_Bus
{
private:
  enum Mode { HARDWARE, SOFTWARE };
  
  bool m_hardware_SPI;
  uint8_t m_clock_div; // hardware SPI
  uint8_t m_clock_pin; // software SPI
  uint8_t m_data_pin;  // software SPI
  uint8_t m_select_pin;
  uint8_t m_bit_order;
  uint8_t m_data[bandwidth_];
  
  void init()
  {
    memset(m_data, 0, sizeof(m_data));
    
    pinMode(m_select_pin, OUTPUT);
    ::digitalWrite(m_select_pin, HIGH);
  }
  
  void update()
  {
    SPI.setClockDivider(m_clock_div);
    SPI.setBitOrder(m_bit_order);
    
    ::digitalWrite(m_select_pin, LOW);
    
    const uint8_t *p_current_byte = (m_bit_order == MSBFIRST ? m_data + bandwidth_ : m_data);
    
    while ((m_bit_order == MSBFIRST ? (p_current_byte >= m_data) : (p_current_byte - m_data < bandwidth_)))
    {
      if (m_hardware_SPI)
        SPI.transfer(*p_current_byte);
      else
        shiftOut(m_data_pin, m_clock_pin, m_bit_order, *p_current_byte);
      
      m_bit_order == MSBFIRST ? --p_current_byte : ++p_current_byte;
    }
    
    ::digitalWrite(m_select_pin, HIGH);
  }
  
public:
  SPI_Bus(uint8_t select_pin, uint8_t bit_order = MSBFIRST):
    m_hardware_SPI(true), m_bit_order(bit_order), m_clock_div(SPI_CLOCK_DIV4),
    m_select_pin(select_pin), m_clock_pin(-1), m_data_pin(-1)
  {
    init();
    SPI.begin();
  }
  
  SPI_Bus(uint8_t select_pin, uint8_t clock_pin, uint8_t data_pin, uint8_t bit_order = MSBFIRST):
    m_hardware_SPI(false), m_bit_order(bit_order), m_clock_div(1),
    m_select_pin(select_pin), m_clock_pin(clock_pin), m_data_pin(data_pin)
  {
    init();
    
    pinMode(m_clock_pin, OUTPUT);
    ::digitalWrite(m_clock_pin, LOW);
    
    pinMode(m_data_pin, OUTPUT);
    ::digitalWrite(m_data_pin, LOW);
  }
  
  SPI_Bus(const SPI_Bus &prototype)
  {
    m_data = prototype.m_data;
  }
  
  SPI_Bus& operator=(const void *data)
  {
    memcpy(m_data, data, sizeof(m_data));
    update();
    
    return *this;
  }
  
  SPI_Bus& operator=(const SPI_Bus &right)
  {
    m_data = right.m_data;
    
    return *this;
  }
  
  SPI_Bus& operator=(uint8_t data)
  {
    *m_data = data;
    update();
    
    return *this;
  }
  
  SPI_Bus& operator=(uint16_t data)
  {
    if (bandwidth_ >= 2)
      *reinterpret_cast<uint16_t*>(m_data) = data;
    else
      *m_data = static_cast<uint8_t>(data);
    
    update();
    
    return *this;
  }
  
  SPI_Bus& operator=(uint32_t data)
  {
    if (bandwidth_ >= 4)
      *reinterpret_cast<uint32_t*>(m_data) = data;
    else if (bandwidth_ >= 2)
    {
      if (bandwidth_ == 3)
        m_data[3] = highByte(data);
      
      *reinterpret_cast<uint16_t*>(m_data) = static_cast<uint16_t>(data);
    }
    
    update();
    
    return *this;
  }
  
  void digitalWrite(uint8_t bit, uint8_t value)
  {
    if (bit < bandwidth_ * 8)
    {
      const uint8_t byte_index = bit / 8, bit_index = bit % 8;
      
      m_data[byte_index] &= ~(1 << bit_index);
      m_data[byte_index] |= ((value == LOW ? 0 : 1) << bit_index);
      
      update();
    }
  }
  
  const void* getValue() const
  {
    return m_data;
  }

  uint8_t bandwidth() const
  {
    return bandwidth_;
  }
  
  void setBitOrder(uint8_t bit_order)
  {
    m_bit_order = bit_order;
  }
  
  void setClockDivider(uint8_t clock_divider)
  {
    m_clock_div = clock_divider;
  }
  
  void setMode(Mode mode)
  {
    m_hardware_SPI = mode != SOFTWARE;
  }
};

typedef SPI_Bus<_8bit> SPI_8bit_Bus;
typedef SPI_Bus<_16bit> SPI_16bit_Bus;
typedef SPI_Bus<_24bit> SPI_24bit_Bus;
typedef SPI_Bus<_32bit> SPI_32bit_Bus;
typedef SPI_Bus<_64bit> SPI_64bit_Bus;
typedef SPI_Bus<_128bit> SPI_128bit_Bus;

#endif // SPI_BUS_H_
