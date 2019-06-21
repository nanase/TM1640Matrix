#include "tm1640matrix.h"

#if (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__))

#define CLK_LOW() (*this->clk_port &= ~this->clk_mask)
#define CLK_HIGH() (*this->clk_port |= this->clk_mask)

#define DAT_LOW() (*this->data_port &= ~this->data_mask)
#define DAT_HIGH() (*this->data_port |= this->data_mask)

static void setPinConfig(uint8_t pin, volatile uint8_t **port, uint8_t *mask)
{
  if (pin < 8)
  {
    *port = &PORTD;
    *mask = 1 << pin;
    DDRD |= *mask;
  }
  else if (pin < 14)
  {
    *port = &PORTB;
    *mask = 1 << (pin - 8);
    DDRB |= *mask;
  }
  else if (pin < 20)
  {
    *port = &PORTC;
    *mask = 1 << (pin - 14);
    DDRC |= *mask;
  }
  else
  {
    // error
    while (1)
      ;
  }
}

#else // defined(__AVR_ATmega328P__)

#define CLK_LOW() (digitalWrite(this->clk_pin, LOW))
#define CLK_HIGH() (digitalWrite(this->clk_pin, HIGH))

#define DAT_LOW() (digitalWrite(this->data_pin, LOW))
#define DAT_HIGH() (digitalWrite(this->data_pin, HIGH))

static void setPinConfig(uint8_t pin)
{
  pinMode(pin, OUTPUT);
}

#endif // defined(__AVR_ATmega328P__)

// -----------------------------------------------------------------------------

TM1640Matrix::TM1640Matrix(uint8_t clk_pin, uint8_t data_pin, int8_t brightness)
{
#if (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__))

  setPinConfig(clk_pin, &clk_port, &clk_mask);
  setPinConfig(data_pin, &data_port, &data_mask);

#else // defined(__AVR_ATmega328P__)

  this->clk_pin = clk_pin;
  this->data_pin = data_pin;
  setPinConfig(clk_pin);
  setPinConfig(data_pin);

#endif // defined(__AVR_ATmega328P__)

  CLK_HIGH();
  DAT_HIGH();

  sendCommand(TM1640_CMD_ADDRESS_AUTO);
  clear();
  setBrightness(brightness);
};

void TM1640Matrix::setBrightness(int8_t brightness)
{
  this->brightness = constrain(brightness, -1, 7);

  if (brightness < 0)
    sendCommand(TM1640_MSK_DISPLAY_OFF);
  else
    sendCommand(TM1640_MSK_DISPLAY_ON | this->brightness);
}

int8_t TM1640Matrix::getBrightness()
{
  return this->brightness;
}

void TM1640Matrix::setDot(uint8_t x, uint8_t y, uint8_t dot)
{
  if (dot & 1)
    this->data[x] |= 1 << y;
  else
    this->data[x] &= ~(1 << y);
}

void TM1640Matrix::setLine(uint8_t x, uint8_t line)
{
  this->data[x] = line;
}

void TM1640Matrix::flush()
{
  sendCommand(TM1640_CMD_ADDRESS_AUTO);
  DAT_LOW();
  send(TM1640_CMD_ADDRESS_00H);

  for (uint8_t i = 0; i < 16; i++)
    send(this->data[i]);

  DAT_HIGH();
}

void TM1640Matrix::clear()
{
  sendCommand(TM1640_CMD_ADDRESS_AUTO);
  DAT_LOW();
  send(TM1640_CMD_ADDRESS_00H);

  for (uint8_t i = 0; i < 16; i++)
    send(0x00);

  DAT_HIGH();
}

// -----------------------------------------------------------------------------

void TM1640Matrix::send(uint8_t data)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    CLK_LOW();
    (data & 1 ? DAT_HIGH() : DAT_LOW());
    CLK_HIGH();
    data >>= 1;
  }
}

void TM1640Matrix::sendCommand(uint8_t data)
{
  // for stable
  DAT_LOW();
  CLK_LOW();
  CLK_HIGH();
  DAT_HIGH();

  DAT_LOW();
  send(data);
  DAT_HIGH();

  // for stable
  DAT_LOW();
  CLK_LOW();
  CLK_HIGH();
  DAT_HIGH();
}

void TM1640Matrix::sendData(uint8_t address, uint8_t data)
{
  sendCommand(TM1640_CMD_ADDRESS_FIXED);
  DAT_LOW();
  send(TM1640_MSK_ADDRESS | address);
  send(data);
  DAT_HIGH();
}
