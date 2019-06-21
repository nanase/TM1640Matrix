#pragma once

#include "Arduino.h"

#define TM1640_CMD_ADDRESS_AUTO 0x40
#define TM1640_CMD_ADDRESS_FIXED 0x44
#define TM1640_CMD_ADDRESS_00H 0xc0

#define TM1640_MSK_ADDRESS 0xc0
#define TM1640_MSK_DISPLAY_OFF 0x80
#define TM1640_MSK_DISPLAY_ON 0x88

class TM1640Matrix
{
public:
  TM1640Matrix(uint8_t clk_pin, uint8_t data_pin, int8_t brightness = 7);

  virtual void setBrightness(int8_t brightness);
  virtual int8_t getBrightness();

  virtual void setDot(uint8_t x, uint8_t y, uint8_t dot);
  virtual void setLine(uint8_t x, uint8_t line);

  virtual void flush();
  virtual void clear();

private:
  virtual void send(uint8_t data);
  virtual void sendData(uint8_t address, uint8_t data);
  virtual void sendCommand(uint8_t data);

  int8_t brightness;
  uint8_t data[16];

#if (defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__))

  volatile uint8_t *clk_port;
  uint8_t clk_mask;
  volatile uint8_t *data_port;
  uint8_t data_mask;

#else // defined(__AVR_ATmega328P__)

  uint8_t clk_pin;
  uint8_t data_pin;

#endif // defined(__AVR_ATmega328P__)
};
