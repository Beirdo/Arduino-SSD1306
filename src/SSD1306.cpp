/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to
interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

/*
 * Reworked by Gavin Hurlbut <gjhurlbu@gmail.com> to use an attached SPI FRAM
 * for buffer storage, default to 128x64 display, I2C only
 *
 * changes (c) 2016 Gavin Hurlbut <gjhurlbu@gmail.com>
 * released under the BSD License
 */

#ifdef __AVR__
  #include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32)
 #include <pgmspace.h>
#else
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) && !defined(ESP32) && !defined(__arc__)
 #include <util/delay.h>
#endif

#include <stdlib.h>

#include <Wire.h>
#include "Adafruit_GFX.h"
#include "SSD1306.h"
#include "Adafruit_FRAM_SPI.h"

// the memory buffer for the LCD
// Move the initial logo display into FLASH

const PROGMEM uint8_t lcd_logo[SSD1306_RAM_MIRROR_SIZE] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF,
#if (SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH > 96*16)
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#if (SSD1306_LCDHEIGHT == 64)
0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x0F,
0x87, 0xC7, 0xF7, 0xFF, 0xFF, 0x1F, 0x1F, 0x3D, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0x7C, 0x7D, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x00, 0x30, 0x30, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xC0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x1F,
0x0F, 0x07, 0x1F, 0x7F, 0xFF, 0xFF, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0,
0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00,
0x00, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x0E, 0xFC, 0xF8, 0x00, 0x00, 0xF0, 0xF8, 0x1C, 0x0E,
0x06, 0x06, 0x06, 0x0C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFC,
0xFE, 0xFC, 0x00, 0x18, 0x3C, 0x7E, 0x66, 0xE6, 0xCE, 0x84, 0x00, 0x00, 0x06, 0xFF, 0xFF, 0x06,
0x06, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x06, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0xC0, 0xF8,
0xFC, 0x4E, 0x46, 0x46, 0x46, 0x4E, 0x7C, 0x78, 0x40, 0x18, 0x3C, 0x76, 0xE6, 0xCE, 0xCC, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x03,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x07, 0x0E, 0x0C,
0x18, 0x18, 0x0C, 0x06, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x01, 0x0F, 0x0E, 0x0C, 0x18, 0x0C, 0x0F,
0x07, 0x01, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x07,
0x07, 0x0C, 0x0C, 0x18, 0x1C, 0x0C, 0x06, 0x06, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
#endif
};

#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }

void SSD1306::initializeLogo(void)
{
  if (!m_fram) {
    return;
  }
  
  for (uint16_t i = 0; i < SSD1306_RAM_MIRROR_SIZE; i += SSD1306_BUFFER_SIZE) {
    memcpy_P(m_buffer, lcd_logo + i, SSD1306_BUFFER_SIZE);
#if 0
    Serial.print("Logo  i: ");
    Serial.print(i, HEX);
    Serial.print(" addr: ");
    Serial.println(m_logo_addr + i, HEX);
#endif
    m_fram->writeEnable(true);
    m_fram->write(m_logo_addr + i, m_buffer, SSD1306_BUFFER_SIZE);
#if 0
    for (uint8_t j = 0; j < SSD1306_BUFFER_SIZE; j++) {
      Serial.print(m_buffer[j], HEX);
      if ((j & 0x0F) == 0x0F) {
        Serial.println("");
      } else {
        Serial.print(" ");
      }
    }
#endif
  }
}

void SSD1306::attachRAM(Adafruit_FRAM_SPI *fram, uint16_t buffer,
                                 uint16_t logo)
{
  m_fram = fram;
  m_buffer_addr = buffer;
  m_logo_addr = logo;
  m_show_logo = true;
  m_pages_empty = 0;
  m_cache_clean = true;
  m_cache_address = 0xFFFF;
}

void SSD1306::getCacheLine(int16_t x, int16_t y)
{
  (void)x;
  y &= 0xFFF0;
  uint16_t addr = SSD1306_PIXEL_ADDR(0, y);
  if (!m_show_logo) {
    if (addr == m_cache_address) {
      return;
    }

    if (!m_cache_clean) {
      flushCacheLine();
    }
  }
  
  uint16_t baseAddr = (m_show_logo ? m_logo_addr : m_buffer_addr);

#if 0
  Serial.print("Reading: ");
  Serial.println(baseAddr + addr, HEX);
#endif

  bool empty1 = !(!(m_pages_empty & SSD1306_PAGE_BIT(y)));
  bool empty2 = !(!(m_pages_empty & SSD1306_PAGE_BIT(y + 8)));

  if (!empty1) {
    m_fram->read(baseAddr + addr, m_buffer, SSD1306_BUFFER_SIZE);
  } else {
    memset(m_buffer, 0x00, SSD1306_BUFFER_SIZE);
  }

  if (!empty2) {
    m_fram->read(baseAddr + addr + SSD1306_BUFFER_SIZE,
                 &m_buffer[SSD1306_BUFFER_SIZE], SSD1306_BUFFER_SIZE);
  } else {
    memset(&m_buffer[SSD1306_BUFFER_SIZE], 0x00, SSD1306_BUFFER_SIZE);
  }

#if 0
  for (uint8_t i = 0; i < SSD1306_BUFFER_SIZE; i++) {
    Serial.print(m_buffer[i], HEX);
    if ((i & 0x0F) == 0x0F) {
      Serial.println("");
    } else {
      Serial.print(" ");
    }
  }
#endif
  m_cache_address = addr;
  m_cache_clean = true;
}

void SSD1306::flushCacheLine(void)
{
  if (m_show_logo) {
    return;
  }
#if 0
  Serial.print("Writing: ");
  Serial.println(m_buffer_addr + m_cache_address, HEX);
#endif

  m_fram->writeEnable(true);
  m_fram->write(m_buffer_addr + m_cache_address, m_buffer, 
                2 * SSD1306_BUFFER_SIZE);
#if 0
  for (uint8_t i = 0; i < SSD1306_BUFFER_SIZE; i++) {
    Serial.print(m_buffer[i], HEX);
    if ((i & 0x0F) == 0x0F) {
      Serial.println("");
    } else {
      Serial.print(" ");
    }
  }
#endif
  m_cache_clean = true;
}

// the most basic function, set a single pixel
void SSD1306::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
    return;

  // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 1:
    ssd1306_swap(x, y);
    x = WIDTH - x - 1;
    break;
  case 2:
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:
    ssd1306_swap(x, y);
    y = HEIGHT - y - 1;
    break;
  }

  // x is which column
  getCacheLine(x, y);
  uint8_t data = m_buffer[SSD1306_BUFFER_ADDR(x, y)];
  uint8_t mask = (1 << (y & 0x07));
  
#if 0
  Serial.print("PIXEL ");
  Serial.print("X: ");
  Serial.print(x, DEC);
  Serial.print(" Y: ");
  Serial.print(y, DEC);
  Serial.print(" data: ");
  Serial.print(data, HEX);
  Serial.print(" mask: ");
  Serial.print(mask, HEX);
  Serial.print(" color: ");
  Serial.print(color, DEC);
#endif

  switch (color)
  {
    case WHITE:   
      data |= mask;  
      break;
    case BLACK:   
      data &= ~mask; 
      break;
    case INVERSE: 
      data ^= mask;  
      break;
    default:
      return;
  }
  m_buffer[SSD1306_BUFFER_ADDR(x, y)] = data;
#if 0
  Serial.print(" data: ");
  Serial.println(data, HEX);
#endif
  m_cache_clean = false;
  m_pages_empty &= ~SSD1306_PAGE_BIT(y);
}

SSD1306::SSD1306(uint8_t i2caddr) :
Adafruit_GFX(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT) {
  m_i2caddr = i2caddr;
  m_fram = NULL;
}


void SSD1306::begin(uint8_t vccstate) {
  m_vccstate = vccstate;

  // I2C Init
  Wire.begin();
#ifdef __SAM3X8E__
  // Force 400 KHz I2C, rawr! (Uses pins 20, 21 for SDA, SCL)
  TWI1->TWI_CWGR = 0;
  TWI1->TWI_CWGR = ((VARIANT_MCK / (2 * 400000)) - 4) * 0x101;
#endif

  // Init sequence
  ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
  ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  ssd1306_command(0x80);                                  // the suggested ratio 0x80

  ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
  ssd1306_command(SSD1306_LCDHEIGHT - 1);

  ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
  ssd1306_command(0x0);                                   // no offset
  ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
  ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
  if (vccstate == SSD1306_EXTERNALVCC) {
    ssd1306_command(0x10);
  } else {
    ssd1306_command(0x14);
  }
  ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
  ssd1306_command(0x00);                                  // 0x0 act like ks0108
  ssd1306_command(SSD1306_SEGREMAP | 0x1);
  ssd1306_command(SSD1306_COMSCANDEC);

#if defined SSD1306_128_32
  ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_command(0x02);
  ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  ssd1306_command(0x8F);
#elif defined SSD1306_128_64
  ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_command(0x12);
  ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  if (vccstate == SSD1306_EXTERNALVCC) {
    ssd1306_command(0x9F);
  } else {
    ssd1306_command(0xCF);
  }
#elif defined SSD1306_96_16
  ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_command(0x2);   //ada x12
  ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  if (vccstate == SSD1306_EXTERNALVCC) {
    ssd1306_command(0x10);
  } else {
    ssd1306_command(0xAF);
#endif

  ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
  if (vccstate == SSD1306_EXTERNALVCC) {
    ssd1306_command(0x22);
  } else {
    ssd1306_command(0xF1);
  }
  ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
  ssd1306_command(0x40);
  ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

  ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

  ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}


void SSD1306::invertDisplay(uint8_t i) {
  if (i) {
    ssd1306_command(SSD1306_INVERTDISPLAY);
  } else {
    ssd1306_command(SSD1306_NORMALDISPLAY);
  }
}

void SSD1306::ssd1306_command(uint8_t c) {
  // I2C
  uint8_t control = 0x00;   // Co = 0, D/C = 0

  Wire.setClock(400000);
  Wire.beginTransmission(m_i2caddr);
  Wire.write(control);
  Wire.write(c);
  Wire.endTransmission();
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrollright(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
  ssd1306_command(0x00);
  ssd1306_command(start);
  ssd1306_command(0x00);
  ssd1306_command(stop);
  ssd1306_command(0x00);
  ssd1306_command(0xFF);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrollleft(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
  ssd1306_command(0x00);
  ssd1306_command(start);
  ssd1306_command(0x00);
  ssd1306_command(stop);
  ssd1306_command(0x00);
  ssd1306_command(0xFF);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrolldiagright(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
  ssd1306_command(0x00);
  ssd1306_command(SSD1306_LCDHEIGHT);
  ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  ssd1306_command(0x00);
  ssd1306_command(start);
  ssd1306_command(0x00);
  ssd1306_command(stop);
  ssd1306_command(0x01);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
  ssd1306_command(0x00);
  ssd1306_command(SSD1306_LCDHEIGHT);
  ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  ssd1306_command(0x00);
  ssd1306_command(start);
  ssd1306_command(0x00);
  ssd1306_command(stop);
  ssd1306_command(0x01);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void SSD1306::stopscroll(void){
  ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void SSD1306::dim(boolean dim) {
  uint8_t contrast;

  if (dim) {
    contrast = 0; // Dimmed display
  } else {
    if (m_vccstate == SSD1306_EXTERNALVCC) {
      contrast = 0x9F;
    } else {
      contrast = 0xCF;
    }
  }
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  ssd1306_command(SSD1306_SETCONTRAST);
  ssd1306_command(contrast);
}

void SSD1306::display(void) {
  ssd1306_command(SSD1306_COLUMNADDR);
  ssd1306_command(0);   // Column start address (0 = reset)
  ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

  ssd1306_command(SSD1306_PAGEADDR);
  ssd1306_command(0); // Page start address (0 = reset)
  ssd1306_command((SSD1306_LCDHEIGHT >> 3) - 1); // Page end address

  //TWBR = 12; // upgrade to 400KHz!

  if (!m_cache_clean) {
    flushCacheLine();
  }

#if 0
  Serial.println("Display");
#endif

  for (int16_t y = 0; y < SSD1306_LCDHEIGHT; y += 8) {
    bool empty = !(!(m_pages_empty & SSD1306_PAGE_BIT(y)));
    if (!empty) {
      getCacheLine(0, y);
    }

    for (uint8_t x = 0; x < SSD1306_BUFFER_SIZE; x += 16) {
      // send a bunch of data in one transmission
#if 0
      Serial.print("X: ");
      Serial.println(x, HEX);
#endif
      Wire.setClock(400000);
      Wire.beginTransmission(m_i2caddr);
      Wire.write(0x40);
      for (uint8_t i = 0; i < 16; i++) {
	uint8_t data = (empty ? 0x00 :
	                m_buffer[SSD1306_BUFFER_ADDR(x + i, y)]);
#if 0
	Serial.print(data, HEX);
	Serial.print(" ");
#endif
        Wire.write(data);
      }
      Wire.endTransmission();
#if 0
      Serial.println("");
#endif
    }
  }

  if (m_show_logo) {
    clearDisplay();
  }
}

// clear everything
void SSD1306::clearDisplay(void) {
  m_show_logo = false;
  m_pages_empty = 0xFF;
  m_cache_clean = true;
  m_cache_address = 0xFFFF;
}

void SSD1306::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  boolean bSwap = false;
  switch(rotation) {
    case 0:
      // 0 degree rotation, do nothing
      break;
    case 1:
      // 90 degree rotation, swap x & y for rotation, then invert x
      bSwap = true;
      ssd1306_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      // 180 degree rotation, invert x and y - then shift y around for height.
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      x -= (w-1);
      break;
    case 3:
      // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
      bSwap = true;
      ssd1306_swap(x, y);
      y = HEIGHT - y - 1;
      y -= (w-1);
      break;
  }

  if(bSwap) {
    drawFastVLineInternal(x, y, w, color);
  } else {
    drawFastHLineInternal(x, y, w, color);
  }
}

void SSD1306::drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) {
  // Do bounds/limit checks
  if (y < 0 || y >= HEIGHT) {
    return;
  }

  // make sure we don't try to draw below 0
  if (x < 0) {
    w += x;
    x = 0;
  }

  // make sure we don't go off the edge of the display
  if ((x + w) > WIDTH) {
    w = (WIDTH - x);
  }

  // if our width is now negative, punt
  if (w <= 0) {
    return;
  }

  getCacheLine(x, y);
  register uint8_t mask = SSD1306_PIXEL_MASK(y);

  if (color == BLACK) {
    mask = ~mask;
  }

#if 0
  Serial.print("HLINE ");
  Serial.print("X: ");
  Serial.print(x, DEC);
  Serial.print(" W: ");
  Serial.print(w, DEC);
  Serial.print(" Y: ");
  Serial.print(y, DEC);
  Serial.print(" mask: ");
  Serial.print(mask, HEX);
  Serial.print(" color: ");
  Serial.println(color, DEC);
#endif

  for (uint8_t i = x; i < x + w; i++) {
#if 0
    Serial.print("i :");
    Serial.print(i, DEC);
    Serial.print(" before: ");
    Serial.print(m_buffer[SSD1306_BUFFER_ADDR(i, y)], HEX);
#endif

    uint8_t data = m_buffer[SSD1306_BUFFER_ADDR(i, y)];

    switch (color)
    {
      case WHITE:
        data |= mask;
        break;
      case BLACK:
        data &= mask;
        break;
      case INVERSE:
        data ^= mask;
        break;
      default:
        return;
    }

    m_buffer[SSD1306_BUFFER_ADDR(i, y)] = data;

#if 0
    Serial.print(" after: ");
    Serial.println(m_buffer[i], HEX);
#endif
  }
  m_cache_clean = false;
  m_pages_empty &= ~SSD1306_PAGE_BIT(y);
}

void SSD1306::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  bool bSwap = false;
  switch(rotation) {
    case 0:
      break;
    case 1:
      // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
      bSwap = true;
      ssd1306_swap(x, y);
      x = WIDTH - x - 1;
      x -= (h-1);
      break;
    case 2:
      // 180 degree rotation, invert x and y - then shift y around for height.
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      y -= (h-1);
      break;
    case 3:
      // 270 degree rotation, swap x & y for rotation, then invert y
      bSwap = true;
      ssd1306_swap(x, y);
      y = HEIGHT - y - 1;
      break;
  }

  if(bSwap) {
    drawFastHLineInternal(x, y, h, color);
  } else {
    drawFastVLineInternal(x, y, h, color);
  }
}


void SSD1306::drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color) {

  // do nothing if we're off the left or right side of the screen
  if (x < 0 || x >= WIDTH) {
    return;
  }

  // make sure we don't try to draw below 0
  if (__y < 0) {
    // __y is negative, this will subtract enough from __h to account for __y being 0
    __h += __y;
    __y = 0;
  }

  // make sure we don't go past the height of the display
  if ((__y + __h) > HEIGHT) {
    __h = (HEIGHT - __y);
  }

  // if our height is now negative, punt
  if (__h <= 0) {
    return;
  }

  // this display doesn't need ints for coordinates, use local byte registers
  // for faster juggling
  register uint8_t y = __y;
  register uint8_t h = __h;

  // do the first partial byte, if necessary - this requires some masking
  register uint8_t mod = (y & 0x07);
  register uint8_t data;
  if (mod) {
    // mask off the high n bits we want to set
    mod = 8 - mod;

    // note - lookup table results in a nearly 10% performance improvement in
    // fill* functions
    // register uint8_t mask = ~(0xFF >> (mod));
    static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC,
                                 0xFE };
    register uint8_t mask = premask[mod];

    getCacheLine(x, y);

    // adjust the mask if we're not going to reach the end of this byte
    if (h < mod) {
      mask &= (0xFF >> (mod-h));
    }

#if 0
    Serial.print("VLINE A  ");
    Serial.print("X: ");
    Serial.print(x, DEC);
    Serial.print(" Y: ");
    Serial.print(y, DEC);
    Serial.print(" H: ");
    Serial.print(h, DEC);
    Serial.print(" mod: ");
    Serial.print(mod, DEC);
    Serial.print(" mask: ");
    Serial.print(mask, HEX);
    Serial.print(" color: ");
    Serial.print(color, DEC);
#endif

    data = m_buffer[SSD1306_BUFFER_ADDR(x, y)];

#if 0
    Serial.print(" before: ");
    Serial.print(data, HEX);
#endif

    switch (color)
    {
      case WHITE:
        data |=  mask;
        break;
      case BLACK:
        data &= ~mask;
        break;
      case INVERSE:
        data ^=  mask;
        break;
      default:
        return;
    }

#if 0
    Serial.print(" after: ");
    Serial.println(data, HEX);
#endif

    m_buffer[SSD1306_BUFFER_ADDR(x, y)] = data;
    m_cache_clean = false;
    m_pages_empty &= ~SSD1306_PAGE_BIT(y);

    // fast exit if we're done here!
    if (h < mod) {
      return;
    }

    h -= mod;
    y += mod;
  }

  // write solid bytes while we can - effectively doing 8 rows at a time
  if (h >= 8) {
    if (color == INVERSE)  {
      // separate copy of the code so we don't impact performance of the
      // black/white write version with an extra comparison per loop
      do {
        getCacheLine(x, y);
        data = m_buffer[SSD1306_BUFFER_ADDR(x, y)];
        m_buffer[SSD1306_BUFFER_ADDR(x, y)] = ~data;

        // adjust h & y (there's got to be a faster way for me to do this, but
        // this should still help a fair bit for now)
        h -= 8;
        y += 8;
      } while (h >= 8);
    } else {
      // store a local value to work with
      data = (color == WHITE) ? 0xFF : 0;

      do  {
        // write our value in
        getCacheLine(x, y);

#if 0
        Serial.print("VLINE B  ");
        Serial.print("X: ");
        Serial.print(x, DEC);
        Serial.print(" Y: ");
        Serial.print(y, DEC);
        Serial.print(" H: ");
        Serial.print(h, DEC);
        Serial.print(" color: ");
        Serial.print(color, DEC);
        Serial.print(" before: ");
        Serial.print(m_buffer[SSD1306_BUFFER_ADDR(x, y)], HEX);
#endif

        m_buffer[SSD1306_BUFFER_ADDR(x, y)] = data;

#if 0
        Serial.print(" after: ");
        Serial.println(m_buffer[SSD1306_BUFFER_ADDR(x, y)], HEX);
#endif

        m_cache_clean = false;
        m_pages_empty &= ~SSD1306_PAGE_BIT(y);

        // adjust h & y (there's got to be a faster way for me to do this, but
        // this should still help a fair bit for now)
        h -= 8;
        y += 8;
      } while (h >= 8);
    }
  }

  // now do the final partial byte, if necessary
  if (h) {
    mod = h & 0x07;
    // this time we want to mask the low bits of the byte, vs the high bits we
    // did above
    // register uint8_t mask = (1 << mod) - 1;
    // note - lookup table results in a nearly 10% performance improvement in
    // fill* functions
    static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F,
                                  0x7F};
    register uint8_t mask = postmask[mod];

    getCacheLine(x, y);

#if 0
    Serial.print("VLINE C  ");
    Serial.print("X: ");
    Serial.print(x, DEC);
    Serial.print(" Y: ");
    Serial.print(y, DEC);
    Serial.print(" H: ");
    Serial.print(h, DEC);
    Serial.print(" mod: ");
    Serial.print(mod, DEC);
    Serial.print(" mask: ");
    Serial.print(mask, HEX);
    Serial.print(" color: ");
    Serial.print(color, DEC);
#endif

    data = m_buffer[SSD1306_BUFFER_ADDR(x, y)];

#if 0
    Serial.print(" before: ");
    Serial.print(data, HEX);
#endif

    switch (color)
    {
      case WHITE:
        data |=  mask;
        break;
      case BLACK:
        data &= ~mask;
        break;
      case INVERSE:
        data ^=  mask;
        break;
    }
    m_buffer[SSD1306_BUFFER_ADDR(x, y)] = data;

#if 0
    Serial.print(" after: ");
    Serial.println(data, HEX);
#endif

    m_cache_clean = false;
    m_pages_empty &= ~SSD1306_PAGE_BIT(y);
  }
}
