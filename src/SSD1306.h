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
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

/*
 * Reworked by Gavin Hurlbut <gjhurlbu@gmail.com> to use an attached SPI FRAM
 * for buffer storage, default to 128x64 display, I2C only
 *
 * changes (c) 2016 Gavin Hurlbut <gjhurlbu@gmail.com>
 * released under the BSD License
 */

#ifndef _SSD1306_H_
#define _SSD1306_H_

#if ARDUINO >= 100
 #include "Arduino.h"
 #define WIRE_WRITE Wire.write
#else
 #include "WProgram.h"
  #define WIRE_WRITE Wire.send
#endif

#include <Adafruit_GFX.h>
#include <Adafruit_FRAM_SPI.h>
#include <FRAM_Cache.h>

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define SSD1306_I2C_ADDRESS   0x3C  // 011110+SA0+RW - 0x3C or 0x3D
// Address for 128x32 is 0x3C
// Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)

/*=========================================================================
    SSD1306 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
    Select the appropriate display below to create an appropriately
    sized framebuffer, etc.

    SSD1306_128_64  128x64 pixel display

    SSD1306_128_32  128x32 pixel display

    SSD1306_96_16

    -----------------------------------------------------------------------*/
#define SSD1306_128_64
//   #define SSD1306_128_32
//   #define SSD1306_96_16
/*=========================================================================*/

#if defined SSD1306_128_64 && defined SSD1306_128_32
  #error "Only one SSD1306 display can be specified at once in SSD1306.h"
#endif
#if !defined SSD1306_128_64 && !defined SSD1306_128_32 && !defined SSD1306_96_16
  #error "At least one SSD1306 display must be specified in SSD1306.h"
#endif

#if defined SSD1306_128_64
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 64
#endif
#if defined SSD1306_128_32
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 32
#endif
#if defined SSD1306_96_16
  #define SSD1306_LCDWIDTH                  96
  #define SSD1306_LCDHEIGHT                 16
#endif

#define SSD1306_BUFFER_SIZE SSD1306_LCDWIDTH
#define SSD1306_RAM_MIRROR_SIZE (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8)

#define SSD1306_PIXEL_ADDR(x, y) ((x) + ((y) >> 3) * SSD1306_LCDWIDTH)
#define SSD1306_BUFFER_ADDR(x, y) SSD1306_PIXEL_ADDR(x, y & 0x0008)
#define SSD1306_PIXEL_MASK(y)	 (1 << ((y) & 0x07))
#define SSD1306_PAGE_BIT(y)	 (1 << ((y) >> 3))

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

class SSD1306 : public Adafruit_GFX {
 public:
  SSD1306(uint8_t i2caddr = SSD1306_I2C_ADDRESS);

  void begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC);
  void attachRAM(Adafruit_FRAM_SPI *fram, uint16_t buffer, uint16_t logo);
  void initializeLogo(void);

  void clearDisplay(void);
  void invertDisplay(uint8_t i);
  void display();

  void startscrollright(uint8_t start, uint8_t stop);
  void startscrollleft(uint8_t start, uint8_t stop);

  void startscrolldiagright(uint8_t start, uint8_t stop);
  void startscrolldiagleft(uint8_t start, uint8_t stop);
  void stopscroll(void);

  void dim(boolean dim);

  void drawPixel(int16_t x, int16_t y, uint16_t color);

  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

 private:
  inline void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color) __attribute__((always_inline));
  inline void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) __attribute__((always_inline));

  void ssd1306_command(uint8_t c);

  uint8_t readCache(int16_t x, int16_t y);
  void writeCache(int16_t x, int16_t y, uint8_t value);
  void operCache(int16_t x, int16_t y, oper_t oper_, uint8_t mask);

  uint16_t getPixelAddress(int16_t x, int16_t y);

  int8_t m_i2caddr;
  int8_t m_vccstate;

  Cache_Segment *m_logo_cache;
  Cache_Segment *m_draw_cache;

  Adafruit_FRAM_SPI *m_fram;
  bool m_show_logo;
};

#endif /* _SSD1306_H_ */
