/* 
  SPI.h - SPI library for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define SPI_NUM_0     0
#define SPI_NUM_1     1

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1

class SPISettings
{
public:
    SPISettings() :_clock(1000000), _bitOrder(SPI_MSBFIRST), _dataMode(SPI_MODE0) {}
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) :_clock(clock), _bitOrder(bitOrder), _dataMode(dataMode) {}
    uint32_t _clock;
    uint8_t  _bitOrder;
    uint8_t  _dataMode;
};

class SPIClass
{
private:
	int8_t _ss;
	int8_t _spi_num;
	uint32_t _freq;
	bool _inTransaction;
	uint8_t  _bitOrder;
	uint8_t  _dataMode;

public:
    SPIClass(int8_t ss,int8_t spiNum);
    void begin(int8_t ss = -1, uint32_t freq = 6000000, int8_t spiNum = -1);
    void end();
    void setFrequency(uint32_t freq);
    void beginTransaction(SPISettings settings);
    void endTransaction(void);
    void transfer(uint8_t * data, uint32_t size);
    uint8_t transfer(uint8_t data);
  
    void transferBytes(uint8_t * data, uint8_t * out, uint32_t size);
};
typedef SPIClass SPIC;

extern SPIClass SPI;
extern SPIClass SPI1;

#endif

