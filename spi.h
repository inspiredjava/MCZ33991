/** Copyright (C) 2016 Oleksandr Los
 *  This file is part of MCZ33991 library.
 *
 *  MCZ33991 library is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  MCZ33991 library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with MCZ33991 library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SPI_H
#define __SPI_H

// Define SPI pins
#define SPI_DDR    DDRB
#define SPI_PORT   PORTB
#define SPI_SCK    PB5
#define SPI_MOSI   PB3
#define SPI_MISO   PB4

//****************************** USER DEFINE *******************************//
#define DRIVER_PIN     PB2
//**************************************************************************//

void SPI_init();
void SPI_tranferWord(uint8_t ssPin, uint16_t* data);
uint16_t SPI_readWord(uint8_t ssPin);
void SPI_chipSelect(uint8_t ssPin);
void SPI_chipRelease(uint8_t ssPin);

#endif // __SPI_H
