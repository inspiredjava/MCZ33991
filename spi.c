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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "spi.h"

/** Turn slave device to active state
 *  Pulling down SS line
 *
 *  \param  ssPin   Chip select pin
 */
void SPI_chipSelect(uint8_t ssPin) {
    SPI_PORT &= ~(1 << ssPin);
}

/** Turn slave device to inactive state
 *  Pulling up SS line
 *
 *  \param  ssPin   Chip select pin
 */
void SPI_chipRelease(uint8_t ssPin) {
     SPI_PORT |= (1 << ssPin);
}

/** SPI initialization function.
 *  Enbling SPI interface, sets up controller as Master.
 *  We need 1MHz - 3MHz SPI rate for MCZ33991, so
 *  setting up SPI bus frequency to fck/16 to get 1MHz on 16MHz chip.
 *  Data order - most significant bit first
 *  SPI Mode 0
 *  By default standard SS pin is used and can be reffer as DRIVER_PIN from the
 *  source code. If needs to communicate with more then one device user
 *  should define and configure it manually.
 */
void SPI_init() {
    // Set MOSI, SCK and SS as output
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << DRIVER_PIN);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << CPHA);
    SPI_chipRelease(DRIVER_PIN);
}

/** Sending and receiveing words of data over SPI.
 *  Puts data to SPI Data Register and waits for transmission complete.
 *
 *  \param      ssPin   Chip select pin
 *  \param      data    Pointer to the first byte of sending data. After transfer
 *                      completed it points to received data.
 */
void SPI_tranferWord(uint8_t ssPin, uint16_t* data) {
    uint8_t sreg;

    sreg = SREG;
    cli();

    SPI_chipSelect(ssPin);
    _delay_us(0.001);
    SPDR = *data >> 8;
    while (!(SPSR & (1 << SPIF)));
    *data &= 0x00FF | (uint16_t)SPDR << 8;

    SPDR = *data & 0x00FF;
    while (!(SPSR & (1 << SPIF)));
    *data |= SPDR;
    _delay_us(1);
    SPI_chipRelease(ssPin);
    SREG = sreg;
    // Required setup delay
    _delay_us(5);
}
