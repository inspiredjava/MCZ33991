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
#include <util/delay.h>
#include "spi.h"
#include "MCZ33991.h"

#define GAUGE0  1
#define GAUGE1  2
#define BOTH    3

uint8_t main() {
    uint16_t x27Sweep = 315 * 12; /**< max sweep for X27 stepper motor*/

    // Initialization of SPI periphery to work with 33991 driver
    SPI_init();

    if (MCZ33991_init(DRIVER_PIN, GAUGE0) == 0) {
        // Set velocity for Gauge0
        MCZ33991_setVelocityGauge0(DRIVER_PIN, 0xFF);
        while (1) {
            // Imitation of working process
            MCZ33991_setPosGauge0(DRIVER_PIN, x27Sweep / 4);
            _delay_ms(2000);
            MCZ33991_setPosGauge0(DRIVER_PIN, x27Sweep / 4 * 2);
            _delay_ms(2000);
            MCZ33991_setPosGauge0(DRIVER_PIN, x27Sweep / 4 * 3);
            _delay_ms(2000);
            MCZ33991_setPosGauge0(DRIVER_PIN, x27Sweep);
            _delay_ms(2000);

            // When work is done return gauge to zero position
            MCZ33991_Return(DRIVER_PIN, GAUGE0);
        }
    }

    return 0;
}
