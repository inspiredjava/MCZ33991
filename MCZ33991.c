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
#include <avr/interrupt.h>
#include "MCZ33991.h"
#include "spi.h"

/** Sending calibration pulse.
 *
 *  Pulling down SS line for specific period of time.
 *
 *  \param  ssPin   Chip select pin
 *  \param  period  Period of pulse
 */
void MCZ33991_calibrationPulse(uint8_t ssPin) {
    uint8_t sreg;
    sreg = SREG;
    cli();
    SPI_chipSelect(ssPin);
    #ifdef SLOW_MOTOR
    _delay_us(12);
    #else
    _delay_us(8);
    #endif
    SPI_chipRelease(ssPin);
    SREG = sreg;
}

/** Reset propcedure.
 *
 *  Pulling down reset pin for 3us and waiting for 5us before starting
 *  data transfer.
 */
void MCZ33991_resetPulse() {
    MCZ33991_DDR |= (1 << MCZ33991_RST_PIN);
    MCZ33991_PORT &= (1 << MCZ33991_RST_PIN);
    _delay_us(3);
    MCZ33991_PORT |= (1 << MCZ33991_RST_PIN);
    _delay_us(5);
}

/** Configureing 33991 driver.
 *
 *  Configure driver with setting clock Calibration frequency to
 *  maximum value.
 *
 *  \param  ssPin   Chip select pin
 *  \param  gauge   Defines which gauge to enable.
 *                  1 - Gouge0
 *                  2 - Gouge1
 *                  3 - Gouge0 & Gouge1
 *
 *  \retval     Last value of status regiter
 */
uint16_t MCZ33991_configureDriver(uint8_t ssPin, uint8_t gauge) {
    uint16_t configWord;
    uint16_t status;

    // define gouges, enable clock calibration and set it to max
    configWord = (gauge & 0x03) | 0x10 | 0x08;
    #ifdef SLOW_MOTOR
    // slow oscillator
    configWord |= 0x04;
    #endif
    SPI_tranferWord(ssPin, &configWord);
    MCZ33991_calibrationPulse(ssPin);

    // configure RTZCR
    configWord = MCZ33991_RTZCR | 0x21;
    SPI_tranferWord(ssPin, &configWord);
    status = MCZ33991_getStatus(ssPin);

    return configWord;
}

/** Set position for Gauge0.
 *
 *  Checking the value of position param and send command to move
 *  Gouge0 pointer to desire position. Be aware if pointer value exceeds 12 bit
 *  it will be truncuted.
 *
 *  \param      ssPin       Chip select pin
 *  \param      position    Desired position to move Gauge0
 *
 *  \retval     Last value of status regiter
 */
uint16_t MCZ33991_setPosGauge0(uint8_t ssPin, uint16_t position) {
    // Position can be defined only by 12 bit
    uint16_t configWord = MCZ33991_POS0R | (position & 0x0FFF);
    SPI_tranferWord(ssPin, &configWord);

    return configWord;
}

/** Set position for Gauge1.
 *
 *  Checking the value of position param and send command to move
 *  Gouge0 pointer to desire position. Be aware if pointer value exceeds 12 bit
 *  it will be truncuted.
 *
 *  \param      ssPin   Chip select pin
 *  \param      position    Desired position to move Gauge1
 *
 *  \retval     Last value of status regiter
 */
uint16_t MCZ33991_setPosGauge1(uint8_t ssPin, uint16_t position) {
    // Position can be defined only by 12 bit
    uint16_t configWord = MCZ33991_POS1R | (position & 0x0FFF);
    SPI_tranferWord(ssPin, &configWord);

    return configWord;
}

/** Return to zero position for Gauge0.
 *
 *  \param      ssPin   Chip select pin
 *
 *  \retval     Last value of status regiter
 */
uint16_t MCZ33991_rtzGauge0(uint8_t ssPin) {
    uint16_t configWord = MCZ33991_RTZR | 0x02;
    SPI_tranferWord(ssPin, &configWord);

    return configWord;
}

/** Return to zero position for Gauge1.
 *
 *  \param      ssPin   Chip select pin
 *
 *  \retval     Last value of status regiter
 */
uint16_t MCZ33991_rtzGauge1(uint8_t ssPin) {
    uint16_t configWord = MCZ33991_RTZR | 0x03;
    SPI_tranferWord(ssPin, &configWord);

    return configWord;
}

/** Set maximum velocity for Gauge0.
 *
 *  \param      ssPin   Chip select pin
 *
 *  \retval     Last value of status regiter
 */
uint16_t MCZ33991_setVelocityGauge0(uint8_t ssPin, uint8_t velocity) {
    uint16_t configWord = MCZ33991_VELR | 0x100 | velocity;
    SPI_tranferWord(ssPin, &configWord);

    return configWord;
}

/** Set maximum velocity for Gauge1.
 *
 *  \param      ssPin   Chip select pin
 *
 *  \retval     Last value of status regiter
 */
uint16_t MCZ33991_setVelocityGauge1(uint8_t ssPin, uint8_t velocity) {
    uint16_t configWord = MCZ33991_VELR | 0x200 | velocity;
    SPI_tranferWord(ssPin, &configWord);

    return configWord;
}

/** Getting status register data.
 *
 *  Send command to slave device to get status register
 */
uint16_t MCZ33991_getStatus(uint8_t ssPin) {
    uint16_t statusRequest = MCZ33991_PECR | 0x1000;
    SPI_tranferWord(ssPin, &statusRequest);

    return statusRequest;
}

/** Initilize driver.
 *
 *  Performs first init steps. Resetting driver, configurign it,
 *  checking stepper motor is connected and can rotate,
 *  establishing zero reference.
 *
 *  \param      ssPin   Chip select pin
 *  \param      gauge   Defines which gauge to enable.
 *                      1 - Gouge0
 *                      2 - Gouge1
 *                      3 - Gouge0 & Gouge1
 *
 *  \retval     0 - if initialization is ok
 *              1 - if Gauge0 didn't move
 *              2 - if Gauge1 didn't move
 *              3 - if Gauge0 and Gauge1 didn't move
 */
uint8_t MCZ33991_init(uint8_t ssPin, uint8_t gauge) {
    MCZ33991_resetPulse();
    uint16_t status; /**< status values of 33991 here*/
    uint16_t result = 0; /**< error code */

    // Repeat configuration routine while bit D7 (Calibrated clock out of Spec)
    // of status register is 1
    do {
        // Enable Gauge(s)
        MCZ33991_configureDriver(ssPin, gauge);
        status = MCZ33991_getStatus(ssPin);
    } while (status & MCZ33991_CALIBRATION_CLOCK_ERROR);

    if (gauge == 1 || gauge == 3) {
        // move gouge0 on one step
        MCZ33991_setPosGauge0(ssPin, 12);
        _delay_ms(1);
        status = MCZ33991_getStatus(ssPin);
        // here we can check if gauge moved to first microstep
        if (!(status & MCZ33991_GAUGE0_POS_CHANGED)) {
            result = 1;
        }

        // Initiate RTZ event
        MCZ33991_rtzGauge0(ssPin);
        _delay_ms(1);
        status = MCZ33991_getStatus(ssPin);
        while (status & MCZ33991_GAUGE0_RTZ_INPROCESS) {
            status = MCZ33991_getStatus(ssPin);
            _delay_ms(1);
        }
    }

    if (gauge == 2 || gauge == 3) {
        // move gouge0 on one step
        MCZ33991_setPosGauge1(ssPin, 12);
        _delay_ms(1);
        status = MCZ33991_getStatus(ssPin);
        // here we can check if gauge moved to first microstep
        if (!(status & MCZ33991_GAUGE1_POS_CHANGED)) {
            result |= 2;
        }

        // Initiate RTZ event
        MCZ33991_rtzGauge1(ssPin);
        _delay_ms(1);
        status = MCZ33991_getStatus(ssPin);
        while (status & MCZ33991_GAUGE1_RTZ_INPROCESS) {
            status = MCZ33991_getStatus(ssPin);
            _delay_ms(1);
        }
    }

    return result;
}

/** Returns gauge to start position.
 *
 *  \param      ssPin   Chip select pin
 *  \param      gauge   Defines which gauge to enable.
 *                      1 - Gouge0
 *                      2 - Gouge1
 *                      3 - Gouge0 & Gouge1
 */
void MCZ33991_Return(uint8_t ssPin, uint8_t gauge) {
    uint16_t status; /**< status values of 33991 here*/

    if (gauge == 1 || gauge == 3) {
        // move gouge0 closer to 0 position
        MCZ33991_setPosGauge0(ssPin, 0x00);
        do {
            _delay_ms(20);
            status = MCZ33991_getStatus(ssPin);
        } while (status & MCZ33991_GAUGE0_POS_CHANGED);

        // move gauge0 to one step forward
        status = MCZ33991_setPosGauge0(ssPin, 12);
        // Initiate RTZ event
        MCZ33991_rtzGauge0(ssPin);
        _delay_ms(2);
        do {
            status = MCZ33991_getStatus(ssPin);
            _delay_ms(1);
        } while (status & MCZ33991_GAUGE0_RTZ_INPROCESS);
    }

    if (gauge == 2 || gauge == 3) {
        // move gouge1 closer to 0 position
        MCZ33991_setPosGauge1(ssPin, 0x00);
        do {
            _delay_ms(20);
            status = MCZ33991_getStatus(ssPin);
        } while (status & MCZ33991_GAUGE1_POS_CHANGED);

        // move gauge1 to one step forward
        status = MCZ33991_setPosGauge1(ssPin, 12);
        // Initiate RTZ event
        MCZ33991_rtzGauge1(ssPin);
        _delay_ms(2);
        do {
            status = MCZ33991_getStatus(ssPin);
            _delay_ms(1);
        } while (status & MCZ33991_GAUGE1_RTZ_INPROCESS);
    }
}
