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

#ifndef __MCZ33991_H
#define __MCZ33991_H

//****************************** USER DEFINE *******************************//

/**
 *  Define type of the motor to work with. Calibration pulse depends on it's
 *  gear ratio.
 */
#define SLOW_MOTOR // for X27 Stepper motor (gear ratio 180:1)

/**
 *  Define RESET pin
 */
#define MCZ33991_DDR        DDRB
#define MCZ33991_PORT       PORTB
#define MCZ33991_RST_PIN    PB1

//**************************************************************************//

#define MCZ33991_GAUGE0_OVER_TEMPERATURE    0x01
#define MCZ33991_GAUGE1_OVER_TEMPERATURE    0x02
#define MCZ33991_GAUGE0_RTZ_INPROCESS       0x04
#define MCZ33991_GAUGE1_RTZ_INPROCESS       0x08
#define MCZ33991_GAUGE0_POS_CHANGED         0x10
#define MCZ33991_GAUGE1_POS_CHANGED         0x20
#define MCZ33991_POWER_WARNING              0x40
#define MCZ33991_CALIBRATION_CLOCK_ERROR    0x80


/** MODULE MEMORY MAP.
 *  The 33991 uses six registers to configure the device and
 *  control the state of the four H-bridge outputs.
 *  The registers are addressed via bits D15-D13 of the incoming SPI word
 *  ______________________________________________________
 *  |D15|D14|D13|D12|D11|D10|D9|D8|D7|D6|D5|D4|D3|D2|D1|D0|
 *  ------------------------------------------------------
 * See datasheet for detailed information
 */

/** POWER, ENABLE, AND CALIBRATION REGISTER.
 *  D12     Null Command - [1] for Status Read
 *  D11-D5  Must be 0
 *  D4      Clock Calibration Frequency Selector
 *  D3      Clock Calibration Enable
 *  D2      Oscillator Adjustment
 *  D1      Gauge 1 Enable
 *  D0      Gauge 0 Enable
 */
#define MCZ33991_PECR   0x0000

/** MAXIMUM VELOCITY REGISTER.
 *
 *  D12-D10 Must be 0
 *  D9      Velocity applies to Gauge 1
 *  D8      Velocity applies to Gauge 0
 *  D7-D0   Maximum Velocity, contain a position value from 1–255
 *          representative of the table position value
 */
#define MCZ33991_VELR   0x2000

/** GAUGE 0 POSITION REGISTER.
 *
 *  D12     Must be a logic [0]
 *  D11-D0  Desired pointer position of Gauge 0
 *
 *  Pointer positions can range from 0 (000000000000) to
 *  position 4095 (111111111111). For a stepper motor requiring
 *  12 microsteps per degree of pointer movement, the
 *  maximum pointer sweep is 341.25°.
 */
#define MCZ33991_POS0R  0x4000

/** GAUGE 1 POSITION REGISTER.
 *
 *  Same as previous but applying to Gouge1
 */
#define MCZ33991_POS1R  0x6000

/** RETURN TO 0 REGISTER.
 *
 *  D1 enables a Return to Zero for Gauge 0 if D0 is logic [0],
 *  and Gauge 1 if D0 is [1], respectively
 *  Similarly, a logic [0] written to bit D1 disables a Return to Zero
 *  for Gauge 0 when D0 is logic [0], and Gauge 1 when D0 is 1,
 *  respectively
 *  Bits D3 and D2 are used to determine which eight bits of
 *  the 15-bit RTZ will be the 8 MSBs
 *  Bits D12:D5 must be at logic [0]
 *  Bit D4 is used to enable an unconditional RTZ event.
 */
#define MCZ33991_RTZR   0x8000

/** RETURN TO 0 CONFIGURATION REGISTER.
 *
 *  D12-D5  This bits determine the preloaded value
 *          into the RTZ integration accumulator to adjust
 *          the detection threshold.
 *  D4      Determines the provided blanking time
 *  D3-D0   Determine the time spent at each full step during an RTZ event
 */
#define MCZ33991_RTZCR  0xA000

void MCZ33991_resetPulse();
uint16_t MCZ33991_configureDriver(uint8_t ssPin, uint8_t gauge);
uint16_t MCZ33991_setPosGauge0(uint8_t ssPin, uint16_t position);
uint16_t MCZ33991_setPosGauge1(uint8_t ssPin, uint16_t position);
uint16_t MCZ33991_rtzGauge0(uint8_t ssPin);
uint16_t MCZ33991_rtzGauge1(uint8_t ssPin);
uint16_t MCZ33991_setVelocityGauge0(uint8_t ssPin, uint8_t velocity);
uint16_t MCZ33991_setVelocityGauge1(uint8_t ssPin, uint8_t velocity);
uint16_t MCZ33991_getStatus(uint8_t ssPin);
uint8_t MCZ33991_init(uint8_t ssPin, uint8_t gauge);
void MCZ33991_Return(uint8_t ssPin, uint8_t gauge);

#endif // __MCZ33991_H
