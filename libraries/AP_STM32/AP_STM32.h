/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_STM32 {
public:
    AP_STM32();


    /* Do not allow copies */
    AP_STM32(const AP_STM32 &other) = delete;
    AP_STM32 &operator=(const AP_STM32&) = delete;

    void init(const AP_SerialManager& serial_manager);

    bool update(void);

    
    uint8_t xd = 0;
    uint8_t yd = 0;
    
    uint32_t last_frame_ms;

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to receiver

    uint8_t _step;

    uint8_t _xd_temp;
    uint8_t _yd_temp;
};
