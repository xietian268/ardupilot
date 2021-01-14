/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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

/* 
   FRSKY Telemetry library
*/

#define AP_SERIALMANAGER_STM32_BAUD             115200
#define AP_SERIALMANAGER_STM32_BUFSIZE_RX           64
#define AP_SERIALMANAGER_STM32_BUFSIZE_TX           64

#include "AP_STM32.h"


extern const AP_HAL::HAL& hal;

//constructor
AP_STM32::AP_STM32(void)
{
    _port = NULL;
    _step = 0;
}

void AP_STM32::init(const AP_SerialManager& serial_manager)
{
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_STM32, 0))){
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

        _port->begin(AP_SERIALMANAGER_STM32_BAUD, AP_SERIALMANAGER_STM32_BUFSIZE_RX, AP_SERIALMANAGER_STM32_BUFSIZE_TX);
    }
}

bool AP_STM32::update()
{
    if(_port ==NULL)
        return false;

    int16_t numc = _port->available();
    uint8_t data;
    uint8_t checkbyte = 0x0D;

    for (int16_t i = 0; i < numc; i++){
        data = _port->read();

        switch(_step) {
        case 0:
            if(data == 0xA5)
                _step = 1;
            break;

        case 1:
            if(data == 0x5A)
                _step = 2;
            break;

        case 2:
            _xd_temp = data;
            _step = 3;
            break;

        case 3:
            _yd_temp = data;
            _step = 4;
            break;

        case 4:
            _step = 0;
            if(checkbyte == data){
                xd = _xd_temp;
                yd = _yd_temp;
                last_frame_ms = AP_HAL::millis();
                return true;
            }
            break;

        default:
            _step = 0;
        }
    }

    return false;
}
