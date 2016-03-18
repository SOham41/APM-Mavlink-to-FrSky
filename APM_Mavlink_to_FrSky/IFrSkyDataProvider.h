/*
    @author     Nils Högberg
    @contact     nils.hogberg@gmail.com

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

#include <inttypes.h>

class IFrSkyDataProvider
{
public:
    // Absolute GPS altitude in meters
    virtual const int32_t    getGpsAltitude() = 0;
    virtual const int    getTemp1() = 0;
    virtual const int    getEngineSpeed() = 0;
    virtual const int    getFuelLevel() = 0;
    virtual const int    getTemp2() = 0;
    virtual const float    getAltitude() = 0;
    //speed in m/s
    virtual const float    getGpsGroundSpeed() = 0;
    //Longitude in 1E7 degrees
    virtual const int32_t    getLongitude() = 0;
    //Latitude in 1E7 degrees
    virtual const int32_t    getLatitude() = 0;
    //heading in degrees
    virtual const int    getCourse() = 0;
    virtual const int    getYear() = 0;
    virtual const int    getDate() = 0;
    virtual const int    getTime() = 0;
    virtual const float    getAccX() = 0;
    virtual const float    getAccY() = 0;
    virtual const float    getAccZ() = 0;
    virtual const float    getBatteryCurrent() = 0;
    virtual const float    getMainBatteryVoltage() = 0;
};

