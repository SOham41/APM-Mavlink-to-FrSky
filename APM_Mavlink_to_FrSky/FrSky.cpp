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

#include "FrSky.h"

FrSky::FrSky()
{
    bufferLength = 0;
}
FrSky::~FrSky(void)
{
}

void FrSky::sendFrSky05Hz(Stream& port, IFrSkyDataProvider* dataProvider)
{
    // Date, Time
    /*
    bufferLength += addBufferData(DATE);
    bufferLength += addBufferData(TIME);
    frskyBuffer[bufferLength++] = tail_value;
    bufferLength = writeBuffer(bufferLength);
    */
}

// Send 1000 ms frame
void FrSky::sendFrSky1Hz(Stream& port, IFrSkyDataProvider* dataProvider)
{

    // Course, Latitude, Longitude, Speed, Altitude (GPS), Fuel Level
    bufferLength += addBufferData(COURSE, dataProvider);
    bufferLength += addBufferData(LATITUDE,dataProvider);
    bufferLength += addBufferData(LONGITUDE, dataProvider);
    bufferLength += addBufferData(GPSSPEED, dataProvider);
    bufferLength += addBufferData(GPSALT,dataProvider);
    bufferLength += addBufferData(FUEL, dataProvider);
    frskyBuffer[bufferLength++] = tail_value;
    bufferLength = writeBuffer(bufferLength, port);

}

// Send 200 ms frame
void FrSky::sendFrSky5Hz(Stream& port, IFrSkyDataProvider* dataProvider)
{

    // Three-axis Acceleration Values, Altitude (variometer-0.01m), Tempature1, Temprature2, Voltage , Current & Voltage (Ampere Sensor) , RPM
    bufferLength += addBufferData(ACCX,dataProvider);
    bufferLength += addBufferData(ACCY, dataProvider);
    bufferLength += addBufferData(ACCZ, dataProvider);
    bufferLength += addBufferData(ALTITUDE, dataProvider);
    bufferLength += addBufferData(TEMP1, dataProvider);
    bufferLength += addBufferData(TEMP2, dataProvider);
    bufferLength += addBufferData(CURRENT, dataProvider);
    bufferLength += addBufferData(VFAS, dataProvider);
    bufferLength += addBufferData(RPM, dataProvider);
    frskyBuffer[bufferLength++] = tail_value;
    bufferLength = writeBuffer(bufferLength, port);

}

byte FrSky::lsByte(int value)
{
  return ((byte) ((value) & 0xff));
}

byte FrSky::msByte(int value)
{
  return ((byte) ((value) >> 8));
}

unsigned char FrSky::addBufferData(const char id, IFrSkyDataProvider* dataProvider)
{

    switch(id) {
        case GPSALT :
        {
            int gpsAltitude = dataProvider->getGpsAltitude();
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = GPSALT;
            frskyBuffer[bufferLength + 2] = lsByte(gpsAltitude);
            frskyBuffer[bufferLength + 3] = msByte(gpsAltitude);
            //we send 0 for decimal places
            frskyBuffer[bufferLength + 4] = header_value;
            frskyBuffer[bufferLength + 5] = GPSALT + decimal;
            frskyBuffer[bufferLength + 6] = 0;
            frskyBuffer[bufferLength + 7] = 0;
            return 8;
        }
        case TEMP1 :
        {
            // APM mode
            int temp1 = dataProvider->getTemp1();
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = TEMP1;
            frskyBuffer[bufferLength + 2] = lsByte(temp1);
            frskyBuffer[bufferLength + 3] = msByte(temp1);
            return 4;
        }
        case RPM :
        {
            // Throttle out
            int engineSpeed = dataProvider->getEngineSpeed();
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = RPM;
            frskyBuffer[bufferLength + 2] = lsByte(engineSpeed);
            frskyBuffer[bufferLength + 3] = msByte(engineSpeed);
            return 4;
        }
        case FUEL :
        {
            // Battery remaining in %
            int fuelLevel = dataProvider->getFuelLevel();
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = FUEL;
            frskyBuffer[bufferLength + 2] = lsByte(fuelLevel);
            frskyBuffer[bufferLength + 3] = msByte(fuelLevel);
            return 4;
        }
        case TEMP2 :
        {
            // GPS status mode, number of satelites in view
            int value = dataProvider->getTemp2();
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = TEMP2;
            frskyBuffer[bufferLength + 2] = lsByte(value);
            frskyBuffer[bufferLength + 3] = msByte(value);
            return 4;
        }
        case ALTITUDE :
        {
            int alt = (int)(dataProvider->getAltitude() * 100.0f + 0.5f);
            int16_t bp = alt / 100;
            int16_t ap = alt % 100;
            if (ap < 0) ap = -ap;
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = ALTITUDE;
            frskyBuffer[bufferLength + 2] = lsByte(bp);
            frskyBuffer[bufferLength + 3] = msByte(bp);
            frskyBuffer[bufferLength + 4] = header_value;
            frskyBuffer[bufferLength + 5] = ALTIDEC;
            frskyBuffer[bufferLength + 6] = lsByte(ap);
            frskyBuffer[bufferLength + 7] = msByte(ap);
            return 8;
        }
        case GPSSPEED :
        {
            // convert from m/s to knots  (multiply by 1.94384)
            uint16_t gpsSpeed  = (uint16_t)(dataProvider->getGpsGroundSpeed() * 1.94384f + 0.5f);
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = GPSSPEED;
            frskyBuffer[bufferLength + 2] = lsByte(gpsSpeed);
            frskyBuffer[bufferLength + 3] = msByte(gpsSpeed);
            // send zero for decimals
            frskyBuffer[bufferLength + 4] = header_value;
            frskyBuffer[bufferLength + 5] = GPSSPEED + decimal;
            frskyBuffer[bufferLength + 6] = 0;
            frskyBuffer[bufferLength + 7] = 0;
            return 8;
        }
        case LATITUDE :
        {
            int32_t val = dataProvider->getLatitude();
            //convert from 1E7 degrees to frsky format which is
            // BP = DDMM   degrees * 100 + minutes (whole part)
            // AP = MMMM   minutes (decimal part)
            int degrees = val / 10000000;
            uint32_t minutes = ((val % 10000000) * 6) / 100;
            if (minutes < 0) minutes = -minutes;
            uint16_t minutes_bp = 100 * degrees + minutes / 10000;
            uint16_t minutes_ap = minutes % 10000;

            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = LATITUDE;
            frskyBuffer[bufferLength + 2] = minutes_bp & 0xFF;
            frskyBuffer[bufferLength + 3] = (minutes_bp >> 8) & 0xFF;
            frskyBuffer[bufferLength + 4] = header_value;
            frskyBuffer[bufferLength + 5] = LATITUDE + decimal;
            frskyBuffer[bufferLength + 6] = minutes_ap & 0xFF;
            frskyBuffer[bufferLength + 7] = (minutes_ap >> 8) & 0xFF;
            frskyBuffer[bufferLength + 8] = header_value;
            frskyBuffer[bufferLength + 9] = NORTHSOUTH;
            frskyBuffer[bufferLength + 10] = (degrees < 0) ? 'S' : 'N';
            frskyBuffer[bufferLength + 11] = 0;
            return 12;
        }
        case LONGITUDE :
        {
            int32_t val = dataProvider->getLongitude();
            //convert from 1E7 degrees to frsky format which is
            // BP = DDMM   degrees * 100 + minutes (whole part)
            // AP = MMMM   minutes (decimal part)
            int degrees = val / 10000000;
            uint32_t minutes = ((val % 10000000) * 6) / 100;
            if (minutes < 0) minutes = -minutes;
            uint16_t minutes_bp = 100 * degrees + minutes / 10000;
            uint16_t minutes_ap = minutes % 10000;

            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = LONGITUDE;
            frskyBuffer[bufferLength + 2] = minutes_bp & 0xFF;
            frskyBuffer[bufferLength + 3] = (minutes_bp >> 8) & 0xFF;
            frskyBuffer[bufferLength + 4] = header_value;
            frskyBuffer[bufferLength + 5] = LONGITUDE + decimal;
            frskyBuffer[bufferLength + 6] = minutes_ap & 0xFF;
            frskyBuffer[bufferLength + 7] = (minutes_ap >> 8) & 0xFF;
            frskyBuffer[bufferLength + 8] = header_value;
            frskyBuffer[bufferLength + 9] = EASTWEST;
            frskyBuffer[bufferLength + 10] = (degrees < 0 ) ? 'W' : 'E';
            frskyBuffer[bufferLength + 11] = 0;
            return 12;
        }
        case COURSE :
        {
            int16_t course = dataProvider->getCourse();
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = COURSE;
            frskyBuffer[bufferLength + 2] = lsByte(course);
            frskyBuffer[bufferLength + 3] = msByte(course);
      // send zero for decimal part
            frskyBuffer[bufferLength + 4] = header_value;
            frskyBuffer[bufferLength + 5] = COURSE + decimal;
            frskyBuffer[bufferLength + 6] = 0;
            frskyBuffer[bufferLength + 7] = 0;
            return 8;
        }
        case DATE :
        {
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = DATE;
            frskyBuffer[bufferLength + 2] = lsByte(dataProvider->getDate());
            frskyBuffer[bufferLength + 3] = msByte(dataProvider->getDate());
            return 4;
        }
        case YEAR :
        {
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = DATE;
            frskyBuffer[bufferLength + 2] = lsByte(dataProvider->getYear());
            frskyBuffer[bufferLength + 3] = msByte(dataProvider->getYear());
            return 4;
        }
        case TIME :
        {
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = TIME;
            frskyBuffer[bufferLength + 2] = lsByte(dataProvider->getTime());
            frskyBuffer[bufferLength + 3] = msByte(dataProvider->getTime());
            return 4;
        }
        case SECOND :
        {
            return 0;
        }
        case ACCX :
        {
            //float accX = par->termToDecimal(17) / 100.0f;
            float accX = dataProvider->getAccX()*1000.0f;
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = ACCX;
            frskyBuffer[bufferLength + 2] = lsByte((int)(accX));
            frskyBuffer[bufferLength + 3] = msByte((int)(accX));
            return 4;
        }
        case ACCY :
        {
            //float accY = par->termToDecimal(18) / 100.0f;
            float accY =  dataProvider->getAccY()*1000.0f;
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = ACCY;
            frskyBuffer[bufferLength + 2] = lsByte((int)(accY));
            frskyBuffer[bufferLength + 3] = msByte((int)(accY));
            return 4;
        }
        case ACCZ :
        {
            //float accZ = par->termToDecimal(19) / 100.0f;
            float accZ = dataProvider->getAccZ()*1000.0f;
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = ACCZ;
            frskyBuffer[bufferLength + 2] = lsByte((int)(accZ));
            frskyBuffer[bufferLength + 3] = msByte((int)(accZ));
            return 4;
        }
        case CURRENT :
        {
            float current = dataProvider->getBatteryCurrent() * 10.0f + 0.5f;
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = CURRENT;
            frskyBuffer[bufferLength + 2] = lsByte((int)(current));
            frskyBuffer[bufferLength + 3] = msByte((int)(current));
            return 4;
        }
        case VFAS :
        {
            float batteryVoltage = dataProvider->getMainBatteryVoltage() * 10.0f + 0.5f;
            frskyBuffer[bufferLength] = header_value;
            frskyBuffer[bufferLength + 1] = VFAS;
            frskyBuffer[bufferLength + 2] = lsByte((int)batteryVoltage);
            frskyBuffer[bufferLength + 3] = msByte((int)batteryVoltage);
            return 4;
        }
        default :
            return 0;
  }
  return 0;
}

unsigned char FrSky::writeBuffer(const int length, Stream &frSkySerial)
{

  int i = 0;
  while(i < length)
  {
    // If a data value is equal to header (0x5E), tail (0x5E) or escape (0x5D) value exchange it.
    // There is always a id and two bytes between header and tail therefor every 4th byte is a header and should not be checked
    if ((i % 4))
    {
      switch (frskyBuffer[i])
      {
        case header_value :
        {
          frSkySerial.write((byte)0x5D);
          frSkySerial.write((byte)0x3E);
          break;
        }
        case escape_value :
        {
          frSkySerial.write((byte)0x5D);
          frSkySerial.write((byte)0x3D);
          break;
        }
        default :
        {
          frSkySerial.write((byte)frskyBuffer[i]);
        }
      }
    }
    else
    {
      frSkySerial.write((byte)frskyBuffer[i]);
    }

    i++;
  }

  return 0;
}

void FrSky::printValues(Stream &port, IFrSkyDataProvider* dataProvider)
{
    port.print("Voltage: ");
    port.print(dataProvider->getMainBatteryVoltage(), 2);
    port.print(" Current: ");
    port.print(dataProvider->getBatteryCurrent(), 2);
    port.print(" Fuel: ");
    port.print(dataProvider->getFuelLevel());
    port.print(" Latitude: ");
    port.print(dataProvider->getLatitude());
    port.print(" Longitude: ");
    port.print(dataProvider->getLongitude());
    port.print(" GPS Alt: ");
    port.print(dataProvider->getGpsAltitude(), 2);
    //serialPort->print(" GPS hdop: ");
    //serialPort->print(dataProvider->getGpsHdop(), 2);
    port.print(" GPS status + sats: ");
    port.print(dataProvider->getTemp2());
    port.print(" GPS speed: ");
    port.print(dataProvider->getGpsGroundSpeed(), 2);
    port.print(" Home alt: ");
    port.print(dataProvider->getAltitude(), 2);
    port.print(" Mode: ");
    port.print(dataProvider->getTemp1());
    port.print(" Course: ");
    port.print(dataProvider->getCourse());
    port.print(" RPM: ");
    port.print(dataProvider->getEngineSpeed());
    port.print(" AccX: ");
    port.print(dataProvider->getAccX(), 2);
    port.print(" AccY: ");
    port.print(dataProvider->getAccY(), 2);
    port.print(" AccZ: ");
    port.print(dataProvider->getAccZ(), 2);
    port.println("");
}
