#ifndef frsky_h
#define frsky_h

//#define lowByte(w) ((uint8_t) ((w) & 0xff))
//#define highByte(w) ((uint8_t) ((w) >> 8))

#include "parser.h"
#include <SoftwareSerial.h>
#include "defines.h"

// The following data is sent from APM
//   $ - Header
// 0  Main battery voltage in V              
// 1  Battery current in mA                  
// 2  Battery remaining in %                 
// 3  GPS Status 0:No Fix, 2:2D Fix, 3:3D Fix
// 4  GPS Latitude in decimal degrees        
// 5  GPS Longitude in decimal degrees       
// 6  GPS Altitude in cm                     
// 7  GPS hdop                               
// 8  GPS Number of satelites in view        
// 9  GPS Ground speed in cm/s               
// 10 GPS Course in 1/100 degree                 
// 11 Altitude in cm         
// 12 Home altitude in cm    
// 13 APM mode                               
// 14 Compass                                
// 15 Throttle out           
// 16 Accel-X
// 17 Accel-Y
// 18 Accel-Z
// * - end

class FrSky
{
	
public:
		FrSky(SoftwareSerial* serialPort, SoftwareSerial* debugSerialPort, parser* p);
		FrSky(SoftwareSerial* serialPort, parser* p);
		~FrSky(void);
		void saveValues();
		void sendFrSky5Hz();
		void sendFrSky1Hz();
		void sendFrSky05Hz();
		unsigned char addBufferData(const char id);
		unsigned char writeBuffer(const int length);
		void printValues(SoftwareSerial* debugPort);
		
private:
		SoftwareSerial	*frSkySerial;
		SoftwareSerial	*debugPort;
		parser			*par;
		unsigned char	frskyBuffer[64];
		int				bufferLength;
		long FixInt(long val, byte mp);
		float gpsDdToDmsFormat(float ddm);
		byte lsByte(int value);
		byte msByte(int value);
};

#endif