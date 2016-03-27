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
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

#undef PSTR
#define PSTR(s) (__extension__({static char __c[] PROGMEM = (s); &__c[0];}))

#include <SoftwareSerial.h>
#include <FlexiTimer2.h>
#include "Mavlink.h"
#include "FrSky.h"
#include "SimpleFIFO.h"
#include <GCS_MAVLink.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <Adafruit_NeoPixel.h>

#define HEARTBEATLED 13
#define HEARTBEATFREQ 500
#define NUMPIXELS      22

// Do not enable both at the same time
#define DEBUG
//#define DEBUGFRSKY

Mavlink *dataProvider;
FrSky *frSky;
SoftwareSerial *frSkySerial;

Adafruit_NeoPixel leds(NUMPIXELS, 5, NEO_GRB + NEO_KHZ800);

#ifdef DEBUG
Stream *debugSerial = NULL;
#elif defined DEBUGFRSKY
SoftwareSerial *frskyDebugSerial;
#endif

SimpleFIFO<char, 128> queue;

int        counter = 0;
unsigned long    hbMillis = 0;
unsigned long    rateRequestTimer = 0;
byte    hbState;
bool    firstParse = false;

void setup() {

// Debug serial port pin 11 rx, 12 tx
#ifdef DEBUG
    // debugSerial = new SoftwareSerial(12, 11);
    Serial.begin(115200);
    debugSerial = &Serial;
    // debugSerial->begin(38400);
#elif defined DEBUGFRSKY
    frskyDebugSerial = new SoftwareSerial(12, 11);
    frskyDebugSerial->begin(38400);
#endif

    leds.begin();
    leds.setPixelColor(0, 0x110000);
    leds.show();

    // FrSky data port pin 6 rx, 5 tx
    frSkySerial = new SoftwareSerial(2, 3, true);
    frSkySerial->begin(9600);
    // Incoming data from APM
    Serial1.begin(57600);
    // Serial1.flush();

#ifdef DEBUG
    debugSerial->println("Initializing...");
    debugSerial->print("Free ram: ");
    debugSerial->print(freeRam());
    debugSerial->println(" bytes");
#endif
    dataProvider = new Mavlink(&Serial1, debugSerial);


    frSky = new FrSky();

    digitalWrite(HEARTBEATLED, HIGH);
    hbState = HIGH;

    FlexiTimer2::set(200, 1.0/1000, sendFrSkyData); // call every 200 1ms "ticks"
    FlexiTimer2::start();

#ifdef DEBUG
    debugSerial->println("Waiting for APM to boot.");
#endif

    // Blink fast a couple of times to wait for the APM to boot
    for (int i = 0; i < 100; i++)
    {
        if (i % 2)
        {
            digitalWrite(HEARTBEATLED, HIGH);
            hbState = HIGH;
        }
        else
        {
            digitalWrite(HEARTBEATLED, LOW);
            hbState = LOW;
        }
        delay(5);
    }

#ifdef DEBUG
    debugSerial->println("Initialization done.");
    debugSerial->print("Free ram: ");
    debugSerial->print(freeRam());
    debugSerial->println(" bytes");
#endif

    //done, leds off
    leds.setPixelColor(0, 0x000000);
    leds.show();
    leds.setBrightness(30);
}

void loop() {
    while (dataProvider->getChannel().available() > 0) {
        if (queue.count() < 128) {
            char c = dataProvider->getChannel().read();
            queue.enqueue(c);
        } else {
            #ifdef DEBUG
            debugSerial->println("QUEUE IS FULL!");
            #endif
        }
    }

    processData();
    updateHeartbeat();

    uint32_t time = millis();
    static uint32_t lastLed = 0;
    if ((time - lastLed) > 50) {
        uint16_t j = (millis() >> 3) % 256;
        for (uint16_t i=0; i < NUMPIXELS / 2; i++) {
            uint32_t right = 0, left = 0;
            if (dataProvider->getMotorArmed()) {
                leds.setBrightness(100);
                left = ((i * 256 / leds.numPixels()) + j) & 255; //blue
                right = left << 8; //green
                if (dataProvider->apmMode > 10) { //rtl, loiter, .. auto modes
                    if (i == NUMPIXELS / 2 - 1) //back
                        if (j < 190) //75% on blinking
                            left = right = 0x0000FF;
                }
            } else {
                uint32_t rainbow = Wheel(((i * 256 / leds.numPixels()) + j) & 255);
                uint32_t statsColor = (dataProvider->getGpsStatus() > 2)? 0x00FF00 : 0x0000FF; //ok=green
                left = (dataProvider->numberOfSatelites >= i)? statsColor : 0;
                if (dataProvider->numberOfSatelites == i)
                    left = (j > 125)? statsColor : 0; //blink last
                right = (map(dataProvider->getMainBatteryVoltage(), 10, 12.6, 0, 11) >= i)? rainbow : 0;
            }
            leds.setPixelColor(i, left);
            leds.setPixelColor(i + NUMPIXELS / 2, right);
        }
        leds.show();
        lastLed = time;
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return leds.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return leds.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return leds.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void updateHeartbeat() {
    long currentMilillis = millis();
    if(currentMilillis - hbMillis > HEARTBEATFREQ) {
        hbMillis = currentMilillis;
        digitalWrite(HEARTBEATLED, (hbState == LOW));
    }
}

void sendFrSkyData() {
    counter++;

    if (counter >= 25) {
        // Send 5000 ms frame
        frSky->sendFrSky05Hz(*frSkySerial, dataProvider);
        counter = 0;
    } else if ((counter % 5) == 0) {
        // Send 1000 ms frame
        frSky->sendFrSky1Hz(*frSkySerial, dataProvider);
    } else {
        // Send 200 ms frame
        frSky->sendFrSky5Hz(*frSkySerial, dataProvider);
    }
}

void processData() {
    while (queue.count() > 0) {
        bool done = dataProvider->parseMessage(queue.dequeue());
        if (done && !firstParse) {
            firstParse = true;
            #ifdef DEBUG
            debugSerial->println("First parse done. Start sending on frSky port.");
            #endif
        }
    }
}


int freeRam () {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
