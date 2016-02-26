// This #include statement was automatically added by the Spark IDE.
#include "Adafruit_GPS.h"

#include <math.h>

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

int lastSecond = 0;
bool ledState = false;

// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(SEMI_AUTOMATIC);


void setup() {
    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    GPS.begin(9600);
    mySerial.begin(9600);
    Serial.begin(9600);


    //# request a HOT RESTART, in case we were in standby mode before.
    GPS.sendCommand("$PMTK101*32");
    delay(250);


    // request everything!
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    delay(250);

    // turn off antenna updates
    GPS.sendCommand(PGCMD_NOANTENNA);
    delay(250);
}



void loop() {

    bool hasGPSTime = ((GPS.year != 80) && (GPS.year != 0));

    // process and dump everything from the module through the library.
    while (mySerial.available()) {
        char c = GPS.read();

        // lets echo the GPS output until we get a good clock reading, then lets calm things down.
        if (!hasGPSTime) {
            Serial.print(c);
        }


        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
        }
    }

    // wait 60 seconds, or until we have a fix before we get online to help get a quicker fix.
    if (Particle.connected() == false) {
        if ((GPS.latitude != 0) || (millis() > 120000)) {
            Particle.connect();
        }
    }


    if (GPS.seconds != lastSecond) {
        lastSecond = GPS.seconds;

        // every second, toggle the LED.
        digitalWrite(D7, (ledState) ? HIGH : LOW);
        ledState = !ledState;


        if (hasGPSTime) {
            String currentTime = String::format("GPS TIME is %d/%d/%d at %d:%d:%d, location is %f, %f",
                GPS.month, GPS.day, GPS.year,
                GPS.hour, GPS.minute, GPS.seconds,

                GPS.latitude, GPS.longitude
                );

            Serial.println(currentTime);
        }
        else {
            Serial.println("GPS TIME: searching the skies...");
        }
    }
}