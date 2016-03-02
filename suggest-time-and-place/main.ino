// This #include statement was automatically added by the Spark IDE.
#include "Adafruit_GPS.h"

#include <math.h>

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

int lastSecond = 0;
bool ledState = false;

// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(SEMI_AUTOMATIC);

#define STARTING_LATITUDE_LONGITUDE_ALTITUDE "41.8369,-87.6847,200"


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


    //
    //  The fun part!
    //
    suggest_time_and_location();



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

void suggest_time_and_location() {
    String locationString = STARTING_LATITUDE_LONGITUDE_ALTITUDE;


    //TODO: guarantee that the clock has actually been set by the cloud, and we're not early.
    //Particle.syncTime();

    time_t timeValue = Time.now();
    String timeString = Time.format(timeValue, "%Y,%m,%d,%H,%M,%S");



    Particle.publish("GPS", "Time was " + String(timeString));


    //  PMTK740,YYYY,MM,DD,hh,mm,ss*CS<CR><LF>
    //  PMTK741,Lat,Long,Alt,YYYY,MM,DD,hh,mm,ss *CS<CR><LF>
    //The packet contains reference location for the GPS receiver. To have faster TTFF, the accuracy of the location shall be better than 30km.
    String gpsTimeCmd = "PMTK740," + timeString;
    String locationTimeCmd = "PMTK741,"+locationString + "," + timeString;


    String cmd = String::format("$%s*%02x", gpsTimeCmd.c_str(), crc8(gpsTimeCmd));
    mySerial.println(cmd);
    //GPS.sendCommand(cmd.c_str());     // why doesn't this support const char *...
    delay(250);


    cmd = String::format("$%s*%02x", locationTimeCmd.c_str(), crc8(locationTimeCmd));
    mySerial.println(cmd);
    //GPS.sendCommand(cmd.c_str());     // why doesn't this support const char *...
    delay(250);

}


int crc8(String str) {
  int len = str.length();
  const char * buffer = str.c_str();

  int crc = 0;
  for(int i=0;i<len;i++) {
    crc ^= (buffer[i] & 0xff);
  }
  return crc;
}