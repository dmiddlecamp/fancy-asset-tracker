// This #include statement was automatically added by the Spark IDE.
#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "GPS_Math.h"
#include "ublox_helpers.h"

#include <math.h>
#include <ctype.h>


//#include "Adafruit_MLX90614.h"
//Adafruit_MLX90614 mlx = Adafruit_MLX90614();



uint8_t internalANT[]={0xB5,0x62,0x06,0x13,0x04,0x00,0x00,0x00,0xF0,0x7D,0x8A,0x2A};
uint8_t externalANT[]={0xB5,0x62,0x06,0x13,0x04,0x00,0x01,0x00,0xF0,0x7D,0x8B,0x2E};


#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2, A5, A4, A3);
FuelGauge fuel;

#define MY_NAME "AssetTrackerPro"

#define CLICKTHRESHHOLD 100


int lastSecond = 0;
bool ledState = false;
//int lastLevel = 0;

unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
unsigned long lastReading = 0;
//time_t lastIdleCheckin = 0;

#define PUBLISH_DELAY (60 * 1000)

//// if no motion for 2 minutes, sleep! (milliseconds)
//#define NO_MOTION_IDLE_SLEEP_DELAY (2 * 60 * 1000)
//
//// lets wakeup every 6 hours and check in (seconds)
//#define HOW_LONG_SHOULD_WE_SLEEP (6 * 60 * 60)

// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
// (seconds)
#define MAX_IDLE_CHECKIN_DELAY (HOW_LONG_SHOULD_WE_SLEEP - 60)



// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(MANUAL);

//
//
//

//PRODUCT_ID(5397);
//PRODUCT_VERSION(1);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));





void setup() {
    // Am I in the gray asset tracker demo enclosure?
    //RGB.mirrorTo(B3,B2,B1,true, true);

//    // POWER TEMPERATURE SENSOR
//	  pinMode(A1,OUTPUT);
//    pinMode(B5,OUTPUT);
//    digitalWrite(B5, HIGH);
//    digitalWrite(A1, LOW);

    lastMotion = 0;
    lastPublish = 0;

    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    Serial.println("connecting...");


    Particle.connect();
    waitFor(Particle.connected, 60000);


    Serial.println("connected!");


    GPS.begin(9600);
    mySerial.begin(9600);
    Serial.begin(9600);


    //# request a HOT RESTART, in case we were in standby mode before.
    GPS.sendCommand("$PMTK101*32");
    delay(250);

    // request everything!
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    //delay(250);

    // turn off antenna updates
    GPS.sendCommand(PGCMD_NOANTENNA);
    delay(250);

    //delay(2000);    // give the module a long time to warm up just in case?

    // TODO: Make me a define switch
    // select internal antenna
    //antennaSelect(internalANT);
    //antennaSelect(externalANT);


    // TODO: cellular location suggestion

    suggest_time_to_gps();

    initAccel();
}


unsigned long lastMotionCheck = 0;

void loop() {
    checkGPS();

    unsigned long now = millis();

    if ((now - lastMotionCheck) > 50) {
        lastMotionCheck = now;

        bool hasMotion = digitalRead(WKP);
        digitalWrite(D7, (hasMotion) ? HIGH : LOW);
    }


    // have we published recently?
    //Serial.println("lastPublish is " + String(lastPublish));
    if (((now - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0)) {
        lastPublish = now;

        publishGPS();
    }

    suggest_time_to_gps();
    Particle.process();
}


void checkGPS() {
    int count = 0;
    int maxBytes = 4096;

    // process and dump everything from the module through the library.
    while ((mySerial.available()) && (count < maxBytes)) {
        char c = GPS.read();

        if (GPS.newNMEAreceived()) {
            Serial.println(GPS.lastNMEA());
            GPS.parse(GPS.lastNMEA());

            //Serial.println("my location is " + String::format(" %f, %f, ", GPS.latitude, GPS.longitude));
        }

        count++;
    }
}

void initAccel() {
    accel.begin(LIS3DH_DEFAULT_ADDRESS);

    // Default to 5kHz low-power sampling
    accel.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

    // Default to 2 gravities range
    accel.setRange(LIS3DH_RANGE_2_G);

    // listen for single-tap events at the threshold
    // keep the pin high for 1s, wait 1s between clicks

    //uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow
    accel.setClick(1, CLICKTHRESHHOLD);//, 0, 100, 50);
}

void publishGPS() {
    unsigned int msSinceLastMotion = (millis() - lastMotion);
    int motionInTheLastMinute = (msSinceLastMotion < 60000);

    /*
    String gps_line = String::format("%f,%f,%f,%f,
        convertDegMinToDecDeg(GPS.latitude), convertDegMinToDecDeg(GPS.longitude), GPS.altitude, GPS.speed);
    */

//    String gps_line =
//          "{\"lat\":"    + String(convertDegMinToDecDeg(GPS.latitude))
//        + ",\"lon\":-"   + String(convertDegMinToDecDeg(GPS.longitude))
//        + ",\"a\":"     + String(GPS.altitude)
//        + ",\"q\":"     + String(GPS.fixquality)
//        + ",\"spd\":"   + String(GPS.speed)

//        + ",\"s\": "  + String(GPS.satellites)
//        + ",\"vcc\":"   + String(fuel.getVCell())
//        + ",\"soc\":"   + String(fuel.getSoC())
//        + "}";
//
//    Particle.publish(MY_NAME + String("_location"), gps_line, 60, PRIVATE);



    float latitude = convertDegMinToDecDeg(GPS.latitude);
    float longitude = convertDegMinToDecDeg(GPS.longitude);

    if ((latitude != 0) && (longitude != 0)) {
        String trkJsonLoc = String("{")
            + "\"c_lat\":" + String(convertDegMinToDecDeg(GPS.latitude))
            + ",\"c_lng\":" + String(convertDegMinToDecDeg(GPS.longitude))
            + ",\"c_unc\":" + String(GPS.fixquality)
            + ",\"c_alt\":" + String(GPS.altitude)
            + ",\"mot\":"   + String(motionInTheLastMinute)
            + "}";
         Particle.publish("trk/loc", trkJsonLoc, 60, PRIVATE);
     }


    float batteryVoltage = fuel.getVCell();
    CellularSignal signalInfo = Cellular.RSSI();
    String devJson = String("{")
            + "\"vcell\":" + String::format("%.4f", batteryVoltage)
            + ",\"cell_rssi\":" + String(signalInfo.rssi)
            + ",\"cell_qual\":" + String(signalInfo.qual)
            + "}";
     Particle.publish("trk/dev", devJson, 60, PRIVATE);


    publishLevel();

//          int value = rand() * 100;
//     String sensorJson = String("{")
//            + "\"level\":" + String::format("%d", value)
//            + "}";
//     Particle.publish("trk/level", sensorJson, 60, PRIVATE);
}



void publishLevel() {
    unsigned int now = millis();
    if ((now - lastReading) < 5000) {
        return;
    }
    lastReading = now;


    accel.read();
    float aX = accel.x;
    float aY = accel.y;
    float aZ = accel.z;

    //float temperatureC = ((3300*analogRead(A0)/4096.0)-500)/10.0;
    //float temperatureF = (temperatureC * (9/5)) + 32;
    //int temperatureF = 103;
    //float temperatureF = mlx.readObjectTempF();

     String sensorJson = String("{")
            //+ "\"level\":" + String::format("%d", levelValue)
            //+ ",\"tempF\":" + String::format("%.2f", temperatureF)

            + ",\"x\":" + String::format("%.2f", aX)
            + ",\"y\":" + String::format("%.2f", aY)
            + ",\"z\":" + String::format("%.2f", aZ)
            + "}";

     Particle.publish("trk/env", sensorJson, 60, PRIVATE);
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

unsigned long lastTimeCheck = 0;
void suggest_time_to_gps() {
    if ((millis() - lastTimeCheck) < 5000) {
        return;
    }
    lastTimeCheck = millis();


    String gpsTime = String::format("%d/%d/%d %d:%d:%d",
    GPS.day, GPS.month, GPS.year, GPS.hour, GPS.minute, GPS.seconds);
    Serial.println("The GPS Time is " + gpsTime);

    if (GPS.year > 0) {
        Serial.println("I think I have a good date?");
        return;
    }

    //#define STARTING_LATITUDE_LONGITUDE_ALTITUDE "44.9778,-93.2650,200"
    //String locationString = STARTING_LATITUDE_LONGITUDE_ALTITUDE;

    Particle.syncTime();
    waitFor(Time.isValid, 30000);
    Serial.println("The time is: " + Time.timeStr());


    time_t timeValue = Time.now();
    String timeString = Time.format(timeValue, "%Y,%m,%d,%H,%M,%S");

    //Particle.publish("GPS", "Time was " + String(timeString));


    //  PMTK740,YYYY,MM,DD,hh,mm,ss*CS<CR><LF>
    //  PMTK741,Lat,Long,Alt,YYYY,MM,DD,hh,mm,ss *CS<CR><LF>
    //The packet contains reference location for the GPS receiver. To have faster TTFF, the accuracy of the location shall be better than 30km.
    // String locationTimeCmd = "PMTK741,"+locationString + "," + timeString;

    String gpsTimeCmd = "PMTK740," + timeString;


    Serial.println("Suggesting time to GPS module: " + timeString);

    sendGPSAidIni();

//    String cmd = String::format("$%s*%02X", gpsTimeCmd.c_str(), crc8(gpsTimeCmd));
//    mySerial.print(cmd + "\r\n");
//    Serial.println("Gps command was: " + cmd);

}
//
//void suggest_time_to_gps() {
//    if ((millis() - lastTimeCheck) < 5000) {
//        return;
//    }
//    lastTimeCheck = millis();
//
//
//    String gpsTime = String::format("%d/%d/%d %d:%d:%d",
//    GPS.day, GPS.month, GPS.year, GPS.hour, GPS.minute, GPS.seconds);
//    Serial.println("The GPS Time is " + gpsTime);
//
//    if (GPS.year > 0) {
//        Serial.println("I think I have a good date?");
//        return;
//    }
//
//    //#define STARTING_LATITUDE_LONGITUDE_ALTITUDE "44.9778,-93.2650,200"
//    //String locationString = STARTING_LATITUDE_LONGITUDE_ALTITUDE;
//
//    Particle.syncTime();
//    waitFor(Time.isValid, 30000);
//    Serial.println("The time is: " + Time.timeStr());
//
//
//    time_t timeValue = Time.now();
//    String timeString = Time.format(timeValue, "%Y,%m,%d,%H,%M,%S");
//
//    //Particle.publish("GPS", "Time was " + String(timeString));
//
//
//    //  PMTK740,YYYY,MM,DD,hh,mm,ss*CS<CR><LF>
//    //  PMTK741,Lat,Long,Alt,YYYY,MM,DD,hh,mm,ss *CS<CR><LF>
//    //The packet contains reference location for the GPS receiver. To have faster TTFF, the accuracy of the location shall be better than 30km.
//    // String locationTimeCmd = "PMTK741,"+locationString + "," + timeString;
//
//    String gpsTimeCmd = "PMTK740," + timeString;
//
//
//    Serial.println("Suggesting time to GPS module: " + timeString);
//
//    String cmd = String::format("$%s*%02X", gpsTimeCmd.c_str(), crc8(gpsTimeCmd));
//    mySerial.print(cmd + "\r\n");
//    Serial.println("Gps command was: " + cmd);
//
//    //GPS.sendCommand(cmd.c_str());     // why doesn't this support const char *...
//    //delay(250);
//
//
//    //cmd = String::format("$%s*%02x", locationTimeCmd.c_str(), crc8(locationTimeCmd));
//    //mySerial.println(cmd);
//    //GPS.sendCommand(cmd.c_str());     // why doesn't this support const char *...
//    //delay(250);
//}

//TODO: if we get a cellular location, and don't have a GPS fix, maybe suggest our location to the GPS



void antennaSelect(uint8_t *buf){

    for(uint8_t i=0;i<12;i++)
    {
        Serial1.write(buf[i]); //send the command to gps module
        Serial.print(buf[i],HEX);
        Serial.print(",");
    }
    Serial.println("");
}

bool value_within(float testValue, float referenceValue, float plusMinus) {
     return ((testValue >= (referenceValue - plusMinus))
          && (testValue <= (referenceValue + plusMinus)));
}

/*

    https://github.com/GAVLab/ublox/blob/master/examples/assist_example.cpp

*/
