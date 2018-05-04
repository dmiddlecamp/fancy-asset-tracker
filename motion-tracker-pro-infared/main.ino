// This #include statement was automatically added by the Spark IDE.
#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "GPS_Math.h"

#include <math.h>
#include "math.h"
#include <ctype.h>

#include "google-maps-device-locator.h"

GoogleMapsDeviceLocator locator;

float cellularLatitude;
float cellularLongitude;
float cellularAccuracy;
bool usingCellularLocation = false;

#include "Adafruit_MLX90614.h"

Adafruit_MLX90614 mlx = Adafruit_MLX90614();


#define STARTING_LATITUDE_LONGITUDE_ALTITUDE "44.9778,-93.2650,200"
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
int lastLevel = 0;

// lets keep the radio off until we get a fix, or 2 minutes go by.
//SYSTEM_MODE(SEMI_AUTOMATIC);

//
//
//

PRODUCT_ID(5397);
PRODUCT_VERSION(3);

STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));



unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
unsigned long lastReading = 0;
time_t lastIdleCheckin = 0;

#define PUBLISH_DELAY (60 * 1000)

// if no motion for 2 minutes, sleep! (milliseconds)
#define NO_MOTION_IDLE_SLEEP_DELAY (2 * 60 * 1000)

// lets wakeup every 6 hours and check in (seconds)
#define HOW_LONG_SHOULD_WE_SLEEP (6 * 60 * 60)

// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
// (seconds)
#define MAX_IDLE_CHECKIN_DELAY (HOW_LONG_SHOULD_WE_SLEEP - 60)


//
//
//loop
//set wakeOnMotion with accelerometer
//<wake>
//send GPS every 1? minute while in motion (or just publish start and end of trip)
//when no motion for 3 minutes - check battery, publish if low
//if after 6:30 PM - deep sleep till morning
//wake on morning - send position and check battery


void setup() {
    // mirror RGB PINS
    RGB.mirrorTo(B3,B2,B1,true, true);

//    // POWER TEMPERATURE SENSOR
//	pinMode(A1,OUTPUT);
//    pinMode(B5,OUTPUT);
//    digitalWrite(B5, HIGH);
//    digitalWrite(A1, LOW);

    // water sensor
    pinMode(A1, OUTPUT);
    pinMode(A0, INPUT);

    digitalWrite(A1, HIGH);         // power pin for water sensor

    lastMotion = 0;
    lastPublish = 0;

    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    // wait a little for the GPS to wakeup
    delay(250);

    GPS.begin(9600);
    mySerial.begin(9600);
    Serial.begin(9600);


    //# request a HOT RESTART, in case we were in standby mode before.
    GPS.sendCommand("$PMTK101*32");
    delay(250);

    // request everything!
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    //delay(250);

    // turn off antenna updates
    GPS.sendCommand(PGCMD_NOANTENNA);
    delay(250);

    delay(2000);    // give the module a long time to warm up just in case?

    // select internal antenna
    antennaSelect(internalANT);
    //antennaSelect(externalANT);


    //suggest_time_and_location();

    initAccel();

    mlx.begin();

    locator.withSubscribe(locationCallback).withLocatePeriodic(60);
}



void loop() {
    if (lastIdleCheckin == 0) {
        lastIdleCheckin = Time.now();
    }

    unsigned long now = millis();

    if (lastMotion > now) { lastMotion = now; }
    //if (lastPublish > now) { lastPublish = now; }

    checkGPS();

    // we'll be woken by motion, lets keep listening for more motion.
    // if we get two in a row, then we'll connect to the internet and start reporting in.
    bool hasMotion = digitalRead(WKP);
    digitalWrite(D7, (hasMotion) ? HIGH : LOW);
    if (hasMotion) {
        Serial.println("BUMP!");
        lastMotion = now;

        Particle.publish("motion!", String(millis()));
        delay(500);

        if (Particle.connected() == false) {
            Serial.println("CONNECTING DUE TO MOTION!");
            Particle.connect();
        }
    }

    // use the real-time-clock here, instead of millis.
    if ((Time.now() - lastIdleCheckin) >= MAX_IDLE_CHECKIN_DELAY) {

        // it's been too long!  Lets say hey!
        if (Particle.connected() == false) {
            Serial.println("CONNECTING DUE TO IDLE!");
            Particle.connect();
        }

        Particle.publish(MY_NAME + String("_status"), "miss you <3");
        lastIdleCheckin = Time.now();
    }


    // have we published recently?
    //Serial.println("lastPublish is " + String(lastPublish));
    if (((millis() - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0)) {
        lastPublish = millis();

        publishGPS();
    }


//    // use "now" instead of millis...  If it takes us a REALLY long time to connect, we don't want to
//    // accidentally idle out.
//    if ((now - lastMotion) > NO_MOTION_IDLE_SLEEP_DELAY) {
//        // hey, it's been longer than xx minutes and nothing is happening, lets go to sleep.
//        // if the accel triggers an interrupt, we'll wakeup earlier than that.
//
//        Particle.publish(MY_NAME + String("_status"), "sleeping!");
//
//        lastPublish = 0;
//        lastMotion = 0;
//
//        // Hey GPS, please stop using power, kthx.
//        digitalWrite(D6, HIGH);
//
//        // lets give ourselves a chance to settle, deal with anything pending, achieve enlightenment...
//        delay(10*1000);
//        System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP);
//    }


    publishLevel();

    locator.loop();

    delay(10);
}


void checkGPS() {
    // process and dump everything from the module through the library.
    while (mySerial.available()) {
        char c = GPS.read();

        if (GPS.newNMEAreceived()) {
            Serial.println(GPS.lastNMEA());
            GPS.parse(GPS.lastNMEA());

            if (GPS.latitude != 0) {
                usingCellularLocation = false;
            }

            //Serial.println("my location is " + String::format(" %f, %f, ", GPS.latitude, GPS.longitude));
        }
    }
}




void initAccel() {
    accel.begin(LIS3DH_DEFAULT_ADDRESS);

    // Default to 5kHz low-power sampling
    accel.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

    // Default to 4 gravities range
    accel.setRange(LIS3DH_RANGE_4_G);

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
//        + ",\"mot\":"   + String(motionInTheLastMinute)
//        + ",\"s\": "  + String(GPS.satellites)
//        + ",\"vcc\":"   + String(fuel.getVCell())
//        + ",\"soc\":"   + String(fuel.getSoC())
//        + "}";
//
//    Particle.publish(MY_NAME + String("_location"), gps_line, 60, PRIVATE);



//---
    float latitude, longitude, accuracy;

    if (usingCellularLocation) {
        latitude = cellularLatitude;
        longitude = cellularLongitude;
        accuracy = cellularAccuracy;
    }
    else {
        latitude = convertDegMinToDecDeg(GPS.latitude);
        longitude = convertDegMinToDecDeg(GPS.longitude);
        accuracy = GPS.fixquality;
    }

    if ((latitude != 0) && (longitude != 0)) {
        String trkJsonLoc = String("{")
            + "\"c_lat\":" + String(latitude)
            + ",\"c_lng\":" + String(longitude)
            + ",\"c_unc\":" + String(accuracy)
            + ",\"c_alt\":" + String(GPS.altitude)
            + "}";
         Particle.publish("trk/loc", trkJsonLoc, PRIVATE, WITH_ACK);
         lastPublish = millis();
    }
    //---


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
    int levelValue = mlx.readObjectTempF();

    accel.read();
    float aX = accel.x;
    float aY = accel.y;
    float aZ = accel.z;

    //float temperatureC = ((3300*analogRead(A0)/4096.0)-500)/10.0;
    //float temperatureF = (temperatureC * (9/5)) + 32;
    //int temperatureF = 103;
    float temperatureF = mlx.readObjectTempF();

     unsigned int now = millis();
     int elapsed = (now - lastReading);

    // if it's different and at least 500ms have elapsed, or 15 seconds have elapsed
    bool differentEnough = !value_within(levelValue, lastLevel, 1);

    //if  ((levelValue != lastLevel) && (elapsed > 1500) || (elapsed> 15000)) {
    if  (differentEnough && (elapsed > 1500) || (elapsed> 15000)) {
         lastReading = now;
         lastLevel = levelValue;

         String sensorJson = String("{")
                + "\"level\":" + String::format("%d", levelValue)
                + ",\"tempF\":" + String::format("%.2f", temperatureF)
                + ",\"x\":" + String::format("%.2f", aX)
                + ",\"y\":" + String::format("%.2f", aY)
                + ",\"z\":" + String::format("%.2f", aZ)

                + "}";
         Particle.publish("trk/env", sensorJson, 60, PRIVATE);


     }
}


//int getLevelReading() {
//
//    //
//    int emptyLevelValue = 3500;
//    int fullLevelValue = 2460;
//    // about 2 inches of water ->
//    //int levelValue = analogRead(D0) - 2434;
//
//    //delay(50);
//    int levelReading = analogRead(A0);
//    int levelValue = map(levelReading, fullLevelValue, emptyLevelValue, 0, 100);
//    levelValue = 100 - levelValue;  // flip it
//
//    //Serial.println("water level is " + String(levelReading) + " percentage full is " + String(levelValue));
//
//    return levelValue;
//}

int crc8(String str) {
  int len = str.length();
  const char * buffer = str.c_str();

  int crc = 0;
  for(int i=0;i<len;i++) {
    crc ^= (buffer[i] & 0xff);
  }
  return crc;
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

void locationCallback(float lat, float lon, float accuracy) {
  // Handle the returned location data for the device. This method is passed three arguments:
  // - Latitude
  // - Longitude
  // - Accuracy of estimated location (in meters)

  //Particle.publish("cellLocation" "CB called " + String(lat));

  // if we don't have a fix from our GPS sensor yet, then use the cellular location
  if (GPS.latitude == 0) {
    cellularLatitude = lat;
    cellularLongitude = lon;
    cellularAccuracy = accuracy;
    usingCellularLocation = true;
  }
}

//
//#include "AssetTracker.h"
//
//AssetTracker t = AssetTracker();
//

//void setup() {
//
//    t.begin();
//
//    t.gpsOn();
//
//    Serial.begin(9600);
//    delay(2000);
//
//    // uncomment the appropriate line
//    antennaSelect(internalANT);
//    //antennaSelect(externalANT);
//
//    delay(300);
//
//}
//
//void loop() {
//
//    t.updateGPS();
//    Serial.println(t.preNMEA());
//
//    delay(1000);
//
//}
//
