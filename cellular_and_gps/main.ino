// This #include statement was automatically added by the Spark IDE.
#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "GPS_Math.h"
#include "google-maps-device-locator.h"

GoogleMapsDeviceLocator locator;

#include <math.h>
#include "math.h"
#include <ctype.h>

/*

*/

#define STARTING_LATITUDE_LONGITUDE_ALTITUDE NULL
uint8_t internalANT[]={0xB5,0x62,0x06,0x13,0x04,0x00,0x00,0x00,0xF0,0x7D,0x8A,0x2A};
uint8_t externalANT[]={0xB5,0x62,0x06,0x13,0x04,0x00,0x01,0x00,0xF0,0x7D,0x8B,0x2E};


#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2, A5, A4, A3);
FuelGauge fuel;

#define MY_NAME "AssetTrackerPro"

#define CLICKTHRESHHOLD 100
#define CELL_DELAY 15000
unsigned long lastCellLocation = 0;


int lastSecond = 0;
bool ledState = false;
int lastTemperature = 0;
bool usingCellularLocation = false;

// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(SEMI_AUTOMATIC);

//
//
//

//PRODUCT_ID(3917);
//PRODUCT_VERSION(5);

STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));



unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
unsigned long lastReading = 0;
unsigned long lastMotionCheck = 0;

float cellularLatitude;
float cellularLongitude;
float cellularAccuracy;

time_t lastIdleCheckin = 0;

#define PUBLISH_DELAY (60 * 1000)

// if no motion for 2 minutes, sleep! (milliseconds)
#define NO_MOTION_IDLE_SLEEP_DELAY (5 * 60 * 1000)

// lets wakeup every 6 hours and check in (seconds)
#define HOW_LONG_SHOULD_WE_SLEEP (12 * 60 * 60)

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


    // POWER TEMPERATURE SENSOR

	pinMode(A1,OUTPUT);
	pinMode(A0,INPUT);
    pinMode(B5,OUTPUT);
    digitalWrite(B5, HIGH);
    digitalWrite(A1, LOW);

    lastMotion = 0;
    lastPublish = 0;

    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);


    Particle.connect();

    delay(2000);    // give the module a long time to warm up just in case?

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


    // select internal antenna
    //antennaSelect(internalANT);
    //antennaSelect(externalANT);

    initAccel();

    locator.withSubscribe(locationCallback).withLocatePeriodic(60);
}



void loop() {
    if (lastIdleCheckin == 0) {
        lastIdleCheckin = Time.now();
    }

    unsigned long now = millis();
    if (lastMotion > now) { lastMotion = now; }


    checkGPS();


    if ((now - lastMotionCheck) > 50) {
        lastMotionCheck = now;

        bool hasMotion = digitalRead(WKP);
        digitalWrite(D7, (hasMotion) ? HIGH : LOW);
    }


    // have we published recently?
    //Serial.println("lastPublish is " + String(lastPublish));
    if (((now - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0)) {
        publishGPS();
    }

    if ((now - lastReading) > PUBLISH_DELAY) {
        publishLevel();
        lastReading = now;
    }


    locator.loop();
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
            + ",\"mot\":"   + String(motionInTheLastMinute)
            + "}";
         Particle.publish("trk/loc", trkJsonLoc, PRIVATE, WITH_ACK);
         lastPublish = millis();
    }
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




void publishLevel() {
    accel.read();
    float aX = accel.x;
    float aY = accel.y;
    float aZ = accel.z;

    float temperatureC = ((3300*analogRead(A0)/4096.0)-500)/10.0;
    float temperatureF = (temperatureC * (9/5)) + 32;

    String sensorJson = String("{")
        + "\"tempF\":" + String::format("%f", temperatureF)
        + ",\"x\":" + String::format("%.2f", aX)
        + ",\"y\":" + String::format("%.2f", aY)
        + ",\"z\":" + String::format("%.2f", aZ)
        + "}";
    Particle.publish("trk/env", sensorJson, PRIVATE, WITH_ACK);

    float batteryVoltage = fuel.getVCell();
    CellularSignal signalInfo = Cellular.RSSI();
    String devJson = String("{")
        + "\"vcell\":" + String::format("%.4f", batteryVoltage)
        + ",\"cell_rssi\":" + String(signalInfo.rssi)
        + ",\"cell_qual\":" + String(signalInfo.qual)
        + "}";
    Particle.publish("trk/dev", devJson, PRIVATE, WITH_ACK);
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

void antennaSelect(uint8_t *buf){
    for(uint8_t i=0;i<12;i++)
    {
        Serial1.write(buf[i]); //send the command to gps module
        Serial.print(buf[i],HEX);
        Serial.print(",");
    }
    Serial.println("");
}
