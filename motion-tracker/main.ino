// This #include statement was automatically added by the Spark IDE.
#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "GPS_Math.h"

#include <math.h>
#include "math.h"
#include <ctype.h>




#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2, A5, A4, A3);
FuelGauge fuel;

#define MY_NAME "AssetTracker"

#define CLICKTHRESHHOLD 100


int lastSecond = 0;
bool ledState = false;

// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(SEMI_AUTOMATIC);


STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));



unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
time_t lastIdleCheckin = 0;

#define PUBLISH_DELAY (60 * 1000)

// if no motion for 3 minutes, sleep! (milliseconds)
#define NO_MOTION_IDLE_SLEEP_DELAY (3 * 60 * 1000)

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
    lastMotion = 0;
    lastPublish = 0;

    initAccel();

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
    if (lastIdleCheckin == 0) {
        lastIdleCheckin = Time.now();
    }

    unsigned long now = millis();

    if (lastMotion > now) { lastMotion = now; }
    if (lastPublish > now) { lastPublish = now; }

    checkGPS();

    // we'll be woken by motion, lets keep listening for more motion.
    // if we get two in a row, then we'll connect to the internet and start reporting in.
    bool hasMotion = digitalRead(WKP);
    digitalWrite(D7, (hasMotion) ? HIGH : LOW);
    if (hasMotion) {
        Serial.println("BUMP!");
        lastMotion = now;

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


    // use "now" instead of millis...  If it takes us a REALLY long time to connect, we don't want to
    // accidentally idle out.
    if ((now - lastMotion) > NO_MOTION_IDLE_SLEEP_DELAY) {
        // hey, it's been longer than xx minutes and nothing is happening, lets go to sleep.
        // if the accel triggers an interrupt, we'll wakeup earlier than that.

        Particle.publish(MY_NAME + String("_status"), "sleeping!");

        lastPublish = 0;
        lastMotion = 0;

        // Hey GPS, please stop using power, kthx.
        digitalWrite(D6, HIGH);

        // lets give ourselves a chance to settle, deal with anything pending, achieve enlightenment...
        delay(10*1000);
        System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP);
    }

    delay(10);
}


void checkGPS() {
    // process and dump everything from the module through the library.
    while (mySerial.available()) {
        char c = GPS.read();

        // lets echo the GPS output until we get a good clock reading, then lets calm things down.
        //if (!hasGPSTime) {
        //   Serial.print(c);
        //}

        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
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

    String gps_line =
          "{\"lat\":"    + String(convertDegMinToDecDeg(GPS.latitude))
        + ",\"lon\":-"   + String(convertDegMinToDecDeg(GPS.longitude))
        + ",\"a\":"     + String(GPS.altitude)
        + ",\"q\":"     + String(GPS.fixquality)
        + ",\"spd\":"   + String(GPS.speed)
        + ",\"mot\":"   + String(motionInTheLastMinute)
        + ",\"s\": "  + String(GPS.satellites)
        + ",\"vcc\":"   + String(fuel.getVCell())
        + ",\"soc\":"   + String(fuel.getSoC())
        + "}";

    Particle.publish(MY_NAME + String("_location"), gps_line, 60, PRIVATE);
}
