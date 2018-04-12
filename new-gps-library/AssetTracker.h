#include "application.h"
#include "math.h"

#include "Adafruit_LIS3DH.h"
#include "Adafruit_GPS.h"

#ifndef AssetTracker_h
#define AssetTracker_h

class AssetTracker {

 public:

  AssetTracker();

  void
    begin(void),
    updateGPS(void),
    gpsOn(void),
    gpsOff(void);
  int
    readX(void),
    readY(void),
    readZ(void),
    readXYZmagnitude(void);
  float
    readLat(void),
    readLon(void);
  bool
    gpsFix(void);
  char
    checkGPS(void),
    *preNMEA(void);
  String
    readLatLon(void);
 private:

};

#endif
