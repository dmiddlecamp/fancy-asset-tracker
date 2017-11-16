#include "Particle.h"
#include "google-maps-device-locator.h"

#if Wiring_Cellular
# include "CellularHelper.h"
#endif

static char requestBuf[256];
static char *requestCur;
static int numAdded = 0;

GoogleMapsDeviceLocator::GoogleMapsDeviceLocator() : locatorMode(LOCATOR_MODE_MANUAL), periodMs(10000), eventName("deviceLocator"),
	stateTime(0), state(CONNECT_WAIT_STATE), callback(NULL), waitAfterConnect(8000) {

}

GoogleMapsDeviceLocator::~GoogleMapsDeviceLocator() {

}

GoogleMapsDeviceLocator &GoogleMapsDeviceLocator::withLocateOnce() {
	locatorMode = LOCATOR_MODE_ONCE;
	return *this;
}

GoogleMapsDeviceLocator &GoogleMapsDeviceLocator::withLocatePeriodic(unsigned long secondsPeriodic) {
	locatorMode = LOCATOR_MODE_PERIODIC;
	if (secondsPeriodic < 5) {
		secondsPeriodic = 5;
	}
	periodMs = secondsPeriodic * 1000;
	return *this;
}

GoogleMapsDeviceLocator &GoogleMapsDeviceLocator::withEventName(const char *name) {
	this->eventName = name;
	return *this;
}

GoogleMapsDeviceLocator &GoogleMapsDeviceLocator::withSubscribe(GoogleMapsDeviceLocatorSubscriptionCallback callback) {
	this->callback = callback;

	snprintf(requestBuf, sizeof(requestBuf), "hook-response/%s/%s", eventName.c_str(), System.deviceID().c_str());

	Particle.subscribe(requestBuf, &GoogleMapsDeviceLocator::subscriptionHandler, this, MY_DEVICES);

	return *this;
}

void GoogleMapsDeviceLocator::loop() {
	switch(state) {
	case CONNECT_WAIT_STATE:
		if (Particle.connected()) {
			state = CONNECTED_WAIT_STATE;
			stateTime = millis();
		}
		break;

	case CONNECTED_WAIT_STATE:
		if (millis() - stateTime >= waitAfterConnect) {
			// Wait several seconds after connecting before doing the location
			if (locatorMode == LOCATOR_MODE_ONCE) {
				publishLocation();

				state = IDLE_STATE;
			}
			else
			if (locatorMode == LOCATOR_MODE_MANUAL) {
				state = IDLE_STATE;
			}
			else {
				state = CONNECTED_STATE;
				stateTime = millis() - periodMs;
			}
		}
		break;

	case CONNECTED_STATE:
		if (Particle.connected()) {
			if (millis() - stateTime >= periodMs) {
				stateTime = millis();
				publishLocation();
			}
		}
		else {
			// We have disconnected, rec
			state = CONNECT_WAIT_STATE;
		}
		break;


	case IDLE_STATE:
		// Just hang out here forever (entered only on LOCATOR_MODE_ONCE)
		break;
	}

}

const char *GoogleMapsDeviceLocator::scan() {
#if Wiring_WiFi
	return wifiScan();
#endif
#if Wiring_Cellular
	return cellularScan();
#endif
}


void GoogleMapsDeviceLocator::publishLocation() {

	Serial.println("publishLocation");

	const char *scanData = scan();

	Serial.printlnf("scanData=%s", scanData);

	if (scanData[0]) {

		if (Particle.connected()) {
			Particle.publish(eventName, scanData, PRIVATE);
		}
	}
}

void GoogleMapsDeviceLocator::subscriptionHandler(const char *event, const char *data) {
	// event: hook-response/deviceLocator/<deviceid>/0

	if (callback) {
		// float lat, float lon, float accuracy
		char *mutableCopy = strdup(data);
		char *part, *end;
		float lat, lon, accuracy;

		part = strtok_r(mutableCopy, ",", &end);
		if (part) {
			lat = atof(part);
			part = strtok_r(NULL, ",", &end);
			if (part) {
				lon = atof(part);
				part = strtok_r(NULL, ",", &end);
				if (part) {
					accuracy = atof(part);

					(*callback)(lat, lon, accuracy);
				}
			}
		}

		free(mutableCopy);
	}
}



#if Wiring_WiFi

static void wifiScanCallback(WiFiAccessPoint* wap, void* data) {
	// The - 3 factor here to leave room for the closing JSON array ] object }} and the trailing null
	size_t spaceLeft = &requestBuf[sizeof(requestBuf) - 3] - requestCur;
	if (spaceLeft < 30) {
		return;
	}

	int sizeNeeded = snprintf(requestCur, spaceLeft,
			"%s{\"m\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"s\":%d,\"c\":%d}",
			(requestCur[-1] == '[' ? "" : ","),
			wap->bssid[0], wap->bssid[1], wap->bssid[2], wap->bssid[3], wap->bssid[4], wap->bssid[5],
			wap->rssi, wap->channel);
	if (sizeNeeded > 0 && sizeNeeded < (int)spaceLeft) {
		// There is enough space to store the whole entry, so save it
		requestCur += sizeNeeded;
		numAdded++;
	}
}


const char *GoogleMapsDeviceLocator::wifiScan() {

	requestCur = requestBuf;
	numAdded = 0;

	requestCur += sprintf(requestCur, "{\"w\":{\"a\":");
	*requestCur++ = '[';

	WiFi.scan(wifiScanCallback);

	*requestCur++ = ']';
	*requestCur++ = '}';
	*requestCur++ = '}';
	*requestCur++ = 0;

	if (numAdded == 0) {
		requestBuf[0] = 0;
	}

	return requestBuf;
}

#endif /* Wiring_WiFi */


#if Wiring_Cellular

static void cellularAddTower(const CellularHelperEnvironmentCellData *cellData) {
	// The - 4 factor here to leave room for the closing JSON array ], object }}, and the trailing null
	size_t spaceLeft = &requestBuf[sizeof(requestBuf) - 4] - requestCur;

	int sizeNeeded = snprintf(requestCur, spaceLeft,
			"%s{\"i\":%d,\"l\":%u,\"c\":%d,\"n\":%d}",
			(requestCur[-1] == '[' ? "" : ","),
			cellData->ci, cellData->lac, cellData->mcc, cellData->mnc);

	if (sizeNeeded > 0 && sizeNeeded < (int)spaceLeft && cellData->lac != 0 && cellData->lac != 65535 && cellData->mcc != 65535 && cellData->mnc != 65535) {
		// There is enough space to store the whole entry, so save it
		requestCur += sizeNeeded;
		numAdded++;
	}

}

const char *GoogleMapsDeviceLocator::cellularScan() {

	// First try to get info on neighboring cells. This doesn't work for me using the U260
	CellularHelperEnvironmentResponseStatic<4> envResp;

	CellularHelper.getEnvironment(5, envResp);

	if (envResp.resp != RESP_OK) {
		// We couldn't get neighboring cells, so try just the receiving cell
		CellularHelper.getEnvironment(3, envResp);
	}
	// envResp.serialDebug();


	requestCur = requestBuf;
	numAdded = 0;

	// We know these things fit, so just using sprintf instead of snprintf here
	requestCur += sprintf(requestCur, "{\"c\":{\"o\":\"%s\",",
			CellularHelper.getOperatorName().c_str());

	requestCur += sprintf(requestCur, "\"a\":[");

	cellularAddTower(&envResp.service);

	for(size_t ii = 0; ii < envResp.getNumNeighbors(); ii++) {
		cellularAddTower(&envResp.neighbors[ii]);
	}

	*requestCur++ = ']';
	*requestCur++ = '}';
	*requestCur++ = '}';
	*requestCur++ = 0;

	if (numAdded == 0) {
		requestBuf[0] = 0;
	}

	return requestBuf;
}


#endif /* Wiring_Cellular */





