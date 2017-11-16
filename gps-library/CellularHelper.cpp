#include "Particle.h"

#include "CellularHelper.h"

// This check is here so it can be a library dependency for a library that's compiled for both
// cellular and Wi-Fi.
#if Wiring_Cellular

// When using system firmware 0.6.0RC1 or later, use the actual Log object, but for older
// versions just add a dummy class that does nothing so the code will compile.
#ifndef SYSTEM_VERSION_060RC1
class LogClass {
public:
	inline void info(const char *fmt, ...) {
		// This is used in the unit test, not running on an actual device
		/*
		va_list ap;
		va_start(ap, fmt);
		vprintf(fmt, ap);
		va_end(ap);
		printf("\n");
		*/
	}
};
static LogClass Log;
#endif

CellularHelperClass CellularHelper;

void CellularHelperCommonResponse::logCellularDebug(int type, const char *buf, int len) const {
	String typeStr;
	switch(type) {
	case TYPE_UNKNOWN:
		typeStr = "TYPE_UNKNOWN";
		break;

	case TYPE_OK:
		typeStr = "TYPE_OK";
		break;

	case TYPE_ERROR:
		typeStr = "TYPE_ERROR";
		break;

	case TYPE_RING:
		typeStr = "TYPE_ERROR";
		break;

	case TYPE_CONNECT:
		typeStr = "TYPE_CONNECT";
		break;

	case TYPE_NOCARRIER:
		typeStr = "TYPE_NOCARRIER";
		break;

	case TYPE_NODIALTONE:
		typeStr = "TYPE_NODIALTONE";
		break;

	case TYPE_BUSY:
		typeStr = "TYPE_NODIALTONE";
		break;

	case TYPE_NOANSWER:
		typeStr = "TYPE_NOANSWER";
		break;

	case TYPE_PROMPT:
		typeStr = "TYPE_PROMPT";
		break;

	case TYPE_PLUS:
		typeStr = "TYPE_PLUS";
		break;

	case TYPE_TEXT:
		typeStr = "TYPE_PLUS";
		break;

	case TYPE_ABORTED:
		typeStr = "TYPE_ABORTED";
		break;

	default:
		typeStr = String::format("type=0x%x", type);
		break;
	}

	Log.info("cellular response type=%s len=%d", typeStr.c_str(), len);
	String out;
	for(int ii = 0; ii < len; ii++) {
		if (buf[ii] == '\n') {
			out += "\\n";
			Log.info(out);
			out = "";
		}
		else
		if (buf[ii] == '\r') {
			out += "\\r";
		}
		else
		if (buf[ii] < ' ' || buf[ii] >= 127) {
			char hex[10];
			snprintf(hex, sizeof(hex), "0x%02x", buf[ii]);
			out.concat(hex);
		}
		else {
			out.concat(buf[ii]);
		}
	}
	if (out.length() > 0) {
		Log.info(out);
	}
}




int CellularHelperStringResponse::parse(int type, const char *buf, int len) {
	if (enableDebug) {
		logCellularDebug(type, buf, len);
	}
	if (type == TYPE_UNKNOWN) {
		CellularHelper.appendBufferToString(string, buf, len, true);
	}
	return WAIT;
}


int CellularHelperPlusStringResponse::parse(int type, const char *buf, int len) {
	if (enableDebug) {
		logCellularDebug(type, buf, len);
	}
	if (type == TYPE_PLUS) {
		// Copy to temporary string to make processing easier
		char *copy = (char *) malloc(len + 1);
		if (copy) {
			strncpy(copy, buf, len);
			copy[len] = 0;

			// We return the parts of the + response corresponding to the command we requested
			char searchFor[32];
			snprintf(searchFor, sizeof(searchFor), "\n+%s: ", command.c_str());

			//Log.info("searching for: +%s:", command.c_str());

			char *start = strstr(copy, searchFor);
			if (start) {
				start += strlen(searchFor);

				char *end = strchr(start, '\r');
				CellularHelper.appendBufferToString(string, start, end - start);
				//Log.info("found %s", string.c_str());
			}
			else {
				//Log.info("not found");
			}

			free(copy);
		}
	}
	return WAIT;
}

String CellularHelperPlusStringResponse::getDoubleQuotedPart(bool onlyFirst) const {
	String result;
	bool inQuoted = false;

	result.reserve(string.length());

	for(size_t ii = 0; ii < string.length(); ii++) {
		char ch = string.charAt(ii);
		if (ch == '"') {
			inQuoted = !inQuoted;
			if (!inQuoted && onlyFirst) {
				break;
			}
		}
		else {
			if (inQuoted) {
				result.concat(ch);
			}
		}
	}

	return result;
}


void CellularHelperRSSIQualResponse::postProcess() {
	if (sscanf(string.c_str(), "%d,%d", &rssi, &qual) == 2) {

		// The range is the following:
		// 0: -113 dBm or less
		// 1: -111 dBm
		// 2..30: from -109 to -53 dBm with 2 dBm steps
		// 31: -51 dBm or greater
		// 99: not known or not detectable or currently not available
		if (rssi < 99) {
			rssi = -113 + (rssi * 2);
		}
		else {
			rssi = 0;
		}

		resp = RESP_OK;
	}
	else {
		// Failed to parse result
		resp = RESP_ERROR;
	}
}


CellularHelperEnvironmentResponse::CellularHelperEnvironmentResponse(CellularHelperEnvironmentCellData *neighbors, size_t numNeighbors) :
	neighbors(neighbors), numNeighbors(numNeighbors) {


}

int CellularHelperEnvironmentResponse::parse(int type, const char *buf, int len) {
	if (enableDebug) {
		logCellularDebug(type, buf, len);
	}

	if (type == TYPE_UNKNOWN || type == TYPE_PLUS) {
		// We get this for AT+CGED=5
		// Copy to temporary string to make processing easier
		char *copy = (char *) malloc(len + 1);
		if (copy) {
			strncpy(copy, buf, len);
			copy[len] = 0;

			// This is used for skipping over the +CGED: part of the response
			char searchFor[32];
			size_t searchForLen = snprintf(searchFor, sizeof(searchFor), "+%s: ", command.c_str());

			char *endStr;

			char *line = strtok_r(copy, "\r\n", &endStr);
			while(line) {
				if (line[0]) {
					// Not an empty line

					if (type == TYPE_PLUS && strncmp(line, searchFor, searchForLen) == 0) {
						line += searchForLen;
					}

					if (strncmp(line, "MCC:", 4) == 0) {
						// Line begins with MCC:
						// This happens for 2G and 3G
						if (curDataIndex < 0) {
							service.parse(line);
							curDataIndex++;
						}
						else
						if (neighbors && (size_t)curDataIndex < numNeighbors) {
							neighbors[curDataIndex++].parse(line);
						}
					}
					else
					if (strncmp(line, "RAT:", 4) == 0) {
						// Line begins with RAT:
						// This happens for 3G in the + response so you know whether
						// the response is for a 2G or 3G tower
						service.parse(line);
					}
				}
				line = strtok_r(NULL, "\r\n", &endStr);
			}

			free(copy);
		}
	}
	return WAIT;
}

void CellularHelperEnvironmentCellData::parse(const char *str) {
	char *mutableCopy = strdup(str);

	char *endStr;

	char *pair = strtok_r(mutableCopy, ",", &endStr);
	while(pair) {
		// Remove leading spaces caused by ", " combination
		while(*pair == ' ') {
			pair++;
		}

		char *colon = strchr(pair, ':');
		if (colon != NULL) {
			*colon = 0;
			const char *key = pair;
			const char *value = ++colon;

			addKeyValue(key, value);
		}

		pair = strtok_r(NULL, ",", &endStr);
	}


	free(mutableCopy);
}

bool CellularHelperEnvironmentCellData::isValid(bool ignoreCI) const {

	if (mcc > 999) {
		return false;
	}

	if (!ignoreCI) {
		if (isUMTS) {
			if (ci >= 0xfffffff) {
				return false;
			}
		}
		else {
			if (ci >= 0xffff) {
				return false;
			}
		}
	}
	return true;
}


void CellularHelperEnvironmentCellData::addKeyValue(const char *key, const char *value) {
	if (strcmp(key, "RAT") == 0) {
		isUMTS = (strstr(value, "UMTS") != NULL);
	}
	else
	if (strcmp(key, "MCC") == 0) {
		mcc = atoi(value);
	}
	else
	if (strcmp(key, "MNC") == 0) {
		mnc = atoi(value);
	}
	else
	if (strcmp(key, "LAC") == 0) {
		lac = (int) strtol(value, NULL, 16); // hex
	}
	else
	if (strcmp(key, "CI") == 0) {
		ci = (int) strtol(value, NULL, 16); // hex
	}
	else
	if (strcmp(key, "BSIC") == 0) {
		bsic = (int) strtol(value, NULL, 16); // hex
	}
	else
	if (strcmp(key, "Arfcn") == 0) {
		// Documentation says this is hex, but this does not appear to be the case!
		// arfcn = (int) strtol(value, NULL, 16); // hex
		arfcn = atoi(value);
	}
	else
	if (strcmp(key, "Arfcn_ded") == 0 || strcmp(key, "RxLevSub") == 0 || strcmp(key, "t_adv") == 0) {
		// Ignored 2G fields
	}
	else
	if (strcmp(key, "RxLev") == 0 || strcmp(key, "RXLEV") == 0) {
		rxlev = (int) strtol(value, NULL, 16); // hex
	}
	else
	if (strcmp(key, "DLF") == 0) {
		dlf = atoi(value);
	}
	else
	if (strcmp(key, "ULF") == 0) {
		ulf = atoi(value);

		// For AT+COPS=5, we don't get a RAT, but if ULF is present it's 3G
		isUMTS = true;
	}
	else
	if (strcmp(key, "RSCP LEV") == 0) {
		rscpLev = atoi(value);
	}
	else
	if (strcmp(key, "RAC") == 0 || strcmp(key, "SC") == 0 || strcmp(key, "ECN0 LEV") == 0) {
		// We get these with AT+COPS=5, but we don't need the values
	}
	else {
		Log.info("unknown key=%s value=%s", key, value);
	}

}

int CellularHelperEnvironmentCellData::getBand() const {
	int freq = 0;

	if (isUMTS) {
		// 3G radio
		if (ulf >= 0 && ulf <= 124) {
			freq = 900;
		}
		else
		if (ulf >= 128 && ulf <= 251) {
			freq = 850;
		}
		else
		if (ulf >= 512 && ulf <= 885) {
			freq = 1800;
		}
		else
		if (ulf >= 975 && ulf <= 1023) {
			freq = 900;
		}
		else
		if (ulf >= 1312 && ulf <= 1513) {
			freq = 1700;
		}
		else
		if (ulf >= 2712 && ulf <= 2863) {
			freq = 900;
		}
		else
		if (ulf >= 4132 && ulf <= 4233) {
			freq = 850;
		}
		else
		if ((ulf >= 4162 && ulf <= 4188) || (ulf >= 20312 && ulf <= 20363)) {
			freq = 800;
		}
		else
		if (ulf >= 9262 && ulf <= 9538) {
			freq = 1900;
		}
		else
		if (ulf >= 9612 && ulf <= 9888) {
			freq = 2100;
		}
	}
	else {
		// 2G, use arfcn
		if (arfcn >= 0 && arfcn <= 124) {
			freq = 900;
		}
		else
		if (arfcn >= 128 && arfcn <= 251) {
			freq = 850;
		}
		else
		if (arfcn >= 512 && arfcn <= 885) {
			freq = 1800;
		}
		else
		if (arfcn >= 975 && arfcn <= 1023) {
			freq = 900;
		}
	}
	return freq;
}

// Calculated
String CellularHelperEnvironmentCellData::getBandString() const {
	String band;

	int freq = getBand();

	if (isUMTS) {
		// 3G radio
		if ((ulf >= 0 && ulf <= 124) ||
			(ulf >= 128 && ulf <= 251)) {
			band = "GSM " + String(freq);
		}
		else
		if (ulf >= 512 && ulf <= 885) {
			band = "DCS 1800";
		}
		else
		if (ulf >= 975 && ulf <= 1023) {
			band = "ESGM 900";
		}
		else
		if (freq != 0) {
			band = "UMTS " + String(freq);
		}
		else {
			band = "3G unknown";
		}
	}
	else {
		// 2G, use arfcn

		if (arfcn >= 512 && arfcn <= 885) {
			band = "DCS 1800 or 1900";
		}
		else
		if (arfcn >= 975 && arfcn <= 1024) {
			band = "EGSM 900";
		}
		else
		if (freq != 0) {
			band = "GSM " + String(freq);
		}
		else {
			band = "2G unknown";
		}
	}


	return band;
}

int CellularHelperEnvironmentCellData::getRSSI() const {
	int rssi = 0;

	if (isUMTS) {
		// 3G radio

		if (rscpLev <= 96) {
			rssi = -121 + rscpLev;
		}
	}
	else {
		// 2G
		if (rxlev <= 96) {
			rssi = -121 + rxlev;
		}
	}
	return rssi;
}

int CellularHelperEnvironmentCellData::getBars() const {
	return CellularHelperClass::rssiToBars(getRSSI());
}

String CellularHelperEnvironmentCellData::toString() const {
	String common = String::format("mcc=%d, mnc=%d, lac=%x ci=%x band=%s rssi=%d",
			mcc, mnc, lac, ci, getBandString().c_str(), getRSSI());

	if (isUMTS) {
		return String::format("rat=UMTS %s dlf=%d ulf=%d", common.c_str(), dlf, ulf);
	}
	else {
		return String::format("rat=GSM %s bsic=%x arfcn=%d rxlev=%d", common.c_str(), bsic, arfcn, rxlev);
	}
}

void CellularHelperEnvironmentResponse::clear() {
	curDataIndex = -1;
}


void CellularHelperEnvironmentResponse::postProcess() {
}


void CellularHelperEnvironmentResponse::logResponse() const {
	Log.info("service %s", service.toString().c_str());
	if (neighbors) {
		for(size_t ii = 0; ii < numNeighbors; ii++) {
			if (neighbors[ii].isValid(true /* ignoreCI */)) {
				Log.info("neighbor %d %s", ii, neighbors[ii].toString().c_str());
			}
		}
	}
}

size_t CellularHelperEnvironmentResponse::getNumNeighbors() const {
	if (curDataIndex < 0) {
		return 0;
	}
	else {
		if (neighbors) {
			for(size_t ii = 0; ii < (size_t)curDataIndex; ii++) {
				if (!neighbors[ii].isValid()) {
					return ii;
				}
			}
		}
		return curDataIndex;
	}
}



// +UULOC: <date>,<time>,<lat>,<long>,<alt>,<uncertainty>

void CellularHelperLocationResponse::postProcess() {
	char *mutableCopy = strdup(string.c_str());
	if (mutableCopy) {
		char *part, *endStr;

		part = strtok_r(mutableCopy, ",", &endStr);
		if (part) {
			// part is date
			part = strtok_r(NULL, ",", &endStr);
			if (part) {
				// part is time
				part = strtok_r(NULL, ",", &endStr);
				if (part) {
					// part is lat
					lat = atof(part);

					part = strtok_r(NULL, ",", &endStr);
					if (part) {
						// part is lon
						lon = atof(part);

						part = strtok_r(NULL, ",", &endStr);
						if (part) {
							// part is alt
							alt = atoi(part);

							part = strtok_r(NULL, ",", &endStr);
							if (part) {
								// part is uncertainty
								uncertainty = atoi(part);
								valid = true;
								resp = RESP_OK;
							}
						}
					}
				}
			}
		}

		free(mutableCopy);
	}
}

String CellularHelperLocationResponse::toString() const {
	if (valid) {
		return String::format("lat=%f lon=%f alt=%d uncertainty=%d", lat, lon, alt, uncertainty);
	}
	else {
		return "valid=false";
	}
}

String CellularHelperClass::getManufacturer() const {
	CellularHelperStringResponse resp;

	Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CGMI\r\n");

	return resp.string;
}

String CellularHelperClass::getModel() const {
	CellularHelperStringResponse resp;

	Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CGMM\r\n");

	return resp.string;
}

String CellularHelperClass::getOrderingCode() const {
	CellularHelperStringResponse resp;

	Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "ATI0\r\n");

	return resp.string;
}

String CellularHelperClass::getFirmwareVersion() const {
	CellularHelperStringResponse resp;

	Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CGMR\r\n");

	return resp.string;
}

String CellularHelperClass::getIMEI() const {
	CellularHelperStringResponse resp;

	Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CGSN\r\n");

	return resp.string;
}

String CellularHelperClass::getIMSI() const {
	CellularHelperStringResponse resp;

	Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CGMI\r\n");

	return resp.string;
}

String CellularHelperClass::getICCID() const {
	CellularHelperPlusStringResponse resp;
	resp.command = "CCID";

	Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CCID\r\n");

	return resp.string;
}

String CellularHelperClass::getOperatorName(int operatorNameType) const {
	String result;

	// The default is OPERATOR_NAME_LONG_EONS (9).
	// If the EONS name is not available, then the other things tried in order are:
	// NITZ, CPHS, ROM
	// So basically, something will be returned

	CellularHelperPlusStringResponse resp;
	resp.command = "UDOPN";

	int respCode = Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+UDOPN=%d\r\n", operatorNameType);

	if (respCode == RESP_OK) {
		result = resp.getDoubleQuotedPart();
	}

	return result;
}

/**
 * Get the RSSI and qual values for the receiving cell site.
 *
 * The qual value is always 99 for me on the G350 (2G).
 */
CellularHelperRSSIQualResponse CellularHelperClass::getRSSIQual() const {
	CellularHelperRSSIQualResponse resp;
	resp.command = "CSQ";

	resp.resp = Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CSQ\r\n");

	if (resp.resp == RESP_OK) {
		resp.postProcess();
	}

	return resp;
}

void CellularHelperClass::getEnvironment(int mode, CellularHelperEnvironmentResponse &resp) const {
	resp.command = "CGED";
	// resp.enableDebug = true;

	resp.resp = Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+CGED=%d\r\n", mode);
	if (resp.resp == RESP_OK) {
		resp.postProcess();
	}
}

CellularHelperLocationResponse CellularHelperClass::getLocation(unsigned long timeoutMs) const {
	CellularHelperLocationResponse resp;

	// Note: Command is ULOC, but the response is UULOC
	resp.command = "UULOC";
	// resp.enableDebug = true;

	// Initialize the mode
	resp.resp = Cellular.command(5000, "AT+ULOCCELL=0\r\n");
	if (resp.resp == RESP_OK) {
		unsigned long startTime = millis();

		resp.resp = Cellular.command(responseCallback, (void *)&resp, timeoutMs, "AT+ULOC=2,2,0,%d,5000\r\n", timeoutMs / 1000);

		// This command is weird because it returns an OK, and theoretically could return +UULOC response right away,
		// but usually does not.
		if (resp.resp == RESP_OK) {
			resp.postProcess();

			// In the case where we don't get an immediate response, we send empty commands to the
			// modem to pick up the late +UULOC response
			while(!resp.valid && millis() - startTime < timeoutMs) {
				// Allow for some cloud processing before checking again
				delay(10);

				// Have not received a response yet. Send an empty command so we can get responses that
				// come afte the OK due to the weird structure of this command
				Cellular.command(responseCallback, (void *)&resp, 500, "");
				resp.postProcess();
			}
		}
	}

	return resp;
}


bool CellularHelperClass::ping(const char *addr) const {
	CellularHelperStringResponse resp;

	resp.resp = Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+UPING=\"%s\"\r\n", addr);

	return resp.resp == RESP_OK;
}

IPAddress CellularHelperClass::dnsLookup(const char *hostname) const {
	IPAddress result;

	CellularHelperPlusStringResponse resp;
	resp.command = "UDNSRN";

	resp.resp = Cellular.command(responseCallback, (void *)&resp, DEFAULT_TIMEOUT, "AT+UDNSRN=0,\"%s\"\r\n", hostname);
	if (resp.resp == RESP_OK) {
		String quotedPart = resp.getDoubleQuotedPart();
		int addr[4];
		if (sscanf(quotedPart.c_str(), "%u.%u.%u.%u", &addr[0], &addr[1], &addr[2], &addr[3]) == 4) {
			result = IPAddress(addr[0], addr[1], addr[2], addr[3]);
		}
	}

	return result;
}




// There isn't an overload of String that takes a buffer and length, but that's what comes back from
// the Cellular.command callback, so that's why this method exists.
void CellularHelperClass::appendBufferToString(String &str, const char *buf, int len, bool noEOL) const {
	str.reserve(str.length() + (size_t)len + 1);
	for(int ii = 0; ii < len; ii++) {
		if (!noEOL || (buf[ii] != '\r' && buf[ii] != '\n')) {
			str.concat(buf[ii]);
		}
	}
}

// static
int CellularHelperClass::rssiToBars(int rssi) {
	int bars = 0;

	if (rssi < 0) {
		if (rssi >= -57)      bars = 5;
		else if (rssi > -68)  bars = 4;
		else if (rssi > -80)  bars = 3;
		else if (rssi > -92)  bars = 2;
		else if (rssi > -104) bars = 1;
	}
	return bars;
}

// static
int CellularHelperClass::responseCallback(int type, const char* buf, int len, void *param) {
	CellularHelperCommonResponse *presp = (CellularHelperCommonResponse *)param;

	return presp->parse(type, buf, len);
}

#endif /* Wiring_Cellular */


