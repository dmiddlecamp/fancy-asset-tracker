
//PACK(
    struct UbloxHeader {
        uint8_t sync1;   //!< start of packet first byte (0xB5)
        uint8_t sync2;   //!< start of packet second byte (0x62)
        uint8_t message_class; //!< Class that defines basic subset of message (NAV, RXM, etc.)
        uint8_t message_id;		//!< Message ID
        uint16_t payload_length; //!< length of the payload data, excluding header and checksum
};
//);


//////////////////////////////////////////////////////////////
// AIDING DATA MESSAGES
//////////////////////////////////////////////////////////////
/*!
 * AID-INI Message Structure
 * Reciever Position, Time, Clock Drift, Frequency
 * ID: 0x0B  0x01 Payload Length=48 bytes
 */
#define PAYLOAD_LENGTH_AID_INI 48
#define FULL_LENGTH_AID_INI 48+8
//PACK(
    struct AidIni {
        UbloxHeader header;		//!< Ublox header
        int32_t ecefXorLat;  //!< ECEF x position or latitude [cm or deg*1e-7]
        int32_t ecefYorLon;  //!< ECEF y position or longitude [cm or deg*1e-7]
        int32_t ecefZorAlt;  //!< ECEF z position or altitude [cm]
        uint32_t position_accuracy; //!< position accuracy - std dev [cm]
        uint16_t time_configuration; //!< time configuration bit misk
        uint16_t week_number; //!< actual week number
        uint32_t time_of_week; //!< actual time of week [ms]
        int32_t time_of_week_ns; //!< fractional part of time of week [ns]
        uint32_t time_accuracy_ms; //!< time accuracy [ms]
        uint32_t time_accuracy_ns; //!< time accuracy [ns]
        int32_t clock_drift_or_freq; //!< clock drift or frequency [ns/s or Hz*1e-2]
        uint32_t clock_drift_or_freq_accuracy; //!< clock drift or frequency accuracy [ns/s or ppb]
        uint32_t flags; //!< bit field that determines contents of other fields
        uint8_t checksum[2];
};
//);


// defines for AidIni flags
#define AIDINI_FLAG_POSITION_VALID 0x01
#define AIDINI_FLAG_TIME_VALID 0x02
#define AIDINI_FLAG_CLOCK_DRIFT_VALID 0x04
#define AIDINI_FLAG_USE_TIME_PULSE 0X08
#define AIDINI_FLAG_CLOCK_FREQ_VALID 0x10
#define AIDINI_FLAG_USE_LLA 0x20
#define AIDINI_FLAG_ALTITUDE_INVALID 0X40
#define AIDINI_USE_PREV_TIME_PULSE 0X80

bool SendMessage(uint8_t* msg_ptr, size_t length)
{

    for(int i=0;i<length;i++) {
        uint8_t c = msg_ptr[i];
        Serial1.write(c);
        delay(1);
    }



//	try {
//		stringstream output1;
//		//std::cout << length << std::endl;
//		//std::cout << "Message Pointer" << endl;
//		//printHex((char*) msg_ptr, length);
//        size_t bytes_written;
//
//        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
//		  bytes_written=serial_port_->write(msg_ptr, length);
//        } else {
//            log_error_("Unable to send message. Serial port not open.");
//            return false;
//        }
//
//		// check that full message was sent to serial port
//		if (bytes_written == length) {
//			return true;
//		}
//		else {
//			log_error_("Full message was not sent over serial port.");
//			output1 << "Attempted to send " << length << "bytes. " << bytes_written << " bytes sent.";
//			log_error_(output1.str());
//			return false;
//		}
//	} catch (std::exception &e) {
//		std::stringstream output;
//		output << "Error sending ublox message: " << e.what();
//		log_error_(output.str());
//		return false;
//	}
    return true;
}


void calculateCheckSum(uint8_t* in, size_t length, uint8_t* out) {

    uint8_t a = 0;
    uint8_t b = 0;

    for (uint8_t i = 0; i < length; i++) {
        a = a + in[i];
        b = b + a;
    }

    out[0] = (a & 0xFF);
    out[1] = (b & 0xFF);
}

bool sendGPSAidIni() {
    AidIni cur_aid_ini;

    cur_aid_ini.flags = cur_aid_ini.flags & 0xF7; // clear time pulse flag
    cur_aid_ini.flags = cur_aid_ini.flags & 0xFB; // clear clock drift flag
    cur_aid_ini.flags = cur_aid_ini.flags & 0xFE; // clear clock freq flag

    cur_aid_ini.flags = cur_aid_ini.flags | 1024; // set the 10th bit ON // enable UTC TIME MODE

    cur_aid_ini.clock_drift_or_freq=0;
    cur_aid_ini.time_accuracy_ms=1000;
    //cur_aid_ini.time_of_week=cur_aid_ini.time_of_week + time_correction;


//    boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
//    boost::posix_time::time_duration duration(present_time.time_of_day());
//    return duration.total_seconds();
//

    // january 1st, 1980 0:0:0      -- https://www.epochconverter.com/
    unsigned long epoch = 315532800;
    unsigned long now = Time.now();
    long elapsedSeconds = now - epoch;
    int weekAsSeconds = 7 * 24 * 60 * 60;
    int elapsedWeeks = elapsedSeconds / weekAsSeconds;
    int timeOfWeekSeconds = elapsedSeconds % weekAsSeconds;

    //week number is an integer starting with the first full week in January 1980
    //time_of_week is seconds

    cur_aid_ini.week_number = elapsedWeeks;         // actual week number
    cur_aid_ini.time_of_week = timeOfWeekSeconds;   //!< actual time of week [ms]

    Serial.println(String::format("Suggesting week %d, plus %d seconds ", elapsedWeeks, timeOfWeekSeconds));


    unsigned char* msg_ptr = (unsigned char*)&cur_aid_ini;
    calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_INI + 4,
                        cur_aid_ini.checksum);

    return SendMessage(msg_ptr, FULL_LENGTH_AID_INI);
}