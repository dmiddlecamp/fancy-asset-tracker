fast time to first fix
---

This example is meant to use a combination of cellular location and stored previous location, along with suggested time
to get the fastest fix time possible

The PMTK741 command isn't working on the ublox module like I would have expected.

Instead I found the AID-INI command described in the datasheet ( u-blox7-V14_ReceiverDescrProtSpec_(GPS.G7-SW-12001)_Public.pdf )
https://github.com/spark/shields/blob/master/electron-shields/asset-tracker/datasheet/u-blox7-V14_ReceiverDescrProtSpec_(GPS.G7-SW-12001)_Public.pdf


15.5 Aiding Sequence
A typical aiding sequence comprises the following steps:
• Power-up the GNSS receiver
• Send UBX-AID-INI (time, clock and position) message.
• Send UBX-AID-EPH (ephemeris) message.
• Apply optional hardware time synchronization pulse within 0.5 s after (or before, depending on the configuration in UBX-AID-INI) sending the UBX-AID-INI message if hardware time synchronization is required. When sending the message before applying the pulse, make sure to allow the GNSS receiver to parse and process the aiding message. The time for parsing depends on the baud rate. The processing time is 100 ms maximum.
• Send optional UBX-AID-HUI (health, UTC and ionosphere parameters) message.
• Send optional UBX-AID-ALM (almanac) message.



https://github.com/GAVLab/ublox/blob/master/examples/assist_example.cpp
https://github.com/GAVLab/ublox/blob/master/src/ublox.cpp
https://github.com/GAVLab/ublox/blob/master/include/ublox/ublox_structures.h

11.2.2.5 Do not enter 'inactive for search' state when no fix
If this option is enabled, the receiver acts differently in case it can't get a fix: instead of entering Inactive for
search state, it keeps trying to acquire a fix. In other words, the receiver will never be in Inactive for search state
and therefore the search period and the acquisition timeout are obsolete

