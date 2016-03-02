ephemeris
===

An example that sets the time on the GPS module using the Particle cloud clock,
 and then proceeds to load Satellite ephemeris data from NASA onto the module! 
  This demo should have a separate ephemeris data download app, and save that data into firmware.  
  I believe this data can be valid for as long as 14 days, so it should give you a good heads up.  
  Future demos should be able to transmit this via the cloud, but doing it locally now proves the concept, 
  and saves bandwidth.  :) 
  
  

ephemeris data for GPS
    http://cddis.nasa.gov/Data_and_Derived_Products/GNSS/broadcast_ephemeris_data.html#codeTable
    http://cddis.nasa.gov/Data_and_Derived_Products/GNSS/Real-time_products.html
    
    ftp://cddis.gsfc.nasa.gov/gnss/00readme
    ftp://cddis.gsfc.nasa.gov/gnss/data/satellite/
    ftp://cddis.gsfc.nasa.gov/gnss/data/daily/2016/056/16n/
    ftp://cddis.gsfc.nasa.gov/gnss/data/hourly/2016/056/19/


almanac data for GPS
    http://www.navcen.uscg.gov/?pageName=gpsAlmanacs
    
    
    http://www.gdgps.net/products/broadcast-ephemeris.html


PDFs on NMEA / GPS

    http://www.mouser.com/ds/2/737/adafruit-ultimate-gps-779243.pdf
    
    https://www.adafruit.com/datasheets/PMTK_A08.pdf
    https://www.adafruit.com/datasheets/PMTK_A11.pdf
    https://www.adafruit.com/datasheets/GTop%20EPO%20Format%20and%20Protocol-v14.pdf
    https://www.adafruit.com/datasheets/GlobalTop-FGPMMOPA6H-Datasheet-V0A.pdf
    https://www.adafruit.com/datasheets/PA6B-Datasheet-A07.pdf
    
    
    https://www2.u-blox.com/images/downloads/Product_Docs/NMEA-CommandManual_(FTX-HW-13002).pdf
    ftp://77.234.201.131/Support/SimCom/SIM68/SIM28@SIM68R@SIM68V_NMEA%20Messages%20Specification_V1.00.pdf
    http://www.telit.com/fileadmin/user_upload/Telit_SGEE-EPO_Application_Note_r2.pdf


NMEA checksum calculator - 
    http://www.hhhh.org/wiml/proj/nmeaxor.html


other projects in this space
    http://arduino-projects4u.com/gps/
    https://code.google.com/archive/p/trackuino/


best example / available source code for loading EPO data - lets do this in firmware (and maybe node)
(will not accept anything that does not give proper attribution / licensing to f5eng and others)

    https://github.com/f5eng/mt3339-utils

