
ISSUES
     
* we can now suggest a starting location to the GPS
 
    * express this as a PR, store last fix in the flash?  restore on load?
* serial polling is lossy, we should be polling complete phrases at least
 
    * express this as a PR

* Google wants an example that sleeps when accel isn't going, wakes on movement, and then publishes until unit 
 stops moving.
 
    * express this as a generic example / PR when done?
   


* GPS RTC battery doesn't seem to work
    * datasheet shows 1u cap needed on vbackup line to gnd
  
    * fixing this will make a big difference.
    
* can't load / acquire almanac / satellite EPO / ephremetis data
    * pre-loading this would be cool.
    
    
* we can now suggest a clock time to the GPS
    * express this as a PR, make sure it only does it when GPS clock isn't set, and
        good server time is available
        
    
* GPS might get stuck in a weird state?
    * should we hot-restart every 10 minutes or something?
    
* cell modem might interfere with GPS
    * can we try getting a fix with the radio off?  before powering up the radio and reporting in?
    
    

