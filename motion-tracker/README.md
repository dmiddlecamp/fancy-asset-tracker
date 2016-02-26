motion-tracker
===

Super awesome motion tracker demo!

This app does some fancy things.  First of all, it sets the accelerometer to wake up the electron if the device moves
in the 1-2G range.  If it wakes up and then senses motion again from the accelerometer, it will connect to the cloud
and start reporting its GPS fix every minute until motion stops for more than 3 minutes, and then it will go back to
sleep again.

If the device doesn't sense motion, it will wakeup anyway every 6 hours and report in, in case it didn't have a good
fix before.