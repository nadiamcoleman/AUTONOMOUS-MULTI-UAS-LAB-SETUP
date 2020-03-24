## Calibrate Flight Controller
  * [Reference](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-Setup#calibration)  
  * Click Gear Icon to enter vehicle setup
    * Configure Airframe (if summary doesn't already show Intel Aero RTF Drone)
      * If Intel Aero RTF Drone is not listed under the Quadrotor X group...
        * Uninstall QGroundcontrol and remove Airframe meta cache... see [here](https://github.com/mavlink/qgroundcontrol/issues/6794)
        * Delete the PX4AirframeFactMetaData.xml in your ~/.config/QGroundControl.org directory (Ubuntu or Mac)
    * Calibrate Sensors
      * Calibrate Compass Gyro \& Accelerometer following QGC prompts
    * Power Setup (Battery)
      * Number of cells: `4`
      * Voltage divider: `9.12139702`
        * Click `Calculate`, Type in measured battery voltage (use multimeter) and click `Calculate`
      * Amps per volt: `36.36751556`???
