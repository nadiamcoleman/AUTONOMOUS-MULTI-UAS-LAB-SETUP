## Accessing Camera Feeds
  * Reference [06 Cameras and Video](https://github.com/intel-aero/meta-intel-aero/wiki/06-Cameras-and-Video)
  * The camera data is streaming through the camera streaming daemon ([csd](https://github.com/intel-aero/meta-intel-aero/wiki/90-(References)-OS-user-Installation#intel-camera-streaming-daemon))
    * Use `sudo systemctl stop/start/restart csd` to control the daemon
    * NOTE: This is separate from the `systemctl stop/start/restart mavlink-router` which streams flight data
      * The UDP endpoint for this can be configured in `/etc/mavlink-router/config.d/qgc.conf`
    * Troubleshooting:
    * "UVCIOC_CTRL_QUERY:UVC_GET_CUR error 5"
      * This is likely because some other process is using the camera(s)
        * e.g. `sudo systemctl stop csd` will stop camera streaming daemon
  * Save camera setting defaults to file:
    *
    ```
    sudo v4l2-ctl --list-devices
    mkdir ~/R200_settings/
    v4l2-ctl --list-ctrls -d /dev/video13 > ~/R200_settings/video13.txt
    ```
    
  * Recording Video (based on [these instructions](https://github.com/intel-aero/meta-intel-aero/wiki/06-Cameras-and-Video#record-video))
    * Create bash function for easy access as "aero_record":
    ```
    # Record R200 video based on GitHub Tutorials for Aero
    # USAGE: 'aero_record <device> <width> <height> <output_filename>'
    #        e.g. 'aero_record /dev/video13 1920 1080 encoded_video'
    function aero_record() {
        sudo gst-launch-1.0 -e v4l2src device=$1 num-buffers=2000 ! autovideoconvert format=i420 width=$2 height=$3 framerate=30/1 ! vaapih264enc rate-control=cbr tune=high-compression ! qtmux ! filesink location=$4.mp4
        ffmpeg -i $4.mp4
    }
    ```
