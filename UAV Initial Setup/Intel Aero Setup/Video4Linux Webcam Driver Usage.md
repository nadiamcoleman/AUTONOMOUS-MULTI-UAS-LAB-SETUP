## Video4Linux Webcam Driver Usage
* Show available options for webcam 
  * `v4l2-ctl --list-formats-ext`  (shows options, with associated fps)
* Set video output format
  * `v4l2-ctl --set-fmt-video=width=1920,height=1080,pixelformat=YUYV`
* Show video capture options
  * `v4l2-ctl --help-vidcap (shows video capture options)`
* Automatically configure webcam on plug-in
  * Create udev rule in `/etc/udev/rules.d/99-webcam.rules`
  * `udevadm info --attribute-walk --name /dev/video0`
  * `SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9230", PROGRAM="/usr/bin/v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=MJPG --device /dev/%k"`
    
