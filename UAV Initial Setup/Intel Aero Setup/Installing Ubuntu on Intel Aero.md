## Install \& Set Up Ubuntu
1. Install Ubuntu
  * Make bootable USB with Ubuntu 16.04.3
  * Boot from USB, and click `Install Ubuntu`
  * Do NOT check `Install third-party software for graphics...`
  * Select `Erase disk and install Ubuntu`
    * When done a pop-up will say: `Installation is complete... Restart Now`
    * Click `Restart Now`... may take long time, ok to do hard reboot
* Install Intel Aero repository following instructions
* Set up WiFi
  * Click Ubuntu icon in top left of dock to search computer
  * Search for Network and select `Network` (orange folder with plug on it)
  * Disable hotspot and connect to LabRouter5
  * Set up QGC config as described [here](https://github.com/intel-aero/meta-intel-aero/wiki/90-(References)-OS-user-Installation)
* Install Intel RealSense SDK based on [this](https://github.com/intel-aero/meta-intel-aero/wiki/90-(References)-OS-user-Installation#intel-realsense-sdk) with some modifications
    * 
    ```
    cd ~
    sudo apt-get -y install git libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev cmake
    git clone -b legacy --single-branch https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    mkdir build && cd build
    cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
    make -j4
    sudo make install
    ```
  * Update the **PYTHONPATH** env. variable (add path to pyrealsense lib)
    * Add the following line to **~/.bashrc**
      * `export PYTHONPATH=$PYTHONPATH:/usr/local/lib`
      
  * Install PyRealsense
    * `sudo pip install pycparser`
    * `sudo pip install cython`
    * `sudo pip install pyrealsense`
