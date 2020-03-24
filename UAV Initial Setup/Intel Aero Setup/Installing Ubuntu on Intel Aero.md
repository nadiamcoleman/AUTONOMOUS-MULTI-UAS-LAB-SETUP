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
    
    
    
## Set up Basic Applications/Environment
1. Enable root login
  * `sudo passwd root`
  * Enter new UNIX password: `intel2`
* Add user to dialout group
  * `sudo usermod -a -G dialout intel2`
  * `sudo reboot`
  
* Install basic applications
  * 
  ```
  sudo apt update
  sudo apt install git cmake kate chrony screen unzip terminator v4l-utils htop curl vlc chromium-browser
  ```
* Configure **Chrony**
  * This application will synchronize the clocks between multiple machines on the same network. This is necessary since the Time class in ROS uses the system clock for timing, and the system clock may differ between two machines, even if they are both using the same ntp server. Thus making it impossible to accurately compare timestamps between topics sent from different machines.
  * References: [1](https://chrony.tuxfamily.org/documentation.html), [2](https://wiki.archlinux.org/index.php/Chrony), [3](http://cmumrsdproject.wikispaces.com/file/view/ROS_MultipleMachines_Wiki_namank_fi_dshroff.pdf), [4](http://wiki.ros.org/ROS/NetworkSetup), [5](https://github.com/pr2-debs/pr2/blob/master/pr2-chrony/root/unionfs/overlay/etc/chrony/chrony.conf), [6](https://www.digitalocean.com/community/tutorials/how-to-set-up-time-synchronization-on-ubuntu-16-04)
  * To check the time difference between machines, use:
    * `ntpdate -q <IP address of other machine>` 
  * Turn off **timesyncd**
    * `sudo timedatectl set-ntp no` 
  * Configure Chrony (make these changes in `/etc/chrony/chrony.conf`)
    * Add Purdue’s ntp servers
      * ```
      sudo sed -i 's/pool 2.debian.pool.ntp.org offline iburst$/pool 2.debian.pool.ntp.org offline iburst\nserver ntp3.itap.purdue.edu offline iburst\nserver ntp4.itap.purdue.edu offline iburst/' /etc/chrony/chrony.conf
      ```
  * Set up chrony server for other devices
    * ```
    sudo sed -i 's/#local stratum 10$/local stratum 10/' /etc/chrony/chrony.conf
    sudo sed -i 's~#allow ::/0 (allow access by any IPv6 node)$~#allow ::/0 (allow access by any IPv6 node)\nallow 192.168~' /etc/chrony/chrony.conf
    ```
  * Allow for step time changes at startup (for initial time setup)
    * ```
    su
    echo -e “# In first three updates step the system clock instead of slew\n# if the adjustment is larger than 10 seconds\nmakestep 10 3” | tee -a /etc/chrony/chrony.conf > /dev/null
    exit
    ```
  * Set timezone
    * Get the exact name of the desired timezone (`America/Indiana/Indianapolis`)
      * `timedatectl list-timezones`
    * `sudo timedatectl set-timezone America/Indiana/Indianapolis`
    * `date`
  * Restart chrony to apply changes
    * `sudo systemctl restart chrony`
    
* Configure **Chromium Browser**
  * From Chrome Browser, search for `Markdown Viewer Chrome` [link](https://chrome.google.com/webstore/detail/markdown-viewer/ckkdlimhmcjmikdlpkmbgfkaikojcbjk?hl=en)
  * Click `Add to Chrome`, Confirm in popup window: `Add extension`
  * Allow access to file:/// URLs
    * Click Markdown Viewer icon in chromium menu bar ('m' icon)
    * Click `Advanced Options`
    * Click `ALLOW ACCESS TO FILE:///URLS` which opens the extension options
    * Scroll down and toggle `Allow access to file URLs` to ON position
    * Open a markdown file (.m extension) and click Markdown Viewer icon to select desired Theme
      * e.g. GitHub, Markdown9, etc.
* Configure **Kate**
  * Top Menu --> Settings --> Configure Kate
    * Application
      * Sessions --> Check `Load last used session`
    * Editor Component
      * Appearance --> Borders --> Check `Show line numbers`
      * Fonts & Colors --> Font --> `Size: 10`
      * Editing --> General --> Check `Show static word wrap marker`
      * Editing --> General --> Check `Enable automatic brackets`
      * Editing --> Indentation --> `Tab width: 4 characters`
      * Editing --> Indentation --> `Keep extra spaces`
      * Open/Save --> Modes & Filetypes
        * Select Filetype: `Markup/XML`
        * Add `;*.launch` to 'File extensions'
          * NOTE: Be careful not to add a space after the `;`
  * Click `Apply` then `OK`
  * Set Kate as default file editor (instead of GEdit)
    * ```
    sudo sed -i 's/gedit.desktop/kate.desktop/g' /etc/gnome/defaults.list
    sudo cp /usr/share/applications/gedit.desktop /usr/share/applications/kate.desktop
    sudo sed -i 's/gedit/kate/g' /usr/share/applications/kate.desktop
    sudo sed -i 's/=Text\ Editor/=Kate/g' /usr/share/applications/kate.desktop
    sudo reboot (to apply changes)
    ```
* Configure **Terminal**
  * Top Menu --> Edit --> Profile Preferences
    * General --> Initial terminal size: **80 x 20**
    * Scrolling --> Check **Unlimited**
    * Colors --> Check **Use transparent background**
* Configure **Terminator**
  * Right Click --> Preferences --> Profiles
    * Profiles --> Edit 'default' Profile: 
      * Colors --> UnCheck **Use colors from system theme**
      * Colors --> Built-in schemes: **Ambience**
      * Background --> Check **Transparent background**
      * Background --> Transparency slider: 0.9
      * Scrolling --> Check **Infinite Scrollback**
    * Layouts
      * Split Terminator screen into 4 terminals and then click 'Add' at bottom left
      * Name the layout, e.g. 'Quad'
* Set up Desktop Apps
  * Create directory for helper scripts
    * `mkdir ~/helper_scripts && cd helper_scripts`
  * Copy over the icons folder from Spira server at:
      * `MSRAL/Current\ Projects/MAVs/Linux/desktop_apps_new/my_scripts`
  * Create script to open Terminator using Quad layout
    * Create a file named `runTerminator` and add the following:
    ```
    #!/bin/bash
    nohup terminator -l Quad
    exit
    ```
    * Make runTerminator executable
      * `chmod +x runTerminator`
    * Create a desktop app to launch terminator
      * Create file called `termQuad.desktop` and add the following:
      ```
      [Desktop Entry]
      Name=Terminator (Quad Layout)
      Comment=Launches Terminator with 4 shells
      Exec=/home/intel2/helper_scripts/runTerminator
      Icon=/home/intel2/helper_scripts/icons/terminator.png
      Terminal=false
      Type=Application
      ```
    * Navigate to Desktop and right-click termQuad.desktop and select 'Properties'
      * Check 'Allow executing file as program'
        * Icon should now appear
      * Double-click to open Terminator, then Lock to Launcher
      
* Configure **File Manager**
  * Open File Browser, then select Edit --> Preferences
    * Views --> View new folders using: **List View**
    * Views --> List View Defaults: zoom level: `33%`
    * Display --> List View --> Check **Navigate folders in a tree**
* Configure window size of **Compiz File Browser**
  * `cd ~/.config/compiz-1/compizconfig`
  * Add **size** setting in Compiz config file
    * `nano config`
    * Add `size = 800, 100` at bottom of file

