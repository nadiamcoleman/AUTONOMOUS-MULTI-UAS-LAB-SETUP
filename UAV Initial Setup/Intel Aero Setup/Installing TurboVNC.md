## TurboVNC Installation
  * References: 
    * [XKEYBOARD Issue](https://unix.stackexchange.com/questions/346107/keyboard-mapping-wrong-only-in-specific-applications-under-tightvnc)
    * [Website](https://www.turbovnc.org/)
  
1. Pre-requisite: libjpeg-turbo
  * Install pre-req: `sudo apt install libpam0g-dev`
  * Download, build \& install source
  ```
  cd ~
  git clone https://github.com/libjpeg-turbo/libjpeg-turbo.git
  cd libjpeg-turbo
  git checkout tags/2.0.1
  mkdir build
  cd build
  cmake -G"Unix Makefiles" ..
  make -j4
  sudo make install
  ```
  
* Download \& Build From Source
  ```
  cd ~
  git clone https://github.com/TurboVNC/turbovnc.git
  git checkout tags/2.2.1
  mkdir build
  cd build
  cmake -G"Unix Makefiles" -DTVNC_BUILDJAVA=0 ..
  make -j4
  ```
* Set up the vnc password: `vncpasswd`
    * Password: **intel2**
    * View-only password: **No**
    
* Create shell functions to start/stop vncserver
  * Add a function to `~/.bashrc` to start server
    * ```
      echo -e "\n# Start TurboVNC Server\nfunction ovnc() {\
      \n  ~/turbovnc/build/bin/vncserver\n}"\
      >> ~/.bashrc 
      source ~/.bashrc
      ```
    * To run the server, type: `ovnc` in the shell
      * This will output:
      `New 'X' desktop is intel2-aero2:1`
      * Calling `ovnc` again will start new servers at `:2`, `:3`, etc.
      * View server at `<IP Address>:5901`
        * **NOTE:** `5901` is for `:1`, `5902` for `:2`, etc.
  * Add a function to `~/.bashrc` to kill server
    * ```
      echo -e "\n# Kill TurboVNC Server\nfunction okill() {\
      \n  ~/turbovnc/build/bin/vncserver -kill :$1\n}"\
      >> ~/.bashrc 
      source ~/.bashrc
      ```
    * To kill the server: `okill 1`
      * **NOTE:** `1` above indicates index number (could be `1`, `2`...etc.)

* Install TurboVNC Viewer
  * Download from [here](https://sourceforge.net/projects/turbovnc/files/)
* NOTE: 
  * Executables are now in `~/turbovnc/build/bin/`
