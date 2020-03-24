## Update Yocto OS
  * [Reference](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-Setup)

1. Format USB drive (10 - 15 minutes)
  * Insert USB drive into Laptop/PC
  * Erase & Format USB drive
    * **Windows 10**
      * Right-click USB drive and select **Format**
    * **Mac**
      * Open **Disk Utility**
      * Select the USB drive and click **Erase**
      * Set new format to ExFAT and leave other options as defaults
  * Navigate to the [Intel Download Center](https://downloadcenter.intel.com/download/27833/Intel-Aero-Platform-for-UAVs-Installation-Files?v=t)
    * Download [intel-aero-image-1.6.2.iso](https://downloadcenter.intel.com/download/27833/Intel-Aero-Platform-for-UAVs-Installation-Files?v=t)
    * Download and install [Etcher](https://etcher.io/)
    * Insert USB Drive
    * Open Etcher
    * Click **Select Image** and browse to the ***.iso** downloaded above
    * Select the USB Drive
    * Click **Flash**
* Cable Connection Notes
  * Use an **OTG** USB adapter (NOT JUST A NORMAL USB) to connect a USB hub with a keyboard, mouse and USB flash drive
  * Use a micro-HDMI to HDMI cable and connect directly to a monitor with HDMI (avoid using e.g. HDMI to DVI connector since it seems to make the Aero crash)
* Install Yocto
  * Hold ESC while booting up to get to BIOS
  * Select the bootable USB to startup
  * Select `install`
    * Refer to [02-Initial Setup](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-Setup) for information about install process (e.g. LED colors, etc.)
  * Flash BIOS
    * Copy the BIOS firmware onto the Aero Board
        * [BIOS Link](https://downloadcenter.intel.com/downloads/eula/27833/Intel-Aero-Platform-for-UAVs-Installation-Files?httpDown=https%3A%2F%2Fdownloadmirror.intel.com%2F27833%2Feng%2Faero-bios-01.00.16-r1.corei7_64.rpm)
        * Save it onto a USB drive, and then connect USB to aero board
        * Mount the USB drive
        ```
        mkdir /media/usb-drive
        mount /dev/sda /media/usb-drive
        cd /media/usb-drive
        cp aero-bios-01.00.16-r1.corei7_64.rpm /home/root/
        ```
    * Update bios by typing `aero-bios-update` in the terminal
      * Reboot after it completes (this may require unplugging power supply)
          * This may require unplugging the power supply if the terminal hangs after displaying the messages listed on the above '02-Initial-Setup' page

  * Flash FPGA
    * `cd /etc/fpga/`
    * `jam -aprogram aero-rtf.jam`
    * Terminal will say `Exit code = 0... Success` when done
  * Flash Flight Controller with PX4 Firmware
    * `cd /etc/aerofc/px4/`
    * `aerofc-update.sh nuttx-aerofc-v1-default.px4`
  * Reboot
  * Confirm all versions are correct: `aero-get-version.py`
  
