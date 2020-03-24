## General udev rule setup
  1. Obtain device info
    * References: [Debian Wiki: udev](https://wiki.debian.org/udev), [Overview](http://reactivated.net/writing_udev_rules.html#udevinfo)
    * `udevadm info --attribute-walk --name /dev/<device_name>`
  * Place udev rule files in: `/etc/udev/rules.d/`
    * Example file: `99-usb-serial.rules` 
    * ```
    # 3.3/5V FTDI Adapter
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AI04XENV", SYMLINK+="ttyFTDI3_5"
    # 5V FTDI Adapter
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AK05C4L0", SYMLINK+="ttyFTDI5"
    ```
