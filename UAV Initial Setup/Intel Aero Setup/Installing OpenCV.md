## Install OpenCV 3.4 (~1 hr)
  * Reference: [OpenCV Docs](https://docs.opencv.org/3.4.3/d7/d9f/tutorial_linux_install.html)
  * Full list of [CMake Options](https://github.com/opencv/opencv/blob/master/CMakeLists.txt#L198)

1. Install Dependencies
  * General dependencies based on OpenCV Documentation
  ```
  sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev \
       libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev \
       libjpeg-dev libjasper-dev libdc1394-22-dev checkinstall yasm libxine2-dev \
       libv4l-dev libav-tools unzip libdc1394-22 libpng12-dev libtiff5-dev tar dtrx \
       libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev \
       libxvidcore-dev x264 libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev \
       libqt4-dev libmp3lame-dev
  ```
  ```
  sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev \
       libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev \
       libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev \
       libqt4-dev default-jdk
  ```
  
* Download OpenCV and OpenCV contrib source code
  ```
  cd ~
  mkdir OpenCV
  cd OpenCV
  git clone -b 3.4 --single-branch https://github.com/opencv/opencv.git
  git clone -b 3.4 --single-branch https://github.com/opencv/opencv_contrib.git
  ```
  
* Build from source (~40 minutes)
  ```
  cd ~/OpenCV/opencv
  mkdir release
  cd release
  sudo cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_NEW_PYTHON_SUPPORT=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_GTK=ON -D WITH_QT=ON -D WITH_GSTREAMER_0_10=ON -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_OPENGL-ES=ON -D WITH_V4L=ON -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
  sudo make -j4
  sudo make install
  ```
* Create config file (if it doesn't exist)
  * `sudo nano /etc/ld.so.conf.d/opencv.conf`
  * Add `/usr/local/lib/` to the file
  * `sudo ldconfig`
* Test installation with Python
  ```
  python
  import cv2
  recognizer = cv2.face.LBPHFaceRecognizer_create()
  detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
  ```
* If Needed, Remove OpenCV source to free up storage
  * `sudo rm -r ~/OpenCV/opencv/release`
  * OR
  * `sudo rm -r ~/OpenCV`
