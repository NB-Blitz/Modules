# Raspberry Pi
Code run on a raspberry pi. This code is intended for use in the First Robotics Competition.

## Pi Setup
### Install Raspbian
Download: [Here](https://www.raspberrypi.org/software/operating-systems/#raspberry-pi-os-32-bit)
Install: [Here](https://www.balena.io/etcher/)

### Install OpenCV
**Update**
```bash
sudo apt-get update
```

**Install Dependencies**
```bash
sudo apt-get install build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran python3-dev python3-pip
```

**Pip Install Dependencies**
```bash
sudo pip3 install numpy scipy
```

**Download OpenCV Source**
```bash
wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.3.zip
```

**Extract**
```bash
unzip opencv.zip
```

**Compile**
```bash
cd ~/opencv-3.4.3/
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \ 
      -D CMAKE_INSTALL_PREFIX=/usr/local \ 
      -D INSTALL_PYTHON_EXAMPLES=ON \ 
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.4.3/modules \ 
      -D BUILD_EXAMPLES=ON ..
```

**Build** *(This step will take awhile)*
```bash
make -j4
sudo make install
sudo ldconfig
sudo apt-get update
sudo reboot
```

### Run Code
**Download**
```bash
git clone https://github.com/NB-Blitz/Modules.git
cd ./Modules/Raspberry Pi/some/other/folder/
```

**Run**
```bash
make
./main
```