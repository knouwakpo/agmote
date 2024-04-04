# agmote
Suite of scripts and program to collect sensor data from agricultural fields and send to the cloud

The Gateway are composed of 2 main components:

The local radio based on the RFM69 radio module

The Cellular communication component based on the Blues Wireless notecard


# The Local RFM69 radio module.
We use the Adafruit Radio Bonnets with OLED Display - RFM69
The RFM69App is written to configure the radio and display incoming data information on the OLED display
The RFM69App makes heavy use of the pigpio library.

## Pre-requisite:
Ensure the Raspberry Pi is up-to-date
```
$ sudo apt update
$ sudo apt upgrade
```
For the OLED to work properly, follow instructions in the README.md inside libssd1306
#### Key steps
```
$ sudo apt -y install libfreetype6-dev fonts-freefont-ttf ttf-bitstream-vera \
        autoconf automake libtool autotools-dev build-essential pkg-config
``` 
### optional pre-requisites
```
$ sudo apt -y install libev-dev libunistring-dev
```
### optional Microsoft Fonts installer, if you want it
```
$ sudo apt -y install ttf-mscorefonts-installer
```

## Compile and install libssd1306.

Make sure to run

```
$ sudo make install
```
at the end to ensure that RFM69App can successfuly compile


## If needed, compile RFM69App
This is usually not necessary since the rfm69app executable is already available in the RFM69App/build directory

Make the apps and Flume directories on the Pi home
```
$ mkdir .apps
$ mkdir Flume
```
## Copy the rfm69app executable to ~/.apps
```
$ cp rfm69app ~/.apps/rfm69app
$ chmod +x ~/.apps/rfm69app
```

# For the Blues Wireless notecard communication

Communication with the notecard is done using python


## Pre-requisites
```
$ pip3 install note-python
$ pip3 install python-periphery
```


# Setup the crontab

# Copy the rest of the files inside RFM69_gateway_apps to ~/.apps

# Make sure I2C and SPI are enabled on the pi
