
# imu gnss localizer

## Overview

It is a program that combines IMU and GNSS to perform highly accurate self position estimation.

## Description

Write markdown text in this textarea.

## Install

1) First, download the modified RTKLIB to your home directory.

		cd  
		git clone https://github.com/MapIV/RTKLIB.git  

	[About RTKLIB](https://wikipedia.org)

2) Build RTKLIB.

		cd RTKLIB/app  
		chmod 755 makeall.sh  
		./makeall.sh  

3) Change the permissions of the two files.

		cd  
		cd RTKLIB/app/rtkrcv/gcc  
		chmod 755 rtkstart.sh  
		chmod 755 rtkshut.sh  

4) Next, download and build imu gnss localizer.

		cd  
		cd catkin_ws/src  
		git clone https://github.com/MapIV/imu_gnss_localizer.git  
		cd ..  
		catkin_make  

5) Download and build ADI's IMU driver.

		cd
		cd catkin_ws/src
		git clone https://github.com/tork-a/adi_driver.git
		cd ..
		catkin_make

## Configuration
1) Open RTKLIB settings.

		cd..
		gedit RTKLIB/app/rtkrcv/imu_gnss_localizer.conf

2) Set the serial device on line 10. If you connect using USB, it is OK.

>Line 10:  
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:9600:8:n:1:off  

※If you know the device number "/dev/ttyACM-" but OK.

3) Next, configure the receiver from ublox application, u-center.The usage of u-center is not described here. Below is an overview of the settings.  

* Disenable NMEA message
* Enable UBX message ※Set to output only RAWX and SFRBX
* Set the output rate to 10 Hz.
* Save your settings last.

	[About u-center](https://www.u-blox.com/product/u-center)

4) Configure the AMU IMU driver settings. Configure the AMU IMU driver settings. Open the launch file and configure the serial settings and output rate settings.

		cd..
		gedit catkin_ws/src/adi_driver/launch/adis16470.launch

>Line 2:  
>arg name="with_filter" default="false"
>Line 5:  
>arg name="device" default="/dev/serial/by-id/usb-Devantech_Ltd._USB-ISS._00041745-if00"  
>Line 9:  
>arg name="rate" default="50" 
>Line 10:  
>arg name="publish_tf" default="false"

※If you know the device number "/dev/ttyACM-" but OK.

5) Please do this work when using ADIS 16475.
