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

2) Configure the receiver settings. Line 10 contains the serial device setting items. 
>Example)
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:9600:8:n:1:off

* 17-Feb-2013 re-design

----
## thanks
* [markdown-js](https://github.com/evilstreak/markdown-js)
