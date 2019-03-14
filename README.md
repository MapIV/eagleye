
# imu gnss localizer

## Overview

It is a program that combines IMU and GNSS to perform highly accurate self position estimation.

## Description

This is an algorithm to estimate the vehicle position with high cycle and high accuracy using IMU, GNSS receiver and wheel speed sensor.
The main functions are as follows.
*  IMU temperature drift estimation
*  Wheel speed scale factor estimation Azimuth estimation
*  Latitude and longitude estimation
is.  

Both relative position and absolute position can be output.

## Recommended sensor

GNSS receiver: [Ublox EVK-M8T](https://www.u-blox.com/product/evk-8evk-m8)  
GNSS antenna: [Tallysman TW2710](http://www.tallysman.com/index.php/gnss/products/antennas-gpsbeidougalileoglonass/tw2710/)  
IMU: [ANALOG DEVICES ADIS16475](https://www.analog.com/products/adis16475.html#product-overview)  
Wheel speed sensor: Sensor equipped on the vehicle.

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

5) When using ADIS 16475, rewrite the program as follows. ※It is not necessary to use  ADIS 16470.

		cd..  
		gedit catkin_ws/src/adi_driver/src/adis16470.cpp  

>Line 361~362:  
>    gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 2621440.0;  
    accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 262144000.0;  

[ADIS 16475 Data Sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16475.pdf)  

		cd..  
		cd catkin_ws  
		catkin_make  

## Usage
1) The wheel speed information (vehicle speed information) is published as follows.

* Topic name: /Vehicle/Velocity  
* Message type: geometry_msgs/Twist/liner.x

2) Connect the IMU and start the ADI driver.

		cd..  
		cd catkin_ws/src/adi_driver/launch  
		roslaunch adis16470.launch  

3) Connect the GNSS receiver and start RTKLIB.

		cd..  
		cd RTKLIB  
		./launchscript.sh  

4) Check the status of RTKLIB. If GPS Time is moving, it is OK. Execute the following command in the terminal of item 3.

		status 0.1  

※If GPS Time is not working, there may be a mistake in the receiver settings or RTKLIB settings.

5) Start imu gnss localizer.

		cd..  
		cd catkin_ws/src/imu_gnss_localizer/launch
		roslaunch imu_gnss_localizer.launch

6) After traveling for a while and estimation is started, latitude and longitude will be output with the following topic names.

* Topic name: /imu_gnss_pose   
* Message type: geometry_msgs/PoseStamped
