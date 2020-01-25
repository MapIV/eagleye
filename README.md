
# eagleye

[![CircleCI](https://circleci.com/gh/MapIV/eagleye.svg?style=svg&circle-token=7961cc3947c36b93252f458a679dfcb9aa977b5b)](https://circleci.com/gh/MapIV/eagleye)

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

		cd $HOME  
		git clone https://github.com/MapIV/RTKLIB.git
		cd $HOME/RTKLIB     
		git checkout rtklib_ros_bridge    

	[About RTKLIB](http://www.rtklib.com)

2) Build RTKLIB.

		cd $HOME/RTKLIB/lib/iers/gcc/  
		make   
		cd $HOME/RTKLIB/app  
		make   

3) Change the permissions of the two files.

		cd $HOME/RTKLIB/app/rtkrcv/gcc  
		chmod 755 rtkstart.sh  
		chmod 755 rtkshut.sh  

4) Next, download and build rtklib_ros_bridge.

		cd $HOME/catkin_ws/src  
		git clone https://github.com/MapIV/rtklib_ros_bridge.git  
		cd ..  
		catkin_make -DCMAKE_BUILD_TYPE=Release  

5) Download and build ADI's IMU driver.

		cd
		cd catkin_ws/src
		git clone https://github.com/tork-a/adi_driver.git
		cd ..
		catkin_make

## Configuration
1) Open RTKLIB settings.

		gedit $HOME/RTKLIB/app/rtkrcv/conf/rtklib_ros_bridge_sample.conf

2) Set the serial device on line 10. If you connect using USB, it is OK.

>Line 10:  
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:9600:8:n:1:off  

※If you know the device number "/dev/ttyACM-" but OK.

3) Next, configure the receiver from ublox application, u-center.The usage of u-center is not described here. Below is an overview of the settings. (Here is how to use a Ublox receiver)  

* Enable UBX message ※Set to output only RAWX and SFRBX
* Save your settings last.

	[About u-center](https://www.u-blox.com/product/u-center)  

4) Configure the ADI IMU driver settings. Configure the ADI IMU driver settings. Open the launch file and configure the serial settings and output rate settings.

		cd
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

		cd  
		gedit catkin_ws/src/adi_driver/src/adis16470.cpp  

>Line 361~362:  
>    gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 2621440.0;  
    accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 262144000.0;  

[ADIS 16475 Data Sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16475.pdf)  

		cd  
		cd catkin_ws  
		catkin_make  

6) Check the rotation direction of z axis of IMU being used.If you look from the top of the vehicle, if the left turn is positive, set "reverse_imu" in the launch file to "true".

		cd  
		gedit catkin_ws/src/eagleye/launch/eagleye.launch  

>param name="/eagleye/reverse_imu" type="bool" value="true"  

## Usage
1) The wheel speed information (vehicle speed information) is published as follows.

* Topic name: /can_twist
* Message type: geometry_msgs/TwistStamped twist.liner.x

2) Connect the IMU and start the ADI driver.

		cd  
		cd catkin_ws/src/adi_driver/launch  
		roslaunch adis16470.launch  

3) Connect the GNSS receiver and start RTKLIB.

		cd $HOME/RTKLIB  
		bash rtklib_ros_bridge_sample.sh  

4) Check the status of RTKLIB. If GPS Time is moving, it is OK. Execute the following command in the terminal of item 3.

		status 0.1  

※If GPS Time is not working, there may be a mistake in the receiver settings or RTKLIB settings.

5) Start rtklib_ros_bridge.

		rosrun rtklib_bridge rtklib_bridge   

6) Start eagleye.

		roslaunch eagleye_localization.launch


## Research Papers for Citation
1. J Meguro, T Arakawa, S Mizutani, A Takanose, "Low-cost Lane-level Positioning in Urban Area Using Optimized Long Time Series GNSS and IMU Data", International Conference on Intelligent Transportation Systems(ITSC), 2018 [Link](https://www.researchgate.net/publication/329619280_Low-cost_Lane-level_Positioning_in_Urban_Area_Using_Optimized_Long_Time_Series_GNSS_and_IMU_Data)

1. Takeyama Kojiro, Kojima Yoshiko, Meguro Jun-ichi, Iwase Tatsuya, Teramoto Eiji, "Trajectory Estimation Based on Tightly Coupled Integration of GPS Doppler and INS" -Improvement of Trajectory Estimation in Urban Area-, Transactions of Society of Automotive Engineers of Japan   44(1) 199-204, 2013 [Link](https://www.jstage.jst.go.jp/article/jsaeronbun/44/1/44_20134048/_article/-char/en)

1. Junichi Meguro, Yoshiko Kojima, Noriyoshi Suzuki, Teramoto Eiji, "Positioning Technique Based on Vehicle Trajectory Using GPS Raw Data and Low-cost IMU", International Journal of Automotive Engineering 3(2) 75-80,  2012 [Link](https://www.jstage.jst.go.jp/article/jsaeijae/3/2/3_20124032/_article/-char/ja)

1. K Takeyama, Y Kojima, E Teramoto, "Trajectory estimation improvement based on time-series constraint of GPS Doppler and INS in urban areas", IEEE/ION Position, Location and Navigation Symposium(PLANS), 2012 [Link](https://ieeexplore.ieee.org/document/6236946)

1. Junichi Meguro, Yoshiko Kojima, Noriyoshi Suzuki, Eiji Teramoto, "Automotive Positioning Based on Bundle Adjustment of GPS Raw Data and Vehicle Trajectory", International Technical Meeting of the Satellite Division of the Institute of Navigation (ION), 2011 [Link](https://www.researchgate.net/publication/290751834_Automotive_positioning_based_on_bundle_adjustment_of_GPS_raw_data_and_vehicle_trajectory)

1. Yoshiko Kojima, et., al., "Precise Localization using Tightly Coupled Integration based on Trajectory estimated from GPS Doppler", International Symposium on Advanced Vehicle Control(AVEC), 2010 [Link](https://ci.nii.ac.jp/naid/10029931657/)
