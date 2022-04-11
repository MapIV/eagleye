
<img src="docs/logo.png" height="45"> (Alpha version)

![example workflow](https://github.com/MapIV/eagleye/actions/workflows/build.yml/badge.svg)

[Demo Video](https://youtu.be/u8Nan38BkDw)

[![](http://img.youtube.com/vi/u8Nan38BkDw/0.jpg)](http://www.youtube.com/watch?v=u8Nan38BkDw "Eagleye")

## What is Eagleye

Eagleye is an open-source software for vehicle localization utilizing GNSS and IMU[[1]](https://www.researchgate.net/publication/329619280_Low-cost_Lane-level_Positioning_in_Urban_Area_Using_Optimized_Long_Time_Series_GNSS_and_IMU_Data). Eagleye provides highly accurate and stable vehicle position and orientation by using GNSS Doppler[[2]](https://www.jstage.jst.go.jp/article/jsaeronbun/44/1/44_20134048/_article/-char/en)[[3]](https://www.jstage.jst.go.jp/article/jsaeijae/3/2/3_20124032/_article/-char/ja)[[4]](https://ieeexplore.ieee.org/document/6236946)[[5]](https://www.researchgate.net/publication/290751834_Automotive_positioning_based_on_bundle_adjustment_of_GPS_raw_data_and_vehicle_trajectory)[[6]](https://ci.nii.ac.jp/naid/10029931657/). The flowchart of the algorithm is shown in the figure below. The algorithms in this software are based on the outcome of the research undertaken by [Machinery Information Systems Lab (Meguro Lab)](https://www2.meijo-u.ac.jp/~meguro/index.html) in Meijo University.

![Flowchart of Eagleye](docs/flowchart.png)

## Recommended Sensors
**GNSS receiver**
* [Ublox NEO-M8T](https://www.u-blox.com/en/product/neolea-m8t-series) / [EVK-M8T](https://www.u-blox.com/product/evk-8evk-m8)
* [Ublox ZED-F9P](https://www.u-blox.com/en/product/zed-f9p-module) / [C099-F9P](https://www.u-blox.com/en/product/c099-f9p-application-board)

**GNSS Antenna**

* [Ublox ANN-MB Series](https://www.u-blox.com/en/product/ann-mb-series)
* [Tallysman TW2710](http://www.tallysman.com/index.php/gnss/products/antennas-gpsbeidougalileoglonass/tw2710/)

**IMU**
* [Tamagawa Seiki TAG300 Series](http://mems.tamagawa-seiki.com/en/product/)
* [ANALOG DEVICES ADIS16475](https://www.analog.com/products/adis16475.html#product-overview)

**Wheel speed sensor**

* Eagleye uses vehicle speed acquired from CAN bus.

## How to install

### RTKLIB

Clone and Build MapIV's fork of [RTKLIB](https://github.com/MapIV/RTKLIB/tree/rtklib_ros_bridge). You can find more details about RTKLIB [here](http://www.rtklib.com/).

	sudo apt-get install gfortran
	cd $HOME
	git clone -b rtklib_ros_bridge https://github.com/MapIV/RTKLIB.git
	cd $HOME/RTKLIB/lib/iers/gcc/
	make
	cd $HOME/RTKLIB/app
	make 

### ROS Packages

Clone and build the necessary packages for Eagleye. ([rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge), [nmea_comms](https://github.com/MapIV/nmea_comms.git))

	cd $HOME/catkin_ws/src
	git clone https://github.com/MapIV/eagleye.git
	git clone https://github.com/MapIV/rtklib_ros_bridge.git
	git clone https://github.com/MapIV/nmea_comms.git
	rosdep install --from-paths src --ignore-src -r -y
	catkin_make -DCMAKE_BUILD_TYPE=Release

## Configuration

### GNSS
#### single point positioning by F9P

1. RTKLIB settings.

Change `inpstr1-path` of `$HOME/RTKLIB/app/rtkrcv/conf/rtklib_ros_bridge_single.conf` according to the serial device you use.

ie)
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:230400:8:n:1:off  

2. nmea_comms settings.

Change `arg name="port"` of `$HOME/catkin_ws/src/nmea_comms/launch/f9p_nmea_sentence.launch` according to the serial device you use.

ie)
>\<arg name="port" default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG0JNPDS-if00-port0" />

3. GNSS receiver settings.
Configure the receiver settings using [u-center](https://www.u-blox.com/product/u-center).

* UART1(Connect to RTKLIB) Enable UBX message (output rate 5Hz, baudrate 230400) ※ Set to output only RAWX and SFRBX
* UART2(Connect to nmea_comms) Enable NMEA message (output rate 1Hz, baudrate 115200) ※ Set to output only GGA and RMC

[This file (eagleye_f9p_conf.txt)](https://www.dropbox.com/s/5mq9hbygnviojoh/eagleye_f9p_conf.txt?dl=0) is a sample configuration file for F9P.  
Open u-center.  
Tools/Receiver Configuration.../Load configuration "Transfer file -> GNSS"

To load the configuration, change the ublox FW version to 1.10.

#### Real Time Kinematic by F9P

1. RTKLIB settings.

Change `inpstr1-path`, `inpstr2-path`, `inpstr2-format`, and `ant2-postype` of `$HOME/RTKLIB/app/rtkrcv/conf/rtklib_ros_bridge_meijo_rtk.conf` according to the serial device you use.

ie)
>inpstr1-path =/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:230400:8:n:1:off  
>inpstr2-path =:@rtk2go.com:2101/Meijo-Ublox  
>inpstr2-format =ubx  
>ant2-postype       =llh        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw)  
ant2-pos1          =35.1348599331534          # (deg|m) If ant2-postype is llh or xyz, the position of the reference station must be specified by ant2-pos1, ant2-pos2, and ant2-pos3.  
ant2-pos2          =136.973613158051          # (deg|m)  
ant2-pos3          =102.502548295454          # (m|m)  

2. nmea_comms settings.  

Change `arg name="port"` of `$HOME/catkin_ws/src/nmea_comms/launch/f9p_nmea_sentence.launch` according to the serial device you use.

ie)
>\<arg name="port" default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG0JNPDS-if00-port0" />

3. GNSS receiver settings.
Configure the receiver settings using [u-center](https://www.u-blox.com/product/u-center).

The following is a sample configuration file for F9P.  

(1) Settings for receivers that output aircraft that output NMEA (positioning results RTK'd by the F9P internal engine)  
eagleye_f9p_nmea_conf.txt  
https://www.dropbox.com/s/3viqyqutipn5dpj/eagleye_f9p_nmea_conf.txt?dl=0  
(2) Settings for receivers that measure RAW data through RTKLIB and output Doppler velocity.  
eagleye_f9p_raw_conf.txt  
https://www.dropbox.com/s/acz98v30rtgbmsx/eagleye_f9p_raw_conf.txt?dl=0  
Open u-center.  
Tools/Receiver Configuration.../Load configuration "Transfer file -> GNSS"

To load the configuration, change the ublox FW version to 1.10.

### IMU

1. IMU settings.

* Output rate 50Hz

2. Check the rotation direction of z axis of IMU being used. If you look from the top of the vehicle, if the left turn is positive, set "reverse_imu" to `true` in `eagleye/eagleye_rt/config/eagleye_config.yaml`.

		 reverse_imu: true

### Eagleye parameters

The parameters of eagleye can be set in the [eagleye_config.yaml](https://github.com/MapIV/eagleye/blob/master/eagleye_rt/config/eagleye_config.yaml). The default settings are 5Hz for GNSS and 50Hz for IMU.


The TF between sensors can be set in [sensors_tf.yaml](https://github.com/MapIV/eagleye/blob/master/eagleye_util/tf/config/sensors_tf.yaml).
The settings are reflected by describing the positional relationship of each sensor with respect to base_link. If you want to change the base frame, [change basic_parent_flame](https://github.com/MapIV/eagleye/blob/master/eagleye_util/tf/config/sensors_tf.yaml#L2) to reflect the change.


## eagleye_rt
### How to run 
#### Use sample data

1. Play the sample data.  

		rosparam set use_sim_time true
		rosbag play --clock eagleye_sample.bag

2. Launch eagleye.  

		roslaunch eagleye_rt eagleye_rt.launch

The estimated results will be output about 100 seconds after playing the rosbag. This is because we need to wait for the data to accumulate for estimation.

### Running real-time operation

1. Check if wheel speed (vehicle speed) is published in `/can_twist` topic.

* Topic name: /can_twist
* Message type: geometry_msgs/TwistStamped twist.liner.x


2. Check if the IMU data is published in `/imu/data_raw` topic.

3. Start RTKLIB.

	ex. single point positioning

		cd $HOME/RTKLIB
		bash rtklib_ros_bridge_single.sh

	ex. Real Time Kinematic
 
 		cd $HOME/RTKLIB
		bash rtklib_ros_bridge_meijo_rtk.sh

4. Check if RTKLIB is working by execute the following command in the terminal. If the RTKLIB is working correctly, positioning information is appeared continuously in the terminal.

		status 0.1  

5. Start rtklib_ros_bridge.

		roslaunch rtklib_bridge rtklib_bridge.launch   

6. Start nmea_comms.

		roslaunch nmea_comms f9p_nmea_sentence.launch

7. Start eagleye.

		roslaunch eagleye_rt eagleye_rt.launch

### Note

To visualize the eagleye output location /eagleye/fix, for example, use the following command  

	roslaunch eagleye_fix2kml fix2kml.launch

## eagleye_pp

- [eagleye_pp](eagleye_pp):post-processing version

## Sample data
### ROSBAG

| No. | Date | Place | Sensors | Link |
|-----|------|-------|---------| ---- |
|1|2020/01/27|Moriyama, Nagoya<br>[route](https://www.google.com/maps/d/edit?mid=1pK4BgrGtoo14nguArDf-rZDqIL5Cg-v5&usp=sharing)|GNSS: Ublox F9P<br>IMU: Tamagawa AU7684<br>LiDAR: Velodyne HDL-32E|[Download](https://www.dropbox.com/s/pfs307qn7gfeou5/eagleye_sample_20200127.bag?dl=0)|
|2|2020/07/15|Moriyama, Nagoya<br>[route](https://www.google.com/maps/d/edit?mid=1DnXfZBTSsHpWlzTAcENmFxo17r3PxGxM&usp=sharing)|GNSS: Ublox F9P with RTK<br>IMU: Tamagawa AU7684<br>LiDAR: Velodyne VLP-32C|[Download](https://www.dropbox.com/s/w9ag6gs17bi80st/eagleye_sample_20200715.bag?dl=0)

### Maps
The 3D maps (point cloud and vector data) of the route is also available from [Autoware sample data](https://drive.google.com/file/d/1Uwp9vwvcZwaoZi4kdjJaY55-LEXIzSxf/view).

## Research Papers for Citation
1. J Meguro, T Arakawa, S Mizutani, A Takanose, "Low-cost Lane-level Positioning in Urban Area Using Optimized Long Time Series GNSS and IMU Data", International Conference on Intelligent Transportation Systems(ITSC), 2018 [Link](https://www.researchgate.net/publication/329619280_Low-cost_Lane-level_Positioning_in_Urban_Area_Using_Optimized_Long_Time_Series_GNSS_and_IMU_Data)

1. Takeyama Kojiro, Kojima Yoshiko, Meguro Jun-ichi, Iwase Tatsuya, Teramoto Eiji, "Trajectory Estimation Based on Tightly Coupled Integration of GPS Doppler and INS" -Improvement of Trajectory Estimation in Urban Area-, Transactions of Society of Automotive Engineers of Japan   44(1) 199-204, 2013 [Link](https://www.jstage.jst.go.jp/article/jsaeronbun/44/1/44_20134048/_article/-char/en)

1. Junichi Meguro, Yoshiko Kojima, Noriyoshi Suzuki, Teramoto Eiji, "Positioning Technique Based on Vehicle Trajectory Using GPS Raw Data and Low-cost IMU", International Journal of Automotive Engineering 3(2) 75-80,  2012 [Link](https://www.jstage.jst.go.jp/article/jsaeijae/3/2/3_20124032/_article/-char/ja)

1. K Takeyama, Y Kojima, E Teramoto, "Trajectory estimation improvement based on time-series constraint of GPS Doppler and INS in urban areas", IEEE/ION Position, Location and Navigation Symposium(PLANS), 2012 [Link](https://ieeexplore.ieee.org/document/6236946)

1. Junichi Meguro, Yoshiko Kojima, Noriyoshi Suzuki, Eiji Teramoto, "Automotive Positioning Based on Bundle Adjustment of GPS Raw Data and Vehicle Trajectory", International Technical Meeting of the Satellite Division of the Institute of Navigation (ION), 2011 [Link](https://www.researchgate.net/publication/290751834_Automotive_positioning_based_on_bundle_adjustment_of_GPS_raw_data_and_vehicle_trajectory)

1. Yoshiko Kojima, et., al., "Precise Localization using Tightly Coupled Integration based on Trajectory estimated from GPS Doppler", International Symposium on Advanced Vehicle Control(AVEC), 2010 [Link](https://ci.nii.ac.jp/naid/10029931657/)

1. A. Takanose, et., al., "Eagleye: A Lane-Level Localization Using Low-Cost GNSS/IMU", Intelligent Vehicles (IV) workshop, 2021 [Link](https://www.autoware.org/iv2021video-workshoppapers3)

## License
Eagleye is provided under the [BSD 3-Clause](https://github.com/MapIV/eagleye/blob/master/LICENSE) License.

## Contacts

If you have further question, email to map4@tier4.jp.
