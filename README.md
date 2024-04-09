<img src="docs/logo.png" height="45"> (Alpha version)

![example workflow](https://github.com/MapIV/eagleye/actions/workflows/build.yml/badge.svg)

[Demo Video](https://youtu.be/u8Nan38BkDw)

[![](http://img.youtube.com/vi/u8Nan38BkDw/0.jpg)](http://www.youtube.com/watch?v=u8Nan38BkDw "Eagleye")

## What is Eagleye

Eagleye is an open-source software for vehicle localization utilizing GNSS and IMU[[1]](https://www.researchgate.net/publication/329619280_Low-cost_Lane-level_Positioning_in_Urban_Area_Using_Optimized_Long_Time_Series_GNSS_and_IMU_Data). Eagleye provides highly accurate and stable vehicle position and orientation by using GNSS Doppler[[2]](https://www.jstage.jst.go.jp/article/jsaeronbun/44/1/44_20134048/_article/-char/en)[[3]](https://www.jstage.jst.go.jp/article/jsaeijae/3/2/3_20124032/_article/-char/ja)[[4]](https://ieeexplore.ieee.org/document/6236946)[[5]](https://www.researchgate.net/publication/290751834_Automotive_positioning_based_on_bundle_adjustment_of_GPS_raw_data_and_vehicle_trajectory)[[6]](https://ci.nii.ac.jp/naid/10029931657/). The flowchart of the algorithm is shown in the figure below. The algorithms in this software are based on the outcome of the research undertaken by [Machinery Information Systems Lab (Meguro Lab)](https://www2.meijo-u.ac.jp/~meguro/index.html) in Meijo University.

![Flowchart of Eagleye](docs/flowchart.png)

## Architecture

![Architecture of Eagleye](docs/eagleye_architecture.drawio.svg)

## Recommended Sensors
**GNSS receiver**
* [Septentrio Mosaic development kit with GNSS antenna](https://shop.septentrio.com/en/shop/mosaic-x5-devkit) 

**IMU**
* [Tamagawa Seiki TAG300 Series](http://mems.tamagawa-seiki.com/en/product/)
* [ANALOG DEVICES ADIS16475](https://www.analog.com/products/adis16475.html#product-overview)

**Wheel speed sensor**

* Eagleye uses vehicle speed acquired from CAN bus.

## How to install

### RTKLIB

Clone and Build MapIV's fork of [RTKLIB](https://github.com/MapIV/rtklib_ros_bridge/tree/ros2-v0.1.0). You can find more details about RTKLIB [here](http://www.rtklib.com/).

	sudo apt-get install gfortran
	cd $HOME
	git clone -b rtklib_ros_bridge_b34 https://github.com/MapIV/RTKLIB.git
	cd $HOME/RTKLIB/lib/iers/gcc/
	make
	cd $HOME/RTKLIB/app/consapp
	make

### ROS Packages

Clone and build the necessary packages for Eagleye. ([rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge/tree/ros2-v0.1.0), [nmea_ros_bridge](https://github.com/MapIV/nmea_ros_bridge/tree/ros2-v0.1.0))

	cd $HOME/catkin_ws/src
	git clone https://github.com/MapIV/eagleye.git -b autoware-main
	vcs import . < eagleye/eagleye.repos
	sudo apt-get install -y libgeographic-dev geographiclib-tools geographiclib-doc
	sudo geographiclib-get-geoids best
	sudo mkdir /usr/share/GSIGEO
	sudo cp llh_converter/data/gsigeo2011_ver2_1.asc /usr/share/GSIGEO/
	cd ..
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

## Configuration

### GNSS

#### Real Time Kinematic by Mosaic

1. nmea_ros_bridge settings.

Change `adress` and `port` of `$HOME/ros2_ws/src/nmea_ros_bridge/config/udp_config.yaml` according to the serial device you use.

ie)
>adress: 192.168.30.10
>port: 62001

2. GNSS receiver settings.

Access mosaic's web ui and upload the following file in Admin/Configuration.

https://www.dropbox.com/s/uckt9

### IMU

1. IMU settings.

* Output rate 50Hz

2. Please be careful with the coordinate system when using the [tamagawa ros driver](https://github.com/MapIV/tamagawa_imu_driver) created by MAP IV. If the x-direction indicated on the IMU is set to the vehicle's direction and the y-direction to the right side of the vehicle, please set the `roll` in the `eagleye/eagleye_util/tf/config/sensors_tf.yaml file` to `3.14159`.

		 roll: 3.14159

### Eagleye parameters

The parameters of eagleye can be set in the [eagleye_config.yaml](https://github.com/MapIV/eagleye/tree/ros2-galactic-v1.1.5/eagleye_rt/config/eagleye_config.yaml). The default settings are 5Hz for GNSS and 50Hz for IMU.


The TF between sensors can be set in [sensors_tf.yaml](https://github.com/MapIV/eagleye/tree/ros2-galactic-v1.1.5/eagleye_util/tf/config/sensors_tf.yaml).
The settings are reflected by describing the positional relationship of each sensor with respect to base_link. If you want to change the base frame, [change basic_parent_flame](https://github.com/MapIV/eagleye/tree/ros2-galactic-v1.1.5/eagleye_util/tf/config/sensors_tf.yaml#L2) to reflect the change.


## How to run
### Use sample data

1. Play the sample data.

		ros2 bag play -s rosbag_v2 eagleye_sample.bag

2. Launch eagleye.

		ros2 launch eagleye_rt eagleye_rt.launch.xml

The estimated results will be output about 100 seconds after playing the rosbag. This is because we need to wait for the data to accumulate for estimation.

### Running real-time operation

1. Check if wheel speed (vehicle speed) is published in `/can_twist` topic.

* Topic name: /can_twist
* Message type: geometry_msgs/TwistStamped twist.liner.x or geometry_msgs/TwistWithCovarianceStamped twist.twist.liner.x


2. Check if the IMU data is published in `/imu/data_raw` topic.

3. Start RTKLIB.

 		cd $HOME/RTKLIB
		bash rtklib_ros_bridge.sh

4. Check if RTKLIB is working by execute the following command in the terminal. If the RTKLIB is working correctly, positioning information is appeared continuously in the terminal.

		status 0.1

5. Start rtklib_ros_bridge.

		ros2 run rtklib_bridge rtklib_bridge --ros-args --params-file $HOME/ros2_ws/src/rtklib_ros_bridge/rtklib_bridge/param/param.yaml 

6. Start nmea_ros_bridge.

		ros2 launch nmea_ros_bridge nmea_udp.launch.py

7. Start eagleye.

		ros2 launch eagleye_rt eagleye_rt.launch.xml

## Node
### Subscribed Topics
 - /navsat/nmea_sentence (nmea_msgs/Sentence)

 - /can_twist (geometry_msgs/TwistStamped or geometry_msgs/TwistWithCovarianceStamped)

 - /rtklib_nav (rtklib_msgs/RtklibNav)

 - /imu/data_raw (sensor_msgs/Imu)

### Main Published Topics

 - /eagleye/fix (sensor_msgs/NavSatFix)

 - /eagleye/twist (geometry_msgs/TwistStamped)

 - /eagleye/twist_with_covariance (geometry_msgs/TwistWithCovarianceStamped)


### Note

To visualize the eagleye output location /eagleye/fix, for example, use the following command

	ros2 launch eagleye_fix2kml fix2kml.xml


To convert from eagleye/fix to eagleye/pose, use the following command　

	ros2 launch eagleye_geo_pose_fusion geo_pose_fusion.launch.xml
	ros2 launch eagleye_geo_pose_converter geo_pose_converter.launch.xml

## Sample data
### ROSBAG(ROS1)

| No. | Date | Place | Sensors | Link |
|-----|------|-------|---------| ---- |
|1|2020/01/27|Moriyama, Nagoya<br>[route](https://www.google.com/maps/d/edit?mid=1pK4BgrGtoo14nguArDf-rZDqIL5Cg-v5&usp=sharing)|GNSS: Ublox F9P<br>IMU: Tamagawa AU7684<br>LiDAR: Velodyne HDL-32E|[Download](https://www.dropbox.com/sh/ks5kg8033f5n3w8/AADv9plEjXnwlxex23Z91kR_a?dl=0)|
|2|2020/07/15|Moriyama, Nagoya<br>[route](https://www.google.com/maps/d/edit?mid=1DnXfZBTSsHpWlzTAcENmFxo17r3PxGxM&usp=sharing)|GNSS: Ublox F9P with RTK<br>IMU: Tamagawa AU7684<br>LiDAR: Velodyne VLP-32C|[Download](https://www.dropbox.com/sh/mhdib1m1oivotiu/AAD0UnANDsuIsKqcSKHt9WAJa?dl=0)

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
Eagleye is provided under the [BSD 3-Clause](https://github.com/MapIV/eagleye/tree/ros2-galactic-v1.1.5/LICENSE) License.

## Contacts

If you have further question, email to support@map4.jp .
