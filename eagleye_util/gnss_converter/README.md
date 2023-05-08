gnss_converter
==========

A ros package that converts gnss nmea messages to navsatfix messages


# Launch

~~~
source $HOME/ros2_ws/devel/setup.bash
ros2 launch eagleye_gnss_converter gnss_converter.launch.xml
~~~

# Node

## Subscribed Topics
 - /nmea_sentence (nmea_msgs/Sentence)

## Published Topics

 - /fix (sensor_msgs/NavSatFix)

 - /gga (nmea_msgs/Gpgga)

 - /rmc (nmea_msgs/Grmc)


# Parameter description

The parameters are set in `launch/gnss_converter.launch.xml` .

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|use_multi_antenna_mode|bool|It enables the second gnss_converter node to start when set to true|
