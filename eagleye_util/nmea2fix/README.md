nmea2fix
==========

A ros package that converts gnss nmea messages to navsatfix messages


# Launch

~~~
source $HOME/catkin_ws/devel/setup.bash
roslaunch nmea2fix nmea2fix.launch
~~~

# Node

## Subscribed Topics
 - /f9p/nmea_sentence (nmea_msgs/Sentence)

## Published Topics

 - /navsat/fix (sensor_msgs/NavSatFix) (This topic will not be published unless its location has been estimated.)

 - /gga (nmea_msgs/Gpgga)


# Parameter description

The parameters are set in `launch/nmea2fix.launch` .

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|sub_topic_name|bool|Topic name of nmea_msgs/Gpgga to subscribe|/f9p/nmea_sentence|
|pub_fix_topic_name|double|Topic name of nmea_msgs/Gpgga to publish|/navsat/fix|
|pub_gga_topic_name|bool|Topic name of nmea_msgs/Gpgga to publish|/gga|
|output_gga|bool|Whether to output nmea_msgs/Gpgga|false|