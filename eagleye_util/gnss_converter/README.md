gnss_converter
==========

A ros package that converts gnss nmea messages to navsatfix messages


# Launch

~~~
source $HOME/ros2_ws/devel/setup.bash
ros2 launch eagleye_gnss_converter gnss_converter.xml
~~~

# Node

## Subscribed Topics
 - /nmea_sentence (nmea_msgs/Sentence)

## Published Topics

 - /fix (sensor_msgs/NavSatFix)

 - /gga (nmea_msgs/Gpgga)

 - /rmc (nmea_msgs/Grmc)


# Parameter description

The parameters are set in `launch/gnss_converter.xml` .

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
<<<<<<< HEAD
|nmea_sentence_topic|bool|Topic name of nmea_msgs/Sentence to subscribe|/navsat/nmea_sentence|
|pub_fix_topic_name|double|Topic name of sensor_msgs/NavSatFix to publish|/navsat/fix|
|pub_gga_topic_name|bool|Topic name of nmea_msgs/Gpgga to publish|gnss/gga|
|pub_rmc_topic_name|bool|Topic name of nmea_msgs/Gprmc to publish|gnss/rmc|
|output_gga|bool|Whether to output nmea_msgs/Gpgga|false|
|output_rmc|bool|Whether to output nmea_msgs/Gprmc|false|
=======
|nmea_sentence_topic|bool|Topic name of nmea_msgs/Sentence to subscribe|/nmea_sentence|
>>>>>>> 47c9d0b83db1524e6347f2c39333881242678d6b
