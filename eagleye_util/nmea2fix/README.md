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
 - /nmea_sentence (nmea_msgs/Sentence)

## Published Topics

 - /fix (sensor_msgs/NavSatFix)

 - /gga (nmea_msgs/Gpgga)

 - /rmc (nmea_msgs/Grmc)


# Parameter description

The parameters are set in `launch/nmea2fix.launch` .

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|nmea_sentence_topic|bool|Topic name of nmea_msgs/Sentence to subscribe|/nmea_sentence|