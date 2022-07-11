# eagleye_rt
## How to run 

### Eagleye RT parameters

The parameters of eagleye_rt can be set in the [eagleye_config.yaml](https://github.com/MapIV/eagleye/blob/master/eagleye_rt/config/eagleye_config.yaml). The default settings are 5Hz for GNSS and 50Hz for IMU.


The TF between sensors can be set in [sensors_tf.yaml](https://github.com/MapIV/eagleye/blob/master/eagleye_util/tf/config/sensors_tf.yaml).
The settings are reflected by describing the positional relationship of each sensor with respect to base_link. If you want to change the base frame, [change basic_parent_frame](https://github.com/MapIV/eagleye/blob/master/eagleye_util/tf/config/sensors_tf.yaml#L2) to reflect the change.

### Running real-time operation


1. Start each sensor driver

2. Start eagleye.

		roslaunch eagleye_rt eagleye_rt.launch

### Use sample data

1. Play the sample data.  

		rosparam set use_sim_time true
		rosbag play --clock eagleye_sample.bag

2. Launch eagleye.  

		roslaunch eagleye_rt eagleye_rt.launch

The estimated results will be output about 100 seconds after playing the rosbag. This is because we need to wait for the data to accumulate for estimation.

## Node
### Subscribed Topics
 - /nmea_sentence (nmea_msgs/Sentence)

 - /can_twist (geometry_msgs/TwistStamped)

 - /rtklib_nav (rtklib_msgs/RtklibNav)

 - /imu/data_raw (sensor_msgs/Imu)

### Main Published Topics

 - /eagleye/fix (sensor_msgs/NavSatFix) 

 - /eagleye/twist (ngeometry_msgs/TwistStamped)


## Note

To visualize the eagleye output location /eagleye/fix, for example, use the following command  

	roslaunch eagleye_fix2kml fix2kml.launch


To convert from eagleye/fix to eagleye/pose, use the following command　

	roslaunch eagleye_fix2pose fix2pose.launch

