# eagleye_rt
## How to run 

## Eagleye RT parameters

The parameters of eagleye_rt can be set in the [eagleye_config.yaml](https://github.com/MapIV/eagleye/blob/master/eagleye_rt/config/eagleye_config.yaml). The default settings are 5Hz for GNSS and 50Hz for IMU.


The TF between sensors can be set in [sensors_tf.yaml](https://github.com/MapIV/eagleye/blob/master/eagleye_util/tf/config/sensors_tf.yaml).
The settings are reflected by describing the positional relationship of each sensor with respect to base_link. If you want to change the base frame, [change basic_parent_flame](https://github.com/MapIV/eagleye/blob/master/eagleye_util/tf/config/sensors_tf.yaml#L2) to reflect the change.


## Use sample data

1. Play the sample data.  

		rosparam set use_sim_time true
		rosbag play --clock eagleye_sample.bag

2. Launch eagleye.  

		roslaunch eagleye_rt eagleye_rt.launch

The estimated results will be output about 100 seconds after playing the rosbag. This is because we need to wait for the data to accumulate for estimation.

## Running real-time operation

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

## Note

To visualize the eagleye output location /eagleye/fix, for example, use the following command  

	roslaunch eagleye_fix2kml fix2kml.launch
