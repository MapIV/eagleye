Header header

#IMU raw data
sensor_msgs/Imu imu

#RTKLIB data
rtklib_msgs/RtklibNav rtklib_nav
sensor_msgs/NavSatFix fix

#CAN velocity data
geometry_msgs/TwistStamped velocity

#Velocity scale factor
eagleye_msgs/VelocityScaleFactor velocity_scale_factor

#Distance
eagleye_msgs/Distance distance

#Heading 1st
eagleye_msgs/Heading heading_1st

#Heading interpolate 1st
eagleye_msgs/Heading heading_interpolate_1st

#Heading 2nd
eagleye_msgs/Heading heading_2nd

#Heading interpolate 2nd
eagleye_msgs/Heading heading_interpolate_2nd

#Heading 3rd
eagleye_msgs/Heading heading_3rd

#Heading interpolate 3rd
eagleye_msgs/Heading heading_interpolate_3rd

#Yawrate offset stop
eagleye_msgs/YawrateOffset yawrate_offset_stop

#Yawrate offset 1st
eagleye_msgs/YawrateOffset yawrate_offset_1st

#Yawrate offset 2nd
eagleye_msgs/YawrateOffset yawrate_offset_2nd

#Slip angle
eagleye_msgs/SlipAngle slip_angle

#Trajectory
eagleye_msgs/Position enu_relative_pos
geometry_msgs/Vector3Stamped enu_vel

#Position
eagleye_msgs/Position enu_absolute_pos

#Position interpolate
eagleye_msgs/Position enu_absolute_pos_interpolate

#Eagleye fix
sensor_msgs/NavSatFix eagleye_fix

#Eagleye twist
geometry_msgs/TwistStamped eagleye_twist
