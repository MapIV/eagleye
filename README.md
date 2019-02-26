imu_gnss_localizer

Overview------------------------------------------------------------------------

It is a program that combines imu and GNSS to perform highly accurate self position estimation.

Required input------------------------------------------------------------------

IMU(Recommended IMU Analog Devices adis16475)

topicname   /imu/data_raw

message     sensor_msgs/Imu

GNSS(Required GNSS receiver ublox M8 M9 Series)

topicname   /ublox_gps/navpvt

message     ublox_msgs/NavPVT7

Wheel speed(Vehicle speed information from CAN etc.)

topicname   /Vehicle/Velocity

message     geometry_msgs/Twist (linear.x)
