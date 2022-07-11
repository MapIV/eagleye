# eagleye_pp

## How to use

		./src/eagleye_pp/scripts/launch_pp.sh /home/user_name/rosbag/eagleye/eagleye_sample.bag src/eagleye_pp/eagleye_pp/config/eagleye_pp_config.yaml 

if multiple rosbags  

		./src/eagleye_pp/scripts/launch_pp.sh <BAG_0> ... <BAG_N> src/eagleye_pp/eagleye_pp/config/eagleye_pp_config.yaml   

## IO

### Input
The input topics can be changed in `config/eagleye_pp.config`.

 - /can_twist (geometry_msgs/TwistStamped)

 - /imu/data_raw (sensor_msgs/Imu)

 - /rtklib_nav (rtklib_msgs/RtklibNav)

 - /navsat/fix (sensor_msgs/NavSatFix) or /navsat/nmea_sentence (nmea_msgs/Sentence)

### Output
The output file will be created in the log directory, which is created in the same directory as the rosbag used.

 - eagleye.csv  
	Position and orientation file.  
	eagleye_pp_llh.status indicates the status of the result: status 0 is 2D error less than 0.3m, status 1 is 2D error less than 1.5m, and status 2 is 2D error greater than 1.5m.  
	eagleye_pp_llh.height_status indicates whether flag is true or false for pitching estimation.  
 - eagleye.kml  
	・RTKLIB section in the CAR Tragectory  
	  The magenta point plot is the positioning solution for rtklib.  
	  The square plot is the fix solution, and the circle plot is the float solution or the single positioning solution.  
	・GNSS section in the CAR Tragector  
	  The green point plots are positioning solutions from navsat's internal engine.  
      Square plots are fix solutions, round plots are float solutions or single positioning solutions.  
	・EAGLEYE_FORWARD_LINE  
	  Red line plot of the trajectory of the eagleye turned in the forward direction in time.  
	・EAGLEYE_BACKWARD_LINE  
	  Green line plot of the trajectory of the eagleye turned in the opposite direction in time.  
	・EAGLEYE_PP_LINE  
	  Blue line plot of a trajectory that combines Forward and Backward processing.
 - eagleye_line.kml  
 　Only line plots are extracted from the plots in eagleye.kml.  
 - eagleye_log.csv  
 　Log files of observables and internal states in forward processing.  
 - eagleye_log_back.csv  
 　Log files of observables and internal states in backward processing. 

## Parameter
The parameters are set in `config/eagleye_pp.config` .

## Tool
- [status_checker](common/status_checker/README.md)：This tool displays the measured time, the cumulative distance traveled, and the percentage of FIX solutions.
