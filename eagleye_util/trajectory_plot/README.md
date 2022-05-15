# eagleye_evaluation

## Set up
```
pip3 install scipy
pip3 install open3d
```

## Usage
### eagleye_pp_single_evaluation
```
python3 eagleye_pp_single_evaluation.py <eagleye_log.csv>
```

ex)
```
python3 scripts/eagleye_pp_single_evaluation.py sample_data/eagleye_log.csv
```

### evaluation_plot
```
python3 scripts/evaluation_plot.py [ref_data_input_option] <ref_data.csv> [eagleye_data_input_option] <eagleye_data.csv>
```

ex1)(recommendation)
```
python3 scripts/evaluation_plot.py [-df_ref] <ref_data.csv> [-log] <eagleye_log.csv>
```

ex2)(geometry_msgs/PoseStamped)
```
python3 scripts/evaluation_plot.py [-df_ref_ros] <ref_data.csv> [-df_csv] <eagleye.csv>
```

ref data input options
* [`-ref`]: Import reference data referenced by column count
* [`-df_ref`]: Import reference data by reference with the name of the HEADER(POSLV csv)
* [`-df_ref_ros`]: Import reference data of type geometry_msgs/PoseStamped with reference to the name of the HEADER

eagleye data input options
* [`-csv`]: Import eagleye data referenced by column count
* [`-df_csv`]: Import eagleye data by reference with the name of the HEADER(/eagleye/pose)
* [`-log`]: Import eagleye_pp data by reference with the name of the HEADER(eagleye_log.csv)
* [`-p`]: Plane Cartesian coordinate system number

Time synchronization options
* [`-s`]: Gap time to be regarded as synchronous[s]
* [`-l`]: Offset time between reference data and eagleye data[s]

Position correction options for reference and eagleye data
* [`-tf_x`]: Correction amount in East-West direction[m]
* [`-tf_y`]: Correction amount in North-South direction[m]
* [`-tf_across`]: Correction amount in vehicle orthogonal direction[m]
* [`-tf_along`]: Correction amount in the direction of vehicle travel[m]
* [`-tf_height`]: Correction amount in height direction[m]
* [`-tf_yaw`]: Correction amount in yaw[deg]

Options for relative positional accuracy evaluation
* [`-dr_l`]: Length of distance to evaluate relative position[m]
* [`-dr_s`]: Start distance interval to evaluate relative position[m]

plot options
* [`-ref_name`]: Name of reference data in graph legend
