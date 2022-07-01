# eagleye_evaluation

## Set up
```
pip3 install scipy
pip3 install open3d
pip3 install mgrs
```

## Usage
### eagleye_pp_single_evaluation
```
python3 eagleye_pp_single_evaluation.py <eagleye_log.csv> <trajectory_plot.yaml>
```

ex)
```
python3 scripts/eagleye_pp_single_evaluation.py eagleye_log.csv trajectory_plot.yaml
```
Output figures
* 6DoF
* Velocity scale factor/Velocity
* 2D trajectory
* GNSS position solution
* 3D trajectory

### evaluation_plot
```
python3 scripts/evaluation_plot.py [ref_data_input_option] <ref_data.csv> [target data input option] <target.csv> -yaml <trajectory_plot.yaml>
```

ex1)(evaluation eagleye_log.csv)
```
python3 scripts/evaluation_plot.py [-ref] <ref_data.csv> [-log] <eagleye_log.csv> -yaml <trajectory_plot.yaml>
```
Output figures
* 6DoF
* 6DoF Error
* X Error/Y Error/Across Error/Along Error
* X-Y Error/Across-Along Error
* Cumulative Error Distribution
* Cumulative Error Distribution(relative position)
* DR trajectory
* Velocity/Velocity Error
* 2D trajectory
* GNSS position solution
* 3D trajectory


ex2)(evaluation target.csv)
```
python3 scripts/evaluation_plot.py [-ref] <ref_data.csv> [-target] <target.csv> -yaml <trajectory_plot.yaml>
```
Output figures
* 6DoF
* 6DoF Error
* X Error/Y Error/Across Error/Along Error
* X-Y Error/Across-Along Error
* Cumulative Error Distribution
* Velocity
* 2D trajectory
* 3D trajectory

### twist_evaluation
```
python3 scripts/twist_evaluation.py [ref_data_input_option] <ref_data.csv> [twist data input option] <twist.csv> -yaml <trajectory_plot.yaml>
```

ex1)(evaluation eagleye_log.csv)
```
python3 scripts/twist_evaluation.py [-ref] <ref_data.csv> [-log] <eagleye_log.csv> -yaml <trajectory_plot.yaml>
```
Output figures
* Cumulative Error Distribution(relative position)
* DR trajectory

ex2)(evaluation twist.csv)
```
python3 scripts/evaluation_plot.py [-ref] <ref_data.csv> [-twist] <twist.csv> -yaml <trajectory_plot.yaml>
```
Output figures
* Cumulative Error Distribution(relative position)
* DR trajectory

#### Options
ref data input options
* [`-ref`]: Import reference data referenced by column count
* [`-ref_log`]: Import eagleye_pp data by reference with the name of the HEADER(eagleye_log.csv)

target data input options
* [`-log`]: Import eagleye_pp data by reference with the name of the HEADER(eagleye_log.csv)
* [`-target`]: Import target data referenced by column count

twist data input options
* [`-log`]: Import eagleye_pp data by reference with the name of the HEADER(eagleye_log.csv)
* [`-twist`]: Import target twist data referenced by column count

Common Options
* [`-yaml`]: Import yaml file

