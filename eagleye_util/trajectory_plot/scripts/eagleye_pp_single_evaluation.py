# Copyright (c) 2022, Map IV, Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the Map IV, Inc. nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# eagleye_pp_single_evaluation.py
# Author MapIV Hoda

import argparse
import yaml
from re import X
from typing import List
import pandas as pd
import sys
import matplotlib.pyplot as plt

import util.preprocess as util_prepro
import util.calc as util_calc
import util.plot as util_plot

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", action="store")
    parser.add_argument("input_yaml", action="store")
    args= parser.parse_args()
    print(args)
    input_ref_path=sys.argv[1]
    yaml_path=sys.argv[2]

    yaml_path_str: str = yaml_path
    with open(yaml_path_str,"r") as yml:
        config = yaml.safe_load(yml)

    # set param
    missing_gnss_type = config["eagleye_log"]["missing_gnss_type"]
    reverse_imu = config["param"]["reverse_imu_flag"]
    plane = config["param"]["plane_num"]
    plot_text_data = config["evaluation_plot"]["plot_text_data"]
    plot_text_step = config["evaluation_plot"]["plot_text_step"]
    data_name = config["param"]["data_name_param"]
    font_size = config["param"]["font_size_param"]

    print('plane',plane)

    eagleye_df,raw_df = util_prepro.set_log_df(input_ref_path,plane,config)
    print("set eagleye_data")

    eagleye_ecef_base = pd.concat([eagleye_df['ecef_base_x'],eagleye_df['ecef_base_y'],eagleye_df['ecef_base_z']],axis=1)
    for i in range(len(eagleye_ecef_base)):
        if not eagleye_ecef_base['ecef_base_x'][i] == 0 and not eagleye_ecef_base['ecef_base_y'][i] == 0:
            org_x = eagleye_ecef_base['ecef_base_x'][i]
            org_y = eagleye_ecef_base['ecef_base_y'][i]
            org_z = eagleye_ecef_base['ecef_base_z'][i]
            org_xyz = [org_x,org_y,org_z]
            break

    eagleye_xyz = pd.concat([eagleye_df['x'],eagleye_df['y'],eagleye_df['z']],axis=1)

    raw_llh = pd.concat([raw_df['latitude'],raw_df['longitude'],raw_df['altitude']],axis=1)
    raw_xyz = util_prepro.latlon_to_19(raw_llh,plane)

    rtk_llh_tmp = pd.concat([raw_df['rtk_latitude'],raw_df['rtk_longitude'],raw_df['rtk_altitude']],axis=1)
    rtk_llh = rtk_llh_tmp.rename(columns={'rtk_latitude': 'latitude', 'rtk_longitude': 'longitude','rtk_altitude': 'altitude'})
    rtk_xyz = util_prepro.latlon_to_19(rtk_llh,plane)

    raw_xyz_vel = pd.concat([raw_df['vel_x'],raw_df['vel_y'],raw_df['vel_z']],axis=1)
    vel = util_calc.xyz2enu_vel(raw_xyz_vel,org_xyz)
    dopplor_heading = util_calc.calc_dopplor_heading(vel)
    dopplor = pd.concat([raw_df['elapsed_time'],vel, dopplor_heading],axis=1)

    eagleye_plot_rpy = pd.concat([eagleye_df['roll'],eagleye_df['pitch'],eagleye_df['yaw']],axis=1)
    eagleye_6dof = pd.concat([eagleye_xyz,eagleye_plot_rpy],axis=1)
    raw_6dof = pd.concat([raw_xyz,dopplor_heading],axis=1)

    if missing_gnss_type == 0:
        util_plot.plot_6DoF_single(eagleye_df['elapsed_time'],raw_df['elapsed_time'],raw_df['elapsed_time'], eagleye_xyz, rtk_xyz, raw_xyz, eagleye_plot_rpy,dopplor, font_size)
        util_plot.plot_traj_text_tree('2D Trajectory', raw_xyz, rtk_xyz, eagleye_xyz, eagleye_df[plot_text_data], plot_text_step, font_size, data_name, "gnss rtk data(nmea)")
        util_plot.plot_traj_qual(eagleye_xyz,eagleye_df['qual'], eagleye_df[plot_text_data], plot_text_step, font_size)
        #util_plot.plot_traj_3d_three(raw_xyz, rtk_xyz, eagleye_xyz, font_size, data_name, "gnss rtk data(nmea)")
    elif missing_gnss_type == 1:
        util_plot.plot_6DoF(eagleye_df['elapsed_time'], eagleye_6dof, rtk_xyz , data_name, "gnss rtk data(nmea)",font_size)
        util_plot.plot_traj_text('2D Trajectory', rtk_xyz, eagleye_xyz, eagleye_df[plot_text_data], plot_text_step, font_size, data_name, "gnss rtk data(nmea)")
        util_plot.plot_traj_qual(rtk_xyz,eagleye_df['qual'], eagleye_df[plot_text_data], plot_text_step, font_size)
        #util_plot.plot_traj_3d(rtk_xyz, eagleye_xyz, font_size, data_name, "gnss rtk data(nmea)")
    elif missing_gnss_type == 2:
        util_plot.plot_6DoF(eagleye_df['elapsed_time'], eagleye_6dof, raw_6dof , data_name, "gnss raw data(rtklib)",font_size)
        util_plot.plot_traj_text('2D Trajectory', rtk_xyz, eagleye_xyz, eagleye_df[plot_text_data], plot_text_step, font_size, data_name, "gnss raw data(rtklib)")
        #util_plot.plot_traj_3d(raw_xyz, eagleye_xyz, font_size, data_name, "gnss raw data(rtklib)")

    fig2 = plt.figure()
    ax_sf = fig2.add_subplot(2, 1, 1)
    util_plot.plot_one(ax_sf, eagleye_df, 'elapsed_time', 'sf', 'Velocity scal factor', 'Time [s]', 'Velocity scal factor []', "None", 1, font_size)

    ax_vel = fig2.add_subplot(2, 1, 2)
    ax_vel.set_title('Velocity')
    ax_vel.plot(dopplor['elapsed_time'] , dopplor['velocity'] ,  marker="s",linestyle="None",markersize=1 , color = "green",  label="dopplor")
    ax_vel.plot(raw_df['elapsed_time'] , raw_df['velocity'] , marker=".",linestyle="None",markersize=1, color = "red",  label="can velocity")
    ax_vel.plot(eagleye_df['elapsed_time'] , eagleye_df['velocity'] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
    ax_vel.set_xlabel('Time [s]')
    ax_vel.set_ylabel('Velocity [m/s]')
    ax_vel.legend(loc='upper right')
    ax_vel.grid()

    plt.show()
    
