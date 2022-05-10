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
    parser.add_argument("-r", "--reverse_imu", action="store_true",help="change reverse_imu true")
    parser.add_argument("-p", "--plane_num",help="Plane Cartesian coordinate system number")
    args= parser.parse_args()
    print(args)
    input_ref_path=sys.argv[1]
    reverse_imu: bool = args.reverse_imu

    plane = 7 # Plane Cartesian coordinate system number (default:7 = 7)

    # set param
    if args.plane_num != None:
        plane = float(args.plane_num)

    print('plane',plane)

    eagleye_df,raw_df = util_prepro.set_log_df(input_ref_path,plane)
    print("set ref_data")

    eagleye_ecef_base = pd.concat([eagleye_df['ecef_base_x'],eagleye_df['ecef_base_y'],eagleye_df['ecef_base_z']],axis=1)
    for i in range(len(eagleye_ecef_base)):
        if not eagleye_ecef_base['ecef_base_x'][i] == 0 and eagleye_ecef_base['ecef_base_y'][i]:
            org_x = eagleye_ecef_base['ecef_base_x'][i]
            org_y = eagleye_ecef_base['ecef_base_y'][i]
            org_z = eagleye_ecef_base['ecef_base_z'][i]
            break
    org_xyz = [org_x,org_y,org_z]

    eagleye_llh = pd.concat([eagleye_df['latitude'],eagleye_df['longitude'],eagleye_df['altitude']],axis=1)
    eagleye_xyz = util_calc.llh2xyz(eagleye_llh)
    eagleye_enu = util_calc.xyz2enu(eagleye_xyz,org_xyz)
    print("calc eagleye enu")

    raw_llh = pd.concat([raw_df['latitude'],raw_df['longitude'],raw_df['altitude']],axis=1)
    raw_xyz = util_calc.llh2xyz(raw_llh)
    raw_enu = util_calc.xyz2enu(raw_xyz,org_xyz)
    print("calc raw enu")

    rtk_llh_tmp = pd.concat([raw_df['rtk_latitude'],raw_df['rtk_longitude'],raw_df['rtk_altitude']],axis=1)
    rtk_llh = rtk_llh_tmp.rename(columns={'rtk_latitude': 'latitude', 'rtk_longitude': 'longitude','rtk_altitude': 'altitude'})
    rtk_xyz = util_calc.llh2xyz(rtk_llh)
    rtk_enu = util_calc.xyz2enu(rtk_xyz,org_xyz)
    print("calc rtk enu")

    raw_xyz_vel = pd.concat([raw_df['vel_x'],raw_df['vel_y'],raw_df['vel_z']],axis=1)
    vel = util_calc.xyz2enu_vel(raw_xyz_vel,org_xyz)
    dopplor = pd.concat([raw_df['elapsed_time'],vel],axis=1)

    eagleye_rpy = pd.concat([eagleye_df['heading_1st'],eagleye_df['heading_2nd'],eagleye_df['yaw_rad'],eagleye_df['roll_rad'],eagleye_df['pitch_rad']],axis=1)
    eagleye_df_tmp = util_calc.get_heading_deg(eagleye_rpy)
    eagleye_df['heading_1st_deg'] = eagleye_df_tmp['heading_1st_deg']
    eagleye_df['heading_2nd_deg'] = eagleye_df_tmp['heading_2nd_deg']
    eagleye_df['heading_3rd_deg'] = eagleye_df_tmp['heading_3rd_deg']
    eagleye_df['heading'] = eagleye_df_tmp['heading_3rd_deg']
    eagleye_df['roll'] = eagleye_df_tmp['roll']
    eagleye_df['pitch'] = eagleye_df_tmp['pitch']

    eagleye_plot_rpy = pd.concat([eagleye_df['roll'],eagleye_df['pitch'],eagleye_df['heading']],axis=1)
    util_plot.plot_6DoF_single(eagleye_df['elapsed_time'], eagleye_enu, rtk_enu, raw_enu, eagleye_plot_rpy)

    fig2 = plt.figure()
    ax_sf = fig2.add_subplot(2, 1, 1)
    util_plot.plot_one(ax_sf, eagleye_df, 'elapsed_time', 'sf', 'Velocity scal factor', 'Time [s]', 'Velocity scal factor []', "None")

    ax_vel = fig2.add_subplot(2, 1, 2)
    ax_vel.set_title('Velocity')
    ax_vel.plot(dopplor['elapsed_time'] , dopplor['velocity'] ,  marker="s",linestyle="None",markersize=1 , color = "green",  label="dopplor")
    ax_vel.plot(raw_df['elapsed_time'] , raw_df['velocity'] , marker=".",linestyle="None",markersize=1, color = "red",  label="can velocity")
    ax_vel.plot(eagleye_df['elapsed_time'] , eagleye_df['velocity'] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
    ax_vel.set_xlabel('Time [s]')
    ax_vel.set_ylabel('Velocity [m/s]')
    ax_vel.legend(loc='upper right')
    ax_vel.grid()

    util_plot.plot_traj_three(raw_enu, rtk_enu, eagleye_enu, "gnss rtk data(nmea)")

    util_plot.plot_traj_qual(eagleye_enu,raw_df['qual'])

    util_plot.plot_traj_3d_three(raw_enu, rtk_enu, eagleye_enu, "gnss rtk data(nmea)")

    plt.show()
    