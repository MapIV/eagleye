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

# twist_evaluation.py
# Author MapIV Hoda

import argparse
import math
import yaml
from typing import List
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.ticker

import util.preprocess as util_prepro
import util.calc as util_calc
import util.plot as util_plot

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-ref", "--ref", help="Path to the ref data(POSLV csv)")
    parser.add_argument("-ref_twist", "--ref_twist", help="Path to the ref twist data")
    parser.add_argument("-df_ref_ros", "--input_df_ref", help="Path to the ref data by df")
    parser.add_argument("-twist", "--input_twist", help="Path to the twist csv")
    parser.add_argument("-df_csv", "--input_df_csv", help="Path to the csv by df")
    parser.add_argument("-log", "--input_log_csv", help="Path to the csv by df")
    parser.add_argument("-ref_log", "--input_ref_log", help="Path to the csv by df")
    parser.add_argument("-y", "--yaml_path",help="yaml name")
    args = parser.parse_args()

    yaml_path_str: str = args.yaml_path
    with open(yaml_path_str,"r") as yml:
        config = yaml.safe_load(yml)

    # set param
    reverse_imu = config["param"]["reverse_imu_flag"]
    plane = config["param"]["plane_num"]
    sync_threshold_time = config["param"]["sync_threshold_time_data_param"]
    leap_time = config["param"]["leap_time_param"]
    tf_x = config["param"]["tf_x_param"]
    tf_y = config["param"]["tf_y_param"]
    tf_across = config["param"]["tf_across_param"]
    tf_along = config["param"]["tf_along_param"]
    tf_height = config["param"]["tf_height_param"]
    tf_yaw = config["param"]["tf_yaw_param"]
    distance_length = config["param"]["distance_length_param"]
    distance_step = config["param"]["distance_step_param"]
    ref_data_name = config["param"]["ref_data_name_param"]
    eval_step_max = config["param"]["eval_step_max_param"]
    
    print('plane',plane)
    print('sync_threshold_time',sync_threshold_time)
    print('leap_time',leap_time)
    print('tf_x',tf_x)
    print('tf_y',tf_y)
    print('tf_across',tf_across)
    print('tf_along',tf_along)
    print('tf_height',tf_height)
    print('tf_yaw',tf_yaw)
    print('reverse_imu',reverse_imu)
    print('distance_length',distance_length)
    print('distance_step',distance_step)
    print('ref_data_name',ref_data_name)

    # set data
    if args.ref != None:
        ref_data_df = util_prepro.set_ref_data(args.ref,config)
        print("set ref_data")

    if args.input_twist != None:
        twist_data_df = util_prepro.set_twist_data(args.input_twist,config)
        print("set csv_data")
        
    if args.ref_twist != None:
        ref_data_df = util_prepro.set_twist_data(args.ref_twist,config)
        print("set ref_data")

    if args.input_ref_log != None:
        ref_data_df, ref_raw_df = util_prepro.set_log_df(args.input_ref_log,plane,config)
        eagleye_ecef_base = pd.concat([ref_data_df['ecef_base_x'],ref_data_df['ecef_base_y'],ref_data_df['ecef_base_z']],axis=1)
        for i in range(len(eagleye_ecef_base)):
            if not eagleye_ecef_base['ecef_base_x'][i] == 0 and not eagleye_ecef_base['ecef_base_y'][i] == 0:
                org_x = eagleye_ecef_base['ecef_base_x'][i]
                org_y = eagleye_ecef_base['ecef_base_y'][i]
                org_z = eagleye_ecef_base['ecef_base_z'][i]
                org_xyz = [org_x,org_y,org_z]
                break
        raw_xyz_vel = pd.concat([ref_raw_df['vel_x'],ref_raw_df['vel_y'],ref_raw_df['vel_z']],axis=1)
        vel = util_calc.xyz2enu_vel(raw_xyz_vel,org_xyz)
        dopplor = pd.concat([ref_raw_df['elapsed_time'],vel],axis=1)
        print("set ref_data")

    if args.input_log_csv != None:
        csv_data_df, raw_df = util_prepro.set_log_df(args.input_log_csv,plane,config)
        eagleye_ecef_base = pd.concat([csv_data_df['ecef_base_x'],csv_data_df['ecef_base_y'],csv_data_df['ecef_base_z']],axis=1)
        for i in range(len(eagleye_ecef_base)):
            if not eagleye_ecef_base['ecef_base_x'][i] == 0 and not eagleye_ecef_base['ecef_base_y'][i] == 0:
                org_x = eagleye_ecef_base['ecef_base_x'][i]
                org_y = eagleye_ecef_base['ecef_base_y'][i]
                org_z = eagleye_ecef_base['ecef_base_z'][i]
                org_xyz = [org_x,org_y,org_z]
                break
        raw_xyz_vel = pd.concat([raw_df['vel_x'],raw_df['vel_y'],raw_df['vel_z']],axis=1)
        vel = util_calc.xyz2enu_vel(raw_xyz_vel,org_xyz)
        dopplor = pd.concat([raw_df['elapsed_time'],vel],axis=1)
        print("set csv_data")
    
    # syne time
    print("start sync_time")
    ref_df , data_df = util_calc.sync_time(ref_data_df,csv_data_df,sync_threshold_time,leap_time)
    print("finished sync_time")

    if reverse_imu == True:
        data_df['angular_z'] = -1 * data_df['angular_z']
        data_df['yaw'] = -1 * data_df['yaw']
        data_df['yawrate_offset_stop'] = -1 * data_df['yawrate_offset_stop']
        data_df['yawrate_offset'] = -1 * data_df['yawrate_offset']
        data_df['slip'] = -1 * data_df['slip']
    if not tf_yaw == 0:
        data_df['yaw'] = data_df['yaw'] + tf_yaw

    error_velocity = pd.DataFrame()
    if 'velocity' in data_df.columns and 'velocity' in ref_df.columns:
        fig1 = plt.figure()
        ax_vel = fig1.add_subplot(2, 1, 1)
        util_plot.plot_each(ax_vel, ref_df['elapsed_time'], data_df, ref_df, 'velocity', 'Velocity', 'Velocity [m/s]',ref_data_name)

        error_velocity = util_calc.calc_velocity_error(data_df['velocity'],ref_df['velocity'])
        error_plot_velocity = pd.concat([ref_df['elapsed_time'],error_velocity],axis=1)
        ax_err_vel = fig1.add_subplot(2, 1, 2)
        fig1.suptitle(ref_data_name + ' - eagleye Error')
        util_plot.plot_one(ax_err_vel, error_plot_velocity, 'elapsed_time', 'velocity', 'Velocity Error', 'time [s]', 'Velocity error[m/s]', 'None', 1)

    elif 'velocity' in ref_df.columns or 'velocity' in data_df.columns:
        fig11 = plt.figure()
        ax_vel = fig11.add_subplot(1, 1, 1)
        util_plot.plot_each(ax_vel, ref_df['elapsed_time'], data_df, ref_df, 'velocity', 'Velocity', 'Velocity [m/s]',ref_data_name)
    elif 'velocity' in data_df.columns and 'velocity' in ref_df.columns and args.input_ref_log != None or args.input_log_csv != None:
        fig1 = plt.figure()
        ax_vel = fig1.add_subplot(2, 1, 1)
        ax_vel.set_title('Velocity')
        ax_vel.plot(dopplor['elapsed_time'] , dopplor['velocity'] ,  marker="s",linestyle="None",markersize=1 , color = "brack",  label="dopplor")
        ax_vel.plot(raw_df['elapsed_time'] , raw_df['velocity'] , marker=".",linestyle="None",markersize=1, color = "red",  label="can velocity")
        ax_vel.plot(data_df['elapsed_time'] , data_df['velocity'] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
        ax_vel.plot(ref_df['elapsed_time'] , ref_df['velocity'] ,  marker="s",linestyle="None",markersize=1 , color = "green",  label=ref_data_name)
        ax_vel.set_xlabel('Time [s]')
        ax_vel.set_ylabel('Velocity [m/s]')
        ax_vel.legend(loc='upper right')
        ax_vel.grid() 

        eagleye_error_velocity = util_calc.calc_velocity_error(data_df['velocity'],ref_df['velocity'])
        raw_error_velocity = util_calc.calc_velocity_error(raw_df['velocity'],ref_df['velocity'])
        ax_err_vel = fig1.add_subplot(2, 1, 2)
        fig1.suptitle(ref_data_name + ' - eagleye Error')
        util_plot.plot_two(ax_err_vel, raw_error_velocity, eagleye_error_velocity, 'elapsed_time', 'velocity', 'Velocity Error', 'time [s]', 'Velocity error[m/s]', 'None', "can twist")


    if 'yaw' in ref_df.columns and 'x' in ref_df.columns and 'y' in ref_df.columns and 'velocity' in ref_df.columns:
        print("calc dr")
        ref_xyz = pd.concat([ref_df['x'],ref_df['y']],axis=1)
        if 'angular_z' in data_df and 'angular_z' in ref_df:
            twist_data = pd.concat([data_df['angular_z'],data_df['velocity']],axis=1)
            ref_twist = pd.concat([ref_df['angular_z'],ref_df['velocity']],axis=1)
            eagleye_angular_velocity, eagleye_angular, eagleye_velocity, ref_enu_vel = util_calc.calc_enu_vel(ref_df["TimeStamp"],twist_data,ref_twist,np.deg2rad(ref_df["yaw"]))
            
        elif 'yawrate' in raw_df and 'angular_z' in ref_df and 'yawrate_offset_stop' in data_df and'yawrate_offset' in data_df and 'slip' in data_df and 'distance' in ref_df:
            ref_twist = pd.concat([ref_df['angular_z'],ref_df['velocity']],axis=1)
            # eagleye_angular_velocity, heading_angular_velocity = util_calc.calc_enu_vel_eagleye(ref_df["TimeStamp"],raw_df['yawrate'],data_df['yawrate_offset_stop'],data_df['yawrate_offset'],data_df['slip'],data_df['velocity'],ref_twist['velocity'],np.deg2rad(ref_df["yaw"]))
            eagleye_angular, heading_angular = util_calc.calc_enu_vel_eagleye(ref_df["TimeStamp"],raw_df['yawrate'],data_df['yawrate_offset_stop'],data_df['yawrate_offset'],data_df['slip'],ref_twist['velocity'],ref_twist['velocity'],np.deg2rad(ref_df["yaw"]))
            if 'angular_z' in ref_twist.columns: # angular: ref , velocity: twist
                twist_velocity = util_calc.calc_enu_vel_each(ref_df["TimeStamp"],ref_twist["angular_z"],data_df["velocity"],ref_twist["velocity"],np.deg2rad(ref_df["yaw"]))
            
            if 'angular_z' in ref_twist.columns: # angular: ref , velocity: ref
                ref_enu_vel = util_calc.calc_enu_vel_each(ref_df["TimeStamp"],ref_twist["angular_z"],ref_twist["velocity"],ref_twist["velocity"],np.deg2rad(ref_df["yaw"]))

        eagleye_angular_velocity_error = util_calc.calc_dr(ref_df["TimeStamp"],ref_df["distance"],eagleye_angular_velocity,ref_xyz,distance_length,distance_step)
        eagleye_angular_error = util_calc.calc_dr(ref_df["TimeStamp"],ref_df["distance"],eagleye_angular,ref_xyz,distance_length,distance_step)
        eagleye_velocity_error = util_calc.calc_dr(ref_df["TimeStamp"],ref_df["distance"],eagleye_velocity,ref_xyz,distance_length,distance_step)

        fig2 = plt.figure()
        ax_dr = fig2.add_subplot(2, 1, 1)
        util_plot.plot_one(ax_dr, eagleye_angular_velocity_error, 'start_distance', 'error_2d', 'relative position Error', 'start distance [m]', '2D Error [m]', '-', 1)
        util_plot.plot_one(ax_dr, eagleye_angular_error, 'start_distance', 'error_2d', 'relative position Error', 'start distance [m]', '2D Error [m]', '-', 1)
        util_plot.plot_one(ax_dr, eagleye_velocity_error, 'start_distance', 'error_2d', 'relative position Error', 'start distance [m]', '2D Error [m]', '-', 1)

        eagleye_angular_velocity_ErrTra = util_calc.calc_TraRate(eagleye_angular_velocity_error["error_2d"] , eval_step_max)
        eagleye_angular_ErrTra = util_calc.calc_TraRate(eagleye_angular_error["error_2d"] , eval_step_max)
        eagleye_velocity_ErrTra = util_calc.calc_TraRate(eagleye_velocity_error["error_2d"] , eval_step_max)

        ax_trarate_dr = fig2.add_subplot(2, 1, 2)
        util_plot.plot_one(ax_trarate_dr, eagleye_angular_velocity_ErrTra, 'x_label', 'ErrTra', 'Cumulative Error Distribution (relative position)', '2D error [m]', 'Rate [%]', '-', 10)
        util_plot.plot_one(ax_trarate_dr, eagleye_angular_ErrTra, 'x_label', 'ErrTra', 'Cumulative Error Distribution (relative position)', '2D error [m]', 'Rate [%]', '-', 10)
        util_plot.plot_one(ax_trarate_dr, eagleye_velocity_ErrTra, 'x_label', 'ErrTra', 'Cumulative Error Distribution (relative position)', '2D error [m]', 'Rate [%]', '-', 10)
        ax_trarate_dr.set_xscale('log') 
        ax_trarate_dr.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax_trarate_dr.set_xticks([0.01, 0.05, 0.1, 0.5, 1, 3])


    if 'x' in ref_df.columns and 'y' in ref_df.columns and 'yaw' in ref_df.columns and'angular_z' in data_df and 'yawrate_offset_stop' in data_df and'yawrate_offset' in data_df and 'slip' in data_df and 'distance' in data_df and 'velocity' in data_df:
        ref_xyz = pd.concat([ref_df['x'],ref_df['y']],axis=1)
        eagleye_angular_velocity, heading_angular_velocity = util_calc.calc_enu_vel_eagleye(ref_df["TimeStamp"],data_df['angular_z'],data_df['yawrate_offset_stop'],data_df['yawrate_offset'],data_df['slip'],data_df['velocity'],data_df['velocity'],np.deg2rad(ref_df["yaw"]))
        eagleye_angular_velocity_error = util_calc.calc_dr(ref_df["TimeStamp"],data_df["distance"],eagleye_angular_velocity,ref_xyz,distance_length,distance_step)

        fig2 = plt.figure()
        ax_dr = fig2.add_subplot(2, 1, 1)
        util_plot.plot_one(ax_dr, eagleye_angular_velocity_error, 'start_distance', 'error_2d', 'relative position Error', 'start distance [m]', '2D Error [m]', '-', 1)

        eagleye_angular_velocity_ErrTra = util_calc.calc_TraRate(eagleye_angular_velocity_error["error_2d"] , eval_step_max)

        ax_trarate_dr = fig2.add_subplot(2, 1, 2)
        util_plot.plot_one(ax_trarate_dr, eagleye_angular_velocity_ErrTra, 'x_label', 'ErrTra', 'Cumulative Error Distribution (relative position)', '2D error [m]', 'Rate [%]', '-', 10)
        ax_trarate_dr.set_xscale('log') 
        ax_trarate_dr.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax_trarate_dr.set_xticks([0.01, 0.05, 0.1, 0.5, 1, 3])

        correct_heading = pd.concat([heading_angular_velocity,ref_df['elapsed_time']],axis=1)
        correct_heading['yaw'] = np.rad2deg(correct_heading['yaw'])

        fig3 = plt.figure()
        ax_heading = fig3.add_subplot(1, 1, 1)
        util_plot.plot_three(ax_heading, ref_df, data_df, correct_heading, 'elapsed_time', 'yaw', 'heading', 'time[s]',  'heading [deg]', '-', ref_data_name)

        
    plt.show()
    