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

# evaluation_plot.py
# Author MapIV Hoda

import csv
import argparse
import math
from typing import List
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib.ticker

import util.preprocess as util_prepro
import util.calc as util_calc
import util.plot as util_plot

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-ref", "--ref", help="Path to the ref data(POSLV csv)")
    parser.add_argument("-df_ref", "--df_ref", help="Path to the ref data by df")
    parser.add_argument("-df_ref_ros", "--input_df_ref", help="Path to the ref data by df")
    parser.add_argument("-csv", "--input_csv", help="Path to the csv")
    parser.add_argument("-df_csv", "--input_df_csv", help="Path to the csv by df")
    parser.add_argument("-log", "--input_log_csv", help="Path to the csv by df")
    parser.add_argument("-ref_log", "--input_ref_log", help="Path to the csv by df")
    parser.add_argument("-p", "--plane_num",help="Plane Cartesian coordinate system number")
    parser.add_argument("-s", "--sync_threshold_time_data_param",help="Gap time to be regarded as synchronous")
    parser.add_argument("-l", "--leap_time_param",help="Time offset correction")
    parser.add_argument("-tf_x", "--tf_x_param",help="tf offset correction")
    parser.add_argument("-tf_y", "--tf_y_param",help="tf offset correction")
    parser.add_argument("-tf_across", "--tf_across_param",help="tf offset correction")
    parser.add_argument("-tf_along", "--tf_along_param",help="tf offset correction")
    parser.add_argument("-tf_height", "--tf_height_param",help="tf offset correction")
    parser.add_argument("-tf_yaw", "--tf_yaw_param",help="tf offset correction")
    parser.add_argument("-r", "--reverse_imu", action="store_true",help="change reverse_imu true")
    parser.add_argument("-dr_l", "--distance_length_param",help="tf offset correction")
    parser.add_argument("-dr_s", "--distance_step_param",help="tf offset correction")
    parser.add_argument("-ref_name", "--ref_data_name_param",help="ref data name")
    parser.add_argument("-y", "--yaml_path",help="yaml name")
    args = parser.parse_args()
    reverse_imu: bool = args.reverse_imu

    # default_param
    plane = 7 # Plane Cartesian coordinate system number (default:7 = 7)
    sync_threshold_time = 0.005 # Time threshold for judgment at synchronized. (default:0.005 = 0.005[s])
    leap_time = 0.0 # Offset correction for time synchronization. (default:0.0 = 0.0[s])
    tf_x = 0.0
    tf_y = 0.0
    tf_across = 0.0
    tf_along = 0.0
    tf_height = 0.0
    tf_yaw = 0.0
    distance_length = 100 # Distance to calculate relative trajectory. (default:100 = 100[m])
    distance_step = 50 # Calculate relative trajectories step (default:50 = 50[m])
    ref_data_name = 'ref data' # Reference Legend Name. (default:'ref data')
    eval_step_max = 3.0 # Maximum value of error to be evaluated default (default:3.0 = 3.0[m])

    # set param
    if args.plane_num != None:
        plane = float(args.plane_num)

    if args.sync_threshold_time_data_param != None:
        sync_threshold_time = float(args.sync_threshold_time_data_param)

    if args.leap_time_param != None:
        leap_time = float(args.leap_time_param)

    if args.tf_x_param != None:
        tf_x = float(args.tf_x_param)
    
    if args.tf_y_param != None:
        tf_y = float(args.tf_y_param)

    if args.tf_across_param != None:
        tf_across = float(args.tf_across_param)

    if args.tf_along_param != None:
        tf_along = float(args.tf_along_param)

    if args.tf_height_param != None:
        tf_height = float(args.tf_height_param)

    if args.tf_yaw_param != None:
        tf_yaw = float(args.tf_yaw_param)

    if args.distance_length_param != None:
        distance_length = float(args.distance_length_param)

    if args.distance_step_param != None:
        distance_step = float(args.distance_step_param)

    if args.ref_data_name_param != None:
        ref_data_name = args.ref_data_name_param
    
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
        ref_data_df = util_prepro.set_ref_data(args.ref,args.yaml_path)
        print("set ref_data")

    if args.input_csv != None:
        csv_data_df = util_prepro.set_csv_data(args.input_csv,args.yaml_path)
        print("set csv_data")
        
    if args.df_ref != None:
        ref_data_df = util_prepro.set_ref_df(args.df_ref)
        print("set ref_data")

    if args.input_df_ref != None:
        ref_data_df = util_prepro.set_df(args.input_df_ref)
        print("set ref_data")

    if args.input_df_csv != None:
        csv_data_df = util_prepro.set_df(args.input_df_csv)
        print("set csv_data")

    if args.input_ref_log != None:
        ref_data_df, ref_raw_df = util_prepro.set_log_df(args.input_ref_log,plane)
        print("set ref_data")

    if args.input_log_csv != None:
        csv_data_df, raw_df = util_prepro.set_log_df(args.input_log_csv,plane)
        print("set csv_data")
    
    if tf_x != 0 or tf_y != 0:
        ref_data_df = util_prepro.set_tf_xy(ref_data_df,tf_x,tf_y)
        print('set tf xy')
    

    # syne time
    print("start sync_time")
    ref_df , data_df = util_calc.sync_time(ref_data_df,csv_data_df,sync_threshold_time,leap_time)
    print("finished sync_time")

    # Quaternion to Euler conversion
    eagleye_rpy = pd.DataFrame()
    if 'ori_x' in data_df.columns and 'ori_y' in data_df.columns and 'ori_z' in data_df.columns and 'ori_w' in data_df.columns:
        eagleye_ori_df = pd.concat([data_df['ori_x'],data_df['ori_y'],data_df['ori_z'],data_df['ori_w']],axis=1)
        eagleye_rpy = util_calc.quaternion_to_euler_zyx(eagleye_ori_df)
    elif 'roll' in data_df.columns and 'pitch' in data_df.columns and 'yaw' in data_df.columns:
        eagleye_rpy = pd.concat([data_df['roll'],data_df['pitch'],data_df['yaw']],axis=1)

    if tf_yaw != 0 or reverse_imu == True :
        print("set eagleye yaw data")
        set_heading_data: List[float] = []
        if reverse_imu == True:
            eagleye_rpy['yaw'] = -1 * eagleye_rpy['yaw']
        eagleye_rpy['yaw'] = eagleye_rpy['yaw'] + tf_yaw
        for i in range(len(eagleye_rpy)):
            yaw_tmp = util_calc.change_anglel_limit_pi(math.radians(eagleye_rpy['yaw'][i]))
            yaw = math.degrees(yaw_tmp)
            set_heading_data.append([yaw])
        eagleye_rpy['yaw'] = pd.DataFrame(set_heading_data,columns=['yaw'])

    
    ref_rpy = pd.DataFrame()
    if 'ori_x' in ref_df.columns and 'ori_y' in ref_df.columns and 'ori_z' in ref_df.columns and 'ori_w' in ref_df.columns:
        ref_ori_df = pd.concat([ref_df['ori_x'],ref_df['ori_y'],ref_df['ori_z'],ref_df['ori_w']],axis=1)
        ref_rpy = util_calc.quaternion_to_euler_zyx(ref_ori_df)
        print("finished calc ref angle")
    elif 'roll' in ref_df.columns and 'pitch' in ref_df.columns and 'yaw' in ref_df.columns:
        ref_rpy = pd.concat([ref_df['roll'],ref_df['pitch'],ref_df['yaw']],axis=1)

    # calc ref velocity
    ref_velocity = pd.DataFrame()
    if 'vel_x' in ref_df.columns and 'vel_y' in ref_df.columns and 'vel_z' in ref_df.columns and not 'velocity' in ref_df.columns:
        ref_vel_df = pd.concat([ref_df['vel_x'],ref_df['vel_y'],ref_df['vel_z']],axis=1)
        ref_velocity = util_calc.calc_velocity(ref_vel_df)
    elif 'velocity' in ref_df.columns:
        ref_velocity['velocity'] = ref_df['velocity']

    # correct anntena position
    if tf_across != 0 or tf_along != 0 or tf_height != 0:
        data_df = util_prepro.correct_anntenapos(data_df,ref_rpy['yaw'],tf_across,tf_along,tf_height)
        print('set tf')
    
    # plot 6dof
    ref_data_xyz = pd.concat([ref_df['x'],ref_df['y'],ref_df['z']],axis=1)
    eagleye_xyz = pd.concat([data_df['x'],data_df['y'],data_df['z']],axis=1)
    eaegleye_6dof = pd.concat([eagleye_xyz,eagleye_rpy],axis=1)
    ref_6dof = pd.concat([ref_data_xyz,ref_rpy],axis=1)
    util_plot.plot_6DoF(ref_df['elapsed_time'], eaegleye_6dof, ref_6dof,ref_data_name)
    

    # calc 6dof error
    Error_data = util_calc.calc_error_xyz(ref_df['elapsed_time'],ref_df['TimeStamp'],data_df['TimeStamp'],ref_data_xyz,eagleye_xyz,ref_rpy['yaw'])
    print("finished calc_error_xyz")

    error_rpy = util_calc.calc_error_rpy(ref_rpy,eagleye_rpy)
    print("finished calc error rpy")

    error_velocity = pd.DataFrame()
    if 'velocity' in data_df.columns and 'velocity' in ref_velocity.columns:
        error_velocity = util_calc.calc_velocity_error(data_df['velocity'],ref_velocity['velocity'])

    # plot 6dof error
    error_plot_df = pd.concat([Error_data,error_rpy,error_velocity],axis=1)
    error_table = util_calc.error_evaluation(error_plot_df)
    util_plot.plot_error_6DoF(error_plot_df,ref_data_name,error_table)
    util_plot.plot_error(error_plot_df,ref_data_name)
    util_plot.plot_error_distributiln(error_plot_df,ref_data_name)

    # plot Cumulative Error Distribution
    diff_2d: List[float] = []
    if '2d' in Error_data.columns:
        diff_2d = Error_data['2d'].values.tolist()
        ErrTra_Rate = util_calc.calc_TraRate(diff_2d , eval_step_max)

        fig = plt.figure()
        ax1 = fig.add_subplot(1, 1, 1)
        util_plot.plot_one(ax1, ErrTra_Rate, 'x_label', 'ErrTra', 'Cumulative Error Distribution', '2D error [m]', 'Rate [%]', '-', 10)
        ax1.set_xscale('log') 
        ax1.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax1.set_xticks([0.01, 0.05, 0.1, 0.5, 1, 3])

    # twist Performance Evaluation
    if 'vel_x' in data_df.columns and 'vel_y' in data_df.columns and 'distance' in data_df.columns:
        print("start calc relative position")
        eagleye_vel_xyz = pd.concat([data_df['vel_x'],data_df['vel_y'],data_df['vel_z']],axis=1)
        ref_xyz = pd.concat([ref_df['x'],ref_df['y'],ref_df['z']],axis=1)
        calc_error = util_calc.clac_dr(data_df['TimeStamp'],data_df['distance'],eagleye_xyz,eagleye_vel_xyz,ref_xyz,distance_length,distance_step)
        print("finished calc relative position")

        dr_error_2d = calc_error['error_2d'].values.tolist()
        ErrTra_dr_df = util_calc.calc_TraRate(dr_error_2d , eval_step_max)

        fig15 = plt.figure()
        ax_dr = fig15.add_subplot(2, 1, 1)
        util_plot.plot_one(ax_dr, calc_error, 'start_distance', 'error_2d', 'relative position Error', 'start distance [m]', '2D Error [m]', '-', 1)
        
        ax_trarate_dr = fig15.add_subplot(2, 1, 2)
        util_plot.plot_one(ax_trarate_dr, ErrTra_dr_df, 'x_label', 'ErrTra', 'Cumulative Error Distribution (relative position)', '2D error [m]', 'Rate [%]', '-', 10)
        ax_trarate_dr.set_xscale('log') 
        ax_trarate_dr.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax_trarate_dr.set_xticks([0.01, 0.05, 0.1, 0.5, 1, 3])

    #  plot velocity
    if 'velocity' in ref_velocity.columns and 'velocity' in data_df.columns:
        fig11 = plt.figure()
        ax_vel = fig11.add_subplot(2, 1, 1)
        util_plot.plot_each(ax_vel, ref_df['elapsed_time'], data_df, ref_velocity, 'velocity', 'Velocity', 'Velocity [m/s]',ref_data_name)

        ax_err_vel = fig11.add_subplot(2, 1, 2)
        fig11.suptitle(ref_data_name + ' - eagleye Error')
        util_plot.plot_one(ax_err_vel, error_plot_df, 'elapsed_time', 'velocity', 'Velocity Error', 'time [s]', 'Velocity error[m/s]', 'None', 1)

    elif 'velocity' in ref_velocity.columns or 'velocity' in data_df.columns:
        fig11 = plt.figure()
        ax_vel = fig11.add_subplot(1, 1, 1)
        util_plot.plot_each(ax_vel, ref_df['elapsed_time'], data_df, ref_velocity, 'velocity', 'Velocity', 'Velocity [m/s]',ref_data_name)

    # plot 2D trajectory
    util_plot.plot_traj(ref_data_xyz, eagleye_xyz, ref_data_name)

    if 'qual' in data_df.columns:
        util_plot.plot_traj_qual(eagleye_xyz,data_df['qual'])

    # plot 3d trajectory
    util_plot.plot_traj_3d( ref_df, data_df, ref_data_name)

    print(error_table)

    plt.show()
    