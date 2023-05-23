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
    parser.add_argument("-target", "--input_target", help="Path to the csv")
    parser.add_argument("-log", "--input_log_csv", help="Path to the csv by df")
    parser.add_argument("-ref_log", "--input_ref_log", help="Path to the csv by df")
    parser.add_argument("-yaml", "--yaml_path",help="yaml name")
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
    based_heaing_angle = config["param"]["based_heaing_angle"]
    distance_length = config["param"]["distance_length_param"]
    distance_step = config["param"]["distance_step_param"]
    eval_step_max = config["param"]["eval_step_max_param"]
    dr_error_ylim = config["evaluation_plot"]["dr_error_ylim"]
    plot_text_data = config["evaluation_plot"]["plot_text_data"]
    plot_text_step = config["evaluation_plot"]["plot_text_step"]
    ref_data_name = config["param"]["ref_data_name_param"]
    data_name = config["param"]["data_name_param"]
    font_size = config["param"]["font_size_param"]
    
    print('plane',plane)
    print('sync_threshold_time',sync_threshold_time)
    print('leap_time',leap_time)
    print('tf_x',tf_x)
    print('tf_y',tf_y)
    print('tf_across',tf_across)
    print('tf_along',tf_along)
    print('tf_height',tf_height)
    print('tf_yaw',tf_yaw)
    print('based_heaing_angle',based_heaing_angle)
    print('reverse_imu',reverse_imu)
    print('distance_length',distance_length)
    print('distance_step',distance_step)
    print('eval_step_max',eval_step_max)
    print('dr_error_ylim',dr_error_ylim)
    print('plot_text_data',plot_text_data)
    print('plot_text_step',plot_text_step)
    print('ref_data_name',ref_data_name)
    print('data_name',data_name)

    # set data
    if args.ref != None:
        ref_data_df = util_prepro.set_ref_data(args.ref,config)
        print("set ref_data")

    if args.input_target != None:
        target_data_df = util_prepro.set_target_data(args.input_target,config)
        print("set target_data")

    if args.input_ref_log != None:
        ref_data_df, ref_raw_df = util_prepro.set_log_df(args.input_ref_log,plane,config)
        print("set ref_data")

    if args.input_log_csv != None:
        target_data_df, raw_df = util_prepro.set_log_df(args.input_log_csv,plane,config)
        print("set target_data")
    
    if tf_x != 0 or tf_y != 0:
        ref_data_df = util_prepro.set_tf_xy(ref_data_df,tf_x,tf_y)
        print('set tf xy')
    

    # syne time
    print("start sync_time")
    ref_df , data_df = util_calc.sync_time(ref_data_df,target_data_df,sync_threshold_time,leap_time)
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
        if reverse_imu == True:
            eagleye_rpy['yaw'] = -1 * eagleye_rpy['yaw']
            if 'angular_z' in data_df.columns:
                data_df['angular_z'] = -1 * data_df['angular_z']
            if 'yaw_rate_offset_stop' in data_df.columns:
                data_df['yaw_rate_offset_stop'] = -1 * data_df['yaw_rate_offset_stop']
            if 'yaw_rate_offset' in data_df.columns:
                data_df['yaw_rate_offset'] = -1 * data_df['yaw_rate_offset']
            if 'slip' in data_df.columns:
                data_df['slip'] = -1 * data_df['slip']
        eagleye_rpy['yaw'] = eagleye_rpy['yaw'] + tf_yaw
        eagleye_rpy['yaw'] = np.rad2deg(util_calc.change_anglel_limit(np.deg2rad(eagleye_rpy['yaw'])))

    # correct anntena position
    ref_rpy = pd.concat([ref_df['roll'],ref_df['pitch'],ref_df['yaw']],axis=1)
    if tf_across != 0 or tf_along != 0 or tf_height != 0:
        data_df = util_prepro.correct_anntenapos(data_df,ref_rpy['yaw'],tf_across,tf_along,tf_height)
        print('set tf')
    
    # plot 6dof
    ref_data_xyz = pd.concat([ref_df['x'],ref_df['y'],ref_df['z']],axis=1)
    eagleye_xyz = pd.concat([data_df['x'],data_df['y'],data_df['z']],axis=1)
    eaegleye_6dof = pd.concat([eagleye_xyz,eagleye_rpy],axis=1)
    ref_6dof = pd.concat([ref_data_xyz,ref_rpy],axis=1)
    util_plot.plot_6DoF(ref_df['elapsed_time'], eaegleye_6dof, ref_6dof,data_name, ref_data_name, font_size)
    

    # calc 6dof error
    Error_data = util_calc.calc_error_xyz(ref_df['elapsed_time'],ref_df['TimeStamp'],data_df['TimeStamp'],ref_data_xyz,eagleye_xyz,ref_rpy['yaw'])
    print("finished calc_error_xyz")

    error_rpy = util_calc.calc_error_rpy(ref_rpy,eagleye_rpy)
    print("finished calc error rpy")

    error_velocity = pd.DataFrame()
    if 'velocity' in data_df.columns and 'velocity' in ref_df.columns:
        error_velocity = util_calc.calc_velocity_error(data_df['velocity'],ref_df['velocity'])

    # plot 6dof error
    error_plot_df = pd.concat([Error_data,error_rpy,error_velocity],axis=1)
    error_table = util_calc.error_evaluation(error_plot_df)
    util_plot.plot_error_6DoF(error_plot_df,ref_data_name,error_table, font_size)
    util_plot.plot_error(error_plot_df,ref_data_name, font_size)
    util_plot.plot_error_distributiln(error_plot_df, font_size,ref_data_name)

    # plot Cumulative Error Distribution
    diff_2d: List[float] = []
    if '2d' in Error_data.columns:
        diff_2d = Error_data['2d'].values.tolist()
        ErrTra_Rate = util_calc.calc_TraRate(diff_2d , eval_step_max)

        fig = plt.figure()
        ax1 = fig.add_subplot(1, 1, 1)
        util_plot.plot_one(ax1, ErrTra_Rate, 'x_label', 'ErrTra', 'Cumulative Error Distribution', '2D error [m]', 'Rate [%]', '-', 10, font_size)
        ax1.set_xscale('log') 
        ax1.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax1.set_xticks([0.01, 0.05, 0.1, 0.5, 1, 3])

    # twist Performance Evaluation
    if 'vel_x' in data_df.columns and 'vel_y' in data_df.columns and 'yaw_rate_offset_stop' in data_df.columns and 'yaw_rate_offset' in data_df.columns and 'slip' in data_df.columns:
        print("start calc relative position")
        eagleye_vel_xyz = pd.concat([data_df['vel_x'],data_df['vel_y'],data_df['vel_z']],axis=1)
        ref_xyz = pd.concat([ref_df['x'],ref_df['y'],ref_df['z']],axis=1)
        eagleye_twist_data = pd.concat([data_df['angular_z'],data_df['yaw_rate_offset_stop'],data_df['yaw_rate_offset'],data_df['velocity'],data_df['slip']],axis=1)
        calc_error, dr_trajcetory = util_calc.calc_dr_eagleye(ref_df["TimeStamp"],data_df["distance"],eagleye_twist_data,np.deg2rad(ref_rpy["yaw"]),ref_xyz,distance_length,distance_step,based_heaing_angle)
        print("finished calc relative position")

        dr_error_2d = calc_error['error_2d'].values.tolist()
        ErrTra_dr_df = util_calc.calc_TraRate(dr_error_2d , eval_step_max)

        fig2 = plt.figure()
        ax_dr = fig2.add_subplot(2, 1, 1)
        util_plot.plot_one(ax_dr, calc_error, 'start_distance', 'error_2d', 'relative position Error', 'start distance [m]', '2D Error [m]', '-', 1, font_size)
        ax_dr.set_ylim([0.0,dr_error_ylim])
        
        ax_trarate_dr = fig2.add_subplot(2, 1, 2)
        util_plot.plot_one(ax_trarate_dr, ErrTra_dr_df, 'x_label', 'ErrTra', 'Cumulative Error Distribution (relative position)', '2D error [m]', 'Rate [%]', '-', 10, font_size)
        ax_trarate_dr.set_xscale('log') 
        ax_trarate_dr.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax_trarate_dr.set_xticks([0.01, 0.05, 0.1, 0.5, 1, 3])

        util_plot.plot_traj_text('DR Trajectory', ref_xyz, dr_trajcetory,ref_df[plot_text_data], plot_text_step, font_size, data_name, ref_data_name)
        calc_error_csv = pd.concat([calc_error['start_distance'],calc_error['error_2d']],axis=1)
        calc_error_csv.to_csv("eagleye_dr_error.csv", header=True, index=False,float_format='%.9f')

    #  plot velocity
    if 'velocity' in ref_df.columns and 'velocity' in data_df.columns:
        fig3 = plt.figure()
        ax_vel = fig3.add_subplot(2, 1, 1)
        util_plot.plot_each(ax_vel, ref_df['elapsed_time'], data_df, ref_df, 'velocity', 'Velocity', 'Velocity [m/s]',data_name, ref_data_name, font_size)

        ax_err_vel = fig3.add_subplot(2, 1, 2)
        fig3.suptitle(ref_data_name + ' - eagleye Error')
        util_plot.plot_one(ax_err_vel, error_plot_df, 'elapsed_time', 'velocity', 'Velocity Error', 'time [s]', 'Velocity error[m/s]', 'None', 1, font_size)

    elif 'velocity' in ref_df.columns or 'velocity' in data_df.columns:
        fig3 = plt.figure()
        ax_vel = fig3.add_subplot(1, 1, 1)
        util_plot.plot_each(ax_vel, ref_df['elapsed_time'], data_df, ref_df, 'velocity', 'Velocity', 'Velocity [m/s]',data_name,ref_data_name, font_size)

    # plot 2D trajectory
    util_plot.plot_traj_text('2D Trajectory', ref_data_xyz, eagleye_xyz, ref_df[plot_text_data], plot_text_step, font_size, data_name, ref_data_name)

    if 'qual' in data_df.columns:
        util_plot.plot_traj_qual(eagleye_xyz,data_df['qual'], data_df[plot_text_data], plot_text_step, font_size)
    elif 'qual' in ref_df.columns:
        util_plot.plot_traj_qual(ref_data_xyz,ref_df['qual'], ref_df[plot_text_data], plot_text_step, font_size)


    # plot 3d trajectory
    util_plot.plot_traj_3d( ref_df, data_df, font_size, data_name, ref_data_name)

    print(error_table)

    plt.show()
    