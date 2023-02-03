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

# plot.py
# Author MapIV Hoda

from typing import List
import pandas as pd
import numpy as np
from math import log10 , floor

import matplotlib.pyplot as plt
# plt.rcParams["font.size"] = 18

def judge_qual(xyz,qual,qual_num):
    set_output_df: List[float] = []
    for i in range(len(xyz)):
        if qual_num == qual[i]:
            x = xyz['x'][i]
            y = xyz['y'][i]
            z = xyz['z'][i]
            qual_data = qual[i]
            set_output_df.append([x,y,z,qual_data])
    output_df = pd.DataFrame(set_output_df,columns=['x','y','z','qual'])
    return output_df

def plot_xyz(ax, eagleye_x_data, rtk_x_data, raw_x_data, eagleye_xyz, rtk_xyz, raw_xyz, elem, title, y_label, font_size):
    if elem in raw_xyz.columns:
        ax.plot(raw_x_data , raw_xyz[elem] , marker=".", linestyle="None",markersize=1, color = "red",  label="gnss raw data(rtklib)")
    if elem in rtk_xyz.columns:
        ax.plot(rtk_x_data , rtk_xyz[elem] , marker="o", linestyle="None",markersize=1, color = "green",  label="gnss rtk data(nmea)")
    if elem in eagleye_xyz.columns:
        ax.plot(eagleye_x_data , eagleye_xyz[elem] , marker="s", linestyle="None",markersize=1, alpha=0.3, color = "blue",  label="eagleye")
    ax.set_xlabel('time [s]', fontsize=font_size)
    ax.set_ylabel(y_label, fontsize=font_size)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_rpy(ax, x_data, eagleye_plot_rpy, dopplor, elem, title, y_label, font_size):
    if elem in dopplor.columns:
        ax.plot(x_data , dopplor[elem] , marker="o", linestyle="None",markersize=1, color = "green",  label="dopplor")
    if elem in eagleye_plot_rpy.columns:
        ax.plot(x_data , eagleye_plot_rpy[elem] , marker="s", linestyle="None",markersize=1, color = "blue",  label="eagleye")
    ax.set_xlabel('time [s]', fontsize=font_size)
    ax.set_ylabel(y_label, fontsize=font_size)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_each(ax, x_data , eagleye, ref, y_data, title, y_label,data_name, ref_data_name, font_size):
    if y_data in ref.columns:
        ax.plot(x_data, ref[y_data] , marker="o", linestyle="None",markersize=1, color = "green",  label=ref_data_name)
    if y_data in eagleye.columns:
        ax.plot(x_data , eagleye[y_data] , marker="s", linestyle="None",markersize=1, alpha=0.3, color = "blue",  label=data_name)
    ax.set_xlabel('time [s]', fontsize=font_size)
    ax.set_ylabel(y_label, fontsize=font_size)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_6DoF_single(eagleye_x_data, rtk_x_data, raw_x_data, eagleye_xyz, rtk_xyz, raw_xyz, eagleye_plot_rpy,dopplor, font_size):
    fig1 = plt.figure()
    ax_x = fig1.add_subplot(2, 3, 1)
    ax_y = fig1.add_subplot(2, 3, 2)
    ax_z = fig1.add_subplot(2, 3, 3)
    ax_roll = fig1.add_subplot(2, 3, 4)
    ax_pitch = fig1.add_subplot(2, 3, 5)
    ax_yaw = fig1.add_subplot(2, 3, 6)
    plot_xyz(ax_x, eagleye_x_data, rtk_x_data, raw_x_data, eagleye_xyz, rtk_xyz, raw_xyz, 'x', 'X (East-West)','East [m]', font_size)
    plot_xyz(ax_y, eagleye_x_data, rtk_x_data, raw_x_data, eagleye_xyz, rtk_xyz, raw_xyz, 'y', 'Y (North-South)','North [m]', font_size)
    plot_xyz(ax_z,eagleye_x_data, rtk_x_data, raw_x_data, eagleye_xyz, rtk_xyz, raw_xyz, 'z', 'Z (Height)','Height [m]', font_size)
    plot_rpy(ax_roll, eagleye_x_data, eagleye_plot_rpy, dopplor, 'roll', 'Roll' , 'Roll [deg]', font_size)
    plot_rpy(ax_pitch, eagleye_x_data, eagleye_plot_rpy, dopplor, 'pitch', 'Pitch', 'Pitch [deg]', font_size)
    plot_rpy(ax_yaw, eagleye_x_data, eagleye_plot_rpy, dopplor, 'yaw', 'Yaw', 'Yaw [deg]', font_size)

def plot_6DoF(x_data,eagleye, ref,data_name, ref_data_name, font_size):
    fig1 = plt.figure()
    ax_x = fig1.add_subplot(2, 3, 1)
    ax_y = fig1.add_subplot(2, 3, 2)
    ax_z = fig1.add_subplot(2, 3, 3)
    ax_roll = fig1.add_subplot(2, 3, 4)
    ax_pitch = fig1.add_subplot(2, 3, 5)
    ax_yaw = fig1.add_subplot(2, 3, 6)
    plot_each(ax_x, x_data, eagleye, ref, 'x', 'X (East-West)','East [m]',data_name, ref_data_name, font_size)
    plot_each(ax_y, x_data ,eagleye, ref, 'y', 'Y (North-South)','North [m]',data_name, ref_data_name, font_size)
    plot_each(ax_z, x_data , eagleye, ref, 'z', 'Z (Height)','Height [m]',data_name, ref_data_name, font_size)
    plot_each(ax_roll, x_data , eagleye, ref, 'roll', 'Roll' , 'Roll [deg]',data_name, ref_data_name, font_size)
    plot_each(ax_pitch, x_data , eagleye, ref, 'pitch', 'Pitch', 'Pitch [deg]',data_name, ref_data_name, font_size)
    plot_each(ax_yaw, x_data , eagleye, ref, 'yaw', 'Yaw', 'Yaw [deg]',data_name, ref_data_name, font_size)

def plot_each_error(ax, error_data, y_data, title, y_label, font_size):
    ax.plot(error_data['elapsed_time'] , error_data[y_data] , marker="s", linestyle="None",markersize=1, color = "blue")
    ax.set_xlabel('time [s]', fontsize=font_size)
    ax.set_ylabel(y_label, fontsize=font_size)
    ax.set_title(title)
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_one(ax, error_data, x_data, y_data, title, x_label, y_label, line_style, marker_size, font_size):
    ax.plot(error_data[x_data] , error_data[y_data] , marker="s", linestyle=line_style, markersize=marker_size, color = "blue")
    ax.set_xlabel(x_label, fontsize=font_size)
    ax.set_ylabel(y_label, fontsize=font_size)
    ax.set_title(title)
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_two(ax, ref, eagleye, x_data, y_data, title, x_label, y_label, line_style, font_size, data_name, ref_data_name):
    ax.plot(ref[x_data] , ref[y_data] , marker=".", linestyle=line_style, markersize=1, color = "red", label=ref_data_name)
    ax.plot(eagleye[x_data] , eagleye[y_data] , marker="s", linestyle=line_style, markersize=1, alpha=0.3, color = "blue", label=data_name)
    ax.set_xlabel(x_label, fontsize=font_size)
    ax.set_ylabel(y_label, fontsize=font_size)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_three(ax, ref, eagleye, raw, x_data, y_data, title, x_label, y_label, line_style, font_size, data_name, ref_data_name):
    ax.plot(ref[x_data] , ref[y_data] , marker=".", linestyle=line_style, markersize=1, color = "red", label=ref_data_name)
    ax.plot(raw[x_data] , raw[y_data] , marker="o", linestyle=line_style, markersize=1, color = "green", label="initial ref heading")
    ax.plot(eagleye[x_data] , eagleye[y_data] , marker="s", linestyle=line_style, markersize=1, alpha=0.3, color = "blue", label=data_name)
    ax.set_xlabel(x_label, fontsize=font_size)
    ax.set_ylabel(y_label, fontsize=font_size)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_error_6DoF(error_data,ref_data_name, error_table, font_size):
    fig2 = plt.figure()
    fig2.suptitle(ref_data_name + ' - eagleye Error')
    ax_x = fig2.add_subplot(4, 3, 1)
    ax_y = fig2.add_subplot(4, 3, 2)
    ax_z = fig2.add_subplot(4, 3, 3)
    ax_x_table = fig2.add_subplot(4, 3, 4)
    ax_y_table = fig2.add_subplot(4, 3, 5)
    ax_z_table = fig2.add_subplot(4, 3, 6)
    ax_roll = fig2.add_subplot(4, 3, 7)
    ax_pitch = fig2.add_subplot(4, 3, 8)
    ax_yaw = fig2.add_subplot(4, 3, 9)
    ax_roll_table = fig2.add_subplot(4, 3, 10)
    ax_pitch_table = fig2.add_subplot(4, 3, 11)
    ax_yaw_table = fig2.add_subplot(4, 3, 12)
    plot_each_error(ax_x, error_data, 'x', 'X (East-West) Error','East error [m]', font_size)
    plot_each_error(ax_y, error_data, 'y', 'Y (North-South) Error','North error [m]', font_size)
    plot_each_error(ax_z, error_data, 'z', 'Z (Height) Error','Height error [m]', font_size)
    plot_each_error(ax_roll, error_data, 'roll', 'Roll Error' , 'Roll error [deg]', font_size)
    plot_each_error(ax_pitch, error_data, 'pitch', 'Pitch Error', 'Pitch error [deg]', font_size)
    plot_each_error(ax_yaw, error_data, 'yaw', 'Yaw Error', 'Yaw error [deg]', font_size)
    plot_table(ax_x_table, error_table[error_table["data"] == "x"], "[m]", font_size)
    plot_table(ax_y_table, error_table[error_table["data"] == "y"], "[m]", font_size)
    plot_table(ax_z_table, error_table[error_table["data"] == "z"], "[m]", font_size)
    plot_table(ax_roll_table, error_table[error_table["data"] == "roll"], "[deg]", font_size)
    plot_table(ax_pitch_table, error_table[error_table["data"] == "pitch"], "[deg]", font_size)
    plot_table(ax_yaw_table, error_table[error_table["data"] == "yaw"], "[deg]", font_size)

def plot_table(ax, error_table, unit, font_size):
    table_data = error_table.set_index('data')
    table = ax.table(cellText = table_data.values, colLabels = table_data.columns + unit, loc = 'center')
    table.auto_set_font_size(False)
    table.set_fontsize(font_size)
    ax.axis('tight')
    ax.axis('off')

def plot_error(error_data,ref_data_name, font_size):
    fig4 = plt.figure()
    fig4.suptitle(ref_data_name + ' - eagleye Error')
    ax_x = fig4.add_subplot(2, 2, 1)
    ax_y = fig4.add_subplot(2, 2, 2)
    ax_across = fig4.add_subplot(2, 2, 3)
    ax_along = fig4.add_subplot(2, 2, 4)
    plot_each_error(ax_x, error_data, 'x', 'X (East-West) Error','East error [m]', font_size)
    plot_each_error(ax_y, error_data, 'y', 'Y (North-South) Error','North error [m]', font_size)
    plot_each_error(ax_across, error_data, 'across', 'Across Error','across error [m]', font_size)
    plot_each_error(ax_along, error_data, 'along', 'Along Error','along error [m]', font_size)

def plot_error_distributiln(error_data, font_size,ref_data_name):
    fig5 = plt.figure()
    fig5.suptitle(ref_data_name + ' - eagleye Error')
    ax_xy = fig5.add_subplot(1, 2, 1)
    ax_aa = fig5.add_subplot(1, 2, 2)
    plot_one(ax_xy, error_data, 'x', 'y', 'X-Y error', 'x error [m]', 'y error [m]', 'None', 1, font_size)
    plot_one(ax_aa, error_data, 'across', 'along', 'Across-Along error', 'across error [m]', 'along error [m]', 'None', 1, font_size)
    ax_xy.set_aspect('equal')
    ax_xy.axis('square')
    ax_aa.set_aspect('equal')
    ax_aa.axis('square')

def plot_traj(ref_data, data, font_size,data_name, ref_data_name):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(ref_data['x']-ref_data['x'][0] , ref_data['y']-ref_data['y'][0] , marker=".",linestyle="None",markersize=3, color = "red",  label=ref_data_name)
    ax.plot(data['x']-ref_data['x'][0] , data['y']-ref_data['y'][0] ,  marker="s",linestyle="None",markersize=3,alpha=0.3 , color = "blue",  label=data_name)
    ax.set_xlabel('East [m]', fontsize=font_size)
    ax.set_ylabel('North [m]', fontsize=font_size)
    ax.set_title('2D Trajectory')
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()
    ax.set_aspect('equal')
    ax.axis('square')

def plot_traj_text(title, ref_data, data, text, step, font_size, data_name, ref_data_name):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(ref_data['x']-ref_data['x'][0] , ref_data['y']-ref_data['y'][0] , marker=".",linestyle="None",markersize=3, color = "red",  label=ref_data_name)
    ax.plot(data['x']-ref_data['x'][0] , data['y']-ref_data['y'][0] ,  marker="s",linestyle="None",markersize=3,alpha=0.3 , color = "blue",  label=data_name)
    cnt = 0
    for i in np.arange(0, len(text.index), step):
        if text[i] >= cnt * step:
            ax.text(ref_data['x'][i] - ref_data['x'][0] , ref_data['y'][i] - ref_data['y'][0] , int(text[i]), size=font_size)
            cnt = cnt + 1
    ax.set_xlabel('East [m]', fontsize=font_size)
    ax.set_ylabel('North [m]', fontsize=font_size)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()
    ax.set_aspect('equal')
    ax.axis('square')

def plot_traj_text_tree(title, raw_data, ref_data, data, text, step, font_size, data_name, ref_data_name):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(raw_data['x']-ref_data['x'][0] , raw_data['y']-ref_data['y'][0] , marker=".",linestyle="None",markersize=1, color = "red",  label="gnss raw data(rtklib)")
    ax.plot(ref_data['x']-ref_data['x'][0] , ref_data['y']-ref_data['y'][0] , marker=".",linestyle="None",markersize=3, color = "green",  label=ref_data_name)
    ax.plot(data['x']-ref_data['x'][0] , data['y']-ref_data['y'][0] ,  marker="s",linestyle="None",markersize=3,alpha=0.3 , color = "blue",  label=data_name)
    cnt = 0
    for i in np.arange(0, len(text.index), step):
        if text[i] >= cnt * step:
            ax.text(ref_data['x'][i] - ref_data['x'][0] , ref_data['y'][i] - ref_data['y'][0] , int(text[i]), size=16)
            cnt = cnt + 1
    ax.set_xlabel('East [m]', fontsize=font_size)
    ax.set_ylabel('North [m]', fontsize=font_size)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()
    ax.set_aspect('equal')
    ax.axis('square')

def plot_traj_three(raw_data, ref_data, data, font_size, data_name, ref_data_name):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(raw_data['x']-ref_data['x'][0] , raw_data['y']-ref_data['y'][0] , marker=".",linestyle="None",markersize=1, color = "red",  label="gnss raw data(rtklib)")
    ax.plot(ref_data['x']-ref_data['x'][0] , ref_data['y']-ref_data['y'][0] , marker=".",linestyle="None",markersize=1, color = "green",  label=ref_data_name)
    ax.plot(data['x']-ref_data['x'][0] , data['y']-ref_data['y'][0] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label=data_name)
    ax.set_xlabel('East [m]', fontsize=font_size)
    ax.set_ylabel('North [m]', fontsize=font_size)
    ax.set_title('2D Trajectory')
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()
    ax.set_aspect('equal')
    ax.axis('square')

def plot_traj_3d(ref_data, data, font_size, data_name, ref_data_name):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_title('3D Trajectory')
    ax.plot3D(ref_data['x'] - ref_data['x'][0] , ref_data['y'] - ref_data['y'][0] ,ref_data['z'] - ref_data['z'][0] , marker=".",linestyle="None",markersize=1, color = "red",label=ref_data_name)
    ax.plot3D(data['x'] - ref_data['x'][0] , data['y'] - ref_data['y'][0] ,data['z'] - ref_data['z'][0] , marker="s",linestyle="None",markersize=1, alpha=0.3, color = "blue",label=data_name)
    ax.set_xlabel('East [m]', fontsize=font_size)
    ax.set_ylabel('North [m]', fontsize=font_size)
    ax.set_zlabel('Height [m]', fontsize=font_size)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_traj_3d_three(raw_data, ref_data, data, font_size, data_name, ref_data_name):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_title('3D Trajectory')
    ax.plot3D(raw_data['x'] - ref_data['x'][0] , raw_data['y'] - ref_data['y'][0] ,raw_data['z'] - ref_data['z'][0] , marker=".",linestyle="None",markersize=1, color = "red",label="gnss raw data(rtklib)")
    ax.plot3D(ref_data['x'] - ref_data['x'][0] , ref_data['y'] - ref_data['y'][0] ,ref_data['z'] - ref_data['z'][0] , marker=".",linestyle="None",markersize=1, color = "green",label=ref_data_name)
    ax.plot3D(data['x'] - ref_data['x'][0] , data['y'] - ref_data['y'][0] ,data['z'] - ref_data['z'][0] , marker="s",linestyle="None",markersize=1, alpha=0.3, color = "blue",label=data_name)
    ax.set_xlabel('East [m]', fontsize=font_size)
    ax.set_ylabel('North [m]', fontsize=font_size)
    ax.set_zlabel('Height [m]', fontsize=font_size)
    ax.legend(loc='upper right')
    ax.tick_params(labelsize=font_size)
    ax.grid()

def plot_traj_qual(xyz, qual, text, step, font_size):
    gnss_single = judge_qual(xyz,qual,1)
    gnss_differential = judge_qual(xyz,qual,2)
    gnss_fix = judge_qual(xyz,qual,4)
    gnss_float = judge_qual(xyz,qual,5)

    fig = plt.figure()
    ax_fix = fig.add_subplot(1, 1, 1)
    ax_fix.set_title('GNSS positioning solution')
    ax_fix.plot(gnss_single['x']-xyz['x'][0] , gnss_single['y']-xyz['y'][0] , marker=".",linestyle="None",markersize=1, color = "red",  label="gnss single")
    ax_fix.plot(gnss_differential['x']-xyz['x'][0] , gnss_differential['y']-xyz['y'][0] , marker=".",linestyle="None",markersize=1, color = "blue",  label="gnss differential")
    ax_fix.plot(gnss_fix['x']-xyz['x'][0] , gnss_fix['y']-xyz['y'][0] , marker=".",linestyle="None",markersize=1, color = "green",  label="gnss fix")
    ax_fix.plot(gnss_float['x']-xyz['x'][0] , gnss_float['y']-xyz['y'][0] , marker=".",linestyle="None",markersize=1, color = "yellow",  label="gnss float")
    cnt = 0
    for i in np.arange(0, len(text.index), step):
        if text[i] >= cnt * step:
            ax_fix.text(xyz['x'][i] - xyz['x'][0] , xyz['y'][i] - xyz['y'][0] , int(text[i]), size=16)
            cnt = cnt + 1
    ax_fix.set_xlabel('East [m]', fontsize=font_size)
    ax_fix.set_ylabel('North [m]', fontsize=font_size)
    ax_fix.legend(loc='upper right')
    ax_fix.tick_params(labelsize=font_size)
    ax_fix.grid()
    ax_fix.set_aspect('equal','box')
    ax_fix.axis('square')
