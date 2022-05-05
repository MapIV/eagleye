import csv
import argparse
from typing import List
import pandas as pd
import numpy as np
import sys
import quaternion
import math

import matplotlib.pyplot as plt
from tqdm import tqdm
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R



def set_ref_data(ref_data_tmp): # Creation of dataset with reference to column number
    set_data: List[float] = []
    for i, data in enumerate(ref_data_tmp):
        # print(i)
        if i == 0: continue
        # ref_data_sec: float = float(data[1])
        # ref_data_nsec: float = float(data[2])
        ref_data_time: float = float(data[1]) + float(data[2]) * 1e-9
        ref_data_x: float = float(data[4])
        ref_data_y: float = float(data[5])
        ref_data_z: float = float(data[6])
        ref_data_ori_x: float = float(data[8])
        ref_data_ori_y: float = float(data[9])
        ref_data_ori_z: float = float(data[10])
        ref_data_ori_w: float = float(data[11])
        set_data.append([ref_data_time,ref_data_x,ref_data_y,ref_data_z,ref_data_ori_x,ref_data_ori_y,ref_data_ori_z,ref_data_ori_w])
        df = pd.DataFrame(set_data,columns=['TimeStamp', 'x', 'y', 'z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])
    return df

def set_csv_data(csv_data_tmp): # Creation of dataset with reference to column number
    set_data: List[float] = []
    for i, data in enumerate(csv_data_tmp):
        if i == 0: continue
        # ref_data_sec: float = float(data[2])
        # ref_data_nsec: float = float(data[3])
        ref_data_time: float = float(data[2]) + float(data[3]) * 1e-9
        ref_data_x: float = float(data[5])
        ref_data_y: float = float(data[6])
        ref_data_z: float = float(data[7])
        ref_data_ori_x: float = float(data[8])
        ref_data_ori_y: float = float(data[9])
        ref_data_ori_z: float = float(data[10])
        ref_data_ori_w: float = float(data[11])
        set_data.append([ref_data_time,ref_data_x,ref_data_y,ref_data_z,ref_data_ori_x,ref_data_ori_y,ref_data_ori_z,ref_data_ori_w])
        df = pd.DataFrame(set_data,columns=['TimeStamp', 'x', 'y', 'z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])
    return df

def set_df(input): # Creation of dataset with reference to labels in df
    df = pd.read_csv(input)
    df = df[[".header.stamp.secs",
             ".header.stamp.nsecs",
             ".pose.pose.position.x",
             ".pose.pose.position.y",
             ".pose.pose.position.z",
             ".pose.pose.orientation.x",
             ".pose.pose.orientation.y",
             ".pose.pose.orientation.z",
             ".pose.pose.orientation.w",
             ]]
    df = df.rename(columns={'.header.stamp.secs': 'TimeStamp_sec',
                            '.header.stamp.nsecs': 'TimeStamp_nsec',
                            '.pose.pose.position.x': 'x',
                            '.pose.pose.position.y': 'y',
                            '.pose.pose.position.z': 'z',
                            '.pose.pose.orientation.x': 'ori_x',
                            '.pose.pose.orientation.y': 'ori_y',
                            '.pose.pose.orientation.z': 'ori_z',
                            '.pose.pose.orientation.w': 'ori_w',
                            })
    df['TimeStamp'] = df['TimeStamp_sec'] + df['TimeStamp_nsec'] * 1e-9
    df = df.reindex(columns=['TimeStamp', 'x', 'y', 'z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])
    return df

def set_ref_df(input): # Creation of dataset with reference to labels in df
    df = pd.read_csv(input)
    # print(df)
    df = df[[".header.stamp.secs",
             ".header.stamp.nsecs",
             ".pose.pose.position.x",
             ".pose.pose.position.y",
             ".pose.pose.position.z",
             "roll",
             "pitch",
             "yaw",
             "vel_x",
             "vel_y",
             "vel_z",
             ]]
    df = df.rename(columns={'.header.stamp.secs': 'TimeStamp_sec',
                            '.header.stamp.nsecs': 'TimeStamp_nsec',
                            '.pose.pose.position.x': 'x',
                            '.pose.pose.position.y': 'y',
                            '.pose.pose.position.z': 'z',
                            'roll': 'roll',
                            'pitch': 'pitch',
                            'yaw': 'yaw',
                            'vel_x': 'vel_x',
                            'vel_y': 'vel_y',
                            'vel_z': 'vel_z',
                            })
    df['TimeStamp'] = df['TimeStamp_sec'] + df['TimeStamp_nsec'] * 1e-9
    return df

def set_log_df(input,plane): # Creation of dataset with reference to labels in df
    df = pd.read_csv(input,delimiter=None, header='infer',  index_col=None, usecols=None)
    eagleye_df = df[["timestamp",
             "rtklib_nav.tow",
             "velocity_scale_factor.scale_factor",
             "velocity_scale_factor.correction_velocity.linear.x",
             "distance.distance",
             "rolling.rolling_angle",
             "pitching.pitching_angle",
             "heading_interpolate_3rd.heading_angle",
             "enu_vel.vector.x",
             "enu_vel.vector.y",
             "enu_vel.vector.z",
             "eagleye_pp_llh.latitude",
             "eagleye_pp_llh.longitude",
             "eagleye_pp_llh.altitude",
             'gga_llh.gps_qual',
             ]]

    eagleye_df = eagleye_df.rename(columns={'timestamp': 'TimeStamp_tmp',
                            'velocity_scale_factor.scale_factor': 'sf',
                            'velocity_scale_factor.correction_velocity.linear.x': 'velocity',
                            'distance.distance': 'distance',
                            'rolling.rolling_angle': 'roll_rad',
                            'pitching.pitching_angle': 'pitch_rad',
                            'heading_interpolate_3rd.heading_angle': 'yaw_rad',
                            'enu_vel.vector.x': 'vel_x',
                            'enu_vel.vector.y': 'vel_y',
                            'enu_vel.vector.z': 'vel_z',
                            'eagleye_pp_llh.latitude': 'latitude',
                            'eagleye_pp_llh.longitude': 'longitude',
                            'eagleye_pp_llh.altitude': 'altitude',
                            'gga_llh.gps_qual': 'qual',
                            })
    eagleye_df['TimeStamp'] = eagleye_df['TimeStamp_tmp'] * 10 ** (-9)
    rpy_df = set_rpy_deg(eagleye_df)
    eagleye_df['elapsed_time'] = eagleye_df['TimeStamp'] - eagleye_df['TimeStamp'][0]
    xyz = latlon_to_19(eagleye_df,plane)
    set_eagleye_df = pd.concat([eagleye_df,xyz, rpy_df],axis=1)
    return set_eagleye_df

def set_tf_xy(df,tf_x,tf_y):
    df['x'] = df['x'] + tf_x
    df['y'] = df['y'] + tf_y
    return df

def correct_anntenapos(eagleye_df,ref_df,tf_across,tf_along,tf_height):
    d = [[tf_across],[tf_along],[0]]
    for i in range(len(ref_df)):
        sinphi = math.sin(math.radians(ref_df['yaw'][i]))
        cosphi = math.cos(math.radians(ref_df['yaw'][i]))
        R = [[cosphi , sinphi , 0],[-sinphi , cosphi , 0],[0,0,0]]
        diff = np.dot(R, d)
        eagleye_df['x'][i] = eagleye_df['x'][i] + diff[0]
        eagleye_df['y'][i] = eagleye_df['y'][i] + diff[1]
    eagleye_df['z'] = eagleye_df['z'] + tf_height
    return eagleye_df

def set_rpy_deg(eagleye_df):
    set_heading_data: List[float] = []
    for i in range(len(eagleye_df)):
        roll_tmp = eagleye_df['roll_rad'][i]
        pitch_tmp = eagleye_df['pitch_rad'][i]
        yaw_tmp = change_anglel_limit(eagleye_df['yaw_rad'][i])
        roll = math.degrees(roll_tmp)
        pitch = math.degrees(pitch_tmp)
        yaw = math.degrees(yaw_tmp)
        set_heading_data.append([roll,pitch,yaw])
    df = pd.DataFrame(set_heading_data,columns=['roll','pitch','yaw'])
    return df

def change_anglel_limit(heading):
    while heading < 0 or math.pi * 2 < heading:
        if heading < 0:
            heading += math.pi * 2
        else:
            heading -= math.pi * 2
    return heading

def latlon_to_19(df,plane):
    phi0_deg , lambda0_deg = plane_table(plane)
    set_xy: List[float] = []
    for i in range(len(df)):
        phi_deg=df['latitude'][i]
        lambda_deg=df['longitude'][i]
        z=df['altitude'][i]
        """ 緯度経度を平面直角座標に変換する
        - input:
            (phi_deg, lambda_deg): 変換したい緯度・経度[度]（分・秒でなく小数であることに注意）
            (phi0_deg, lambda0_deg): 平面直角座標系原点の緯度・経度[度]（分・秒でなく小数であることに注意）
        - output:
        x: 変換後の平面直角座標[m]
        y: 変換後の平面直角座標[m]
        """
        # 緯度経度・平面直角座標系原点をラジアンに直す
        phi_rad = np.deg2rad(phi_deg)
        lambda_rad = np.deg2rad(lambda_deg)
        phi0_rad = np.deg2rad(phi0_deg)
        lambda0_rad = np.deg2rad(lambda0_deg)

        # 補助関数
        def A_array(n):
            A0 = 1 + (n**2)/4. + (n**4)/64.
            A1 = -     (3./2)*( n - (n**3)/8. - (n**5)/64. ) 
            A2 =     (15./16)*( n**2 - (n**4)/4. )
            A3 = -   (35./48)*( n**3 - (5./16)*(n**5) )
            A4 =   (315./512)*( n**4 )
            A5 = -(693./1280)*( n**5 )
            return np.array([A0, A1, A2, A3, A4, A5])

        def alpha_array(n):
            a0 = np.nan # dummy
            a1 = (1./2)*n - (2./3)*(n**2) + (5./16)*(n**3) + (41./180)*(n**4) - (127./288)*(n**5)
            a2 = (13./48)*(n**2) - (3./5)*(n**3) + (557./1440)*(n**4) + (281./630)*(n**5)
            a3 = (61./240)*(n**3) - (103./140)*(n**4) + (15061./26880)*(n**5)
            a4 = (49561./161280)*(n**4) - (179./168)*(n**5)
            a5 = (34729./80640)*(n**5)
            return np.array([a0, a1, a2, a3, a4, a5])

        # 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
        m0 = 0.9999 
        a = 6378137.
        F = 298.257222101

        # (1) n, A_i, alpha_iの計算
        n = 1. / (2*F - 1)
        A_array = A_array(n)
        alpha_array = alpha_array(n)

        # (2), S, Aの計算
        A_ = ( (m0*a)/(1.+n) )*A_array[0] # [m]
        S_ = ( (m0*a)/(1.+n) )*( A_array[0]*phi0_rad + np.dot(A_array[1:], np.sin(2*phi0_rad*np.arange(1,6))) ) # [m]

        # (3) lambda_c, lambda_sの計算
        lambda_c = np.cos(lambda_rad - lambda0_rad)
        lambda_s = np.sin(lambda_rad - lambda0_rad)

        # (4) t, t_の計算
        t = np.sinh( np.arctanh(np.sin(phi_rad)) - ((2*np.sqrt(n)) / (1+n))*np.arctanh(((2*np.sqrt(n)) / (1+n)) * np.sin(phi_rad)) )
        t_ = np.sqrt(1 + t*t)

        # (5) xi', eta'の計算
        xi2  = np.arctan(t / lambda_c) # [rad]
        eta2 = np.arctanh(lambda_s / t_)

        # (6) x, yの計算
        x = A_ * (xi2 + np.sum(np.multiply(alpha_array[1:],
                                        np.multiply(np.sin(2*xi2*np.arange(1,6)),
                                                    np.cosh(2*eta2*np.arange(1,6)))))) - S_ # [m]
        y = A_ * (eta2 + np.sum(np.multiply(alpha_array[1:],
                                            np.multiply(np.cos(2*xi2*np.arange(1,6)),
                                                        np.sinh(2*eta2*np.arange(1,6)))))) # [m]
        set_xy.append([y , x , z])
    xyz = pd.DataFrame(set_xy,columns=['x','y','z'])
    return xyz

def plane_table(plane):
    if plane == 1:
        phi0_deg = 33
        lambda0_deg = 129+30./60
    if plane == 2:
        phi0_deg = 33
        lambda0_deg = 131
    if plane == 3:
        phi0_deg = 36
        lambda0_deg = 132+10./60
    if plane == 4:
        phi0_deg = 33
        lambda0_deg = 133+30./60
    if plane == 5:
        phi0_deg = 36
        lambda0_deg = 134+20./60
    if plane == 6:
        phi0_deg = 36
        lambda0_deg = 136
    if plane == 7:
        phi0_deg = 36
        lambda0_deg = 137+10./60
    if plane == 8:
        phi0_deg = 36
        lambda0_deg = 138+30./60
    if plane == 9:
        phi0_deg = 36
        lambda0_deg = 139+50./60
    if plane == 10:
        phi0_deg = 40
        lambda0_deg = 140+50./60
    if plane == 11:
        phi0_deg = 44
        lambda0_deg = 140+15./60
    if plane == 12:
        phi0_deg = 44
        lambda0_deg = 142+15./60
    if plane == 13:
        phi0_deg = 44
        lambda0_deg = 144+15./60
    if plane == 14:
        phi0_deg = 26
        lambda0_deg = 142
    if plane == 15:
        phi0_deg = 26
        lambda0_deg = 127+30./60
    if plane == 16:
        phi0_deg = 26
        lambda0_deg = 124+30./60
    if plane == 17:
        phi0_deg = 26
        lambda0_deg = 131
    if plane == 18:
        phi0_deg = 26
        lambda0_deg = 136
    if plane == 19:
        phi0_deg = 26
        lambda0_deg = 154
    return phi0_deg , lambda0_deg


def sync_time(long_data,short_data,sync_threshold_time_data,leap_time_data): # Time synchronization
    sync_index = np.zeros(len(short_data['TimeStamp']))

    first_flag_data = 0
    data_time_tmp = np.zeros(len(short_data['TimeStamp']))
    data_x_tmp = np.zeros(len(short_data['TimeStamp']))
    data_y_tmp = np.zeros(len(short_data['TimeStamp']))
    data_z_tmp = np.zeros(len(short_data['TimeStamp']))
    data_ori_x_tmp = np.zeros(len(short_data['TimeStamp']))
    data_ori_y_tmp = np.zeros(len(short_data['TimeStamp']))
    data_ori_z_tmp = np.zeros(len(short_data['TimeStamp']))
    data_ori_w_tmp = np.zeros(len(short_data['TimeStamp']))
    data_elapsed_time_tmp = np.zeros(len(short_data['TimeStamp']))
    data_distance_tmp = np.zeros(len(short_data['TimeStamp']))
    data_qual_tmp = np.zeros(len(short_data['TimeStamp']))
    set_data_df: List[float] = []
    set_data_ori: List[float] = []
    set_data_rpy: List[float] = []
    set_data_velocity: List[float] = []
    set_data_vel: List[float] = []
    set_data_distance: List[float] = []
    set_data_qual: List[float] = []
    first_flag_ref = 0
    ref_time_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_x_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_y_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_z_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_ori_x_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_ori_y_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_ori_z_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_ori_w_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_elapsed_time_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_distance_tmp = np.zeros(len(short_data['TimeStamp']))
    ref_qual_tmp = np.zeros(len(short_data['TimeStamp']))
    set_ref_data_df: List[float] = []
    set_ref_data_ori: List[float] = []
    set_ref_data_rpy: List[float] = []
    set_ref_data_velocity: List[float] = []
    set_ref_data_vel: List[float] = []
    set_ref_data_distance: List[float] = []
    set_ref_data_qual: List[float] = []
    for i in range(len(short_data)):
        if i == 0: continue
        time_tmp: List[float] = []
        time_tmp = abs(short_data.iloc[i]['TimeStamp']-long_data['TimeStamp'] + leap_time_data)
        sync_threshold_time_data_tmp = min(time_tmp)
        if sync_threshold_time_data_tmp < sync_threshold_time_data:
            sync_index[i] = np.argmin(time_tmp) # 2解目min計算
            num = int(sync_index[i])
            data_time_tmp = short_data.iloc[i]['TimeStamp']
            data_x_tmp = short_data.iloc[i]['x']
            data_y_tmp = short_data.iloc[i]['y']
            data_z_tmp = short_data.iloc[i]['z']
            if (first_flag_data == 0):
                first_time_data = short_data.iloc[i]['TimeStamp']
                first_flag_data = 1
            data_elapsed_time_tmp = short_data.iloc[i]['TimeStamp'] - first_time_data
            set_data_df.append([data_elapsed_time_tmp,data_time_tmp,data_x_tmp,data_y_tmp,data_z_tmp])
            if 'ori_x' in short_data.columns:
                data_ori_x_tmp = short_data.iloc[i]['ori_x']
                data_ori_y_tmp = short_data.iloc[i]['ori_y']
                data_ori_z_tmp = short_data.iloc[i]['ori_z']
                data_ori_w_tmp = short_data.iloc[i]['ori_w']
                set_data_ori.append([data_ori_x_tmp,data_ori_y_tmp,data_ori_z_tmp,data_ori_w_tmp])
            if 'roll' in short_data.columns:
                data_roll_tmp = short_data.iloc[i]['roll']
                data_pitch_tmp = short_data.iloc[i]['pitch']
                data_yaw_tmp = short_data.iloc[i]['yaw']
                set_data_rpy.append([data_roll_tmp,data_pitch_tmp,data_yaw_tmp])
            if 'velocity' in short_data.columns:
                velocity_tmp = short_data.iloc[i]['velocity']
                set_data_velocity.append([velocity_tmp])
            if 'vel_x' in short_data.columns:
                data_vel_x_tmp = short_data.iloc[i]['vel_x']
                data_vel_y_tmp = short_data.iloc[i]['vel_y']
                data_vel_z_tmp = short_data.iloc[i]['vel_z']
                set_data_vel.append([data_vel_x_tmp,data_vel_y_tmp,data_vel_z_tmp])
            if 'distance' in short_data.columns:
                data_distance_tmp = short_data.iloc[i]['distance']
                set_data_distance.append([data_distance_tmp])
            if 'qual' in short_data.columns:
                data_qual_tmp = short_data.iloc[i]['qual']
                set_data_qual.append([data_qual_tmp])

            ref_time_tmp = long_data.iloc[num]['TimeStamp']
            ref_x_tmp = long_data.iloc[num]['x']
            ref_y_tmp = long_data.iloc[num]['y']
            ref_z_tmp = long_data.iloc[num]['z']
            if (first_flag_ref == 0):
                first_time_ref = long_data.iloc[num]['TimeStamp']
                first_flag_ref = 1
            ref_elapsed_time_tmp = long_data.iloc[num]['TimeStamp'] - first_time_ref
            set_ref_data_df.append([ref_elapsed_time_tmp,ref_time_tmp,ref_x_tmp,ref_y_tmp,ref_z_tmp])
            if 'ori_x' in long_data.columns:
                ref_ori_x_tmp = long_data.iloc[num]['ori_x']
                ref_ori_y_tmp = long_data.iloc[num]['ori_y']
                ref_ori_z_tmp = long_data.iloc[num]['ori_z']
                ref_ori_w_tmp = long_data.iloc[num]['ori_w']
                set_ref_data_ori.append([ref_ori_x_tmp,ref_ori_y_tmp,ref_ori_z_tmp,ref_ori_w_tmp])
            if 'roll' in long_data.columns:
                ref_roll_tmp = long_data.iloc[num]['roll']
                ref_pitch_tmp = long_data.iloc[num]['pitch']
                ref_yaw_tmp = long_data.iloc[num]['yaw']
                set_ref_data_rpy.append([ref_roll_tmp,ref_pitch_tmp,ref_yaw_tmp])
            if 'velocity' in long_data.columns:
                velocity_tmp = long_data.iloc[num]['velocity']
                set_ref_data_velocity.append([velocity_tmp])
            if 'vel_x' in long_data.columns:
                ref_vel_x_tmp = long_data.iloc[num]['vel_x']
                ref_vel_y_tmp = long_data.iloc[num]['vel_y']
                ref_vel_z_tmp = long_data.iloc[num]['vel_z']
                set_ref_data_vel.append([ref_vel_x_tmp,ref_vel_y_tmp,ref_vel_z_tmp])
            if 'distance' in long_data.columns:
                ref_distance_tmp = long_data.iloc[num]['distance']
                set_ref_data_distance.append([ref_distance_tmp])
            if 'qual' in long_data.columns:
                ref_qual_tmp = long_data.iloc[num]['qual']
                set_ref_data_qual.append([ref_qual_tmp])


    data_ori = pd.DataFrame()
    data_rpy = pd.DataFrame()
    data_velocity = pd.DataFrame()
    data_vel = pd.DataFrame()
    data_distance = pd.DataFrame()
    data_qual = pd.DataFrame()
    data_df = pd.DataFrame(set_data_df,columns=['elapsed_time','TimeStamp', 'x', 'y', 'z'])
    if 'ori_x' in short_data.columns:
        data_ori = pd.DataFrame(set_data_ori,columns=['ori_x', 'ori_y', 'ori_z', 'ori_w'])
    if 'roll' in short_data.columns:
        data_rpy = pd.DataFrame(set_data_rpy,columns=['roll', 'pitch', 'yaw'])
    if 'velocity' in short_data.columns:
        data_velocity = pd.DataFrame(set_data_velocity,columns=['velocity'])
    if 'vel_x' in short_data.columns:
        data_vel = pd.DataFrame(set_data_vel,columns=['vel_x', 'vel_y', 'vel_z'])
    if 'distance' in short_data.columns:
        data_distance = pd.DataFrame(set_data_distance,columns=['distance'])
    if 'qual' in short_data.columns:
        data_qual = pd.DataFrame(set_data_qual,columns=['qual'])
    data_df_output = pd.concat([data_df,data_ori,data_rpy,data_velocity,data_vel,data_distance,data_qual],axis=1)

    ref_ori = pd.DataFrame()
    ref_rpy = pd.DataFrame()
    ref_velocity = pd.DataFrame()
    ref_vel = pd.DataFrame()
    ref_distance = pd.DataFrame()
    ref_qual = pd.DataFrame()
    ref_df = pd.DataFrame(set_ref_data_df,columns=['elapsed_time','TimeStamp', 'x', 'y', 'z'])
    if 'ori_x' in long_data.columns:
        ref_ori = pd.DataFrame(set_ref_data_ori,columns=['ori_x', 'ori_y', 'ori_z', 'ori_w'])
    if 'roll' in long_data.columns:
        ref_rpy = pd.DataFrame(set_ref_data_rpy,columns=['roll', 'pitch', 'yaw'])
    if 'velocity' in long_data.columns:
        ref_velocity = pd.DataFrame(set_ref_data_velocity,columns=['velocity'])
    if 'vel_x' in long_data.columns:
        ref_vel = pd.DataFrame(set_ref_data_vel,columns=['vel_x', 'vel_y', 'vel_z'])
    if 'distance' in long_data.columns:
        ref_distance = pd.DataFrame(set_ref_data_distance,columns=['distance'])
    if 'qual' in long_data.columns:
        ref_qual = pd.DataFrame(set_ref_data_qual,columns=['qual'])
    
    ref_df_output = pd.concat([ref_df,ref_ori,ref_rpy,ref_velocity,ref_vel,ref_distance,ref_qual],axis=1)
   
    return ref_df_output , data_df_output



def calc_error_xyz(ref_df,data_df):
    error_xyz = pd.DataFrame()
    error_xyz['elapsed_time'] = ref_df['elapsed_time']
    error_xyz['TimeStamp'] = ref_df['TimeStamp'] - data_df['TimeStamp']
    error_xyz['x'] = data_df['x'] - ref_df['x']
    error_xyz['y'] = data_df['y'] - ref_df['y']
    error_xyz['z'] = data_df['z'] - ref_df['z']
    error_xyz['2d'] = (error_xyz['x']**2 + error_xyz['y']**2)**0.5
    
    set_aa: List[float] = []
    for i in range(len(ref_df)):
        across =  error_xyz.iloc[i]['x'] * math.cos(math.radians(ref_df.iloc[i]['yaw'])) - error_xyz.iloc[i]['y'] * math.sin(math.radians(ref_df.iloc[i]['yaw']))
        along =  error_xyz.iloc[i]['x'] * math.sin(math.radians(ref_df.iloc[i]['yaw'])) + error_xyz.iloc[i]['y'] * math.cos(math.radians(ref_df.iloc[i]['yaw']))
        set_aa.append([across,along])
    error_aa = pd.DataFrame(set_aa,columns=['across','along'])

    error = pd.concat([error_xyz,error_aa],axis=1)
    return error
  

def calc_TraRate(diff_2d , eval_step_max): # Calculation of error , x_label , ErrTra_Rater rate
    ErrTra_cnt: List[int] = []
    set_ErrTra: List[float] = []
    ErrTra_cnt = len(diff_2d)
    step = 0.1

    for j in range(0 , eval_step_max , 1):
        cnt = 0
        eval_step = j * step
        for i, ErrTra in tqdm(enumerate(diff_2d)):
            if ErrTra < eval_step:
                cnt = cnt + 1
        rate = cnt / ErrTra_cnt
        set_ErrTra.append([eval_step,rate,rate *100])
    ErrTra_Rate = pd.DataFrame(set_ErrTra,columns=['x_label','ErrTra_Rate','ErrTra'])
    return ErrTra_Rate


def quaternion_to_euler_zyx(df):
    set_eular_angle: List[float] = []
    for i in range(len(df)):
        q = [df.iloc[i]['ori_w'],df.iloc[i]['ori_x'],df.iloc[i]['ori_y'],df.iloc[i]['ori_z']]
        r = R.from_quat([q[0], q[1], q[2], q[3]])
        roll = r.as_euler('zyx', degrees=True)[0]
        pitch = r.as_euler('zyx', degrees=True)[1]
        yaw = r.as_euler('zyx', degrees=True)[2]
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        set_eular_angle.append([roll_deg,pitch_deg,yaw_deg])
    euler_angle = pd.DataFrame(set_eular_angle,columns=['roll','pitch','yaw'])
    return euler_angle

def calc_error_rpy(ref_angle,eagleye_angle):
    error_rpy = pd.DataFrame()
    error_rpy['roll'] = eagleye_angle['roll'] - ref_angle['roll']
    error_rpy['pitch'] = eagleye_angle['pitch'] - ref_angle['pitch']
    error_rpy['yaw'] = eagleye_angle['yaw'] - ref_angle['yaw']
    return error_rpy

def calc_velocity(df):
    velocity = pd.DataFrame()
    velocity['velocity'] = (df['vel_x'] ** 2 + df['vel_y'] ** 2 + df['vel_z'] ** 2) ** 0.5
    return velocity

def calc_velocity_error(eagleye_velocity,ref_velocity):
    error_velocity = pd.DataFrame()
    error_velocity['velocity'] = eagleye_velocity['velocity'] - ref_velocity['velocity']
    return error_velocity

def judge_qual(df,qual_num):
    set_output_df: List[float] = []
    for i in range(len(df)):
        if qual_num == df['qual'][i]:
            time = df['elapsed_time'][i]
            x = df['x'][i]
            y = df['y'][i]
            z = df['z'][i]
            qual = df['qual'][i]
            set_output_df.append([time,x,y,z,qual])
    output_df = pd.DataFrame(set_output_df,columns=['elapsed_time','x','y','z','qual'])
    return output_df

def clac_dr(eagleye_df,ref_df,distance_length,distance_step):
    # distance_length = 100
    # distance_step = 50
    last_distance = 0
    set_calc_error: List[float] = []
    for i in range(len(eagleye_df)):
        if i == 0: continue
        if last_distance + distance_step < eagleye_df['distance'][i]:
            start_distance = eagleye_df['distance'][i]
            start_pos_x = ref_df['x'][i]
            start_pos_y = ref_df['y'][i]
            previous_pos_x = 0
            previous_pos_y = 0
            last_distance = start_distance
            last_time = eagleye_df['TimeStamp'][i]
            for j in range(i,len(eagleye_df)):
                if eagleye_df['x'][j] == 0 and eagleye_df['y'][j] == 0: break
                if eagleye_df['distance'][j] - start_distance < distance_length:
                    dr_pos_x = previous_pos_x + eagleye_df['vel_x'][j] * (eagleye_df['TimeStamp'][j] - last_time)
                    dr_pos_y = previous_pos_y + eagleye_df['vel_y'][j] * (eagleye_df['TimeStamp'][j] - last_time)
                    previous_pos_x = dr_pos_x
                    previous_pos_y = dr_pos_y
                    last_time = eagleye_df['TimeStamp'][j]
                    distance = eagleye_df['distance'][j] - start_distance
                    absolute_pos_x = ref_df['x'][j] - start_pos_x
                    absolute_pos_y = ref_df['y'][j] - start_pos_y
                else:
                    error_x = absolute_pos_x - dr_pos_x
                    error_y = absolute_pos_y - dr_pos_y
                    error_2d_fabs = math.fabs(error_x ** 2 + error_y ** 2)
                    error_2d= math.sqrt(error_2d_fabs)
                    set_calc_error.append([start_distance,distance,absolute_pos_x,dr_pos_x,absolute_pos_y,dr_pos_y,error_x,error_y,error_2d])
                    break
    calc_error = pd.DataFrame(set_calc_error,columns=['start_distance','distance','absolute_pos_x','absolute_pos_y','dr_pos_x','dr_pos_y','error_x','error_y','error_2d'])
    return calc_error


def plot_each(ax, eagleye, ref, elem, title, y_label,ref_data_name):
    if elem in ref.columns:
        ax.plot(ref['elapsed_time'] , ref[elem] , marker="o", linestyle="None",markersize=1, color = "green",  label=ref_data_name)
    if elem in eagleye.columns:
        ax.plot(eagleye['elapsed_time'] , eagleye[elem] , marker="s", linestyle="None",markersize=1, alpha=0.3, color = "blue",  label="eagleye")
    ax.set_xlabel('time [s]')
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.grid()

def plot_6DoF(eagleye, ref,ref_data_name):
    fig1 = plt.figure()
    ax_x = fig1.add_subplot(2, 3, 1)
    ax_y = fig1.add_subplot(2, 3, 2)
    ax_z = fig1.add_subplot(2, 3, 3)
    ax_roll = fig1.add_subplot(2, 3, 4)
    ax_pitch = fig1.add_subplot(2, 3, 5)
    ax_yaw = fig1.add_subplot(2, 3, 6)
    plot_each(ax_x, eagleye, ref, 'x', 'X (East-West)','East [m]',ref_data_name)
    plot_each(ax_y, eagleye, ref, 'y', 'Y (North-South)','North [m]',ref_data_name)
    plot_each(ax_z, eagleye, ref, 'z', 'Z (Height)','Height [m]',ref_data_name)
    plot_each(ax_roll, eagleye, ref, 'roll', 'Roll' , 'Roll [deg]',ref_data_name)
    plot_each(ax_pitch, eagleye, ref, 'pitch', 'Pitch', 'Pitch [deg]',ref_data_name)
    plot_each(ax_yaw, eagleye, ref, 'yaw', 'Yaw', 'Yaw [deg]',ref_data_name)

def plot_each_error(ax, error_data, elem, title, y_label):
    ax.plot(error_data['elapsed_time'] , error_data[elem] , marker="s", linestyle="None",markersize=1, alpha=0.3, color = "blue")
    ax.set_xlabel('time [s]')
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.grid()

def plot_one(ax, error_data, x_data, y_data, title, x_label, y_label, line_style):
    ax.plot(error_data[x_data] , error_data[y_data] , marker="s", linestyle=line_style, markersize=1, alpha=0.3, color = "blue")
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.grid()


def plot_error_6DoF(error_data,ref_data_name):
    fig2 = plt.figure()
    fig2.suptitle(ref_data_name + ' - eagleye Error')
    ax_x = fig2.add_subplot(2, 3, 1)
    ax_y = fig2.add_subplot(2, 3, 2)
    ax_z = fig2.add_subplot(2, 3, 3)
    ax_roll = fig2.add_subplot(2, 3, 4)
    ax_pitch = fig2.add_subplot(2, 3, 5)
    ax_yaw = fig2.add_subplot(2, 3, 6)
    plot_each_error(ax_x, error_data, 'x', 'X (East-West) Error','East error [m]')
    plot_each_error(ax_y, error_data, 'y', 'Y (North-South) Error','North error [m]')
    plot_each_error(ax_z, error_data, 'z', 'Z (Height) Error','Height error [m]')
    plot_each_error(ax_roll, error_data, 'roll', 'Roll Error' , 'Roll error, [deg]')
    plot_each_error(ax_pitch, error_data, 'pitch', 'Pitch Error', 'Pitch error [deg]')
    plot_each_error(ax_yaw, error_data, 'yaw', 'Yaw Error', 'Yaw error [deg]')

def plot_error(error_data,ref_data_name):
    fig4 = plt.figure()
    fig4.suptitle(ref_data_name + ' - eagleye Error')
    ax_x = fig4.add_subplot(2, 2, 1)
    ax_y = fig4.add_subplot(2, 2, 2)
    ax_across = fig4.add_subplot(2, 2, 3)
    ax_along = fig4.add_subplot(2, 2, 4)
    plot_each_error(ax_x, error_data, 'x', 'X (East-West) Error','East error [m]')
    plot_each_error(ax_y, error_data, 'y', 'Y (North-South) Error','North error [m]')
    plot_each_error(ax_across, error_data, 'across', 'Across Error','across error [m]')
    plot_each_error(ax_along, error_data, 'along', 'Along Error','along error [m]')

def plot_error_distributiln(error_data,ref_data_name):
    fig5 = plt.figure()
    fig5.suptitle(ref_data_name + ' - eagleye Error')
    ax_xy = fig5.add_subplot(1, 2, 1)
    ax_aa = fig5.add_subplot(1, 2, 2)
    plot_one(ax_xy, error_data, 'x', 'y', 'X-Y error', 'x error [m]', 'y error [m]', 'None')
    plot_one(ax_aa, error_data, 'across', 'along', 'Across-Along error', 'across error [m]', 'along error [m]', 'None')

def plot_traj(ax, ref_data, data, x_data, y_data, title, x_label, y_label,ref_data_name):
    ax.plot(ref_data[x_data]-ref_data[x_data][0] , ref_data[y_data]-ref_data[y_data][0] , marker=".",linestyle="None",markersize=1, color = "red",  label=ref_data_name)
    ax.plot(data[x_data]-ref_data[x_data][0] , data[y_data]-ref_data[y_data][0] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.grid()
    ax.set_aspect('equal')
    ax.axis('square')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-ref", "--ref", help="Path to the ref data")
    parser.add_argument("-df_ref", "--df_ref", help="Path to the ref data by df")
    parser.add_argument("-df_ref_ros", "--input_df_ref", help="Path to the ref data by df")
    parser.add_argument("-csv", "--input_csv", help="Path to the csv")
    parser.add_argument("-df_csv", "--input_df_csv", help="Path to the csv by df")
    parser.add_argument("-log", "--input_log_csv", help="Path to the csv by df")
    parser.add_argument("-p", "--plane_num",help="Plane Cartesian coordinate system number")
    parser.add_argument("-s", "--sync_threshold_time",help="Gap time to be regarded as synchronous")
    parser.add_argument("-l", "--leap_time",help="Time offset correction")
    parser.add_argument("-tf_x", "--tf_x_param",help="tf offset correction")
    parser.add_argument("-tf_y", "--tf_y_param",help="tf offset correction")
    parser.add_argument("-tf_across", "--tf_across_param",help="tf offset correction")
    parser.add_argument("-tf_along", "--tf_along_param",help="tf offset correction")
    parser.add_argument("-tf_height", "--tf_height_param",help="tf offset correction")
    parser.add_argument("-dr_l", "--distance_length_param",help="tf offset correction")
    parser.add_argument("-df_s", "--distance_step_param",help="tf offset correction")
    parser.add_argument("-ref_name", "--ref_data_name_param",help="ref data name")
    args = parser.parse_args()

    # default_param
    plane = 7
    sync_threshold_time_data = 0.005
    leap_time_data = 0.0
    tf_x = 0.0
    tf_y = 0.0
    tf_across = 0.0
    tf_along = 0.0
    tf_height = 0.0
    distance_length = 100
    distance_step = 50
    ref_data_name = 'ref data'
    eval_step_max = 15 # Maximum value of error to be evaluated default = 15 = 1.5[m]

    # set param
    if args.plane_num != None:
        plane = float(args.plane_num)

    if args.sync_threshold_time != None:
        sync_threshold_time_data = float(args.sync_threshold_time)

    if args.leap_time != None:
        leap_time_data = float(args.leap_time)

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

    if args.distance_length_param != None:
        distance_length = float(args.distance_length_param)

    if args.distance_step_param != None:
        distance_step = float(args.distance_step_param)

    if args.ref_data_name_param != None:
        ref_data_name = args.ref_data_name_param
    
    print('plane',plane)
    print('sync_threshold_time_data',sync_threshold_time_data)
    print('leap_time_data',leap_time_data)
    print('tf_x',tf_x)
    print('tf_y',tf_y)
    print('tf_across',tf_across)
    print('tf_along',tf_along)
    print('tf_height',tf_height)
    print('distance_length',distance_length)
    print('distance_step',distance_step)
    print('ref_data_name',ref_data_name)

    # set data
    ref_data_df = pd.DataFrame()
    csv_data_df = pd.DataFrame()

    if args.ref != None:
        input_ref_path: str = args.ref
        with open(input_ref_path) as f:
            reader = csv.reader(f)
            ref_data_tmp = [row for row in reader]
            ref_data_df = set_ref_data(ref_data_tmp)
            print("set ref_data")

    if args.input_csv != None:
        input_csv_path: str = args.input_csv
        with open(input_csv_path) as r:
            reader = csv.reader(r)
            csv_data_tmp = [row for row in reader]
            csv_data_df = set_csv_data(csv_data_tmp)
            print("set csv_data")
        
    if args.df_ref != None:
        ref_data_df = set_ref_df(args.df_ref)
        print("set ref_data")

    if args.input_df_ref != None:
        ref_data_df = set_df(args.df_ref)
        print("set ref_data")

    if args.input_df_csv != None:
        csv_data_df = set_df(args.input_df_csv)
        print("set csv_data")

    if args.input_log_csv != None:
        csv_data_df = set_log_df(args.input_log_csv,plane)
        print("set csv_data")
    
    if tf_x != 0 or tf_y != 0:
        ref_data_df = set_tf_xy(ref_data_df,tf_x,tf_y)
        print('set tf xy')
    
    # syne time
    ref_df = pd.DataFrame()
    data_df = pd.DataFrame()
    print("start sync_time")
    ref_df , data_df = sync_time(ref_data_df,csv_data_df,sync_threshold_time_data,leap_time_data)
    print("finished sync_time")

    # Quaternion to Euler conversion
    eagleye_angle = pd.DataFrame()
    if 'ori_x' in data_df.columns and 'ori_y' in data_df.columns and 'ori_z' in data_df.columns and 'ori_w' in data_df.columns:
        eagleye_ori_df = pd.concat([data_df['ori_x'],data_df['ori_y'],data_df['ori_z'],data_df['ori_w']],axis=1)
        eagleye_angle = quaternion_to_euler_zyx(eagleye_ori_df)

    
    ref_angle = pd.DataFrame()
    if 'ori_x' in ref_df.columns and 'ori_y' in ref_df.columns and 'ori_z' in ref_df.columns and 'ori_w' in ref_df.columns:
        ref_ori_df = pd.concat([ref_df['ori_x'],ref_df['ori_y'],ref_df['ori_z'],ref_df['ori_w']],axis=1)
        ref_angle = quaternion_to_euler_zyx(ref_ori_df)
        print("finished calc ref angle")

    # calc ref velocity
    ref_velocity = pd.DataFrame()
    if 'vel_x' in ref_df.columns and 'vel_y' in ref_df.columns and 'vel_z' in ref_df.columns and not 'velocity' in ref_df.columns:
        ref_vel_df = pd.concat([ref_df['vel_x'],ref_df['vel_y'],ref_df['vel_z']],axis=1)
        ref_velocity = calc_velocity(ref_vel_df)
    
    ref_plot_df = pd.concat([ref_df,ref_angle,ref_velocity],axis=1)
    eagleye_plot_df = pd.concat([data_df,eagleye_angle],axis=1)

    # correct anntena position
    if tf_across != 0 or tf_along != 0 or tf_height != 0:
        eagleye_plot_df = correct_anntenapos(eagleye_plot_df,ref_plot_df,tf_across,tf_along,tf_height)
        print('set tf')
    
    # plot 6dof
    plot_6DoF(eagleye_plot_df, ref_plot_df,ref_data_name)
    
    # judge GNSS quality
    if 'qual' in eagleye_plot_df.columns:
        gnss_single = judge_qual(eagleye_plot_df,1)
        gnss_differential = judge_qual(eagleye_plot_df,2)
        gnss_fix = judge_qual(eagleye_plot_df,4)
        gnss_float = judge_qual(eagleye_plot_df,5)

    # twist Performance Evaluation
    if 'vel_x' in data_df.columns and 'vel_y' in data_df.columns and 'distance' in data_df.columns:
        print("start calc dr")
        eagleye_dr_df = pd.concat([data_df['TimeStamp'],data_df['distance'],data_df['x'],data_df['y'],data_df['vel_x'],data_df['vel_y']],axis=1)
        calc_error = clac_dr(eagleye_dr_df,ref_df,distance_length,distance_step)
        print("finished calc dr")

        dr_error_2d = calc_error['error_2d'].values.tolist()
        ErrTra_dr_df = calc_TraRate(dr_error_2d , eval_step_max)

        fig15 = plt.figure()
        ax_dr = fig15.add_subplot(2, 1, 1)
        plot_one(ax_dr, calc_error, 'start_distance', 'error_2d', 'DR', 'start distance [m]', '2D Error [m]', '-')
        
        ax_trarate_dr = fig15.add_subplot(2, 1, 2)
        plot_one(ax_trarate_dr, ErrTra_dr_df, 'x_label', 'ErrTra', 'Cumulative Error Distribution dr', '2D error [m]', 'Rate [%]', '-')
        ax_trarate_dr.set_xscale('log') 
    
    # calc 6dof error
    Error_data = pd.DataFrame()
    ref_data_xyz = pd.concat([ref_plot_df['elapsed_time'],ref_plot_df['TimeStamp'],ref_plot_df['x'],ref_plot_df['y'],ref_plot_df['z'],ref_plot_df['yaw']],axis=1)
    eagleye_data_xyz = pd.concat([eagleye_plot_df['elapsed_time'],eagleye_plot_df['TimeStamp'],eagleye_plot_df['x'],eagleye_plot_df['y'],eagleye_plot_df['z']],axis=1)
    Error_data = calc_error_xyz(ref_data_xyz,eagleye_data_xyz)
    print(Error_data)
    print("finished calc_error_xyz")

    ref_data_rpy = pd.concat([ref_plot_df['roll'],ref_plot_df['pitch'],ref_plot_df['yaw']],axis=1)
    eagleye_data_rpy = pd.concat([eagleye_plot_df['roll'],eagleye_plot_df['pitch'],eagleye_plot_df['yaw']],axis=1)
    error_rpy = calc_error_rpy(ref_data_rpy,eagleye_data_rpy)
    print(error_rpy)
    print("finished calc error rpy")

    error_velocity = pd.DataFrame()
    if 'velocity' in data_df.columns and 'velocity' in ref_velocity.columns:
        error_velocity = calc_velocity_error(data_df,ref_velocity)
    
    error_plot_df = pd.concat([Error_data,error_rpy,error_velocity],axis=1)

    plot_error_6DoF(error_plot_df,ref_data_name)
    plot_error(error_plot_df,ref_data_name)
    plot_error_distributiln(error_plot_df,ref_data_name)

    # plot Cumulative Error Distribution
    diff_2d: List[float] = []
    if '2d' in Error_data.columns:
        diff_2d = Error_data['2d'].values.tolist()
        print(diff_2d)
        ErrTra_Rate = calc_TraRate(diff_2d , eval_step_max)

        fig = plt.figure()
        ax1 = fig.add_subplot(1, 1, 1)
        plot_one(ax1, ErrTra_Rate, 'x_label', 'ErrTra', 'Cumulative Error Distribution', '2D error [m]', 'Rate [%]', '-')
        ax1.set_xscale('log') 

    #  plot velocity
    if 'velocity' in ref_velocity.columns or 'velocity' in data_df.columns:
        fig11 = plt.figure()
        ax_vel = fig11.add_subplot(2, 1, 1)
        ax_vel.set_title('Velocity')
        ax_vel.plot(ref_df['elapsed_time'] , ref_velocity['velocity'] , marker=".",linestyle="None",markersize=1, color = "red",  label=ref_data_name)
        ax_vel.plot(data_df['elapsed_time'] , data_df['velocity'] , marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
        ax_vel.set_xlabel('time [s]')
        ax_vel.set_ylabel('Velocity [m/s]')
        ax_vel.legend(loc='upper right')
        ax_vel.grid()

    if 'velocity' in error_plot_df.columns:
        ax_err_vel = fig11.add_subplot(2, 1, 2)
        fig11.suptitle(ref_data_name + ' - eagleye Error')
        plot_one(ax_err_vel, error_plot_df, 'elapsed_time', 'velocity', 'Velocity Error', 'time [s]', 'Velocity error[m/s]', 'None')


    # plot 2D trajectory
    if 'qual' in eagleye_plot_df.columns:
        fig14 = plt.figure()
        ax_traj = fig14.add_subplot(1, 2, 1)
        plot_traj(ax_traj, ref_plot_df, eagleye_plot_df, 'x', 'y', '2D Trajectory', 'East [m]', 'North [m]',ref_data_name)

        gnss_single = judge_qual(eagleye_plot_df,1)
        gnss_differential = judge_qual(eagleye_plot_df,2)
        gnss_fix = judge_qual(eagleye_plot_df,4)
        gnss_float = judge_qual(eagleye_plot_df,5)
        ax_fix = fig14.add_subplot(1, 2, 2)
        ax_fix.set_title('GNSS positioning solution')
        ax_fix.plot(gnss_single['x']-ref_plot_df['x'][0] , gnss_single['y']-ref_plot_df['y'][0] , marker=".",linestyle="None",markersize=1, color = "red",  label="gnss single")
        ax_fix.plot(gnss_differential['x']-ref_plot_df['x'][0] , gnss_differential['y']-ref_plot_df['y'][0] , marker=".",linestyle="None",markersize=1, color = "blue",  label="gnss differential")
        ax_fix.plot(gnss_fix['x']-ref_plot_df['x'][0] , gnss_fix['y']-ref_plot_df['y'][0] , marker=".",linestyle="None",markersize=1, color = "green",  label="gnss fix")
        ax_fix.plot(gnss_float['x']-ref_plot_df['x'][0] , gnss_float['y']-ref_plot_df['y'][0] , marker=".",linestyle="None",markersize=1, color = "yellow",  label="gnss float")    
        ax_fix.set_xlabel('East [m]')
        ax_fix.set_ylabel('North [m]')
        ax_fix.legend(loc='upper right')
        ax_fix.grid()
        ax_fix.set_aspect('equal','box')
        ax_fix.axis('square')
    else:
        fig14 = plt.figure()
        ax_traj = fig14.add_subplot(1, 1, 1)
        plot_traj(ax_traj, ref_plot_df, eagleye_plot_df, 'x', 'y', '2D Trajectory', 'East [m]', 'North [m]',ref_data_name)

    # plot 3d trajectory
    fig20 = plt.figure()
    ax_3d = fig20.add_subplot(projection='3d')
    ax_3d.set_title('3D Trajectory')
    ax_3d.plot3D(ref_df['x'] , ref_df['y'] ,ref_df['z'] , marker=".",linestyle="None",markersize=1, color = "red",label=ref_data_name)
    ax_3d.plot3D(data_df['x'] , data_df['y'] ,data_df['z'] , marker="s",linestyle="None",markersize=1, alpha=0.3, color = "blue",label="eagleye")
    ax_3d.set_xlabel('East [m]')
    ax_3d.set_ylabel('North [m]')
    ax_3d.set_zlabel('Height [m]')
    ax_3d.legend(loc='upper right')
    ax_3d.grid()

    plt.show()
    