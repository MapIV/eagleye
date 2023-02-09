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

# preprocess.py
# Author MapIV Hoda

from typing import List
import pandas as pd
import numpy as np
import math
import yaml

import util.calc as util_calc

def set_ref_data(input_path,config): # Creation of dataset with reference to column number
    use_separate_time_stamp = config["ref_index"]["separate_time_stamp"]
    index_time_unit = config["ref_index"]["time_unit"]
    use_quaternion_flag = config["ref_index"]["use_quaternion"]
    use_radian_flag = config["ref_index"]["use_radian"]
    use_vel_flag = config["ref_index"]["use_vel"]
    use_angular_flag = config["ref_index"]["use_angular"]
    use_qual_flag = config["ref_index"]["use_qual"]
    index_time_sec = config["ref_index"]["time_sec"]
    index_time_nsec = config["ref_index"]["time_nsec"]
    index_time = config["ref_index"]["time"]
    index_x = config["ref_index"]["x"]
    index_y = config["ref_index"]["y"]
    index_z = config["ref_index"]["z"]
    index_ori_x = config["ref_index"]["ori_x"]
    index_ori_y = config["ref_index"]["ori_y"]
    index_ori_z = config["ref_index"]["ori_z"]
    index_ori_w = config["ref_index"]["ori_w"]
    index_roll = config["ref_index"]["roll"]
    index_pitch = config["ref_index"]["pitch"]
    index_yaw = config["ref_index"]["yaw"]
    index_vel_x = config["ref_index"]["vel_x"]
    index_vel_y = config["ref_index"]["vel_y"]
    index_vel_z = config["ref_index"]["vel_z"]
    index_angular_x = config["ref_index"]["angular_x"]
    index_angular_y = config["ref_index"]["angular_y"]
    index_angular_z = config["ref_index"]["angular_z"]
    tmp_df = pd.read_csv(input_path,header=0, index_col=None)
    if use_separate_time_stamp == True:
        ref_data_time = pd.Series(tmp_df.iloc[:,index_time_sec] + tmp_df.iloc[:,index_time_nsec] * 10 ** (-9), name='TimeStamp')
    elif index_time_unit == 1:
        ref_data_time = pd.Series(tmp_df.iloc[:,index_time] * 10 ** (-9), name='TimeStamp')
    else:
        ref_data_time = pd.Series(tmp_df.iloc[:,index_time], name='TimeStamp')
    ref_data_x = pd.Series(tmp_df.iloc[:,index_x], name='x')
    ref_data_y = pd.Series(tmp_df.iloc[:,index_y], name='y')
    ref_data_z = pd.Series(tmp_df.iloc[:,index_z], name='z')
    ref_data_xyz = pd.concat([ref_data_x,ref_data_y,ref_data_z],axis=1)
    if use_quaternion_flag == True:
        ref_data_ori_x = pd.Series(tmp_df.iloc[:,index_ori_x], name='ori_x')
        ref_data_ori_y = pd.Series(tmp_df.iloc[:,index_ori_y], name='ori_y')
        ref_data_ori_z = pd.Series(tmp_df.iloc[:,index_ori_z], name='ori_z')
        ref_data_ori_w = pd.Series(tmp_df.iloc[:,index_ori_w], name='ori_w')
        ref_data_ori = pd.concat([ref_data_ori_x,ref_data_ori_y,ref_data_ori_z,ref_data_ori_w],axis=1)
        ref_rpy = util_calc.quaternion_to_euler_zyx(ref_data_ori)
    elif use_quaternion_flag == False and use_radian_flag == True:
        ref_data_roll = pd.Series(tmp_df.iloc[:,index_roll], name='roll')
        ref_data_pitch = pd.Series(tmp_df.iloc[:,index_pitch], name='pitch')
        ref_data_yaw_tmp = pd.Series(tmp_df.iloc[:,index_yaw], name='yaw')
        ref_data_yaw = util_calc.change_anglel_limit(ref_data_yaw_tmp)
        ref_rpy = pd.concat([np.rad2deg(ref_data_roll),np.rad2deg(ref_data_pitch),np.rad2deg(ref_data_yaw)],axis=1)
    elif use_quaternion_flag == False and use_radian_flag == False:
        ref_data_roll = pd.Series(tmp_df.iloc[:,index_roll], name='roll')
        ref_data_pitch = pd.Series(tmp_df.iloc[:,index_pitch], name='pitch')
        ref_data_yaw_tmp = pd.Series(tmp_df.iloc[:,index_yaw], name='yaw')
        ref_data_yaw = util_calc.change_anglel_limit(np.deg2rad(ref_data_yaw_tmp))
        ref_rpy = pd.concat([ref_data_roll,ref_data_pitch,np.rad2deg(ref_data_yaw)],axis=1)

    set_ref_vel = pd.DataFrame()
    if use_vel_flag == True:
        ref_data_vel_x = pd.Series(tmp_df.iloc[:,index_vel_x], name='vel_x')
        ref_data_vel_y = pd.Series(tmp_df.iloc[:,index_vel_y], name='vel_y')
        ref_data_vel_z = pd.Series(tmp_df.iloc[:,index_vel_z], name='vel_z')
        set_ref_vel_tmp = pd.concat([ref_data_vel_x, ref_data_vel_y, ref_data_vel_z],axis=1)
        ref_velocity = util_calc.calc_velocity(set_ref_vel_tmp)
        distance = util_calc.calc_distance_vel(ref_velocity["velocity"],ref_data_time)
        set_ref_vel = pd.concat([set_ref_vel_tmp, ref_velocity],axis=1)
    else:
        distance = util_calc.calc_distance_xy(ref_data_xyz)
    set_ref_angular = pd.DataFrame()
    if use_angular_flag == True:
        ref_data_angular_x = pd.Series(tmp_df.iloc[:,index_angular_x], name='angular_x')
        ref_data_angular_y = pd.Series(tmp_df.iloc[:,index_angular_y], name='angular_y')
        ref_data_angular_z = pd.Series(tmp_df.iloc[:,index_angular_z], name='angular_z')
        set_ref_angular = pd.concat([ref_data_angular_x, ref_data_angular_y, ref_data_angular_z],axis=1)
    if use_qual_flag == True:
        index_qual = config["ref_index"]["qual"]
        ref_data_qual = pd.Series(tmp_df.iloc[:,index_qual], name='qual')
    else:
        index_qual = config["ref_index"]["qual"]
        ref_data_qual = pd.Series(tmp_df.iloc[:,index_qual], name='qual_not_set')
    set_ref_df = pd.concat([ref_data_time, ref_data_xyz, ref_rpy, set_ref_vel, set_ref_angular, distance, ref_data_qual],axis=1)
    return set_ref_df

def set_target_data(input_path,config): # Creation of dataset with reference to column number
    use_separate_time_stamp = config["target_data_index"]["separate_time_stamp"]
    index_time_unit = config["target_data_index"]["time_unit"]
    use_qual_flag = config["target_data_index"]["use_qual"]
    index_time_sec = config["target_data_index"]["time_sec"]
    index_time_nsec = config["target_data_index"]["time_nsec"]
    index_time = config["target_data_index"]["time"]
    index_x = config["target_data_index"]["x"]
    index_y = config["target_data_index"]["y"]
    index_z = config["target_data_index"]["z"]
    index_ori_x = config["target_data_index"]["ori_x"]
    index_ori_y = config["target_data_index"]["ori_y"]
    index_ori_z = config["target_data_index"]["ori_z"]
    index_ori_w = config["target_data_index"]["ori_w"]
    tmp_df = pd.read_csv(input_path,header=0, index_col=None)
    if use_separate_time_stamp == True:
        data_time = pd.Series(tmp_df.iloc[:,index_time_sec] + tmp_df.iloc[:,index_time_nsec] * 10 ** (-9), name='TimeStamp')
    elif index_time_unit == 1:
        data_time = pd.Series(tmp_df.iloc[:,index_time] * 10 ** (-9), name='TimeStamp')
    else:
        data_time = pd.Series(tmp_df.iloc[:,index_time], name='TimeStamp')
    data_x = pd.Series(tmp_df.iloc[:,index_x], name='x')
    data_y = pd.Series(tmp_df.iloc[:,index_y], name='y')
    data_z = pd.Series(tmp_df.iloc[:,index_z], name='z')
    data_ori_x = pd.Series(tmp_df.iloc[:,index_ori_x], name='ori_x')
    data_ori_y = pd.Series(tmp_df.iloc[:,index_ori_y], name='ori_y')
    data_ori_z = pd.Series(tmp_df.iloc[:,index_ori_z], name='ori_z')
    data_ori_w = pd.Series(tmp_df.iloc[:,index_ori_w], name='ori_w')
    if use_qual_flag == True:
        index_qual = config["target_data_index"]["qual"]
        data_qual = pd.Series(tmp_df.iloc[:,index_qual], name='qual')
    else:
        index_qual = config["target_data_index"]["qual"]
        data_qual = pd.Series(tmp_df.iloc[:,index_qual], name='qual_not_set')
    set_df = pd.concat([data_time, data_x, data_y, data_z, data_ori_x, data_ori_y, data_ori_z, data_ori_w, data_qual],axis=1)
    return set_df

def set_twist_data(input_path,config): # Creation of dataset with reference to column number
    use_separate_time_stamp = config["twist_index"]["separate_time_stamp"]
    index_time_unit = config["twist_index"]["time_unit"]
    index_time_sec = config["twist_index"]["time_sec"]
    index_time_nsec = config["twist_index"]["time_nsec"]
    index_time = config["twist_index"]["time"]
    index_linear_x = config["twist_index"]["linear_x"]
    index_linear_y = config["twist_index"]["linear_y"]
    index_linear_z = config["twist_index"]["linear_z"]
    index_angular_x = config["twist_index"]["angular_x"]
    index_angular_y = config["twist_index"]["angular_y"]
    index_angular_z = config["twist_index"]["angular_z"]
    tmp_df = pd.read_csv(input_path,header=0, index_col=None)
    if use_separate_time_stamp == True:
        data_time = pd.Series(tmp_df.iloc[:,index_time_sec] + tmp_df.iloc[:,index_time_nsec] * 10 ** (-9), name='TimeStamp')
    elif index_time_unit == 1:
        data_time = pd.Series(tmp_df.iloc[:,index_time] * 10 ** (-9), name='TimeStamp')
    else:
        data_time = pd.Series(tmp_df.iloc[:,index_time], name='TimeStamp')
    data_linear_x = pd.Series(tmp_df.iloc[:,index_linear_x], name='velocity')
    data_linear_y = pd.Series(tmp_df.iloc[:,index_linear_y], name='linear_y')
    data_linear_z = pd.Series(tmp_df.iloc[:,index_linear_z], name='linear_z')
    data_angular_x = pd.Series(tmp_df.iloc[:,index_angular_x], name='angular_x')
    data_angular_y = pd.Series(tmp_df.iloc[:,index_angular_y], name='angular_y')
    data_angular_z = pd.Series(tmp_df.iloc[:,index_angular_z], name='angular_z')
    set_df = pd.concat([data_time, data_linear_x, data_linear_y, data_linear_z, data_angular_x, data_angular_y, data_angular_z],axis=1)
    return set_df

def set_log_df(input,plane,config): # Creation of dataset with reference to labels in df
    df = pd.read_csv(input,delimiter=None, header='infer',  index_col=None, usecols=None)
    index_time_unit = config["eagleye_log"]["time_unit"]
    tf_num = config["eagleye_log"]["tf_num"]
    ros_reverse_imu = config["eagleye_log"]["ros_reverse_imu"]
    missing_gnss_type = config["eagleye_log"]["missing_gnss_type"]
    eagleye_df = df[["timestamp",
             "rtklib_nav.tow",
             "velocity_scale_factor.scale_factor",
             "correction_velocity.twist.linear.x",
             "distance.distance",
             "enu_absolute_pos_interpolate.ecef_base_pos.x",
             "enu_absolute_pos_interpolate.ecef_base_pos.y",
             "enu_absolute_pos_interpolate.ecef_base_pos.z",
             "rolling.rolling_angle",
             "pitching.pitching_angle",
             "heading_interpolate_1st.heading_angle",
             "heading_interpolate_2nd.heading_angle",
             "heading_interpolate_3rd.heading_angle",
             "yaw_rate_offset_stop.yaw_rate_offset",
             "yaw_rate_offset_2nd.yaw_rate_offset",
             "slip_angle.slip_angle",
             "enu_vel.vector.x",
             "enu_vel.vector.y",
             "enu_vel.vector.z",
             "eagleye_pp_llh.latitude",
             "eagleye_pp_llh.longitude",
             "eagleye_pp_llh.altitude",
             "imu.angular_velocity.x",
             "imu.angular_velocity.y",
             "imu.angular_velocity.z",
             'gga_llh.gps_qual',
             ]]

    eagleye_df = eagleye_df.rename(columns={'timestamp': 'TimeStamp_tmp',
                            'rtklib_nav.tow': 'TOW',
                            'velocity_scale_factor.scale_factor': 'sf',
                            'correction_velocity.twist.linear.x': 'velocity',
                            'distance.distance': 'distance',
                            'enu_absolute_pos_interpolate.ecef_base_pos.x': 'ecef_base_x',
                            'enu_absolute_pos_interpolate.ecef_base_pos.y': 'ecef_base_y',
                            'enu_absolute_pos_interpolate.ecef_base_pos.z': 'ecef_base_z',
                            'rolling.rolling_angle': 'roll_rad',
                            'pitching.pitching_angle': 'pitch_rad',
                            'heading_interpolate_1st.heading_angle': 'heading_1st',
                            'heading_interpolate_2nd.heading_angle': 'heading_2nd',
                            'heading_interpolate_3rd.heading_angle': 'yaw_rad',
                            'yaw_rate_offset_stop.yaw_rate_offset': 'yaw_rate_offset_stop',
                            'yaw_rate_offset_2nd.yaw_rate_offset': 'yaw_rate_offset',
                            'slip_angle.slip_angle': 'slip',
                            'enu_vel.vector.x': 'vel_x',
                            'enu_vel.vector.y': 'vel_y',
                            'enu_vel.vector.z': 'vel_z',
                            'eagleye_pp_llh.latitude': 'latitude',
                            'eagleye_pp_llh.longitude': 'longitude',
                            'eagleye_pp_llh.altitude': 'altitude',
                            "imu.angular_velocity.x":'angular_x',
                            "imu.angular_velocity.y":'angular_y',
                            "imu.angular_velocity.z":'angular_z',
                            'gga_llh.gps_qual': 'qual',
                            })

    raw_df = df[["timestamp",
                "rtklib_nav.tow",
                "velocity.twist.linear.x",
                "distance.distance",
                "imu.angular_velocity.x",
                "imu.angular_velocity.y",
                "imu.angular_velocity.z",
                "rtklib_nav.ecef_vel.x",
                "rtklib_nav.ecef_vel.y",
                "rtklib_nav.ecef_vel.z",
                "rtklib_nav.status.latitude",
                "rtklib_nav.status.longitude",
                "rtklib_nav.status.altitude",
                "gga_llh.latitude",
                "gga_llh.longitude",
                "gga_llh.altitude",
                "gga_llh.gps_qual",
                ]]

    raw_df = raw_df.rename(columns={'timestamp': 'TimeStamp_tmp',
                                    'rtklib_nav.tow': 'TOW',
                                    'velocity.twist.linear.x': 'velocity',
                                    'distance.distance': 'distance',
                                    'imu.angular_velocity.x': 'rollrate',
                                    'imu.angular_velocity.y': 'pitch_rate',
                                    'imu.angular_velocity.z': 'yaw_rate',
                                    'rtklib_nav.ecef_vel.x': 'vel_x',
                                    'rtklib_nav.ecef_vel.y': 'vel_y',
                                    'rtklib_nav.ecef_vel.z': 'vel_z',
                                    'rtklib_nav.status.latitude': 'latitude',
                                    'rtklib_nav.status.longitude': 'longitude',
                                    'rtklib_nav.status.altitude': 'altitude',
                                    'gga_llh.latitude': 'rtk_latitude',
                                    'gga_llh.longitude': 'rtk_longitude',
                                    'gga_llh.altitude': 'rtk_altitude',
                                    'gga_llh.gps_qual': 'qual',
                                    })

    raw_index = raw_df[raw_df['latitude'] == 0].index
    rtk_index = raw_df[raw_df['rtk_latitude'] == 0].index
    if missing_gnss_type == 1:
        raw_df = raw_df.drop(rtk_index)
        eagleye_df = eagleye_df.drop(rtk_index)
    elif missing_gnss_type == 2:
        raw_df = raw_df.drop(raw_index)
        eagleye_df = eagleye_df.drop(raw_index)
    elif len(rtk_index) < len(raw_index):
        raw_df = raw_df.drop(raw_index)
        eagleye_df = eagleye_df.drop(raw_index)
    else:
        raw_df = raw_df.drop(rtk_index)
        eagleye_df = eagleye_df.drop(rtk_index)
    raw_df = raw_df.reset_index()
    eagleye_df = eagleye_df.reset_index()

    if index_time_unit == 1:
        eagleye_df['TimeStamp'] = eagleye_df['TimeStamp_tmp'] * 10 ** (-9)
        raw_df['TimeStamp'] = raw_df['TimeStamp_tmp'] * 10 ** (-9)
    elif index_time_unit == 0:
        eagleye_df['TimeStamp'] = eagleye_df['TimeStamp_tmp']
        raw_df['TimeStamp'] = raw_df['TimeStamp_tmp']
    eagleye_df['elapsed_time'] = eagleye_df['TimeStamp'] - raw_df['TimeStamp'][0]
    raw_df['elapsed_time'] = raw_df['TimeStamp'] - raw_df['TimeStamp'][0]
    eagleye_df['yaw'] = np.rad2deg(util_calc.change_anglel_limit(eagleye_df['yaw_rad']))
    eagleye_df['roll'] = np.rad2deg(eagleye_df['roll_rad'])
    eagleye_df['pitch'] = np.rad2deg(eagleye_df['pitch_rad'])
    llh = pd.concat([eagleye_df['latitude'],eagleye_df['longitude'],eagleye_df['altitude']],axis=1)
    if tf_num == 0:
        xyz = latlon_to_19(llh,plane)
    elif tf_num == 1:
        xyz = util_calc.ll2mgrs(llh)
    if ros_reverse_imu == True:
        eagleye_df['angular_z'] = -1 * eagleye_df['angular_z']
        eagleye_df['yaw_rate_offset_stop'] = -1 * eagleye_df['yaw_rate_offset_stop']
        eagleye_df['yaw_rate_offset'] = -1 * eagleye_df['yaw_rate_offset']
        eagleye_df['slip'] = -1 * eagleye_df['slip']
    set_eagleye_df = pd.concat([eagleye_df, xyz],axis=1)
    return set_eagleye_df, raw_df

def set_tf_xy(xy,tf_x,tf_y):
    xy['x'] = xy['x'] + tf_x
    xy['y'] = xy['y'] + tf_y
    return xy

def correct_anntenapos(eagleye_df,ref_yaw,tf_across,tf_along,tf_height):
    d = [[tf_across],[tf_along],[0]]
    for i in range(len(ref_yaw)):
        sinphi = math.sin(math.radians(ref_yaw[i]))
        cosphi = math.cos(math.radians(ref_yaw[i]))
        R = [[cosphi , sinphi , 0],[-sinphi , cosphi , 0],[0,0,0]]
        diff = np.dot(R, d)
        eagleye_df['x'][i] = eagleye_df['x'][i] + diff[0]
        eagleye_df['y'][i] = eagleye_df['y'][i] + diff[1]
    eagleye_df['z'] = eagleye_df['z'] + tf_height
    return eagleye_df

def latlon_to_19(llh,plane):
    phi0_deg , lambda0_deg = plane_table(plane)
    set_xy: List[float] = []
    for i in range(len(llh)):
        phi_deg=llh['latitude'][i]
        lambda_deg=llh['longitude'][i]
        z=llh['altitude'][i]
        """ Converts latitude and longitude to xy in plane rectangular coordinates
        - input:
            (phi_deg, lambda_deg): Latitude and longitude [degrees] to be converted
                                   (note that these are decimal numbers, not minutes and seconds)
            (phi0_deg, lambda0_deg): Latitude and longitude [degrees] of the origin of the rectangular coordinate system
                                   (note that these are decimal degrees, not minutes and seconds)
        - output:
        x: Transformed plane rectangular coordinates[m]
        y: Transformed plane rectangular coordinates[m]
        """
        # Correcting the latitude-longitude and plane-rectangular coordinate system origin from degree to radian
        phi_rad = np.deg2rad(phi_deg)
        lambda_rad = np.deg2rad(lambda_deg)
        phi0_rad = np.deg2rad(phi0_deg)
        lambda0_rad = np.deg2rad(lambda0_deg)

        # Auxiliary function
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

        # Constant (a, F: World Geodetic System - Geodetic Reference System 1980 (GRS80) Ellipsoid)
        m0 = 0.9999 
        a = 6378137.
        F = 298.257222101

        # (1), Calculation of n, A_i, alpha_i
        n = 1. / (2*F - 1)
        A_array = A_array(n)
        alpha_array = alpha_array(n)

        # (2), Calculation of S, A
        A_ = ( (m0*a)/(1.+n) )*A_array[0] # [m]
        S_ = ( (m0*a)/(1.+n) )*( A_array[0]*phi0_rad + np.dot(A_array[1:], np.sin(2*phi0_rad*np.arange(1,6))) ) # [m]

        # (3) Calculation of lambda_c, lambda_s
        lambda_c = np.cos(lambda_rad - lambda0_rad)
        lambda_s = np.sin(lambda_rad - lambda0_rad)

        # (4)Calculation of  t, t_
        t = np.sinh( np.arctanh(np.sin(phi_rad)) - ((2*np.sqrt(n)) / (1+n))*np.arctanh(((2*np.sqrt(n)) / (1+n)) * np.sin(phi_rad)) )
        t_ = np.sqrt(1 + t*t)

        # (5) Calculation of xi', eta'
        xi2  = np.arctan(t / lambda_c) # [rad]
        eta2 = np.arctanh(lambda_s / t_)

        # (6) Calculation of x, y
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