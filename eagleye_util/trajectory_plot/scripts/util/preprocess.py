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

def set_ref_data(ref_data_tmp): # Creation of dataset with reference to column number
    set_data: List[float] = []
    for i, data in enumerate(ref_data_tmp):
        if i == 0: continue
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
             "enu_absolute_pos_interpolate.enu_pos.x",
             "enu_absolute_pos_interpolate.enu_pos.y",
             "enu_absolute_pos_interpolate.enu_pos.z",
             "enu_absolute_pos.ecef_base_pos.x",
             "enu_absolute_pos.ecef_base_pos.y",
             "enu_absolute_pos.ecef_base_pos.z",
             "rolling.rolling_angle",
             "pitching.pitching_angle",
             "heading_interpolate_1st.heading_angle",
             "heading_interpolate_2nd.heading_angle",
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
                            'rtklib_nav.tow': 'TOW',
                            'velocity_scale_factor.scale_factor': 'sf',
                            'velocity_scale_factor.correction_velocity.linear.x': 'velocity',
                            'distance.distance': 'distance',
                            'enu_absolute_pos.ecef_base_pos.x': 'ecef_base_x',
                            'enu_absolute_pos.ecef_base_pos.y': 'ecef_base_y',
                            'enu_absolute_pos.ecef_base_pos.z': 'ecef_base_z',
                            'rolling.rolling_angle': 'roll_rad',
                            'pitching.pitching_angle': 'pitch_rad',
                            'heading_interpolate_1st.heading_angle': 'heading_1st',
                            'heading_interpolate_2nd.heading_angle': 'heading_2nd',
                            'heading_interpolate_3rd.heading_angle': 'yaw_rad',
                            'enu_vel.vector.x': 'enu_vel_x',
                            'enu_vel.vector.y': 'enu_vel_y',
                            'enu_vel.vector.z': 'enu_vel_z',
                            'eagleye_pp_llh.latitude': 'latitude',
                            'eagleye_pp_llh.longitude': 'longitude',
                            'eagleye_pp_llh.altitude': 'altitude',
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
                                    'imu.angular_velocity.y': 'pitchrate',
                                    'imu.angular_velocity.z': 'yawrate',
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

    eagleye_df['TimeStamp'] = eagleye_df['TimeStamp_tmp'] * 10 ** (-9)
    eagleye_df['elapsed_time'] = eagleye_df['TimeStamp'] - eagleye_df['TimeStamp'][0]
    rpy_df = get_rpy_deg(eagleye_df)
    raw_df['TimeStamp'] = raw_df['TimeStamp_tmp'] * 10 ** (-9)
    raw_df['elapsed_time'] = raw_df['TimeStamp'] - raw_df['TimeStamp'][0]
    xyz = latlon_to_19(eagleye_df,plane)
    set_eagleye_df = pd.concat([eagleye_df,xyz, rpy_df],axis=1)
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

def get_rpy_deg(rpy_rad):
    set_heading_data: List[float] = []
    for i in range(len(rpy_rad)):
        roll_tmp = rpy_rad['roll_rad'][i]
        pitch_tmp = rpy_rad['pitch_rad'][i]
        yaw_tmp = change_anglel_limit(rpy_rad['yaw_rad'][i])
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