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

# calc.py
# Author MapIV Hoda


from typing import List
import pandas as pd
import numpy as np
import math

import matplotlib.pyplot as plt
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

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

def xyz2enu(ecef_xyz,org_xyz):
    org_llh = xyz2llh(org_xyz)
    phi = org_llh[0]
    lam = org_llh[1]
    sinphi = np.sin(phi)
    cosphi = np.cos(phi)
    sinlam = np.sin(lam)
    coslam = np.cos(lam)
    R = np.matrix([[-sinlam , coslam , 0],[-sinphi*coslam , -sinphi*sinlam , cosphi],[cosphi*coslam  , cosphi*sinlam , sinphi]])
    
    set_enu_data: List[float] = []
    for i in range(len(ecef_xyz)):
        if ecef_xyz['ecef_x'][i] == 0 and ecef_xyz['ecef_y'][i] == 0:
            set_enu_data.append([0,0,0])
            continue
        diff_xyz = np.array([[ecef_xyz['ecef_x'][i] - org_xyz[0]] , [ecef_xyz['ecef_y'][i] - org_xyz[1]] , [ecef_xyz['ecef_z'][i] - org_xyz[2]]])
        enu = R * diff_xyz
        x: float = float(enu[0])
        y: float = float(enu[1])
        z: float = float(enu[2])
        set_enu_data.append([x,y,z])
    set_enu = pd.DataFrame(set_enu_data,columns=['x','y','z'])
    return set_enu

def xyz2llh(xyz):
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]
    x2 = x**2
    y2 = y**2
    z2 = z**2

    a = 6378137.0000    # earth radius in meters
    b = 6356752.3142    # earth semiminor in meters    
    e = (1-(b/a)**2)**0.5
    b2 = b*b
    e2 = e**2
    ep = e*(a/b)
    r = (x2+y2)**0.5
    r2 = r*r
    E2 = a**2 - b**2
    F = 54*b2*z2
    G = r2 + (1-e2)*z2 - e2*E2
    c = (e2*e2*F*r2)/(G*G*G)
    s = ( 1 + c + (c*c + 2*c)**0.5 )**(1/3)
    P = F / (3 * (s+1/s+1)**2 * G*G)
    Q = (1+2*e2*e2*P)**0.5
    ro = -(P*e2*r)/(1+Q) + ((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) ** 0.5
    tmp = (r - e2*ro)**2
    U = ( tmp + z2 )**0.5
    V = ( tmp + (1-e2)*z2 )**0.5
    zo = (b2*z)/(a*V)

    height = U*( 1 - b2/(a*V) )
    
    lat = math.atan( (z + ep*ep*zo)/r )
    temp = math.atan(y/x)
    if x >=0:
        long = temp
    elif (x < 0) & (y >= 0):
        long = math.pi + temp
    else:
        long = temp - math.pi

    llh = [lat,long,height]
    return llh

def llh2xyz(llh):
    set_xyz_data: List[float] = []
    for i in range(len(llh)):
        lat = llh['latitude'][i]
        lon = llh['longitude'][i]
        ht = llh['altitude'][i]

        PI_180 = math.pi / 180.0
        A      = 6378137.0
        ONE_F  = 298.257223563
        E2     = (1.0 / ONE_F) * (2 - (1.0 / ONE_F))

        n = lambda x: A / \
            math.sqrt(1.0 - E2 * math.sin(x * PI_180)**2)
        x = (n(lat) + ht) \
            * math.cos(lat * PI_180) \
            * math.cos(lon * PI_180)
        y = (n(lat) + ht) \
            * math.cos(lat * PI_180) \
            * math.sin(lon * PI_180)
        z = (n(lat) * (1.0 - E2) + ht) \
            * math.sin(lat * PI_180)
        set_xyz_data.append([x,y,z])
    set_xyz = pd.DataFrame(set_xyz_data,columns=['ecef_x','ecef_y','ecef_z'])
    return set_xyz

def change_anglel_limit_pi(heading):
    while heading < -math.pi or math.pi < heading:
        if heading < -math.pi:
            heading += math.pi * 2
        else:
            heading -= math.pi * 2
    return heading

def get_heading_deg(eagleye_df):
    set_heading_data: List[float] = []
    for i in range(len(eagleye_df)):
        heading_1st_deg_tmp = change_anglel_limit_pi(eagleye_df['heading_1st'][i])
        heading_2nd_deg_tmp = change_anglel_limit_pi(eagleye_df['heading_2nd'][i])
        heading_3rd_deg_tmp = change_anglel_limit_pi(eagleye_df['yaw_rad'][i])
        heading_1st_deg = math.degrees(heading_1st_deg_tmp)
        heading_2nd_deg = math.degrees(heading_2nd_deg_tmp)
        heading_3rd_deg = math.degrees(heading_3rd_deg_tmp)
        rolling = math.degrees(eagleye_df['roll_rad'][i])
        pitching = math.degrees(eagleye_df['pitch_rad'][i])
        set_heading_data.append([heading_1st_deg,heading_2nd_deg,heading_3rd_deg,rolling,pitching])
    df = pd.DataFrame(set_heading_data,columns=['heading_1st_deg','heading_2nd_deg','heading_3rd_deg','roll','pitch'])
    return df

def xyz2enu_vel(vel,org_xyz):
    set_vel_data: List[float] = []
    for i in range(len(vel)):
        x_vel = vel['vel_x'][i]
        y_vel = vel['vel_y'][i]
        z_vel = vel['vel_z'][i]

        org_llh = xyz2llh(org_xyz)
        phi = org_llh[0]
        lam = org_llh[1]

        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        sin_lam = math.sin(lam)
        cos_lam = math.cos(lam)

        e_vel = (-x_vel * sin_lam) + (y_vel * cos_lam)
        n_vel = (-x_vel * cos_lam * sin_phi) - (y_vel * sin_lam * sin_phi) + (z_vel * cos_phi)
        u_vel = (x_vel * cos_lam * cos_phi) +( y_vel * sin_lam * cos_phi) + (z_vel * sin_phi)
        vel_2d = (e_vel ** 2 + n_vel ** 2) ** 0.5

        set_vel_data.append([e_vel,n_vel,u_vel,vel_2d])
    df = pd.DataFrame(set_vel_data,columns=['east_vel','north_vel','up_vel','velocity'])
    return df

def latlon_to_19(llh,plane):
    phi0_deg , lambda0_deg = plane_table(plane)
    set_xy: List[float] = []
    for i in range(len(llh)):
        phi_deg=llh['latitude'][i]
        lambda_deg=llh['longitude'][i]
        z=llh['altitude'][i]
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


def sync_time(ref_data,csv_data,sync_threshold_time,leap_time): # Time synchronization
    sync_index = np.zeros(len(csv_data['TimeStamp']))

    first_flag_data = 0
    set_data_df: List[float] = []
    set_data_ori: List[float] = []
    set_data_rpy: List[float] = []
    set_data_velocity: List[float] = []
    set_data_vel: List[float] = []
    set_data_distance: List[float] = []
    set_data_qual: List[float] = []
    first_flag_ref = 0
    set_ref_data_df: List[float] = []
    set_ref_data_ori: List[float] = []
    set_ref_data_rpy: List[float] = []
    set_ref_data_velocity: List[float] = []
    set_ref_data_vel: List[float] = []
    set_ref_data_distance: List[float] = []
    set_ref_data_qual: List[float] = []
    for i in range(len(csv_data)):
        if i == 0: continue
        time_tmp: List[float] = []
        time_tmp = abs(csv_data.iloc[i]['TimeStamp']-ref_data['TimeStamp'] + leap_time)
        sync_index[i] = np.argmin(time_tmp)
        sync_time_tmp = time_tmp[sync_index[i]]
        if sync_time_tmp < sync_threshold_time:
            num = int(sync_index[i])
            data_time_tmp = csv_data.iloc[i]['TimeStamp']
            data_x_tmp = csv_data.iloc[i]['x']
            data_y_tmp = csv_data.iloc[i]['y']
            data_z_tmp = csv_data.iloc[i]['z']
            if (first_flag_data == 0):
                first_time_data = csv_data.iloc[i]['TimeStamp']
                first_flag_data = 1
            data_elapsed_time_tmp = csv_data.iloc[i]['TimeStamp'] - first_time_data
            set_data_df.append([data_elapsed_time_tmp,data_time_tmp,data_x_tmp,data_y_tmp,data_z_tmp])
            if 'ori_x' in csv_data.columns:
                data_ori_x_tmp = csv_data.iloc[i]['ori_x']
                data_ori_y_tmp = csv_data.iloc[i]['ori_y']
                data_ori_z_tmp = csv_data.iloc[i]['ori_z']
                data_ori_w_tmp = csv_data.iloc[i]['ori_w']
                set_data_ori.append([data_ori_x_tmp,data_ori_y_tmp,data_ori_z_tmp,data_ori_w_tmp])
            if 'roll' in csv_data.columns:
                data_roll_tmp = csv_data.iloc[i]['roll']
                data_pitch_tmp = csv_data.iloc[i]['pitch']
                data_yaw_tmp = csv_data.iloc[i]['yaw']
                set_data_rpy.append([data_roll_tmp,data_pitch_tmp,data_yaw_tmp])
            if 'velocity' in csv_data.columns:
                velocity_tmp = csv_data.iloc[i]['velocity']
                set_data_velocity.append([velocity_tmp])
            if 'vel_x' in csv_data.columns:
                data_vel_x_tmp = csv_data.iloc[i]['vel_x']
                data_vel_y_tmp = csv_data.iloc[i]['vel_y']
                data_vel_z_tmp = csv_data.iloc[i]['vel_z']
                set_data_vel.append([data_vel_x_tmp,data_vel_y_tmp,data_vel_z_tmp])
            if 'distance' in csv_data.columns:
                data_distance_tmp = csv_data.iloc[i]['distance']
                set_data_distance.append([data_distance_tmp])
            if 'qual' in csv_data.columns:
                data_qual_tmp = csv_data.iloc[i]['qual']
                set_data_qual.append([data_qual_tmp])

            ref_time_tmp = ref_data.iloc[num]['TimeStamp']
            ref_x_tmp = ref_data.iloc[num]['x']
            ref_y_tmp = ref_data.iloc[num]['y']
            ref_z_tmp = ref_data.iloc[num]['z']
            if (first_flag_ref == 0):
                first_time_ref = ref_data.iloc[num]['TimeStamp']
                first_flag_ref = 1
            ref_elapsed_time_tmp = ref_data.iloc[num]['TimeStamp'] - first_time_ref
            set_ref_data_df.append([ref_elapsed_time_tmp,ref_time_tmp,ref_x_tmp,ref_y_tmp,ref_z_tmp])
            if 'ori_x' in ref_data.columns:
                ref_ori_x_tmp = ref_data.iloc[num]['ori_x']
                ref_ori_y_tmp = ref_data.iloc[num]['ori_y']
                ref_ori_z_tmp = ref_data.iloc[num]['ori_z']
                ref_ori_w_tmp = ref_data.iloc[num]['ori_w']
                set_ref_data_ori.append([ref_ori_x_tmp,ref_ori_y_tmp,ref_ori_z_tmp,ref_ori_w_tmp])
            if 'roll' in ref_data.columns:
                ref_roll_tmp = ref_data.iloc[num]['roll']
                ref_pitch_tmp = ref_data.iloc[num]['pitch']
                ref_yaw_tmp = ref_data.iloc[num]['yaw']
                set_ref_data_rpy.append([ref_roll_tmp,ref_pitch_tmp,ref_yaw_tmp])
            if 'velocity' in ref_data.columns:
                velocity_tmp = ref_data.iloc[num]['velocity']
                set_ref_data_velocity.append([velocity_tmp])
            if 'vel_x' in ref_data.columns:
                ref_vel_x_tmp = ref_data.iloc[num]['vel_x']
                ref_vel_y_tmp = ref_data.iloc[num]['vel_y']
                ref_vel_z_tmp = ref_data.iloc[num]['vel_z']
                set_ref_data_vel.append([ref_vel_x_tmp,ref_vel_y_tmp,ref_vel_z_tmp])
            if 'distance' in ref_data.columns:
                ref_distance_tmp = ref_data.iloc[num]['distance']
                set_ref_data_distance.append([ref_distance_tmp])
            if 'qual' in ref_data.columns:
                ref_qual_tmp = ref_data.iloc[num]['qual']
                set_ref_data_qual.append([ref_qual_tmp])


    data_ori = pd.DataFrame()
    data_rpy = pd.DataFrame()
    data_velocity = pd.DataFrame()
    data_vel = pd.DataFrame()
    data_distance = pd.DataFrame()
    data_qual = pd.DataFrame()
    data_df = pd.DataFrame(set_data_df,columns=['elapsed_time','TimeStamp', 'x', 'y', 'z'])
    if 'ori_x' in csv_data.columns:
        data_ori = pd.DataFrame(set_data_ori,columns=['ori_x', 'ori_y', 'ori_z', 'ori_w'])
    if 'roll' in csv_data.columns:
        data_rpy = pd.DataFrame(set_data_rpy,columns=['roll', 'pitch', 'yaw'])
    if 'velocity' in csv_data.columns:
        data_velocity = pd.DataFrame(set_data_velocity,columns=['velocity'])
    if 'vel_x' in csv_data.columns:
        data_vel = pd.DataFrame(set_data_vel,columns=['vel_x', 'vel_y', 'vel_z'])
    if 'distance' in csv_data.columns:
        data_distance = pd.DataFrame(set_data_distance,columns=['distance'])
    if 'qual' in csv_data.columns:
        data_qual = pd.DataFrame(set_data_qual,columns=['qual'])
    data_df_output = pd.concat([data_df,data_ori,data_rpy,data_velocity,data_vel,data_distance,data_qual],axis=1)

    ref_ori = pd.DataFrame()
    ref_rpy = pd.DataFrame()
    ref_velocity = pd.DataFrame()
    ref_vel = pd.DataFrame()
    ref_distance = pd.DataFrame()
    ref_qual = pd.DataFrame()
    ref_df = pd.DataFrame(set_ref_data_df,columns=['elapsed_time','TimeStamp', 'x', 'y', 'z'])
    if 'ori_x' in ref_data.columns:
        ref_ori = pd.DataFrame(set_ref_data_ori,columns=['ori_x', 'ori_y', 'ori_z', 'ori_w'])
    if 'roll' in ref_data.columns:
        ref_rpy = pd.DataFrame(set_ref_data_rpy,columns=['roll', 'pitch', 'yaw'])
    if 'velocity' in ref_data.columns:
        ref_velocity = pd.DataFrame(set_ref_data_velocity,columns=['velocity'])
    if 'vel_x' in ref_data.columns:
        ref_vel = pd.DataFrame(set_ref_data_vel,columns=['vel_x', 'vel_y', 'vel_z'])
    if 'distance' in ref_data.columns:
        ref_distance = pd.DataFrame(set_ref_data_distance,columns=['distance'])
    if 'qual' in ref_data.columns:
        ref_qual = pd.DataFrame(set_ref_data_qual,columns=['qual'])
    
    ref_df_output = pd.concat([ref_df,ref_ori,ref_rpy,ref_velocity,ref_vel,ref_distance,ref_qual],axis=1)
   
    return ref_df_output , data_df_output


def calc_error_xyz(elapsed_time,ref_time,eagleye_time,ref_xyz,data_xyz,ref_yaw):
    error_xyz = pd.DataFrame()
    error_xyz['elapsed_time'] = elapsed_time
    error_xyz['TimeStamp'] = ref_time - eagleye_time
    error_xyz['x'] = data_xyz['x'] - ref_xyz['x']
    error_xyz['y'] = data_xyz['y'] - ref_xyz['y']
    error_xyz['z'] = data_xyz['z'] - ref_xyz['z']
    error_xyz['2d'] = (error_xyz['x']**2 + error_xyz['y']**2)**0.5
    
    set_aa: List[float] = []
    for i in range(len(ref_yaw)):
        across =  error_xyz.iloc[i]['x'] * math.cos(math.radians(ref_yaw[i])) - error_xyz.iloc[i]['y'] * math.sin(math.radians(ref_yaw[i]))
        along =  error_xyz.iloc[i]['x'] * math.sin(math.radians(ref_yaw[i])) + error_xyz.iloc[i]['y'] * math.cos(math.radians(ref_yaw[i]))
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


def quaternion_to_euler_zyx(ori):
    set_eular_angle: List[float] = []
    for i in range(len(ori)):
        q = [ori.iloc[i]['ori_w'],ori.iloc[i]['ori_x'],ori.iloc[i]['ori_y'],ori.iloc[i]['ori_z']]
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

def calc_velocity(vel):
    velocity = pd.DataFrame()
    velocity['velocity'] = (vel['vel_x'] ** 2 + vel['vel_y'] ** 2 + vel['vel_z'] ** 2) ** 0.5
    return velocity

def calc_velocity_error(eagleye_velocity,ref_velocity):
    error_velocity = pd.DataFrame()
    error_velocity['velocity'] = eagleye_velocity - ref_velocity
    return error_velocity


def clac_dr(TimeStamp,distance,eagleye_xyz,eagleye_vel_xyz,ref_xyz,distance_length,distance_step):
    last_distance = 0
    set_calc_error: List[float] = []
    for i in range(len(TimeStamp)):
        if i == 0: continue
        if last_distance + distance_step < distance[i]:
            start_distance = distance[i]
            start_pos_x = ref_xyz['x'][i]
            start_pos_y = ref_xyz['y'][i]
            previous_pos_x = 0
            previous_pos_y = 0
            last_distance = start_distance
            last_time = TimeStamp[i]
            for j in range(i,len(TimeStamp)):
                if eagleye_xyz['x'][j] == 0 and eagleye_xyz['y'][j] == 0: break
                if distance[j] - start_distance < distance_length:
                    dr_pos_x = previous_pos_x + eagleye_vel_xyz['vel_x'][j] * (TimeStamp[j] - last_time)
                    dr_pos_y = previous_pos_y + eagleye_vel_xyz['vel_y'][j] * (TimeStamp[j] - last_time)
                    previous_pos_x = dr_pos_x
                    previous_pos_y = dr_pos_y
                    last_time = TimeStamp[j]
                    distance_data = distance[j] - start_distance
                    absolute_pos_x = ref_xyz['x'][j] - start_pos_x
                    absolute_pos_y = ref_xyz['y'][j] - start_pos_y
                else:
                    error_x = absolute_pos_x - dr_pos_x
                    error_y = absolute_pos_y - dr_pos_y
                    error_2d_fabs = math.fabs(error_x ** 2 + error_y ** 2)
                    error_2d= math.sqrt(error_2d_fabs)
                    set_calc_error.append([start_distance,distance_data,absolute_pos_x,dr_pos_x,absolute_pos_y,dr_pos_y,error_x,error_y,error_2d])
                    break
    calc_error = pd.DataFrame(set_calc_error,columns=['start_distance','distance','absolute_pos_x','absolute_pos_y','dr_pos_x','dr_pos_y','error_x','error_y','error_2d'])
    return calc_error

