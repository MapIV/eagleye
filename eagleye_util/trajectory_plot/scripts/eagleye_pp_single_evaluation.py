import argparse
from re import X
from typing import List
import pandas as pd
import numpy as np
import sys
import math
import matplotlib.pyplot as plt

def set_df(input): # Creation of dataset with reference to labels in df
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

    eagleye_df = eagleye_df.rename(columns={'timestamp': 'TimeStamp',
                            'rtklib_nav.tow': 'TOW',
                            'velocity_scale_factor.scale_factor': 'sf',
                            'velocity_scale_factor.correction_velocity.linear.x': 'velocity',
                            'distance.distance': 'distance',
                            'enu_absolute_pos.ecef_base_pos.x': 'ecef_base_x',
                            'enu_absolute_pos.ecef_base_pos.y': 'ecef_base_y',
                            'enu_absolute_pos.ecef_base_pos.z': 'ecef_base_z',
                            'rolling.rolling_angle': 'rolling',
                            'pitching.pitching_angle': 'pitching',
                            'heading_interpolate_1st.heading_angle': 'heading_1st',
                            'heading_interpolate_2nd.heading_angle': 'heading_2nd',
                            'heading_interpolate_3rd.heading_angle': 'heading_3rd',
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

    raw_df = raw_df.rename(columns={'timestamp': 'TimeStamp',
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

    eagleye_df['elapsed_time'] = eagleye_df['TimeStamp'] - eagleye_df['TimeStamp'][0]
    raw_df['elapsed_time'] = raw_df['TimeStamp'] - raw_df['TimeStamp'][0]
    return eagleye_df , raw_df

def xyz2enu(df,org_xyz):
    org_llh = xyz2llh(org_xyz)
    phi = org_llh[0]
    lam = org_llh[1]
    sinphi = np.sin(phi)
    cosphi = np.cos(phi)
    sinlam = np.sin(lam)
    coslam = np.cos(lam)
    R = np.matrix([[-sinlam , coslam , 0],[-sinphi*coslam , -sinphi*sinlam , cosphi],[cosphi*coslam  , cosphi*sinlam , sinphi]])
    
    set_enu_data: List[float] = []
    for i in range(len(df)):
        if df['ecef_x'][i] == 0 and df['ecef_y'][i] == 0:
            set_enu_data.append([0,0,0])
            continue
        diff_xyz = np.array([[df['ecef_x'][i] - org_x] , [df['ecef_y'][i] - org_y] , [df['ecef_z'][i] - org_z]])
        enu = R * diff_xyz
        x: float = float(enu[0])
        y: float = float(enu[1])
        z: float = float(enu[2])
        time = df['elapsed_time'][i]
        distance = df['distance'][i]
        if 'qual' in df.columns:
            qual = df['qual'][i]
        else:
            qual = 1
        set_enu_data.append([time,distance,x,y,z,qual])
    set_enu = pd.DataFrame(set_enu_data,columns=['elapsed_time','distance','x','y','z','qual'])
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

def llh2xyz(df):
    set_xyz_data: List[float] = []
    first_time = 0
    for i in range(len(df)):
        if df['latitude'][i] == 0 and df['longitude'][i] == 0:
            first_time = df['TimeStamp'][i]
            continue
        elif first_time == 0:
            first_time = df['TimeStamp'][i]
        lat = df['latitude'][i]
        lon = df['longitude'][i]
        ht = df['altitude'][i]
        time = (df['TimeStamp'][i] - first_time) * 10 ** (-9)
        distance = df['distance'][i]
        if 'qual' in df.columns:
            qual = df['qual'][i]
        else:
            qual = 1

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
        set_xyz_data.append([time,distance,x,y,z,qual])
    set_xyz = pd.DataFrame(set_xyz_data,columns=['elapsed_time','distance','ecef_x','ecef_y','ecef_z','qual'])
    return set_xyz


def xyz2enu_vel(df,org_xyz):
    x_vel = df['vel_x']
    y_vel = df['vel_y']
    z_vel = df['vel_z']

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

    enu_vel = [e_vel,n_vel,u_vel]
    return enu_vel

def set_heading_deg(eagleye_df):
    set_heading_data: List[float] = []
    for i in range(len(eagleye_df)):
        heading_1st_deg_tmp = change_anglel_limit(eagleye_df['heading_1st'][i])
        heading_2nd_deg_tmp = change_anglel_limit(eagleye_df['heading_2nd'][i])
        heading_3rd_deg_tmp = change_anglel_limit(eagleye_df['heading_3rd'][i])
        heading_1st_deg = math.degrees(heading_1st_deg_tmp)
        heading_2nd_deg = math.degrees(heading_2nd_deg_tmp)
        heading_3rd_deg = math.degrees(heading_3rd_deg_tmp)
        set_heading_data.append([heading_1st_deg,heading_2nd_deg,heading_3rd_deg])
    df = pd.DataFrame(set_heading_data,columns=['heading_1st_deg','heading_2nd_deg','heading_3rd_deg'])
    return df

def change_anglel_limit(heading):
    while heading < -math.pi or math.pi < heading:
        if heading < -math.pi:
            heading += math.pi * 2
        else:
            heading -= math.pi * 2
    return heading

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

def plot_xyz(ax, eagleye_enu, rtk_enu, raw_enu, elem, title, y_label):
    if elem in raw_enu.columns:
        ax.plot(raw_enu['elapsed_time'] , raw_enu[elem] , marker=".", linestyle="None",markersize=1, color = "red",  label="gnss raw data")
    if elem in rtk_enu.columns:
        ax.plot(rtk_enu['elapsed_time'] , rtk_enu[elem] , marker="o", linestyle="None",markersize=1, color = "green",  label="gnss rtk data")
    if elem in eagleye_enu.columns:
        ax.plot(eagleye_enu['elapsed_time'] , eagleye_enu[elem] , marker="s", linestyle="None",markersize=1, alpha=0.3, color = "blue",  label="eagleye")
    ax.set_xlabel('time [s]')
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.grid()

def plot_rpy(ax, eagleye_plot_df, elem, title, y_label):
    if elem in eagleye_plot_df.columns:
        ax.plot(eagleye_plot_df['elapsed_time'] , eagleye_plot_df[elem] , marker="s", linestyle="None",markersize=1, alpha=0.3, color = "blue",  label="eagleye")
    ax.set_xlabel('time [s]')
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.legend(loc='upper right')
    ax.grid()

def plot_6DoF(eagleye_enu, rtk_enu, raw_enu, eagleye_plot_df):
    fig1 = plt.figure()
    ax_x = fig1.add_subplot(2, 3, 1)
    ax_y = fig1.add_subplot(2, 3, 2)
    ax_z = fig1.add_subplot(2, 3, 3)
    ax_roll = fig1.add_subplot(2, 3, 4)
    ax_pitch = fig1.add_subplot(2, 3, 5)
    ax_yaw = fig1.add_subplot(2, 3, 6)
    plot_xyz(ax_x, eagleye_enu, rtk_enu, raw_enu, 'x', 'X (East-West)','East [m]')
    plot_xyz(ax_y, eagleye_enu, rtk_enu, raw_enu, 'y', 'Y (North-South)','North [m]')
    plot_xyz(ax_z, eagleye_enu, rtk_enu, raw_enu, 'z', 'Z (Height)','Height [m]')
    plot_rpy(ax_roll, eagleye_plot_df, 'rolling', 'Roll' , 'Roll [deg]')
    plot_rpy(ax_pitch, eagleye_plot_df, 'pitching', 'Pitch', 'Pitch [deg]')
    plot_rpy(ax_yaw, eagleye_plot_df, 'heading', 'Yaw', 'Yaw [deg]')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", action="store")
    parser.add_argument("-r", "--reverse_imu", action="store_true",help="change reverse_imu true")
    args= parser.parse_args()
    print(args)
    input_ref_path=sys.argv[1]
    reverse_imu: bool = args.reverse_imu

    eagleye_df,raw_df = set_df(input_ref_path)
    print("set ref_data")

    eagleye_ecef_base = pd.concat([eagleye_df['ecef_base_x'],eagleye_df['ecef_base_y'],eagleye_df['ecef_base_z']],axis=1)
    for i in range(len(eagleye_ecef_base)):
        if not eagleye_ecef_base['ecef_base_x'][i] == 0 and eagleye_ecef_base['ecef_base_y'][i]:
            org_x = eagleye_ecef_base['ecef_base_x'][i]
            org_y = eagleye_ecef_base['ecef_base_y'][i]
            org_z = eagleye_ecef_base['ecef_base_z'][i]
            break
    org_xyz = [org_x,org_y,org_z]

    eagleye_llh = pd.concat([eagleye_df['TimeStamp'],eagleye_df['distance'],eagleye_df['latitude'],eagleye_df['longitude'],eagleye_df['altitude'],eagleye_df['qual']],axis=1)
    eagleye_xyz = llh2xyz(eagleye_llh)
    eagleye_enu = xyz2enu(eagleye_xyz,org_xyz)
    print("calc eagleye enu")

    raw_llh = pd.concat([raw_df['TimeStamp'],raw_df['distance'],raw_df['latitude'],raw_df['longitude'],raw_df['altitude']],axis=1)
    raw_xyz = llh2xyz(raw_llh)
    raw_enu = xyz2enu(raw_xyz,org_xyz)
    print("calc raw enu")

    rtk_llh_tmp = pd.concat([raw_df['TimeStamp'],raw_df['distance'],raw_df['rtk_latitude'],raw_df['rtk_longitude'],raw_df['rtk_altitude'],raw_df['qual']],axis=1)
    rtk_llh = rtk_llh_tmp.rename(columns={'rtk_latitude': 'latitude', 'rtk_longitude': 'longitude','rtk_altitude': 'altitude'})
    rtk_xyz = llh2xyz(rtk_llh)
    rtk_enu = xyz2enu(rtk_xyz,org_xyz)
    print("calc rtk enu")

    rtk_enu_single = judge_qual(rtk_enu,1)
    rtk_enu_differential = judge_qual(rtk_enu,2)
    rtk_enu_fix = judge_qual(rtk_enu,4)
    rtk_enu_float = judge_qual(rtk_enu,5)

    raw_xyz_vel = pd.concat([raw_df['vel_x'],raw_df['vel_y'],raw_df['vel_z']],axis=1)
    vel = xyz2enu_vel(raw_xyz_vel,org_xyz)
    vel_2d = (vel[0] ** 2 + vel[1] ** 2) ** 0.5

    eagleye_heading = pd.concat([eagleye_df['heading_1st'],eagleye_df['heading_2nd'],eagleye_df['heading_3rd']],axis=1)
    eagleye_df_tmp = set_heading_deg(eagleye_heading)
    eagleye_df['heading_1st_deg'] = eagleye_df_tmp['heading_1st_deg']
    eagleye_df['heading_2nd_deg'] = eagleye_df_tmp['heading_2nd_deg']
    eagleye_df['heading_3rd_deg'] = eagleye_df_tmp['heading_3rd_deg']
    eagleye_df['heading'] = eagleye_df_tmp['heading_3rd_deg']


    eagleye_plot_df = pd.concat([eagleye_df['elapsed_time'],eagleye_df['distance'],eagleye_df['rolling'],eagleye_df['pitching'],eagleye_df['heading'],raw_df['qual']],axis=1)
    plot_6DoF(eagleye_enu, rtk_enu, raw_enu, eagleye_plot_df)

    fig2 = plt.figure()
    ax_sf = fig2.add_subplot(2, 1, 1)
    ax_sf.set_title('Velocity scal factor')
    ax_sf.plot(eagleye_df['elapsed_time'] , eagleye_df['sf'] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
    ax_sf.set_xlabel('Time [s]')
    ax_sf.set_ylabel('Velocity scal factor []')
    ax_sf.legend()
    ax_sf.grid()

    ax_vel = fig2.add_subplot(2, 1, 2)
    ax_vel.set_title('Velocity')
    ax_vel.plot(raw_df['elapsed_time'] , vel_2d ,  marker="s",linestyle="None",markersize=1 , color = "green",  label="dopplor")
    ax_vel.plot(raw_df['elapsed_time'] , raw_df['velocity'] , marker=".",linestyle="None",markersize=1, color = "red",  label="can velocity")
    ax_vel.plot(eagleye_df['elapsed_time'] , eagleye_df['velocity'] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
    ax_vel.set_xlabel('Time [s]')
    ax_vel.set_ylabel('Velocity [m/s]')
    ax_vel.legend()
    ax_vel.grid()

    fig4 = plt.figure()
    ax_traj = fig4.add_subplot(1, 2, 1)
    ax_traj.set_title('2D Trajectory')
    ax_traj.plot(raw_enu['x']-eagleye_enu['x'][0] , raw_enu['y']-eagleye_enu['y'][0] , marker=".",linestyle="None",markersize=1, color = "red",  label="gnss raw data")
    ax_traj.plot(rtk_enu['x']-eagleye_enu['x'][0] , rtk_enu['y']-eagleye_enu['y'][0] , marker="o",linestyle="None",markersize=1, color = "green",  label="gnss rtk data")
    ax_traj.plot(eagleye_enu['x']-eagleye_enu['x'][0] , eagleye_enu['y']-eagleye_enu['y'][0] ,  marker="s",linestyle="None",markersize=1,alpha=0.3 , color = "blue",  label="eagleye")
    ax_traj.set_xlabel('East [m]')
    ax_traj.set_ylabel('North [m]')
    ax_traj.legend()
    ax_traj.grid()
    ax_traj.set_aspect('equal')
    ax_traj.axis('square')

    ax_fix = fig4.add_subplot(1, 2, 2)
    ax_fix.set_title('GNSS positioning solution')
    ax_fix.plot(rtk_enu_single['x']-eagleye_enu['x'][0] , rtk_enu_single['y']-eagleye_enu['y'][0] , marker=".",linestyle="None",markersize=1, color = "red",  label="gnss single")
    ax_fix.plot(rtk_enu_differential['x']-eagleye_enu['x'][0] , rtk_enu_differential['y']-eagleye_enu['y'][0] , marker=".",linestyle="None",markersize=1, color = "blue",  label="gnss differential")
    ax_fix.plot(rtk_enu_fix['x']-eagleye_enu['x'][0] , rtk_enu_fix['y']-eagleye_enu['y'][0] , marker=".",linestyle="None",markersize=1, color = "green",  label="gnss fix")
    ax_fix.plot(rtk_enu_float['x']-eagleye_enu['x'][0] , rtk_enu_float['y']-eagleye_enu['y'][0] , marker=".",linestyle="None",markersize=1, color = "yellow",  label="gnss float")    
    ax_fix.set_xlabel('East [m]')
    ax_fix.set_ylabel('North [m]')
    ax_fix.legend()
    ax_fix.grid()
    ax_fix.set_aspect('equal','box')
    ax_fix.axis('square')

    fig5 = plt.figure()
    ax_3d = fig5.add_subplot(projection='3d')
    ax_3d.set_title('3D Trajectory')
    ax_3d.plot3D(raw_enu['x'] , raw_enu['y'] ,raw_enu['z'] , marker=".",linestyle="None",markersize=1, color = "red",label="gnss raw data")
    ax_3d.plot3D(rtk_enu['x'] , rtk_enu['y'] ,rtk_enu['z'] , marker="o",linestyle="None",markersize=1, color = "green",label="gnss rtk data")
    ax_3d.plot3D(eagleye_enu['x'] , eagleye_enu['y'] ,eagleye_enu['z'] , marker="s",linestyle="None",markersize=1, alpha=0.3, color = "blue",label="eagleye")
    ax_3d.set_xlabel('East [m]')
    ax_3d.set_ylabel('North [m]')
    ax_3d.set_zlabel('Height [m]')
    ax_3d.legend()
    ax_3d.grid()

    plt.show()
    