


from typing import List
import pandas as pd
import numpy as np
import math

from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
from decimal import Decimal, ROUND_HALF_UP

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
        set_eular_angle.append([roll,pitch,yaw])
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

def error_evaluation_each(error, elem):
    data_max = max(error[elem])
    data_average = np.average(error[elem])
    data_std = np.std(error[elem])
    data_rms = np.sqrt(np.square(error[elem]).mean(axis = 0))
    digits_num = '0.01'
    return [elem, \
           (Decimal(str(data_max)).quantize(Decimal(digits_num), rounding=ROUND_HALF_UP) ), \
           (Decimal(str(data_average)).quantize(Decimal(digits_num), rounding=ROUND_HALF_UP) ), \
           (Decimal(str(data_std)).quantize(Decimal(digits_num), rounding=ROUND_HALF_UP) ), \
           (Decimal(str(data_rms)).quantize(Decimal(digits_num), rounding=ROUND_HALF_UP) )]

def error_evaluation(error):
    x = error_evaluation_each(error, 'x')
    y = error_evaluation_each(error, 'y')
    z = error_evaluation_each(error, 'z')
    xy = error_evaluation_each(error, '2d')
    roll = error_evaluation_each(error, 'roll')
    pitch = error_evaluation_each(error, 'pitch')
    yaw = error_evaluation_each(error, 'yaw')
    error_table = pd.DataFrame([x, y, z, xy, roll, pitch, yaw], columns = ['data', 'max', 'average', 'std', 'rms'])
    print(error_table)

