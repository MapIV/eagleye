


from typing import List
import pandas as pd
import numpy as np
import math
import mgrs
import sys

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

def change_anglel_limit(heading):
    set_output: List[float] = []
    for i in range(len(heading)):
        heading_tmp = heading[i]
        while heading_tmp < -math.pi or math.pi < heading_tmp:
            if heading_tmp < -math.pi:
                heading_tmp += math.pi * 2
            else:
                heading_tmp -= math.pi * 2
        set_output.append(heading_tmp)
    output = pd.DataFrame(set_output,columns=['yaw'])
    return output

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

def calc_dopplor_heading(vel):
    heading = np.degrees(np.arctan2(vel['east_vel'], vel['north_vel']))
    df = pd.DataFrame(heading,columns=['yaw'])
    return df

def ll2mgrs(llh):
    set_mgrs: List[float] = []
    m = mgrs.MGRS()
    for i in range(len(llh)):
        ll = (llh['latitude'][i], llh['longitude'][i] , 4)
        coords = m.toMGRS(*ll)
        zone, x, y = coords[:5], float(coords[5:10]), float(coords[10:])
        height = llh['altitude'][i]
        set_mgrs.append([x,y,height])
    xyz = pd.DataFrame(set_mgrs,columns=['x','y','z'])
    return xyz

def calc_distance_vel(velocity, time):
    last_distance = 0
    set_distance: List[float] = []
    for i in range(len(time)):
        if i == 0:
            last_time = time.iloc[i]
            continue
        distance = last_distance + velocity.iloc[i] * (time.iloc[i] - last_time)
        last_time = time.iloc[i]
        last_distance = distance
        set_distance.append([distance])
    distance = pd.DataFrame(set_distance,columns=['distance'])
    return distance

def calc_distance_xy(xy):
    last_distance = 0
    set_distance: List[float] = []
    for i in range(len(xy)):
        if i == 0: 
            distance = 0
        else:
            distance = last_distance + (((xy.iloc[i]['x']-xy.iloc[i-1]['x']) ** 2) + (xy.iloc[i]['y']-xy.iloc[i-1]['y']) ** 2)** 0.5
            last_distance = distance
        set_distance.append([distance])
    distance = pd.DataFrame(set_distance,columns=['distance'])
    return distance

def sync_time(ref_data,csv_data,sync_threshold_time,leap_time): # Time synchronization
    sync_index = np.zeros(len(csv_data['TimeStamp']))

    set_target_index: List[float] = []
    set_ref_index: List[float] = []

    if ref_data['TimeStamp'][0] >  csv_data['TimeStamp'][0]:
        csv_data = csv_data[csv_data['TimeStamp'] > ref_data['TimeStamp'][0] - sync_threshold_time]
        csv_data = csv_data.reset_index(drop=True)
    else:
        ref_data = ref_data[ref_data['TimeStamp'] > csv_data['TimeStamp'][0] - sync_threshold_time]
        ref_data = ref_data.reset_index(drop=True)

    if len(csv_data) == 0 or len(ref_data) == 0:
        print("[ERROR] Target time and ref time ranges do not match.")
        sys.exit(1)

    if ref_data.iloc[-1]['TimeStamp'] < csv_data.iloc[-1]['TimeStamp']:
        csv_data = csv_data[csv_data['TimeStamp'] < ref_data.iloc[-1]['TimeStamp'] + sync_threshold_time]
        csv_data = csv_data.reset_index(drop=True)
    else:
        ref_data = ref_data[ref_data['TimeStamp'] < csv_data.iloc[-1]['TimeStamp'] + sync_threshold_time]
        ref_data = ref_data.reset_index(drop=True)

    if len(csv_data) == 0 or len(ref_data) == 0:
        print("[ERROR] Target time and ref time ranges do not match.")
        sys.exit(1)

    sync_ref_time_tmp = ref_data['TimeStamp']

    num = 0
    len_drop_num = 0
    for i in range(len(csv_data)):
        if i == 0: continue
        time_tmp: List[float] = []
        time_tmp = csv_data.iloc[i]['TimeStamp']-sync_ref_time_tmp + leap_time
        sync_index[i] = np.argmin(abs(time_tmp))
        sync_time_tmp = time_tmp[sync_index[i]]
        if sync_time_tmp < sync_threshold_time:
            tmp_num = int(sync_index[i])
            set_target_index.append(i)
            num = num + len_drop_num
            drop_num = list(range(0,tmp_num - len_drop_num - 1,1))
            if not drop_num == None:
                sync_ref_time_tmp = sync_ref_time_tmp.drop(drop_num, axis=0)
                len_drop_num = len(drop_num)
                sync_ref_time_tmp = sync_ref_time_tmp.reset_index(drop=True)
            set_ref_index.append(num)

    data_df_output = csv_data.iloc[set_target_index]
    data_df_output = data_df_output.reset_index(drop=True)
    data_df_output['elapsed_time'] = data_df_output['TimeStamp'] - data_df_output['TimeStamp'][0]
    ref_df_output = ref_data.iloc[set_ref_index]
    ref_df_output = ref_df_output.reset_index(drop=True)
    ref_df_output['elapsed_time'] = ref_df_output['TimeStamp'] - ref_df_output['TimeStamp'][0]

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
  

def calc_TraRate(diff_2d , eval_step_max): # Calculation of error , x_label , ErrTra_Rate rate
    ErrTra_cnt: List[int] = []
    set_ErrTra_middle: List[float] = []
    set_ErrTra: List[float] = []
    ErrTra_cnt = len(diff_2d)

    step = 0.01
    eval_step_middle = 0.1
    for i in np.arange(0, eval_step_middle, step):
        cnt = 0
        eval_step = i
        for data in diff_2d:
            if data < eval_step:
                cnt = cnt + 1
        rate = cnt / ErrTra_cnt
        set_ErrTra_middle.append([eval_step,rate,rate *100])
    ErrTra_Rate_middle = pd.DataFrame(set_ErrTra_middle,columns=['x_label','ErrTra_Rate','ErrTra'])

    step = 0.1
    for i in np.arange(eval_step_middle, eval_step_max, step):
        cnt = 0
        eval_step = i
        for data in diff_2d:
            if data < eval_step:
                cnt = cnt + 1
        rate = cnt / ErrTra_cnt
        set_ErrTra.append([eval_step,rate,rate *100])
    ErrTra_Rate_tmp = pd.DataFrame(set_ErrTra,columns=['x_label','ErrTra_Rate','ErrTra'])
    ErrTra_Rate = pd.concat([ErrTra_Rate_middle,ErrTra_Rate_tmp])
    return ErrTra_Rate

def quaternion_to_euler_zyx(ori):
    set_eular_angle: List[float] = []
    for i in range(len(ori)):
        q = [ori.iloc[i]['ori_x'],ori.iloc[i]['ori_y'],ori.iloc[i]['ori_z'],ori.iloc[i]['ori_w']]
        r = R.from_quat([q[0], q[1], q[2], q[3]])
        roll = r.as_euler('ZYX', degrees=True)[2]
        pitch = r.as_euler('ZYX', degrees=True)[1]
        yaw = r.as_euler('ZYX', degrees=True)[0]
        set_eular_angle.append([roll,pitch,yaw])
    euler_angle = pd.DataFrame(set_eular_angle,columns=['roll','pitch','yaw'])
    return euler_angle

def calc_error_rpy(ref_angle,eagleye_angle):
    error_rpy = pd.DataFrame()
    error_rpy['roll'] = eagleye_angle['roll'] - ref_angle['roll']
    error_rpy['pitch'] = eagleye_angle['pitch'] - ref_angle['pitch']
    error_rpy['yaw'] = eagleye_angle['yaw'] - ref_angle['yaw']
    for i in range(len(error_rpy)):
        if error_rpy['yaw'][i] > math.degrees(math.pi):
            error_rpy['yaw'][i] = error_rpy['yaw'][i] - 2 * math.degrees(math.pi) 
        elif error_rpy['yaw'][i] < -math.degrees(math.pi):
            error_rpy['yaw'][i] = error_rpy['yaw'][i] + 2 * math.degrees(math.pi)
    return error_rpy

def calc_velocity(vel):
    velocity = pd.DataFrame()
    velocity['velocity'] = (vel['vel_x'] ** 2 + vel['vel_y'] ** 2 + vel['vel_z'] ** 2) ** 0.5
    return velocity

def calc_velocity_error(eagleye_velocity,ref_velocity):
    error_velocity = pd.DataFrame()
    error_velocity['velocity'] = eagleye_velocity - ref_velocity
    return error_velocity

def calc_dr(TimeStamp,distance,eagleye_vel_xyz,ref_xyz,distance_length,distance_step):
    last_distance = 0
    set_dr_trajcetory: List[float] = []
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
                if eagleye_vel_xyz['vel_x'][j] == 0 and eagleye_vel_xyz['vel_y'][j] == 0:
                    last_time = TimeStamp[j]
                    distance_data = distance[j] - start_distance
                    absolute_pos_x = ref_xyz['x'][j] - start_pos_x
                    absolute_pos_y = ref_xyz['y'][j] - start_pos_y
                    continue
                if distance[j] - start_distance < distance_length:
                    dr_pos_x = previous_pos_x + eagleye_vel_xyz['vel_x'][j] * (TimeStamp[j] - last_time)
                    dr_pos_y = previous_pos_y + eagleye_vel_xyz['vel_y'][j] * (TimeStamp[j] - last_time)
                    previous_pos_x = dr_pos_x
                    previous_pos_y = dr_pos_y
                    last_time = TimeStamp[j]
                    distance_data = distance[j] - start_distance
                    absolute_pos_x = ref_xyz['x'][j] - start_pos_x
                    absolute_pos_y = ref_xyz['y'][j] - start_pos_y
                    absolute_dr_pos_x = start_pos_x + dr_pos_x
                    absolute_dr_pos_y = start_pos_y + dr_pos_y
                    set_dr_trajcetory.append([absolute_dr_pos_x,absolute_dr_pos_y])
                else:
                    error_x = absolute_pos_x - dr_pos_x
                    error_y = absolute_pos_y - dr_pos_y
                    error_2d_fabs = math.fabs(error_x ** 2 + error_y ** 2)
                    error_2d= math.sqrt(error_2d_fabs)
                    set_calc_error.append([start_distance,distance_data,absolute_pos_x,dr_pos_x,absolute_pos_y,dr_pos_y,error_x,error_y,error_2d])
                    break
    calc_error = pd.DataFrame(set_calc_error,columns=['start_distance','distance','absolute_pos_x','absolute_pos_y','dr_pos_x','dr_pos_y','error_x','error_y','error_2d'])
    dr_trajcetory = pd.DataFrame(set_dr_trajcetory,columns=['x','y'])
    return calc_error,dr_trajcetory

def calc_dr_twist(TimeStamp,distance,twist_data,ref_heading,ref_xyz,distance_length,distance_step,based_heaing_angle):
    last_distance = 0
    last_heading_integtation = 0
    set_dr_trajcetory: List[float] = []
    set_calc_error: List[float] = []
    for i in range(len(TimeStamp)):
        if i == 0: continue
        if last_distance + distance_step < distance[i]:
            data_set_flag = True
            start_distance = distance[i]
            start_pos_x = ref_xyz['x'][i]
            start_pos_y = ref_xyz['y'][i]
            previous_pos_x = 0
            previous_pos_y = 0
            last_distance = start_distance
            last_time = TimeStamp[i]
            for j in range(i,len(TimeStamp)):
                if data_set_flag == True:
                    last_time = TimeStamp[j]
                    last_heading_integtation = ref_heading[j]
                    distance_data = distance[j] - start_distance
                    absolute_pos_x = ref_xyz['x'][j] - start_pos_x
                    absolute_pos_y = ref_xyz['y'][j] - start_pos_y
                    data_set_flag = False
                    continue
                else:
                    heading_integtation = last_heading_integtation + twist_data['angular_z'][j] * (TimeStamp[j] - last_time)
                    if based_heaing_angle == True:
                        usr_vel_x = math.sin(heading_integtation) * twist_data['velocity'][j]
                        usr_vel_y = math.cos(heading_integtation) * twist_data['velocity'][j]
                    else:
                        usr_vel_x = math.cos(heading_integtation) * twist_data['velocity'][j]
                        usr_vel_y = math.sin(heading_integtation) * twist_data['velocity'][j]
                if distance[j] - start_distance < distance_length:
                    dr_pos_x = previous_pos_x + usr_vel_x * (TimeStamp[j] - last_time)
                    dr_pos_y = previous_pos_y + usr_vel_y * (TimeStamp[j] - last_time)
                    last_heading_integtation = heading_integtation
                    previous_pos_x = dr_pos_x
                    previous_pos_y = dr_pos_y
                    last_time = TimeStamp[j]
                    distance_data = distance[j] - start_distance
                    absolute_pos_x = ref_xyz['x'][j] - start_pos_x
                    absolute_pos_y = ref_xyz['y'][j] - start_pos_y
                    absolute_dr_pos_x = start_pos_x + dr_pos_x
                    absolute_dr_pos_y = start_pos_y + dr_pos_y
                    set_dr_trajcetory.append([absolute_dr_pos_x,absolute_dr_pos_y])
                else:
                    error_x = absolute_pos_x - dr_pos_x
                    error_y = absolute_pos_y - dr_pos_y
                    error_2d_fabs = math.fabs(error_x ** 2 + error_y ** 2)
                    error_2d= math.sqrt(error_2d_fabs)
                    set_calc_error.append([start_distance,distance_data,absolute_pos_x,dr_pos_x,absolute_pos_y,dr_pos_y,error_x,error_y,error_2d])
                    break
    calc_error = pd.DataFrame(set_calc_error,columns=['start_distance','distance','absolute_pos_x','absolute_pos_y','dr_pos_x','dr_pos_y','error_x','error_y','error_2d'])
    dr_trajcetory = pd.DataFrame(set_dr_trajcetory,columns=['x','y'])
    return calc_error,dr_trajcetory

def calc_dr_eagleye(TimeStamp,distance,eagleye_twist_data,ref_heading,ref_xyz,distance_length,distance_step,based_heaing_angle):
    last_distance = 0
    last_heading_integtation = 0
    velocity_stop_threshold = 0.01
    velocity_slip_threshold = 1
    set_dr_trajcetory: List[float] = []
    set_calc_error: List[float] = []
    for i in range(len(TimeStamp)):
        if i == 0: continue
        if last_distance + distance_step < distance[i]:
            data_set_flag = True
            start_distance = distance[i]
            start_pos_x = ref_xyz['x'][i]
            start_pos_y = ref_xyz['y'][i]
            previous_pos_x = 0
            previous_pos_y = 0
            last_distance = start_distance
            last_time = TimeStamp[i]
            for j in range(i,len(TimeStamp)):
                if data_set_flag == True:
                    last_time = TimeStamp[j]
                    last_heading_integtation = ref_heading[j]
                    distance_data = distance[j] - start_distance
                    absolute_pos_x = ref_xyz['x'][j] - start_pos_x
                    absolute_pos_y = ref_xyz['y'][j] - start_pos_y
                    data_set_flag = False
                    continue
                if eagleye_twist_data['velocity'][j] > velocity_stop_threshold:
                    heading_integtation = last_heading_integtation + (eagleye_twist_data['angular_z'][j] + eagleye_twist_data['yaw_rate_offset'][j]) * (TimeStamp[j] - last_time)
                else:
                    heading_integtation = last_heading_integtation + (eagleye_twist_data['angular_z'][j] + eagleye_twist_data['yaw_rate_offset_stop'][j]) * (TimeStamp[j] - last_time)
                if eagleye_twist_data['velocity'][j] > velocity_slip_threshold:
                    heading_correction_slip = heading_integtation + eagleye_twist_data['slip'][j]
                else:
                    heading_correction_slip = heading_integtation
                if distance[j] - start_distance < distance_length:
                    if based_heaing_angle == True:
                        usr_vel_x = math.sin(heading_correction_slip) * eagleye_twist_data['velocity'][j]
                        usr_vel_y = math.cos(heading_correction_slip) * eagleye_twist_data['velocity'][j]
                    else:
                        usr_vel_x = math.cos(heading_correction_slip) * eagleye_twist_data['velocity'][j]
                        usr_vel_y = math.sin(heading_correction_slip) * eagleye_twist_data['velocity'][j]
                    dr_pos_x = previous_pos_x + usr_vel_x * (TimeStamp[j] - last_time)
                    dr_pos_y = previous_pos_y + usr_vel_y * (TimeStamp[j] - last_time)
                    last_heading_integtation = heading_correction_slip
                    previous_pos_x = dr_pos_x
                    previous_pos_y = dr_pos_y
                    last_time = TimeStamp[j]
                    distance_data = distance[j] - start_distance
                    absolute_pos_x = ref_xyz['x'][j] - start_pos_x
                    absolute_pos_y = ref_xyz['y'][j] - start_pos_y
                    absolute_dr_pos_x = start_pos_x + dr_pos_x
                    absolute_dr_pos_y = start_pos_y + dr_pos_y
                    set_dr_trajcetory.append([absolute_dr_pos_x,absolute_dr_pos_y])
                else:
                    error_x = absolute_pos_x - dr_pos_x
                    error_y = absolute_pos_y - dr_pos_y
                    error_2d_fabs = math.fabs(error_x ** 2 + error_y ** 2)
                    error_2d= math.sqrt(error_2d_fabs)
                    set_calc_error.append([start_distance,distance_data,absolute_pos_x,dr_pos_x,absolute_pos_y,dr_pos_y,error_x,error_y,error_2d])
                    break
    calc_error = pd.DataFrame(set_calc_error,columns=['start_distance','distance','absolute_pos_x','absolute_pos_y','dr_pos_x','dr_pos_y','error_x','error_y','error_2d'])
    dr_trajcetory = pd.DataFrame(set_dr_trajcetory,columns=['x','y'])
    return calc_error,dr_trajcetory

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
    return error_table

