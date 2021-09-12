# 火箭悬停控制
import krpc
from my_math_nb import *
from math import pi
from numpy import *
import time
from pid_nb import clamp, PID

conn = krpc.connect(name='controller')
space_center = conn.space_center # SpaceCenter对象
vessel = space_center.active_vessel # 当前载具
body = vessel.orbit.body # 当前载具所处的天体
vessel_frame = vessel.reference_frame # 机体系
body_frame = body.reference_frame # 地固系

# 构造发射台着陆点坐标系
lat = -0.0972
lon = -74.5577
# 绕y轴旋转-lon度
temp1 = space_center.ReferenceFrame.create_relative(body_frame,
    rotation=(0., sin(-lon / 2. * pi / 180), 0., cos(-lon / 2. * pi / 180)))
# 绕z轴旋转lat度
temp2 = space_center.ReferenceFrame.create_relative(temp1,
    rotation=(0., 0., sin(lat / 2. * pi / 180), cos(lat / 2. * pi / 180)))
# 沿x轴平移
height = body.surface_height(lat, lon) + body.equatorial_radius
target_frame = space_center.ReferenceFrame.create_relative(temp2,
    position=(height, 0., 0.))


def roll_cal():  # 在pitch和yaw较小时近似计算roll角
    k_v=(0,0,1)  # 机体系z轴方向单位向量
    k_l=space_center.transform_direction(k_v,vessel_frame,target_frame) # 机体系坐标变换至着陆点坐标系
    k_k = (0,0,1) # 着陆点坐标系Z轴方向单位向量
    arg = angle_between_vectors(k_l,k_k)
    if k_l[1] > 0 and k_l[2] > 0 :
        return pi/2 - arg
    elif k_l[1] < 0 and k_l[2] > 0 :
        return pi/2 + arg
    elif k_l[1] > 0 and k_l[2] < 0 :
        return pi/2 - arg
    elif k_l[1] < 0 and k_l[2] < 0 :
        return arg-pi*3/2


# 计算pitch
def pitch_cal():
    i_l = (1, 0, 0)  # 着陆点坐标系中x轴方向单位向量
    global i_v
    i_v = space_center.transform_direction(i_l,target_frame,vessel_frame) # 着陆点坐标系坐标变换至机体系
    i_i = (1, 0, 0)  # 机体系x轴方向单位向量
    k_k = (0, 0, 1)  # 机体系z轴方向单位向量
    pitch_hor = cross_product(i_i,i_v)
    arg = angle_between_vectors(pitch_hor,k_k)
    if pitch_hor[1] < 0:
        return arg
    else:
        return -arg


# 计算yaw
def yaw_cal():
    i_l = (1,0,0)  # 着陆点坐标系中x轴方向单位向量
    global i_v
    k_k = (0,0,1) # 机体系z轴方向单位向量
    i_i = (1,0,0) # 机体系x轴方向单位向量
    yaw_hor = cross_product(i_v,k_k)
    arg = angle_between_vectors(yaw_hor,i_i)
    if yaw_hor[1] < 0:
        return arg
    else:
        return -arg


# 初始化PID控制器
height_pid = PID(kp=0.08, ki=0.02, kd=0.2)
roll_pid = PID(kp=-0.20, ki = 0, kd=-0.6 )
position_y_pid = PID(kp=-0.02, ki = 0, kd=-0.1)
position_p_pid = PID(kp=-0.02, ki = 0, kd=-0.1)
yaw_pid = PID(kp=-5.0, ki = 0, kd=-4.2)
pitch_pid = PID(kp=5.0, ki = 0, kd=4.2)

h1 = 117
h_landing = 105.1
height_ref = h1
roll_ref = 45
position_ref = (0,0)
yaw_ref = 0; pitch_ref = 0;
game_prev_time = space_center.ut # 记录上一帧时间
roll_flag = False
num_roll = 0
position_flag1 = True
position_flag2 = True
landing_flag = False
landing_cnt = 0
landing_time = 5
while True:
    time.sleep(0.001)
    ut = space_center.ut # 获取游戏内时间
    game_delta_time = ut - game_prev_time # 计算上一帧到这一帧消耗的时间
    if game_delta_time < 0.039: # 如果游戏中还没有经过一个物理帧，不进行计算
        continue

    game_prev_time = ut  # 更新上一帧时间记录
    # 在这里写控制代码

    # 高度控制
    position = vessel.position(target_frame)
    height = position[0]
    error_height = height_ref - height
    vessel.control.throttle = height_pid.update(error_height, game_delta_time)

    # 当高度误差小于5m时，启用roll控制
    roll = roll_cal()
    error_roll = roll_ref * pi / 180 - roll
    if not roll_flag:
        if abs(error_height) < 2 and abs(error_roll) < 0.04:
            num_roll += 1
            if num_roll > 2*20:
                roll_flag = True
        else:
            num_roll = 0

    if abs(error_height) < 5:
        pid_output_roll = roll_pid.update(error_roll, game_delta_time)
        vessel.control.roll = clamp(pid_output_roll,-1,1)
    else:
        roll_pid.reset()

    # position控制
    position_ref1 = (-625.28548 * 0.55, 4.399 * 0.55)
    position_ref2 = (-625.28548 * 0.88, 4.399 * 0.88)
    position_ref3 = (-625.28548, 4.399)
    if position_flag1 and sqrt((position_ref1[0] - position[2])**2 + (position_ref1[1] - position[1])**2 ) > 15:
        position_ref = position_ref1
    else:
        position_flag1 = False
        if position_flag2 and sqrt((position_ref2[0] - position[2])**2 + (position_ref2[1] - position[1])**2 ) >15:
            position_ref = position_ref2
        else:
            position_ref = position_ref3
            position_flag2 = False
    error_z = position_ref[0] - position[2]
    error_y = position_ref[1] - position[1]
    error_py = zy_2_yp(roll,error_z,error_y)
    if abs(error_height) < 5 and roll_flag:
        pitch_ref = position_p_pid.update(error_py[0], game_delta_time)
        yaw_ref = position_y_pid.update(error_py[1], game_delta_time)
        ang_lim = 9*pi/180
        pitch_ref = clamp(pitch_ref,-ang_lim,ang_lim)
        yaw_ref = clamp(yaw_ref,-ang_lim,ang_lim)
    else:
        pitch_ref = 0
        yaw_ref = 0
        position_p_pid.reset()
        position_y_pid.reset()

    # pitch，yaw控制
    pitch = pitch_cal(); yaw = yaw_cal();
    error_pitch = pitch_ref - pitch
    error_yaw = yaw_ref - yaw
    vessel.control.pitch = pitch_pid.update(error_pitch, game_delta_time)
    vessel.control.yaw = yaw_pid.update(error_yaw, game_delta_time)

    # 着陆控制
    if sqrt(error_y**2 + error_z**2) < 0.35 or landing_flag:
        if landing_flag or abs(height - h_landing) < 0.1:
            landing_flag = True
            for engine in vessel.parts.engines:
                if engine.active:
                    engine.active = False
            vessel.parts.pitch = 0
            vessel.control.yaw = 0
            vessel.control.roll = 0
        else:
            height_ref = h1 - landing_cnt/(landing_time*10)*(h1 - h_landing)
            landing_cnt += 1

    # 打印出dt，error,height和roll角

    print('dt=%.3f, error=%.2f, height =%.2f, error_roll = %.2f, roll=%.2f \
pitch = %.2f, yaw = %.2f, pos_east = %.2f, pos_north = %.2f' % (
        game_delta_time, error_height, height, error_roll*180/pi, roll*180/pi,
        pitch*180/pi, yaw*180/pi, position[2], position[1]), end='\r')

    # print('height =%.2f, error_east = %.2f, error_north = %.2f' % (
    #     height, abs(position[2]-position_ref3[0]), abs(position[1]-position_ref3[1])), end='\r')


