"""
@Time : 2023/9/19 21:14
@Author : BreezeConfirming
@Software : Pycharm
@File : AEBCarMoveDemo.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""

from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *
import HDMapAPI as HDMapAPI

from SimOneEvaluation import *

from PythonHDMapAPISample import SampleGetLaneST, SampleGetLaneSample, SampleGetNearLanesWithAngle, \
    SampleGetNearLanes, SampleGetDistanceToLaneBoundary, SampleGetParkingSpaceIds,SampleGetLaneType
import time

import random
import math

import numpy as np
import pandas as pd
from scipy import optimize, stats, signal, linalg
from decimal import Decimal
import sympy

from transforms3d.quaternions import *
from transforms3d.euler import *

import control as ct
from control.matlab import *
import cvxopt

import time
import multiprocessing
import threading
import scipy.io as sio
import csv
import json
import yaml

from controller_trajectory import *

# import  sklearn

"""
CallBack Data Declare

"""

GetObstacle = None
Flag = False
judge = False
endJudge = False


def SoObstacleCB(mainVehicleId, Data_obs):
    global GetObstacle
    if Data_obs is not None:
        GetObstacle = Data_obs


def start():
    print("start")


def stop():
    print("stop")


def SoGetPD(mainVechileId, Data_pd):
    if Data_pd:
        pass


def SoGpsCB(mainVehicleId, Data_Gps):
    if Data_Gps:
        print("mainVehicleId:{0}, posX:{1}, posY: {2}, posZ: {3}".format(mainVehicleId, Data_Gps[0].posX,
                                                                         Data_Gps[0].posY, Data_Gps[0].posZ))


def SoV2XCB(mainVehicleId, sensorId, Data_V2XNFS):
    if Data_V2XNFS:
        print("Data_V2XNFS:{0}, SensorID:{1}, Data_V2XNFS_Size:{2}, Data_V2XNFS_Frame: {3}".format(mainVehicleId,
                                                                                                   sensorId,
                                                                                                   Data_V2XNFS[
                                                                                                       0].V2XMsgFrameSize,
                                                                                                   Data_V2XNFS[
                                                                                                       0].MsgFrameData))


def SoMainVehicleStaus(mainVehicleId, data):
    if data:
        print("mainVehicleId:{0},data:{1}".format(mainVehicleId, data.mainVehicleStatus))


def SoSetSensorDetectionUpdateCBTest(mainVehicleId, sensorId, data):
    if data:
        # print("mainVehicleId:{0},SensorId:{1}, data:{2}".format(mainVehicleId,sensorId,data[0].objectSize))
        for i in range(data[0].objectSize):
            print("data[0][{0}].type:{1}".format(i, data[0].objects[i].type.value))


def db_func(x, a, b, c):
    return a * x ** 2 + b * x + c


def exp_func(x, a, b, c):
    return a * np.exp(b * x) + c


"""
predefined auxiliary function 
"""


def normalize(quaternion):
    """
    Normalize a quaternion

    :param quaternion: An array representing a quaternion (q0, q1, q2, q3)
    :return: A normalized quaternion
    """
    norm = np.linalg.norm(quaternion)
    return quaternion / norm


def quaternion_to_bryant(q):
    '''
    Convert a given quaternion into Euler angles (Bryant angles)
    :param q: A 4 element array representing the quaternion (q0,q1,q2,q3)
    :return: A 3 element array representing the Euler angles (rotation about z, y, x) in radians
    '''
    q = normalize(q)  # It's usually a good idea to normalize the quaternion first
    m = quat2mat(q)  # Convert quaternion to rotation matrix
    angles = mat2euler(m, 'sxyz')  # Convert rotation matrix to Euler (Bryant) angles
    return np.degrees(angles)  # Convert radians to degrees


def get_current_pos(gpsData):


    cX = gpsData.posX
    cY = gpsData.posY


    if cX!=prev_X:

        slope = (cY - prev_Y) / (cX - prev_X)
        cdeg = np.arctan(slope)
    else:
        cdeg=0


    return (cX,cY,cdeg)

"""
predefined algorithm class
"""


class Controller:
    def __init__(self, vehicle, target_speed, dt):
        self.vehicle = vehicle
        self.target_speed = target_speed
        self.dt = dt
        # PID controller for the speed
        self.speed_controller = ct.PIDController(
            kp=1.0, ki=0.0, kd=0.0, dt=self.dt)
        # PID controller for the steering
        self.steering_controller = ct.PIDController(
            kp=1.0, ki=0.0, kd=0.0, dt=self.dt)

    def control(self, target_location):
        # Get the vehicle location
        vehicle_location = self.vehicle.get_location()
        # Get the vehicle speed
        vehicle_speed = self.vehicle.get_velocity()
        vehicle_speed = np.sqrt(
            vehicle_speed.x ** 2 + vehicle_speed.y ** 2 + vehicle_speed.z ** 2)
        # Calculate the error
        speed_error = self.target_speed - vehicle_speed
        # Get the throttle command
        throttle = self.speed_controller.step(speed_error)
        # Calculate the desired steering angle
        desired_steering = np.arctan2(
            target_location.y - vehicle_location.y,
            target_location.x - vehicle_location.x)
        # Get the steering command
        steering = self.steering_controller.step(desired_steering)


"""
initial param list
"""




Event_cnt=0
Target_cnt=3


if __name__ == '__main__':
    mainVehicleID = "0"
    try:
        if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1") == 1:
            print("################## API init success!!!")
            Flag = True
        else:
            print("################## API init fail!!!")
    except Exception as e:
        print(e)
        pass

    # SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_API")
    SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_Driver")

    InitEvaluationServiceWithLocalData(mainVehicleID)

    while 1:
        logger_control = ESimOne_LogLevel_Type(0)
        if HDMapAPI.loadHDMap(20):
            SoSetLogOut(logger_control, "HDMAP LOADED")

            # pass
            break

    velX = np.linspace(0, 10, 100)
    velY = np.linspace(0, 20, 100)
    velT = np.linspace(0, 0.5, 100)
    velS = np.linspace(0, 0.3, 100)

    M = 1696 * 1e3

    r = 33.90

    a_f = 0.6
    a_g=9.3


    cnt = 1

    """
    control data update
    """
    pose_upd = SimOne_Data_Pose_Control()
    control_upd = SimOne_Data_Control()
    print(f" init timestamp is {control_upd.timestamp}")

    control_esp_upd = SimOne_Data_ESP_Control()

    pd_data = SimOne_Data_Point_Cloud()
    gps_data = SimOne_Data_Gps()
    case_data = SimOne_Data_CaseInfo()

    waypointData = SimOne_Data_WayPoints()
    Obstacle = SimOne_Data_Obstacle()
    stream = SimOne_Data_Image()

    car_state = SimOne_Data_Driver_Status()
    img_data = SimOne_Data_Image()
    sencfg_data = SimOne_Data_SensorConfigurations()
    control_stats = SimOne_Data_Control()

    waypoint_data = SimOne_Data_WayPoints()

    # if SoGetGps(mainVehicleID, gps_data):
    #     print(f"init velX : {gps_data.velX}")

    b_cnt = 0
    b_cnt_max = 20
    brake_regression_data = np.linspace(0.01, 0.02, b_cnt_max)

    popt = None
    # initX = -50.7
    initX = None
    initY=None
    endX = None
    endY=None


    """
    AEB Param List
    """

    """
    scenario 2
    """
    initX = -85
    initAudX = -35
    init_diff = initAudX - initX

    gps_accelX = []
    total_dist = 0

    init_vel = 6.94
    init_audVelX = 30 / 3.6
    audAccelX = 2

    tar_dist = 60
    a_g = 9.33
    dt = init_vel / a_g
    aud_dt = init_audVelX / audAccelX
    warning_dist = (tar_dist - 1.8 - (init_vel * dt - 1 / 2 * 10 * dt ** 2))
    aud_maxDist = 3 * init_audVelX + init_audVelX * aud_dt - 1 / 2 * aud_dt ** 2 * audAccelX
    t = sympy.symbols('t')
    total_audDist = 3 * init_audVelX + init_audVelX * t - 1 / 2 * t ** 2 * audAccelX
    total_allowDist = (3 + t) * init_vel
    eq = sympy.Eq(total_allowDist - init_diff, total_audDist)
    sol = sympy.solve(eq, t)

    aud_endVelX = 1 / 2 * init_audVelX

    sol_flag = False

    sol_ans = 1000
    for ans in sol:
        if ans > 0:
            sol_ans = min(sol_ans, ans)
    print("solve collision time is {} and max accel Aud Time is {}".format(sol_ans, aud_dt))

    if sol_ans <= aud_dt and sol_ans > 0:
        sol_flag = True
    else:
        warning_dist = init_diff + aud_maxDist - 1.8 - (init_vel * dt - 1 / 2 * 10 * dt ** 2)




    """
    LKA param list
    """
    sim_start_stamp = 0

    dist_left = None
    dist_right = None

    cur_TimeStamp = None
    prev_TimeStamp = None

    cur_Frame = None
    prev_Frame = None

    with open("LKA_waypoints.txt") as f:
        poly_dots = list(csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC))

    poly_dots = np.asarray(poly_dots)
    # poly_center = np.mean(poly_dots,axis=0)
    poly_center = [poly_dots[1][0], poly_dots[1][1] - 50]
    poly_circle = poly_dots[1:-1, ...]
    param, _ = optimize.curve_fit(circle_curve, (poly_circle[:, 0].T, poly_circle[:, 1].T), np.zeros(3), p0=poly_center)

    """
    APA param list
    """
    stayCarLine=False
    count_abs = 0

    simulate_time = 0

    cur_X = None
    cur_Y = None
    cur_Ax=None
    cur_Vx=None
    cur_Ay=None
    cur_Vy=None
    prev_X = None
    prev_Y = None

    car_degree = None

    stopCarX1,stopCarX2,stopCarY1,stopCarY2 = None,None,None,None


    parkRightKnotX,parkLeftKnotX=None,None

    stepRFlag1=False
    stepRFlag2=False
    stayCarStop = False
    stepOneY=None
    newCycle=True

    while Flag:

        mainVehicleID = "0"
        if newCycle:
            try:
                if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1") == 1:
                    print("################## API init success!!!")
                    Flag = True
                else:
                    print("################## API init fail!!!")
            except Exception as e:
                print(e)
                pass

            # SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_API")
            SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_Driver")

            InitEvaluationServiceWithLocalData(mainVehicleID)

            while 1:
                logger_control = ESimOne_LogLevel_Type(0)
                if HDMapAPI.loadHDMap(20):
                    SoSetLogOut(logger_control, "HDMAP LOADED")

                    # pass
                    break
            SoAPIGetCaseInfo(case_data)
            newCycle=False

            case_zhName = case_data.caseName.decode('utf-8')

        # 02.前方车辆制动
        if case_zhName == "02.前方车辆制动":

            S = SoGetCaseRunStatus()
            if S == 1:
                cnt=0
                total_dist = 0

                GetObstacle = None
                Flag = False
                judge = False
                endJudge = False

                cur_X = None
                cur_Y = None
                cur_Ax = None
                cur_Vx = None
                cur_Ay = None
                cur_Vy = None
                prev_X = None
                prev_Y = None
                car_degree = None
                stopCarX1, stopCarX2, stopCarY1, stopCarY2 = None, None, None, None
                parkRightKnotX, parkLeftKnotX = None, None
                stepRFlag1 = False
                stepRFlag2 = False
                stayCarStop = False
                stepOneY = None
                Event_cnt += 1
                if Event_cnt == Target_cnt:
                    SaveEvaluationRecord()
                time.sleep(0.3)
                S = 0
                newCycle=True
                continue

            if SoGetSensorConfigurations(mainVehicleID, sencfg_data) and cnt % 5 == 0:
                print(sencfg_data.data[0].mainVehicle)
                print(sencfg_data.data[0].sensorType)
                print(sencfg_data.data[0].yaw)

                print(sencfg_data.data[0].sensorId.decode('ascii'))
                print("********")

            control_upd.gear = ESimOne_Gear_Mode(1)
            if SoGetGps(mainVehicleID, gps_data) and gps_data.velX < aud_endVelX:
                control_upd.throttle = 0.10



            if sol_flag:
                warning_dist = (3 + sol) * init_vel - (init_vel * dt - 1 / 2 * 10 * dt ** 2) - 1.5

            if total_dist > warning_dist:
                control_upd.gear = ESimOne_Gear_Mode(3)
                control_upd.brake = 1.0


            if SoGetGps(mainVehicleID, gps_data) and abs(
                    gps_data.velX - init_audVelX + audAccelX * sol_ans) < 0.1 and sol_flag:
                control_upd.brake = 0.0

            if SoGetGps(mainVehicleID, gps_data) and sol_flag and abs(
                    initAudX + aud_maxDist - gps_data.posX - 0.5) < 1e-4:
                control_upd.brake = 1.0


            if not judge and SoGetGps(mainVehicleID, gps_data) and gps_data.velX > 0:
                initX = gps_data.posX
                print("init position is :{}".format(initX))
                judge = True

            if SoSetDrive(mainVehicleID, control_upd) and SoGetGps(mainVehicleID, gps_data) and gps_data.velX > 0:

                if not endJudge and initX is not None:
                    endX = gps_data.posX
                    total_dist = abs(endX - initX)
                    # print("total move distance is {}".format(endX - initX))

                if cnt % 5 == 0 and not endJudge:
                    print("running online: {}".format(S))
                    # print("the brake data {}".format(control_upd.brake))
                    print("accel X: {}".format(gps_data.accelX))
                    print("vel X: {}".format(gps_data.velX))
                    print("brake : {}".format(gps_data.brake))
                    print("throttle : {}".format(gps_data.throttle))

                    print("warning dist is : {}".format(warning_dist))
                    print("total dist is {}".format(total_dist))
                    print("########")

                    Ip = "127.0.0.1"
                    Port = 13956
                    if SoGetStreamingImage(Ip, Port, img_data):
                        print("ImageData.width:{0},ImageData.height:{1}, ImageData.format:{2}, \
                    	 ImageData.imagedata:{3}".format(img_data.width, \
                                                         img_data.height, img_data.format, img_data.imagedata))
                    # print("error is {}".format(gps_data.accelX - tar_accelX))
                    # print(case_data.caseName.decode('utf-8'))
                    print("#########")

                if abs(gps_data.velX) < 1e-4 and not endJudge:
                    endJudge = True
        if case_zhName == "07.垂直泊车驶出":


            S = SoGetCaseRunStatus()
            if S == 1 :
                cnt = 0
                total_dist = 0
                GetObstacle = None
                Flag = False
                judge = False
                endJudge = False

                cur_X = None
                cur_Y = None
                cur_Ax = None
                cur_Vx = None
                cur_Ay = None
                cur_Vy = None
                prev_X = None
                prev_Y = None
                car_degree = None
                stopCarX1, stopCarX2, stopCarY1, stopCarY2 = None, None, None, None
                parkRightKnotX, parkLeftKnotX = None, None
                stepRFlag1 = False
                stepRFlag2 = False
                stayCarStop = False
                stepOneY = None
                Event_cnt += 1
                if Event_cnt==Target_cnt:
                    SaveEvaluationRecord()
                time.sleep(0.3)
                S=0
                newCycle = True
                continue


            if SoGetSensorConfigurations(mainVehicleID, sencfg_data) and cnt % 5 == 0:
                print(sencfg_data.data[0].mainVehicle)
                print(sencfg_data.data[0].sensorType)
                print(sencfg_data.data[0].yaw)
                print(sencfg_data.data[0].sensorId.decode('ascii'))
                print("********")

            parking_space_list = HDMapAPI.getParkingSpaceList()
            parkSpace = parking_space_list.GetElement(1)

            parkRightSpace = parking_space_list.GetElement(parking_space_list.Size()-1)
            parkLeftSpace = parking_space_list.GetElement(0)

            Knots = parkSpace.boundaryKnots

            RightKnots  = parkRightSpace.boundaryKnots
            LeftKnots  = parkLeftSpace.boundaryKnots

            Rk0x,Rk0y = RightKnots.GetElement(0).x,RightKnots.GetElement(0).y
            Rk1x,Rk1y = RightKnots.GetElement(1).x,RightKnots.GetElement(1).y
            Lk0x,Lk0y = LeftKnots.GetElement(0).x,LeftKnots.GetElement(0).y
            Lk1x,Lk1y = LeftKnots.GetElement(1).x,LeftKnots.GetElement(1).y

            K0x,K0y = Knots.GetElement(0).x, Knots.GetElement(0).y
            K1x,K1y = Knots.GetElement(1).x, Knots.GetElement(1).y


            if cnt %50==0 or cnt==1 :
                print(f"k0 : {K0x} and {K0y} ; k1 : {K1x} and {K1y}")
            if not stepRFlag1:
                control_upd.gear = ESimOne_Gear_Mode(1)
            control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)

            if SoGetGps(mainVehicleID,gps_data):
                if cur_X is not None and cur_Y is not None:
                    prev_Y=cur_Y
                    prev_X=cur_X
                cur_X = gps_data.posX
                cur_Y = gps_data.posY
                cur_Vx = gps_data.velX
                cur_Vy = gps_data.velY


            if prev_X is not None and prev_Y is not None and cur_X != prev_X:


                slope = abs(cur_Y-prev_Y) / abs(cur_X - prev_X)
                car_degree =np.rad2deg(np.arctan(slope))



            # control_upd.EBrakeMode = ESimOne_Brake_Mode(4)
            # control_upd.ESteeringMode = ESimOne_Steering_Mode(1)
            if not stepRFlag1 and not stepRFlag2:
                control_upd.throttle = 10/ 3.6
                control_upd.steering = -(1.65/ 45)
                control_upd.brake = 0

            if cur_X is not None and  initX is not None:
                if abs(cur_X - initX) < 15:
                    control_upd.throttle = 10/3.6
                    control_upd.steering =0


            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None and cnt==1:
                print(GetObstacle.contents.obstacleSize)
                print("target pos X1 {} is: ".format(GetObstacle.contents.obstacle[0].posX))
                print("target pos Y1 {} is: ".format(GetObstacle.contents.obstacle[0].posY))
                print("target pos X2 {} is: ".format(GetObstacle.contents.obstacle[1].posX))
                print("target pos Y2 {} is: ".format(GetObstacle.contents.obstacle[1].posY))
                stopCarX1 = GetObstacle.contents.obstacle[0].posX
                stopCarY1 = GetObstacle.contents.obstacle[1].posY
                stopCarX2 = GetObstacle.contents.obstacle[0].posX
                stopCarY2 = GetObstacle.contents.obstacle[1].posY


            if  cur_X is not None:
                if (cur_X > K0x + 5.5 or cur_Y > K1y + 4.2) and not stepRFlag2:
                    control_upd.gear = ESimOne_Gear_Mode(2)
                    control_upd.throttle = 2 / 3.6

                    control_upd.steering = 0
                    stepRFlag1 = True
                if stepRFlag1:
                    control_upd.throttle = 2 / 3.6

                    control_upd.steering = 0
                    if cur_X < K0x + 2.8 and cur_Y < K1y + 2.5:
                        control_upd.steering = 0.9
                        control_upd.throttle = 2.1 / 3.6
                    if cur_Y < K1y - 1.5 and car_degree > 85:
                        control_upd.throttle = 0.8 / 3.6
                        control_upd.steering = -1 / 45
                    if car_degree > 88:
                        control_upd.steering = 0
                    if cur_Y < K0y + 1.5:
                        control_upd.throttle = 0
                        control_upd.steering = 0
                        control_upd.brake = 1
                        control_upd.gear = ESimOne_Gear_Mode(3)
                        SoSetDrive(mainVehicleID, control_upd)
                        stepRFlag2 = True
                        stepRFlag1 = False
                        stayCarStop = True
                        stepOneY = cur_Y

                if stepRFlag2:
                    control_upd.gear = ESimOne_Gear_Mode(0)
                    control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)

                    control_upd.gear = ESimOne_Gear_Mode(1)
                    control_upd.throttle = 2 / 3.6
                    control_upd.steering = 0
                    control_upd.brake = 0  ### notice nonzero brake effect!!!

                    if cur_Y > K1y - 2.6 and car_degree > 0.5:
                        control_upd.steering = 0.8
                        control_upd.throttle = 7 / 3.6
                    if -0.5 < car_degree < 0.5:
                        stayCarLine = True
                        control_upd.steering = 0
                        control_upd.throttle = 20 / 3.6
                    if stayCarLine:
                        control_upd.steering = 0
                        control_upd.throttle = 20 / 3.6



            """
            control data timestamp
            """
            #control_upd.timestamp += 1



            if SoGetDriverControl(mainVehicleID, control_stats) and cnt % 3 == 0:
                # print("control get brake is {}".format(control_stats.brake))
                # print("control get steering is {}".format(control_stats.steering))
                # print("control get steering mode is {}".format(control_stats.ESteeringMode.value))
                # print("control get brake mode is {}".format(control_stats.EBrakeMode.value))

                print(f" current time is {control_upd.timestamp}")
                print("###########")

            simulate_time = control_upd.timestamp

            if simulate_time >= 10:
                pass

            if not judge and SoGetGps(mainVehicleID, gps_data):
                initX = gps_data.posX
                print("init position is :{}".format(initX))
                judge = True

            if SoGetGps(mainVehicleID, gps_data):

                if not endJudge and initX is not None:
                    endX = cur_X
                    total_dist = abs(endX - initX)
                    # print("total move distance is {}".format(endX - initX))

                if cnt % 3 == 0 and not endJudge:
                    print("running online: {}".format(S))
                    # print("the brake data {}".format(control_upd.brake))
                    print("accel X: {}".format(gps_data.accelX))
                    print("vel X: {}".format(gps_data.velX))
                    print("ori X: {}".format(gps_data.oriX))
                    print("rot X: {}".format(gps_data.imuData.rotX))

                    print("car throttle : {}".format(gps_data.throttle))
                    print("*******")
                    # print("car brake : {} and {} than {}".format(gps_data.brake, a_g * gps_data.brake, gps_data.accelX))
                    # print("car steering : {} and {} than {}".format(control_stats.steering, gps_data.steering / 540, \
                    #                                                 np.rad2deg(gps_data.imuData.rotX)))
                    print("*******")

                    print("total dist is {}".format(total_dist))
                    print("current pos  is ({},{})".format(cur_X,cur_Y))
                    print("*******")


            if car_degree is not None:
                if cnt % 3 ==0:
                    print(f"car degree is {car_degree}")
                    print("*******")



                    # Ip = "127.0.0.1"
                    # Port = 13956
                    # if SoGetStreamingImage(Ip, Port, img_data) :
                    # 	print("ImageData.width:{0},ImageData.height:{1}, ImageData.format:{2}, \
                    # 	 ImageData.imagedata:{3}".format( img_data.width, \
                    #                                           img_data.height,  img_data.format, img_data.imagedata))
                    # #print("error is {}".format(gps_data.accelX - tar_accelX))
                    # #print(case_data.caseName.decode('utf-8'))
                    # print("#########")


            SoSetDrive(mainVehicleID, control_upd)
            if S == 1 and not endJudge:
                endJudge = True
            if stayCarStop:
                time.sleep(13)
                stayCarStop=False

        if case_zhName == "05.弯道车道偏离抑制":

            S = SoGetCaseRunStatus()

            if S == 1:
                cnt = 0
                total_dist = 0
                GetObstacle = None
                Flag = False
                judge = False
                endJudge = False

                cur_X = None
                cur_Y = None
                cur_Ax = None
                cur_Vx = None
                cur_Ay = None
                cur_Vy = None
                prev_X = None
                prev_Y = None
                car_degree = None
                stopCarX1, stopCarX2, stopCarY1, stopCarY2 = None, None, None, None
                parkRightKnotX, parkLeftKnotX = None, None
                stepRFlag1 = False
                stepRFlag2 = False
                stayCarStop = False
                stepOneY = None
                Event_cnt += 1
                if Event_cnt == Target_cnt:
                    SaveEvaluationRecord()
                time.sleep(0.3)
                S = 0
                newCycle = True
                continue

            control_upd.gear = ESimOne_Gear_Mode(1)

            if not judge and SoGetGps(mainVehicleID, gps_data):
                initX = gps_data.posX
                initY = gps_data.posY
                sim_start_stamp = gps_data.timestamp / 1000
                print("init position  X is :{}".format(initX))
                print("init position Y is :{}".format(initY))
                print("init timestamp is {}".format(sim_start_stamp))

                judge = True

            if SoGetGps(mainVehicleID, gps_data):
                if cur_X is not None and cur_Y is not None:
                    prev_Y = cur_Y
                    prev_X = cur_X
                cur_X = gps_data.posX
                cur_Y = gps_data.posY
                cur_TimeStamp = gps_data.timestamp / 1000
                cur_Frame = gps_data.frame

                print("current time is {}".format(cur_TimeStamp))

                pos = HDMapAPI.pySimPoint3D(gps_data.posX, gps_data.posY, gps_data.posZ)
                lanesInfo = HDMapAPI.getNearLanes(pos, np.deg2rad(2.0))  # radius

                if pos is not None:
                    SampleGetDistanceToLaneBoundary(pos)
                if lanesInfo.exists:
                    idListSize = lanesInfo.laneIdList.Size()
                    curlaneid = lanesInfo.laneIdList.GetElement(0)

            control_upd.throttle = 0.0
            control_upd.steering = 0
            control_upd.brake = 1.0
            control_upd.gear = ESimOne_Gear_Mode(1)

            current_x, current_y, current_yaw = \
                get_current_pos(gps_data)

            if curlaneid is not None and cnt % 5 == 0:
                SampleGetLaneType(curlaneid)

            if  prev_X is not None and prev_Y is not None:
                if curlaneid is not None and cnt % 5 == 0:
                    SampleGetLaneType(curlaneid)
                if gps_data.velX:
                    if cnt % 5 == 0 and not endJudge:
                        print("accel X: {}".format(gps_data.accelX))
                        print("vel X: {}".format(gps_data.velX))
                        print(f"current position is ({gps_data.posX}, {gps_data.posY})")
                        # print(case_data.caseName.decode('utf-8'))
                        print("#########")
                    cmd_throttle=0.3
                    cmd_throttle = np.fmax(np.fmin(cmd_throttle, 1.0), 0)
                    cmd_steer=0

                    if cur_X < param[0]+0.2:
                        cmd_steer = np.fmax(np.fmin(cmd_steer, 0), -(1/45))
                    else:
                        #cmd_steer=5/45
                        cmd_throttle = 5/3.6
                        control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
                        target_yaw = np.arctan((cur_Y-param[1])/(cur_X-param[0]))



                        print("%%%%%%%%")
                        print(f"current yaw is :{current_yaw}")
                        print(f"target beta is {target_yaw-np.pi/2}")
                        print("%%%%%%%%")

                        if (dist_right+dist_left) > -0.25:
                            cmd_steer = np.fmax(np.fmin(abs(cmd_steer),
                                                np.rad2deg(abs(target_yaw - np.pi / 2)) / 75),
                                               2/45 )
                        elif (dist_right + dist_left) <-0.3:
                            cmd_steer=0

                        else:
                            cmd_steer=4/45
                        if cur_X > 135:
                            cmd_throttle=2/3.6
                        if cur_X>143:
                            cmd_steer=4/45
                            cmd_throttle=2/3.6
                        if cur_Y <14.5:
                            if abs(-np.pi/2-current_yaw)>0.01:
                                cmd_steer=0.5/45
                                cmd_throttle = 5 / 3.6
                            else:
                                cmd_steer=0
                                cmd_throttle = 10 / 3.6



        cnt += 1
        time.sleep(0.5)

        if Event_cnt == Target_cnt:
            break

        # vehicleState = (ESimOne_Data_Vehicle_State * 3)(
        # 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_M_SW,
        # 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_S0_Vz_SM,
        # 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_My_DR_L1);
        # vehicleStatelen = len(vehicleState)
        # if(SoRegisterVehicleState(mainVehicleID,vehicleState,vehicleStatelen)):
        # 	print("RegisterVehicleState Success")

        # vehicleExtraState = SimOne_Data_Vehicle_Extra();
        # if(SoGetVehicleState(mainVehicleID,vehicleExtraState)):
        # 	for i in range(0,vehicleStatelen):
        # 		print("SoGetVehicleState Success: state{0}:= {1}".format(i,vehicleExtraState.extra_states[i]))

        """
        V2X Data
        """

        # SoApiSetV2XInfoUpdateCB(SoV2XCB)
        # SoAPISetMainVehicleStatusUpdateCB(SoMainVehicleStaus)
        # SoApiSetSensorDetectionsUpdateCB(SoSetSensorDetectionUpdateCBTest)
        # SoApiSetGpsUpdateCB(SoGpsCB)

        """
        forward aeb
        """

        # control_upd.steering = 30
        # pred_accelX = 0.3
        # control_upd.brake = M * r * pred_accelX

        # if SoGetGps(mainVehicleID,gps_data) and gps_data.velX<aud_endVelX:
        #     control_upd.EThrottleMode=ESimOne_Throttle_Mode(1)
        #     control_upd.throttle=aud_endVelX

        # control_upd.brake = 0.0263
        #     if b_cnt < b_cnt_max:
        #         control_upd.brake = brake_regression_data[b_cnt]
        #     control_upd.EBrakeMode = ESimOne_Brake_Mode(0)
        #     SoSetDrive(mainVehicleID, control_upd)
        #     if SoGetGps(mainVehicleID,gps_data)  and b_cnt<b_cnt_max:
        #
        #         gps_accelX.append(gps_data.accelX)
        #         b_cnt+=1
        #     if b_cnt==b_cnt_max:
        #         popt,_ =optimize.curve_fit(db_func,
        #                                    gps_accelX,brake_regression_data)
        #         b_cnt+=1
        #     elif b_cnt>b_cnt_max:
        #         tar_accelX=0.5

        # if tar_accelX is not None:
        #     control_upd.brake=db_func(tar_accelX, *popt)

        # if sol_flag:
        #     warning_dist=(3 + sol) * init_vel-(init_vel*dt-1/2*10*dt**2)-1.5
        #
        # if total_dist>warning_dist:
        #     control_upd.gear=ESimOne_Gear_Mode(3)
        #     control_upd.brake = 1.0
        #
        # """
        # 02 depend
        # """
        # if SoGetGps(mainVehicleID,gps_data) and abs(gps_data.velX-init_audVelX+audAccelX*sol_ans)<0.1 and sol_flag:
        #     control_upd.brake=0.0
        #
        #
        # if SoGetGps(mainVehicleID,gps_data) and sol_flag and abs(initAudX+aud_maxDist-gps_data.posX-0.5)<1e-4:
        #     control_upd.brake=1.0

        # control_upd.gear=ESimOne_Gear_Mode(1)
        # control_upd.EThrottleMode = ESimOne_Throttle_Mode(3)
        # control_upd.throttle = -0.5

        # print("Throttle Mode is {}".format(control_upd.EThrottleMode))
        # control_upd.brake=1200

        # if SoGetWayPoints(mainVehicleID,waypointData):
        # 	pass

        # cnt=1;
        # if SoGetGps(mainVehicleID, gpsData):
        # 	cnt+=1
        # 	if cnt%100==0:
        # 		print("timestamp:{0},posX:{1},posY:{2},brake:{3},steering:{4},throttle:{5}".format(gpsData.timestamp,gpsData.posX,gpsData.posY,gpsData.brake,gpsData.steering,gpsData.throttle))
        # 		print("IMU: angVelX:{0}, angVelY: {1}, angVelZ: {2}".format(gpsData.imuData.angVelX, gpsData.imuData.angVelY, gpsData.imuData.angVelZ))

        # obstacleData = SimOne_Data_Obstacle()
        # if SoGetGroundTruth(mainVehicleID,obstacleData):
        # 	print("Size:{0}".format(obstacleData.obstacleSize))

        # v2xData = SimOne_Data_V2XNFS()
        # if SoGetV2XInfo(mainVehicleID,"obu1",1,v2xData):
        # 	print("Size:{0},MsgFrameData:{1}".format(v2xData.V2XMsgFrameSize,v2xData.MsgFrameData))

        """
        deal with planeinfo
        @bug:report get camera  not found due to configure sensor fusion failed
        """
        # planeInfo = SimOne_Data_LaneInfo()
        # if SoGetSensorLaneInfo(mainVehicleID,"sensorFusion1",planeInfo): # "objectBasedCamera1"
        # 	id = planeInfo.id
        # 	laneType = planeInfo.laneType
        # 	laneLeftID = planeInfo.laneLeftID
        # 	laneRightID = planeInfo.laneRightID
        # 	lanePredecessorID = planeInfo.lanePredecessorID
        # 	laneSuccessorID = planeInfo.laneSuccessorID
        # 	print("id: {0}".format(id))
        # 	print("laneType: {0}".format(laneType))
        # 	print("laneLeftID: {0}".format(laneLeftID))
        # 	print("laneRightID: {0}".format(laneRightID))
        # 	i = 0
        # 	while i <3:
        # 		print("lanePredecessorID[{0}]: {1}".format(i, lanePredecessorID[i]))
        # 		i += 1
        # 	i = 0
        # 	while i <3:
        # 		print("laneSuccessorID[{0}]: {1}".format(i, laneSuccessorID[i]))
        # 		i += 1
        # 	l_Line = planeInfo.l_Line
        # 	l_Line_lineID = l_Line.lineID
        # 	l_Line_lineType = l_Line.lineType.value
        # 	l_Line_lineColor = l_Line.lineColor.value
        # 	l_Line_linewidth = l_Line.linewidth
        # 	# l_Line_linePoints = l_Line.linePoints
        # 	# l_Line_linecurveparameter = l_Line.linecurveParameter
        # 	print("l_Line_lineID: {0}".format(l_Line_lineID))
        # 	print("l_Line_lineType: {0}".format(l_Line_lineType))
        # 	print("l_Line_lineColor: {0}".format(l_Line_lineColor))
        # 	print("l_Line_linewidth: {0}".format(l_Line_linewidth))
        # 	c_Line = planeInfo.c_Line
        # 	c_Line_lineID = c_Line.lineID
        # 	c_Line_lineType = c_Line.lineType.value
        # 	c_Line_lineColor = c_Line.lineColor.value
        # 	c_Line_linewidth = c_Line.linewidth
        # 	# c_Line_linePoints = c_Line.linePoints
        # 	# c_Line_linecurveparameter = c_Line.linecurveParameter
        # 	print("c_Line_lineID: {0}".format(c_Line_lineID))
        # 	print("c_Line_lineType: {0}".format(c_Line_lineType))
        # 	print("c_Line_lineColor: {0}".format(c_Line_lineColor))
        # 	print("c_Line_linewidth: {0}".format(c_Line_linewidth))
        # 	r_Line = planeInfo.r_Line
        # 	r_Line_lineID = r_Line.lineID
        # 	r_Line_lineType = r_Line.lineType.value
        # 	r_Line_lineColor = r_Line.lineColor.value
        # 	r_Line_linewidth = r_Line.linewidth
        # 	# r_Line_linePoints = r_Line.linePoints
        # 	# r_Line_linecurveparameter = r_Line.linecurveParameter
        # 	print("r_Line_lineID: {0}".format(r_Line_lineID))
        # 	print("r_Line_lineType: {0}".format(r_Line_lineType))
        # 	print("r_Line_lineColor: {0}".format(r_Line_lineColor))
        # 	print("r_Line_linewidth: {0}".format(r_Line_linewidth))
        # 	ll_Line = planeInfo.ll_Line
        # 	ll_Line_lineID = ll_Line.lineID
        # 	ll_Line_lineType = ll_Line.lineType.value
        # 	ll_Line_lineColor = ll_Line.lineColor.value
        # 	ll_Line_linewidth = ll_Line.linewidth
        # 	# ll_Line_linePoints = ll_Line.linePoints
        # 	# ll_Line_linecurveparameter = ll_Line.linecurveParameter
        # 	print("ll_Line_lineID: {0}".format(ll_Line_lineID))
        # 	print("ll_Line_lineType: {0}".format(ll_Line_lineType))
        # 	print("ll_Line_lineColor: {0}".format(ll_Line_lineColor))
        # 	print("ll_Line_linewidth: {0}".format(ll_Line_linewidth))
        # 	rr_Line = planeInfo.rr_Line
        # 	rr_Line_lineID = rr_Line.lineID
        # 	rr_Line_lineType = rr_Line.lineType.value
        # 	rr_Line_lineColor = rr_Line.lineColor.value
        # 	rr_Line_linewidth = rr_Line.linewidth
        # 	# rr_Line_linePoints = rr_Line.linePoints
        # 	# rr_Line_linecurveparameter = rr_Line.linecurveParameter
        # 	print("rr_Line_lineID: {0}".format(rr_Line_lineID))
        # 	print("rr_Line_lineType: {0}".format(rr_Line_lineType))
        # 	print("rr_Line_lineColor: {0}".format(rr_Line_lineColor))
        # 	print("rr_Line_linewidth: {0}".format(rr_Line_linewidth))
        # 	print("------------ SoGetSensorLaneInfo ------------")

        # pMainVehicleInfo = SimOne_Data_MainVehicle_Info();
        # if SoGetMainVehicleList(pMainVehicleInfo):
        # 	print("	pMainVehicleInfo.size:{0}, pMainVehicleInfo.idList:{1}".format(pMainVehicleInfo.size,pMainVehicleInfo.id_list[0]))
        # 	for index in range(pMainVehicleInfo.size):
        # 		print("pMainVehicleInfo.id:		{0}".format(pMainVehicleInfo.id_list[index].value))
        # 	for indextype in range(pMainVehicleInfo.size):
        # 		print("pMainVehicleInfo.type:		{0}".format(pMainVehicleInfo.type_list[indextype].value))

        # pSensorConfigs = SimOne_Data_SensorConfigurations()
        # if SoGetSensorConfigurations(mainVehicleID, pSensorConfigs):
        # 	for index in range(pSensorConfigs.dataSize):
        # 		print("pSensorConfig.sensorId:{0}, pSensorConfig.SensorType:{1}".format(pSensorConfigs.data[index].sensorId,pSensorConfigs.data[index].sensorType))

        # pHdmapData = SimOne_Data_Map()
        # if SoGetHDMapData(pHdmapData):
        # 	print("pHdmapData.openDrive:{0},pHdmapData.openDriveUrl:{1}".format(pHdmapData.openDrive,pHdmapData.openDriveUrl))

        """
        waypoints

        @bug forbid continuously getting data
        """

        # SoGetWayPoints(mainVehicleID, waypoint_data)

        # print(waypoint_data.wayPoints[0].index)
        # print(f"wp zero x is {waypoint_data.wayPoints[0].posX}")
        # print(f"wp zero y is {waypoint_data.wayPoints[0].posY}")

        # if cnt % 5==0:
        #     print(waypoint_data.wayPointsSize)
        #     for sz in range(waypoint_data.wayPointsSize):
        #
        #         px = waypoint_data.wayPoints[sz].posX
        #         py= waypoint_data.wayPoints[sz].posY
        #         if abs(px)<=1e-5 and abs(py)<=1e-5:
        #             continue
        #
        #         print(f"{px} and {py} is calculated")

        # quat_x=waypoint_data.wayPoints[0].heading_x
        # quat_y=waypoint_data.wayPoints[0].heading_y
        # quat_z=waypoint_data.wayPoints[0].heading_z
        # quat_w=waypoint_data.wayPoints[0].heading_w
        #
        # quater = [quat_x,quat_y,quat_z,quat_w]
        #
        # angles_byant = quaternion_to_bryant(quater)
        # print(angles_byant[0])

        # print(waypoint_data.wayPointsSize)
        # wp_cnt=waypoint_data.wayPointsSize
        # px=waypoint_data.wayPoints[1].posX
        # py=waypoint_data.wayPoints[1].posY
        #
        # print("show wpx {} and wpy {}".format(px,py))

