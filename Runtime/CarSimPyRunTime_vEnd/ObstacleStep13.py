"""
@Time : 2023/9/26 20:48
@Author : BreezeConfirming
@Software : Pycharm
@File : APACarHorizon.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""
from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *
import HDMapAPI as HDMapAPI
from PythonHDMapAPISample import SampleGetParkingSpaceIds, SampleGetParkingSpaceKnots

from SimOneEvaluation import *

import time

import random
import math

import numpy as np
import pandas as pd
from scipy import optimize, stats, signal, linalg
import sympy

import control as ct
from control.matlab import *
# import cvxopt

import time
import threading
import multiprocessing

PosX = 0
PosY = 0
PosZ = 0
Pi = 3.14159265
isTurning = False
finishParking = False


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


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = HDMapAPI.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetNearLanes(pos, radius):
    print("SampleGetNearLanes:")
    nearLanesInfo = HDMapAPI.getNearLanes(pos, radius)
    if nearLanesInfo.exists == False:
        print("Not exists!")
        return
    lanesInfo = nearLanesInfo.laneIdList
    print("lanesInfo size:", lanesInfo.Size())
    print("lanesInfo id list:")
    for i in range(lanesInfo.Size()):
        element = lanesInfo.GetElement(i)
        print(element.GetString())
        print(",")


GetObstacle = None


def SoObstacleCB(mainVehicleId, Data_obs):
    global GetObstacle
    if Data_obs is not None:
        GetObstacle = Data_obs


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output


pose_upd = SimOne_Data_Pose_Control()
control_upd = SimOne_Data_Control()

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
signal_light = SimOne_Data_Signal_Lights()

judge = False

Flag = False
reverse_drive = False
turningLeft = False
turningRight = False
middleStage = False
straightDrive = False

c = 0
initX = 0
initY = 0

previous_X = 0
previous_Y = 0
curr_X = None
curr_Y = None
car_degree = None
# curr_Angle = 0

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

    if not judge and SoGetGps(mainVehicleID, gps_data):
        initX = gps_data.posX
        initY = gps_data.posY
        previous_X = initX
        previous_Y = initY
        print("init position is :{}".format(initX))
        print("init position is :{}".format(initY))
        judge = True

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
    case_zhName = case_data.caseName.decode('utf-8')


    obs_X = None
    obs_Y = None
    last_degree = None


    brakeStage = True
    while Flag:

        if case_zhName == "13.前方障碍物起步":
            S = SoGetCaseRunStatus()
            if S == 1:
                SaveEvaluationRecord()
                break

            control_upd.EThrottleMode = 2
            control_upd.gear = 1
            control_upd.throttle = 2 / 3.6
            control_upd.steering = 0

            signal_light.signalLights = 1 << 1

            # 获取障碍物信息
            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None:
                obs_X = GetObstacle.contents.obstacle[1].posX
                obs_Y = GetObstacle.contents.obstacle[1].posY

            # 计算当前车头角度
            curr_X = gps_data.posX
            curr_Y = gps_data.posY
            if SoGetGps(mainVehicleID, gps_data):
                if curr_X is not None and curr_Y is not None:
                    previous_Y = curr_Y
                    previous_X = curr_X
                curr_X = gps_data.posX
                curr_Y = gps_data.posY

            if curr_X is not None and curr_Y is not None and curr_X != previous_X:
                slope = abs(curr_Y - previous_Y) / abs(curr_X - previous_X)
                car_degree = np.rad2deg(np.arctan(slope))
            # 第一阶段刹车
            stop_posX = 0
            if brakeStage:
                control_upd.gear = 0
                control_upd.EThrottleMode = 0
                control_upd.throttle = 0
                control_upd.brake = 1
                print("in BrakeStage")
                if gps_data.velX == 0 and gps_data.velY == 0:
                    brakeStage = False
                    reverse_drive = True
                    stop_posX = gps_data.posX
                    # turningLeft = True
                    time.sleep(1)
            # 第二阶段，短暂倒车（测试后发现也没有倒车，暂时先这样，出问题了再调整）
            if reverse_drive:
                control_upd.gear = 2
                control_upd.EThrottleMode = 2
                control_upd.throttle = 5 / 3.6
                control_upd.steering = 0
                print("in reverse")
                if abs(gps_data.posX - stop_posX) > 10 and stop_posX is not None:
                    reverse_drive = False
                    turningLeft = True

            # 第三阶段向左
            if turningLeft:
                print("in Left")
                control_upd.steering = -30/45
                if car_degree is not None and last_degree is not None\
                        and car_degree > 40 and abs(gps_data.posY - initY) > 5.3\
                        and car_degree - last_degree < 1:
                    turningLeft = False
                    turningRight = True
                    middleStage = True

            # 第三阶段，中间阶段
            # if middleStage and not turningRight:
            #     control_upd.steering = -40/45
            #     print("in middleStage")
            #     if car_degree is not None and car_degree > 45 and abs(gps_data.posY - initY) > 5:
            #         middleStage = False
            #         turningRight = True

            # 第四阶段，右转回正
            if turningRight:
                print("in Right")
                control_upd.steering = 40/45
                if car_degree is not None and car_degree < 1:
                    turningRight = False
                    straightDrive = True
            # 直行的防偏离抑制
            if straightDrive:
                print("in straight")
                control_upd.throttle = 20/3.6
                if car_degree is not None and car_degree > 0.1 and curr_Y >= previous_Y:
                    control_upd.steering = 0.3 / 30
                if car_degree is not None and car_degree > 0.1 and curr_Y < previous_Y:
                    control_upd.steering = -0.3 / 30

            last_degree = car_degree
            # print(control_upd.throttle)
            # print(control_upd.brake)
            # print(control_upd.steering)
            print(car_degree)
            # print(control_upd.gear)
            SoSetDrive(mainVehicleID, control_upd)
            time.sleep(0.1)