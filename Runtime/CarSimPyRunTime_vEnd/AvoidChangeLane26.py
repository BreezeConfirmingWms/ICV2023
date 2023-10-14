"""
@Time : 2023/10/10 22:23  
@Author : BreezeConfirming
@Software : Pycharm
@File : AvoidChangeLane26.py
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
straightDrive = False

cnt = 0
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

    SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_API")
    # SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_Driver")
    InitEvaluationServiceWithLocalData(mainVehicleID)
    turnBack = False

    if not judge and SoGetGps(mainVehicleID, gps_data):
        initX = gps_data.posX
        initY = gps_data.posY
        previous_X = initX
        previous_Y = initY
        print("init position is :{}".format(initX))
        print("init position is :{}".format(initY))
        judge = True

    while 1:
        logger_control = ESimOne_LogLevel_Type(0)
        if HDMapAPI.loadHDMap(20):
            SoSetLogOut(logger_control, "HDMAP LOADED")

            # pass
            break

    SoAPIGetCaseInfo(case_data)
    case_zhName = case_data.caseName.decode('utf-8')
    brakeStage = True
    control_upd.ESteeringMode = 0
    danger = True
    while Flag:
        SoAPIGetCaseInfo(case_data)
        case_zhName = case_data.caseName.decode('utf-8')

        if case_zhName == "26.避让故障车辆变道":

            S = SoGetCaseRunStatus()
            if S == 1:
                SaveEvaluationRecord()
                break

            cnt += 1

            control_upd.EThrottleMode = 2
            control_upd.gear = 1
            control_upd.throttle = 20 / 3.6
            control_upd.steering = 0

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

            # 获取故障车辆位置
            tarCar_x = 0
            tarCar_y = 0
            most_Left_car_Y = -10000
            most_Near_car_X = 10000
            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None:
                most_Near_car_X = GetObstacle.contents.obstacle[1].posX
                most_Left_car_Y = GetObstacle.contents.obstacle[1].posY

                if cnt % 5 == 0:
                    print(GetObstacle.contents.obstacleSize)
                    print("target pos X1 is: {}".format(GetObstacle.contents.obstacle[1].posX))
                    print("target pos Y1 is: {}".format(GetObstacle.contents.obstacle[1].posY))
                    # print("target pos X1 is: {}".format(most_Near_car_X))
                    # print("target pos Y1 is: {}".format(most_Left_car_Y))
                    print("current posX:".format(gps_data.posX))
                    print("current posY:".format(gps_data.posY))

            # 判定危险信号

            # 危险解除后回正

            # if (gps_data.posX - slowCar_x) > -60 and (gps_data.posX - slowCar_x) < 60:
            signal_light.signalLights = 2

            if (gps_data.posX - most_Near_car_X) > -50 and \
                    (gps_data.posX - most_Near_car_X) < -10 \
                    and gps_data.posY - most_Left_car_Y < 3.5:
                danger = True

            # 危险情况下的转向控制
            if not turnBack and danger:
                if car_degree is not None and car_degree < 16:
                    control_upd.throttle = 10 / 3.6
                    control_upd.steering = -4 / 30
                if car_degree is not None and car_degree > 16:
                    turnBack = True
                    danger = False
            # 回正阶段
            if turnBack:
                control_upd.throttle = 10 / 3.6
                control_upd.steering = 3 / 30
                # 回正结束，进入直线行驶
                if car_degree < 0.8:
                    turnBack = False
                    straightDrive = True

            if straightDrive:
                control_upd.throttle = 25/3.6
                print("in straight")
                if car_degree is not None and car_degree > 0.1 and curr_Y >= previous_Y:
                    control_upd.steering = 0.3/30
                if car_degree is not None and car_degree > 0.1 and curr_Y < previous_Y:
                    control_upd.steering = -0.3/30



            print(car_degree)
            # print(control_upd.gear)
            SoSetDrive(mainVehicleID, control_upd)
            time.sleep(0.1)