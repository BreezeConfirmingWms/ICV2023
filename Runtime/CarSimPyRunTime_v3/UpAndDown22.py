"""
@Time : 2023/9/26 20:48
@Author : BreezeConfirming
@Software : Pycharm
@File : UpAndDown22.py
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

judge = False
parkingPlace = None
Flag = False
c = 0
initX = 0
initY = 0

previous_X = 0
previous_Y = 0
GetObstacle = None
# initX = -80
# initAudX = -50
# init_diff = initAudX - initX
# endX = None
# total_dist = 0
# init_vel = 6.94
# init_audVelX = 20 / 3.6
# audAccelX = 0
# tar_dist = 30
# warning_dist = 20
# aud_dist = 0
# judge = None
# endJudge = None
# aud_endVelX = 17 / 3.6
# new_diff = 0
# sim_start_stamp = 0
# cur_TimeStamp = 0
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
    while Flag:
        SoAPIGetCaseInfo(case_data)
        case_zhName = case_data.caseName.decode('utf-8')
        if case_zhName == "22.上坡-下坡路跟车":
            S = SoGetCaseRunStatus()
            if S == 1:
                # stop
                SaveEvaluationRecord()
                break
            SoSetDrive(mainVehicleID, control_upd)
        # 计算当前车头角度
        #     if (gps_data.posY - previous_Y) != 0:
        #         curr_Angle = math.atan((gps_data.posX - previous_X) / (gps_data.posY - previous_Y))

            if SoGetGps(mainVehicleID, gps_data):
                print(gps_data.posY)
                
             # 获取低速车辆位置
            slowCar_x = 0
            slowCar_y = 0
            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None:
                slowCar_x = GetObstacle.contents.obstacle[0].posX
                slowCar_y = GetObstacle.contents.obstacle[0].posY

            control_upd.EThrottleMode = 2
            control_upd.gear = 1
            control_upd.throttle = 20.5 / 3.6
            print(control_upd.throttle)
            print(control_upd.gear)
            control_upd.steering = 0
            # if c % 10 == 0:
            if (gps_data.posY - slowCar_y) < -0.1:
                control_upd.steering = -0.2 / 30
            elif (gps_data.posY - slowCar_y) > 0.1:
                control_upd.steering = 0.2 / 30
            else:
                control_upd.steering = 0

            # """
            # 控制环节
            # """
            # if gps_data.posX < 0:
            #     control_upd.gear = 1
            #     control_upd.throttle = 10 / 3.6
            # # if (gps_data.posX - initX) > 20 and not finishParking:
            # if gps_data.posX > -10 and not finishParking:
            #     isTurning = True
            # if isTurning:
            #     control_upd.throttle = 5/3.6
            #     control_upd.steering = -(8/45)
            #     # print(gps_data.oriX)
            #     if gps_data.oriX:
            #         pass
            #

            # expectation = 20
            # error_t = expectation - gps_data.oriX
            # control_upd.steering = transversePID.update(error_t, dt)
            # if control_upd.steering > 1:
            #     control_upd.steering = 0.1
            # if control_upd.steering < -1:
            #     control_upd.steering = -0.1

            # print("oriX:",gps_data.oriX)
            # print("转向角度：",control_upd.steering)
            #
            c += 1
            time.sleep(0.1)
