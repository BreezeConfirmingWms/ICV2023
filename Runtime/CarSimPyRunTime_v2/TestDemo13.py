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

Flag = False
turningLeft = True
turningRight = False

c = 0
initX = 0
initY = 0

previous_X = 0
previous_Y = 0
curr_Angle = 0

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


    control_upd.ESteeringMode = 0
    transversePID = PIDController(1, 0.5, 1.5)
    while Flag:

        # if case_zhName == "07.垂直泊车驶出":
        # SoApiSetGroundTruthUpdateCB(SoObstacleCB)
        # if GetObstacle is not None:
        #     print(GetObstacle.contents.obstacleSize)
        #     # print(GetObstacle.contents.obstacle[0].posX)
        #     parkingPlace = SampleGetParkingSpaceIds()
        # SampleGetParkingSpaceKnots(parkingPlace[1])

        # control_upd.steering = 0.1
        # if c % 5 == 0:
        #     SoApiSetGroundTruthUpdateCB(SoObstacleCB)
        #     if GetObstacle is not None:
        #         print(GetObstacle.contents.obstacleSize)
        #         for i in range(GetObstacle.contents.obstacleSize):
        #             print(GetObstacle.contents.obstacle[i].posX)

        control_upd.EThrottleMode = 1
        control_upd.gear = 1
        control_upd.brake = 1
        control_upd.throttle = 0.005
        control_upd.steering = -40/45
        # 计算当前车头角度
        if (gps_data.posX - previous_X) != 0:
            curr_Angle = math.atan((gps_data.posY - previous_Y)/(gps_data.posX - previous_X))
            print(curr_Angle)
        if SoGetGps(mainVehicleID, gps_data):
            # print(gps_data.posY)
            pass
        if curr_Angle > 40:
            turningLeft = False
            c += 1

        if c > 2:
            turningRight = True

        if turningLeft:
            control_upd.steering = -40/45
            control_upd.throttle = 0.005

        if turningRight and curr_Angle > 0:
            control_upd.steering = 40/45
            control_upd.throttle = 0.005

        print(control_upd.throttle)
        print(control_upd.gear)

        SoSetDrive(mainVehicleID, control_upd)


        time.sleep(0.1)