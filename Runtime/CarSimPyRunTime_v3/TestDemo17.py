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
    transversePID = PIDController(1, 0.5, 1.5)
    while Flag:
        SoAPIGetCaseInfo(case_data)
        case_zhName = case_data.caseName.decode('utf-8')

        if case_zhName == "17.避让低速行驶车辆变道":

            S = SoGetCaseRunStatus()
            if S == 1:
                SaveEvaluationRecord()
                break

            cnt += 1
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
            # SoSetDrive(mainVehicleID, control_upd)
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

            # 获取低速车辆位置
            slowCar_x = 0
            slowCar_y = 0
            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None:
                slowCar_x = GetObstacle.contents.obstacle[0].posX
                slowCar_y = GetObstacle.contents.obstacle[0].posY
                if cnt % 5 == 0:
                    print(GetObstacle.contents.obstacleSize)
                    print("target pos X1 {} is: ".format(GetObstacle.contents.obstacle[0].posX))
                    print("target pos Y1 {} is: ".format(GetObstacle.contents.obstacle[0].posY))

            # 判定危险信号
            danger = False
            # 危险解除后回正

            # if (gps_data.posX - slowCar_x) > -60 and (gps_data.posX - slowCar_x) < 60:
            signal_light.signalLights = 1 << 1

            if (gps_data.posX - slowCar_x) > -35 and (gps_data.posX - slowCar_x) < 0 \
                    and abs(gps_data.posY - slowCar_y) < 5:
                danger = True

            # 危险情况下的转向控制
            if not turnBack and danger:
                if car_degree is not None and car_degree < 15:
                    control_upd.throttle = 10 / 3.6
                    control_upd.steering = -4 / 30
            if car_degree is not None and car_degree > 15:
                turnBack = True
                danger = False
            # 回正阶段
            if turnBack:
                control_upd.throttle = 10 / 3.6
                control_upd.steering = 4 / 30
                # 回正结束，进入直线行驶
                if car_degree < 0.8:
                    turnBack = False
                    straightDrive = True

            if straightDrive:
                control_upd.throttle = 20/3.6
                print("in straight")
                if car_degree is not None and car_degree > 0.1 and curr_Y >= previous_Y:
                    control_upd.steering = 0.3/30
                if car_degree is not None and car_degree > 0.1 and curr_Y < previous_Y:
                    control_upd.steering = -0.3/30
                # SoSetDrive(mainVehicleID, control_upd)
                # time.sleep(0.1)
                # continue

            # print(control_upd.throttle)
            # print(control_upd.brake)
            # print(control_upd.steering)
            print(car_degree)
            # print(control_upd.gear)
            SoSetDrive(mainVehicleID, control_upd)
            time.sleep(0.1)