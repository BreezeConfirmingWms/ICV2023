from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *
import HDMapAPI as HDMapAPI

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

import time
import multiprocessing
import threading


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


Flag = False

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

    c = 0
    cnt = 1

    """
    control data update
    """
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
    # simone_data = SimOne_Data()

    # if SoGetGps(mainVehicleID, gps_data):
    #     print(f"init velX : {gps_data.velX}")

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


    b_cnt = 0
    b_cnt_max = 20
    brake_regression_data = np.linspace(0.01, 0.02, b_cnt_max)

    popt = None
    # initX = -50.7
    initX = -80
    initAudX = -50
    init_diff = initAudX - initX
    endX = None
    """
    AEB Param List
    """
    total_dist = 0
    init_vel = 6.94
    init_audVelX = 20 / 3.6
    audAccelX = 0
    tar_dist = 30
    warning_dist = 20
    aud_dist = 0
    judge = None
    endJudge = None
    aud_endVelX = 18 / 3.6
    new_diff = 0
    sim_start_stamp = 0
    cur_TimeStamp = 0
    while Flag:
        # if SoGetGps(mainVehicleID, gps_data):
        #     aud_dist += init_audVelX * 0.1
        #     print("aud_dist is {}".format(aud_dist))
        SoAPIGetCaseInfo(case_data)
        case_zhName = case_data.caseName.decode('utf-8')
        # 14.稳定跟车
        if case_zhName == "14.稳定跟车":
            S = SoGetCaseRunStatus()
            if S == 1:
                # stop
                SaveEvaluationRecord()
                break
            if SoGetSensorConfigurations(mainVehicleID, sencfg_data) and cnt % 5 == 0:
                print(sencfg_data.data[0].mainVehicle)
                print(sencfg_data.data[0].sensorType)
                print(sencfg_data.data[0].yaw)
                print(sencfg_data.data[0].sensorId.decode('ascii'))
                print("********")
            """
            protected
            """

            if not judge and SoGetGps(mainVehicleID, gps_data) and gps_data.velX > 0:
                initX = gps_data.posX
                print("init position is :{}".format(initX))
                sim_start_stamp = gps_data.timestamp / 1000
                print("sim_start_stamp is :{}".format(sim_start_stamp))
                judge = True
            if SoGetGps(mainVehicleID, gps_data):
                cur_TimeStamp = gps_data.timestamp / 1000
                print("cur_TimeStamp is :{}".format(cur_TimeStamp))
            aud_dist = init_audVelX * (cur_TimeStamp - sim_start_stamp)
            print("aud_dist is {}".format(aud_dist))
            control_upd.gear = ESimOne_Gear_Mode(1)
            if SoGetGps(mainVehicleID, gps_data) and gps_data.velX > init_audVelX:
                control_upd.gear = ESimOne_Gear_Mode(3)
                control_upd.brake = 0.1
            if SoGetGps(mainVehicleID, gps_data) and gps_data.velX < aud_endVelX:
                control_upd.gear = ESimOne_Gear_Mode(1)
                control_upd.throttle = 0.1
            if new_diff < warning_dist:
                control_upd.gear = ESimOne_Gear_Mode(3)
                control_upd.brake = 0.1

            """
            02 depend
            """
            if SoGetGps(mainVehicleID, gps_data) and abs(
                    gps_data.velX - init_audVelX) < 0.1:
                control_upd.brake = 0.0
            if SoSetDrive(mainVehicleID, control_upd) and SoGetGps(mainVehicleID, gps_data) and gps_data.velX > 0:
                if not endJudge and initX is not None:
                    endX = gps_data.posX
                    new_diff = init_diff + aud_dist - abs(endX - initX)
                    print("new_diff is {}".format(endX - initX))
                if cnt % 5 == 0 and not endJudge:
                    print("running online: {}".format(S))
                    # print("the brake data {}".format(control_upd.brake))
                    print("accel X: {}".format(gps_data.accelX))
                    print("vel X: {}".format(gps_data.velX))
                    print("brake : {}".format(gps_data.brake))
                    print("throttle : {}".format(gps_data.throttle))

                    print("warning dist is : {}".format(warning_dist))
                    print("########")

                    Ip = "127.0.0.1"
                    Port = 13956
                    if SoGetStreamingImage(Ip, Port, img_data):
                        print("ImageData.width:{0},ImageData.height:{1}, ImageData.format:{2}, \
                    	 ImageData.imagedata:{3}".format(img_data.width, \
                                                         img_data.height, img_data.format, img_data.imagedata))
                    print("#########")

                if abs(gps_data.velX) < 1e-4 and not endJudge:
                    endJudge = True

            if c < 99:
                c += 1
            cnt += 1
            time.sleep(0.1)
