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

import time

import random
import math

import numpy as np
import pandas as pd
from scipy import optimize, stats, signal, linalg
import sympy

import control as ct
from control.matlab import *
import cvxopt

import time
import multiprocessing
import threading


GetObstacle=None

cur_X = None
cur_Y = None
car_X=None
car_Y=None
car_Vel =None
car_Accel =None

def start():
    print("start")


def stop():
    print("stop")


def SoObstacleCB(mainVehicleId, Data_obs):
    global GetObstacle
    if Data_obs is not None:
        GetObstacle = Data_obs

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


    # if SoGetGps(mainVehicleID, gps_data):
    #     print(f"init velX : {gps_data.velX}")



    b_cnt = 0
    b_cnt_max = 20
    brake_regression_data = np.linspace(0.01, 0.02, b_cnt_max)

    popt = None
    # initX = -50.7
    initX = -85
    initAudX = -35
    init_diff = initAudX - initX

    endX = None

    """

    AEB Param List
    """

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

    f_dist = (aud_dt + 3) * init_audVelX
    judge = None
    endJudge = None

    print("Aud_MaxDist is {}".format(aud_maxDist))
    print("Total Ego Dist is {}".format(f_dist))

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

    while Flag:
        if SoGetGps(mainVehicleID, gps_data):
            warning_dist = 221.1 - 1.6 - (gps_data.velX * gps_data.velX / a_g - 1 / 2 * 10 * (gps_data.velX / a_g) ** 2)
        # warning_dist=221-1.8-(init_vel*dt-1/2*10*dt**2)
        SoAPIGetCaseInfo(case_data)

        case_zhName = case_data.caseName.decode('utf-8')

        # 02.前方车辆制动

        if case_zhName == "02.前方车辆制动":

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
            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None:
                car_X = GetObstacle.contents.obstacle[0].posX
                car_Y = GetObstacle.contents.obstacle[0].posY
                car_Vel = GetObstacle.contents.obstacle[0].velX

            control_upd.gear = ESimOne_Gear_Mode(1)
            if SoGetGps(mainVehicleID, gps_data) and gps_data.velX < aud_endVelX:
                control_upd.throttle = 0.10

            if sol_flag:
                warning_dist = (3 + sol) * init_vel - (init_vel * dt - 1 / 2 * 10 * dt ** 2) - 1.5

            if total_dist > warning_dist:
                control_upd.gear = ESimOne_Gear_Mode(3)
                control_upd.brake = 1.0

            """
            02 depend
            """
            if SoGetGps(mainVehicleID,gps_data):
                cur_X = gps_data.posX
                cur_y  =gps_data.posY


            if SoGetGps(mainVehicleID, gps_data) and abs(
                    gps_data.velX - init_audVelX + audAccelX * sol_ans) < 0.1 and sol_flag:
                control_upd.brake = 0.0

            if SoGetGps(mainVehicleID, gps_data) and sol_flag and abs(
                    initAudX + aud_maxDist - gps_data.posX - 0.5) < 1e-4:
                control_upd.brake = 1.0


            if cur_X is not None and car_X is not None:
                if(car_X -cur_X)<1.3:

                    control_upd.brake=1.0

            # control_upd.gear=ESimOne_Gear_Mode(1)
            # control_upd.EThrottleMode = ESimOne_Throttle_Mode(3)
            # control_upd.throttle = -0.5

            # print("Throttle Mode is {}".format(control_upd.EThrottleMode))
            # control_upd.brake=1200

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


                if abs(gps_data.velX) < 1e-4 and not endJudge:
                    endJudge = True

            if c < 99:
                c += 1
            cnt += 1
            time.sleep(0.1)

        # if SoGetWayPoints(mainVehicleID,waypointData):
        # 	pass

        # if SoGetStreamingImage(ip = "127.0.0.1",port= 13956,imageData=stream):
        #     #print(stream.width, stream.height)
        #     pass

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

        #

        # if not SoGetCaseRunStatus():
        #     SaveEvaluationRecord()
        #     break
        # waypoint = SimOne_Data_WayPoints()
        # SoGetWayPoints(mainVehicleID,waypoint)
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
