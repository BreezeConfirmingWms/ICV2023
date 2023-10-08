"""
@Time : 2023/9/24 23:08  
@Author : BreezeConfirming
@Software : Pycharm
@File : LKACurveDemo05.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""


"""
@Time : 2023/9/29 11:24  
@Author : BreezeConfirming
@Software : Pycharm
@File : LKA_base.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""

from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *
import HDMapAPI as HDMapAPI
from transforms3d.quaternions import *
from SimOneEvaluation import *

from controller_trajectory import  *

import sys
import os
import time
import logging
import random
import math
import csv

import numpy as np
from matplotlib import pyplot as plt
from scipy import optimize, stats, signal, linalg
import sympy

import control as ct
import cvxopt


GetObstacle = None
Flag = False

judge = False
endJudge = False


cur_X = None
cur_Y = None
prev_X=None
prev_Y = None


cur_Vx = None
cur_Vy = None
cur_Ax = None
cur_Ay = None

cur_TimeStamp = None
prev_TimeStamp = None


cur_Frame = None
prev_Frame = None

sim_start_stamp = 0

dist_left = None
dist_right = None



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
def SoGpsInitCB(mainVehicleId,Data_Gps):
    global sim_start_stamp
    if Data_Gps and sim_start_stamp is None:
        sim_start_stamp = Data_Gps[0].timestamp/1000

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


def SampleGetRoadST(laneId, pos):
    print("SampleGetRoadST:")
    stzInfo = HDMapAPI.getRoadST(laneId, pos)
    if stzInfo.exists == False:
        print("Not exists!")
        return
    # print("[s,t] relative to this road:", stzInfo.s, ",", stzInfo.t)
    # print("height of input point:", stzInfo.z)
    return stzInfo.t


distanceToLaneBoundaryInfo = None
distToLeft = None
distToRight = None


def SampleGetDistanceToLaneBoundary(pos):
    global  dist_left
    global  dist_right
    print("SampleGetDistanceToLaneBoundary:")
    distanceToLaneBoundaryInfo = HDMapAPI.getDistanceToLaneBoundary(pos)
    if distanceToLaneBoundaryInfo.exists == False:
        print("Not exists!")
        return
    print("laneId:", distanceToLaneBoundaryInfo.laneId.GetString())
    dist_left=distanceToLaneBoundaryInfo.distToLeft
    dist_right=distanceToLaneBoundaryInfo.distToRight
    print("distToLeft:", dist_left)
    print("distToRight:", dist_right)
    print("distToLeft2D:", distanceToLaneBoundaryInfo.distToLeft2D)
    print("distToRight2D:", distanceToLaneBoundaryInfo.distToRight2D)

    print("###########")


def SampleGetLaneType(laneId):
    print("SampleGetLaneType:")
    laneType = HDMapAPI.getLaneType(laneId)
    if laneType.exists == False:
        print("Not exists!")
        return
    print("lane type:", laneType.laneType)

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

predefined function 
"""

curlaneid = None
Flag = False
pos = None

judge = False
endJudge = False



M = 1696 * 1e3
r = 33.90
a_f = 0.6


"""
MACROS variables
"""
INTERP_DISTANCE_RES = 0.01
#WP_FILENAME="LKA_waypoints.txt"
WP_FILENAME="poly_waypoints.txt"
INTERP_LOOKAHEAD_DISTANCE = 15
WAITFOR_TIME = 4.0

DIST_THRESHOLD_TO_LAST_WAYPOINT=1.5

"""

save execute time:
@ preprocess

"""


pose_upd = SimOne_Data_Pose_Control()
control_upd = SimOne_Data_Control()
control_esp_upd = SimOne_Data_ESP_Control()

pd_data = SimOne_Data_Point_Cloud()
gps_data = SimOne_Data_Gps()
case_data = SimOne_Data_CaseInfo()
waypointData = SimOne_Data_WayPoints()
Obstacle = SimOne_Data_Obstacle()
controlTrajectory=SimOne_Data_Trajectory()

initX = None
initY = None
initYaw = np.rad2deg(2*np.pi)
endX = None
endY = None



wp_file = WP_FILENAME

waypoints_np = None

with open(wp_file) as f:
    wps = list(csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC))
waypoints_np = np.array(wps)

wp_distance = []
for i in range(1, waypoints_np.shape[0]):
    wp_distance.append(
        np.sqrt((waypoints_np[i, 0] - waypoints_np[i - 1, 0]) ** 2 +
                (waypoints_np[i, 1] - waypoints_np[i - 1, 1]) ** 2))

wp_interp = []
wp_interp_hash = []
interp_counter = 0

for i in range(waypoints_np.shape[0] - 1):
    # Add original waypoint to interpolated waypoints list (and append
    # it to the hash table)
    wp_interp.append(list(waypoints_np[i]))
    wp_interp_hash.append(interp_counter)
    interp_counter += 1

    # Interpolate to the next waypoint. First compute the number of
    # points to interpolate based on the desired resolution and
    # incrementally add interpolated points until the next waypoint
    # is about to be reached.
    num_pts_to_interp = int(np.floor(wp_distance[i] / \
                                     float(INTERP_DISTANCE_RES)) - 1)
    wp_vector = waypoints_np[i + 1] - waypoints_np[i]
    wp_uvector = wp_vector / np.linalg.norm(wp_vector)
    for j in range(num_pts_to_interp):
        next_wp_vector = INTERP_DISTANCE_RES * float(j + 1) * wp_uvector
        wp_interp.append(list(waypoints_np[i] + next_wp_vector))
        interp_counter += 1
    # add last waypoint at the end
wp_interp.append(list(waypoints_np[-1]))
wp_interp_hash.append(interp_counter)
interp_counter += 1

controller = ControllerTraj(wps,control_method='PurePursuit')

num_iterations=10 # doubt temp

with open("LKA_waypoints.txt") as f:
    poly_dots = list(csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC))

poly_dots = np.asarray(poly_dots)
# poly_center = np.mean(poly_dots,axis=0)
poly_center = [poly_dots[1][0], poly_dots[1][1] - 50]
poly_circle = poly_dots[1:-1,...]
param, _ = optimize.curve_fit(circle_curve, (poly_circle[:, 0].T, poly_circle[:, 1].T), np.zeros(3), p0=poly_center)

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

    # SoApiSetV2XInfoUpdateCB(SoV2XCB)
    # SoAPISetMainVehicleStatusUpdateCB(SoMainVehicleStaus)
    # SoApiSetSensorDetectionsUpdateCB(SoSetSensorDetectionUpdateCBTest)
    # SoApiSetGpsUpdateCB(SoGpsCB)

    SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_API")
    InitEvaluationServiceWithLocalData(mainVehicleID)



    while 1:
        logger_control = ESimOne_LogLevel_Type(0)
        if HDMapAPI.loadHDMap(20):
            SoSetLogOut(logger_control, "HDMAP LOADED")

            # pass
            break


    controlTrajectory.trajectory[0].posX=96.1
    controlTrajectory.trajectory[0].posY=48.35
    controlTrajectory.trajectory[0].vel=10/3.6

    controlTrajectory.trajectory[1].posX=131.95
    controlTrajectory.trajectory[1].posY=43.08
    controlTrajectory.trajectory[1].vel=8/3.6
    controlTrajectory.trajectory[2].posX=146.11
    controlTrajectory.trajectory[2].posY=12.77
    controlTrajectory.trajectory[2].vel=4/3.6
    controlTrajectory.trajectory[3].posX=146.86
    controlTrajectory.trajectory[3].posY=-20
    controlTrajectory.trajectory[3].vel=6/3.6


    traject_flag = False
    pos = None

    steer_c1 = 0
    steer_c2 = 0

    c = 0
    cnt = 1

    #SoApiSetGpsUpdateCB(SoGpsInitCB)
    # print(isinstance(sim_start_stamp,float))
    # assert (isinstance(sim_start_stamp,float))




    x_history = [initX]
    y_history = [initY]
    yaw_history = [initYaw]
    time_history = [0]
    speed_history = [0]
    sim_duration = 0


    # for i in range(num_iterations):
    #     # Gather current data
    #     measurement_data, sensor_data = client.read_data()
    #     # Send a control command to proceed to next iteration
    #     send_control_command(client, throttle=0.0, steer=0, brake=1.0)
    #     # Last stamp
    #     if i == num_iterations - 1:
    #         sim_duration = measurement_data.game_timestamp / 1000.0 - \
    #                        sim_start_stamp
    cur_simNum = 0
    reached_the_end = False
    skip_first_frame = True
    closest_index = 0  # Index of waypoint that is currently closest to
    # the car (assumed to be the first index)
    closest_distance = 0  # Closest distance of closest waypoint to car

    while Flag:

            SoAPIGetCaseInfo(case_data)
            case_zhName = case_data.caseName.decode('utf-8')


            if case_zhName == "05.弯道车道偏离抑制":
            #if case_zhName == "19.车道线识别及响应":

                S = SoGetCaseRunStatus()


                if S == 1:
                    # stop
                    SaveEvaluationRecord()
                    break
                control_upd.gear = ESimOne_Gear_Mode(1)

                """
                protected
                """

                # if not traject_flag:
                #     SoSetDriveTrajectory(mainVehicleID,controlTrajectory)
                #     print("trajectory control activated!")
                #     traject_flag = True

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
                    cur_TimeStamp = gps_data.timestamp/1000
                    cur_Frame = gps_data.frame





                    print("current time is {}".format(cur_TimeStamp))
                    print("current frame  is {}".format(cur_Frame))



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

                if cur_simNum<num_iterations and prev_X is not None and prev_Y is not None:
                    if cur_simNum == num_iterations - 1:
                        cur_simNum = 0
                        if sim_duration is None:
                            assert (cur_TimeStamp is not None)
                            sim_duration = cur_TimeStamp  - \
                                           sim_start_stamp

                    SIMULATION_TIME_STEP = sim_duration / float(num_iterations)
                    control_upd.throttle=0.0
                    control_upd.steering=0
                    control_upd.brake=1.0
                    control_upd.gear  =ESimOne_Gear_Mode(1)
                    control_upd.EThrottleMode = ESimOne_Throttle_Mode(0)

                    current_x, current_y, current_yaw = \
                        get_current_pos(gps_data)

                    current_speed = gps_data.velX

                    if cur_TimeStamp <= WAITFOR_TIME + sim_start_stamp:
                        SoSetDrive(mainVehicleID,control_upd)
                        continue
                    else:
                        cur_TimeStamp = cur_TimeStamp- WAITFOR_TIME-sim_start_stamp

                    x_history.append(current_x)
                    y_history.append(current_y)
                    yaw_history.append(current_yaw)
                    speed_history.append(current_speed)
                    time_history.append(cur_TimeStamp)

                    closest_distance = np.linalg.norm(np.array([
                        waypoints_np[closest_index, 0] - current_x,
                        waypoints_np[closest_index, 1] - current_y]))
                    new_distance = closest_distance
                    new_index = closest_index
                    while new_distance <= closest_distance:
                        closest_distance = new_distance
                        closest_index = new_index
                        new_index += 1
                        if new_index >= waypoints_np.shape[0]:  # End of path
                            break
                        new_distance = np.linalg.norm(np.array([
                            waypoints_np[new_index, 0] - current_x,
                            waypoints_np[new_index, 1] - current_y]))
                    new_distance = closest_distance
                    new_index = closest_index
                    while new_distance <= closest_distance:
                        closest_distance = new_distance
                        closest_index = new_index
                        new_index -= 1
                        if new_index < 0:  # Beginning of path
                            break
                        new_distance = np.linalg.norm(np.array([
                            waypoints_np[new_index, 0] - current_x,
                            waypoints_np[new_index, 1] - current_y]))


                    waypoint_subset_first_index = closest_index - 1
                    if waypoint_subset_first_index < 0:
                        waypoint_subset_first_index = 0

                    waypoint_subset_last_index = closest_index
                    total_distance_ahead = 0
                    while total_distance_ahead < INTERP_LOOKAHEAD_DISTANCE:
                        total_distance_ahead += wp_distance[waypoint_subset_last_index]
                        waypoint_subset_last_index += 1
                        if waypoint_subset_last_index >= waypoints_np.shape[0]:
                            waypoint_subset_last_index = waypoints_np.shape[0] - 1
                            break



                    """
                    curve control parameter
                    """



                    if curlaneid is not None and cnt % 5 ==0:
                            SampleGetLaneType(curlaneid)





                    if gps_data.velX:
                        if cnt % 5 == 0 and not endJudge:
                            print("accel X: {}".format(gps_data.accelX))
                            print("vel X: {}".format(gps_data.velX))
                            print(f"current position is ({gps_data.posX}, {gps_data.posY})")
                            # print(case_data.caseName.decode('utf-8'))
                            print("#########")

                    new_waypoints = \
                        wp_interp[wp_interp_hash[waypoint_subset_first_index]: \
                                  wp_interp_hash[waypoint_subset_last_index] + 1]
                    controller.update_waypoints(new_waypoints)
                    controller.update_values(current_x, current_y, current_yaw,
                                             current_speed,
                                             cur_TimeStamp,frame=cnt,closest_distance=new_distance)
                    controller.update_controls()
                    cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()

                    #cmd_steer = np.fmax(np.fmin(cmd_steer, 1.0), -1.0)
                    cmd_throttle = np.fmax(np.fmin(cmd_throttle, 1.0), 0)
                    cmd_brake = np.fmax(np.fmin(cmd_brake, 1.0), 0)

                    cur_simNum+=1

                    if cur_X < param[0]+0.2:
                        cmd_steer = np.fmax(np.fmin(cmd_steer, 0), -(1/45))
                    else:

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
                        elif (dist_right + dist_left) < -0.3:
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

                        #
                        # if 0.1<(dist_right+dist_left)<0.8 :
                        #     cmd_steer=14/45+np.rad2deg(abs(target_yaw-np.pi/2))/120
                        #     if  abs(current_yaw>target_yaw-np.pi/2)<0.3:
                        #         cmd_steer=np.rad2deg(abs(target_yaw-np.pi/2))/90
                        #     cmd_throttle = 3 / 3.6
                        # if -0.5<dist_right+dist_left <- 0.1:
                        #     cmd_steer=-2/45+np.rad2deg(abs(target_yaw-np.pi/2))/180
                        #     if dist_right > 1.5 and np.rad2deg(abs(current_yaw))<45:
                        #         cmd_steer=0
                        #     else:
                        #         cmd_steer=2/45
                        #     cmd_throttle = 2/ 3.6
                        # if abs(target_yaw-np.pi/2-current_yaw)<0.15:
                        #     cmd_steer=0
                        # print("%%%%%%%%")
                        # print(f"current yaw is :{current_yaw}")
                        # print(f"target beta is {target_yaw-np.pi/2}")
                        # print("%%%%%%%%")
                        # if  current_yaw>target_yaw-np.pi/2+0.3 and  dist_left<-1.4:
                        #     cmd_steer = 15/45
                        #     cmd_throttle = 2/3.6
                        #     control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
                        #     # cmd_steer = np.fmax(np.fmin(cmd_steer,(8/45)),(10/45))
                        #     print("too left!!!")
                        # elif  current_yaw<target_yaw-np.pi/2+0.15 and dist_right<1.5:
                        #     cmd_steer = -2/45
                        #     cmd_throttle = 2/3.6
                        #     control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
                        #     # cmd_steer = np.fmax(np.fmin(cmd_steer, -3/45), -(10/45))
                        #     print("too right!!!")
                        # elif abs(current_yaw<target_yaw-np.pi/2)<0.03 :
                        #     cmd_steer = 2/45
                        #     if dist_right<1.5:
                        #         cmd_steer = -3 / 45
                        #     if dist_left < -1.4:
                        #         cmd_steer = 15 / 45
                        #     cmd_throttle = 3/ 3.6
                        #     control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
                        # else :
                        #     cmd_steer =  np.fmax(np.fmin(cmd_steer, -4/45), 4/45)
                        #     cmd_throttle = 3 / 3.6
                        #     control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)

                        # cmd_steer = 8 / 45
                        # cmd_throttle = 3/ 3.6
                        # control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
                        # if abs(target_yaw-np.pi/2-current_yaw)<0.05:
                        #     cmd_steer=-3/45

                        # if abs(current_yaw-target_yaw+np.pi/2)<0.01 or(dist_left>-1.7 and dist_right > 1.5):
                        #     cmd_steer=(1/45)
                        #     cmd_throttle = 4 / 3.6
                        #     control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)



                    control_upd.throttle=cmd_throttle
                    control_upd.steering = cmd_steer
                    control_upd.brake = cmd_brake


                    dist_to_last_waypoint = np.linalg.norm(np.array([
                        wps[-1][0] - current_x,
                        wps[-1][1] - current_y]))
                    if dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
                        reached_the_end = True

                SoSetDrive(mainVehicleID, control_upd)

                cnt += 1
                time.sleep(0.05)




