# 仿真系统和自动驾驶系统数据通信接口
########################################################################
# 被测车辆 GPS 和底盘信息

import os
from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *


import time

# Global
PosX = 0
PosY = 0
PosZ = 0

# 方法一：使用回调函数
def SamplegpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


# 方法二：直接调用
def SamplegpsByDirect():
    while (1):
        dataGps = SimOne_Data_Gps()
        if SoGetGps(0, dataGps):
            print(dataGps.posX)
            print(dataGps.posY)
            print(dataGps.posZ)

if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(SamplegpsCB)
    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            time.sleep(2)
    # SamplegpsByDirect()


##########################################################################

import time

# 方法一：使用回调函数
def SampleGetGroundTruthByCB():
    def getGroundTruth(mainVehicleId, data):
    # mainVehicleId 默认为 0
        if data:
            ObstacleSize = data[0].obstacleSize
            print(ObstacleSize)
    SoApiStart()
    SoApiSetGroundTruthUpdateCB(getGroundTruth)
    time.sleep(0.1)

# 方法二：直接调用
def SampleGetGroundTruthByByDirect():
    while (1):
        time.sleep(0.1)
        dataObstacle = SimOne_Data_Obstacle()
        if SoGetGroundTruth(0, dataObstacle):
            print(dataObstacle.obstacleSize)

if __name__ == '__main__':
    SampleGetGroundTruthByCB()
    # SampleGetGroundTruthByByDirect()
    # SampleGetGroundTruthByByDirect()


###########################################################################
# 获取案例主车路径的终点
from SimOneSMStruct import *
def SampleGetWayPoints():
    while(1):
        wayPoints = SimOne_Data_WayPoints_Entry()
    if SoGetTerminalPoint(wayPoints):
        print(wayPoints.posX)
        print(wayPoints.posY)

if __name__ == '__main__':
    SampleGetWayPoints()


#######################################################################
# 被测车辆 Pose 操控


# Global
posX = 0
posY = 0
posZ = 0
timestamp = 0

# 方法一：使用回调函数
def SampleSetGpsUpdateCB():
    def SetGpsUpdateData(mainVehicleId, Data_Gps ):
        global posX, posY, posZ, timestamp
        posX = Data_Gps[0].posX
        posY = Data_Gps[0].posY
        posZ = Data_Gps[0].posZ
        timestamp = Data_Gps[0].timestamp

    SoApiStart()
    SoApiSetGpsUpdateCB(SetGpsUpdateData)

    if posX != 0:
        print("gpsCB: X:{0} Y:{1} Z:{2}".format(posX, posY, posZ))
        time.sleep(2)
        pose.timestamp = timestamp
        pose.posX = posX + 10
        pose.posY = posY + 10
        pose.posZ = posZ
        SoApiSetPose(0, pose)

# 方法二：直接调用
def SampleSetGpsByDirect():
    gps = SimOne_Data_Gps()
    pose = SimOne_Data_Pose_Control()
    while (1):
        time.sleep(0.1)
        SoGetGps(0, gps)
        print(gps.posX, gps.posY, gps.posZ)
        pose.timestamp = gps.timestamp
        pose.posX = gps.posX + 10
        pose.posY = gps.posY + 10
        pose.posZ = gps.posZ
        SoApiSetPose(0, pose)
        print(pose.posX, pose.posY, pose.posZ)

if __name__ == '__main__':
    SampleSetGpsUpdateCB()
    # SampleSetGpsByDirect()


########################################################################
# 被测车辆 Drive 操控
import os,sys
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
posX = 0
timestamp = 0
control = SimOne_Data_Control()
gps = SimOne_Data_Gps()


# 方法一：使用回调函数
def SampleSetDriveCB():
    def SetGpsUpdateData(mainVehicleId, Data_Gps):
        global posX, timestamp
        posX = Data_Gps[0].posX
        timestamp = Data_Gps[0].timestamp
    SoApiStart()
    SoApiSetGpsUpdateCB(SetGpsUpdateData)
    if posX != 0:
        time.sleep(2)
        control.timestamp = timestamp
        control.throttle = 0.1
        control.brake = 0.0
        control.steering = 0.0
        control.handbrake = False
        control.isManualGear = False
        control.gear = EGearMode.EGearManualMode_1
        SoApiSetDrive(0, control)

# 方法二：直接调用
def SampleSetDriveByDirect():
    SoApiStart()
    while (1):
        time.sleep(0.1)
        SoGetGps(0, gps)
        control.timestamp = gps.timestamp
        control.throttle = 0.1
        control.brake = 0.0
        control.steering = 0.0
        control.handbrake = False
        control.isManualGear = False
        control.gear = EGearMode.EGearManualMode_1
        SoApiSetDrive(0, control)

if __name__ == '__main__':
    SampleSetDriveCB()
    # SampleSetDriveByDirect()


#########################################################################
# 设置主车预警信息
import os,sys
import pySimOneSM
import time
from SimOneSMStruct import *
# Global
timestamp = 0

def SampleSetEvent():
    gps = SimOne_Data_Gps()
    vehicleEvent = SimOne_Data_Vehicle_EventInfo()
    SoApiStart()
    while (1):
        SoGetGps(0, gps)
        vehicleEvent.timestamp = gps.timestamp
        vehicleEvent.type = ESimone_Vehicle_EventInfo_Type.ESimOne_VehicleEventInfo_Forward_Collision
        SoApiSetVehicleEvent(0, vehicleEvent)

if __name__ == '__main__':
    SampleSetEvent()


########################################################################
# 获取当前环境相关信息（天气、光照、地面等）
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from SimOneSMStruct import *
import json

# 获取天气
def SampleGetEnvironment():
    getEnvironmentJson = dict()
    getEnvironment = SimOne_Data_Environment()
    if SoGetEnvironment(getEnvironment):
        getEnvironmentJson.update({'timeOfDay': getEnvironment.timeOfDay})
        getEnvironmentJson.update({'directionalLight': getEnvironment.directionalLight})
        getEnvironmentJson.update({'artificialLight': getEnvironment.artificialLight})
        getEnvironmentJson.update({'ambientLight': getEnvironment.ambientLight})
        getEnvironmentJson.update({'heightAngle': getEnvironment.heightAngle})
        getEnvironmentJson.update({'cloudDensity': getEnvironment.cloudDensity})
        getEnvironmentJson.update({'rainDensity': getEnvironment.rainDensity})
        getEnvironmentJson.update({'snowDensity': getEnvironment.snowDensity})
        getEnvironmentJson.update({'groundHumidityLevel': getEnvironment.groundHumidityLevel})
        getEnvironmentJson.update({'groundDirtyLevel': getEnvironment.groundDirtyLevel})
        print(json.dumps(getEnvironmentJson, ensure_ascii=False))

if __name__ == '__main__':
    SampleGetEnvironment()


#################################################################
# 设置当前环境相关信息（天气、光照、地面等）
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from SimOneSMStruct import *
import json
import time

# 设置天气
def SampleSetEnvironment():
    setEnvironmentJson = dict()
    setEnvironment = SimOne_Data_Environment()
    setEnvironment.timeOfDay = 1200
    setEnvironment.snowDensity = 1.0
    SoSetEnvironment(setEnvironment)
    if SoGetEnvironment(setEnvironment):
        setEnvironmentJson.update({'timeOfDay': setEnvironment.timeOfDay})
        setEnvironmentJson.update({'snowDensity': setEnvironment.snowDensity})
    print(json.dumps(setEnvironmentJson, ensure_ascii=False))

if __name__ == '__main__':
    SampleSetEnvironment()


################################################################
# 设置车辆信号灯状态
from SimOneSMStruct import *
import time, sys
control = SimOne_Data_Control()
signalLightsData = SimOne_Data_Signal_Lights()
gps = SimOne_Data_Gps()

def sampleSoSetSignalLights():
    if SoGetGps(0, gps):
        control.timestamp = gps.timestamp
    control.throttle = 0.5
    control.gear = EGearMode.EGearManualMode_1
    SoApiSetDrive(0, control)
    signalLightsData.signalLights |= SimOne_Signal_Light.ESimOne_Signal_Light_LeftBlinker
    SoSetSignalLights(0, signalLightsData)

if __name__ == '__main__':
    sampleSetSignalLights()


############################################################
# 获取 SimOneDriver 运行状态
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from SimOneSMStruct import *
import json
import time

def SampleSoGetDriverStatus():
    DriverStatus = SimOne_Data_Driver_Status()
    while(1):
        time.sleep(0.1)
        if SoGetDriverStatus(0, driverStatus):
        status = DriverStatus.driverStatus.value
        print(status)


if __name__ == '__main__':
    SampleSoGetDriverStatus()


# 高精地图运行时接口
##########################################################
# 查询位置车道
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0

def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)
    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            SampleGetNearMostLane(pos)
            time.sleep(2)


##################################################################
# 获得附近车道
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):

    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearLanes(pos, radius):
    print("SampleGetNearLanes:")
    nearLanesInfo = pySimOneSM.getNearLanes(pos, radius)
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


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)
    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            SampleGetNearLanes(pos, 5)
            time.sleep(2)


###################################################
# 获得面前所有车道
import os
import pySimOneSM
import time
from SimOneSMStruct import *

M_PI = 3.14159265358979323846
# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearLanesWithAngle(pos, radius, headingAngle, shiftAngle):
    print("SampleGetNearLanesWithAngle:")
    nearLanesInfo = pySimOneSM.getNearLanesWithAngle(pos, radius, headingAngle, shiftAngle)
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


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            headingAngle = 30 / 180 * M_PI
            shiftAngle = 90 / 180 * M_PI
            SampleGetNearLanesWithAngle(pos, 5, headingAngle, shiftAngle)
            time.sleep(2)


#######################################################
# 查询位置两侧车道边缘
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetDistanceToLaneBoundary(pos):
    print("SampleGetDistanceToLaneBoundary:")
    distanceToLaneBoundaryInfo = pySimOneSM.getDistanceToLaneBoundary(pos);
    if distanceToLaneBoundaryInfo.exists == False:
        print("Not exists!")
        return
    print("laneId:", distanceToLaneBoundaryInfo.laneId.GetString())
    print("distToLeft:", distanceToLaneBoundaryInfo.distToLeft)
    print("distToRight:", distanceToLaneBoundaryInfo.distToRight)
    print("distToLeft2D:", distanceToLaneBoundaryInfo.distToLeft2D)
    print("distToRight2D:", distanceToLaneBoundaryInfo.distToRight2D)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            SampleGetDistanceToLaneBoundary(pos)
            time.sleep(2)


####################################################
# 查询车道信息
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetLaneSample(laneId):
    print("SampleGetLaneSample:")
    sampleInfo = pySimOneSM.getLaneSample(laneId)
    if sampleInfo.exists == False:
        print("Not exists!")
        return
    leftBoundary = sampleInfo.laneInfo.leftBoundary
    print("leftBoundary knot size:", leftBoundary.Size())
    print("leftBoundary knot list:")
    for i in range(leftBoundary.Size()):
        element = leftBoundary.GetElement(i)
        print("(", element.x, ",", element.y, ",", element.z, "),")
    rightBoundary = sampleInfo.laneInfo.rightBoundary
    print("rightBoundary knot size:", rightBoundary.Size())
    print("rightBoundary knot list:")
    for i in range(rightBoundary.Size()):
        element = rightBoundary.GetElement(i)
        print("(", element.x, ",", element.y, ",", element.z, "),")
    centerLine = sampleInfo.laneInfo.centerLine
    print("centerLine knot size:", centerLine.Size())
    print("centerLine knot list:")
    for i in range(centerLine.Size()):
        element = centerLine.GetElement(i)
        print("(", element.x, ",", element.y, ",", element.z, "),")


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetLaneSample(laneId)
            time.sleep(2)


##############################################
# 获取车道连接信息
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetLaneLink(laneId):
    print("SampleGetLaneLink:")
    laneLinkInfo = pySimOneSM.getLaneLink(laneId)
    if laneLinkInfo.exists == False:
        print("Not exists!")
        return
    laneLink = laneLinkInfo.laneLink
    predecessorIds = laneLink.predecessorLaneIds
    print("predecessorLaneIds size:", predecessorIds.Size())
    if predecessorIds.Size() > 0:
        print("predecessorLaneIds:")
    for i in range(predecessorIds.Size()):
        element = predecessorIds.GetElement(i)
        print(element.GetString())
    successorIds = laneLink.successorLaneIds
    print("successorLaneIds size:", successorIds.Size())
    if successorIds.Size() > 0:
        print("successorLaneIds:")
    for i in range(successorIds.Size()):
        element = successorIds.GetElement(i)
        print(element.GetString())
    print("leftNeighborLaneId:", laneLink.leftNeighborLaneId.GetString())
    print("rightNeighborLaneId:", laneLink.rightNeighborLaneId.GetString())


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetLaneLink(laneId)
            time.sleep(2)


############################################################
# 获取车道类型
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetLaneType(laneId):
    print("SampleGetLaneType:")
    laneType = pySimOneSM.getLaneType(laneId)
    if laneType.exists == False:
        print("Not exists!")
        return
    print("lane type:", laneType.laneType)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetLaneType(laneId)
            time.sleep(2)


#########################################################
# 获取车道宽度
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetLaneWidth(laneId, pos):
    print("SampleGetLaneWidth:")
    laneWidthInfo = pySimOneSM.getLaneWidth(laneId, pos)
    if laneWidthInfo.exists == False:
        print("Not exists!")
        return
    print("lane width:", laneWidthInfo.width)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetLaneWidth(laneId, pos)
            time.sleep(2)


######################################################
# 获取在车道上的 ST 坐标
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetLaneST(laneId, pos):
    print("SampleGetLaneST:")
    stInfo = pySimOneSM.getLaneST(laneId, pos)
    if stInfo.exists == False:
        print("Not exists!")
        return
    print("[s,t] relative to this lane:", stInfo.s, ",", stInfo.t)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetLaneST(laneId, pos)
            time.sleep(2)


###########################################
# 获取在道路上的 ST 坐标
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetRoadST(laneId, pos):
    print("SampleGetRoadST:")
    stzInfo = pySimOneSM.getRoadST(laneId, pos)
    if stzInfo.exists == False:
        print("Not exists!")
        return
    print("[s,t] relative to this road:", stzInfo.s, ",", stzInfo.t)
    print("height of input point:", stzInfo.z)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)
    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetRoadST(laneId, pos)
            time.sleep(2)


#################################################
# 根据指定车道 id 和车道上的 ST 坐标获取局部坐标 xyz
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetInertialFromLaneST(laneId, s, t):
    print("SampleGetInertialFromLaneST:")
    inertialFromLaneSTInfo = pySimOneSM.getInertialFromLaneST(laneId, s, t)
    if inertialFromLaneSTInfo.exists == False:
        print("Not exists!")
        return
    print("inertial vector: (", inertialFromLaneSTInfo.inertial.x, ",", inertialFromLaneSTInfo.inertial.y, ",", inertialFromLaneSTInfo.inertial.z, "),")
    print("dir vector: (", inertialFromLaneSTInfo.dir.x, ",", inertialFromLaneSTInfo.dir.y, ",", inertialFromLaneSTInfo.dir.z, "),")


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            s = 0.1
            t = 3.5
            SampleGetInertialFromLaneST(laneId, s, t)
            time.sleep(2)


#################################################
# 查询指定车道是否存在于地图之中
import os
import pySimOneSM
import time
from SimOneSMStruct import *
# Global
PosX = 0
PosY = 0
PosZ = 0

def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
         print("Not exists!")
         return
    print("lane id:", info.laneId.GetString())
    return info.laneId
def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ
def SampleContainsLane(laneId):
    print("SampleContainsLane:")
    ret = pySimOneSM.containsLane(laneId)
    print("return state:", ret)

if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)
    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleContainsLane(laneId)
            time.sleep(2)


#######################################################
# 获取地图中停车位列表
import os
import pySimOneSM
import time
from SimOneSMStruct import *


def SampleGetParkingSpaceList():
    print("SampleGetParkingSpaceList:")
    parkingSpaceList = pySimOneSM.getParkingSpaceList()
    print("parkingSpace count:", parkingSpaceList.Size())
    for i in range(parkingSpaceList.Size()):
        parkingSpace = parkingSpaceList.GetElement(i)
    print("parkingSpace id:", parkingSpace.id)
    front = parkingSpace.front
    print("roadMark at which side:", front.side.GetString())
    print("roadMark type:", front.type)
    print("roadMark color:", front.color)
    print("roadMark width:", front.width)
    knots = parkingSpace.boundaryKnots
    print("boundaryKnots count:", knots.Size())
    knot0 = knots.GetElement(0)
    print("knot0 point: (", knot0.x, ",", knot0.y, ",", knot0.z, ")")

if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SampleGetParkingSpaceList()               
    

###这里的这个冒号文档就有，可能是多打了###


##########################################################
# 获取预设规划路径
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetPredefinedRoute():
    print("SampleGetPredefinedRoute:")
    routeInfo = pySimOneSM.getPredefinedRoute()
    if routeInfo.exists == False:
        print("Not exists!")
        return
    print("route point count:", routeInfo.route.Size())
    for i in range(routeInfo.route.Size()):
        pt = routeInfo.route.GetElement(i)
        print("route point: (", pt.x, ",", pt.y, ",", pt.z, "),")


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    # SoApiSetGpsUpdateCB(gpsCB)
    SampleGetPredefinedRoute()
    time.sleep(2)


#########################################################
# 获取规划路径所途径道路的 ID 列表
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleNavigate(inputPoints):
    print("SampleNavigate:")
    navigateInfo = pySimOneSM.navigate(inputPoints)
    if navigateInfo.exists == False:
        print("Not exists!")
        return
    print("roadIdList count:", navigateInfo.roadIdList.Size())
    for i in range(navigateInfo.roadIdList.Size()):
        roadId = navigateInfo.roadIdList.GetElement(i)
        print("roadId:", roadId)
    print("indexOfValidPoints count:", navigateInfo.indexOfValidPoints.Size())
    for i in range(navigateInfo.indexOfValidPoints.Size()):
        index = navigateInfo.indexOfValidPoints.GetElement(i)
        print("index:", index)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)

    inputPoints = pySimOneSM.pySimPoint3DVector()
    pt1 = pySimOneSM.pySimPoint3D(562.7, 531.4, 0)
    pt2 = pySimOneSM.pySimPoint3D(837.9, 618.4, 0)
    inputPoints.AddElement(pt1)
    inputPoints.AddElement(pt2)
    SampleNavigate(inputPoints)
    time.sleep(2)


#######################################################
# 根据指定车道 id 和局部坐标获取局部坐标左右两侧车道标线信息
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetRoadMark(pos, laneId):
    print("SampleGetRoadMark:")
    roadMarkInfo = pySimOneSM.getRoadMark(pos, laneId)
    if roadMarkInfo.exists == False:
        print("Not exists!")
        return
    left = roadMarkInfo.left
    print("left roadMark sOffset:", left.sOffset)
    print("left roadMark type:", left.type)
    print("left roadMark color:", left.color)
    print("left roadMark width:", left.width)
    right = roadMarkInfo.right
    print("right roadMark sOffset:", right.sOffset)
    print("right roadMark type:", right.type)
    print("right roadMark color:", right.color)
    print("right roadMark width:", right.width)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetRoadMark(pos, laneId)
            time.sleep(2)


############################################################
# 获取地图中交通灯列表
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ
def SampleGetTrafficLightList():
    print("SampleGetTrafficLightList:")
    trafficLightList = pySimOneSM.getTrafficLightList()
    print("trafficLight count:", trafficLightList.Size())
    for i in range(trafficLightList.Size()):
        light = trafficLightList.GetElement(i)
        print("light id:", light.id)
        print("light type:", light.type.GetString())
        print("light isDynamic:", light.isDynamic)
        print("validity count:", light.validities.Size())
        for j in range(light.validities.Size()):
            validity = light.validities.GetElement(j)
            print("validity roadId:", validity.roadId)
            print("validity fromLaneId:", validity.fromLaneId)
            print("validity toLaneId:", validity.toLaneId)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)

    SampleGetTrafficLightList()
    time.sleep(2)


#######################################################
# 获取地图中交通标志牌列表
import os
import pySimOneSM
import time
from SimOneSMStruct import *
# Global
PosX = 0
PosY = 0
PosZ = 0

def gpsCB(mainVehicleId, data):
 global PosX, PosY, PosZ
 PosX = data[0].posX
 PosY = data[0].posY
 PosZ = data[0].posZ


def SampleGetTrafficSignList():
    print("SampleGetTrafficSignList:")
    trafficSignList = pySimOneSM.getTrafficSignList()
    print("trafficSign count:", trafficSignList.Size())
    for i in range(trafficSignList.Size()):
        sign = trafficSignList.GetElement(i)
        print("sign id:", sign.id)
        print("sign type:", sign.type.GetString())
        print("sign isDynamic:", sign.isDynamic)
        print("validity count:", sign.validities.Size())
        for j in range(sign.validities.Size()):
            validity = sign.validities.GetElement(j)
            print("validity roadId:", validity.roadId)
            print("validity fromLaneId:", validity.fromLaneId)
            print("validity toLaneId:", validity.toLaneId)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)

    SampleGetTrafficSignList()
    time.sleep(2)


###############################################################
# 获取当前车道所属交通灯管辖的停止线列表
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetStoplineList(light, laneId):
    print("SampleGetStoplineList:")
    stoplineList = pySimOneSM.getStoplineList(light, laneId)
    print("stopline count:", stoplineList.Size())
    for i in range(stoplineList.Size()):
        stopline = stoplineList.GetElement(i)
        print("stopline id:", stopline.id)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)
    trafficLightList = pySimOneSM.getTrafficLightList()
    if trafficLightList.Size() < 1:
        print("No traffic light exists!")
        return
    light0 = trafficLightList.GetElement(0)
    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetStoplineList(light0, laneId)
            time.sleep(2)


###################################################
# 获取当前车道所属交通灯管辖的人行横道线列表
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetCrosswalkList(light, laneId):
    print("SampleGetCrosswalkList:")
    crosswalkList = pySimOneSM.getCrosswalkList(light, laneId)
    print("crosswalk count:", crosswalkList.Size())
    for i in range(crosswalkList.Size()):
        crosswalk = crosswalkList.GetElement(i)
        print("crosswalk id:", crosswalk.id)

if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)
    trafficLightList = pySimOneSM.getTrafficLightList()
    if trafficLightList.Size() < 1:
        print("No traffic light exists!")
        return
    light0 = trafficLightList.GetElement(0)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetCrosswalkList(light0, laneId)
            time.sleep(2)


#########################################################
# 获取当前车道所在道路上的网格线列表
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetCrossHatchList(laneId):
    print("SampleGetCrossHatchList:")
    crossHatchList = pySimOneSM.getCrossHatchList(laneId)
    print("crossHatch count:", crossHatchList.Size())
    for i in range(crossHatchList.Size()):
        crossHatch = crossHatchList.GetElement(i)
        print("crossHatch id:", crossHatch.id)

if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetCrossHatchList(laneId)
            time.sleep(2)


#####################################################
# 根据指定车道 id 和局部坐标获取局部坐标到车道中心线上的投影点信息
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0


def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ


def SampleGetNearMostLane(pos):
    print("SampleGetNearMostLane:")
    info = pySimOneSM.getNearMostLane(pos)
    if info.exists == False:
        print("Not exists!")
        return
    print("lane id:", info.laneId.GetString())
    return info.laneId


def SampleGetLaneMiddlePoint(pos, laneId):
    print("SampleGetLaneMiddlePoint:")
    laneMiddlePointInfo = pySimOneSM.getLaneMiddlePoint(pos, laneId)
    if laneMiddlePointInfo.exists == False:
        print("Not exists!")
        return
    print("targetPoint vector: (", laneMiddlePointInfo.targetPoint.x, ",",
          laneMiddlePointInfo.targetPoint.y, ",", laneMiddlePointInfo.targetPoint.z, "),")
    print("dir vector: (", laneMiddlePointInfo.dir.x, ",", laneMiddlePointInfo.dir.y,",",
          laneMiddlePointInfo.dir.z, "),")


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    while (1):
        if PosX != 0:
            print("gpsCB: X:{0} Y:{1} Z:{2}".format(PosX, PosY, PosZ))
            pos = pySimOneSM.pySimPoint3D(PosX, PosY, PosZ)
            laneId = SampleGetNearMostLane(pos)
            SampleGetLaneMiddlePoint(pos, laneId)
            time.sleep(2)


###################################################
# 得到仿真场景中的交通灯的真值
import os
import pySimOneSM
import time
from SimOneSMStruct import *

# Global
PosX = 0
PosY = 0
PosZ = 0

def gpsCB(mainVehicleId, data):
    global PosX, PosY, PosZ
    PosX = data[0].posX
    PosY = data[0].posY
    PosZ = data[0].posZ
def SampleGetTrafficLight():
    print("SampleGetTrafficLightList:")
    trafficLightList = pySimOneSM.getTrafficLightList()
    TrafficLight = SimOne_Data_TrafficLight()
    print("trafficLight count:", trafficLightList.Size())
    for i in range(trafficLightList.Size()):
        light = trafficLightList.GetElement(i)
        print("light id:", light.id)
        if SoGetTrafficLights(0, light.id, TrafficLight):
            print("light status:", TrafficLight.status)


if __name__ == '__main__':
    ret = pySimOneSM.loadHDMap(20)
    print("Load xodr success:", ret)
    SoApiStart()
    SoApiSetGpsUpdateCB(gpsCB)


    SampleGetTrafficLight()
    time.sleep(2)
