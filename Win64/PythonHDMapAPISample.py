from SimOneServiceAPI import *
from SimOneSensorAPI import *
import ctypes
import HDMapAPI as HDMapAPI
#from SimOneIOStruct import  *
import time 
# Global
M_PI = 3.14159265358979323846
PosX = 0
PosY = 0
PosZ = 0
def start():
	print("start")

def stop():
	print("stop")

def hdmap_Sample():
	Flag = False
	mainVehicleID = '0'
	success_count = 0
	apiNames=["getNearLanes", "getTrafficSignList", "getLaneLink", "getCrossHatchList", "getTrafficLightList"]

	try:
		if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1")==1:
			print("################## API init success!!!")
			Flag =True
		else:
			print("################## API init fail!!!")
	except Exception as e:
		print(e)

	if Flag:
		if HDMapAPI.loadHDMap(10):
			somapdata = SimOne_Data_Map()
			try:
				print("get hdmap data success")
				for apiName in apiNames:
					if apiName == "getNearLanes":
						print("------------ getNearLanes")
						gpsData = SimOne_Data_Gps()
						if SoGetGps(mainVehicleID, gpsData):
							pt=HDMapAPI.pySimPoint3D(gpsData.posX,gpsData.posY,gpsData.posZ)
							lanesInfo=HDMapAPI.getNearLanes(pt, 3.0)
							if lanesInfo.exists:
								idListSize = lanesInfo.laneIdList.Size()
								if idListSize>0:
									print(">>>>>>>>>>>>>>>>>>>>>  getNearLanes Size = {0}".format(idListSize))
									success_count+=1
								for i in range(idListSize):
									laneId = lanesInfo.laneIdList.GetElement(i)
									print(">>>>>>>>>>>>>>>>>>>>>  getNearLanes  laneId = {0}".format(laneId.GetString()))
					if apiName == "getTrafficSignList":
						print("------------ getTrafficSignList")
						signList=HDMapAPI.getTrafficSignList()
						signSize = signList.Size()
						if signSize!=0:
							print(">>>>>>>>>>>>>>>>>>>>>  getTrafficSignList Size = {0}".format(signSize))
							success_count+=1
					if apiName == "getLaneLink":
						print("------------ getLaneLink")
						gpsData = SimOne_Data_Gps()
						if SoGetGps(mainVehicleID, gpsData):
							pt=HDMapAPI.pySimPoint3D(gpsData.posX,gpsData.posY,gpsData.posX)
							info=HDMapAPI.getNearMostLane(pt)
							laneLinkInfo = HDMapAPI.getLaneLink(info.laneId)
							#HDMapAPI.pyLaneLink
							if laneLinkInfo.exists:
								laneId = laneLinkInfo.laneLink.leftNeighborLaneId.GetString()
								print(">>>>>>>>>>>>>>>>>>>>>  getLaneLink leftNeighborLaneId = {0}".format(laneId))
								success_count+=1
					if apiName == "getCrossHatchList":
						print("------------ getCrossHatchList")
						gpsData = SimOne_Data_Gps()
						if SoGetGps(mainVehicleID, gpsData):
							pt=HDMapAPI.pySimPoint3D(gpsData.posX,gpsData.posY,gpsData.posX)
							info=HDMapAPI.getNearMostLane(pt)
							HatchList = HDMapAPI.getCrossHatchList(info.laneId)
							hatchSize = HatchList.Size()
							if hatchSize>0:
								print(">>>>>>>>>>>>>>>>>>>>>  GetCrossHatchList size = {0}".format(hatchSize))
								success_count+=1
					if apiName == "getTrafficLightList":
						print("------------ getTrafficLightList")
						taffficLightList = HDMapAPI.pySignalSimVector(HDMapAPI.getTrafficLightList())
						taffficLightListSize = taffficLightList.Size()
						if taffficLightListSize>0:
							print(">>>>>>>>>>>>>>>>>>>>>  getTrafficLightList size = {0}".format(taffficLightListSize))
							success_count+=1
						for i in range(taffficLightListSize):
							pySignali = taffficLightList.GetElement(i)
							trafficLigtData = SimOne_Data_TrafficLight()
							print(">>>>>>>>>>>>>>>>>>>>>  SoGetTrafficLights opendriveId = {0}".format(pySignali.id))
							if SoGetTrafficLights(mainVehicleID,pySignali.id,trafficLigtData):
								print(">>>>>>>>>>>>>>>>>>>>>  SoGetTrafficLights opendriveId = {0}, trafficLigtData.countDown = {1}, trafficLigtData.status = {2}".format(pySignali.id,trafficLigtData.countDown,trafficLigtData.status.value))

					time.sleep(0.5)
				if success_count==len(apiNames):
					print("#####################  Test all api success")

			except Exception as e:
				print(e)

	try:
		if SoTerminateSimOneAPI()==1:
			print("################## API terminate success!!!")
		else:
			print("################## API terminate fail!!!")
	except Exception as e:
		print(e)




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

def SampleGetNearLanesWithAngle(pos, radius, headingAngle, shiftAngle):
	print("SampleGetNearLanesWithAngle:")
	nearLanesInfo = HDMapAPI.getNearLanesWithAngle(pos, radius, headingAngle, shiftAngle)
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

def SampleGetDistanceToLaneBoundary(pos):
	print("SampleGetDistanceToLaneBoundary:")
	distanceToLaneBoundaryInfo = HDMapAPI.getDistanceToLaneBoundary(pos)
	if distanceToLaneBoundaryInfo.exists == False:
		print("Not exists!")
		return
	print("laneId:", distanceToLaneBoundaryInfo.laneId.GetString())
	print("distToLeft:", distanceToLaneBoundaryInfo.distToLeft)
	print("distToRight:", distanceToLaneBoundaryInfo.distToRight)
	print("distToLeft2D:", distanceToLaneBoundaryInfo.distToLeft2D)
	print("distToRight2D:", distanceToLaneBoundaryInfo.distToRight2D)

def SampleGetLaneSample(laneId):
	print("SampleGetLaneSample:")
	sampleInfo = HDMapAPI.getLaneSample(laneId)
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

def SampleGetLaneLink(laneId):
	print("SampleGetLaneLink:")
	laneLinkInfo = HDMapAPI.getLaneLink(laneId)
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

def SampleGetLaneType(laneId):
	print("SampleGetLaneType:")
	laneType = HDMapAPI.getLaneType(laneId)
	if laneType.exists == False:
		print("Not exists!")
		return
	print("lane type:", laneType.laneType)

def SampleGetLaneWidth(laneId, pos):
	print("SampleGetLaneWidth:")
	laneWidthInfo = HDMapAPI.getLaneWidth(laneId, pos)
	if laneWidthInfo.exists == False:
		print("Not exists!")
		return
	print("lane width:", laneWidthInfo.width)

def SampleGetLaneST(laneId, pos):
	print("SampleGetLaneST:")
	stInfo = HDMapAPI.getLaneST(laneId, pos)
	if stInfo.exists == False:
		print("Not exists!")
		return
	print("[s,t] relative to this lane:", stInfo.s, ",", stInfo.t)

def SampleGetRoadST(laneId, pos):
	print("SampleGetRoadST:")
	stzInfo = HDMapAPI.getRoadST(laneId, pos)
	if stzInfo.exists == False:
		print("Not exists!")
		return
	print("[s,t] relative to this road:", stzInfo.s, ",", stzInfo.t)
	print("height of input point:", stzInfo.z)

def SampleContainsLane(laneId):
	print("SampleContainsLane:")
	ret = HDMapAPI.containsLane(laneId)
	print("return state:", ret)

def SampleGetParkingSpaceIds():
	print("SampleGetParkingSpaceIds:")
	"""获取地图中停车位的ID列表"""
	parking_space_list = HDMapAPI.getParkingSpaceList()
	print("parking_space_list.Size():%s" % parking_space_list.Size())
	for i in range(parking_space_list.Size()):
		parkingSpace = parking_space_list.GetElement(i)
		front = parkingSpace.front
		knots = parkingSpace.boundaryKnots
		knot0 = knots.GetElement(0)
		print("parkingSpace count:%s" % parking_space_list.Size())
		print("parkingSpace id:%s" % parkingSpace.id)
		#print("roadMark at which side:%s" % front.side.GetString())
		#print("roadMark type:%s" % front.type)
		#print("roadMark color:%s" % front.color)
		#print("roadMark width:%s" % front.width)
		#print("boundaryKnots count:%s" % knots.Size())
		#print("knot0 point: (%s,%s,%s)" % (knot0.x, knot0.y, knot0.z))
		#print("parking_space_list:", parking_space_list)
	return parking_space_list


def SampleGetParkingSpaceKnots(parkingSpaceId):
	print("SampleGetParkingSpaceKnots:")
	parkingSpaceKnotsInfo = HDMapAPI.getParkingSpaceKnots(parkingSpaceId)
	if parkingSpaceKnotsInfo.exists == False:
		print("Not exists!")
		return
	knotList = parkingSpaceKnotsInfo.knots
	print("parkingSpaceKnots size:", knotList.Size())
	for i in range(knotList.Size()):
		element = knotList.GetElement(i)
		print("(", element.x, ",", element.y, ",", element.z, "),")

def gpsCB(mainVehicleId, data):
	global PosX, PosY, PosZ
	PosX = data[0].posX
	PosY = data[0].posY
	PosZ = data[0].posZ
	#print("gpsCB: V:{0} X:{1} Y:{2} Z:{3}".format(mainVehicleId, PosX, PosY, PosZ))

def Samples(pos):
	print("------------------------1:GetNearMostLane-----------------------------------------")
	#1. SampleGetNearMostLane
	laneId = SampleGetNearMostLane(pos)

	print("------------------------2:GetNearLanes--------------------------------------------")
	#2. SampleGetNearLanes
	SampleGetNearLanes(pos, 5)

	#3. SampleGetNearLanesWithAngle
	print("------------------------3:GetNearLanesWithAngle-----------------------------------")
	headingAngle = 30 / 180 * M_PI
	shiftAngle = 90 / 180 * M_PI
	SampleGetNearLanesWithAngle(pos, 5, headingAngle, shiftAngle)

	print("------------------------4:GetDistanceToLaneBoundary------------------------------")
	#4. GetDistanceToLaneBoundary
	SampleGetDistanceToLaneBoundary(pos)

	print("------------------------5:GetLane------------------------------------------------")
	#5. GetLaneSample
	SampleGetLaneSample(laneId)

	print("------------------------6:GetLaneLink--------------------------------------------")
	#6. GetLaneLink
	SampleGetLaneLink(laneId)

	print("-----------------------------7:GetLaneType---------------------------------------")
	#7. GetLaneType
	SampleGetLaneType(laneId)

	print("------------------------8:GetLaneWidth-------------------------------------------")
	#8. GetLaneWidth
	SampleGetLaneWidth(laneId, pos)

	print("-------------------------9:GetLaneST----------------------------------------------")
	#9. GetLaneST
	SampleGetLaneST(laneId, pos)

	print("-------------------------------10:GetRoadST---------------------------------------")
	#10. GetRoadST
	SampleGetRoadST(laneId, pos)

	print("----------------------------11:ContainsLane---------------------------------------")
	#11. ContainsLane
	SampleContainsLane(laneId)

	print("--------------------------12:GetParkingSpaceIds------------------------------------")
	#12. GetParkingSpaceIds
	SampleGetParkingSpaceIds()

def run_case():
	Flag = False
	mainVehicleID = '0'
	try:
		if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1")==1:
			print("################## API init success!!!")
			Flag =True
		else:
			print("################## API init fail!!!")
	except Exception as e:
		print(e)

	ret = HDMapAPI.loadHDMap(10)
	print("Load xodr success:", ret)

	if Flag and ret:
		pos = HDMapAPI.pySimPoint3D(PosX, PosY, PosZ)
		Samples(pos)

if __name__ == '__main__':
	#hdmap_Sample()
	run_case()


