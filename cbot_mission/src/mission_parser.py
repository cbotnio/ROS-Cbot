#! /usr/bin/env python

###################################################
# Mission Parser made for NIO
# Author: Mohit Gupta, BITS Goa
###################################################

from __future__ import print_function
import rospy
import sys
from std_msgs.msg import Bool
# from geometry_msgs.msg import Pose
from cbot_ros_msgs.srv import GuidanceInputs, GuidanceInputsResponse, GuidanceInputsRequest
from cbot_ros_msgs.srv import MissInputs, MissInputsResponse, MissInputsRequest
import sys, math, time, json
import MissionCompiler

MissionFile = ""
Mission = {}#MissionCompiler.readMission(MissionFile)
	
currPos = (0,0,0)
goalChanged = False
invalidMission = False
MissionCompleted = False
guidanceStatus = False

guidanceInputs = {"X1": 0.0, "Y1": 0.0, "Z1": 0.0,
				  "X2": 0.0, "Y2": 0.0, "Z2": 0.0,
				  "heading": 0.0, "pitch": 0.0, "speed": 0.0,
				  "Xc": 0.0, "Yc": 0.0, "Zc": 0.0, "radius": 0.0,
				  "arc_follow_direction": 0, "start": "nearest",
				  "runway": 1.0, "type": "vision", 
				  "captureRadius": 1.0, "slipRadius": 2.0,
				  "mode": 0,
				  "timeout": 0.0}

guidanceInputsDefault = {"X1": 0.0, "Y1": 0.0, "Z1": 0.0,
						 "X2": 0.0, "Y2": 0.0, "Z2": 0.0,
						 "heading": 0.0, "pitch": 0.0, "speed": 0.0,
						 "Xc": 0.0, "Yc": 0.0, "Zc": 0.0, "radius": 0.0,
						 "arc_follow_direction": 0, "start": "nearest",
						 "runway": 1.0, "type": "vision", 
						 "captureRadius": 1.0, "slipRadius": 2.0,
						 "mode": 0,
						 "timeout": float("inf")}


params = {"wpt": ["position", "depth", "speed", "heading", "captureRadius", "slipRadius", "timeout"],
		  "lfw": ["position1", "position2", "depth", "speed", "heading", "captureRadius", "timeout"],
		  "arc": ["centerCoord", "depth", "radius","speed", "heading", "captureRadius", "direction", "start", "timeout"],
		  "dock": ["position", 	"depth", "heading", "runwayLength"],
		  "constDepth": ["depth"],
		  "constSpeed": ["speed"],
		  "constHeading": ["heading"],
		  "constPitch": ["pitch"],
		  "loiter": ["timeout"]}

modeTable = {"wpt": 0, "lfw": 1, "arc": 2, "stkp": 3}

stopMissionFlag = 0

missionsCompletedFlag = 0

guidance_srv = rospy.ServiceProxy('/guidance_inputs', GuidanceInputs)

def setSafety(safetyParameters):
	for safetyParam in safetyParameters.keys():
		if(rospy.has_param(safetyParam)):
			rospy.set_param(safetyParam,safetyParameters[safetyParam])

def guidanceStatusCallback(data):
	global guidanceStatus
	guidanceStatus = data.data

def addData(message,MissionType):
	global guidanceInputs
	positionCount = 1
	for key in message.keys():
		try:
			# print(key)
			if("position" in key):
				guidanceInputs["X"+str(positionCount)] = message[key][1:-1].split(',')[0]
				guidanceInputs["Y"+str(positionCount)] = message[key][1:-1].split(',')[1]
				positionCount+=1
				# print("/////////////////")
				# print(message[key][1:-1].split(',')[0])
				# print("/////////////////")
			elif(key == "mode"):
				guidanceInputs[key] = modeTable[MissionType]
			else:
				guidanceInputs[key] = message[key]
		except:
			print("could not set key: ", key)
			guidanceInputs[key] = guidanceInputsDefault[key]

def sendMission():
	global guidanceInputs,timeout
	miss = GuidanceInputsRequest()
	miss.desired_pos_x1 = float(guidanceInputs["X1"])
	miss.desired_pos_y1 = float(guidanceInputs["Y1"])
	miss.desired_pos_x2 = float(guidanceInputs["X2"])
	miss.desired_pos_y2 = float(guidanceInputs["Y2"])
	miss.nominal_velocity = float(guidanceInputs["speed"])
	miss.guidance_mode = float(guidanceInputs["mode"])
	timeout = float(guidanceInputs["timeout"])
	print("Sending mission")
	return guidance_srv(miss)

def checkTimeout(startTime,timeout):
	currTime = time.time()
	timeElapsed = [(float(timeout[j]) - (currTime-x))>0 for j,x in enumerate(startTime)]
	try:
		Index = len(timeElapsed) - timeElapsed.index(0)
	except:
		Index = 0
	return Index

def updateTimeout(startTime, timeout):
	difference = time.time() - startTime[-1]
	for i in range(len(startTime)):
		startTime[i] += difference

def  checkStatus():
	global stopMissionFlag
	# Add all check conditions like pause and perform 
	if(rospy.get_param('Mode').lower()=="auv" and rospy.get_param('Status').lower()=="drive"):
		stopMissionFlag = 0
		# rospy.set_param("/HIL_ON",1)
		return 1
	elif(rospy.get_param('Status').lower()=="park"):
		try:
			updateTimeout(startTime,timeout)
		except:
			pass
		# rospy.set_param("/HIL_ON",0)
		stopMissionFlag = 0
		return 2
	elif(rospy.get_param('Status').lower()=="stop"):
		stopMissionFlag = 1
		# rospy.set_param("/HIL_ON",0)
		return 3


def parseSingleMission(names,timeout,startTime):
	global guidanceInputs, guidanceStatus, stopMissionFlag
	i=0
	bhvType = ""
	flag=0
	while i<len(names):
		if(stopMissionFlag):
			break
		name = names[i]
		print(name)
		if(name in Mission["BHVNameTable"].keys()):
			if(flag==0):
				try:
					timeout.append(Mission["BHVNameTable"][name]["timeout"])
				except:
					timeout.append(float('inf'))
				startTime.append(time.time())
			bhvType = Mission["BHVNameTable"][name]["type"]
			# print(Mission["BHVNameTable"][name]["type"])
			count = checkTimeout(startTime,timeout)
			while(count==0):
				parseSingleMission(Mission["BHVNameTable"][name]["names"],timeout,startTime)
				count = checkTimeout(startTime,timeout)
				flag=1
			timeout.pop()
			startTime.pop()
			

		elif(name in Mission["GuidanceNameTable"].keys()):
			MissionType = Mission["GuidanceNameTable"][name]["type"]
			addData(Mission["GuidanceNameTable"][name]["data"], MissionType)
			response = sendMission()
			print("Next Mission Sent: ", response)
			guidanceStatus = False
			try:
				timeout.append(Mission["GuidanceNameTable"][name]["data"]["timeout"])
			except:
				timeout.append(float('inf'))
			startTime.append(time.time())
			n = checkStatus()
			while(not guidanceStatus and stopMissionFlag==0):
				n = checkStatus()
				if(stopMissionFlag==1):
					break
				elif(n==2):
					while(n==2):
						updateTimeout(startTime,timeout)
						n=checkStatus()
						if(stopMissionFlag==1):
							break
					response = sendMission()
				count = checkTimeout(startTime,timeout)
				if(count==1):
					break
				elif(count>1):
					startTime.pop()
					timeout.pop()
					return
			if(guidanceStatus):
				print("reached")
			else:
				print("Timeout")
			startTime.pop()
			timeout.pop()
		i+=1

def serviceCallback(req):
	global Mission 
	Mission = json.loads(req.Mission)
	print("Mission recieved")
	res = MissInputsResponse()
	res.response = 1
	return res

if __name__=='__main__':
		rospy.init_node('mission_parser')
		guidanceStatusSub = rospy.Subscriber('/guidanceStatus',Bool, guidanceStatusCallback)
		missionServer = rospy.Service('/missionParser', MissInputs, serviceCallback)
		r = rospy.Rate(10)
		print("Waiting for Server")
		rospy.wait_for_service('/guidance_inputs')
		print("Connected to Server")
		while not rospy.is_shutdown():
			if(missionsCompletedFlag):
				rospy.set_param("Status","Stop")
				missionsCompletedFlag = 0

			while(not (rospy.get_param('Mode').lower()=="auv" and rospy.get_param('Status').lower()=="drive")):
				print("Checking from outer loop")
				time.sleep(0.1)

			checkStatus()
			if(stopMissionFlag==0 and bool(Mission)):
				print("In Mission")
				setSafety(Mission["Safety"])

				MissionList = ["M"+str(y) for y in sorted([int(x[1:]) for x in Mission["Missions"].keys()])]
				print(MissionList)

				for Mi in MissionList:
					if(stopMissionFlag):
						break
					missionsCompletedFlag = 1
					timeout = []
					startTime = []
					print(Mi)
					parseSingleMission(Mission["Missions"][Mi]["names"],timeout,startTime)

			r.sleep()
		# rospy.spin()