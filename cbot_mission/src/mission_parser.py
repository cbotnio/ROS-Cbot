#! /usr/bin/env python

###################################################
# Mission Parser made for NIO
# Author: Mohit Gupta, BITS Goa
###################################################

from __future__ import print_function
import rospy
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from cbot_ros_msgs.srv import GuidanceInputs, GuidanceInputsResponse, GuidanceInputsRequest
import sys, math, time
import MissionCompiler

MissionFile = "Mission.txt"
Mission = MissionCompiler.readMission(MissionFile)
	
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

guidance_srv = rospy.ServiceProxy('/guidance_inputs', GuidanceInputs)

commonParams = ["depth", "speed", "heading"]

params = {"wpt": ["position", "depth", "speed", "heading", "captureRadius", "slipRadius", "timeout"],
		  "lfw": ["position1", "position2", "depth", "speed", "heading", "captureRadius", "timeout"],
		  "arc": ["centerCoord", "depth", "radius","speed", "heading", "captureRadius", "direction", "start", "timeout"],
		  "dock": ["position", 	"depth", "heading", "runwayLength"],
		  "constDepth": ["depth"],
		  "constSpeed": ["speed"],
		  "constHeading": ["heading"],
		  "constPitch": ["pitch"],
		  "loiter": ["timeout"]}

impParams = {"wpt": ["position","speed"],
			 "lfw": ["position1", "position2", "speed"],
			 "arc": ["centerCoord", "radius", "speed", "start"],
			 "dock": ["position", 	"depth", "heading", "runwayLength"]}

def setSafety(safetyParameters):
	for safetyParam in safetyParameters.keys():
		if(rospy.has_param(safetyParam)):
			rospy.set_param(safetyParam,safetyParameters[safetyParam])

def guidanceStatusCallback(data):
	global guidanceStatus
	guidanceStatus = data.data

def wpt(data):
	global goalChanged, guidanceInputs
	guidanceInputs["X1"] = data["position"][1:-1].split(',')[0]
	guidanceInputs["Y1"] = data["position"][1:-1].split(',')[1]
	guidanceInputs["captureRadius"] = data["captureRadius"]
	guidanceInputs["slipRadius"] = data["slipRadius"]
	guidanceInputs["mode"] = 0
	try:
		guidanceInputs["timeout"] = data["timeout"]
	except:
		guidanceInputs["timeout"] = float('inf')
	addCommonData(data)

def lfw(data):
	global goalChanged, guidanceInputs
	guidanceInputs["X1"] = data["position1"][1:-1].split(',')[0]
	guidanceInputs["Y1"] = data["position1"][1:-1].split(',')[1]
	guidanceInputs["X2"] = data["position2"][1:-1].split(',')[0]
	guidanceInputs["Y2"] = data["position2"][1:-1].split(',')[1]
	guidanceInputs["slipRadius"] = data["slipRadius"]
	guidanceInputs["mode"] = 1
	try:
		guidanceInputs["timeout"] = data["timeout"]
	except:
		guidanceInputs["timeout"] = float('inf')
	addCommonData(data)

def stkp(data):
	global goalChanged, guidanceInputs
	guidanceInputs["X1"] = data["position1"][1:-1].split(',')[0]
	guidanceInputs["Y1"] = data["position1"][1:-1].split(',')[1]
	guidanceInputs["mode"] = 3
	try:
		guidanceInputs["timeout"] = data["timeout"]
	except:
		guidanceInputs["timeout"] = float('inf')
	addCommonData(data)

def addCommonData(data):
	global guidanceInputs, commonParams
	for key in commonParams:
		if key in data.keys():
			guidanceInputs[key] = data[key]

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
	startTime[-1] = time.time() 
	# To be completed
	for i in range(len(startTime)-1):
		# timeout[i] = 
		startTime[i] += time.time()

def  checkStatus():
	# Add all check conditions like pause and perform 
	if(rospy.get_param('Mode').lower()=="auv" and rospy.get_param('Status').lower()=="drive"):
		return True
	elif(rospy.get_param('Mode').lower()=="auv" and rospy.get_param('Status').lower()=="pause"):
		return False
	else:
		return False


def parseSingleMission(names,timeout,startTime):
	global guidanceInputs, guidanceStatus
	i=0
	bhvType = ""
	flag=0
	while i<len(names):
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
			print(Mission["BHVNameTable"][name]["type"])
			count = checkTimeout(startTime,timeout)
			while(count==0):
				parseSingleMission(Mission["BHVNameTable"][name]["names"],timeout,startTime)
				count = checkTimeout(startTime,timeout)
				flag=1
			timeout.pop()
			startTime.pop()
			

		elif(name in Mission["GuidanceNameTable"].keys()):
			MissionType = Mission["GuidanceNameTable"][name]["type"]
			globals()[MissionType](Mission["GuidanceNameTable"][name]["data"])
			response = sendMission()
			print("Next Mission Sent")
			guidanceStatus = False
			try:
				timeout.append(Mission["GuidanceNameTable"][name]["data"]["timeout"])
			except:
				timeout.append(float('inf'))
			startTime.append(time.time())
			while(not guidanceStatus):
				# while(not checkStatus()):
				# 	updateTimeout(startTime,timeout)
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


if __name__=='__main__':
		rospy.init_node('mission_parser',disable_signals=True)
		guidanceStatusSub = rospy.Subscriber('/guidanceStatus',Bool, guidanceStatusCallback)
		r = rospy.Rate(10)
		print("Waiting for Server")
		rospy.wait_for_service('/guidance_inputs')
		print("Connected to Server")

		
		if(rospy.get_param('Mode').lower()=="auv" and rospy.get_param('Status').lower()=="drive"):
			setSafety(Mission["Safety"])

			MissionList = ["M"+str(y) for y in sorted([int(x[1:]) for x in Mission["Missions"].keys()])]
			print(MissionList)

			for Mi in MissionList:
				timeout = []
				startTime = []
				print(Mi)
				parseSingleMission(Mission["Missions"][Mi]["names"],timeout,startTime)