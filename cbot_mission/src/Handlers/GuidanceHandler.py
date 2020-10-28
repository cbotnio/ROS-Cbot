#! /usr/bin/env python
from cbot_ros_msgs.srv import *

class GuidanceHandler():
	def __init__(self):
		self.modeTable = {"wpt": 0, "lfw": 1, "arc": 2, "stkp": 3, "wptFA": 4, "lfwFA": 5, "arcFA": 6, "stkp": 7}
		self.dyn_params = {"actuator":{"thrusters_on":1},
						   "controller":{"controller_on":1,
										 "heading_ctrl":0,
										 "speed_ctrl":0,
										 "depth_ctrl":0,
										 "pitch_ctrl":0,
										 "roll_ctrl":0},
							"guidance":{"guidance_on":1}}

	def parseGuidanceMission(self,data,missionType):
		if(missionType=="wpt" or missionType=="wptFA"):
			return self.waypointGuidance(data,missionType)
		elif(missionType=="stkp" or missionType=="stkpFA"):
			return self.waypointGuidance(data,missionType)
		elif(missionType=="lfw" or missionType=="lfwFA"):
			return self.lineGuidance(data,missionType)
		elif(missionType=="arc" or missionType=="arcFA"):
			return self.arcGuidance(data,missionType)

	def waypointGuidance(self,data,missionType):
		inputs = WaypointInputsRequest()
		inputs.goal.x = float(data["position1"][1:-1].split(',')[0])
		inputs.goal.y = float(data["position1"][1:-1].split(',')[1])

		if(missionType=="wpt"):
			self.dyn_params["controller"]["heading_ctrl"] = 1
			self.dyn_params["controller"]["speed_ctrl"] = 1
			inputs.speed = float(data["speed"])
		elif(missionType=="wptFA"):
			self.dyn_params["controller"]["speed_ctrl"] = 1
			inputs.speed = float(data["speed"])

		if("roll" in data.keys()):
			inputs.goal.roll = data["roll"]
			self.dyn_params["controller"]["roll_ctrl"] = 1
		else:
			self.dyn_params["controller"]["roll_ctrl"] = 0
		
		if("pitch" in data.keys()):
			inputs.goal.pitch = data["pitch"]
			self.dyn_params["controller"]["pitch_ctrl"] = 1
		else:
			self.dyn_params["controller"]["pitch_ctrl"] = 0

		if("heading" in data.keys()):
			inputs.goal.yaw = data["heading"]
			self.dyn_params["controller"]["heading_ctrl"] = 1
		elif(missionType=="wptFA"):
			self.dyn_params["controller"]["heading_ctrl"] = 0

		if("depth" in data.keys()):
			inputs.goal.z = data["depth"]
			self.dyn_params["controller"]["depth_ctrl"] = 1
		else:
			self.dyn_params["controller"]["depth_ctrl"] = 0
		
		if("captureRadius" in data.keys()):
			inputs.captureRadius = data["captureRadius"]
		else:
			inputs.captureRadius = 2.0
		
		if("slipRadius" in data.keys()):
			inputs.slipRadius = data["slipRadius"]
		else:
			inputs.slipRadius = 4.0
		inputs.mode = self.modeTable[missionType]
		
		return inputs,self.dyn_params

	def lineGuidance(self,data,missionType):
		inputs = LineInputsRequest()
		inputs.start.x = float(data["position1"][1:-1].split(',')[0])
		inputs.start.y = float(data["position1"][1:-1].split(',')[1])
		inputs.end.x = float(data["position2"][1:-1].split(',')[0])
		inputs.end.y = float(data["position2"][1:-1].split(',')[1])
		if(missionType=="lfw"):
			self.dyn_params["controller"]["heading_ctrl"] = 1
			self.dyn_params["controller"]["speed_ctrl"] = 1
			inputs.speed = float(data["speed"])
		elif(missionType=="lfwFA"):
			self.dyn_params["controller"]["speed_ctrl"] = 1
			inputs.speed = float(data["speed"])

		if("roll" in data.keys()):
			inputs.start.roll = data["roll"]
			inputs.end.roll = data["roll"]
			self.dyn_params["controller"]["roll_ctrl"] = 1
		else:
			self.dyn_params["controller"]["roll_ctrl"] = 0

		if("pitch" in data.keys()):
			inputs.start.pitch = data["pitch"]
			inputs.end.pitch = data["pitch"]
			self.dyn_params["controller"]["pitch_ctrl"] = 1
		else:
			self.dyn_params["controller"]["pitch_ctrl"] = 0

		if("heading" in data.keys()):
			inputs.start.yaw = data["heading"]
			inputs.end.yaw = data["heading"]
			self.dyn_params["controller"]["heading_ctrl"] = 1
		elif(missionType=="lfwFA"):
			self.dyn_params["controller"]["heading_ctrl"] = 0
		
		if("depth" in data.keys()):
			inputs.start.z = data["depth"]
			inputs.end.z = data["depth"]
			self.dyn_params["controller"]["depth_ctrl"] = 1
		else:
			self.dyn_params["controller"]["depth_ctrl"] = 0

		if("captureRadius" in data.keys()):
			inputs.captureRadius = data["captureRadius"]
		else:
			inputs.captureRadius = 2.0

		inputs.mode = self.modeTable[missionType]
		
		return inputs,self.dyn_params

	def arcGuidance(self,data,missionType):
		inputs = ArcInputsRequest()
		inputs.start.x = float(data["position1"][1:-1].split(',')[0])
		inputs.start.y = float(data["position1"][1:-1].split(',')[1])
		inputs.end.x = float(data["position2"][1:-1].split(',')[0])
		inputs.end.y = float(data["position2"][1:-1].split(',')[1])
		inputs.center.x = float(data["center"][1:-1].split(',')[0])
		inputs.center.y = float(data["center"][1:-1].split(',')[1])
		if(missionType=="arc"):
			self.dyn_params["controller"]["heading_ctrl"] = 1
			self.dyn_params["controller"]["speed_ctrl"] = 1
			inputs.speed = float(data["speed"])
		elif(missionType=="arcFA"):
			self.dyn_params["controller"]["speed_ctrl"] = 1
			inputs.speed = float(data["speed"])

		if("roll" in data.keys()):
			inputs.start.roll = data["roll"]
			inputs.end.roll = data["roll"]
			self.dyn_params["controller"]["roll_ctrl"] = 1
		else:
			self.dyn_params["controller"]["roll_ctrl"] = 0

		if("pitch" in data.keys()):
			inputs.start.pitch = data["pitch"]
			inputs.end.pitch = data["pitch"]
			self.dyn_params["controller"]["pitch_ctrl"] = 1
		else:
			self.dyn_params["controller"]["pitch_ctrl"] = 0

		if("heading" in data.keys()):
			inputs.start.yaw = data["heading"]
			inputs.end.yaw = data["heading"]
			self.dyn_params["controller"]["heading_ctrl"]
		elif(missionType=="arcFA"):
			self.dyn_params["controller"]["heading_ctrl"] = 0

		if("depth" in data.keys()):
			inputs.start.z = data["depth"]
			inputs.end.z = data["depth"]
			self.dyn_params["controller"]["depth_ctrl"] = 1
		else:
			self.dyn_params["controller"]["depth_ctrl"] = 0

		if("captureRadius" in data.keys()):
			inputs.captureRadius = data["captureRadius"]
		else:
			inputs.captureRadius = 2.0
			
		inputs.mode = self.modeTable[missionType]
		
		return inputs,self.dyn_params