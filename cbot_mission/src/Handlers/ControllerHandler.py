#! /usr/bin/env python
from cbot_ros_msgs.msg import ControllerInputs

class ControllerHandler():

	def __init__(self):
		self.dyn_params = {"actuator":{"thrusters_on":1},
						   "controller":{"controller_on":1,
										  "heading_ctrl":0, 
										  "speed_ctrl":0,
										  "depth_ctrl":0,
										  "pitch_ctrl":0,
										  "roll_ctrl": 0},
							"guidance":{"guidance_on":0}}

	def parseControlMission(self,data,missionType):
		inputs = ControllerInputs()
		for param in data.keys():
			if(param == "heading"):
				inputs.desired_heading = float(data[param])
				self.dyn_params["controller"]["heading_ctrl"] = 1
			else:
				self.dyn_params["controller"]["heading_ctrl"] = 0

			if(param == "depth"):
				inputs.desired_depth = float(data[param])
				self.dyn_params["controller"]["depth_ctrl"] = 1
			else:
				self.dyn_params["controller"]["depth_ctrl"] = 0
			
			if(param == "speed"):
				inputs.desired_speed = float(data[param])
				self.dyn_params["controller"]["speed_ctrl"] = 1
			else:
				self.dyn_params["controller"]["speed_ctrl"] = 0
			
			if(param == "pitch"):
				inputs.desired_pitch = float(data[param])
				self.dyn_params["controller"]["pitch_ctrl"] = 1
			else:
				self.dyn_params["controller"]["pitch_ctrl"] = 0

			return inputs, self.dyn_params