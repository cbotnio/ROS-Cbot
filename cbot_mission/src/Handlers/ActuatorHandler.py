#! /usr/bin/env python
from cbot_ros_msgs.msg import ThrusterCMDM

class ActuatorHandler():

	def __init__(self):
		self.dyn_params = {"actuator":{"thrusters_on":1},
					       "controller":{"controller_on":0},
					       "guidance" : {"guidance_on": 0}}

	def parseActuatorMission(self,data,missionType):
		inputs = ThrusterCMDM	()
		for param in data.keys():
			if(param == "cmf"):
				inputs.comm_mode_f = data[param]
			if(param == "dmf"):
				inputs.diff_mode_f = data[param]
			if(param == "cmv"):
				inputs.comm_mode_v = data[param]
			if(param == "dmv"):
				inputs.diff_mode_f = data[param]
			return inputs, self.dyn_params