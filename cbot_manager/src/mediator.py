#!/usr/bin/env python
import rospy
import serial, time, json, os
from cbot_ros_msgs.msg import *
from cbot_ros_msgs.srv import *
from std_srvs.srv import SetBool, SetBoolResponse
import dynamic_reconfigure.client
import MissionCompiler

rospy.init_node("mediator")

ser = serial.Serial("/tmp/GUI-read",timeout=0.01,baudrate=9600, rtscts=True, dsrdtr=True)

# To store and parse message recieved from GCS
message = {}

safetyParams = {"MaxDepth": 10,"MaxPitch":15, "MaxVelocity": 1}

# Status data to be sent
statusData = {"Battery": 100, "Latitude": 15.4507, "Longitude": 73.8041, "Speed": 0.0, "Course": 0.0, "Roll": 0.0, "Pitch":0.0, "Yaw":0.0}

heartbeat_count = 5
last_hearbeat_time = time.time()
heartbeatRate = 1

safety_flag = False

Mission = {}

path = str(os.path.dirname(os.path.abspath(__file__))) #Absolute path of current file. Used for compiling mission file.

def safetyCallback(req):
	global safety_flag
	safety_flag = bool(req.data)
	safety_resp = SetBoolResponse()
	safety_resp.success = True
	safety_resp.message = ""
	return safety_resp

def dynamicCallback(config):
	pass # To be completed. Will be used for getting current system state and updating GCS.

def ahrsCallback(data):
	global statusData
	statusData["Roll"] = round(data.Roll,2)
	statusData["Pitch"] = round(data.Pitch,2)
	statusData["Yaw"] = round(data.YawAngle,2)

def gpsCallback(data):
	global statusData
	statusData["Latitude"] = round(data.latitude,8)
	statusData["Longitude"] = round(data.longitude,8)
	statusData["Course"] = round(data.course,2)
	statusData["Speed"] = round(data.vel,2)

def getData(event):
	global last_hearbeat_time, heartbeat_count, Mission, message, events
	line = ser.readline().decode().strip().split(',')
	if(line[0]=="GUI"):
		if(line[1]=="MISSION"):
			temp = ",".join(line[2:])
			try:
				Mission = json.loads(temp)
			except:
				print("Could not parse Mission File")
		
		elif(line[1]=="EXECUTEMISSION"):
			if(len(line)==2):
				parseMission()
			elif(len(line)==3):
				compileMission(line[2])

		else:
			for data in line[1:]:
				data = data.strip().split(":")
				if(len(data)==2):
					if(data[1]!="null"):
						message[data[0].strip()] = data[1].strip()
			parseData(message)
	
	if(line[0] == "GUIHEARTBEAT"):
		last_hearbeat_time = time.time()
		heartbeat_count = 5

def sendStatusUpdate(event):
	global statusData
	message_send = "$CBOT,"
	for param in statusData:
		message_send += param + ":" + str(statusData[param]) + ","
	message_send = message_send[:-1] + "\r\n"
	ser.flushOutput()
	ser.write(message_send.encode())


def updateSafetyParams():
	for param in safetyParams:
		if(rospy.has_param(param)):
			safetyParams[param] = rospy.get_param(param)
		else:
			print(param + " not set")

def compileMission(mission_filename):
	global Mission
	mission_filename = os.path.join(path,mission_filename)
	if(mission_filename!=""):
		try:
			Mission = MissionCompiler.readMission(mission_filename)
			parseMission()
		except Exception as e:
			print("Error Compiling the Mission.\n" + str(e))

def parseMission():
	global Mission
	rospy.wait_for_service("/missionParser")
	miss_client = rospy.ServiceProxy("/missionParser",MissInputs)
	print("Connected to mission server")
	print("//////////////////Executing mission ////////////////")
	msg = MissInputsRequest()
	msg.Mission = json.dumps(Mission)
	resp = miss_client(msg)

def setROSParam(param,value):
	if(rospy.has_param(param)):
		rospy.set_param(param,value)
	else:
		print("[Central Manager] Could not set param " + str(param) + " to value " + str(value))

def parseData(message):
	global control_client, guidance_client, thrusters_client
	if(safety_flag):
		print("[Central Manager] Setting thrusters off")
		thrusters_client.update_configuration({"thrusters_on":0})

	elif(int(message["AUV_mode"])):
		setROSParam("/mode","AUV")
		if(int(message["MissionInactive"])):
			setROSParam("/status","Stop")
			thrusters_client.update_configuration({"thrusters_on":0})
			guidance_client.update_configuration({"guidance_on":0})
			control_client.update_configuration({"controller_on":0})

		elif(int(message["Park"])):
			setROSParam("/status","Park")
			thrusters_client.update_configuration({"thrusters_on":0})
			guidance_client.update_configuration({"guidance_on":0})
			control_client.update_configuration({"controller_on":0})
				
		elif(int(message["Drive"])):
			setROSParam("/status","Drive")
			thrusters_client.update_configuration({"thrusters_on":1})
			control_client.update_configuration({"controller_on":1})
			guidance_client.update_configuration({"guidance_on":1})

	elif(int(message["Teleop_mode"])):
		setROSParam("/mode","ROV")
		thrusters_client.update_configuration({"thrusters_on":int(message["THRUSTERS_ON"])})
		guidance_client.update_configuration({"guidance_on":int(message["GUIDANCE_ON"])})
		control_client.update_configuration({"controller_on":int(message["CONTROLLER_ON"])})

		if(int(message["GUIDANCE_ON"])):
			pass # To be completed

		elif(int(message["CONTROLLER_ON"])):
			control_client.update_configuration({"heading_ctrl":float(message["HeadingControlON"]), 
												 "speed_ctrl":float(message["SpeedControlON"]), 
												 "pitch_ctrl":float(message["PitchControlON"]), 
												 "depth_ctrl":float(message["DepthControlON"])})
			ctr = ControllerInputsRequest()
			
			if(int(message["HeadingControlON"])):
				ctr.desired_heading = float(message['HCtrl'])
			if(int(message["PitchControlON"])):
				ctr.desired_pitch = float(message['PCtrl'])
			if(int(message["DepthControlON"])):
				ctr.desired_depth = float(message['DCtrl'])
			if(int(message["SpeedControlON"])):
				ctr.desired_u = float(message['SCtrl'])
			controllerResp = controlClient(ctr)

		elif(int(message["THRUSTERS_ON"])):
			if(int(message["Thruster_M1"])):
				thr = ThrusterInputsRequest()
				thr.T1 = float(message["T1"])
				thr.T2 = float(message["T2"])
				thr.T3 = float(message["T3"])
				thr.T4 = float(message["T4"])
				thrusterInputsClient(thr)

			elif(int(message["Thruster_M2"])):
				thr = ThrusterCMDMRequest()
				thr.comm_mode_F = float(message["CMFCtrl"])
				thr.diff_mode_F = float(message["DMFCtrl"])
				thr.comm_mode_V = float(message["CMVCtrl"])
				thr.diff_mode_V = float(message["DMVCtrl"])
				thrusterCMDMClient(thr)

def Timer(event):
	global heartbeat_count, last_hearbeat_time
	time_elapsed = time.time() - last_hearbeat_time
	timeout = 0
	try:
		timeout = float(message["HeartTimeout"])
		if(timeout==0):
			timeout = float("inf")
	except:
		timeout = float("inf")

	if(heartbeat_count>0 and time_elapsed<float(timeout)):
		return
	elif(heartbeat_count>0 and time_elapsed>float(timeout)):
		last_hearbeat_time = time.time()
		heartbeat_count-=1
		return
	else:
		print("[Central Manager] GCS Heartbeat Timeout\n Setting Thrusters OFF")
		thrusters_client.update_configuration({"thrusters_on":0})
		while(heartbeat_count==0):
			time_elapsed = time.time() - last_hearbeat_time
			time.sleep(1)
		print("[Central Manager] GCS Heartbeat Recieved\n You can set Thrusters ON manually")
		return

control_client = dynamic_reconfigure.client.Client("control_node", timeout=5, config_callback=dynamicCallback)
guidance_client = dynamic_reconfigure.client.Client("guidance_node", timeout=5, config_callback=dynamicCallback)
thrusters_client = dynamic_reconfigure.client.Client("thruster_node", timeout=5, config_callback=dynamicCallback)
safety_client = dynamic_reconfigure.client.Client("safety_node", timeout=5, config_callback=dynamicCallback)

controlClient = rospy.ServiceProxy("/controller_inputs", ControllerInputs)
thrusterCMDMClient = rospy.ServiceProxy("/thruster_cmdm", ThrusterCMDM)
thrusterinputsClient = rospy.ServiceProxy("/thruster_inputs", ThrusterInputs)

safetyTrigger = rospy.Service('/safety_trigger', SetBool, safetyCallback)

if __name__ == "__main__":
	rospy.Subscriber("/AHRS",AHRS,ahrsCallback)
	rospy.Subscriber("/GPS",GPS,gpsCallback)	
	rospy.Timer(rospy.Duration(0.1), getData)
	rospy.Timer(rospy.Duration(1), sendStatusUpdate)
	rospy.Timer(rospy.Duration(0.1), Timer)
	rospy.spin()