#!/usr/bin/env python
import rospy
import serial, time, json, os
from cbot_ros_msgs.msg import *
from cbot_ros_msgs.srv import *
from std_srvs.srv import SetBool, SetBoolResponse
import dynamic_reconfigure.client
from Compilers import MissionCompiler

rospy.init_node("mediator")

ser = serial.Serial()

# To store and parse message recieved from GCS
message = {}

safety_params = {"MaxDepth": 10,"MaxPitch":15, "MaxVelocity": 1}

# Status data to be sent
sensor_data = {"Battery": 100, "Latitude": 15.4507, "Longitude": 73.8041, "Speed": 0.0, "Depth":0.0, "Course": 0.0, "Roll": 0.0, "Pitch":0.0, "Yaw":0.0}

auv_status = {"guidance_on":0, 
			  "controller_on":0,
			  "heading_ctrl":0,
			  "speed_ctrl":0,
			  "depth_ctrl":0,
			  "pitch_ctrl":0,
			  "thrusters_on":0,
			  "mode": "auv",
			  "status": "stop"}

heartbeat_count = 5
last_hearbeat_time = time.time()
heartbeat_rate = 1

safety_flag = False

Mission = {}

path = str(os.path.dirname(os.path.abspath(__file__))) #Absolute path of current file. Used for compiling mission file.

def initSerialConnection():
	global ser
	ser_connected = 0
	while(not ser_connected):
		if(rospy.has_param("gcs_port")):
			port_name = rospy.get_param("gcs_port")
		if(rospy.has_param("gcs_baudrate")):
			port_baudrate = rospy.get_param("gcs_baudrate")
		try:
			ser = serial.Serial(port=port_name,timeout=0.01,baudrate=port_baudrate, rtscts=True, dsrdtr=True)
			ser_connected = 1
			print("GCS Serial Connected at port: ", port_name)
		except:
			time.sleep(1)

def safetyCallback(req):
	global safety_flag
	safety_flag = bool(req.data)
	safety_resp = SetBoolResponse()
	safety_resp.success = True
	safety_resp.message = ""
	return safety_resp

def ctrlDynamicCallback(config):
	global auv_status
	auv_status["controller_on"] = config.controller_on
	auv_status["heading_ctrl"] = config.heading_ctrl
	auv_status["depth_ctrl"] = config.depth_ctrl
	auv_status["pitch_ctrl"] = config.pitch_ctrl
	auv_status["speed_ctrl"] = config.speed_ctrl

def guidanceDynamicCallback(config):
	global auv_status
	auv_status["guidance_on"] = config.guidance_on

def thrDynamicCallback(config):
	global auv_status
	auv_status["thrusters_on"] = config.thrusters_on
	auv_status["mode"] = config.mode
	auv_status["status"] = config.status

def safetyDynamicCallback(config):
	pass
	
def ahrsCallback(data):
	global sensor_data
	sensor_data["Roll"] = round(data.Roll,2)
	sensor_data["Pitch"] = round(data.Pitch,2)
	sensor_data["Yaw"] = round(data.YawAngle,2)

def gpsCallback(data):
	global sensor_data
	sensor_data["Latitude"] = round(data.latitude,8)
	sensor_data["Longitude"] = round(data.longitude,8)
	sensor_data["Course"] = round(data.course,2)
	sensor_data["Speed"] = round(data.vel,2)

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
	global sensor_data
	message_send = "$CBOT,"
	for param in sensor_data:
		message_send += param + ":" + str(sensor_data[param]) + ","
	message_send = message_send[:-1] + "\r\n"
	ser.flushOutput()
	ser.write(message_send.encode())


def updateSafetyParams():
	# To de changed to dynamic params
	for param in safety_params:
		if(rospy.has_param(param)):
			safety_params[param] = rospy.get_param(param)
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
	miss_client = rospy.ServiceProxy("/missionParser",String)
	print("Connected to mission server")
	print("//////////////////Executing mission ////////////////")
	msg = StringRequest()
	msg.data = json.dumps(Mission)
	resp = miss_client(msg)

def setDynParams(data):
	try:
		data = data.data
	except:
		data = data
	data = json.loads(data)
	resp = StringResponse()

	if(not safety_flag):
		if("guidance" in data.keys()):
			guidance_client.update_configuration(data["guidance"])
		if("controller" in data.keys()):
			control_client.update_configuration(data["controller"])
		if("actuator" in data.keys()):
			thrusters_client.update_configuration(data["actuator"])

		resp.response = 1
		return resp
	else:
		resp.response = 0
		return resp

def parseData(message):
	global control_client, guidance_client, thrusters_client
	if(safety_flag):
		print("[Central Manager] Safety Flag ON.\nSetting thrusters off")
		thrusters_client.update_configuration({"thrusters_on":0})

	elif(int(message["AUV_mode"])):
		if(int(message["MissionInactive"])):
			setDynParams(json.dumps({"actuator":{"mode":"AUV","status":"Stop","thrusters_on":0},
									 "controller":{"controller_on":0},
									 "guidance":{"guidance_on":0}}))

		elif(int(message["Park"])):
			setDynParams(json.dumps({"actuator":{"mode":"AUV","status":"Park","thrusters_on":0},
									 "controller":{"controller_on":0},
									 "guidance":{"guidance_on":0}}))
				
		elif(int(message["Drive"])):
			setDynParams(json.dumps({"actuator":{"mode":"AUV","status":"Drive"}}))

	elif(int(message["Teleop_mode"])):
		setDynParams(json.dumps({"actuator":{"mode":"HROV","thrusters_on":int(message["THRUSTERS_ON"])},
								 "controller":{"controller_on":int(message["CONTROLLER_ON"])},
								 "guidance":{"guidance_on":int(message["GUIDANCE_ON"])}}))

		if(int(message["GUIDANCE_ON"])):
			pass # To be completed

		elif(int(message["CONTROLLER_ON"])):
			control_client.update_configuration({"heading_ctrl":float(message["HeadingControlON"]), 
												 "speed_ctrl":float(message["SpeedControlON"]), 
												 "pitch_ctrl":float(message["PitchControlON"]), 
												 "depth_ctrl":float(message["DepthControlON"])})
			ctr = ControllerInputs()
			if(int(message["HeadingControlON"])):
				ctr.desired_heading = float(message['HCtrl'])
			if(int(message["PitchControlON"])):
				ctr.desired_pitch = float(message['PCtrl'])
			if(int(message["DepthControlON"])):
				ctr.desired_depth = float(message['DCtrl'])
			if(int(message["SpeedControlON"])):
				ctr.desired_u = float(message['SCtrl'])
			control_pub.publish(ctr)

		elif(int(message["THRUSTERS_ON"])):
			if(int(message["Thruster_M1"])):
				thr = ThrusterInputs()
				thr.T1 = float(message["T1"])
				thr.T2 = float(message["T2"])
				thr.T3 = float(message["T3"])
				thr.T4 = float(message["T4"])
				thrusterInputs_pub.publish(thr)

			elif(int(message["Thruster_M2"])):
				thr = ThrusterCMDM()
				thr.comm_mode_F = float(message["CMFCtrl"])
				thr.diff_mode_F = float(message["DMFCtrl"])
				thr.comm_mode_V = float(message["CMVCtrl"])
				thr.diff_mode_V = float(message["DMVCtrl"])
				thrusterCMDM_pub.publish(thr)

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


# Initialize dynamic reconfigure clients
control_client = dynamic_reconfigure.client.Client("control_node", timeout=5, config_callback=ctrlDynamicCallback)
guidance_client = dynamic_reconfigure.client.Client("guidance_node", timeout=5, config_callback=guidanceDynamicCallback)
thrusters_client = dynamic_reconfigure.client.Client("thruster_node", timeout=5, config_callback=thrDynamicCallback)
safety_client = dynamic_reconfigure.client.Client("safety_node", timeout=5, config_callback=safetyDynamicCallback)

# Initialize publishers
control_pub = rospy.Publisher("/controller_inputs", ControllerInputs, queue_size=1)
thrusterCMDM_pub = rospy.Publisher("/thruster_cmdm", ThrusterCMDM, queue_size=1)
thrusterInputs_pub = rospy.Publisher("/thruster_inputs", ThrusterInputs, queue_size=1)

# Initialize service servers
safetyTrigger_src = rospy.Service('/safety_trigger', SetBool, safetyCallback)
setDynParams_srv = rospy.Service('/set_dyn_params', String, setDynParams)

if __name__ == "__main__":
	initSerialConnection()
	rospy.Subscriber("/AHRS",AHRS,ahrsCallback)
	rospy.Subscriber("/GPS",GPS,gpsCallback)	
	rospy.Timer(rospy.Duration(0.1), getData)
	rospy.Timer(rospy.Duration(1), sendStatusUpdate)
	rospy.Timer(rospy.Duration(0.1), Timer)
	rospy.spin()