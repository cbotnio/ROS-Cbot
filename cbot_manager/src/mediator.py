#!/usr/bin/env python
import rospy
import serial, time, json
from cbot_ros_msgs.msg import *
from cbot_ros_msgs.srv import *
import dynamic_reconfigure.client

rospy.init_node("mediator")

ser = serial.Serial("/tmp/GUI-read",timeout=0.01,baudrate=9600, rtscts=True, dsrdtr=True)
controlClient = rospy.ServiceProxy("/controller_inputs", ControllerInputs)
thrusterClient = rospy.ServiceProxy("/thruster_control", ThrusterControl)
thr_pub = rospy.Publisher("/Thrusters",ThrusterData,queue_size=1)

message = {}

safetyParams = {"MaxDepth": 10,"MaxPitch":15, "MaxVelocity": 1}

sendData = {"Battery": 100, "Latitude": 15.4507, "Longitude": 73.8041, "Speed": 0.0, "Course": 0.0, "Roll": 0.0, "Pitch":0.0, "Yaw":0.0}

heartbeat_count = 5
last_hearbeat_time = time.time()
heartbeatRate = 1

Mission = {}

def dynamicCallback(config):
	print("Updated dynamic params")


def ahrsCallback(data):
	global sendData
	sendData["Roll"] = round(data.Roll,2)
	sendData["Pitch"] = round(data.Pitch,2)
	sendData["Yaw"] = round(data.YawAngle,2)

def gpsCallback(data):
	global sendData
	sendData["Latitude"] = round(data.latitude,5)
	sendData["Longitude"] = round(data.longitude,5)
	sendData["Course"] = round(data.course,2)
	sendData["Speed"] = round(data.vel,2)

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
			parseMission()

		else:
			for data in line[1:]:
				data = data.strip().split(":")
				if(len(data)==2):
					if(data[1]!="null"):
						message[data[0].strip()] = data[1].strip()
			print(message)
			parseData(message)
	
	if(line[0] == "GUIHEARTBEAT"):
		last_hearbeat_time = time.time()
		heartbeat_count = 5

def sendStatusUpdate(event):
	global sendData
	message_send = "MED,"
	for param in sendData:
		message_send += param + ":" + str(sendData[param]) + ","
	message_send += "\r\n"
	ser.flushOutput()
	ser.write(message_send.encode())

def updateSafetyParams():
	for param in safetyParams:
		if(rospy.has_param(param)):
			safetyParams[param] = rospy.get_param(param)
		else:
			print(param + " not set")

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
		print("[MEDIATOR] Could not set param " + str(param) + " to value " + str(value))

def parseData(message):
	global control_client
	print("In Parse data")
	print("Teleop_mode: " + message["Teleop_mode"])
	if(int(message["AUV_mode"])):
		setROSParam("/mode","AUV")
		if(int(message["MissionInactive"])):
			setROSParam("/status","Stop")
			setROSParam("/thrusters_on",0)
			guidance_client.update_configuration({"guidance_on":0})
			control_client.update_configuration({"controller_on":0})

		elif(int(message["Park"])):
			setROSParam("/status","Park")
			setROSParam("/thrusters_on",0)
			guidance_client.update_configuration({"guidance_on":0})
			control_client.update_configuration({"controller_on":0})
				
		elif(int(message["Drive"])):
			setROSParam("/status","Drive")
			setROSParam("/thrusters_on",1)
			control_client.update_configuration({"controller_on":1})
			guidance_client.update_configuration({"guidance_on":1})

	elif(int(message["Teleop_mode"])):
		print("In Teleop_mode Mode")
		setROSParam("/mode","ROV")
		guidance_client.update_configuration({"guidance_on":int(message["GUIDANCE_ON"])})
		setROSParam("/thrusters_on",int(message["THRUSTERS_ON"]))
		control_client.update_configuration({"controller_on":int(message["CONTROLLER_ON"])})
		print("Updated controller")
		if(int(message["GUIDANCE_ON"])):
			pass

		elif(int(message["CONTROLLER_ON"])):
			control_client.update_configuration({"heading_ctrl":float(message["HeadingControlON"]), 
												 "speed_ctrl":float(message["SpeedControlON"]), 
												 "pitch_ctrl":float(message["PitchControlON"]), 
												 "depth_ctrl":float(message["DepthControlON"])})
			ctr = ControllerInputsRequest()
			if(int(message["HeadingControlON"])):
				print(float(message['HCtrl']))
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
				thr = ThrusterData()
				thr.T1 = float(message["T1"])
				thr.T2 = float(message["T2"])
				thr.T3 = float(message["T3"])
				thr.T4 = float(message["T4"])
				thr_pub.publish(thr)
			elif(int(message["Thruster_M2"])):
				thr = ThrusterControlRequest()
				thr.comm_mode_F = float(message["CMFCtrl"])
				thr.diff_mode_F = float(message["DMFCtrl"])
				thr.comm_mode_V = float(message["CMVCtrl"])
				thr.diff_mode_V = float(message["DMVCtrl"])
				thr.update = 1
				thrusterClient(thr)

def Timer(event):
	global heartbeat_count, last_hearbeat_time
	time_elapsed = time.time() - last_hearbeat_time
	timeout = 0
	try:
		timeout = float(message["HeartTimeout"])
	except:
		timeout = float("inf")
	if(heartbeat_count>0 and time_elapsed<float(timeout)):
		return
	elif(heartbeat_count>0 and time_elapsed>float(timeout)):
		last_hearbeat_time = time.time()
		heartbeat_count-=1
		return
	else:
		print("GUI Heartbeat Timeout\n Setting Thrusters OFF")
		rospy.set_param("/thrusters_on",0)
		while(heartbeat_count==0):
			time_elapsed = time.time() - last_hearbeat_time
			time.sleep(0.1)
		print("GUI Heartbeat Recieved\n You can set Thrusters ON manually")
		return

control_client = dynamic_reconfigure.client.Client("control_node", timeout=0, config_callback=dynamicCallback)
guidance_client = dynamic_reconfigure.client.Client("guidance_node", timeout=0, config_callback=dynamicCallback)

if __name__ == "__main__":
	rospy.Subscriber("/AHRS",AHRS,ahrsCallback)
	rospy.Subscriber("/GPS",GPS,gpsCallback)	
	rospy.Timer(rospy.Duration(0.1), getData)
	rospy.Timer(rospy.Duration(1), sendStatusUpdate)
	rospy.Timer(rospy.Duration(0.1), Timer)
	rospy.spin()