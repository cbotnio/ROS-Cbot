import rospy
import serial, time, json
from cbot_ros_msgs.msg import *
from cbot_ros_msgs.srv import *

rospy.init_node("mediator")

ser = serial.Serial("/tmp/GUI-read",timeout=0.01,baudrate=9600, rtscts=True, dsrdtr=True)
controlClient = rospy.ServiceProxy("/controller_inputs", ControllerInputs)
thrusterClient = rospy.ServiceProxy("/thruster_control", ThrusterControl)
thrPub = rospy.Publisher("/Thrusters",ThrusterData,queue_size=10)

message = {}

safetyParams = {"MaxDepth": 10,"MaxPitch":15, "MaxVelocity": 1}

sendData = {"Battery": 100, "Latitude": 15.4507, "Longitude": 73.8041, "Speed": 0.0, "Course": 0.0, "Roll": 0.0, "Pitch":0.0, "Yaw":0.0}

heartbeatCount = 5
lastHearbeatTime = time.time()

Mission = {}

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
	global lastHearbeatTime, heartbeatCount, Mission, message, events
	line = ser.readline().decode().strip().split(',')
	# print(line)
	if(line[0]=="GUI"):
		if(line[1]=="MISSION"):
			temp = ",".join(line[2:])
			try:
				Mission = json.loads(temp)
			except:
				print("Could not parse Mission File")
			# print(Mission)
		
		elif(line[1]=="EXECUTEMISSION"):
			parseMission()

		else:
			for data in line[1:]:
				data = data.strip().split(":")
				if(len(data)==2):
					if(data[1]!="null"):
						message[data[0].strip()] = float(data[1].strip())

			# print(message)
			parseData()
	if(line[0] == "GUIHEARTBEAT"):
		lastHearbeatTime = time.time()
		heartbeatCount = 5

def sendStatusUpdate(event):
	global sendData
	message = "MED,"
	for param in sendData:
		message += param + ":" + str(sendData[param]) + ","
	message += "\r\n"
	ser.flushOutput()
	ser.write(message.encode())

def updateSafetyParams():
	for param in safetyParams:
		if(rospy.has_param(param)):
			safetyParams[param] = rospy.get_param(param)
		else:
			print(param + " not set")

def parseMission():
	global Mission
	
	print("Connected to mission server")
	print("//////////////////////// Executing mission ////////////////")
	msg = MissInputsRequest()
	msg.Mission = json.dumps(Mission)
	# print(msg.Mission)
	resp = missClient(msg)


def parseData():
	if(rospy.has_param("Controller_ON")):
		rospy.set_param("Controller_ON",message["CONTROLLER_ON"])
	else:
		print("Controller not set")
	if(rospy.has_param("GUIDANCE_ON")):
		rospy.set_param("GUIDANCE_ON",message["GUIDANCE_ON"])
	else:
		print("GUIDANCE not set")
	if(rospy.has_param("HIL_ON")):
		rospy.set_param("HIL_ON",message["THRUSTERS_ON"])
	else:
		print("THRUSTERS not set")

	if(message["AUV_mode"]):
		if(rospy.has_param("/Mode")):
			rospy.set_param("/Mode","AUV")
		else:
			print("MODE not set to AUV")

		elif(message["MissionInactive"]):
			if(rospy.has_param("/Status")):
				rospy.set_param("/Status","Stop")
			else:
				print("Status not set to Stop")
			if(rospy.has_param("/HIL_ON")):
				rospy.set_param("/HIL_ON",0)
			else:
				print("Thrusters not set to 0")

		elif(message["Park"]):
			if(rospy.has_param("/Status")):
				rospy.set_param("/Status","Park")
			else:
				print("Status not set to Park")
			if(rospy.has_param("/HIL_ON")):
				rospy.set_param("/HIL_ON",0)
			else:
				print("Thrusters not set to 0")
				
		elif(message["Drive"]):
			if(rospy.has_param("/Status")):
				rospy.set_param("/Status","Drive")
			else:
				print("Status not set to Drive")

	elif(message["Teleop_mode"]):
		if(rospy.has_param("Mode")):
			rospy.set_param("Mode","ROV")
		else:
			print("MODE not set to ROV")
		if(rospy.has_param("Status")):
			rospy.set_param("Status","Stop")
		else:
			print("Status not set to Stop")

		if(message["GUIDANCE_ON"]):
			pass
		elif(message["CONTROLLER_ON"]):
			ctr = ControllerInputsRequest()
			if(rospy.has_param("HeadingCtrl")):
				rospy.set_param("HeadingCtrl",message["HeadingControlON"])
			else:
				print("Heading Control not set")

			if(rospy.has_param("VelocityCtrl")):
				rospy.set_param("VelocityCtrl",message["SpeedControlON"])
			else:
				print("Velocity Control not set")

			if(rospy.has_param("PitchCtrl")):
				rospy.set_param("PitchCtrl",message["PitchControlON"])
			else:
				print("Pitch Control not set")

			if(rospy.has_param("DepthCtrl")):
				rospy.set_param("DepthCtrl",message["DepthControlON"])
			else:
				print("Depth Control not set")		

			if(message["HeadingControlON"]):
				ctr.desired_heading = float(message['HCtrl'])
				print(message['HCtrl'])
			if(message["PitchControlON"]):
				ctr.desired_pitch = float(message['PCtrl'])
			if(message["DepthControlON"]):
				ctr.desired_depth = float(message['DCtrl'])
			if(message["SpeedControlON"]):
				ctr.desired_u = float(message['SCtrl'])
			controllerResp = controlClient(ctr)

		elif(message["THRUSTERS_ON"]):
			if(message["Thruster_M1"]):
				thr = ThrusterData()
				thr.T1 = message["T1"]
				thr.T2 = message["T2"]
				thr.T3 = message["T3"]
				thr.T4 = message["T4"]
				thrPub.publish(thr)
			elif(message["Thruster_M2"]):
				thr = ThrusterControlRequest()
				thr.comm_mode_F = float(message["CMFCtrl"])
				thr.diff_mode_F = float(message["DMFCtrl"])
				thr.comm_mode_V = float(message["CMVCtrl"])
				thr.diff_mode_V = float(message["DMVCtrl"])
				thr.update = 1
				thrusterClient(thr)

def Timer(event):
	global heartbeatCount, lastHearbeatTime
	timeElapsed = time.time() - lastHearbeatTime
	timeout = 0
	try:
		timeout = float(message["HeartTimeout"])
	except:
		timeout = float("inf")
	if(heartbeatCount>0 and timeElapsed<float(timeout)):
		return
	elif(heartbeatCount>0 and timeElapsed>float(timeout)):
		lastHearbeatTime = time.time()
		heartbeatCount-=1
		return
	else:
		print("/////////////")
		print("GUI Heartbeat Timeout\n Setting Thrusters OFF")
		rospy.set_param("/HIL_ON",0)
		while(heartbeatCount==0):
			timeElapsed = time.time() - lastHearbeatTime
			time.sleep(0.1)
		print("GUI Heartbeat Recieved\n You can set Thrusters ON manually")
		return


if __name__ == "__main__":
	rospy.wait_for_service("/missionParser")
	missClient = rospy.ServiceProxy("/missionParser",MissInputs)
	rospy.Subscriber("/AHRS",AHRS,ahrsCallback)
	rospy.Subscriber("/GPS",GPS,gpsCallback)
	rospy.Timer(rospy.Duration(0.1), getData)
	rospy.Timer(rospy.Duration(0.1), sendStatusUpdate)
	rospy.Timer(rospy.Duration(0.1), Timer)
	rospy.spin()