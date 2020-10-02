#!/usr/bin/env python
import PySimpleGUI as sg
import serial, time, re, json, threading, datetime
import MissionCompiler, kmlCompiler

serialPortName = "/tmp/GUI-write"
serialBaudrate = 9600

# ser = serial.Serial(port=serialPortName,baudrate=serialBaudrate,writeTimeout=0.1,timeout=0.01,rtscts=True,dsrdtr=True)
ser = serial.Serial()
boatGPSser = serial.Serial()

serialConnected = 0
statusTextFont = ("Helvetica",15)
statusDataSize = (10,1)
timerFont = ("Helvetica",20)
titleColor = "red"


sg.SetOptions(element_padding=(1,1))

statusData = {"Battery": 100, "Latitude": 15.4507, "Longitude": 73.8041, "Speed": 0.0, "Course": 0.0, "Roll": 0.0, "Pitch":0.0, "Yaw":0.0}

safetyParams = {"MaxDepth": 10,"MaxPitch":15, "MaxVelocity": 1}

HeartbeatTimeout = 5.0
heartbeatRate = 1

stepsVelocity = [-safetyParams["MaxVelocity"],safetyParams["MaxVelocity"],0.1]
stepsDepth = [-safetyParams["MaxDepth"],safetyParams["MaxDepth"],0.1]
stepsPitch = [-safetyParams["MaxPitch"],safetyParams["MaxPitch"],1]
stepsRoll = [-90,90,1]
stepsYaw = [-360,360,1]

startTime = time.time()
connectedTimerFlag = 0
connectedCounter = 0

invalidMessageFlag = 0
missionLoadStatus = 0

missionDictionary = {}

MEDlastHeartbeat = time.time()
MEDheartbeatCount = 5
MEDheartbeatTimeout = 2
MEDconnectionStatus = 1
firstConnectionFlag = 0

sg.theme('DefaultNoMoreNagging')

def getRange(start,end,stepSize=0.1,precision=2):
  return list([round(i*stepSize,precision) for i in range(int(float(start)/stepSize),int(float(end)/stepSize)+1)])

status = [[sg.Frame(layout=[
          [sg.Text("%s:%s:%s"%((datetime.datetime.now()).hour,(datetime.datetime.now()).minute,(datetime.datetime.now()).second),size=statusDataSize, font=timerFont, justification='center', key='-SYSTEMTIME-')]],
          title="Time",title_color=titleColor,font=statusTextFont)],
          [sg.Frame(layout=[
          [sg.Text(text="00:00:00",size=statusDataSize, font=timerFont, justification='center', key='-TIMER-')]],
          title="Connection Time",title_color=titleColor,font=statusTextFont)],
          [sg.Frame(layout=[
          [sg.Text('Battery: ',font=statusTextFont, justification="left"),
           sg.Text("0%",size=statusDataSize,font=statusTextFont,justification="left", key="Battery",enable_events=True)],
          
          [sg.Text('Latitude: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont, justification="left",key="Latitude",enable_events=True)],
          
          [sg.Text('Longitude: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Longitude",enable_events=True)],

          [sg.Text('Depth: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Depth",enable_events=True)],           

          [sg.Text('Speed: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Speed",enable_events=True)],

          [sg.Text('Course: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Course",enable_events=True)],

          [sg.Text('Roll: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Roll",enable_events=True)],

          [sg.Text('Pitch: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Pitch",enable_events=True)],

          [sg.Text('Yaw: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Yaw",enable_events=True)],],

          title="Status",title_color=titleColor,font=statusTextFont)]]

buttons = [[sg.Frame(layout=[
                [sg.Checkbox('AUV', default=False, font=statusTextFont, key='AUV_mode',enable_events=True),
                 sg.Checkbox('ROV', default=True, font=statusTextFont, key='Teleop_mode',enable_events=True)]],
                             title='Select Mode',title_color=titleColor,font=statusTextFont),
            sg.Frame(layout=[
                [sg.Checkbox('Drive', default=False, disabled=True, font=statusTextFont, key='Drive',enable_events=True),
                 sg.Checkbox('Park', default=False, disabled=True, font=statusTextFont, key='Park',enable_events=True),
                 sg.Checkbox('Stop', default=True, disabled=True, font=statusTextFont, key='MissionInactive',enable_events=True)]],
                             title='AUV Mission Status',title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.Checkbox('Guidance', default=False, font=statusTextFont, key='GUIDANCE_ON'),
                 sg.Checkbox('Controller', default=False, font=statusTextFont, key='CONTROLLER_ON',enable_events=True),
                 sg.Checkbox('Thrusters', default=True, font=statusTextFont, key='THRUSTERS_ON',enable_events=True)]],
                             title='Settings',title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.Checkbox('Individual', default=True, font=statusTextFont, key='Thruster_M1',enable_events=True),
                 sg.Checkbox('CMDM', default=False, font=statusTextFont, key='Thruster_M2',enable_events=True)],
                [sg.T('T1:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, background_color='white', text_color='black',key="T1",enable_events=True),
                 sg.T('T2:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, background_color='white', text_color='black',key="T2",enable_events=True),
                 sg.T('T3:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, background_color='white', text_color='black',key="T3",enable_events=True),
                 sg.T('T4:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, background_color='white', text_color='black',key="T4",enable_events=True)],
                [sg.T('Common Mode Forward:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, disabled=True, background_color='white', text_color='black',key="CMFCtrl",enable_events=True),
                 sg.T('Diff Mode Forward:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, disabled=True, background_color='white', text_color='black',key="DMFCtrl",enable_events=True)],
                [sg.T('Common Mode Vertical:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, disabled=True, background_color='white', text_color='black',key="CMVCtrl",enable_events=True),
                 sg.T('Diff Mode Vertical:',font=statusTextFont), sg.In('0.0',size=statusDataSize,font=statusTextFont, disabled=True, background_color='white', text_color='black',key="DMVCtrl",enable_events=True)]],
                            title="Thruster Inputs",title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.Checkbox('Heading Control\t',disabled=True, default=False, font=statusTextFont,key='HeadingControlON',enable_events=True),
                 sg.T('Heading Angle:', font=statusTextFont, size=(20,1),justification="right"), sg.Spin(values=getRange(stepsYaw[0],stepsYaw[1],stepsYaw[2]),initial_value=0.0,font=statusTextFont,size=statusDataSize,disabled=True, background_color='white', text_color='black',key="HCtrl")],
                [sg.Checkbox('Pitch Control\t',disabled=True, default=False, key='PitchControlON',font=statusTextFont,enable_events=True),
                 sg.T('Pitch Angle:', font=statusTextFont,size=(20,1),justification="right"), sg.Spin(values=getRange(stepsPitch[0],stepsPitch[1],stepsPitch[2]),initial_value=0.0,font=statusTextFont,size=statusDataSize, disabled=True, background_color='white', text_color='black',key="PCtrl")],
                [sg.Checkbox('Depth Control\t',disabled=True, default=False,font=statusTextFont, key='DepthControlON',enable_events=True),
                 sg.T('Depth:', font=statusTextFont,size=(20,1),justification="right"), sg.Spin(values=getRange(stepsDepth[0],stepsDepth[1],stepsDepth[2]),initial_value=0.0,font=statusTextFont,size=statusDataSize, disabled=True, background_color='white', text_color='black',key="DCtrl")],
                [sg.Checkbox('Speed Control\t',disabled=True, default=False, font=statusTextFont, key='SpeedControlON',enable_events=True),
                 sg.T('Speed:', font=statusTextFont,size=(20,1),justification="right"), sg.Spin(values=getRange(stepsVelocity[0],stepsVelocity[1],stepsVelocity[2]),initial_value=0.0, font=statusTextFont,size=statusDataSize,disabled=True, background_color='white', text_color='black',key="SCtrl")],],
                            title="Controller Config",title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.Text('File:', font=statusTextFont), sg.Input(key="MissionFile",font=statusTextFont), sg.FileBrowse(font=statusTextFont)],
                [sg.B('Compile', key="MissionCompile",font=statusTextFont),
                sg.B('Load', key="MissionLoad",disabled=True,font=statusTextFont),
                sg.B('Execute', key="MissionExecute",disabled=True,font=statusTextFont)]],
                            title="Mission File", title_color=titleColor,font=statusTextFont)],
            [sg.Submit('Update')]]


col1 =    [ [sg.Frame(layout=[
                [sg.T('DISCONNECTED',font=statusTextFont,justification="center", background_color="red", key='VehicleStatus')]],
                            title="Vehicle Status", title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.T("Timeout: ", font=statusTextFont), 
                 sg.In(str(HeartbeatTimeout),size=(10,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="HeartTimeout")]],
                            title="Heartbeat Timeout", title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.T("Port Name:", font=statusTextFont),
                 sg.In(serialPortName,size=(15,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="PortName")],
                [sg.T("Port Baud: ", font=statusTextFont),
                 sg.In(str(serialBaudrate),size=(15,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="PortBaudrate")],
                [sg.B('Connect', key="ConnectMED",font=statusTextFont)]],
                            title="Vehicle", title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.T("Port Name:", font=statusTextFont),
                 sg.In(serialPortName,size=(15,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="BoatGPSPortName")],
                [sg.T("Port Baud: ", font=statusTextFont),
                 sg.In(str(serialBaudrate),size=(15,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="BoatGPSPortBaudrate")],
                [sg.B('Connect', key="ConnectBoatGPS",font=statusTextFont)]],
                            title="Boat GPS", title_color=titleColor,font=statusTextFont)],
            [sg.Frame(layout=[
                [sg.T("Max Depth: ", font=statusTextFont), 
                 sg.In(str(safetyParams["MaxDepth"]),size=(10,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="MaxDepth")],
                [sg.T("Max Pitch: ", font=statusTextFont), 
                 sg.In(str(safetyParams["MaxPitch"]),size=(10,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="MaxPitch")],
                [sg.T("Max Velocity: ", font=statusTextFont), 
                 sg.In(str(safetyParams["MaxVelocity"]),size=(10,1),font=statusTextFont, disabled=False, background_color='white', text_color='black',key="MaxVelocity")]],
                            title="Safety Parameters", title_color=titleColor,font=statusTextFont)]]


layout = [
    [sg.Column(col1),
    sg.VerticalSeparator(),
    sg.Column(buttons),
    sg.VerticalSeparator(),
    sg.Column(status)]
]

window = sg.Window("CBOT Control", layout)

def updateStatus(ser):
  global window, MEDheartbeatCount, MEDlastHeartbeat, firstConnectionFlag
  if(isSerOpen(ser)==1):
    readData = ser.readline().decode().strip().split(',')
    if(readData[0]=="MED"):
      for param in readData[1:]:
        param = param.split(':')
        try:
          if(param[0]!=""):
            window.Element(param[0].strip()).Update(param[1])
        except:
          pass
      firstConnectionFlag = 1
      MEDlastHeartbeat = time.time()
      MEDheartbeatCount = 5

def updatePort(ser,portname,baudrate):
	global serialBaudrate,serialPortName,window
	if(portname!=serialPortName):
		try:
			ser.port = portname
			if(not ser.isOpen()):
				ser.open()
				serialPortName = portname
		except:
			ser.port = serialPortName
			if(not ser.isOpen()):
				ser.open()
	if(baudrate!=serialBaudrate):
		try:
			ser.baudrate = baudrate
			serialBaudrate = baudrate
		except:
			pass
	window.Element("PortName").Update(serialPortName)
	window.Element("PortBaudrate").Update(serialBaudrate)

def sendMessage(ser,values):
  if(isSerOpen(ser)!=0):
    message = "GUI,"
    for key in values:
      if (values[key]==True):
        msg = "1"
      elif(values[key]==False):
        msg = "0"
      elif(values[key]==""):
        msg = "null"
      else:
        msg = str(values[key])
      if(not ("port" in str(key).lower()) and key!="Browse" and key!="MissionFile"):
        message += key + ":" + msg + ","
    message += "\r\n"
    ser.flushOutput()
    ser.write(message.encode())

def checkValidNumber(num):
  num = str(num)
  return bool(re.match("^-?\d+?\.\d+?$", num)) or bool(re.match("^-?\d+?\.+?$", num)) or bool(re.match("^-?\d+$", num)) or bool(re.match("^.\d+?$", num)) 

def updateHeartbeat(values):
  global window, HeartbeatTimeout, invalidMessageFlag
  if(checkValidNumber(values["HeartTimeout"])):
    HeartbeatTimeout = float(values["HeartTimeout"])
  else:
    invalidMessageFlag = 1
  values["HeartTimeout"] = HeartbeatTimeout
  window.Element("HeartTimeout").Update(value=HeartbeatTimeout)
  return values

def updateSafetyParams(values):
  global window, safetyParams, stepsVelocity, stepsDepth, stepsPitch, stepsYaw, invalidMessageFlag
  invFlag = 0
  if(checkValidNumber(values["MaxDepth"])):
    safetyParams["MaxDepth"] = float(values["MaxDepth"])
    stepsDepth[0:2] = [-safetyParams["MaxDepth"],safetyParams["MaxDepth"]]
  else:
    values["MaxDepth"] = safetyParams["MaxDepth"]
    invFlag=1
  if(checkValidNumber(values["MaxPitch"])):
    safetyParams["MaxPitch"] = float(values["MaxPitch"])
    stepsPitch[0:2] = [-safetyParams["MaxPitch"],safetyParams["MaxPitch"]]
  else:
    values["MaxPitch"] = safetyParams["MaxPitch"]
    invFlag=1
  if(checkValidNumber(values["MaxVelocity"])):
    safetyParams["MaxVelocity"] = float(values["MaxVelocity"])
    stepsVelocity[0:2] = [-safetyParams["MaxVelocity"],safetyParams["MaxVelocity"]]
  else:
    values["MaxVelocity"] = safetyParams["MaxVelocity"]
    invFlag=1
  if(invFlag):
    invalidMessageFlag = 1
    sg.popup("Invalid Safety Parameter",title="Error")
  values["MaxDepth"] = safetyParams["MaxDepth"]
  values["MaxVelocity"] = safetyParams["MaxVelocity"]
  values["MaxPitch"] = safetyParams["MaxPitch"]
  window.Element("MaxDepth").Update(value=values["MaxDepth"])
  window.Element("MaxVelocity").Update(value=values["MaxVelocity"])
  window.Element("MaxPitch").Update(value=values["MaxPitch"])
  return values


def checkControlValues(values):
  global window, invalidMessageFlag
  if(not (checkValidNumber(values["HCtrl"]) and checkValidNumber(values["DCtrl"]) and checkValidNumber(values["PCtrl"]) and checkValidNumber(values["SCtrl"]))):
    values["HCtrl"] = float(values["HCtrl"]) if(checkValidNumber(values["HCtrl"])) else 0.0
    values["DCtrl"] = float(values["DCtrl"]) if(checkValidNumber(values["DCtrl"])) else 0.0
    values["PCtrl"] = float(values["PCtrl"]) if(checkValidNumber(values["PCtrl"])) else 0.0
    values["SCtrl"] = float(values["SCtrl"]) if(checkValidNumber(values["SCtrl"])) else 0.0
    invalidMessageFlag = 1
    sg.popup("Invalid Control Input",title="Error")

  window.Element('HCtrl').Update(value=values["HCtrl"])
  window.Element('DCtrl').Update(value=values["DCtrl"])
  window.Element('PCtrl').Update(value=values["PCtrl"])
  window.Element('SCtrl').Update(value=values["SCtrl"])
  return values

def updateThrusters(values,mode=""):
  global window, invalidMessageFlag
  if(mode=="Update"):
    if(not (checkValidNumber(values["T1"]) and checkValidNumber(values["T2"]) and checkValidNumber(values["T3"]) and checkValidNumber(values["T4"]) and checkValidNumber(values["CMFCtrl"]) and checkValidNumber(values["DMFCtrl"]) and checkValidNumber(values["CMVCtrl"]) and checkValidNumber(values["DMVCtrl"]))):
      values["T1"] = float(values["T1"]) if(checkValidNumber(values["T1"])) else 0.0
      values["T2"] = float(values["T2"]) if(checkValidNumber(values["T2"])) else 0.0
      values["T3"] = float(values["T3"]) if(checkValidNumber(values["T3"])) else 0.0
      values["T4"] = float(values["T4"]) if(checkValidNumber(values["T4"])) else 0.0
      values["CMFCtrl"] = float(values["CMFCtrl"]) if(checkValidNumber(values["CMFCtrl"])) else 0.0
      values["DMFCtrl"] = float(values["DMFCtrl"]) if(checkValidNumber(values["DMFCtrl"])) else 0.0
      values["CMVCtrl"] = float(values["CMVCtrl"]) if(checkValidNumber(values["CMVCtrl"])) else 0.0
      values["DMVCtrl"] = float(values["DMFCtrl"]) if(checkValidNumber(values["DMFCtrl"])) else 0.0
      invalidMessageFlag = 1
  
  window.Element('CMFCtrl').Update(value=values["CMFCtrl"])
  window.Element('DMFCtrl').Update(value=values["DMFCtrl"])
  window.Element('CMVCtrl').Update(value=values["CMVCtrl"])
  window.Element('DMVCtrl').Update(value=values["DMVCtrl"])
  window.Element('T1').Update(value=values["T1"])
  window.Element('T2').Update(value=values["T2"])
  window.Element('T3').Update(value=values["T3"])
  window.Element('T4').Update(value=values["T4"])
  return values

def Timer(starttime,timeout):
  timeElapsed = time.time() - starttime
  if(timeElapsed>float(timeout)):
    return True
  else:
    return False

def Timer2(window):
  global MEDheartbeatCount, MEDlastHeartbeat, MEDheartbeatTimeout
  timeElapsed = time.time() - MEDlastHeartbeat
  if(timeElapsed>MEDheartbeatTimeout and MEDheartbeatCount>0):
    MEDheartbeatCount-=1
    MEDlastHeartbeat = time.time()
  elif(MEDheartbeatCount==0):
    window.Element("VehicleStatus").Update("DISCONNECTED",background_color="red")
    MEDheartbeatCount = 0
    MEDconnectionStatus = 0
  if(MEDheartbeatCount>0):
    window.Element("VehicleStatus").Update("CONNECTED",background_color="green")

def sendMissionFile(ser):
  global missionDictionary, missionLoadStatus
  if(isSerOpen(ser)!=0):
    message2 = json.dumps(missionDictionary)
    message2 = "GUI,MISSION," + str(message2) + "\r\n"
    ser.flushOutput()
    ser.write(message2.encode())


def sendExecuteMessage(ser):
  if(isSerOpen(ser)!=0):
    message3 = "GUI,EXECUTEMISSION" + "\r\n"
    ser.flushOutput()
    ser.write(message3.encode())

def compileMission(values,window):
  global missionLoadStatus, missionDictionary
  missionLoadStatus = 0
  if(values["MissionFile"]!="null" and values["MissionFile"]!=""):
    values["MissionFile"] = str(values["MissionFile"])
    ext = values["MissionFile"].strip().split('.')
    if(ext[-1]=="txt"):
      try:
        missionDictionary = MissionCompiler.readMission(values["MissionFile"])
        window.Element("MissionLoad").Update(disabled=False)
      except Exception as e:
        window.Element("MissionLoad").Update(disabled=True)
    elif(ext[-1]=="kml"):
      try:
        missionDictionary = kmlCompiler.read(filename=values["MissionFile"])
        window.Element("MissionLoad").Update(disabled=False)
      except Exception as e:
        window.Element("MissionLoad").Update(disabled=True)
    else:
      window.Element("MissionLoad").Update(disabled=True)

def isSerOpen(ser):
  global serialConnected
  if(serialConnected):
    try:
      if(ser.in_waiting>0):
        return 1
      elif(ser.in_waiting>=0):
        return 2
      return 0
    except:
      serialConnected = 0
      return 0
  else:
    serialConnected = 0
    return 0

def sendHeartbeat(ser):
  global startTime
  if(isSerOpen(ser)!=0):
    message = "GUIHEARTBEAT\r\n"
    ser.write(message.encode())
    startTime = time.time()

def updateSystemTime(window):
  e = datetime.datetime.now()
  window.Element("-SYSTEMTIME-").Update("{:02d}:{:02d}:{:02d}".format(int(e.hour),int(e.minute),int(e.second)))

def updateConnectedTimer(window,t2):
  t = time.time() - t2
  m = t // 60
  s = int(t % 60)
  h = int(m // 60)
  m = int(m % 60)
  window.Element("-TIMER-").Update("{:02d}:{:02d}:{:02d}".format(h,m,s))

while True:
    event, values = window.read(timeout = 10)
    updateSystemTime(window)

    while(not serialConnected):
      window.Element("VehicleStatus").Update("DISCONNECTED",background_color="red")
      MEDheartbeatCount = 0
      event, values = window.read(timeout=100)
      updateSystemTime(window)
      if(event=="ConnectMED" or event=="ConnectBoatGPS"):
        try:
          ser = serial.Serial(port=values["PortName"],baudrate=values["PortBaudrate"],timeout=0.1,rtscts=True,dsrdtr=True)
          print("CONNECTED to serial")
          serialConnected = 1
          connStartTime = time.time()
        except:
          serialConnected = 0
          pass
      if(event == sg.WIN_CLOSED or event == 'Exit' or event==None):
        break
      time.sleep(0.1)

    if(event == sg.WIN_CLOSED or event == 'Exit' or event==None):
        break

    threading.Thread(target=updateStatus,args=(ser,),daemon=True).start()

    if(Timer(startTime,float(heartbeatRate))):
      threading.Thread(target=sendHeartbeat,args=(ser,),daemon=True).start()
    

    if(serialConnected):
      updateConnectedTimer(window,connStartTime)

    if(firstConnectionFlag):
      Timer2(window)
    
      
    if(values["Teleop_mode"]==1 and event=="Teleop_mode"):
      window.Element("AUV_mode").Update(False)
      window.Element("MissionExecute").Update(disabled=True)
      window.Element("THRUSTERS_ON").Update(value=False,disabled=False)
      window.Element("CONTROLLER_ON").Update(value=False,disabled=False)
      window.Element("GUIDANCE_ON").Update(value=False,disabled=False)
      window.Element("Drive").Update(disabled=True)
      window.Element("Park").Update(disabled=True)
      window.Element("MissionInactive").Update(disabled=True)
    elif(values["AUV_mode"]==1 and event=="AUV_mode"):
      window.Element("Teleop_mode").Update(False)
      window.Element("MissionExecute").Update(disabled=(missionLoadStatus==0))
      window.Element("THRUSTERS_ON").Update(value=True,disabled=True)
      window.Element("CONTROLLER_ON").Update(value=True,disabled=True)
      window.Element("GUIDANCE_ON").Update(value=True,disabled=True)
      window.Element("Drive").Update(disabled=False)
      window.Element("Park").Update(disabled=False)
      window.Element("MissionInactive").Update(disabled=False)
    elif(values["Teleop_mode"]==0 and values["AUV_mode"]==0):
      window.Element("MissionExecute").Update(disabled=True)
      window.Element("THRUSTERS_ON").Update(value=False,disabled=True)
      window.Element("CONTROLLER_ON").Update(value=False,disabled=True)
      window.Element("GUIDANCE_ON").Update(value=False,disabled=True)
      window.Element("Drive").Update(disabled=True)
      window.Element("Park").Update(disabled=True)
      window.Element("MissionInactive").Update(disabled=True)

    if(values["Drive"]==1 and event=="Drive"):
    	window.Element("Drive").Update(value=True)
    	window.Element("Park").Update(value=False)
    	window.Element("MissionInactive").Update(value=False)
    elif(values["Park"]==1 and event=="Park"):
    	window.Element("Drive").Update(value=False)
    	window.Element("Park").Update(value=True)
    	window.Element("MissionInactive").Update(value=False)
    elif(values["MissionInactive"]==1 and event=="MissionInactive"):
    	window.Element("Drive").Update(value=False)
    	window.Element("Park").Update(value=False)
    	window.Element("MissionInactive").Update(value=True)

    if(values["Thruster_M1"]==1 and event=="Thruster_M2"):
      window.Element("Thruster_M1").Update(False)
    elif(values["Thruster_M2"]==1 and event=="Thruster_M1"):
      window.Element("Thruster_M2").Update(False)

    window.Element("Thruster_M1").Update(disabled=(values["THRUSTERS_ON"]==0 or values["CONTROLLER_ON"]!=0 or values["AUV_mode"]==1))
    window.Element("Thruster_M2").Update(disabled=(values["THRUSTERS_ON"]==0 or values["CONTROLLER_ON"]!=0 or values["AUV_mode"]==1))
    window.Element('T1').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M1"]==0  or values["CONTROLLER_ON"]!=0))
    window.Element('T2').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M1"]==0  or values["CONTROLLER_ON"]!=0))
    window.Element('T3').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M1"]==0  or values["CONTROLLER_ON"]!=0))
    window.Element('T4').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M1"]==0  or values["CONTROLLER_ON"]!=0))
    window.Element('CMFCtrl').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M2"]==0 or values["CONTROLLER_ON"]!=0))
    window.Element('DMFCtrl').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M2"]==0 or values["CONTROLLER_ON"]!=0))
    window.Element('CMVCtrl').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M2"]==0 or values["CONTROLLER_ON"]!=0))
    window.Element('DMVCtrl').Update(disabled=(values["THRUSTERS_ON"]==0 or values["Thruster_M2"]==0 or values["CONTROLLER_ON"]!=0))

    if(event=="T1" or event=="T2" or event=="T3" or event=="T4"):
      if(checkValidNumber(values["T1"]) and checkValidNumber(values["T2"])):
        values["CMFCtrl"] = float(values["T1"])+float(values["T2"])
        values["DMFCtrl"] = float(values["T1"])-float(values["T2"])
      else:
        values["CMFCtrl"] = 0
        values["DMFCtrl"] = 0
      if(checkValidNumber(values["T3"]) and checkValidNumber(values["T4"])):
        values["CMVCtrl"] = float(values["T3"])+float(values["T4"])
        values["DMVCtrl"] = float(values["T3"])-float(values["T4"])
      else:
        values["CMVCtrl"] = 0
        values["DMVCtrl"] = 0
      updateThrusters(values)

    elif(event=="CMFCtrl" or event=="DMFCtrl" or event=="CMVCtrl" or event=="DMVCtrl"):
      if(checkValidNumber(values["CMFCtrl"]) and checkValidNumber(values["DMFCtrl"])):
        values["T1"] = (float(values["CMFCtrl"])+float(values["DMFCtrl"]))/2.0
        values["T2"] = (float(values["CMFCtrl"])-float(values["DMFCtrl"]))/2.0
      else:
        values["T1"] = 0
        values["T2"] = 0
      if(checkValidNumber(values["CMVCtrl"]) and checkValidNumber(values["DMVCtrl"])):
        values["T3"] = (float(values["CMVCtrl"])+float(values["DMVCtrl"]))/2.0
        values["T4"] = (float(values["CMVCtrl"])-float(values["DMVCtrl"]))/2.0
      else:
        values["T3"] = 0
        values["T4"] = 0
      updateThrusters(values)

    if(values["CONTROLLER_ON"] and values["Teleop_mode"]==1):
      if(values['HeadingControlON']):
        window.Element('HCtrl').Update(disabled=False)
      else:
        window.Element('HCtrl').Update(disabled=True)
      if(values['PitchControlON']):
        window.Element('PCtrl').Update(disabled=False)
      else:
        window.Element('PCtrl').Update(disabled=True)
      if(values['DepthControlON']):
        window.Element('DCtrl').Update(disabled=False)
      else:
        window.Element('DCtrl').Update(disabled=True)
      if(values['SpeedControlON']):
        window.Element('SCtrl').Update(disabled=False)
      else:
        window.Element('SCtrl').Update(disabled=True)

      window.Element('HeadingControlON').Update(disabled=False)
      window.Element('PitchControlON').Update(disabled=False)
      window.Element('DepthControlON').Update(disabled=False)
      window.Element('SpeedControlON').Update(disabled=False)

    else:
      window.Element('HeadingControlON').Update(False,disabled=True)
      window.Element('HCtrl').Update(disabled=True)
      window.Element('PitchControlON').Update(False,disabled=True)
      window.Element('PCtrl').Update(disabled=True)
      window.Element('DepthControlON').Update(False,disabled=True)
      window.Element('DCtrl').Update(disabled=True)
      window.Element('SpeedControlON').Update(False,disabled=True)
      window.Element('SCtrl').Update(disabled=True)
    
    if(event=="Update"):
      invalidMessageFlag = 0
      values = updateHeartbeat(values)
      values = updateSafetyParams(values)
      values = updateThrusters(values,mode="Update")
      values = checkControlValues(values)

      window.Element("DCtrl").Update(values=getRange(stepsYaw[0],stepsYaw[1],stepsYaw[2]))
      window.Element("DCtrl").Update(values=getRange(stepsDepth[0],stepsDepth[1],stepsDepth[2]))
      window.Element("PCtrl").Update(values=getRange(stepsPitch[0],stepsPitch[1],stepsPitch[2]))
      window.Element("SCtrl").Update(values=getRange(stepsVelocity[0],stepsVelocity[1],stepsVelocity[2]))

      updatePort(ser,values["PortName"],values["PortBaudrate"])
      if(not invalidMessageFlag):
        threading.Thread(target=sendMessage,args=(ser,values,),daemon=True).start()
      else:
        sg.popup("Message not sent",title="Error")

    elif(event=="MissionCompile"):
      threading.Thread(target=compileMission,args=(values,window,),daemon=True).start()

    elif(event=="MissionLoad"):
      threading.Thread(target=sendMissionFile,args=(ser,),daemon=True).start()
      missionLoadStatus = 1
    elif(event=="MissionExecute"):
      threading.Thread(target=sendMessage,args=(ser,values,),daemon=True).start()
      time.sleep(1)
      threading.Thread(target=sendExecuteMessage,args=(ser,),daemon=True).start()
    time.sleep(0.1)

window.close()
ser.close()