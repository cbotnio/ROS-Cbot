#!/usr/bin/env python
import PySimpleGUI27 as sg
import os.path
from PIL import Image
import serial, time

ser = serial.Serial(port="/home/mohit/nio/src/GUI-write",baudrate=9600,writeTimeout=0.1,timeout=0.01,rtscts=True,dsrdtr=True)

# sizeImage = (500,500)
# imageName = "/home/mohit/nio/src/ROS-Cbot-master/cbot_manager/src/cbot_image.png"
# newImageName = "/home/mohit/nio/src/ROS-Cbot-master/cbot_manager/src/cbot_image_resized.png"
# image = Image.open(imageName)
# image = image.resize((sizeImage))
# image.save(newImageName)

statusTextFont = ("Helvetica",12)
statusDataSize = (10,1)

sg.SetOptions(element_padding=(1,1))

statusData = {"Battery": 100, "Latitude": 15.4507, "Longitude": 73.8041, "Speed": 0.0, "Course": 0.0, "Roll": 0.0, "Pitch":0.0, "Yaw":0.0}

safetyParams = {"MaxDepth": 10,"MaxPitch":15, "MaxVelocity": 1}

HeartbeatTimeout = 5.0
heartbeatRate = 1

stepSizeVelocity = 0.1
stepSizeDepth = 0.1
stepSizePitch = 1.0
stepSizeRoll = 1.0
stepSizeYaw = 1.0

startTime = time.time()

status = [[sg.Frame(layout=[
          [sg.Text('Battery: ',font=statusTextFont, justification="left"),
           sg.Text("0%",size=statusDataSize,font=statusTextFont,justification="left", key="Battery",enable_events=True)],
          
          [sg.Text('Latitude: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont, justification="left",key="Latitude",enable_events=True)],
          
          [sg.Text('Longitude: ',font=statusTextFont, justification="left"),
           sg.Text("0",size=statusDataSize,font=statusTextFont,justification="left",key="Longitude",enable_events=True)],

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

          title="Status",title_color="red")]]

buttons = [[sg.Frame(layout=[
                [sg.Checkbox('AUV', default=False, key='AUV_mode',enable_events=True),
                 sg.Checkbox('ROV', default=True, key='Teleop_mode',enable_events=True)]],
                             title='Select Mode',title_color='red'),
            sg.Frame(layout=[
                [sg.T("Timeout: ", pad=((1,0),0)), 
                 sg.In(str(HeartbeatTimeout),size=(10,1), disabled=False, background_color='white', text_color='black',key="HeartTimeout")]],
                            title="Heartbeat Timeout", title_color="red")],
            [sg.Frame(layout=[
                [sg.Checkbox('Drive', default=False, key='Drive'),
                 sg.Checkbox('Park', default=False, key='Park'),
                 sg.Checkbox('Inactive', default=True, key='MissionInactive')]],
                             title='AUV Mission Status',title_color='red')],
            [sg.Frame(layout=[
                [sg.Checkbox('Guidance', default=False, key='GUIDANCE_ON'),
                 sg.Checkbox('Controller', default=False, key='CONTROLLER_ON',enable_events=True),
                 sg.Checkbox('Thrusters', default=True, key='THRUSTERS_ON',enable_events=True)]],
                             title='Settings',title_color='red')],
            [sg.Frame(layout=[
                [sg.Checkbox('Individual', default=True, key='Thruster_M1',enable_events=True),
                 sg.Checkbox('CMDM', default=False, key='Thruster_M2',enable_events=True)],
                [sg.T('T1:', pad=((1,0),0)), sg.In('0.0',size=(10,1), background_color='white', text_color='black',key="T1"),
                 sg.T('T2:', pad=((1,0),0)), sg.In('0.0',size=(10,1), background_color='white', text_color='black',key="T2"),
                 sg.T('T3:', pad=((1,0),0)), sg.In('0.0',size=(10,1), background_color='white', text_color='black',key="T3"),
                 sg.T('T4:', pad=((1,0),0)), sg.In('0.0',size=(10,1), background_color='white', text_color='black',key="T4")],
                [sg.T('Common Mode Forward:', pad=((1,0),0)), sg.In('0.0',size=(10,1), disabled=True, background_color='white', text_color='black',key="CMFCtrl"),
                 sg.T('Diff Mode Forward:', pad=((1,0),0)), sg.In('0.0',size=(10,1), disabled=True, background_color='white', text_color='black',key="DMFCtrl")],
                [sg.T('Common Mode Vertical:', pad=((1,0),0)), sg.In('0.0',size=(10,1), disabled=True, background_color='white', text_color='black',key="CMVCtrl"),
                 sg.T('Diff Mode Vertical:', pad=((1,0),0)), sg.In('0.0',size=(10,1), disabled=True, background_color='white', text_color='black',key="DMVCtrl")]],
                            title="Thruster Inputs",title_color="red")],
            [sg.Frame(layout=[
                [sg.Checkbox('Heading Control\t',disabled=True, default=False, key='HeadingControlON',enable_events=True),
                 sg.T('Heading Angle:', pad=((1,0),0)), sg.Spin(values=[round(i*stepSizeYaw,2) for i in range(-int(360/stepSizeYaw),int(360/stepSizeYaw))],initial_value=0.0,size=(10,1),disabled=True, background_color='white', text_color='black',key="HCtrl")],
                [sg.Checkbox('Pitch Control\t',disabled=True, default=False, key='PitchControlON',enable_events=True),
                 sg.T('Pitch Angle:', pad=((1,0),0)), sg.Spin(values=[round(i*stepSizePitch,2) for i in range(-int(safetyParams["MaxPitch"]/stepSizePitch),int(safetyParams["MaxPitch"]/stepSizePitch+1))],initial_value=0.0,size=(10,1), disabled=True, background_color='white', text_color='black',key="PCtrl")],
                [sg.Checkbox('Depth Control\t',disabled=True, default=False, key='DepthControlON',enable_events=True),
                 sg.T('Depth:', pad=((1,0),0)), sg.Spin(values=[round(i*stepSizeDepth,2) for i in range(0,int(safetyParams["MaxDepth"]/stepSizeDepth+1))],initial_value=0.0,size=(10,1), disabled=True, background_color='white', text_color='black',key="DCtrl")],
                [sg.Checkbox('Speed Control\t',disabled=True, default=False, key='SpeedControlON',enable_events=True),
                 sg.T('Speed:', pad=((1,0),0)), sg.Spin(values=[round(i*stepSizeVelocity,2) for i in range(-int(safetyParams["MaxVelocity"]/stepSizeVelocity),int(safetyParams["MaxVelocity"]/stepSizeVelocity+1))],initial_value=0.0,size=(10,1), disabled=True, background_color='white', text_color='black',key="SCtrl")],],
                            title="Controller Config",title_color="red")],
            [sg.Frame(layout=[
                [sg.Text('File:', size=(8, 1)), sg.Input(key="MissionFile"), sg.FileBrowse()],
                [sg.B('Compile', key="MissionCompile"),sg.B('Load', key="MissionLoad",disabled=True)]],
                            title="Mission File", title_color="red")],
            [sg.Submit('Update')]
            ]
          
safety =    [[sg.Frame(layout=[
                [sg.T("Max Depth: ", pad=((1,0),0)), 
                 sg.In(str(safetyParams["MaxDepth"]),size=(10,1), disabled=False, background_color='white', text_color='black',key="MaxDepth")],
                [sg.T("Max Pitch: ", pad=((1,0),0)), 
                 sg.In(str(safetyParams["MaxPitch"]),size=(10,1), disabled=False, background_color='white', text_color='black',key="MaxPitch")],
                [sg.T("Max Velocity: ", pad=((1,0),0)), 
                 sg.In(str(safetyParams["MaxVelocity"]),size=(10,1), disabled=False, background_color='white', text_color='black',key="MaxVelocity")]],
                            title="Safety Parameters", title_color="red")]]

MFile =     [[sg.Frame(layout=[
                [sg.Text('File:', size=(8, 1)), sg.Input(key="MissionFile"), sg.FileBrowse()],
                [sg.B('Compile', key="MissionCompile"),sg.B('Load', key="MissionLoad",disabled=True)]],
                            title="Mission File", title_color="red")]]

layout = [
    [sg.Column(safety),
    sg.VerticalSeparator(),
    sg.Column(buttons),
    sg.VerticalSeparator(),
    sg.Column(status)]
]

window = sg.Window("CBOT Control", layout)

def updateStatus():
  ser.flushInput()
  readData = ser.readline().decode().strip().split(',')
  if(readData[0]=="MED"):
    for param in readData[1:]:
      param = param.split(':')
      try:
        if(param[0]!=""):
          window.Element(param[0].strip()).Update(param[1])
      except:
        pass

def Timer(starttime):
  timeElapsed = time.time() - starttime
  if(timeElapsed>float(heartbeatRate)):
    return True
  else:
    return False

while True:
    event, values = window.read(timeout = 0.01)
    if(event == "Exit" or event == None):
        break

    if(Timer(startTime)):
       ser.write(b"GUIHEARTBEAT\n")
       startTime = time.time()

    updateStatus()

    if((values["AUV_mode"]==1 or values["Teleop_mode"]==1) and event=="Teleop_mode"):
      window.Element("AUV_mode").Update(False)
      window.Element("THRUSTERS_ON").Update(value=False,disabled=False)
      window.Element("CONTROLLER_ON").Update(value=False,disabled=False)
      window.Element("GUIDANCE_ON").Update(value=False,disabled=False)
    elif((values["Teleop_mode"]==1 or values["AUV_mode"]==1)and event=="AUV_mode"):
      window.Element("Teleop_mode").Update(False)
      window.Element("THRUSTERS_ON").Update(value=True,disabled=True)
      window.Element("CONTROLLER_ON").Update(value=True,disabled=True)
      window.Element("GUIDANCE_ON").Update(value=True,disabled=True)
    elif(values["Teleop_mode"]==0 and values["AUV_mode"]==0):
      window.Element("THRUSTERS_ON").Update(value=False,disabled=True)
      window.Element("CONTROLLER_ON").Update(value=False,disabled=True)
      window.Element("GUIDANCE_ON").Update(value=False,disabled=True)
      

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
      HeartbeatTimeout = values["HeartTimeout"]
      safetyParams["MaxDepth"] = float(values["MaxDepth"])
      safetyParams["MaxPitch"] = float(values["MaxPitch"])
      safetyParams["MaxVelocity"] = float(values["MaxVelocity"])
      window.Element("DCtrl").Update(values=[round(i*stepSizeDepth,2) for i in range(0,int(safetyParams["MaxDepth"]/stepSizeDepth+1))])
      window.Element("PCtrl").Update(values=[round(i*stepSizePitch,2) for i in range(-int(safetyParams["MaxPitch"]/stepSizePitch),int(safetyParams["MaxPitch"]/stepSizePitch+1))])
      window.Element("SCtrl").Update(values=[round(i*stepSizeVelocity,2) for i in range(-int(safetyParams["MaxVelocity"]/stepSizeVelocity),int(safetyParams["MaxVelocity"]/stepSizeVelocity+1))])
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
        message += key + ":" + msg + ","
      message += "\r\n"
      ser.flushOutput()
      ser.write(message.encode())


window.close()
ser.close()
