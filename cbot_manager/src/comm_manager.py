#!/usr/bin/env python
import serial, time, json, os
from cbot_ros_msgs.msg import *
from cbot_ros_msgs.srv import *
from std_srvs.srv import SetBool, SetBoolResponse
from Compilers import MissionCompiler
import serial

class CommManager():
	def __init__(self,port,baudrate,timeout=None,rtscts=True,dsrdtr=True):
		self.name = name
		self.ser_baudrate = baudrate
		try:
			self.ser = serial.Serial(port =port,baudrate=baudrate,timeout=timeout,rtscts=rtscts,dsrdtr=dsrdtr)
			return(self.ser)
		except:
			return 0
		self.serialConnected = 0

	def serStatus(self):
	  if(self.serialConnected):
	    try:
	      if(self.ser.in_waiting>0):
	        return 1
	      elif(self.ser.in_waiting>=0):
	        return 2
	      return 0
	    except:
	      self.serialConnected = 0
	      return 0
	  else:
	    return 0

	def updatePort(self,portname,baudrate):
		if(portname!=self.name):
			try:
				self.ser.port = portname
				if(not self.ser.isOpen()):
					self.ser.open()
					self.name = portname
			except:
				self.ser.port = self.name
				if(not self.ser.isOpen()):
					self.ser.open()
		if(baudrate!=self.baudrate):
			try:
				self.ser.baudrate = baudrate
				self.baudrate = baudrate
			except:
				pass

	def write(self,message):
		if(self.serStatus()!=0):
			ser.flushOutput()
			ser.write(message.encode())

	def read(self):
		if(self.serStatus()==1):
			message = self.ser.readline().decode()