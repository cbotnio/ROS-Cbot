#!/usr/bin/env python
import matplotlib
# import matplotlib.animation as animation
matplotlib.use('Agg')
import rospy,time
from cbot_ros_msgs.msg import AHRS
import matplotlib.pyplot as plt

rospy.init_node('Plotter')
roll = []
pitch = []
yaw = []
rollRate = []
pitchRate = []
yawRate = []
t = []
Ts = 0.1

fig,(rollFig,pitchFig,yawFig,rollRateFig,pitchRateFig,yawRateFig) = plt.subplots(6)
# rollFig.set_xlabel("Time")
# rollFig.set_ylabel("Roll(degree)")
# rollFig.grid()

# pitchFig.set_xlabel("Time")
# pitchFig.set_ylabel("Pitch(degree)")
# pitchFig.grid()

# yawFig.set_xlabel("Time")
# yawFig.set_ylabel("Yaw(degree)")
# yawFig.grid()

# rollRateFig.set_xlabel("Time")
# rollRateFig.set_ylabel("Roll Rate(deg/sec)")
# rollRateFig.grid()

# pitchRateFig.set_xlabel("Time")
# pitchRateFig.set_ylabel("Pitch Rate(deg/sec)")
# pitchRateFig.grid()

# yawRateFig.set_xlabel("Time")
# yawRateFig.set_ylabel("Yaw Rate(deg/sec)")
yawRateFig.grid()

def plotData():
	rollFig.plot(t,roll)
	pitchFig.plot(t,pitch)
	yawFig.plot(t,yaw)
	rollRateFig.plot(t,rollRate)
	pitchRateFig.plot(t,pitchRate)
	yawRateFig.plot(t,yawRate)
	# plt.plot(roll)
	plt.show(block=True)
	# plt.pause(0.000001)
	#plt.close('all')
	print(yaw[-1])
	

def AHRScallback(data):
	global t;
	if(len(t)!=0):
		t.append(t[-1]+Ts)
	else:
		t = [0]
	roll.append(data.Roll)
	pitch.append(data.Pitch)
	yaw.append(data.YawAngle)
	rollRate.append(data.RollRate)
	pitchRate.append(data.PitchRate)
	yawRate.append(data.YawRate)
	plotData()

# while not rospy.is_shutdown():
if __name__=="__main__":
	AHRSsub = rospy.Subscriber('/AHRS',AHRS,AHRScallback)
	plt.ion()
	plt.show()
	rospy.spin()

plt.show()