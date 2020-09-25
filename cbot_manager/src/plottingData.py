#!/usr/bin/env python
import rospy,time
from cbot_ros_msgs.msg import AHRS
import matplotlib.pyplot as plt

rospy.init_node('Plotter')

fileNames = ["roll","pitch","yaw","rollRate","pitchRate","yawRate"]
filePointers = {}
fileData = {}
timeData = []


Fs = 50.0/3.0

for name in fileNames:
	filePointers[name] = open((name+".txt"),'w')

# roll = open('roll.txt','w')
# pitch = open('pitch.txt','w')
# yaw = open('yaw.txt','w')
# rollRate = open('rollRate.txt','w')
# pitchRate = open('pitchRate.txt','w')
# yawRate = open('yawRate.txt','w')

def PlotAll():
	plt.figure()	
	for name in fileNames:
		filePointers[name] = open((name+'.txt'),'r')
		fileData[name] = [float(x[0:-2]) for x in filePointers[name].readlines()]

	plt.subplot(611)
	plt.plot(timeData,fileData['roll'])
	plt.title("Roll")
	plt.xlabel("Time")
	plt.ylabel("Roll(degree)")
	plt.grid()

	plt.subplot(612)
	plt.plot(timeData,fileData['pitch'])
	plt.title("Pitch")
	plt.xlabel("Time")
	plt.ylabel("Pitch(degree)"3)
	plt.grid()

	plt.subplot(613)
	plt.plot(timeData,fileData['yaw'])
	plt.title("Yaw")
	plt.xlabel("Time")
	plt.ylabel("Yaw (degree)")
	plt.grid()

	plt.subplot(614)
	plt.plot(timeData,fileData['rollRate'])
	plt.title("Roll Rate")
	plt.xlabel("Time")
	plt.ylabel("Roll Rate(degree/sec)")
	plt.grid()

	plt.subplot(615)
	plt.plot(timeData,fileData['pitchRate'])
	plt.title("Pitch Rate")
	plt.xlabel("Time")
	plt.ylabel("Pitch Rate(degree/sec)")
	plt.grid()

	plt.subplot(616)
	plt.plot(timeData,fileData['yawRate'])
	plt.title("Yaw Rate")
	plt.xlabel("Time")
	plt.ylabel("Yaw Rate(degree/sec)")
	plt.grid()


	for name in fileNames:
		filePointers[name].close()

	plt.show()


def AHRScallback(data):
	filePointers["roll"].write(str(data.Roll)+'\n')
	filePointers["pitch"].write(str(data.Pitch)+'\n')
	filePointers["yaw"].write(str(data.YawAngle)+'\n')
	filePointers["rollRate"].write(str(data.RollRate)+'\n')
	filePointers["pitchRate"].write(str(data.PitchRate)+'\n')
	filePointers["yawRate"].write(str(data.YawRate)+'\n')
	timeData.append(0 if len(timeData)==0 else (timeData[-1]+(1.0/Fs)))
# while not rospy.is_shutdown():
if __name__=="__main__":
	AHRSsub = rospy.Subscriber('/AHRS',AHRS,AHRScallback)
	rospy.spin()

print("Ended")
for name in fileNames:
	filePointers[name].close()

PlotAll()
# roll.close()
# pitch.close()
# yaw.close()
# rollRate.close()
# pitchRate.close()
# yawRate.close()