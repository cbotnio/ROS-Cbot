import rospy
import simplekml
from cbot_ros_msgs.msg import GPS

count = 0
kml = simplekml.Kml()

rospy.init_node("KMLGenerator")

def gpsCallback(data):
	global count, kml
	kml.newpoint(name=str(count),coords=[(data.longitude,data.latitude)])
	count+=1

r = rospy.Rate(0.5)
sub = rospy.Subscriber('/GPS',GPS,gpsCallback)
try:
	while not rospy.is_shutdown():
		r.sleep()
except:
	kml.save('trajectory.kml')