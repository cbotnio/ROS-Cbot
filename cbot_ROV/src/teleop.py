import rospy
from cbot_ros_msgs.srv import ThrusterControl
import sys, select, termios, tty

rospy.init_node("ROV_Mode")

comm_mode_F = 0
diff_mode_F = 0
comm_mode_V = 0
diff_mode_V = 0

EmergencyStop = 'g'

keyMap = {'i':(1,0,0,0),
		  'j':(0,-1,0,0),
		  'k':(-1,0,0,0),
		  'l':(0,1,0,0),
		  'w':(0,0,1,0),
		  'a':(0,0,0,1),
		  's':(0,0,-1,0),
		  'd':(0,0,0,-1)}

comm_mode_sat_F = 60
diff_mode_sat_F = 30
comm_mode_sat_V = 60
diff_mode_sat_V = 30

eps = 0.0000001

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

while not rospy.is_shutdown():
	r = rospy.Rate(10)
	settings = termios.tcgetattr(sys.stdin)
	ROV_status = rospy.get_param('ROV_mode')
	thrustInputs = rospy.ServiceProxy("thruster_control", ThrusterControl)
	rospy.wait_for_service('thruster_control')
	#thrustInputs = ThrusterControl.Request()

	if(ROV_status):
		# rospy.set_param('Controller_ON', 0)
		# rospy.set_param('AUV_mode', 0)
		try:
			key = getKey()
			if(key==EmergencyStop):
				comm_mode_F = 0
				diff_mode_F = 0
				comm_mode_V = 0
				diff_mode_V = 0
			elif key in keyMap:
				Thrusts = keyMap[key]
				comm_mode_F = ((comm_mode_F+Thrusts[0]+eps)/abs(comm_mode_F+Thrusts[0]+eps))*min(abs(comm_mode_F+Thrusts[0]), comm_mode_sat_F)
				diff_mode_F = ((diff_mode_F+Thrusts[1]+eps)/abs(diff_mode_F+Thrusts[1]+eps))*min(abs(diff_mode_F+Thrusts[1]), diff_mode_sat_F)
				comm_mode_V = ((comm_mode_V+Thrusts[2]+eps)/abs(comm_mode_V+Thrusts[2]+eps))*min(abs(comm_mode_V+Thrusts[2]), comm_mode_sat_V)
				diff_mode_V = ((diff_mode_V+Thrusts[3]+eps)/abs(diff_mode_V+Thrusts[3]+eps))*min(abs(diff_mode_V+Thrusts[3]), diff_mode_sat_V)

			print("CMF: " + str(comm_mode_F) + "  DMF: " + str(diff_mode_F) + "  CMV: " + str(comm_mode_V) + "  DMV: " + str(diff_mode_V))
			resp = thrustInputs(comm_mode_F,diff_mode_F,comm_mode_V,diff_mode_V,1) 

		except Exception as e:
			print("Service Failed: " + str(e))
			sys.exit()
	# rospy.spin()
	r.sleep()