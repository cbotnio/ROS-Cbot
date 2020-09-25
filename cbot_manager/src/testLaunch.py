import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mohit/nio/src/CBOT/cbot_description/launch/upload_cbot.launch"])
launch.start()
rospy.loginfo("started")

# rospy.sleep(10)
launch.spin()

print("Strating node 2")
rospy.sleep(5)
launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mohit/nio/src/ROS-Cbot-master/cbot_actuators/launch/cbot_actuators.launch"])
launch2.start()
# 3 seconds later
launch2.spin()
# launch.shutdown()
