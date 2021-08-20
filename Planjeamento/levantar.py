import rospy
from std_srvs.srv import Trigger


rospy.wait_for_service('/uav1/uav_manager/takeoff')
levantar = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
levantar()