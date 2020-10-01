#!/usr/bin/env python


import rospy
from mrs_msgs.msg import PositionCommand
from geometry_msgs.msg import Point

rospy.init_node('coordenadas')

def callback(msg):
    print(msg.position)
  
   
    
if __name__ == '__main__':
    try:
        abc = rospy.Subscriber('/uav1/control_manager/position_cmd', PositionCommand,callback)
        
        rospy.spin()

    except rospy.ROSInternalException:
        pass    