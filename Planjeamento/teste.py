

from Drone import Drone
import rospy
from mrs_msgs.msg import UavState

"""
def descobrirPosicao():

        rospy.init_node("Tobias")
        print("aaaaaaaaaaaaaaaaaaaaaa")
        global coordenadas
        sub = rospy.Subscriber('/uav1/odometry/uav_state', UavState, callback)
        return coordenadas

def callback(info):
    pos = info.pose
    coordenadas = [pos.position.x, pos.position.y, pos.position.z]
    
"""

drone = Drone()

drone.voarPara([5,3,3])
drone.voarPara([15,3,3])
#drone.retornaInicio()
print("chegou")