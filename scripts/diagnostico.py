#!/usr/bin/env python

import rospy
from mrs_msgs.msg import ConstraintManagerDiagnostics

"""!
Se inscreve no topico que fornece dados de velocidade e heading momentâneos do drone
"""

def callback(mensagem):
    print(mensagem.current_values)
    


if __name__ == '__main__':
    try:
        
        rospy.init_node('info')
           
        abc = rospy.Subscriber('/uav1/constraint_manager/diagnostics',ConstraintManagerDiagnostics, callback )
        rospy.spin()  
        

    except rospy.ROSInternalException:
        pass    
      