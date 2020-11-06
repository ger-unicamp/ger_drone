#!/usr/bin/env python


import rospy

from std_srvs.srv import Trigger 
from mrs_msgs.srv import ReferenceSrv

# uav_manager/takeoff	take off	std_srvs/Trigger

#control_manager/reference	fly to given coordinates in a given frame	mrs_msgs/ReferenceSrv

#geometry_msg/Pose

if __name__ == '__main__':
    try:

        rospy.init_node('fase1')

        rospy.wait_for_service('uav_manager/takeoff')

        req = Trigger._request_class()

    
        srv = rospy.ServiceProxy('uav_manager/takeoff', Trigger)
        
        resp = srv(req)

        while(resp.sucess == False):
            resp = srv(req)  
        

        #Inscrição para publicar/receber aqui!

        rate = rospy.Rate(10) #Define a frequência de execução em Hz

        while not rospy.is_shutdown():
            
            req = ReferenceSrv._request_class()

            req.reference.position.x = 10
            req.reference.position.y = 10
            req.reference.position.z = 10
            req.reference.heading = 0

            proxy = rospy.ServiceProxy('/uav1/control_manager/reference',ReferenceSrv)
            resp = proxy(req)


            rate.sleep() #Espera o tempo para executar o programa na frequência definida


    except rospy.ROSInternalException:
        pass