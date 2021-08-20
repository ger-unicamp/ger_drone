import rospy
from mrs_msgs.srv import ReferenceStampedSrv
from std_srvs.srv import Trigger
from mrs_msgs.msg import PositionCommand, ControlManagerDiagnostics, UavState
import numpy as np

posicao_atual = [0,0,0]

    
def callbackEsperarChegada(msg):
    info = msg.data
    print(info)
    rospy.loginfo("I received a message!")
    global posicao_atual
    posicao_atual = [info.position.x, info.position.y, info.position.z]

def pegarPosicaoAtual():
    rospy.Subscriber('/uav1/control_manager/position_cmd',PositionCommand,callbackEsperarChegada)
    return posicao_atual
"""
def callback_function(msg):
    ponto_atual = msg.data
    rospy.loginfo("I received a message!")
    
def run_loop():
    rate = rospy.Rate(10.0) #Run at 10Hz
    while not rospy.is_shutdown():
        self.rate.sleep()
        #Other periodic work you may need to do

if __name__ == '__main__':
    rospy.init_node('my_node')
    rospy.Subscriber('/your_topic_name',Bool,callback_function)
    run_loop()
"""

class Drone:
    def __init__(self):
        self.posicao_atual = [0,0,0]
        pass

    def voarPara(self, coordenadas):
        rospy.wait_for_service('/uav1/control_manager/reference')
        voar = rospy.ServiceProxy('/uav1/control_manager/reference', ReferenceStampedSrv)
        req = ReferenceStampedSrv._request_class()
        req.reference.position.x = coordenadas[0]
        req.reference.position.y = coordenadas[1]
        req.reference.position.z = coordenadas[2]
        voar(req)
        
        self.esperarChegada(coordenadas)

    def decolar(self):
        rospy.wait_for_service('/uav1/uav_manager/takeoff')
        decolar =  rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
        decolar()

    def pousar(self):
        rospy.wait_for_service('/uav1/uav_manager/land')
        pousar = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
        pousar()

    def getPosicao(self):
        print("antes")
        ponto_atual = rospy.wait_for_message('/uav1/control_manager/position_cmd',PositionCommand)
        print("depois")
        rospy.spin()
        return [ ponto_atual.position.x,
                 ponto_atual.position.y,
                 ponto_atual.position.z]

    def rotina(self):
        self.pousar()
        rospy.sleep(2)
        self.decolar()
        rospy.sleep(2)

    def voarParaPreciso(self, coordenadas): #testar
        rospy.wait_for_service('/uav1/control_manager/switch_controller')
        proxy = rospy.ServiceProxy('/uav1/control_manager/switch_controller', String)
        req = String._request_class()
        
        req.value = "Se3Controller"
        proxy(req)
        rospy.sleep(1)

        self.voarPara(coordenadas)
        
        rospy.sleep(3)
        req,value = "MpcController"
        proxy(req)
        rospy.sleep(1)

    def retornaInicio(self):
        inicio = [10,90,8]
        self.voarPara(inicio)
      #  self.voarParaPreciso(inicio)
        rospy.sleep(2) 
        self.pousar()
        

    
    def comparar(self,ponto_desejado ):
        #t_desejado = np.array([ponto_desejado[0],ponto_desejado[1], ponto_desejado[2]], np.float32)
        #t_atual = np.array([self.posicao_atual[0], self.posicao_atual[1], self.posicao_atual[2]], np.float32)
        t_desejado = np.array([ponto_desejado[0],ponto_desejado[1]], np.float32)
        t_atual = np.array([self.posicao_atual[0], self.posicao_atual[1]], np.float32)
        #print(f"t -  desejado: {t_desejado}")
        #print(f"t -  atual: {t_atual}")
        if (np.linalg.norm(t_desejado-t_atual)<9):
            print('pronto')
            return False
        
        return True

    def esperarChegada(self, ponto_desejado):
        continuar = True
        while(continuar):
            print("a")
            self.posicao_atual = pegarPosicaoAtual()
            #rospy.wait_for_message('/uav1/control_manager/position_cmd',PositionCommand)
            
            continuar = self.comparar(ponto_desejado)
        rospy.sleep(5)


    
