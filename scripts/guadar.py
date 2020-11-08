from ger_drone.msg import Identifier
from ger_drone.srv import GetObject


tres = rospy.ServiceProxy('get_object')

req.identifier.state = Identifier.STATE_NOPROCESSADO
req.identifier.type = Identifier.TYPE_BASE
reponse = tres(req)