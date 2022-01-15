from voronoi_draw.msg import CentralizedMsg
from tip.msg import UnicycleInfoMsg
from tip.msg import ControlMsg
import rospy

import CentralizedControllerBase

class ROSCentralizedController(CentralizedControllerBase):

    def __init__(self) -> None:
        super().__init__()
        self.controlInputPublisher = rospy.Publisher('/centralNode/controlInput', ControlMsg, queue_size=1)
        self.infoPublisher = rospy.Publisher('/centralNode/info', CentralizedMsg, queue_size=1)


    def publishControlMsg(self, ID, v,w):
        msg = ControlMsg()
        msg.ID = ID
        msg.translation = v
        msg.rotation = w		
        self.controlInputPublisher.publish(msg)
        