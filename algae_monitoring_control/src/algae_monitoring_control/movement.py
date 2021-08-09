import rospy

from geometry_msgs.msg  import Point
from mrs_msgs.msg       import Reference
from mrs_msgs.srv       import ReferenceStampedSrv


class MovementBase:
    def go_to(self, goal):
        pass


class ControlManager(MovementBase):

    def __init__(self):
        rospy.wait_for_service('/uav1/control_manager/reference')

        self.control_manager = rospy.ServiceProxy('/uav1/control_manager/reference', ReferenceStampedSrv)

    def go_to(self, goal):
        point = Point(goal.x, goal.y, goal.z)

        # O heading é fixado em 1.57 para manter o drone sempre na mesma orientação (180°)
        reference = Reference(position=point, heading=1.57)

        return self.control_manager(reference=reference)
