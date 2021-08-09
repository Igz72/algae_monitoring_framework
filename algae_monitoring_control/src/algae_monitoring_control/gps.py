import rospy

from geometry_msgs.msg          import PoseWithCovarianceStamped
from algae_monitoring_msgs.msg  import Coordinate3D


class GPSBase:
    def position(self):
        pass
    def x(self):
        pass
    def y(self):
        pass
    def z(self):
        pass


class GroundTruthPose(GPSBase):

    def __init__(self):
        self.__position = Coordinate3D(0, 0, 0)

        rospy.Subscriber("/uav1/ground_truth_pose", PoseWithCovarianceStamped, self.gps_callback)

    def gps_callback(self, data):
        self.__position.x = data.pose.pose.position.x
        self.__position.y = data.pose.pose.position.y
        self.__position.z = data.pose.pose.position.z

    def position(self):
        return self.__position

    def x(self):
        return self.__position.x

    def y(self):
        return self.__position.y

    def z(self):
        return self.__position.z
