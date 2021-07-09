import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class GPSClient:
    def __init__(self, opcao):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        if opcao == "global_position":
            rospy.Subscriber(
                "/uav1/mavros/global_position/local",
                Odometry                            ,
                self.global_position_callback       )

        elif opcao == "ground_truth":
            rospy.Subscriber(
                "/uav1/ground_truth_pose"   ,
                PoseWithCovarianceStamped   ,
                self.ground_truth_callback  )

    def global_position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
    
    def ground_truth_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

    def posicao_atual(self):
        return self.x, self.y, self.z
