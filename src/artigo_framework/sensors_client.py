import rospy
import message_filters

from nav_msgs.msg           import Odometry
from geometry_msgs.msg      import PoseWithCovarianceStamped
from sensor_msgs.msg        import Image
from artigo_framework.srv   import CameraUAV

class Sensors:
    def __init__(self, opcao):
        self.x              = 0.0
        self.y              = 0.0
        self.z              = 0.0
        self.flock_coord    = []

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
        
        image_sub   = message_filters.Subscriber('/uav1/bluefox_optflow/image_raw', Image)
        GPS_sub     = message_filters.Subscriber('/uav1/mavros/global_position/local', Odometry)
        ts          = message_filters.ApproximateTimeSynchronizer([image_sub, GPS_sub], 10, 0.05)
        ts.registerCallback(self.callbackImg_coord)

    def global_position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
    
    def ground_truth_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
    
    def callbackImg_coord(self, img, coord):
        # print
        rospy.wait_for_service('algae_to_coord') ## node service name
        try:
            img_service = rospy.ServiceProxy('algae_to_coord', CameraUAV) ## variable next to the service
            resp = img_service(img, coord)
            output_coord = []
            for i in range(len(resp.x)):
                output_coord.append((resp.x[i], resp.y[i]))
                #print(resp.y[i])

            flock_coord = output_coord
            # print(flock_coord)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            flock_coord = []

    def posicao_atual(self):
        return self.x, self.y, self.z
    
    def path_planning_algas(self):
        return self.flock_coord
