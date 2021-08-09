import rospy
import message_filters

from geometry_msgs.msg          import PoseWithCovarianceStamped
from sensor_msgs.msg            import Image
from algae_monitoring_msgs.msg  import Coordinate3D
from algae_monitoring_msgs.srv  import AlgaeDetector


class SampleClientBase:
    def path(self):
        pass


class SampleClient(SampleClientBase):

    def __init__(self, altitude):
        self.__altitude     = altitude
        self.__coordinate   = Coordinate3D(0, 0, 0)

        __image_sub = message_filters.Subscriber('/uav1/bluefox_optflow/image_raw', Image)
        __gps_sub   = message_filters.Subscriber('/uav1/ground_truth_pose', PoseWithCovarianceStamped)
        __ts        = message_filters.ApproximateTimeSynchronizer([__image_sub, __gps_sub], 10, 0.05)
        __ts.registerCallback(self.algae_detector_callback)
    
    def algae_detector_callback(self, image, coordinate):
        self.__image        = image
        self.__coordinate.x = coordinate.pose.pose.position.x
        self.__coordinate.y = coordinate.pose.pose.position.y
        self.__coordinate.z = coordinate.pose.pose.position.z

    def path(self):
        rospy.wait_for_service('algae_detector')

        try:
            detector_service = rospy.ServiceProxy('algae_detector', AlgaeDetector)
            resp = detector_service(self.__coordinate, self.__image)

        except rospy.ServiceException:
            rospy.loginfo("O serviço falhou em identificar algas")
            return []

        path = []

        for coordinate in resp.coordinates:
            path.append(Coordinate3D(coordinate.x, coordinate.y, self.__altitude))

        return path
