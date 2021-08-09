import rospy

from cv_bridge                          import CvBridge
from algae_monitoring_msgs.msg          import Coordinate2D
from algae_monitoring_msgs.srv          import AlgaeDetector, AlgaeDetectorResponse
from algae_monitoring_detector.detector import stationary_camera_transform, algae_detector


def detector_callback(req):
    uav_x       = req.position.x
    uav_y       = req.position.y
    uav_z       = req.position.z
    image_ros   = req.image
    bridge      = CvBridge()
    image       = bridge.imgmsg_to_cv2(image_ros, desired_encoding='passthrough')
    classified  = algae_detector(image)
    coordinates = []

    for i in classified:
        p = stationary_camera_transform((i[0], 480-i[1]), uav_z)
        algae_coord = Coordinate2D(round(uav_x+p[0],1), round(uav_y+p[1], 1))
        coordinates.append(algae_coord)

    return AlgaeDetectorResponse(coordinates)

def algae_detector_server():
    rospy.init_node("algae_detector_server")
    
    try:
        rospy.Service("algae_detector", AlgaeDetector, detector_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
