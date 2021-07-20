import rospy
import message_filters

from nav_msgs.msg           import Odometry
from geometry_msgs.msg      import PoseWithCovarianceStamped
from sensor_msgs.msg        import Image
from artigo_framework.srv   import CameraUAV

class Sensors:
    def __init__(self, gps_topic, altura_camera):
        self.x              = 0.0
        self.y              = 0.0
        self.z              = 0.0
        self.imagem         = Image()
        self.coordenada_x   = 0.0
        self.coordenada_y   = 0.0
        self.coordenada_z   = 0.0
        self.visao_largura  = 0.0
        self.visao_altura   = 0.0

        self.calcular_visao_camera(altura_camera)

        if gps_topic == "global_position":
            rospy.Subscriber("/uav1/mavros/global_position/local", Odometry, self.global_position_callback)
        elif gps_topic == "ground_truth":
            rospy.Subscriber("/uav1/ground_truth_pose", PoseWithCovarianceStamped, self.ground_truth_callback)

        image_sub   = message_filters.Subscriber('/uav1/bluefox_optflow/image_raw', Image)
        GPS_sub     = message_filters.Subscriber('/uav1/ground_truth_pose', PoseWithCovarianceStamped)
        ts          = message_filters.ApproximateTimeSynchronizer([image_sub, GPS_sub], 10, 0.05)
        ts.registerCallback(self.img_coord_callback)

    def global_position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
    
    def ground_truth_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
    
    def img_coord_callback(self, img, coord):
        self.imagem = img
        self.coordenada_x = coord.pose.pose.position.x
        self.coordenada_y = coord.pose.pose.position.y
        self.coordenada_z = coord.pose.pose.position.z

    def calcular_visao_camera(self, altura_camera):
        K = [215.6810060961547, 0.0, 376.5, 0.0, 215.6810060961547, 240.5, 0.0, 0.0, 1.0]
        baseTerrestreAltura = 0.000009 # 0.5
        Z = altura_camera - baseTerrestreAltura # Distancia do solo
        dX = (0 - K[2]) * Z / K[0]
        dY = (0 - K[5]) * Z / K[4]
        self.visao_largura  = 2 * (0 - dX)
        self.visao_altura   = 2 * (0 - dY)

    def camera_largura(self):
        return self.visao_largura

    def camera_altura(self):
        return self.visao_altura

    def posicao_atual(self):
        return self.x, self.y, self.z
    
    def path_planning_algas(self):
        rospy.wait_for_service('algae_to_coord')
        try:
            img_service = rospy.ServiceProxy('algae_to_coord', CameraUAV)
            resp = img_service(self.imagem, self.coordenada_x, self.coordenada_y, self.coordenada_z)

        except rospy.ServiceException as e:
            rospy.loginfo("O serviço falhou em identificar algas")
            return [], []

        return resp.x, resp.y
