import rospy

from algae_monitoring_msgs.msg import Coordinate3D, Rectangle
from algae_monitoring_msgs.srv import Coverage


class CoverageClientBase:
    def path(self):
        pass


class GridCoverageClient(CoverageClientBase):
    def __init__(self, start, area, altitude):
        self.__start      = start
        self.__area       = area
        self.__altitude   = altitude

        self.__camera     = self.camera_dimension()

    def camera_dimension(self):
        K = [215.6810060961547, 0.0, 376.5, 0.0, 215.6810060961547, 240.5, 0.0, 0.0, 1.0]
        land_height = 0.000009 # 0.5
        Z = self.__altitude - land_height # Distancia do solo
        dX = (0 - K[2]) * Z / K[0]
        dY = (0 - K[5]) * Z / K[4]
        width   = 2 * (0 - dX)
        height  = 2 * (0 - dY)

        return Rectangle(width, height)

    def path(self):
        rospy.wait_for_service('coverage')

        try:
            coverage = rospy.ServiceProxy('coverage', Coverage)

            response = coverage(
                self.__start,
                self.__area,
                self.__camera)
            
            path = []

            for coordinate in response.path:
                path.append(Coordinate3D(coordinate.x, coordinate.y, self.__altitude))

            return path

        except rospy.ServiceException as e:
            rospy.loginfo("Erro na comunicação com o service: %s", e)
