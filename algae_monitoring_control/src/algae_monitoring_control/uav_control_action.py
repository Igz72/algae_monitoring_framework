import rospy

from algae_monitoring_control.gps           import GroundTruthPose
from algae_monitoring_control.movement      import ControlManager
from algae_monitoring_control.uav_control   import UAVControl


def uav_control_server():
    rospy.init_node("uav_control_server")

    try:
        server = UAVControl(
            name="uav_control",
            precision=4,
            gps=GroundTruthPose(),
            uav_movement=ControlManager())

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
