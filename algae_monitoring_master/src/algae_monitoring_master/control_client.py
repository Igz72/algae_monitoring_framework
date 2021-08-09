import rospy
import actionlib

from algae_monitoring_msgs.msg import UAVControlAction, UAVControlGoal


class ControlClientBase:
    def go_to(self):
        pass


class ControlClient(ControlClientBase):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("uav_control", UAVControlAction)
        self.client.wait_for_server()

    def go_to(self, coordinate):
        rospy.loginfo("Enviando o drone para x=%.1lf y=%.1lf z=%.1lf",
            coordinate.x,
            coordinate.y,
            coordinate.z)

        goal = UAVControlGoal(goal=coordinate)
        self.client.send_goal(goal, feedback_cb=self.uav_control_callback)
        self.client.wait_for_result()

        result = self.client.get_result()

        if result.success:
            rospy.loginfo("Destino alcançado")
        else:
            rospy.loginfo("Erro ao comandar o drone")

    def uav_control_callback(self, data):
            rospy.loginfo("Posição atual: x=%.1lf y=%.1lf z=%.1lf",
                data.position.x,
                data.position.y,
                data.position.z)
