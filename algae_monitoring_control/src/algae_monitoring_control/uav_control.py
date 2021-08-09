import rospy
import actionlib

from algae_monitoring_msgs.msg import UAVControlAction, UAVControlFeedback, UAVControlResult


class UAVControl:

    def __init__(self, name, precision, gps, uav_movement):
        self.__name         = name
        self.__precision    = precision
        self.__gps          = gps
        self.__uav_movement = uav_movement

        self.__action       = actionlib.SimpleActionServer(
            self.__name,
            UAVControlAction,
            execute_cb=self.goal_callback,
            auto_start=False)

        self.__feedback     = UAVControlFeedback()
        self.__result       = UAVControlResult()

        self.__action.start()

    def goal_callback(self, data):
        rate = rospy.Rate(1)

        success = self.__uav_movement.go_to(data.goal)

        if success:
            while not self.done(data.goal):

                if self.__action.is_preempt_requested(): # Ação cancelada
                    self.__action.set_preempted()
                    success = False
                    break

                self.__feedback.position = self.__gps.position()
                self.__action.publish_feedback(self.__feedback)

                rate.sleep()

        if success:
            self.__result.success = True
        else:
            self.__result.success = False

        self.__action.set_succeeded(self.__result)

    def done(self, goal):
        if (goal.x - self.__precision < self.__gps.x() < goal.x + self.__precision and
            goal.y - self.__precision < self.__gps.y() < goal.y + self.__precision and
            goal.z - self.__precision < self.__gps.z() < goal.z + self.__precision):
            return True

        else:
            return False
