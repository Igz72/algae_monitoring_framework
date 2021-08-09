import rospy

from algae_monitoring_msgs.srv                      import Coverage, CoverageResponse
from algae_monitoring_coverage.coverage_algorithm   import coverage_algorithm


def coverage_callback(req):

    path = coverage_algorithm(req.start, req.area, req.camera)

    return CoverageResponse(path)

def coverage_server():
    rospy.init_node("coverage_server")

    try:
        rospy.Service("coverage", Coverage, coverage_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
