import rospy

from algae_monitoring_msgs.msg                  import Coordinate2D, Rectangle
from algae_monitoring_master.coverage_client    import GridCoverageClient
from algae_monitoring_master.sample_client      import SampleClient
from algae_monitoring_master.control_client     import ControlClient
from algae_monitoring_master.manager            import Manager


def master_node():
    rospy.init_node("master_node")

    try:
        rate                = rospy.Rate(1)

        start               = Coordinate2D(-125, 140)
        area                = Rectangle(250, 310)
        coverage_altitude   = 50
        sample_altitude     = 10

        rospy.loginfo("Inicializando o cliente do coverage")
        coverage_client     = GridCoverageClient(start, area, coverage_altitude)

        rospy.loginfo("Inicializando o cliente do detector")
        sample_client       = SampleClient(sample_altitude)

        rospy.loginfo("Inicializando o cliente do controle")
        control_client      = ControlClient()

        rospy.loginfo("Inicializando o gerenciador")
        manager = Manager(
            coverage_client = coverage_client,
            sample_client   = sample_client,
            control_client  = control_client)

        while not rospy.is_shutdown():
            manager.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
