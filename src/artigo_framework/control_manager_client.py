import rospy

from geometry_msgs.msg  import Point
from mrs_msgs.msg       import Reference
from mrs_msgs.srv       import ReferenceStampedSrv

def control_manager_client(x, y, z):

    rospy.wait_for_service('/uav1/control_manager/reference')

    try:
        control_manager = rospy.ServiceProxy('/uav1/control_manager/reference', ReferenceStampedSrv)

        ponto = Point(x, y, z)

        # O heading é fixado em 1.57 para manter o drone sempre na mesma orientação (180°)
        objetivo = Reference(position=ponto, heading=1.57)

        resposta = control_manager(reference=objetivo)

        if resposta.success:
            rospy.loginfo("Enviando o drone para x=", x, " y=", y, " z=", z)
            return True
        else:
            rospy.loginfo("Erro ao comandar o drone")
            return False

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
