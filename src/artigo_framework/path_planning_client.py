import rospy

from artigo_framework.srv import PathPlanning

def path_planning_client(
    mapa_inicio_x   ,
    mapa_inicio_y   ,
    mapa_largura    ,
    mapa_altura     ,
    camera_largura  ,
    camera_altura   ):

    rospy.wait_for_service('coverage')

    try:
        path_planning = rospy.ServiceProxy('coverage', PathPlanning)
        resposta = path_planning(
            mapa_inicio_x   ,
            mapa_inicio_y   ,
            mapa_largura    ,
            mapa_altura     ,
            camera_largura  ,
            camera_altura   )
        return resposta.caminho_x, resposta.caminho_y

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
