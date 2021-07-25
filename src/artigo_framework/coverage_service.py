import rospy

from artigo_framework.srv           import PathPlanning, PathPlanningResponse
from artigo_framework.path_planning import Ponto, Camera, Mapa, PathPlanner

def handle_coverage(requisicao):
    mapa_inicio             = Ponto(requisicao.mapa_inicio_x, requisicao.mapa_inicio_y)
    mapa_largura            = requisicao.mapa_largura
    mapa_altura             = requisicao.mapa_altura
    camera_largura          = requisicao.camera_largura
    camera_altura           = requisicao.camera_altura

    mapa                    = Mapa(mapa_inicio, mapa_largura, mapa_altura)
    camera                  = Camera(camera_largura, camera_altura)
    caminho                 = PathPlanner(mapa, camera)
    caminho_x, caminho_y    = caminho.coordenadas()

    return PathPlanningResponse(caminho_x, caminho_y)

def coverage_server():
    rospy.init_node('coverage_server')
    s = rospy.Service('coverage', PathPlanning, handle_coverage)
    rospy.spin()
