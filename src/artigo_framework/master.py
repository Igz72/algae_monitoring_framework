#!/usr/bin/python3
import rospy
from artigo_framework.controle import Controle

def master_node():
    rospy.init_node("master_node")

    rate = rospy.Rate(1)

    controle = Controle(
        -125,   # Coordenada X do canto inferior esquerdo do mapa
         140,   # Coordenada Y do canto inferior esquerdo do mapa
         250,   # Largura do mapa
         310,   # Altura do mapa
          50,   # Altura da visão de coverage
          10,   # Altura da visão de monitoramento
           4)   # Margem de erro para alcançar os objetivos

    try:
        while not rospy.is_shutdown():
            controle.update_estado()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
