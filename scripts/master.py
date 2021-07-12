import rospy

from artigo_framework.controle import Controle

def main():
    rate = rospy.Rate(1)

    controle = Controle(
        -125,   # Coordenada X do canto inferior esquerdo do mapa
         140,   # Coordenada Y do canto inferior esquerdo do mapa
         250,   # Largura do mapa
         310,   # Altura do mapa
          50,   # Altura da visão de coverage
          10,   # Altura da visão de monitoramento
           2)   # Margem de erro para alcançar os objetivos

    while not rospy.is_shutdown():
        controle.update_estado()
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Master")

    try:
        main()
    except rospy.ROSInterruptException:
        pass
