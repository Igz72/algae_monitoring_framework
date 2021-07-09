import rospy
from artigo_framework.controle import Controle

def main():
    rate = rospy.Rate(1)

    controle = Controle(
         115,   # Coordenada X do canto inferior esquerdo do mapa
         115,   # Coordenada Y do canto inferior esquerdo do mapa
          10,   # Largura do mapa
          10,   # Altura do mapa
           5,   # Largura da câmera
           5,   # Altura da câmera
           5,   # Altura da visão de coverage
           5,   # Altura da visão de monitoramento
           3)   # Margem de erro para alcançar os objetivos

    while not rospy.is_shutdown():
        controle.update_estado()
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("Master")

    try:
        main()
    except rospy.ROSInterruptException:
        pass
