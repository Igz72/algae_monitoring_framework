"""
Adaptado de Grid based sweep planner, de Atsushi Sakai
GitHub: https://github.com/AtsushiSakai/PythonRobotics
"""

import matplotlib.pyplot as plt

from artigo_framework.path_planning import Ponto, Mapa, Camera, PathPlanner

def animacao(
    mapa_inicio_x   ,
    mapa_inicio_y   ,
    mapa_largura    ,
    mapa_altura     ,
    camera_largura  ,
    camera_altura   ):

    mapa_inicio = Ponto(mapa_inicio_x, mapa_inicio_y)
    mapa        = Mapa(mapa_inicio, mapa_largura, mapa_altura)
    camera      = Camera(camera_largura, camera_altura)
    path        = PathPlanner(mapa, camera)

    mapa_x, mapa_y          = mapa.coordenadas()
    caminho                 = path.caminho()
    caminho_x, caminho_y    = path.coordenadas()

    for ponto in caminho:
        camera_x, camera_y = camera.coordenadas(ponto)

        plt.cla()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(mapa_x, mapa_y, "-xb")
        plt.plot(caminho_x, caminho_y, "-r")
        plt.plot(ponto.x, ponto.y, "or")
        plt.plot(camera_x, camera_y, "r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.4)

    plt.close()

if __name__ == '__main__':
    animacao(0.0, 0.0, 100.0, 50.0, 10.0, 10.0)
