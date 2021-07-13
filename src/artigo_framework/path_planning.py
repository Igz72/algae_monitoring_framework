import math
import numpy as np

class Ponto:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Mapa:
    def __init__(self, inicio, largura, altura):
        self.inicio = inicio
        self.largura = largura
        self.altura = altura
    
    def coordenadas(self):
        esquerda_x = self.inicio.x
        direita_x = self.inicio.x + self.largura
        inferior_y = self.inicio.y
        superior_y = self.inicio.y + self.altura

        coordenadas_x = [esquerda_x, direita_x, direita_x, esquerda_x, esquerda_x]
        coordenadas_y = [inferior_y, inferior_y, superior_y, superior_y, inferior_y]

        return coordenadas_x, coordenadas_y

class Camera:
    def __init__(self, largura, altura):
        self.largura = largura
        self.altura = altura
    
    def coordenadas(self, centro):
        esquerda_x = centro.x - self.largura / 2
        direita_x = centro.x + self.largura / 2
        inferior_y = centro.y - self.altura / 2
        superior_y = centro.y + self.altura / 2

        coordenadas_x = [esquerda_x, direita_x, direita_x, esquerda_x, esquerda_x]
        coordenadas_y = [inferior_y, inferior_y, superior_y, superior_y, inferior_y]

        return coordenadas_x, coordenadas_y

class PathPlanner:
    def __init__(self, mapa, camera):
        self.mapa = mapa
        self.camera = camera
        self.lista = []

    def montarMosaico(self):
        inverter = False
        coluna = []

        for y in np.arange(0.0, self.mapa.altura, self.camera.altura):
            linha = []

            for x in np.arange(0.0, self.mapa.largura, self.camera.largura):
                ponto_x = self.mapa.inicio.x + x
                ponto_y = self.mapa.inicio.y + y
                ponto = Ponto(ponto_x, ponto_y)
                linha.append(ponto)
            
            if inverter:
                linha.reverse()

            inverter = not inverter
            coluna += linha

        self.lista = coluna

    def ajustarCentro(self):
        deslocamento_x = self.camera.largura / 2
        deslocamento_y = self.camera.altura / 2
        caminho_ajustado = []

        for ponto in self.lista:
            centro_x = ponto.x + deslocamento_x
            centro_y = ponto.y + deslocamento_y
            centro = Ponto(centro_x, centro_y)
            caminho_ajustado.append(centro)
            
        self.lista = caminho_ajustado
    
    def artigo(self):
        self.lista = []

        N = math.ceil(self.mapa.largura / self.camera.largura)
        M = math.ceil(self.mapa.altura / self.camera.altura)

        for j in range(M):
            for i in range(N):

                if j % 2 != 0:
                    i = N - 1 - i
                
                x = self.mapa.inicio.x + self.camera.largura  * (i + 0.5)
                y = self.mapa.inicio.y + self.camera.altura   * (j + 0.5)

                self.lista.append(Ponto(x, y))

    def caminho(self):
        # self.montarMosaico()
        # self.ajustarCentro()
        self.artigo()
        return self.lista

    def coordenadas(self):
        lista_coordenadas = self.caminho()
        coordenadas_x = []
        coordenadas_y = []
        
        for ponto in lista_coordenadas:
            coordenadas_x.append(ponto.x)
            coordenadas_y.append(ponto.y)

        return coordenadas_x, coordenadas_y

if __name__ == '__main__':
    mapa = Mapa(Ponto(0, 0), 100, 50)
    camera = Camera(10, 10)
    path = PathPlanner(mapa, camera)
    caminho = path.caminho()
    coordenadas_x, coordenadas_y = path.coordenadas()
    
    # print(coordenadas_x)
    # print(coordenadas_y)
