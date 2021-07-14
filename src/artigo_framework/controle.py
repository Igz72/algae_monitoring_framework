import rospy

from artigo_framework.path_planning_client      import path_planning_client
from artigo_framework.control_manager_client    import control_manager_client
from artigo_framework.sensors_client            import Sensors

class Controle:
    def __init__(
        self            ,
        mapa_inicio_x   ,
        mapa_inicio_y   ,
        mapa_largura    ,
        mapa_altura     ,
        altura_coverage ,
        altura_foto     ,
        precisao        ):

        self.mapa_inicio_x      = mapa_inicio_x                             # Coordenada X do canto inferior esquerdo do mapa
        self.mapa_inicio_y      = mapa_inicio_y                             # Coordenada Y do canto inferior esquerdo do mapa
        self.mapa_largura       = mapa_largura                              # Largura do mapa
        self.mapa_altura        = mapa_altura                               # Altura do mapa
        self.altura_coverage    = altura_coverage                           # Altura da visão no coverage
        self.altura_foto        = altura_foto                               # Altura da visão nas fotos das algas
        self.precisao           = precisao                                  # Margem de erro para alcançar os objetivos 
        self.coverage_x         = []                                        # Coordenadas X do covarege
        self.coverage_y         = []                                        # Coordenadas Y do covarege
        self.coverage_posicao   = 0                                         # Posição atual no covarege
        self.estado             = 0                                         # Estado atual do controlador
        self.sensors            = Sensors("ground_truth", altura_coverage)  # Contém informações dos sensores
        self.percorrido_x       = []                                        # Coordenada X do caminho percorrido pelo drone
        self.percorrido_y       = []                                        # Coordenada Y do caminho percorrido pelo drone

        rospy.loginfo(
            "Dimensão das imagens capturadas pela câmera: %.1lf m X %.1lf m",
            self.sensors.camera_largura()                                   ,
            self.sensors.camera_altura()                                    )

    def path_planning_coverage(self):
        # Requisição do caminho para o path_planning_server
        self.coverage_x, self.coverage_y = path_planning_client(
            self.mapa_inicio_x              ,
            self.mapa_inicio_y              ,
            self.mapa_largura               ,
            self.mapa_altura                ,
            self.sensors.camera_largura()   ,
            self.sensors.camera_altura()    )

        caminho_x_comprimento = len(self.coverage_x)
        caminho_y_comprimento = len(self.coverage_y)

        if caminho_x_comprimento == caminho_y_comprimento > 0:
            for i in range(caminho_x_comprimento):
                rospy.loginfo("Coordenada %d: %6.1lf %6.1lf %6.1lf",
                    i+1, self.coverage_x[i], self.coverage_y[i], self.altura_coverage)
            self.coverage_posicao = -1 # A próximo coordenada será a inicial
            return True
        else:
            return False

    def mover_para_proxima_posicao_coverage(self):
        if self.coverage_terminou():
            return False
        else:
            x = self.coverage_x[self.coverage_posicao]
            y = self.coverage_y[self.coverage_posicao]
            z = self.altura_coverage

            # Emitir ordem de movimento para o control_manager
            return control_manager_client(x, y, z)

    def destino_alcancado_coverage(self):
        objetivo_x = self.coverage_x[self.coverage_posicao]
        objetivo_y = self.coverage_y[self.coverage_posicao]
        objetivo_z = self.altura_coverage

        # Obter a posição atual
        atual_x, atual_y, atual_z = self.sensors.posicao_atual()

        alcancou_x = objetivo_x - self.precisao < atual_x < objetivo_x + self.precisao
        alcancou_y = objetivo_y - self.precisao < atual_y < objetivo_y + self.precisao
        alcancou_z = objetivo_z - self.precisao < atual_z < objetivo_z + self.precisao
        
        if alcancou_x and alcancou_y and alcancou_z:
            return True
        else:
            return False

    def coverage_terminou(self):
        if self.coverage_posicao >= len(self.coverage_x):
            return True
        else:
            return False

    def path_planning_algas(self):
         # Requisição do caminho para o algae_coord_service
        algas_x, algas_y = self.sensors.path_planning_algas()

        caminho_x_comprimento = len(algas_x)
        caminho_y_comprimento = len(algas_y)

        if caminho_x_comprimento == caminho_y_comprimento > 0:
            for i in range(caminho_x_comprimento):
                rospy.loginfo("Alga %d: %6.1lf %6.1lf %6.1lf",
                    i+1, algas_x[i], algas_y[i], self.altura_foto)
            return True
        elif caminho_x_comprimento == caminho_y_comprimento:
            rospy.loginfo("Algas não foram identificadas")
            return True
        else:
            return False

    def update_estado(self):

        if self.estado == 0: # Solicitar o caminho para o coverage
            rospy.loginfo("Solicitando o caminho para o coverage")
            sucesso = self.path_planning_coverage()
            if sucesso:
                self.estado = 1
            else:
                rospy.loginfo("Erro ao obter o caminho para o coverage")

        elif self.estado == 1: # Tentar avançar no coverage
            self.coverage_posicao += 1
            if not self.coverage_terminou():
                rospy.loginfo("Avançando para a posição %d do coverage", self.coverage_posicao+1)
                self.estado = 2
            else:
                rospy.loginfo("Todas as coordenadas do coverage foram visitadas")
                self.estado = 5

        elif self.estado == 2: # Emitir ordem de movimento para a próxima posição no coverage
            sucesso = self.mover_para_proxima_posicao_coverage()
            if sucesso:
                self.estado = 3

        elif self.estado == 3: # Aguardar o destino
            if not self.destino_alcancado_coverage():
                rospy.loginfo("Aguardando destino")
            else:
                rospy.loginfo("Destino alcançado")
                self.estado = 4
        
        elif self.estado == 4: # Solicitar o caminho para fotografar as algas
            rospy.loginfo("Solicitando o caminho para fotografar as algas")
            sucesso = self.path_planning_algas()
            if sucesso:
                self.estado = 1

        elif self.estado == 5:
            rospy.loginfo("Fim da execução")
            self.estado = 6

        else:
            pass
