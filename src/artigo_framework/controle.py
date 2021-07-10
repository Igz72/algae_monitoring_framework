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
        camera_largura  ,
        camera_altura   ,
        altura_coverage ,
        altura_foto     ,
        precisao        ):

        self.mapa_inicio_x      = mapa_inicio_x             # Coordenada X do canto inferior esquerdo do mapa
        self.mapa_inicio_y      = mapa_inicio_y             # Coordenada Y do canto inferior esquerdo do mapa
        self.mapa_largura       = mapa_largura              # Largura do mapa
        self.mapa_altura        = mapa_altura               # Altura do mapa
        self.camera_largura     = camera_largura            # Largura da câmera
        self.camera_altura      = camera_altura             # Altura da câmera
        self.altura_coverage    = altura_coverage           # Altura da visão no coverage
        self.altura_foto        = altura_foto               # Altura da visão nas fotos das algas
        self.precisao           = precisao                  # Margem de erro para alcançar os objetivos 
        self.coverage_x         = []                        # Coordenadas X do covarege
        self.coverage_y         = []                        # Coordenadas Y do covarege
        self.coverage_posicao   = 0                         # Posição atual no covarege
        self.algas_x            = []                        # Coordenadas X das algas
        self.algas_y            = []                        # Coordenadas Y das algas
        self.algas_posicao      = 0                         # Posição da alga atual
        self.estado             = 0                         # Estado atual do controlador
        self.sensor             = Sensors("ground_truth")   # Contém informações dos sensores

    def path_planning_coverage(self):
        # Requisição do caminho para o path_planning_server
        self.coverage_x, self.coverage_y = path_planning_client(
            self.mapa_inicio_x   ,
            self.mapa_inicio_y   ,
            self.mapa_largura    ,
            self.mapa_altura     ,
            self.camera_largura  ,
            self.camera_altura   )

        caminho_x_comprimento = len(self.coverage_x)
        caminho_y_comprimento = len(self.coverage_y)

        if caminho_x_comprimento == caminho_y_comprimento > 0:
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
        atual_x, atual_y, atual_z = self.sensor.posicao_atual()

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
        self.algas_x, self.algas_y = self.sensor.path_planning_algas()

        caminho_x_comprimento = len(self.algas_x)
        caminho_y_comprimento = len(self.algas_y)

        print(self.algas_x, self.algas_y)

        if caminho_x_comprimento == caminho_y_comprimento > 0:
            self.coverage_posicao = -1 # A próximo coordenada será a inicial
            return True
        elif caminho_x_comprimento == caminho_y_comprimento:
            self.coverage_posicao = 0
            return True
        else:
            return False

    def mover_para_proxima_alga(self):
        if self.monitoramento_terminou():
            return False
        else:
            x = self.algas_x[self.algas_posicao]
            y = self.algas_y[self.algas_posicao]
            z = self.altura_foto

            # Emitir ordem de movimento para o control_manager
            return control_manager_client(x, y, z)

    def destino_alcancado_alga(self):
        objetivo_x = self.algas_x[self.algas_posicao]
        objetivo_y = self.algas_y[self.algas_posicao]
        objetivo_z = self.altura_foto

        # Obter a posição atual
        atual_x, atual_y, atual_z = self.sensor.posicao_atual()

        alcancou_x = objetivo_x - self.precisao < atual_x < objetivo_x + self.precisao
        alcancou_y = objetivo_y - self.precisao < atual_y < objetivo_y + self.precisao
        alcancou_z = objetivo_z - self.precisao < atual_z < objetivo_z + self.precisao
        
        if alcancou_x and alcancou_y and alcancou_z:
            return True
        else:
            return False

    def monitoramento_terminou(self):
        if self.algas_posicao >= len(self.algas_x):
            return True
        else:
            return False

    def update_estado(self):

        if self.estado == 0: # Solicitar o caminho para o coverage
            print("Solicitando o caminho para o coverage ...")
            sucesso = self.path_planning_coverage()
            if sucesso:
                print("Caminho para o coverage obtido!")
                self.estado = 1
            else:
                print("Erro ao obter o caminho para o coverage.")

        elif self.estado == 1: # Tentar avançar no coverage
            self.coverage_posicao += 1
            if not self.coverage_terminou():
                self.estado = 2
            else:
                print("Todos os pontos do coverage foram visitados!")
                self.estado = 8

        elif self.estado == 2: # Emitir ordem de movimento para a próxima posição no coverage
            print("Emitindo ordem de movimento para a próxima posição no coverage ...")
            sucesso = self.mover_para_proxima_posicao_coverage()
            if sucesso:
                print("Ordem enviada!")
                self.estado = 3

        elif self.estado == 3: # Aguardar o destino
            print("Aguardando destino ...")
            if self.destino_alcancado_coverage():
                print("Destino alcançado!")
                self.estado = 4
        
        elif self.estado == 4: # Solicitar o caminho para fotografar as algas
            print("Solicitando o caminho para fotografar as algas ...")
            sucesso = self.path_planning_algas()
            if sucesso:
                print("Caminho obtido!")
                self.estado = 5

        elif self.estado == 5: # Verificar se todas as algas foram fotografadas
            self.algas_posicao += 1
            if not self.monitoramento_terminou():
                self.estado = 6
            else:
                print("Todas as algas foram fotografadas!")
                self.estado = 1

        elif self.estado == 6: # Emitir ordem de movimento para visitar a próxima alga
            print("Emitindo ordem de movimento para a próxima alga ...")
            sucesso = self.mover_para_proxima_alga()
            if sucesso:
                print("Ordem enviada!")
                self.estado = 7

        elif self.estado == 7: # Aguardar o destino
            print("Aguardando destino ...")
            if self.destino_alcancado_alga():
                print("Destino alcançado!")
                self.estado = 5

        else:
            print("Fim da execução!")
            pass
