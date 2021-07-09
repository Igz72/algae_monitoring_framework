from artigo_framework.path_planning_client_utils    import path_planning_client
from artigo_framework.control_manager_client_utils  import control_manager_client
from artigo_framework.gps_client_utils              import GPSClient

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
        self.caminho_x_coverage = []                        # Coordenadas X do covarege
        self.caminho_y_coverage = []                        # Coordenadas Y do covarege
        self.posicao_coverage   = 0                         # Posição atual no covarege
        self.caminho_x_algas    = []                        # Coordenadas X das algas
        self.caminho_y_algas    = []                        # Coordenadas Y das algas
        self.posicao_algas      = 0                         # Posição da alga atual
        self.estado             = 0                         # Estado atual do controlador
        self.gps                = GPSClient("ground_truth") # Contém informações do GPS

    def path_planning_coverage(self):
        # Requisição do caminho para o path_planning_server
        self.caminho_x_coverage, self.caminho_y_coverage = path_planning_client(
            self.mapa_inicio_x   ,
            self.mapa_inicio_y   ,
            self.mapa_largura    ,
            self.mapa_altura     ,
            self.camera_largura  ,
            self.camera_altura   )

        caminho_x_comprimento = len(self.caminho_x_coverage)
        caminho_y_comprimento = len(self.caminho_y_coverage)

        if caminho_x_comprimento == caminho_y_comprimento > 0:
            self.posicao_coverage = -1 # A próximo coordenada será a inicial
            return True
        else:
            return False

    def mover_para_proxima_posicao_coverage(self):
        if self.coverage_terminou():
            return False
        else:
            x = self.caminho_x_coverage[self.posicao_coverage]
            y = self.caminho_y_coverage[self.posicao_coverage]
            z = self.altura_coverage

            # Emitir ordem de movimento para o control_manager
            return control_manager_client(x, y, z)

    def destino_alcancado_coverage(self):
        objetivo_x = self.caminho_x_coverage[self.posicao_coverage]
        objetivo_y = self.caminho_y_coverage[self.posicao_coverage]
        objetivo_z = self.altura_coverage

        # Obter a posição atual
        atual_x, atual_y, atual_z = self.gps.posicao_atual()

        alcancou_x = objetivo_x - self.precisao < atual_x < objetivo_x + self.precisao
        alcancou_y = objetivo_y - self.precisao < atual_y < objetivo_y + self.precisao
        alcancou_z = objetivo_z - self.precisao < atual_z < objetivo_z + self.precisao
        
        if alcancou_x and alcancou_y and alcancou_z:
            return True
        else:
            return False

    def fotografar_coverage(self):
        return True

    def coverage_terminou(self):
        if self.posicao_coverage >= len(self.caminho_x_coverage):
            return True
        else:
            return False

    def path_planning_algas(self):
        return True

    def mover_para_proxima_alga(self):
        if self.monitoramento_terminou:
            return False
        else:
            x = self.caminho_x_algas[self.posicao_algas]
            y = self.caminho_y_algas[self.posicao_algas]
            z = self.altura_foto

            # Emitir ordem de movimento para o control_manager
            return control_manager_client(x, y, z)

    def destino_alcancado_alga(self):
        objetivo_x = self.caminho_x_algas[self.posicao_algas]
        objetivo_y = self.caminho_y_algas[self.posicao_algas]
        objetivo_z = self.altura_foto

        # Obter a posição atual
        atual_x, atual_y, atual_z = self.gps.posicao_atual()

        alcancou_x = objetivo_x - self.precisao < atual_x < objetivo_x + self.precisao
        alcancou_y = objetivo_y - self.precisao < atual_y < objetivo_y + self.precisao
        alcancou_z = objetivo_z - self.precisao < atual_z < objetivo_z + self.precisao
        
        if alcancou_x and alcancou_y and alcancou_z:
            return True
        else:
            return False

    def fotografar_alga():
        return True

    def monitoramento_terminou(self):
        if self.posicao_algas >= len(self.caminho_x_algas):
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

        elif self.estado == 1:
            self.posicao_coverage += 1
            if not self.coverage_terminou():
                self.estado = 2
            else:
                print("Todos os pontos do coverage foram visitados!")
                self.estado = 10

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

        elif self.estado == 4: # Capturar imagem no coverage
            print("Capturando imagem ...")
            sucesso = self.fotografar_coverage()
            if sucesso:
                print("Imagem capturada!")
                self.estado = 5
        
        elif self.estado == 5: # Solicitar o caminho para fotografar as algas
            print("Solicitando o caminho para fotografar as algas ...")
            sucesso = self.path_planning_algas()
            if sucesso:
                print("Caminho obtido!")
                self.estado = 6

        elif self.estado == 6: # Verificar se todas as algas foram fotografadas
            self.posicao_algas += 1
            if not self.monitoramento_terminou():
                self.estado = 7
            else:
                print("Todas as algas foram fotografadas!")
                self.estado = 1

        elif self.estado == 7: # Emitir ordem de movimento para visitar a próxima alga
            print("Emitindo ordem de movimento para a próxima alga ...")
            sucesso = self.mover_para_proxima_alga()
            if sucesso:
                print("Ordem enviada!")
                self.estado = 8

        elif self.estado == 8: # Aguardar o destino
            print("Aguardando destino ...")
            if self.destino_alcancado_alga():
                print("Destino alcançado!")
                self.estado = 9

        elif self.estado == 9: # Capturar imagem da alga
            print("Capturando imagem da alga ...")
            sucesso = self.fotografar_alga()
            if sucesso:
                print("Imagem capturada!")
                self.estado = 6

        else:
            print("Fim da execução!")
            pass
