import rospy

from algae_monitoring_master.path   import Path


class Manager:

    def __init__(self, coverage_client, sample_client, control_client):
        self.__coverage_client  = coverage_client
        self.__sample_client    = sample_client
        self.__control_client   = control_client

        self.__coverage_path    = Path()
        self.__sample_path      = Path()

        self.__state = 0

    def coverage_planning(self):
        coverage = self.__coverage_client.path()

        if len(coverage) > 0:
            self.__coverage_path.add_after(coverage)

            for i in range(len(coverage)):
                rospy.loginfo("Coordenada %d: %6.1lf %6.1lf %6.1lf",
                    i+1,
                    coverage[i].x,
                    coverage[i].y,
                    coverage[i].z)

            return True

        else:
            return False

    def next_coverage_point(self):
        if self.__coverage_path.complete():
            return False

        else:
            self.__control_client.go_to(self.__coverage_path.coordinate())
            return True

    def sample_planning(self):
        coordinates = self.__sample_client.path()

        if len(coordinates) > 0:
            self.__sample_path.add_after(coordinates)

            for i in range(len(coordinates)):
                rospy.loginfo("Coordenada %d: %6.1lf %6.1lf %6.1lf",
                    i+1,
                    coordinates[i].x,
                    coordinates[i].y,
                    coordinates[i].z)
        
        else:
            rospy.loginfo("Algas não foram identificadas")

    def update(self):
        if self.__state == 0: # Solicitar o caminho para o coverage
            rospy.loginfo("Solicitando o caminho para o coverage")

            if self.coverage_planning():
                self.__state = 1
            else:
                rospy.loginfo("Erro ao obter o caminho para o coverage")

        elif self.__state == 1: # Tentar avançar no coverage
            if self.__coverage_path.next():
                rospy.loginfo("Avançando para a posição %d do coverage", self.__coverage_path.position())
                self.__state = 2
            else:
                rospy.loginfo("Todas as coordenadas do coverage foram visitadas")
                self.__state = 4

        elif self.__state == 2: # Emitir ordem de movimento para a próxima posição no coverage
            if self.next_coverage_point():
                self.__state = 3
        
        elif self.__state == 3: # Solicitar o caminho para fotografar as algas
            rospy.loginfo("Solicitando o caminho para fotografar as algas")
            self.sample_planning()
            self.__state = 1

        elif self.__state == 4:
            rospy.loginfo("Fim da execução")
            self.__state = 5

        else:
            pass
