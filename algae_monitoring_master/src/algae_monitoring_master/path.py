class Path:
    def __init__(self):
        self.__position = -1
        self.__path = []

    def coordinate(self):
        return self.__path[self.__position]

    def x(self):
        return self.__path[self.__position].x

    def y(self):
        return self.__path[self.__position].y

    def z(self):
        return self.__path[self.__position].z

    def position(self):
        return self.__position + 1

    def next(self):
        if not self.complete():
            self.__position += 1
            return True

        else:
            return False

    def initialize(self, coordinates):
        self.__path = coordinates

    def add_before(self, coordinates):
        new_path = self.__path[0 : self.__position]
        new_path.extend(coordinates)
        new_path.extend(self.__path[self.__position :])
        self.__path = new_path

    def add_after(self, coordinates):
        self.__path.extend(coordinates)

    def complete(self):
        if self.__position >= len(self.__path) - 1:
            return True

        else:
            return False
