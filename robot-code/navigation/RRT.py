import math
import random


class RRT:

    def __init__(self, map):
        self.map = map 
        self.graph = []


    def random_point(self):
        # random point between map boundaries


    def closest_point(self, point):
        # ponto mais proximo 


    def project_point(self, point):
        # projeta se tiver longe


    def add_graph(self, point):
        closest = self.closest_point(point)
        point = self.project_point(closest)
        # adiciona pro closest


    def explore(self, ticks):
        while ticks > 0:
            ticks -= 1

            point = self.random_point()
            point = self.add_graph(point)

            # se o ponto tá em um lugar n explorado, adiciona ele pra lista de ação

            # vai criar varias vezes a classe sempre q quiser resetar no código principal