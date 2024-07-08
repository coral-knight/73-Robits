import numpy as np
import math
import cv2

class Map:

    def __init__(self):
        self.resolution = 0.06
        self.size = [2,2]
        self.range_x = [-self.resolution, self.resolution]
        self.range_y = [-self.resolution, self.resolution]
        
        # map[x, y]
        self.map = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.map[x, y] = [0]

        # ground tiles
        self.extra_map = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.extra_map[x, y] = [0]


        # 0: unknown | 1: lidar | 2: camera | 3: passed by
        self.seen_map = np.zeros(self.size, dtype=int)


    def expand(self, point):
        if point[0] < (self.range_x[0]+self.resolution):
            dif_map_x = math.ceil((self.range_x[0] - point[0])/self.resolution) + 1
            while dif_map_x:
                self.map = np.insert(self.map, 0, None, axis=0)
                for y in range(np.size(self.map, 1)): self.map[0, y] = [0]
                self.extra_map = np.insert(self.extra_map, 0, None, axis=0)
                for y in range(np.size(self.extra_map, 1)): self.extra_map[0, y] = [0]
                self.seen_map = np.insert(self.seen_map, 0, 0, axis=0)
                self.range_x[0] = self.range_x[0] - self.resolution
                dif_map_x = dif_map_x - 1

        if point[0] > (self.range_x[1]-self.resolution): 
            dif_map_x = math.ceil((point[0] - self.range_x[1])/self.resolution) + 1
            while dif_map_x:
                self.map = np.insert(self.map, np.size(self.map, 0), None, axis=0)
                for y in range(np.size(self.map, 1)): self.map[np.size(self.map, 0)-1, y] = [0]
                self.extra_map = np.insert(self.extra_map, np.size(self.extra_map, 0), None, axis=0)
                for y in range(np.size(self.extra_map, 1)): self.extra_map[np.size(self.extra_map, 0)-1, y] = [0]
                self.seen_map = np.insert(self.seen_map, np.size(self.seen_map, 0), 0, axis=0)
                self.range_x[1] = self.range_x[1] + self.resolution
                dif_map_x = dif_map_x - 1

        if point[1] < (self.range_y[0]+self.resolution): 
            dif_map_y = math.ceil((self.range_y[0] - point[1])/self.resolution) + 1
            while dif_map_y:
                self.map = np.insert(self.map, 0, None, axis=1)
                for x in range(np.size(self.map, 0)): self.map[x, 0] = [0]
                self.extra_map = np.insert(self.extra_map, 0, None, axis=1)
                for x in range(np.size(self.extra_map, 0)): self.extra_map[x, 0] = [0]
                self.seen_map = np.insert(self.seen_map, 0, 0, axis=1)
                self.range_y[0] = self.range_y[0] - self.resolution
                dif_map_y = dif_map_y - 1

        if point[1] > (self.range_y[1]-self.resolution): 
            dif_map_y = math.ceil((point[1] - self.range_y[1])/self.resolution) + 1
            while dif_map_y:
                self.map = np.insert(self.map, np.size(self.map, 1), None, axis=1)
                for x in range(np.size(self.map, 0)): self.map[x, np.size(self.map, 1)-1] = [0]
                self.extra_map = np.insert(self.extra_map, np.size(self.extra_map, 1), None, axis=1)
                for x in range(np.size(self.extra_map, 0)): self.extra_map[x, np.size(self.extra_map, 1)-1] = [0]
                self.seen_map = np.insert(self.seen_map, np.size(self.seen_map, 1), 0, axis=1)
                self.range_y[1] = self.range_y[1] + self.resolution
                dif_map_y = dif_map_y - 1

        return 


    def real_to_map(self, real_point):

        return [math.floor((real_point[0]-self.range_x[0])/self.resolution),
                math.floor((real_point[1]-self.range_y[0])/self.resolution)]
        

    def map_to_real(self, map_point):
        
        return [self.range_x[0]+map_point[0]*self.resolution+self.resolution/2,
                self.range_y[0]+map_point[1]*self.resolution+self.resolution/2]


    def add_obstacle(self, point):
        if point == [1000, 1000]: return

        self.expand(point)

        mapx, mapy = self.real_to_map(point)

        for v in self.map[mapx, mapy]:
            if v != 0 and self.dist_coords(point, v) < 0.005:
                return
        self.map[mapx, mapy].append(point)

        return
    

    def add_extra(self, point, type):
        if point == [1000, 1000]: return

        self.expand(point)

        mapx, mapy = self.real_to_map(point)

        for v in self.extra_map[mapx, mapy]:
            if v != 0 and type == v[1] and self.dist_coords(point, v[0]) < 0.02:
                return
        self.extra_map[mapx, mapy].append([point, type])

        return


    def closest(self, point, max):
        mapx, mapy = self.real_to_map(point)

        closest = [1000,1000]
        depth = 0

        c = 0
        while closest == [1000,1000] and c < max:
            c += 1
            for y in range(mapy-depth, mapy+depth+1):
                for x in range(mapx-depth, mapx+depth+1, (1 if y == mapy-depth or y == mapy+depth else 2*depth)):
                    if x >= 0 and x < np.size(self.map, 0) and y >= 0 and y < np.size(self.map, 1):
                        for v in self.map[x, y]:
                            if v != 0 and self.dist_coords(closest, point) > self.dist_coords(v, point):
                                closest = v
            depth += 1

        return closest


    def seen(self, point):
        mapx, mapy = self.real_to_map(point)
        if mapx >= 0 and mapy >= 0 and mapx < np.size(self.seen_map, 0) and mapy < np.size(self.seen_map, 1):
            if self.seen_map[mapx, mapy] == 0:
                self.seen_map[mapx, mapy] = 2

        return
    

    def explored(self, point):
        mapx, mapy = self.real_to_map(point)
        if mapx >= 0 and mapy >= 0 and mapx < np.size(self.seen_map, 0) and mapy < np.size(self.seen_map, 1):
            self.seen_map[mapx, mapy] = 3

        return
    
    
    def print(self):
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                print(self.map[x, y], end=" ")
            print(" ")

    
    def print_tile_map(self):
        print(self.range_x, self.range_y)
        mapa_normal = np.empty([np.size(self.map, 0), np.size(self.map, 1)], dtype=object)
        mapa_ground = np.zeros([np.size(self.map, 0), np.size(self.map, 1)], dtype=object)
        
        # Ground Tiles
        xMod2 = self.real_to_map([-0.03, -0.03])[0] %2
        yMod2 = self.real_to_map([-0.03, -0.03])[1] %2
        coordBuracos = []
        for y in range(yMod2, np.size(self.extra_map, 1)-1, 2):
            contCor = {}
            xTile, yTile = [0, 0]
            for x in range(xMod2, np.size(self.extra_map, 0)-1, 2):
                for aux in [[0, 0], [1, 0], [0, 1], [1, 1]]:
                    for p in self.extra_map[x+aux[0], y+aux[1]]: # [[x, y], cor]
                        if p == 0:
                            continue
                        coords = p[0]
                        cor = p[1]
                        xTile = (coords[0] - self.range_x[0])/0.06
                        yTile = (coords[1] - self.range_y[0])/0.06
                        xTile = int(xTile)
                        yTile = int(yTile)
                        if(aux[0] == 1): xTile -= 1
                        if(aux[1] == 1): yTile -= 1


                        if(cor == 'bh'):
                            cor = 2
                            inicioDoTile = [int((coords[0] + 0.06)/0.12)*0.12-0.05, int((coords[1] - 0.06)/0.12)*0.12-0.05]
                            finalDoTile = [int((coords[0] + 0.06)/0.12)*0.12+0.05, int((coords[1] - 0.06)/0.12)*0.12+0.05]
                            coordBuracos.append([inicioDoTile, finalDoTile])
                            # set.add(tuple([inicioDoTile, finalDoTile]))
                            # detecta a pos dele e salva num set
                        
                        if(cor == 'sw'): cor = 3
                        if(cor == 'cp'): cor = 4
                        if(cor in contCor): contCor[cor] += 1
                        else: contCor[cor] = 1
                        # Ver qual half-tile o ponto pertence
                        # print("tiles", [xTile, yTile])
                        # print("int(tiles)", [int(xTile), int(yTile)])
                contCor[-1] = -1
                maisCor = -1
                for cor in contCor:
                    if(cor == 5):
                        maisCor = 5
                        mapa_ground[int(xTile), int(yTile)] = cor
                        print("Ponto 5:", xTile, yTile)
                        break
                    if(contCor[cor] >= contCor[maisCor] and maisCor != 5):
                        maisCor = cor
                if(contCor[maisCor] >= 7):
                    print(contCor, xTile, yTile)
                    mapa_ground[int(xTile), int(yTile)] = cor
        print("Mapa ground:")
        print(mapa_ground)
        print("CoordBuracos", coordBuracos)
        # Transforma para uma lista
        # listaCoordBuracos = list(coordBuracos)
        # Paredes
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                mapa_normal[x, y] = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                for p in self.map[x, y]:
                    if p == 0:
                        continue
                    # Checa se tiver dentro de um buraco
                    dentroDoBuraco = False
                    for buraco in coordBuracos:
                        if(p[0] >= buraco[0][0] and p[0] <= buraco[1][0] and p[1] >= buraco[0][1] and p[1] <= buraco[1][1]): dentroDoBuraco = True
                    if dentroDoBuraco:
                        print("Tirou buraco")
                        continue
                    # print("P:", p)
                    # array de 9 elementos
                    """
                    012
                    345
                    678
                    """
                    # Ver qual half-tile o ponto pertence
                    xTile = (p[0] - self.range_x[0])/0.06
                    yTile = (p[1] - self.range_y[0])/0.06
                    # Posição dele no half tile + 0.06*qual half-tile ele pertence
                    xPTile = (p[0] % 0.06) /0.06*3 # varia de 0 a 2
                    yPTile = (p[1] % 0.06) /0.06*3
                    xPTile = min(int(xPTile), 2)
                    yPTile = min(int(yPTile), 2) # só para garantir caso ele vire 3
                    blocoParede = int(xPTile + 3*yPTile)
                    if(blocoParede == 0 or blocoParede == 2 or blocoParede == 6 or blocoParede == 8):
                        # Tirar paredes curvas
                        if((p[0]%0.06 - 0.03)**2 + (p[1]%0.06 - 0.03)**2 <= 0.03**2):
                            continue

                    mapa_normal[int(xTile), int(yTile)][blocoParede] += 1
        
        # Tam do tile: 0.12
        # Mapa aux para na hora de printar
        mapa_imprime = np.empty([np.size(self.map, 0)*4+50, np.size(self.map, 1)*4 +50], dtype=object)
        # xMax = np.size(mapa_imprime, 0)
        # yMax = np.size(mapa_imprime, 1)
        for y in range(np.size(mapa_normal, 1)): # Passa por cada mapa_imprime (pares primeiro(vertices))
            for x in range(np.size(mapa_normal, 0)):
                # Casa central: x*2+1, y*2+1
                for auxY in [0, 1, 2]:
                    for auxX in [0, 1, 2]:
                        if auxX == 1 and auxY == 1:
                            mapa_imprime[x*2+auxX+25, y*2+auxY+25] = mapa_ground[x, y]
                        elif mapa_normal[x, y][auxX+3*auxY] >= 1:
                            mapa_imprime[x*2+auxX+25, y*2+auxY+25] = 1

        # Remove todos as linhas e colunas q só tem None
        darBreak = False
        for y in range(np.size(mapa_imprime, 1)):
            contK = 0
            if darBreak: break
            for x in range(np.size(mapa_imprime, 0)):
                if mapa_imprime[x, y] == None:
                    contK += 1
            if (contK == np.size(mapa_imprime, 0)):
                for x in range(np.size(mapa_imprime, 0)):
                    mapa_imprime[x, y] = '/' # Ignora
            else:
                darBreak = True
                break
        darBreak = False
        for y in range(np.size(mapa_imprime, 1)-1, -1, -1):
            contK = 0
            if darBreak: break
            for x in range(np.size(mapa_imprime, 0)):
                if mapa_imprime[x, y] == None or mapa_imprime[x, y] == '/':
                    contK += 1
            if (contK == np.size(mapa_imprime, 0)):
                for x in range(np.size(mapa_imprime, 0)):
                    mapa_imprime[x, y] = '/' # Ignora
            else:
                darBreak = True
                break
        darBreak = False
        for x in range(np.size(mapa_imprime, 0)):
            contK = 0
            if darBreak: break
            for y in range(np.size(mapa_imprime, 1)):
                if mapa_imprime[x, y] == None or mapa_imprime[x, y] == '/':
                    contK += 1
            if (contK == np.size(mapa_imprime, 1)):
                for y in range(np.size(mapa_imprime, 1)):
                    mapa_imprime[x, y] = '/' # Ignora
            else:
                darBreak = True
                break
        darBreak = False
        for x in range(np.size(mapa_imprime, 0)-1, -1, -1):
            contK = 0
            if darBreak: break
            for y in range(np.size(mapa_imprime, 1)):
                if mapa_imprime[x, y] == None or mapa_imprime[x, y] == '/':
                    contK += 1
            if (contK == np.size(mapa_imprime, 1)):
                for y in range(np.size(mapa_imprime, 1)):
                    mapa_imprime[x, y] = '/' # Ignora
            else:
                darBreak = True
                break

        print("Mapinha bonitinho")
        for y in range(np.size(mapa_imprime, 1)-1, -1, -1):
            contK = 0
            for x in range(np.size(mapa_imprime, 0)):
                if mapa_imprime[x, y] == None or mapa_imprime[x, y] == 0 or mapa_imprime[x, y] == '0':
                    print('.', end=" ")
                elif mapa_imprime[x, y] == '/':
                    contK += 1
                else:
                    print(mapa_imprime[x, y], end=" ")
            if contK != np.size(mapa_imprime, 0): print(" ")

        return_map = []
        for y in range(np.size(mapa_imprime, 1)):
            auxList = []
            for x in range(np.size(mapa_imprime, 0)):
                if mapa_imprime[x, y] != '/': auxList.append(str(mapa_imprime[x, y]) if mapa_imprime[x, y] else '0')
            if len(auxList) != 0: return_map.append(auxList)
        
        # Adjust Map
        # for -> encontra os primeiro 5, pega a pos e passa mod 4, pq passa mod 4.? # p saber se a parede tá dentro de um buraco ou só entre dois buracos
        # 2 é buraco, sla
        # 2 1 2  //-> pd ser parede entre 4 buracos, ou o coiso de marcar p n andar
        # 1 1 1 
        # 2 1 2
        print("bom dia :3")
        xMod4 = -1
        yMod4 = -1
        for y in range(np.size(return_map, 1)):
            # print("y")
            for x in range(np.size(return_map, 0)):
                # print("x")
                if(return_map[x][y] == '5'):
                    yMod4 = y%4
                    xMod4 = x%4 #mod 4 tem aki
                    break
            if(yMod4 != -1): break
        print(xMod4, yMod4)
        print("vai entrar no loop")
        for y in range(yMod4, np.size(return_map, 1), 2): # return_map[x][y]
            print("143")
            for x in range(xMod4, np.size(return_map, 0), 2):
                if(str(return_map[x][y]) != '0'):
                    xTile = int((x-xMod4)/4)
                    yTile = int((y-yMod4)/4)
                    cont = 0
                    inicioTile = [xTile*4+xMod4, yTile*4+yMod4]
                    if(return_map[inicioTile[0]][inicioTile[1]] == return_map[x][y]): cont+=1
                    if(return_map[inicioTile[0]+2][inicioTile[1]] == return_map[x][y]): cont+=1
                    if(return_map[inicioTile[0]][inicioTile[1]+2] == return_map[x][y]): cont+=1
                    if(return_map[inicioTile[0]+2][inicioTile[1]+2] == return_map[x][y]): cont+=1
                    if(cont >= 1):
                        return_map[inicioTile[0]][inicioTile[1]] = return_map[x][y]
                        return_map[inicioTile[0]+2][inicioTile[1]] = return_map[x][y]
                        return_map[inicioTile[0]][inicioTile[1]+2] = return_map[x][y]
                        return_map[inicioTile[0]+2][inicioTile[1]+2] = return_map[x][y]
                    else:
                        return_map[inicioTile[0]][inicioTile[1]] = '0'
                        return_map[inicioTile[0]+2][inicioTile[1]] = '0'
                        return_map[inicioTile[0]][inicioTile[1]+2] = '0'
                        return_map[inicioTile[0]+2][inicioTile[1]+2] = '0'

        # espelha de volta
        return [row[::-1] for row in return_map]


    def print_real(self):
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                print(self.map_to_real([x, y]), end=" ")
            print(" ")

        return
    

    def print_seen(self):
        for y in range(np.size(self.seen_map, 1)):
            for x in range(np.size(self.seen_map, 0)):
                print(self.seen_map[x, y], end=" ")
            print(" ")
    

    def to_png(self):
        transpose = np.zeros([np.size(self.map, 1), np.size(self.map, 0)])

        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                if len(self.map[x, np.size(self.map, 1)-1-y]) > 3:
                    transpose[y, x] = 1

        s = 'map.png'
        M = abs(transpose)*255
        cv2.imwrite(s, M)

        return 
    

    def to_detailed_png(self, current_tick):
        size = [int((0.01+self.range_x[1]-self.range_x[0])/0.01), int((0.01+self.range_y[1]-self.range_y[0])/0.01), 3]
        tile_map = np.zeros(size)

        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                for i in range(1, len(self.map[x, y])):
                    [a, b] = self.map[x, y][i]
                    tile_map[int((a-self.range_x[0])/0.01), int((b-self.range_y[0])/0.01)] = [255, 255, 255]

        for y in range(np.size(self.extra_map, 1)):
            for x in range(np.size(self.extra_map, 0)):
                for i in range(1, len(self.extra_map[x, y])):
                    [[a, b], c] = self.extra_map[x, y][i]

                    rgb = [0, 0, 0]
                    if c == 'b': rgb = [0, 0, 255]
                    if c == 'y': rgb = [255, 255, 0]
                    if c == 'g': rgb = [0, 255, 0]
                    if c == 'p': rgb = [255, 0, 255]
                    if c == 'o': rgb = [255, 100, 0]
                    if c == 'r': rgb = [255, 0, 0]
                    if c == 'cp': rgb = [0, 120, 120]
                    if c == 'sw': rgb = [140, 70, 20]
                    if c == 'bh': rgb = [40, 40, 40]
                    if c == 'ob': rgb = [180, 140, 180]

                    bgr = [rgb[2], rgb[1], rgb[0]]
                    tile_map[int((a-self.range_x[0])/0.01), int((b-self.range_y[0])/0.01)] = bgr

        tile_map = np.rot90(tile_map)
        s = "detailed_" + str(current_tick) + ".png"
        #M = abs(tile_map)*255
        cv2.imwrite(s, tile_map)

        return
    

    def to_png_seen(self):
        transpose = np.zeros([np.size(self.seen_map, 1), np.size(self.seen_map, 0)])

        for y in range(np.size(self.seen_map, 1)):
            for x in range(np.size(self.seen_map, 0)):
                if self.seen_map[x, np.size(self.seen_map, 1)-1-y] > 0:
                    transpose[y, x] = 1

        s = 'explored.png'
        M = abs(transpose)*255
        cv2.imwrite(s, M)

        return 
        

    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist