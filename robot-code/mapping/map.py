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


        # 0: unknown | 1: lidar | 2: camera | 3: totally explored
        self.seen_map = np.empty(self.size, dtype=object)
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                self.seen_map[x, y] = [0, [0, 0]]


    def expand(self, point):
        if point[0] < (self.range_x[0]+self.resolution):
            dif_map_x = math.ceil((self.range_x[0] - point[0])/self.resolution) + 1
            while dif_map_x:
                self.map = np.insert(self.map, 0, None, axis=0)
                for y in range(np.size(self.map, 1)): self.map[0, y] = [0]
                self.extra_map = np.insert(self.extra_map, 0, None, axis=0)
                for y in range(np.size(self.extra_map, 1)): self.extra_map[0, y] = [0]
                self.seen_map = np.insert(self.seen_map, 0, None, axis=0)
                for y in range(np.size(self.seen_map, 1)): self.seen_map[0, y] = [0, [0, 0]]
                self.range_x[0] = self.range_x[0] - self.resolution
                dif_map_x = dif_map_x - 1

        if point[0] > (self.range_x[1]-self.resolution): 
            dif_map_x = math.ceil((point[0] - self.range_x[1])/self.resolution) + 1
            while dif_map_x:
                self.map = np.insert(self.map, np.size(self.map, 0), None, axis=0)
                for y in range(np.size(self.map, 1)): self.map[np.size(self.map, 0)-1, y] = [0]
                self.extra_map = np.insert(self.extra_map, np.size(self.extra_map, 0), None, axis=0)
                for y in range(np.size(self.extra_map, 1)): self.extra_map[np.size(self.extra_map, 0)-1, y] = [0]
                self.seen_map = np.insert(self.seen_map, np.size(self.seen_map, 0), None, axis=0)
                for y in range(np.size(self.seen_map, 1)): self.seen_map[np.size(self.seen_map, 0)-1, y] = [0, [0, 0]]
                self.range_x[1] = self.range_x[1] + self.resolution
                dif_map_x = dif_map_x - 1

        if point[1] < (self.range_y[0]+self.resolution): 
            dif_map_y = math.ceil((self.range_y[0] - point[1])/self.resolution) + 1
            while dif_map_y:
                self.map = np.insert(self.map, 0, None, axis=1)
                for x in range(np.size(self.map, 0)): self.map[x, 0] = [0]
                self.extra_map = np.insert(self.extra_map, 0, None, axis=1)
                for x in range(np.size(self.extra_map, 0)): self.extra_map[x, 0] = [0]
                self.seen_map = np.insert(self.seen_map, 0, None, axis=1)
                for x in range(np.size(self.seen_map, 0)): self.seen_map[x, 0] = [0, [0, 0]]
                self.range_y[0] = self.range_y[0] - self.resolution
                dif_map_y = dif_map_y - 1

        if point[1] > (self.range_y[1]-self.resolution): 
            dif_map_y = math.ceil((point[1] - self.range_y[1])/self.resolution) + 1
            while dif_map_y:
                self.map = np.insert(self.map, np.size(self.map, 1), None, axis=1)
                for x in range(np.size(self.map, 0)): self.map[x, np.size(self.map, 1)-1] = [0]
                self.extra_map = np.insert(self.extra_map, np.size(self.extra_map, 1), None, axis=1)
                for x in range(np.size(self.extra_map, 0)): self.extra_map[x, np.size(self.extra_map, 1)-1] = [0]
                self.seen_map = np.insert(self.seen_map, np.size(self.seen_map, 1), None, axis=1)
                for x in range(np.size(self.seen_map, 0)): self.seen_map[x, np.size(self.extra_map, 1)-1] = [0, [0, 0]]
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
    

    def add_extra(self, point, type, dir):
        if point == [1000, 1000]: return

        self.expand(point)

        mapx, mapy = self.real_to_map(point)

        for v in self.extra_map[mapx, mapy]:
            if v != 0 and type == v[1] and self.dist_coords(point, v[0]) < 0.02:
                return
        self.extra_map[mapx, mapy].append([point, type, dir])

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


    def seen(self, point, ang):
        mapx, mapy = self.real_to_map(point)
        if mapx >= 0 and mapy >= 0 and mapx < np.size(self.seen_map, 0) and mapy < np.size(self.seen_map, 1):
            if self.seen_map[mapx, mapy][0] == 0: self.seen_map[mapx, mapy] = [2, [ang, ang]]

            self.seen_map[mapx, mapy][1][0] = min(self.seen_map[mapx, mapy][1][0], ang)
            self.seen_map[mapx, mapy][1][1] = max(self.seen_map[mapx, mapy][1][1], ang)

            if min(self.seen_map[mapx, mapy][1][1]-self.seen_map[mapx, mapy][1][0], 2*math.pi-(self.seen_map[mapx, mapy][1][1]-self.seen_map[mapx, mapy][1][0])) > math.pi/2:
                self.seen_map[mapx, mapy][0] = 3

        return
    

    def explored(self, point):
        mapx, mapy = self.real_to_map(point)
        if mapx >= 0 and mapy >= 0 and mapx < np.size(self.seen_map, 0) and mapy < np.size(self.seen_map, 1):
            self.seen_map[mapx, mapy][0] = 3

        return
    
    
    def print(self):
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                print(self.map[x, y], end=" ")
            print(" ")


    def UF_find(self, pos):
        if(pos[0] == 'A'): return pos
        if(self.UF_anc[pos[0]][pos[1]] == pos): return pos
        self.UF_anc[pos[0]][pos[1]] = self.UF_find(self.UF_anc[pos[0]][pos[1]])
        return self.UF_anc[pos[0]][pos[1]]


    def UF_join(self, pos1, pos2):
        # print("Erro no find??")
        pos1 = self.UF_find(pos1)
        pos2 = self.UF_find(pos2)
        # print("JOINNNN")
        if(pos1[0] == 'A' and pos2[0] == 'A'):
            print("Union find os dois 'A'")
            return
        if(pos1[0] == 'A'):
            self.UF_anc[pos2[0]][pos2[1]] = pos1
            return
        if(pos2[0] == 'A'):
            self.UF_anc[pos1[0]][pos1[1]] = pos2
            return
        if(pos1 == pos2): return # Já está conectado
        if(self.UF_nivel[pos1[0]][pos1[1]] > self.UF_nivel[pos2[0]][pos2[1]]): pos1, pos2 = pos2, pos1
        self.UF_anc[pos1[0]][pos1[1]] = pos2
        if(self.UF_nivel[pos1[0]][pos1[1]] == self.UF_nivel[pos2[0]][pos2[1]]): self.UF_nivel[pos2[0]][pos2[1]] += 1
        return


    def print_tile_map(self):
        print(self.range_x, self.range_y)
        mapa_normal = np.empty([np.size(self.map, 0), np.size(self.map, 1)], dtype=object)
        mapa_ground = np.zeros([np.size(self.map, 0), np.size(self.map, 1)], dtype=object)
        
        # Ground Tiles
        xMod2 = self.real_to_map([-0.03, -0.03])[0] %2
        yMod2 = self.real_to_map([-0.03, -0.03])[1] %2
        coordBuracos = []
        for x in range(xMod2, np.size(self.extra_map, 0)-1, 2):
            for y in range(yMod2, np.size(self.extra_map, 1)-1, 2):
                contCor = {}
                xTile, yTile = [0, 0]
                for aux in [[0, 0], [1, 0], [0, 1], [1, 1]]:
                    for p in self.extra_map[x+aux[0], y+aux[1]]: # [[x, y], cor]
                        if p == 0:
                            continue
                        coords = p[0]
                        cor = p[1]
                        if(str(cor) == '5'):
                            print("AQUII DIACHOOOO", coords)
                        xTile = (coords[0] - self.range_x[0])/0.06
                        yTile = (coords[1] - self.range_y[0])/0.06
                        xTile = int(xTile)
                        yTile = int(yTile)
                        if(aux[0] == 1): xTile -= 1
                        if(aux[1] == 1): yTile -= 1
                        
                        if(cor == 'ob'):
                            continue

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
                contCor['-1'] = -1
                maisCor = '-1'
                for cor in contCor:
                    if(str(cor) == '5'):
                        maisCor = 5
                        mapa_ground[int(xTile), int(yTile)] = cor
                        print("Ponto 5:", xTile, yTile)
                        print(int((0 - self.range_x[0])/0.06), int((0 - self.range_y[0])/0.06))
                        break
                    if(contCor[cor] >= contCor[maisCor] and maisCor != 5):
                        maisCor = cor
                if(contCor[maisCor] >= 7):
                    print(contCor, xTile, yTile)
                    mapa_ground[int(xTile), int(yTile)] = maisCor
        print("Mapa ground:")
        print(mapa_ground)
        print("CoordBuracos", coordBuracos)
        # Transforma para uma lista
        # listaCoordBuracos = list(coordBuracos)
        # Paredes
        for y in range(np.size(self.map, 1)):
            for x in range(np.size(self.map, 0)):
                mapa_normal[x, y] = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        for x in range(np.size(self.map, 0)):
            for y in range(np.size(self.map, 1)):
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
                        if((p[0]%0.06 - 0.03)**2 + (p[1]%0.06 - 0.03)**2 <= 0.03**2 and (p[0]%0.06 - 0.03)**2 + (p[1]%0.06 - 0.03)**2 >= 0.023**2): continue
                        if((p[0]%0.06 - 0.03)**2 + (p[1]%0.06 - 0.03)**2 <= 0.023**2):
                            mapa_normal[int(xTile), int(yTile)][blocoParede] = 't'
                    if(mapa_normal[int(xTile), int(yTile)][blocoParede] != 't'): mapa_normal[int(xTile), int(yTile)][blocoParede] = 1

        wallTokens = ['F', 'O', 'P', 'C', 'H', 'S', 'U']
        # Vítimas
        for x in range(np.size(self.extra_map, 0)):
            xTile, yTile = [0, 0]
            for y in range(np.size(self.extra_map, 1)):
                for p in self.extra_map[x, y]: # [[x, y], tipo]
                    if p == 0:
                        continue
                    coords = p[0]
                    tipo = p[1]
                    dir = p[2]
                    if str(tipo) in wallTokens:
                        """
                        012
                        345
                        678
                        """
                        # xTile = (coords[0] - self.range_x[0])/0.06
                        # yTile = (coords[1] - self.range_y[0])/0.06
                        xTile = int((coords[0] - self.range_x[0])/0.06)
                        yTile = int((coords[1] - self.range_y[0])/0.06)
                        
                        # Posição dele no half tile + 0.06*qual half-tile ele pertence
                        xPTile = (coords[0]  - self.range_x[0]) % 0.06
                        yPTile = (coords[1]  - self.range_y[0]) % 0.06
                        # xPTile = (coords[0] % 0.06)/0.06*3
                        # yPTile = (coords[1] % 0.06)/0.06*3
                        # xPTile = min(int(xPTile), 2)
                        # yPTile = min(int(yPTile), 2) # só para garantir caso ele vire 3
                        if dir == 0: # 1 ou 7
                            if(yPTile < 0.03): blocoVitima = 1
                            else: blocoVitima = 7
                        else:
                            if(xPTile < 0.03): blocoVitima = 3
                            else: blocoVitima = 5

                        # ang = (math.atan2(xPTile-0.03, yPTile-0.03) +math.pi/4) % (2*math.pi)
                        # if(ang >= 0 and ang <= math.pi/2): blocoVitima = 5
                        # elif(ang >= math.pi/2 and ang <= math.pi): blocoVitima = 1
                        # elif(ang >= math.pi and ang <= 3*math.pi/2): blocoVitima = 3
                        # elif(ang >= 3*math.pi/2 and ang <= 2*math.pi): blocoVitima = 7
                        
                        print("Vitima do tipo", tipo, "no tile", xTile, yTile, "direcao", dir)
                        print("Pos dentro do tile:", xPTile, yPTile, "Bloco Vitima", blocoVitima)
                        
                        # blocoVitima = int(xPTile + 3*yPTile)
                        mapa_normal[int(xTile), int(yTile)][blocoVitima] = str(tipo)
                        # mapa_normal[int(xTile), int(yTile)][1], mapa_normal[int(xTile), int(yTile)][3], mapa_normal[int(xTile), int(yTile)][7], mapa_normal[int(xTile), int(yTile)][5] = 'H', 'S', 'U', 'O'
        
        # Tam do tile: 0.12
        # 
        # Mapa aux para na hora de printar
        mapa_imprime = np.empty([np.size(self.map, 0)*4+50, np.size(self.map, 1)*4 +50], dtype=object)
        # xMax = np.size(mapa_imprime, 0)
        # yMax = np.size(mapa_imprime, 1)
        for x in range(np.size(mapa_normal, 0)):
            for y in range(np.size(mapa_normal, 1)): # Passa por cada mapa_imprime (pares primeiro(vertices))
                # Casa central: x*2+1, y*2+1
                for auxX in [0, 1, 2]:
                    for auxY in [0, 1, 2]:
                        if auxX == 1 and auxY == 1:
                            mapa_imprime[x*2+auxX+25, y*2+auxY+25] = mapa_ground[x, y]
                        # elif mapa_normal[x, y][auxX+3*auxY] = 1:
                        elif str(mapa_normal[x, y][auxX+3*auxY]) == '1' or str(mapa_normal[x, y][auxX+3*auxY]) == 't': # t eh para parede curva
                            if mapa_imprime[x*2+auxX+25, y*2+auxY+25] != 't' and not mapa_imprime[x*2+auxX+25, y*2+auxY+25] in wallTokens:
                                mapa_imprime[x*2+auxX+25, y*2+auxY+25] = mapa_normal[x, y][auxX+3*auxY]
                        elif str(mapa_normal[x, y][auxX+3*auxY]) in wallTokens:
                            if str(mapa_imprime[x*2+auxX+25, y*2+auxY+25])[0] in wallTokens: mapa_imprime[x*2+auxX+25, y*2+auxY+25] = mapa_imprime[x*2+auxX+25, y*2+auxY+25]+mapa_normal[x, y][auxX+3*auxY]
                            else: mapa_imprime[x*2+auxX+25, y*2+auxY+25] = mapa_normal[x, y][auxX+3*auxY]
            
        for x in range(np.size(mapa_imprime, 0)):
            for y in range(np.size(mapa_imprime, 1)): # Passa por cada mapa_imprime (pares primeiro(vertices))
                if(mapa_imprime[x][y] == 't'): mapa_imprime[x][y] = 0

        # Remove todos as linhas e colunas q só tem None
        darBreak = False
        for y in range(np.size(mapa_imprime, 1)): #Aki e no prox é assim msm
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
        for x in range(np.size(mapa_imprime, 0)):
            contK = 0
            for y in range(np.size(mapa_imprime, 1)-1, -1, -1):
                if mapa_imprime[x, y] == None or mapa_imprime[x, y] == 0 or mapa_imprime[x, y] == '0':
                    print('.', end=" ")
                elif mapa_imprime[x, y] == '/':
                    contK += 1
                else:
                    print(mapa_imprime[x, y], end=" ")
            if contK != np.size(mapa_imprime, 0): print(" ")

        return_map = []
        for x in range(np.size(mapa_imprime, 0)):
            auxList = []
            for y in range(np.size(mapa_imprime, 1)):
                if mapa_imprime[x, y] != '/': auxList.append(str(mapa_imprime[x, y]) if mapa_imprime[x, y] else '0')
            if len(auxList) != 0: return_map.append(auxList)
        
        # Adjust Map
        # for -> encontra os primeiro 5, pega a pos e passa mod 4, pq passa mod 4.? # p saber se a parede tá dentro de um buraco ou só entre dois buracos
        # 2 é buraco, sla
        # 2 1 2  //-> pd ser parede entre 4 buracos, ou o coiso de marcar p n andar
        # 1 1 1 
        # 2 1 2
        
        maiorX, maiorY, menorX, menorY = 0, 0, 9999, 9999
        for x in range(np.size(return_map, 0)):
            for y in range(np.size(return_map, 1)):
                if str(return_map[x][y]) == '1':
                    # print("ATUALIZAAAAAAAA")
                    maiorY = max(y, maiorY)
                    maiorX = max(x, maiorX)
                    menorY = min(y, menorY)
                    menorX = min(x, menorX)
        print(maiorX, maiorY, menorX, menorY)
        # Borda de p
        for y in range(menorY, maiorY+1, 1):
            if str(return_map[menorX][y]) != '1' and not str(return_map[menorX][y]) in wallTokens: return_map[menorX][y] = 'w'
            if str(return_map[maiorX][y]) != '1' and not str(return_map[maiorX][y]) in wallTokens: return_map[maiorX][y] = 'w'
        
        for x in range(menorX, maiorX+1, 1):
            if str(return_map[x][maiorY]) != '1' and not str(return_map[x][maiorY]) in wallTokens: return_map[x][maiorY] = 'w'
            if str(return_map[x][menorY]) != '1' and not str(return_map[x][menorY]) in wallTokens: return_map[x][menorY] = 'w'
        
            
        print("bom dia :3")
        xMod4 = -1
        yMod4 = -1
        for x in range(np.size(return_map, 0)):
            # print("x")
            for y in range(np.size(return_map, 1)):
                # print("y")
                if(return_map[x][y] == '5'):
                    yMod4 = y%4
                    xMod4 = x%4 #mod 4 tem aki
                    break
            if(yMod4 != -1): break
        print(xMod4, yMod4)
        print("vai entrar no loop")
        
        for x in range(xMod4, np.size(return_map, 0)-2, 2):
            for y in range(yMod4, np.size(return_map, 1)-2, 2): # return_map[x][y]
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

        print("VAMOS ABACATAR 1")
        # Area 4 -> *
        # Green, Red, Orange
        # return_map = [row[::-1] for row in return_map] # Espelha de volta
        self.UF_anc = [row[:] for row in return_map]
        self.UF_nivel = [row[:] for row in return_map]
        for y in range(len(self.UF_anc[0])):
            for x in range(len(self.UF_anc)):
                self.UF_anc[x][y] = [x, y]
                self.UF_nivel[x][y] = 0

        startPos = [-1, -1]
        for x in range(np.size(return_map, 0)):
            for y in range(np.size(return_map, 1)):
                if(str(return_map[x][y]) == '5'):
                    startPos = [x, y]
                    break
            if(startPos[0] != -1): break
        print(startPos)
        print("Eu estou sendo chamado")
        mapa = [row[:] for row in return_map]
        print("Testando uma coisinha 1:", mapa[0][0])
        # print("Testando uma coisinha 2:", mapa[0, 0])
        print("VAMOS ABACATAR 2")
        # Separando as áreas usando union-find (UF)
        tilesQuePodeIr = ['0', '3', '4', '5', 'b', 'p', 'y']
        for x in range (1, len(mapa)-1, 1):
            for y in range (1, len(mapa[0])-1, 1):
                pos = [x, y]
                if(x == 13 and y == 27): print("AKIII", mapa[pos[0]][pos[1]])
                if mapa[pos[0]][pos[1]] in tilesQuePodeIr:
                    if (mapa[pos[0]+1][pos[1]] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]][pos[1]-1] in tilesQuePodeIr):
                        self.UF_join(pos, [pos[0]+1, pos[1]])
                        self.UF_join([pos[0]+1, pos[1]+1], [pos[0]+1, pos[1]])
                        self.UF_join([pos[0]+1, pos[1]+1], [pos[0]+1, pos[1]-1])
                        self.UF_join([pos[0], pos[1]+1], [pos[0]+1, pos[1]-1])
                        self.UF_join([pos[0], pos[1]+1], [pos[0], pos[1]-1])
                        # print("Sai 1")
                    if (mapa[pos[0]-1][pos[1]] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]][pos[1]-1] in tilesQuePodeIr):
                        # print("Ultimo em 2")
                        # if(x == 13 and y == 27): print('AAA')
                        self.UF_join(pos, [pos[0]-1, pos[1]])
                        self.UF_join([pos[0]-1, pos[1]+1], [pos[0]-1, pos[1]])
                        self.UF_join([pos[0]-1, pos[1]+1], [pos[0]-1, pos[1]-1])
                        self.UF_join([pos[0], pos[1]+1], [pos[0]-1, pos[1]-1])
                        self.UF_join([pos[0], pos[1]+1], [pos[0], pos[1]-1])
                        # print("Sai 2")
                    # elif x==13 and y==27: print("SUMIUUU", mapa[pos[0]-1][pos[1]], mapa[pos[0]-1][pos[1]+1], mapa[pos[0]-1][pos[1]-1], mapa[pos[0]][pos[1]+1], mapa[pos[0]][pos[1]-1]) # 1 0 0 1 0
                    if (mapa[pos[0]][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]] in tilesQuePodeIr):
                        # print("Ultimo em 3")
                        self.UF_join(pos, [pos[0], pos[1]+1])
                        self.UF_join([pos[0]+1, pos[1]+1], [pos[0], pos[1]+1])
                        self.UF_join([pos[0]+1, pos[1]+1], [pos[0]-1, pos[1]+1])
                        self.UF_join([pos[0]+1, pos[1]], [pos[0]-1, pos[1]+1])
                        self.UF_join([pos[0]+1, pos[1]], [pos[0]-1, pos[1]])
                        # print("Sai 3")
                    if (mapa[pos[0]][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]] in tilesQuePodeIr):
                        # print("Ultimo em 4")
                        self.UF_join(pos, [pos[0], pos[1]-1])
                        self.UF_join([pos[0]+1, pos[1]-1], [pos[0], pos[1]-1])
                        self.UF_join([pos[0]+1, pos[1]-1], [pos[0]-1, pos[1]-1])
                        self.UF_join([pos[0]+1, pos[1]], [pos[0]-1, pos[1]-1])
                        self.UF_join([pos[0]+1, pos[1]], [pos[0]-1, pos[1]])
                        # print("Sai 4")


        for x in range (2, len(mapa)-2, 1):
            for y in range (2, len(mapa[0])-2, 1):
                pos = [x, y]
                if mapa[pos[0]][pos[1]] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]] in tilesQuePodeIr and mapa[pos[0]+2][pos[1]] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]] in tilesQuePodeIr and mapa[pos[0]-2][pos[1]] in tilesQuePodeIr and mapa[pos[0]][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]][pos[1]+2] in tilesQuePodeIr and mapa[pos[0]][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]][pos[1]-2] in tilesQuePodeIr:
                    if mapa[pos[0]+1][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]+2][pos[1]+2] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]+2] in tilesQuePodeIr and mapa[pos[0]+2][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]-2][pos[1]-2] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]-2] in tilesQuePodeIr and mapa[pos[0]-2][pos[1]-1] in tilesQuePodeIr:
                        self.UF_join([pos[0]-1, pos[1]-1], [pos[0]+1, pos[1]+1])

                    if mapa[pos[0]-1][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]-2][pos[1]+2] in tilesQuePodeIr and mapa[pos[0]-1][pos[1]+2] in tilesQuePodeIr and mapa[pos[0]-2][pos[1]+1] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]-1] in tilesQuePodeIr and mapa[pos[0]+2][pos[1]-2] in tilesQuePodeIr and mapa[pos[0]+1][pos[1]-2] in tilesQuePodeIr and mapa[pos[0]+2][pos[1]-1] in tilesQuePodeIr:
                        self.UF_join([pos[0]+1, pos[1]-1], [pos[0]-1, pos[1]+1])



        print("VAMOS ABACATAR 3")
        # Lista de connection tiles
        centerMod4Y = (yMod4+1)%4
        centerMod4X = (xMod4+1)%4
        connectionTilesTypes = ['o', 'r', 'g']
        connectionTilesList = []
        for x in range (1, len(mapa)-1, 1):
            for y in range (1, len(mapa[0])-1, 1):
                if(y%4 != centerMod4Y): continue
                if(x%4 != centerMod4X): continue
                if not (mapa[x-1][y-1] == mapa[x-1][y+1] and mapa[x-1][y-1] == mapa[x+1][y-1] and mapa[x-1][y-1] == mapa[x+1][y+1]): continue
                if(mapa[x-1][y-1] in connectionTilesTypes):
                    passagem = []
                    if(str(mapa[x][y+2]) != '1' and str(mapa[x][y+2]) != 'w'):
                        passagem.append([x, y+2])
                    if(str(mapa[x][y-2]) != '1' and str(mapa[x][y-2]) != 'w'):
                        passagem.append([x, y-2])
                    if(str(mapa[x+2][y]) != '1' and str(mapa[x+2][y]) != 'w'):
                        passagem.append([x+2, y])
                    if(str(mapa[x-2][y]) != '1' and str(mapa[x-2][y]) != 'w'):
                        passagem.append([x-2, y])
                    if(len(passagem) != 2): continue
                    connectionTilesList.append([[x, y], passagem])
        for x in [0, 1, 2]:
            for y in [0, 1, 2]:
                self.UF_join([startPos[0]+x, startPos[1]+y], ['A', 1])

        # Td borda por ['A', 0]
        for x in range (len(mapa)):
            for y in range (len(mapa[0])):
                if (x < menorX or x > maiorX) or (y < menorY or y > maiorY): self.UF_join([x, y], ['A', 0])
                

        print("VAMOS ABACATAR 3A")
        travado = False
        while(not travado):
            # print("dentro do while?")
            travado = True
            for i in range(len(connectionTilesList)):
                passagem1, passagem2 = connectionTilesList[i][1]
                if(self.UF_find(passagem1) == ['A', 1]):
                    self.UF_join(passagem2, ['A', 2])
                    travado = False
                    connectionTilesList.pop(i)
                    break
                if(self.UF_find(passagem2) == ['A', 1]):
                    self.UF_join(passagem1, ['A', 2])
                    travado = False
                    connectionTilesList.pop(i)
                    break
                if(self.UF_find(passagem1) == ['A', 2]):
                    self.UF_join(passagem2, ['A', 1])
                    travado = False
                    connectionTilesList.pop(i)
                    break
                if(self.UF_find(passagem2) == ['A', 2]):
                    self.UF_join(passagem1, ['A', 1])
                    travado = False
                    connectionTilesList.pop(i)
                    break
        print("VAMOS ABACATAR 5")
        # Trocar td q for area 4 p asterisco
        # mapa = [row[::-1] for row in mapa]
        mapaFinal = [row[:] for row in mapa]
        for x in range (len(mapa)):
            for y in range (len(mapa[0])):
                if self.UF_find([x, y]) == ['A', 2]:
                    mapaFinal[x][y] = '*'
                # if self.UF_find([x, y]) == ['A', 0]:
                #     mapaFinal[x][y] = '+'



        def trocaFinal():
            for x in range (len(mapaFinal)):
                for y in range (len(mapaFinal[0])):
                    if str(mapaFinal[x][y]) == 'w':
                        mapaFinal[x][y] = '0'
            for x in range (len(mapaFinal)):
                for y in range (len(mapaFinal[0])):
                    if str(mapaFinal[x][y]) == '*' and str(mapaFinal[x+1][y]) == '1':
                        mapaFinal[x+1][y] = '-'
                    if str(mapaFinal[x][y]) == '*' and str(mapaFinal[x][y+1]) == '1':
                        mapaFinal[x][y+1] = '-'
                    if str(mapaFinal[x][y]) == '*' and str(mapaFinal[x-1][y]) == '1':
                        mapaFinal[x-1][y] = '-'
                    if str(mapaFinal[x][y]) == '*' and str(mapaFinal[x][y-1]) == '1':
                        mapaFinal[x][y-1] = '-'

            for x in range (len(mapaFinal)):
                for y in range (len(mapaFinal[0])):
                    cont=0
                    if str(mapaFinal[x][y]) == '1':
                        if str(mapaFinal[x+1][y]) == '-' or str(mapaFinal[x+1][y]) == '*': 
                            cont +=1
                        if str(mapaFinal[x-1][y]) == '-' or str(mapaFinal[x-1][y]) == '*':
                            cont += 1
                        if str(mapaFinal[x][y+1]) == '-' or str(mapaFinal[x][y+1]) == '*':
                            cont += 1
                        if str(mapaFinal[x][y-1]) == '-' or str(mapaFinal[x][y-1]) == '*':
                            cont += 1
                    if cont >= 2:
                        mapaFinal[x][y] = '%'

            for x in range (len(mapaFinal)):
                for y in range (len(mapaFinal[0])):
                    if str(mapaFinal[x][y]) == '-' or str(mapaFinal[x][y]) == '%':
                        # self.UF_join([x, y], ['A', 2])
                        mapaFinal[x][y] = '*'
            for x in range (centerMod4X, len(mapaFinal)-1, 4):
                for y in range (centerMod4Y, len(mapaFinal[0])-1, 4):

                    if mapaFinal[x-1][y-1] == '*' or mapaFinal[x][y-1] == '*' or mapaFinal[x+1][y-1] == '*'   or   mapaFinal[x-1][y] == '*' or mapaFinal[x][y] == '*' or mapaFinal[x+1][y] == '*'   or   mapaFinal[x-1][y+1] == '*' or mapaFinal[x][y+1] == '*' or mapaFinal[x+1][y+1] == '*':
                        mapaFinal[x-2][y-2], mapaFinal[x-1][y-2], mapaFinal[x][y-2], mapaFinal[x+1][y-2], mapaFinal[x+2][y-2] = '*', '*', '*', '*', '*'
                        mapaFinal[x-2][y-1], mapaFinal[x-1][y-1], mapaFinal[x][y-1], mapaFinal[x+1][y-1], mapaFinal[x+2][y-1] = '*', '*', '*', '*', '*'
                        mapaFinal[x-2][y], mapaFinal[x-1][y], mapaFinal[x][y], mapaFinal[x+1][y], mapaFinal[x+2][y] = '*', '*', '*', '*', '*'
                        mapaFinal[x-2][y+1], mapaFinal[x-1][y+1], mapaFinal[x][y+1], mapaFinal[x+1][y+1], mapaFinal[x+2][y+1] = '*', '*', '*', '*', '*'
                        mapaFinal[x-2][y+2], mapaFinal[x-1][y+2], mapaFinal[x][y+2], mapaFinal[x+1][y+2], mapaFinal[x+2][y+2] = '*', '*', '*', '*', '*'
        def dentroDeConnectionTile(pos, matrix):
            dentro = False
            pos = [x, y]
            if str(matrix[pos[0]][pos[1]]) in tilesQuePodeIr:
                if str(matrix[pos[0]+1][pos[1]]) in connectionTilesTypes and str(matrix[pos[0]-1][pos[1]]) in connectionTilesTypes:
                    dentro = True 
                if str(matrix[pos[0]][pos[1]+1]) in connectionTilesTypes and str(matrix[pos[0]][pos[1]-1]) in connectionTilesTypes:
                    dentro = True
                if str(matrix[pos[0]-1][pos[1]-1]) in connectionTilesTypes and str(matrix[pos[0]+1][pos[1]-1]) in connectionTilesTypes and str(matrix[pos[0]+1][pos[1]+1]) in connectionTilesTypes and str(matrix[pos[0]-1][pos[1]+1]) in connectionTilesTypes:
                    dentro = True
            elif str(matrix[pos[0]][pos[1]]) in connectionTilesTypes:
                dentro = True 
            return dentro
        
        for x in range (len(mapaFinal)):
            for y in range (len(mapaFinal[0])):
                if(self.UF_find([x, y])[0] != 'A'):
                    if(dentroDeConnectionTile([x, y], mapaFinal)): self.UF_join([x, y], ['A', 1])

        for x in range (len(mapaFinal)):
            for y in range (len(mapaFinal[0])):
                if(self.UF_find([x, y])[0] != 'A'):
                    adjacents = []
                    adjacents.extend([self.UF_find([x-1, y-1]), self.UF_find([x, y-1]), self.UF_find([x+1, y-1])  ,  self.UF_find([x-1, y]), self.UF_find([x+1, y])  ,  self.UF_find([x-1, y+1]), self.UF_find([x, y+1]), self.UF_find([x+1, y+1]) ])
                    if(['A', 1] in adjacents or ['A', 0] in adjacents): self.UF_join([x, y], ['A', 3])
                    elif (['A', 2] in adjacents): self.UF_join([x, y], ['A', 4])
                    
        for x in range (len(mapaFinal)):
            for y in range (len(mapaFinal[0])):
                if(self.UF_find([x, y])[0] != 'A'):
                    adjacents = []
                    adjacents.extend([self.UF_find([x-1, y-1]), self.UF_find([x, y-1]), self.UF_find([x+1, y-1])  ,  self.UF_find([x-1, y]), self.UF_find([x+1, y])  ,  self.UF_find([x-1, y+1]), self.UF_find([x, y+1]), self.UF_find([x+1, y+1]) ])
                    if(['A', 3] in adjacents): self.UF_join([x, y], ['A', 3])
                    # elif (['A', 2] in adjacents): self.UF_join([x, y], ['A', 5])
        for x in range (len(mapaFinal)):
            for y in range (len(mapaFinal[0])):
                if(self.UF_find([x, y])[0] != 'A'):
                    adjacents = []
                    adjacents.extend([self.UF_find([x-1, y-1]), self.UF_find([x, y-1]), self.UF_find([x+1, y-1])  ,  self.UF_find([x-1, y]), self.UF_find([x+1, y])  ,  self.UF_find([x-1, y+1]), self.UF_find([x, y+1]), self.UF_find([x+1, y+1]) ])
                    if(['A', 4] in adjacents): self.UF_join([x, y], ['A', 4])

        for x in range (len(mapa)):
            for y in range (len(mapa[0])):
                if self.UF_find([x, y]) == ['A', 4]:
                    mapaFinal[x][y] = '*'

        # listita = []
        # for x in range (len(mapaFinal)):
        #     for y in range (len(mapaFinal[0])):
        #         if(self.UF_find([x, y])[0] != 'A'):
        #             if(dentroDeConnectionTile([x, y], mapaFinal)): continue
        #             listita.append([x, y])

        # loopsSemAtualizar = 0
        # while(len(listita)):
        #     if(loopsSemAtualizar > len(listita)): break # entrou em loop infinito
        #     x, y = listita[0]
        #     listita.pop(0)
        #     temArea4Adj = False
        #     if (self.UF_find([x+1, y]) == ['A', 1]) or (self.UF_find([x+1, y]) == ['A', 0]):
        #         loopsSemAtualizar = 0
        #         continue # A gente n vai trocar por nada
        #     if (self.UF_find([x, y+1]) == ['A', 1]) or (self.UF_find([x, y+1]) == ['A', 0]):
        #         loopsSemAtualizar = 0
        #         continue # A gente n vai trocar por nada
        #     if (self.UF_find([x-1, y]) == ['A', 1]) or (self.UF_find([x-1, y]) == ['A', 0]):
        #         loopsSemAtualizar = 0
        #         continue # A gente n vai trocar por nada
        #     if (self.UF_find([x, y-1]) == ['A', 1]) or (self.UF_find([x, y-1]) == ['A', 0]):
        #         loopsSemAtualizar = 0
        #         continue # A gente n vai trocar por nada
            
        #     if(self.UF_find([x+1, y]) == ['A', 2]):
        #         temArea4Adj = True
        #     if(self.UF_find([x, y+1]) == ['A', 2]):
        #         temArea4Adj = True
        #     if(self.UF_find([x-1, y]) == ['A', 2]):
        #         temArea4Adj = True
        #     if(self.UF_find([x, y-1]) == ['A', 2]):
        #         temArea4Adj = True

        #     if temArea4Adj:
        #         self.UF_join([x, y], ['A', 2])
        #         loopsSemAtualizar = 0
        #     else:
        #         listita.append([x, y])
        #         loopsSemAtualizar += 1

        # for x in range (len(mapaFinal)):
        #     for y in range (len(mapaFinal[0])):
        #         if self.UF_find([x, y]) == ['A', 0]: print(' ', end = " ")
        #         elif str(mapaFinal[x][y]) == '0': print('.', end = " ")
        #         else: print(mapaFinal[x][y], end = " ")
        #     print(" ")
        # print(" ")
        # for x in range (len(mapaFinal)):
        #     for y in range (len(mapaFinal[0])):
        #         if self.UF_find([x, y]) == ['A', 2]: mapaFinal[x][y] = '*'
        # # Aki

        # # for row in mapaFinal:
        # #     print(' '.join(row))

        for x in range (len(mapaFinal)):
            for y in range (len(mapaFinal[0])):
                # if(x == 28 and y == 13): print('A', end = ' ')
                # el
                if x == 13 and y == 27: print('A', end = " ")
                elif mapaFinal[x][y] == '*' and x%4 == centerMod4X and y%4 == centerMod4Y: print('X', end = " ")
                elif self.UF_find([x, y]) == ['A', 0] and str(mapaFinal[x][y]) == '0': print(' ', end = " ")
                elif str(mapaFinal[x][y]) == '0': print('.', end = " ")
                else: print(mapaFinal[x][y], end = " ")
            print(" ")

        trocaFinal()
        
        for x in range (len(mapaFinal)):
            for y in range (len(mapaFinal[0])):
                if mapaFinal[x][y] == '*' and x%4 == centerMod4X and y%4 == centerMod4Y: print('X', end = " ")
                # elif self.UF_find([x, y]) == ['A', 1]: print('+', end = " ")
                elif self.UF_find([x, y]) == ['A', 0] and str(mapaFinal[x][y]) == '0': print(' ', end = " ")
                elif str(mapaFinal[x][y]) == '0': print('.', end = " ")
                else: print(mapaFinal[x][y], end = " ")
            print(" ")

        return mapaFinal
    

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
                    [[a, b], c, d] = self.extra_map[x, y][i]

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
                if self.seen_map[x, np.size(self.seen_map, 1)-1-y][0] == 3:
                    transpose[y, x] = 1

        s = 'explored.png'
        M = abs(transpose)*255
        cv2.imwrite(s, M)

        return 
        

    def dist_coords(self, a, b):
        dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        return dist