import cv2
from collections import deque
import numpy as np
import os, shutil
import math

r = 163
g = 73
b = 164
g_r = math.atan(r/g) 
rg_b = math.atan(b/math.sqrt(r^2 + g^2))

def is_wall(color):
    yes = False

    if abs(color[1]-2*color[0]) < 10 and abs(color[2]-color[1]-(color[1]-color[0])/6) < 3:
        yes = True

    if (color[0] == color[1] and color[1] == color[2]):
        yes = False
    if (color[2]-color[1] > abs(color[1]-color[0])):
        yes = False

    return yes

cont_img = 0
tick_count = 0

def triangle_area(p1, p2, p3): # Area do triangulo usando cordenadas p1, p2, p3 com determinante de matriz
    # print(p1, p2, p3)
    return abs((p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])) / 2.0)

def visvalingam_whyatt(coords):
    n = len(coords)
    areas = [] # area[i][0] = Propria area dele, area[i][1] = [x, y], coordenadas do vertice

    for i in range(0, len(coords)):
        area = triangle_area(coords[(i-1)%n], coords[i], coords[(i+1)%n])
        areas.append([area, coords[i]])
    
    while len(areas) > 4:
        n = len(areas)
        # Pega a area minima
        minIndex = 0
        for i in range(0, len(areas)):
            if areas[i][0] < areas[minIndex][0]: minIndex = i
        
        # Atualiza areas dos adjacentes
        areas[(minIndex-1)%n][0] = triangle_area(areas[(minIndex-2)%n][1], areas[(minIndex-1)%n][1], areas[(minIndex+1)%n][1])
        areas[(minIndex+1)%n][0] = triangle_area(areas[(minIndex-1)%n][1], areas[(minIndex+1)%n][1], areas[(minIndex+2)%n][1])

        del areas[minIndex%n]
    return [areas[0][1], areas[1][1], areas[2][1], areas[3][1]]

def perpendicular_line_coefficients(Xa, Ya, Xb, Yb):
    # Calculate the slope of the original line
    if Xb == Xa:
        return (1, 0, -Xa)
    elif Yb == Ya:
        return (0, 1, -Ya)
    else:
        m = (Yb - Ya) / (Xb - Xa)
        
    # Slope of the perpendicular line
    perpendicular_slope = -1 / m
    
    # Coefficients A, B, and C for the perpendicular line in the form Ax + By + C = 0
    A = perpendicular_slope
    B = -1
    C = Ya - perpendicular_slope * Xa
    
    return (A, B, C)

def find_intersection_coef_points(A, B, C, Xc, Yc, Xd, Yd):
    # Calculate the coefficients for the first line
    A1 = A
    B1 = B
    C1 = C

    # Calculate the coefficients for the second line
    A2 = Yd - Yc
    B2 = Xc - Xd
    C2 = A2 * Xc + B2 * Yc

    # Calculate the determinant
    determinant = A1 * B2 - A2 * B1

    if determinant == 0:
        # The lines are parallel
        return [None, None]  # No intersection point

    # Calculate the intersection point
    X = (C1 * B2 - C2 * B1) / determinant
    Y = (A1 * C2 - A2 * C1) / determinant

    return [X, Y]

def find_intersection(Xa, Ya, Xb, Yb, Xc, Yc, Xd, Yd):
    # Calculate the coefficients for the first line
    A1 = Yb - Ya
    B1 = Xa - Xb
    C1 = A1 * Xa + B1 * Ya

    # Calculate the coefficients for the second line
    A2 = Yd - Yc
    B2 = Xc - Xd
    C2 = A2 * Xc + B2 * Yc

    # Calculate the determinant
    determinant = A1 * B2 - A2 * B1

    if determinant == 0:
        # The lines are parallel
        return [None, None]  # No intersection point

    # Calculate the intersection point
    X = (C1 * B2 - C2 * B1) / determinant
    Y = (A1 * C2 - A2 * C1) / determinant

    return [X, Y]

def find_square_vertices(v1, v2):
    # Calculate the midpoint
    mid_x = (v1[0] + v2[0]) / 2
    mid_y = (v1[1] + v2[1]) / 2

    # Calculate the vector from v1 to v2
    vec_x = v2[0] - v1[0]
    vec_y = v2[1] - v1[1]

    # Perpendicular vector (rotated by 90 degrees)
    perp_vec_x = -vec_y
    perp_vec_y = vec_x

    # Calculate half of the length of the side of the square
    half_side_length = np.sqrt(perp_vec_x**2 + perp_vec_y**2) / 2

    # Normalize the perpendicular vector
    norm = np.sqrt(perp_vec_x**2 + perp_vec_y**2)
    perp_vec_x /= norm
    perp_vec_y /= norm

    # Calculate the other two vertices
    v3 = (mid_x + perp_vec_x * half_side_length, mid_y + perp_vec_y * half_side_length)
    v4 = (mid_x - perp_vec_x * half_side_length, mid_y - perp_vec_y * half_side_length)

    return v3, v4

def find_vertex(img):
    mid = [64, 20] # [x, y]
    visited = []
    queue = []
    visited.append(mid)
    queue.append(mid)

    vertices = []
    xMenorTopo = mid[0]
    xMaiorTopo = mid[0]
    xMenorBase = mid[0]
    xMaiorBase = mid[0]
    esqCima = mid
    esqBaixo = mid
    dirCima = mid
    dirBaixo = mid
    cimaEsq = mid
    cimaDir = mid
    baixoEsq = mid 
    baixoDir = mid 

    while queue:
        atual = queue.pop(0)
        # print("Atual:", atual)
        x_atual = atual[0]
        y_atual = atual[1]

        # 

        if y_atual == 0:
            xMenorTopo = min(x_atual, xMenorTopo)
            xMaiorTopo = max(x_atual, xMaiorTopo)
        if y_atual == 39:
            xMenorBase = min(x_atual, xMenorBase)
            xMaiorBase = max(x_atual, xMaiorBase)

        # Caso mais esquerda, em cima
        if x_atual < esqCima[0] or (x_atual == esqCima[0] and y_atual <= esqCima[1]):
            esqCima = [x_atual, y_atual]
        # Caso mais a esquerda, embaixo
        if x_atual < esqBaixo[0] or (x_atual == esqBaixo[0] and y_atual >= esqBaixo[1]):
            esqBaixo = [x_atual, y_atual]
        # Caso direita, cima
        if x_atual > dirCima[0] or (x_atual == dirCima[0] and y_atual <= dirCima[1]):
            dirCima = [x_atual, y_atual]
        # Caso direita, baixo 
        if x_atual > dirBaixo[0] or (x_atual == dirBaixo[0] and y_atual >= dirBaixo[1]):
            dirBaixo = [x_atual, y_atual]

        # Caso cima, esquerda
        if y_atual < cimaEsq[1] or (y_atual == cimaEsq[1] and x_atual <= cimaEsq[0]):
            cimaEsq = [x_atual, y_atual]
        # Caso cima, direita 
        if y_atual < cimaDir[1] or (y_atual == cimaDir[1] and x_atual >= cimaDir[0]):
            cimaDir = [x_atual, y_atual]
        # Caso baixo, esquerda
        if y_atual > baixoEsq[1] or (y_atual == baixoEsq[1] and x_atual <= baixoEsq[0]):
            baixoEsq = [x_atual, y_atual]
        # Caso baixo, direita 
        if y_atual > baixoDir[1] or (y_atual == baixoDir[1] and x_atual >= baixoDir[0]):
            baixoDir = [x_atual, y_atual]

        # Checa se é vertice
        contAdj = 0

        if(y_atual < 39):
            px = [img.item(y_atual+1, x_atual, 2), img.item(y_atual+1, x_atual, 1), img.item(y_atual+1, x_atual, 0)]
            if(not is_wall(px)): contAdj += 1
        if(y_atual > 0):
            px = [img.item(y_atual-1, x_atual, 2), img.item(y_atual-1, x_atual, 1), img.item(y_atual-1, x_atual, 0)]
            if(not is_wall(px)): contAdj += 1
        if(x_atual < 127):
            px = [img.item(y_atual, x_atual+1, 2), img.item(y_atual, x_atual+1, 1), img.item(y_atual, x_atual+1, 0)]
            if(not is_wall(px)): contAdj += 1
        if(x_atual > 0):
            px = [img.item(y_atual, x_atual-1, 2), img.item(y_atual, x_atual-1, 1), img.item(y_atual, x_atual-1, 0)]
            if(not is_wall(px)): contAdj += 1

        if(contAdj == 1 or contAdj == 2):
            angleToCenter = math.atan2(y_atual-mid[1], x_atual-mid[0])
            vertices.append([angleToCenter, [x_atual, y_atual]])
            
        for y in range(-1, 2, 1):
            for x in range(-1, 2, 1):
                x_next = x_atual+x
                y_next = y_atual+y
                if(x_next >= 0 and y_next >= 0 and x_next < 128 and y_next < 40):
                    px = [img.item(y_next, x_next, 2), img.item(y_next, x_next, 1), img.item(y_next, x_next, 0)]
                    if not is_wall(px) and [x_next, y_next] not in visited:
                        visited.append([x_next, y_next])
                        queue.append([x_next, y_next])
    if(len(vertices) == 0): return [[0, 0], [0, 0], [0, 0], [0, 0]]

    cont_column_anterior = 0
    for xi in range (esqCima[0], dirCima[0]+1, 1):
        cont_column_atual = 0
        for yi in range (cimaEsq[1], baixoEsq[1]+1, 1):
            px = [img.item(yi, xi, 2), img.item(yi, xi, 1), img.item(yi, xi, 0)]
            if not is_wall(px):
                cont_column_atual += 1
        if(cont_column_anterior != 0 and abs(cont_column_atual - cont_column_anterior) >= 10):
            vertices.sort()
            sortedVertices = []
            for i in vertices:
                sortedVertices.append(i[1])
            # print(sortedVertices)
            print("d")
            return visvalingam_whyatt(sortedVertices)
        cont_column_anterior = cont_column_atual
 
    if(xMaiorTopo - xMenorTopo >= 1 or xMaiorBase - xMenorBase >= 1):
        v3, v4 = find_square_vertices(esqBaixo, dirCima)
        angleToCenter = math.atan2(v3[1]-mid[1], v3[0]-mid[0])
        vertices.append([angleToCenter, [v3[0], v3[1]]])

        angleToCenter = math.atan2(v4[1]-mid[1], v4[0]-mid[0])
        vertices.append([angleToCenter, [v4[0], v4[1]]])

    # if(xMaiorBase - xMenorBase >= 1):
    #     v3, v4 = find_square_vertices(esqBaixo, dirCima)
        
    vertices.sort()
    sortedVertices = []
    for i in vertices:
        sortedVertices.append(i[1])
    # print(sortedVertices)

    return visvalingam_whyatt(sortedVertices)

imgList = []

def identifyVictim(img):
    # img = np.array(np.frombuffer(img_data, np.uint8).reshape((camera_front.getHeight(), camera_front.getWidth(), 4)))
    imgNp = np.array(np.frombuffer(img, np.uint8).reshape((250, 250)))
    # x passa por -10, 11, 1
    #   Y ^^
    #       [125+X, 125+Y]
    temPreto = False
    for y in range(-10, 11, 1):
        for x in range(-10, 11, 1):
            px = imgNp.item(125+y, 125+x)
            if(px <= 1):
                temPreto = True
    if not temPreto: return 'U'
    else:
        # Branco
        # Preto

        # Para linhas
        contadorBrancoPreto = 0
        anteriorPreto = False
        for x in range(249):
            # Se tem algm preto na linha
            temPreto = False
            for y in range(115, 136, 1):
                px = img.item(y, x)
                if(px <= 1):
                    temPreto = True
            if(temPreto == True and anteriorPreto == False):
                contadorBrancoPreto += 1
            anteriorPreto = temPreto
        if(contadorBrancoPreto >= 3):
            return 'S'
        
        # Para colunas
        contadorBrancoPreto = 0
        anteriorPreto = False

        for y in range(249):
            # Se tem algm preto na linha
            temPreto = False
            for x in range(115, 136, 1): 
                px = img.item(y, x)
                if(px <= 1):
                    temPreto = True
            if(temPreto == True and anteriorPreto == False):
                contadorBrancoPreto += 1
            anteriorPreto = temPreto
        if(contadorBrancoPreto >= 3):
            return 'S'
            
        return 'H'

def identifyHazmat(img):
    imgNp = np.array(np.frombuffer(img, np.uint8).reshape((40, 128, 3)))
    resolution = 250
    
    # Warping
    vertexes = find_vertex(imgNp)
    imgComBorda = cv2.copyMakeBorder(img, 40, 40, 0, 0, cv2.BORDER_CONSTANT, None, value = [255, 255, 255])

    vertexes[0][1] += 40
    vertexes[1][1] += 40
    vertexes[2][1] += 40
    vertexes[3][1] += 40

    # print(fileName, vertexes)
    if vertexes[0] == vertexes[1] and vertexes[1] == vertexes[2] and vertexes[2] == vertexes[3]: return "No victim"
    pts1 = np.float32(vertexes)
    pts2 = np.float32([[0, 0], [resolution, 0], [resolution, resolution], [0, resolution]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    warped = cv2.warpPerspective(imgComBorda, M, (resolution, resolution))
    
    warpedNp = np.array(np.frombuffer(warped, np.uint8).reshape((250, 250, 3)))
    warped_hsv = cv2.cvtColor(warpedNp, cv2.COLOR_BGR2HSV)

    # Define the ranges of the colors
    lower_red = np.array([150, 100, 100])
    upper_red = np.array([180, 255, 255])

    lower_yellow = np.array([10, 100, 100])
    upper_yellow = np.array([50, 255, 255])
    
    # Mask 
    mask_red = cv2.inRange(warped_hsv, lower_red, upper_red)
    mask_yellow = cv2.inRange(warped_hsv, lower_yellow, upper_yellow)

    #count the number of pixels from each mask 
    red = np.sum(mask_red == 255)
    yellow = np.sum(mask_yellow == 255)

    #verify the masks to check Flamables and Organics
    if yellow > 100:
        return 'O'
    elif red > 100:
        return 'F'
    
    warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    
    # Binary value for corrosive
    _, warped_binary = cv2.threshold(warped_gray, 200, 255, cv2.THRESH_BINARY)
    warped_binaryNp = np.array(np.frombuffer(warped_binary, np.uint8).reshape((250, 250)))
    percentage = np.sum(warped_binaryNp == 0)/62500.0
    if(percentage > 0.70):
        return 'C'
    
    # Binary value for poison
    _, warped_binary = cv2.threshold(warped_gray, 100, 255, cv2.THRESH_BINARY)
    warped_binaryNp = np.array(np.frombuffer(warped_binary, np.uint8).reshape((250, 250)))
    percentage = np.sum(warped_binaryNp == 0)/62500.0
    
    if(percentage < 0.20):
        return 'P'
    else:
        letter = identifyVictim(warped_binary)
        print("LETTTER", letter)
        return letter
