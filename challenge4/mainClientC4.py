"""
	RMI, 2021
	
	Authors: 
	Rodrigo Santos, nº mec 89180
	Rui Santos, nº mec 89293
"""

import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import numpy as np
import random

from tree_search import *
from ciberRato import CiberRato

from a_star import *

CELLROWS=7
CELLCOLS=14

movement = 0 # 0 - frente, 1 - cima, 2 - baixo, 3 - tras
init_GPS = [0,0] # initial GPS Position
current_GPS = [0,0] # current GPS position
turning = 0  # turning flag

rotation_counter = 0 # counter to rotate faster
ongoing = False # if the mouse is going to a position
decimal_init_GPS = [0,0]
path_list = []  # movement path list
goingToDest = 0
dontGoNow = False

# saves the previous value given to the motors
previous_power_l = 0
previous_power_r = 0
est_pos = [0,0] # estimated position

w, h = 55, 27  # weight, height of mapp
mapp = [[0 for x in range(h)] for y in range(w)] 
mapp[27][13] = "I"
scale= []  # scale to translate GPS to mapp coordinates

Point1 = [] # 1st ground target
Point2 = [] # 2nd ground target
found = 0 # best path found flag

outputFile = "mapping.out" # nome do output file

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        
        mapings = []
        
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        global init_GPS
        global current_GPS
        global scale
        global decimal_init_GPS

        global est_pos
        
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        self.readSensors()
        # init_GPS = [(self.measures.x), (self.measures.y)]
        init_GPS = [(est_pos[0]), (est_pos[1])]
        current_GPS = init_GPS
        decimal_init_GPS = [init_GPS[0]%1,init_GPS[1]%1]
        
        scale = [init_GPS[0] - 27, init_GPS[1] - 13]
        init_GPS = [(self.measures.x), (self.measures.y)]
        
        self.paintMapp()
        
        while True:
            self.readSensors()
            
            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()


    def wander(self):
        global movement
        global turning
        global rotation_counter
        global ongoing
        global current_GPS
        global path_list
        global goingToDest
        global dontGoNow
        global Point1
        global Point2
        global found

        global est_pos
        
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        # self.translateGPStoMappCoordAndPaint((self.measures.x), (self.measures.y), "X")
        self.translateGPStoMappCoordAndPaint((est_pos[0]), (est_pos[1]), "X")
        current_x, current_y = self.translateGPStoMappCoord(current_GPS[0], current_GPS[1])

        if ongoing:
            moved = self.moveforward()
            if not moved:
                current_GPS = [(est_pos[0]), (est_pos[1])]
                self.paintMapp()
                self.checkGround()
                # if Point1 != [] and Point2 != [] and not found:
                #     done = self.calcBestPath_init_P1_P2()
                #     if done and self.getNearestSpace(current_x, current_y) == []:
                #         found = 1
                #         print('Done!')
                #         self.finish()
                #         sys.exit()
                ongoing = False
        else:

            if turning == 0:
                if path_list:
                    if len(path_list) == 1:
                        path_list.pop()
                        goingToDest = 0
                    else:
                        move = path_list[0]

                        if movement != move:
                            movement = move
                            turning = 1
                        else:
                            path_list.pop(0)
                            path_list.pop(0)

                if goingToDest == 0:
                    # current_x, current_y = self.translateGPStoMappCoord(current_GPS[0], current_GPS[1])
                    # If there is a space near the mouse, go for it!
                    if not ((mapp[current_x][current_y+1] == " " and movement == 1) or (mapp[current_x][current_y-1] == " " and movement == 2) or (mapp[current_x+1][current_y] == " " and movement == 0) or (mapp[current_x-1][current_y] == " " and movement == 3)):
                        if mapp[current_x][current_y+1] == " ":
                            if movement != 1:
                                turning = 1
                                movement = 1
                        elif mapp[current_x][current_y-1] == " ":
                            if movement != 2:
                                turning = 1
                                movement = 2
                        elif mapp[current_x+1][current_y] == " ":
                            if movement != 0:
                                turning = 1
                                movement = 0
                        elif mapp[current_x-1][current_y] == " ":
                            if movement != 3:
                                turning = 1
                                movement = 3
                            
                        else:   
                            goingToDest = self.getPathToSpace(current_x, current_y) # Must return 1 to go!

                            if goingToDest == 2:
                                self.writeMapToFile()
                                sys.exit()

                            dontGoNow = True

                if turning == 0 and not dontGoNow:
                    if self.measures.irSensor[center_id] > 1.1:
                        self.checksides()
                        
                    else:
                        self.moveforward()
                
                dontGoNow = False
            else:
                # No need to use the rotation direction var
                if movement == 1 and (self.measures.compass >= 92 or self.measures.compass <= 88):
                    if (self.measures.compass >= -92 and self.measures.compass <= -88):
                        rotation_counter = -5
                    if self.measures.compass < -90 or (self.measures.compass < 180 and self.measures.compass > 90):
                        self.turnRight()
                    else:
                        self.turnLeft()
                elif movement == 1:
                    rotation_counter = 0
                    turning = 0
                    
                elif movement == 2 and (self.measures.compass <= -92 or self.measures.compass >= -88):
                    if (self.measures.compass <= 92 and self.measures.compass >= 88):
                        rotation_counter = -5
                    if self.measures.compass > 90 or (self.measures.compass > -180 and self.measures.compass < -90):
                        self.turnLeft()
                    else:
                        self.turnRight()
                elif movement == 2:
                    rotation_counter = 0
                    turning = 0
                    
                elif movement == 0 and (self.measures.compass <= -2 or self.measures.compass >= 2):
                    if ((self.measures.compass <= -177 and self.measures.compass >= -180) or (self.measures.compass >= 177 and self.measures.compass <= 180)):
                        rotation_counter = -5
                    if self.measures.compass >= 2:
                        self.turnRight()
                    else:
                        self.turnLeft()
                elif movement == 0:
                    rotation_counter = 0
                    turning = 0
                    
                elif movement == 3 and ((self.measures.compass >= -177 and self.measures.compass <= 0) or (self.measures.compass <= 177 and self.measures.compass >= 0)):
                    if (self.measures.compass <= 2 and self.measures.compass >= -2):
                        rotation_counter = -5
                    if (self.measures.compass <= 177 and self.measures.compass >= 0):
                        self.turnLeft()
                    else:
                        self.turnRight()
                else:
                    rotation_counter = 0
                    turning = 0
    

    def moveforward(self):
        global current_GPS
        global movement
        global ongoing

        global est_pos
        global previous_power_l
        global previous_power_r

        straight_speed = 0.15
        min_distance = 2 - 0.3
        # min_distance = 2
        ongoing = True
        
        # x_int = (abs(self.measures.x - current_GPS[0]))
        # y_int = (abs(self.measures.y - current_GPS[1]))
        x_int = (abs(est_pos[0] - current_GPS[0]))
        y_int = (abs(est_pos[1] - current_GPS[1]))

        # if ((x_int < min_distance) and (y_int < min_distance)) or not self.verifyDecimals(self.measures.x, self.measures.y):
        # if ((x_int < min_distance) and (y_int < min_distance)):
        print("")
        print((x_int < min_distance) and (y_int < min_distance))
        print(not self.verifyDecimals(est_pos[0], est_pos[1]))
        if ((x_int < min_distance) and (y_int < min_distance)) or not self.verifyDecimals(est_pos[0], est_pos[1]):
            if (min_distance - x_int < 0.15) or (min_distance - y_int < 0.15):
                self.driveMotors(0.04, 0.04)
                self.update_previous_motors(0.04, 0.04)
            #else:
            elif movement == 0:
                if self.measures.compass < -1: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                    self.update_previous_motors(0.11, 0.14)
                elif self.measures.compass > 1: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                    self.update_previous_motors(0.14, 0.11)
                else:
                    self.driveMotors(straight_speed, straight_speed)
                    self.update_previous_motors(straight_speed, straight_speed)

            elif movement == 1:
                if self.measures.compass < 89: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                    self.update_previous_motors(0.11, 0.14)
                elif self.measures.compass > 91: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                    self.update_previous_motors(0.14, 0.11)
                else:
                    self.driveMotors(straight_speed, straight_speed)
                    self.update_previous_motors(straight_speed, straight_speed)

            elif movement == 2:
                if self.measures.compass < -91: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                    self.update_previous_motors(0.11, 0.14)
                elif self.measures.compass > -89: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                    self.update_previous_motors(0.14, 0.11)
                else:
                    self.driveMotors(straight_speed, straight_speed)
                    self.update_previous_motors(straight_speed, straight_speed)

            elif movement == 3:
                if self.measures.compass <= 178 and self.measures.compass > 0: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                    self.update_previous_motors(0.11, 0.14)
                elif self.measures.compass >= -178 and self.measures.compass < 0: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                    self.update_previous_motors(0.14, 0.11)
                else:
                    self.driveMotors(straight_speed, straight_speed)
                    self.update_previous_motors(straight_speed, straight_speed)

            
            self.position_estimator(movement)
            return True
            
        else:
            # print("\nReal:")
            # print(self.measures.x-init_GPS[0])
            # print(self.measures.y-init_GPS[1])
            # print("Estimated:")
            # print(est_pos[0])
            # print(est_pos[1])

            self.driveMotors(-0.9*previous_power_l, -0.9*previous_power_r)
            return False

             
    def checksides(self):
        global movement
        global turning
        
        if self.measures.irSensor[2] <= 0.8:
            if movement == 0:
                movement = 2
            elif movement == 1:
                movement = 0
            elif movement == 2:
                movement = 3
            else:
                movement = 1
            turning = 1
            self.turnRight()
        elif self.measures.irSensor[1] <= 0.8:
            if movement == 0:
                movement = 1
            elif movement == 1:
                movement = 3
            elif movement == 2:
                movement = 0
            else:
                movement = 2
            turning = 1
            self.turnLeft()
        else:
            if movement == 0:
                movement = 3
            elif movement == 1:
                movement = 2
            elif movement == 2:
                movement = 1
            else:
                movement = 0
            turning = 1
            self.turnRight()
    

    def turnRight(self):
        global rotation_counter
        rotation_counter += 1
        if (rotation_counter < 6):
            self.driveMotors(0.15, -0.15)
        else:
            self.driveMotors(0.015, -0.015)
    
    def turnLeft(self):
        global rotation_counter
        rotation_counter += 1
        if (rotation_counter < 6):
            self.driveMotors(-0.15, 0.15)
        else:
            self.driveMotors(-0.015, 0.015)

    
    def translateGPStoMappCoord(self, x, y):
        global scale
        return round(x-scale[0]), round(y-scale[1])

    
    def translateGPStoMappCoordAndPaint(self, x, y, symbol):
        global mapp
        pos_x, pos_y = self.translateGPStoMappCoord(x, y)
        if mapp[pos_x][pos_y] == 0 or mapp[pos_x][pos_y] == " ":
            mapp[pos_x][pos_y] = symbol


    def verifyDecimals(self, x, y):
        global movement
        global decimal_init_GPS
        max_gap = 0.05
        if movement == 0 or movement == 3:
            print("x%1: "+str(x%1))
            # return abs((x%1) - decimal_init_GPS[0]) < max_gap
            return x%1 < max_gap or x%1 > 1-max_gap
        elif movement == 1 or movement == 2:
            print("y%1: "+str(y%1))
            # return abs((y%1) - decimal_init_GPS[1]) < max_gap
            return y%1 < max_gap or y%1 > 1-max_gap


    
    def getPathToSpace(self, current_x, current_y):
        global path_list
    
        # goal = self.getNearestSpace(current_x, current_y)
        # if not goal:
        #     return 2
        # print("\nNearest Space: "+str(goal))
        # zeros_map = self.get_zeros_map()

        # # ciberrato = CiberRato(zeros_map)
        # # nextPosition = SearchProblem(ciberrato, [current_x, current_y], goal)
        # # st = SearchTree(nextPosition)
        # # steps = st.search()
        # # path_list = self.path_to_movements(steps)

        # steps = search(zeros_map, 1, [current_x, current_y], [goal[0], goal[1]])
        # path_list = self.map_to_movements(steps, [current_x, current_y], goal)
        # print(path_list)

        path_list = self.getNearestSpace2(current_x, current_y)
        print(path_list)

        return 1

    
    def getNearestSpace(self, current_x, current_y):
        global mapp
        global w
        global h
        minDist = 900
        closestBreach = []

        for i in range(w):
            for j in range(h):
                if mapp[i][j] == ' ':
                    dist = (((abs(i - current_x)**2) + (abs(j - current_y)**2))**0.5)
                    if dist < minDist:
                        closestBreach  = [i, j]
                        minDist = dist

        return closestBreach


    def getNearestSpace2(self, current_x, current_y):
        minLen = 900
        path = []
        lsts = self.getAllSpaces()
        zeros_map = self.get_zeros_map()
        for lst in lsts:
            steps = search(zeros_map, 1, [current_x, current_y], [lst[0], lst[1]])
            path_list = self.map_to_movements(steps, [current_x, current_y], lst)
            if len(path_list) < minLen:
                path = path_list
                minLen = len(path_list)
        return path


    def getAllSpaces(self):
        lst = []
        for i in range(w):
            for j in range(h):
                if mapp[i][j] == ' ':
                    lst.append([i,j])
        return lst
        

    def get_zeros_map(self):
        global mapp
        global w
        global h

        return_map = [[1 for x in range(h)] for y in range(w)] 

        for i in range(w):
            for j in range(h):
                if mapp[i][j] == ' ' or mapp[i][j] == 'X'or mapp[i][j] == 'I':
                    return_map[i][j] = 0

        return return_map


    def path_to_movements(self, list):
        return_list = []
        for i in range(1,len(list)):
            aux_t = [list[i][0]-list[i-1][0], list[i][1]-list[i-1][1]]

            if aux_t[0] == -1:
                return_list.append(3)
            elif aux_t[0] == 1:
                return_list.append(0)
            elif aux_t[1] == 1:
                return_list.append(1)
            elif aux_t[1] == -1:
                return_list.append(2)

        return return_list


    def map_to_movements(self, list, start, goal):
        return_list = []
        current = start
        i = 0

        while current != goal:
            i += 1
            if list[current[0]+1][current[1]] == i:
                current = [current[0]+1,current[1]]
                return_list.append(0)
            elif list[current[0]-1][current[1]] == i:
                current = [current[0]-1,current[1]]
                return_list.append(3)
            elif list[current[0]][current[1]+1] == i:
                current = [current[0],current[1]+1]
                return_list.append(1)
            elif list[current[0]][current[1]-1] == i:
                current = [current[0],current[1]-1]
                return_list.append(2)

        return return_list
    

    def printMapp(self, m):
        
        zipped_rows = zip(*m)
        transpose_matrix = [list(row) for row in zipped_rows]
        for i in range (int(len(transpose_matrix)/2)):
            aux = transpose_matrix[len(transpose_matrix) - 1 - i]
            transpose_matrix[len(transpose_matrix) - 1 - i] = transpose_matrix[i]
            transpose_matrix[i] = aux 

        print('\n'.join([' '.join([str(cell) for cell in row]) for row in transpose_matrix]))

       
    def writeMapToFile(self):
        global mapp

        zipped_rows = zip(*mapp)
        transpose_matrix = [list(row) for row in zipped_rows]
        for i in range (int(len(transpose_matrix)/2)):
            aux = transpose_matrix[len(transpose_matrix) - 1 - i]
            transpose_matrix[len(transpose_matrix) - 1 - i] = transpose_matrix[i]
            transpose_matrix[i] = aux 

        a_file = open(outputFile, "w")
        for row in transpose_matrix:
            for elem in row:
                if elem == 0: elem = " " 
                a_file.write(''.join(str(elem)))
            a_file.write('\n')


    def update_previous_motors(self, l, r):
        global previous_power_r
        global previous_power_l

        previous_power_l = (l + previous_power_l) / 2
        previous_power_r = (r + previous_power_r) / 2


    def round_base(self, x, base = 2):
        return base * round(x/base)

    
    def position_estimator(self, movement):
        global est_pos
        global previous_power_r
        global previous_power_l

        center_dif = 0
        left_dif = 0
        right_dif = 0

        near_wall = False

        if self.measures.irSensor[0] > 1/1.0:
            # print("Aquiii")
            near_wall = True
            center_dif = 0.4 - 1/self.measures.irSensor[0]
        # if 1/self.measures.irSensor[1] > 1/0.7:
        #     left_dif = 0.4 - 1/self.measures.irSensor[1]
        # if 1/self.measures.irSensor[2] > 1/0.7:
        #     right_dif = 0.4 - 1/self.measures.irSensor[2]

        
        # center_dif = 0 # Não está a ser aplicado
        
        motor_pos_weight = 0.2
        sensor_pos_weight = 0.8

        if near_wall:
            if movement == 0:
                print(self.round_base(est_pos[0]))
                print("Prev: "+str(est_pos))
                est_pos = [motor_pos_weight * (est_pos[0] + previous_power_l) + sensor_pos_weight * (self.round_base(est_pos[0]) + center_dif), est_pos[1]]
                print("After 0 " + str(est_pos))
            elif movement == 1:
                print(self.round_base(est_pos[1]))
                print("Prev: "+str(est_pos))
                est_pos = [est_pos[0], motor_pos_weight * (est_pos[1] + previous_power_l) + sensor_pos_weight * (self.round_base(est_pos[1]) + center_dif)]
                print("After 1 " + str(est_pos))
            elif movement == 2:
                print(self.round_base(est_pos[1]))
                print("Prev: "+str(est_pos))
                est_pos = [est_pos[0], motor_pos_weight * (est_pos[1] - previous_power_l) + sensor_pos_weight * (self.round_base(est_pos[1]) - center_dif)]
                print("After 2 " + str(est_pos))
            elif movement == 3:
                print(self.round_base(est_pos[0]))
                print("Prev: "+str(est_pos))
                est_pos = [motor_pos_weight * (est_pos[0] - previous_power_l) + sensor_pos_weight * (self.round_base(est_pos[0]) - center_dif), est_pos[1]]
                print("After 3 " + str(est_pos))

            # print(self.round_base(est_pos[0]))
            # print("Prev: "+str(est_pos))
            # print("After 0 " + str(est_pos))
            
        else:
            if movement == 0:
                est_pos = [est_pos[0] + previous_power_l, est_pos[1]]
            elif movement == 1:
                est_pos = [est_pos[0], est_pos[1] + previous_power_l]
            elif movement == 2:
                est_pos = [est_pos[0], est_pos[1] - previous_power_l]
            elif movement == 3:
                est_pos = [est_pos[0] - previous_power_l, est_pos[1]]


    def paintMapp(self):
        global movement
        global current_GPS
        global mapp
        
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        sensibility = 1
        
        if movement == 0: #direita
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], " ")
        elif movement == 1: #cima
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, " ")
        elif movement == 2: #baixo
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, " ")
        elif movement == 3: #esquerda
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]+1, " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1]-1, " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]-1, current_GPS[1], " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoordAndPaint(current_GPS[0]+1, current_GPS[1], " ")
            
        self.translateGPStoMappCoordAndPaint(current_GPS[0], current_GPS[1], "X")
        self.printMapp(mapp)
        
    def checkGround(self):
        global Point1
        global Point2
         
        current_x, current_y = self.translateGPStoMappCoord(current_GPS[0], current_GPS[1])
        
        if self.measures.ground == 1 and Point1 == []:
             Point1 = [current_x, current_y]
         
        if self.measures.ground == 2 and Point2 == []:
             Point2 = [current_x, current_y]
              
    def calcBestPath_init_P1_P2(self):
        global Point1
        global Point2
        init = [27,13]
         
        zeros_map = self.get_zeros_map()
        ciberrato = CiberRato(zeros_map)
        
        nextPosition = SearchProblem(ciberrato, init, Point1)
        st = SearchTree(nextPosition)
        steps = st.search()
        if steps == []:
            return 0
        
        nextPosition = SearchProblem(ciberrato, Point1, Point2)
        st = SearchTree(nextPosition)
        new_steps = st.search()[1:]
        if new_steps == []:
            return 0
        for n_s in new_steps:
            steps.append(n_s)
        
        nextPosition = SearchProblem(ciberrato, Point2, init)
        st = SearchTree(nextPosition)
        new_steps = st.search()[1:]
        if new_steps == []:
            return 0
        for n_s in new_steps:
            steps.append(n_s)
        
        self.writeBestPathToFile(steps)
        
        return 1
        
    def writeBestPathToFile(self, steps):
        global Point1
        global Point2
        
        i = 0
        a_file = open(outputFile, "w")
        for s in steps:
            if (i % 2) == 0:
                if s == Point1:
                    a_file.write(str(s[0] - 27) + " " + str(s[1] - 13) + " #1 \n")
                elif s == Point2:
                    a_file.write(str(s[0] - 27) + " " + str(s[1] - 13) + " #2 \n")
                else:
                    a_file.write(str(s[0] - 27) + " " + str(s[1] - 13) + "\n")
            i += 1
         

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "Rodrigo-Rui-C2"
host = "localhost"
pos = 1
mapc = None


for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--outputfile" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        outputFile = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
