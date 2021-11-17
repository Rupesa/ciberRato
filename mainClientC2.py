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

CELLROWS=7
CELLCOLS=14

movement = 0 # 0 - frente, 1 - cima, 2 - baixo, 3 - tras
init_GPS = [0,0] # initial GPS Position
current_GPS = [0,0] # current GPS position
turning = 0  # turning flag
rotation = 0 # diretion of rotation

rotation_counter = 0 # counter to rotate faster
ongoing = 0 # 1 if the mouse is going to a position

w, h = 55, 27
mapp = [[0 for x in range(h)] for y in range(w)] 
mapp[27][13] = "I"
#aux_counter = 0
scale= []

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
        
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        self.readSensors()
        init_GPS = [round(self.measures.x), round(self.measures.y)]
        current_GPS = init_GPS
        
        scale = [init_GPS[0] - 27, init_GPS[1] - 13]
        
        
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

    # Auxiliary Functions
    
    def translateGPStoMappCoord(self, x, y, symbol):
    	global mapp
    	global scale

    	pos_x = int(x-scale[0])
    	pos_y = int(y-scale[1])
    	
    	if mapp[pos_x][pos_y] == 0 or mapp[pos_x][pos_y] == " ":
    	    mapp[pos_x][pos_y] = symbol

    
    def printMapp(self, m):
        global mapp
        
        zipped_rows = zip(*mapp)
        transpose_matrix = [list(row) for row in zipped_rows]
        for i in range (int(len(transpose_matrix)/2)):
            aux = transpose_matrix[len(transpose_matrix) - 1 - i]
            transpose_matrix[len(transpose_matrix) - 1 - i] = transpose_matrix[i]
            transpose_matrix[i] = aux 

        print('\n'.join([' '.join([str(cell) for cell in row]) for row in transpose_matrix]))

        #  for i in range(len(mapp[0])):
        #      print("\n")
        #      for j in range(len(mapp)):
        #          print(mapp[j][i], end=" ")
            

    def paintMapp(self):
        global movement
        global current_GPS
        global mapp
        
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        sensibility = 1.7

        print("Sensor da frente: "+str(self.measures.irSensor[center_id]))
        print("Sensor da direita: "+str(self.measures.irSensor[right_id]))
        print("Sensor da esquerda: "+str(self.measures.irSensor[left_id]))
        print("Sensor da trazeira: "+str(self.measures.irSensor[back_id]))
        
        if movement == 0: #direita
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], " ")
        elif movement == 1: #cima
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, " ")
        elif movement == 2: #baixo
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, " ")
        elif movement == 3: #esquerda
            if self.measures.irSensor[right_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]+1, " ")
            if self.measures.irSensor[left_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, "-")
            else:
                self.translateGPStoMappCoord(current_GPS[0], current_GPS[1]-1, " ")
            if self.measures.irSensor[center_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]-1, current_GPS[1], " ")
            if self.measures.irSensor[back_id] > sensibility:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], "|")
            else:
                self.translateGPStoMappCoord(current_GPS[0]+1, current_GPS[1], " ")
            
        self.translateGPStoMappCoord(current_GPS[0], current_GPS[1], "X")
        self.printMapp(mapp)

    def wander(self):
        global movement
        global turning
        global rotation
        global rotation_counter
        global ongoing
        
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        
        if turning == 0:
            if self.measures.irSensor[center_id] > 1.7:
                #self.paintMapp();
                self.checksides()
                
            else:
                self.moveforward()
        else:
            if movement == 1 and (self.measures.compass >= 92 or self.measures.compass <= 88):
                if rotation == 1:
                    self.turnLeft()
                else:
                    self.turnRight()
            elif movement == 1:
                rotation_counter = 0
                turning = 0
                
            elif movement == 2 and (self.measures.compass <= -92 or self.measures.compass >= -88):
                if rotation == 1:
                    self.turnLeft()
                else:
                    self.turnRight()
            elif movement == 2:
                rotation_counter = 0
                turning = 0
                
            elif movement == 0 and (self.measures.compass <= -2 or self.measures.compass >= 2):
                if rotation == 1:
                    self.turnLeft()
                else:
                    self.turnRight()
            elif movement == 0:
                rotation_counter = 0
                turning = 0
                
            elif movement == 3 and ((self.measures.compass >= -177 and self.measures.compass <= 0) or (self.measures.compass <= 177 and self.measures.compass >= 0)):
                if rotation == 1:
                    self.turnLeft()
                else:
                    self.turnRight()
            else:
                rotation_counter = 0
                turning = 0
    

    def moveforward(self):
        global current_GPS
        global movement
        global aux_counter
        
        if (abs(int(self.measures.x - current_GPS[0])) < 1) and (abs(int(self.measures.y - current_GPS[1])) < 1):
            print("\nCurrent GPS: "+str(current_GPS))
            if movement == 0:
                if self.measures.compass < -1: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                elif self.measures.compass > 1: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                else:
                    self.driveMotors(0.13, 0.13)
            if movement == 1:
                if self.measures.compass < 89: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                elif self.measures.compass > 91: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                else:
                    self.driveMotors(0.13, 0.13)
            if movement == 2:
                if self.measures.compass < -91: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                elif self.measures.compass > -89: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                else:
                    self.driveMotors(0.13, 0.13)
            if movement == 3:
                if self.measures.compass <= 178 and self.measures.compass > 0: # Adjust : slowly left
                    self.driveMotors(0.11,0.14)
                elif self.measures.compass >= -178 and self.measures.compass < 0: # Adjust : slowly right
                    self.driveMotors(0.14,0.11)
                else:
                    self.driveMotors(0.13, 0.13)
        else:
            current_GPS = [round(self.measures.x), round(self.measures.y)]
            self.paintMapp()

             
    def checksides(self):
        global movement
        global turning
        global rotation
        
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
            rotation = 0
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
            rotation = 1
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
            rotation = 0
            self.turnRight()
    
    def turnRight(self):
        global rotation_counter
        rotation_counter += 1
        if (rotation_counter < 6):
            self.driveMotors(0.13, -0.13)
        else:
            self.driveMotors(0.015, -0.015)
        # print(rotation_counter)
        # print(self.measures.compass)
    
    def turnLeft(self):
        global rotation_counter
        rotation_counter += 1
        if (rotation_counter < 6):
            self.driveMotors(-0.13, 0.13)
        else:
            self.driveMotors(-0.015, 0.015)
         

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
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
