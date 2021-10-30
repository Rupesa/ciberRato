
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

movement = 0 # 0 - frente, 1 - cima, 2 - baixo, 3 - tras

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
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

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
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        print('Compass: ' + str(self.measures.compass))
        print('GPS x y: ' + str(self.measures.x) + ' ' + str(self.measures.y))
        print('Movement: ' + str(movement))
        if movement == 0:
            if self.measures.compass <= 2 and self.measures.compass >= -2:
                if self.measures.irSensor[center_id] > 1.1:
                   print('Wall incoming')
                   if self.measures.irSensor[right_id] < 2.5:
                      print('Rotate right')
                      movement = 2
                      self.driveMotors(0.13,0.07)
                   elif self.measures.irSensor[left_id] < 2.5:
                      print('Rotate left')
                      movement = 1
                      self.driveMotors(0.07,0.13)
                else:
                   print('Go')
                   self.driveMotors(0.1,0.1)
            elif self.measures.compass < 0:
                print('Adjust : Rotate slowly left')
                self.driveMotors(0.0,0.07)
            elif self.measures.compass > 0:
                print('Adjust : Rotate slowly right')
                self.driveMotors(0.07,0.00)
        elif movement == 2:
            if self.measures.compass >= -92 and self.measures.compass <= -88:
                if self.measures.irSensor[center_id] > 1.1:
                   print('Wall incoming')
                   if self.measures.irSensor[right_id] < 2.5:
                      print('Rotate right')
                      movement = 3
                      self.driveMotors(0.13,0.07)
                   elif self.measures.irSensor[left_id]< 2.5:
                      print('Rotate left')
                      movement = 0
                      self.driveMotors(0.07,0.13)
                else:
                   print('Go')
                   self.driveMotors(0.1,0.1)
            elif self.measures.compass < -90:
                print('Adjust : Rotate slowly left')
                self.driveMotors(0.00,0.07)
            elif self.measures.compass > -90:
                print('Adjust : Rotate slowly right')
                self.driveMotors(0.07,0.00)
        elif movement == 1:
            if self.measures.compass <= 92 and self.measures.compass >= 88:
                if self.measures.irSensor[center_id] > 1.1:
                   print('Wall incoming')
                   if self.measures.irSensor[right_id]< 2.5:
                      print('Rotate right')
                      movement = 0
                      self.driveMotors(0.13,0.07)
                   elif self.measures.irSensor[left_id]< 2.5:
                      print('Rotate left')
                      movement = 3
                      self.driveMotors(0.07,0.13)
                else:
                   print('Go')
                   self.driveMotors(0.1,0.1)
            elif self.measures.compass < 90:
                print('Adjust : Rotate slowly left')
                self.driveMotors(0.00,0.08)
            elif self.measures.compass > 90:
                print('Adjust : Rotate slowly right')
                self.driveMotors(0.08,0.00)
        elif movement == 3:
            if self.measures.compass >= 178 or self.measures.compass <= -178:
                if self.measures.irSensor[center_id] > 1.1:
                   print('Wall incoming')
                   if self.measures.irSensor[right_id]< 2.5:
                      print('Rotate right')
                      movement = 1
                      self.driveMotors(0.13,0.07)
                   elif self.measures.irSensor[left_id]< 2.5:
                      print('Rotate left')
                      movement = 2
                      self.driveMotors(0.07,0.13)
                else:
                   print('Go')
                   self.driveMotors(0.1,0.1)
            elif self.measures.compass < 178 and self.measures.compass > 0:
                print('Adjust : Rotate slowly left')
                self.driveMotors(0.00,0.08)
            elif self.measures.compass < -178 and self.measures.compass < 0:
                print('Adjust : Rotate slowly right')
                self.driveMotors(0.08,0.00)

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


rob_name = "pythonClientC2"
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
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
