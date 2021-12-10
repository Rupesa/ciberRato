# ALUNOS:
# Rodrigo Santos - 89180
# Rui Santos - 89293

from tree_search import *
import math
class CiberRato(SearchDomain):
    def __init__(self, mapp):
        self.mapa = mapp

    def actions(self,node):
        actlist = []
        if (self.mapa[node[0] + 1][node[1]] == 0):
            actlist += [0]
        if (self.mapa[node[0] - 1][node[1]] == 0):
            actlist += [3]
        if (self.mapa[node[0]][node[1] + 1] == 0):
            actlist += [1]
        if (self.mapa[node[0]][node[1] - 1] == 0):
            actlist += [2]
        return actlist

    def result(self,node,action):
        if action == 0:
        	return [node[0] + 1, node[1]]

        if action == 3:
        	return [node[0] - 1, node[1]]

        if action == 1:
        	return [node[0], node[1] + 1]

        if action == 2:
        	return [node[0], node[1] - 1]

    def cost(self, cidade, action):
    	return 1

    def heuristic(self, state, goal_state):
        return math.floor(math.hypot(state[0] - goal_state[0], state[1]- goal_state[1]))
