# ALUNOS:
# Rodrigo Santos - 89180
# Rui Santos - 89293

from abc import ABC, abstractmethod

# Dominios de pesquisa
# Permitem calcular
# as accoes possiveis em cada estado, etc
class SearchDomain(ABC):

    # construtor
    @abstractmethod
    def __init__(self):
        pass

    # lista de accoes possiveis num estado
    @abstractmethod
    def actions(self, state):
        pass

    # resultado de uma accao num estado, ou seja, o estado seguinte
    @abstractmethod
    def result(self, state, action):
        pass

    # custo de uma accao num estado
    @abstractmethod
    def cost(self, state, action):
        pass

    # custo estimado de chegar de um estado a outro
    @abstractmethod
    def heuristic(self, state, goal_state):
        pass

# Problemas concretos a resolver
# dentro de um determinado dominio
class SearchProblem:
    def __init__(self, domain, initial, goal):
        self.domain = domain
        self.initial = initial
        self.goal = goal

    def goal_test(self, state):
        return state == self.goal

# Nos de uma arvore de pesquisa
class SearchNode:
    def __init__(self, state, parent, depth, cost, heuristic): 
        self.state = state
        self.parent = parent
        self.depth = depth
        self.cost = cost
        self.heuristic = heuristic

    def in_parent(self, state):
        if self.parent == None:
            return False
        return self.parent.state == state or self.parent.in_parent(state)

    def __str__(self):
#        return "no(" + str(self.state) + "," + str(self.parent) + ")"
        return f"no({self.state}, {self.parent}, {self.depth})"

    def __repr__(self):
        return str(self)

# Arvores de pesquisa
class SearchTree:

    # construtor
    def __init__(self,problem, strategy='a*'): 
        self.problem = problem
        root = SearchNode(problem.initial, None, 0, 0, self.problem.domain.heuristic(self.problem.initial, self.problem.goal))
        self.open_nodes = [root]
        self.strategy = strategy
        self.length = 0
        self.terminal = 0
        self.non_terminal = 1
        self.ramification = 0 
        self.cost = 0
        
    # obter o caminho (sequencia de estados) da raiz ate um no
    def get_path(self,node):
        if node.parent == None:
            return [node.state]
        path = self.get_path(node.parent)
        path += [node.state]
        return(path)

    # procurar a solucao
    def search(self, limit=1000):
        
        check_loop = 0
        loop_list = []

        while self.open_nodes != []:
            #print(check_loop)
            node = self.open_nodes.pop(0)
            self.length += 1
            self.cost += node.cost
            if self.problem.goal_test(node.state):
                f=self.get_path(node)
                return f
            lnewnodes = []
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                if not node.in_parent(newstate) and node.depth+1 < limit: 
                    lnewnodes += [SearchNode(
                                    newstate,
                                    node, 
                                    node.depth+1, 
                                    node.cost+self.problem.domain.cost(node.state,a),
                                    self.problem.domain.heuristic(newstate, self.problem.goal)
                                )]
            if lnewnodes == []:
                self.terminal += 1
            else:
                self.non_terminal += len(lnewnodes) 
            self.add_to_open(lnewnodes)
            self.ramification = (self.non_terminal + self.terminal -1) / self.non_terminal

            if check_loop > 150:
                return []

            check_loop = check_loop + 1
            loop_list.append([0,0])

        return None

    # juntar novos nos a lista de nos abertos de acordo com a estrategia
    def add_to_open(self,lnewnodes):
        if self.strategy == 'breadth':
            self.open_nodes.extend(lnewnodes)
        elif self.strategy == 'depth':
            self.open_nodes[:0] = lnewnodes
        elif self.strategy == 'uniform':
            self.open_nodes = sorted(lnewnodes + self.open_nodes, key=lambda no: no.cost)    
        elif self.strategy == 'greedy':
            self.open_nodes = sorted(lnewnodes + self.open_nodes, key=lambda no: no.heuristic)    
        elif self.strategy == 'a*':
            self.open_nodes = sorted(lnewnodes + self.open_nodes, key=lambda no: no.heuristic+ no.cost)  
