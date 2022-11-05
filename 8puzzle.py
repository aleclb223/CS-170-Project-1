# RESTART
# Below are sources I used to build the program:
# https://github.com/Pariasrz/N-Puzzle-solver-with-Search-Algorithms
# https://www.geeksforgeeks.org/8-puzzle-problem-using-branch-and-bound/

from queue import PriorityQueue
from queue import Queue
from time import time
import numpy as np

h=0

##############################
##### Search Algorithms ######
##############################

# https://github.com/Pariasrz/N-Puzzle-solver-with-Search-Algorithms/blob/main/Search_Algorithms.py
# Used templates from BFS, AStar_search
# Uniform Cost Search. Similar to BFS
# Recall that UCS is also close to A* with h(n) hardcoded to 0
def ucs(given_state , n):
    root = State(given_state, None, None, 0, 0)
    if root.test():
        return root.solve()
    frontier = Queue()
    frontier.put(root)
    
    ucs.explored = []
    ucs.maxq = 0
    # this will help us find node with the least cost
    # We generate the list of children then pop them from the list
    while not(frontier.empty()):
        current_node = frontier.get()
        ucs.explored.append(current_node.state)
        current_node.print_state()
        children = current_node.expand(n)
        
        for child in children: # generating all possible children
            if child.state not in ucs.explored:
                if child.test():
                    return child.solve(), len(ucs.explored)
                frontier.put(child)
                ucs.maxq+=frontier.qsize()
    return

# A* Manhattan Search implementation.
def AStar_search_man(given_state , n):
    frontier = PriorityQueue()
    AStar_search_man.explored = []
    counter = 0
    root = State(given_state, None, None, 0, 0)
    evaluation = root.Manhattan_Distance(n) # Here manhat. dist. 
    frontier.put((evaluation[1], counter, root)) # based on A* evaluation
    AStar_search_man.maxq=0

    while not frontier.empty():
        current_node = frontier.get()
        current_node = current_node[2]
        AStar_search_man.explored.append(current_node.state)
        current_node.print_state()
        
        if current_node.test():
            return current_node.solve(), len(AStar_search_man.explored)

        children = current_node.expand(n)
        for child in children:
            if child.state not in AStar_search_man.explored:
                counter += 1
                evaluation = child.Manhattan_Distance(n) # we can use Misplaced_Tiles() instead.
                frontier.put((evaluation[1], counter, child)) # based on A* evaluation
                AStar_search_man.maxq+=frontier.qsize()
    return

def AStar_search_missing(given_state , n):
    AStar_search_missing.frontier = PriorityQueue()
    AStar_search_missing.explored = []
    counter = 0
    root = State(given_state, None, None, 0, 0)
    evaluation = root.Misplaced_Tiles(n) # we can use Misplaced_Tiles() instead.
    AStar_search_missing.frontier.put((evaluation[1], counter, root)) # based on A* evaluation
    AStar_search_missing.maxq=0
    print(evaluation)
    while not AStar_search_missing.frontier.empty():
        current_node = AStar_search_missing.frontier.get()
        current_node = current_node[2]
        AStar_search_missing.explored.append(current_node.state)
        
        current_node.print_state()
        print('max queue size',counter)
        
        
        #print(root.Misplaced_Tiles,'misplaced function')
        if current_node.test():
            return current_node.solve(), len(AStar_search_missing.explored)

        children = current_node.expand(n)
        for child in children:
            if child.state not in AStar_search_missing.explored:
                counter += 1
                evaluation = child.Misplaced_Tiles(n) # we can use Misplaced_Tiles() instead.
                AStar_search_missing.frontier.put((evaluation[1], counter, child)) # based on A* evaluation
                AStar_search_missing.maxq+=AStar_search_missing.frontier
    return

####################
##### 8 Puzzle #####
####################

# https://github.com/Pariasrz/N-Puzzle-solver-with-Search-Algorithms/blob/main/State.py
# Used the template for main state, test, Misplaced tiles, and Manhattan
# Expand and movement used.
class State:
    final_state=[1, 2, 3, 4, 5, 6, 7, 8, 0]
    AStar_evaluation_man = None
    AStar_evaluation_mis = None
    heuristic = None
    def __init__(self, state, parent, direction, depth, cost):
        self.state = state
        self.parent = parent
        self.direction = direction
        self.depth = depth
        if parent:
            self.cost = parent.cost + cost

        else:
            self.cost = cost

    def test(self): # check if the given state is goal
        if self.state == self.final_state:
            return True
        return False
    
    def Misplaced_Tiles(self,n):
        counter =0
        self.heuristic = 0
        global h
        
        for i in range(9):
            for j in range(9):
                if (self.state[i] != self.final_state[j]):
                    counter += 1
                self.heuristic = self.heuristic + counter
        self.AStar_evaluation_mis = self.heuristic
        h+=1
        self.greedy_evaluation = self.heuristic
            
        return(counter,self.greedy_evaluation,self.AStar_evaluation_mis)
    
    def Manhattan_Distance(self ,n): 
        self.heuristic = 0
        global h
        for i in range(1 , n*n):
            distance = abs(self.state.index(i) - self.final_state.index(i))
            
            # manhattan distance between the current state and goal state
            self.heuristic = self.heuristic + distance/n + distance%n

        self.greedy_evaluation = self.heuristic
        
        self.AStar_evaluation = self.heuristic + self.cost
        print(self.AStar_evaluation)
        
        
        return( self.greedy_evaluation, self.AStar_evaluation)
