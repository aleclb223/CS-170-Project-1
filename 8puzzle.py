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
    
    # Movement
    # check if you can move a certain direction
    def check_if_possible(self,x,n):
        directions = ['Left', 'Right', 'Up', 'Down']
        if x % n == 0: # if you are all the way to the left
            directions.remove('Left')
        if x % n == n-1: # if you are all the way to the right
            directions.remove('Right')
        if x - n < 0: # if you are at the top
            directions.remove('Up')
        if x + n > n*n - 1: # if you are at the bottom
            directions.remove('Down')

        return directions
    
    # the actual movement using calculations
    def expand(self,n):
        x = self.state.index(0)
        directions = self.check_if_possible(x,n)
        children=[]
        for direction in directions:
            temp = self.state.copy()
            if direction == 'Left':
                temp[x], temp[x - 1] = temp[x - 1], temp[x]
            elif direction == 'Right':
                temp[x], temp[x + 1] = temp[x + 1], temp[x]
            elif direction == 'Up':
                temp[x], temp[x - n] = temp[x - n], temp[x]
            elif direction == 'Down':
                temp[x], temp[x + n] = temp[x + n], temp[x]
            children.append(State(temp, self, direction, self.depth + 1, 1))
        return children
    
    # this is for node traversal
    def solve(self):
        solve = []
        solve.append(self.direction)
        path = self
        while path.parent!= None:
            path = path.parent
            solve.append(path.direction)
        solve = solve[:-1]
        solve.reverse()
        return solve
    
    # reshapes the output for the current state of the matrix into a 3x3
    def print_state(self):
        arr =np.reshape(self.state,(3,3))
        print(arr)
        
##################
##### Driver #####
##################

#https://www.geeksforgeeks.org/8-puzzle-problem-using-branch-and-bound/
#https://github.com/Pariasrz/N-Puzzle-solver-with-Search-Algorithms/blob/main/Main.py
#https://www.geeksforgeeks.org/check-instance-8-puzzle-solvable/
# Used templates solvable, inv_num
def main():
    #start = root
    #make n an input 
    n=3 # this is the puzzle dimension
    root =[]
    print("enter puzzle")
    for i in range (0,n*n):
        pz= int(input())
        root.append(pz)
    print("The starting is:",root)
    c=int(input("Which algorithm? 1 for ucs, 2 for misplaced, 3 for manhattan \n"))
    solvable(root)
    if solvable(root):
        print("Solvable, please wait. \n")
        if c==1: # if choice is 1, run ucs search
            time1 = time()
            ucs_solution = ucs(root, 3)
            ucs_time = time() - time1
            print('ucs depth is ', len(ucs_solution[0]))
            print('Number of expanded nodes is ', ucs_solution[1]-1)    
            print('ucs Time:', ucs_time , "\n")
            print(ucs.maxq, "max queue nodes")
        
        elif c==2: # if choice is 2, run missingtile search
            time2=time()
            missingtilesol=AStar_search_missing(root,n)
            AStar_time_mis=time()-time2
            print('A* mis depth is ', len(missingtilesol[0])-1)
            print('Number of expanded nodes is ', missingtilesol[1])
            
            print('A* mis Time:', AStar_time_mis)
            print(len(AStar_search_missing.explored-1), "explored nodes")
            print(AStar_search_missing.maxq)
        
        elif c==3: # if choice is 3, run manhattan search
            time4 = time()
            AStar_solution = AStar_search_man(root, 3)
            AStar_time = time() - time4
            print('NEWYORK BABY Solution is ', len(AStar_solution[0]))
            print('Number of expanded nodes is ', AStar_solution[1]-1)   
            print('NEWYORK BABY Time:', AStar_time)
            print(AStar_search_man.maxq)

    else:
        print("Not solvable")
        
# inversions. function to count inversions in the array
def inv_num(puzzle):
    inv = 0
    for i in range(len(puzzle)-1):
        for j in range(i+1 , len(puzzle)):
            if (( puzzle[i] > puzzle[j]) and puzzle[i] and puzzle[j]):
                inv += 1
    return inv

def solvable(puzzle): # check if initial state puzzle is solvable: number of inversions should be even.
    inv_counter = inv_num(puzzle)
    if (inv_counter %2 ==0):
        return True
    return False

main()
