
import copy
from heapq import heappush, heappop
 
# for the n x n matrix
n = 3
 
# bottom, left, top, right
row = [ 1, 0, -1, 0 ]
col = [ 0, -1, 0, 1 ]

# main UI from sample in rubric.
def main():
    puzzle_mode = input("Welcome to an 8-Puzzle Solver. Type '1' to use a default puzzle, or '2' to create your own."
                        + '\n')
    if puzzle_mode == "1":
        return init_default_puzzle_mode()
    
# this gets called to solve built in puzzle with the 3 methods
def init_default_puzzle_mode():
    # Initial configuration
    # Value 0 is used for empty space
    initial = [ [ 1, 2, 3 ],
                [ 5, 6, 0 ],
                [ 7, 8, 4 ] ]
    
    # Solvable Final configuration
    # Value 0 is used for empty space
    final = [ [ 1, 2, 3 ],
            [ 5, 8, 6 ],
            [ 0, 7, 4 ] ]
    
    # Blank tile coordinates in
    # initial configuration
    empty_tile_pos = [ 1, 2 ]
    
    # Function call to solve the puzzle
    ucs(initial, empty_tile_pos, final)

# make the min heap for the prioty queue
class priorityQueue:
     
    # initialize the queue
    def __init__(self):
        self.heap = []
 
    # insert a new key called k
    def push(self, k):
        heappush(self.heap, k)
 
    # remove from queue
    def pop(self):
        return heappop(self.heap)
 
    # check if queue is empty
    def empty(self):
        if not self.heap:
            return True
        else:
            return False
 
# make the node for the tree
class node:
    # initialize parameters for node
    def __init__(self, parent, mat, empty_tile_pos, cost, level):
                      
        # store parent node of current node
        self.parent = parent
 
        # stores the matrix
        self.mat = mat
 
        # stores the location of the empty tile
        self.empty_tile_pos = empty_tile_pos
 
        # stores the number of misplaced tiles
        self.cost = cost
 
        # stores the amount of moves made
        self.level = level
 
    # Method used for priority queue based on the cost variable of the objects
    def __lt__(self, nxt):
        return self.cost < nxt.cost
 
# calculate the number of misplaced tiles
def calculateCost(mat, final) -> int:
     
    count = 0
    for i in range(n):
        for j in range(n):
            if ((mat[i][j]) and
                (mat[i][j] != final[i][j])):
                count += 1
                 
    return count
 
def newNode(mat, empty_tile_pos, new_empty_tile_pos,
            level, parent, final) -> node:
                 
    # copy data from parent and paste it to current
    new_mat = copy.deepcopy(mat)
 
    # Movement through the matrix
    x1 = empty_tile_pos[0]
    y1 = empty_tile_pos[1]
    x2 = new_empty_tile_pos[0]
    y2 = new_empty_tile_pos[1]
    new_mat[x1][y1], new_mat[x2][y2] = new_mat[x2][y2], new_mat[x1][y1]
 
     # set the number of misplaced tiles
    cost = calculateCost(new_mat, final)
 
    new_node = node(parent, new_mat, new_empty_tile_pos,
                    cost, level)
    return new_node
 
# print the matrix
def printMatrix(mat):
     
    for i in range(n):
        for j in range(n):
            print("%d " % (mat[i][j]), end = " ")
             
        print()
 
# check for valid matrix location
def isSafe(x, y):
     
    return x >= 0 and x < n and y >= 0 and y < n
 
# shows the path of the node from the root
def printPath(root):
     
    if root == None:
        return
     
    printPath(root.parent)
    printMatrix(root.mat)
    print()
 
#######################
# Uniform Cost Search #
#######################
def ucs(initial, empty_tile_pos, final):

    # making a priority queue
    pq = priorityQueue()

    # makes the root node then pushes the root to the list of visited nodes
    cost = calculateCost(initial, final)
    root = node(None, initial, empty_tile_pos, cost, 0)
    pq.push(root)

    # loop to traverse nodes with lowest cost
    while not pq.empty():
        # locate nodes with lowest cost then pops it from visited nodes
        minimum = pq.pop()
 
        # if we find the lowest cost
        if minimum.cost == 0:
            printPath(minimum)
            return
 
        # for loop to keep track of child nodes
        for i in range(4):
            new_tile_pos = [minimum.empty_tile_pos[0] + row[i], minimum.empty_tile_pos[1] + col[i], ]
            # if safe, we make a new child node and push it to visited nodes
            if isSafe(new_tile_pos[0], new_tile_pos[1]):
                child = newNode(minimum.mat, minimum.empty_tile_pos, new_tile_pos, minimum.level + 1, minimum, final,)
                pq.push(child)

main()