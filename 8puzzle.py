
import copy
import heapq as min_heap_esque_queue #because it sort of acts like a min heap

#######################
# set up the 8-puzzle #
#######################

n=3 # nxn matrix, for an 8-puzzle n=3
rows = [ 1, 0, -1, 0 ]  
cols = [ 0, -1, 0, 1 ] 

# mkae the min heap for the prioty queue
class pq:

    # initialize the queue
    def __init__(self):
        self.heap=[]

    # insert a new key called k
    def push(self, k):
        min_heap_esque_queue.heappush(self.heap, k)

    # remove from queue
    def pop(self):
        return min_heap_esque_queue.heappop(self.heap)

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
        self.parent=parent

        # stores the matrix
        self.mat=mat

        # stores the number of misplaced tiles
        self.cost=cost

        # stores the amount of moves made
        self.level=level

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

def newNode(mat, empty_tile_pos, new_empty_tile_pos, level, parent, final) -> node:

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

    new_node = node(parent, new_mat, new_empty_tile_pos, cost, level)

    return new_node

# print the matrix
def printMatrix(mat):

    for i in range(n):
        for j in range(n):
            print("%d " % (mat[i][j]), end = " ")

    print()

# check for valid matrix location
def printPath(root):

    if root == None:
        return

    printPath(root.parent)
    printMatrix(root.mat)
    print()


#######################
# Uniform Cost Search #
#######################

# REFERENCE: https://sandipanweb.wordpress.com/2017/03/16/using-uninformed-informed-search-algorithms-to-solve-8-puzzle-n-puzzle/
# OR http://www.sci.brooklyn.cuny.edu/~chipp/cis32/lectures/Lecture6.pdf



