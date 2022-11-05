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
