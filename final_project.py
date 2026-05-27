# -*- coding: utf-8 -*-


import configparser
import json
from collections import deque
import heapq # priority queue
import os # debugging

default_path = "config.ini"
configur = configparser.ConfigParser()

#read in the config file
configur.read(default_path)
print(os.path.abspath(default_path))

class node:

    def __init__(self,x,y):
      self.x = x            #node location
      self.y = y            #node location
      self.g = False     #goal position
      self.s = False    #start position
      self.f = False     #is it a forbidden cell
      self.visit = False       #is it a visited cell
      self.parent = None       #previous node for backtracking
      self.in_fringe = False
      self.path = False

    def __repr__(self):
        return f"({self.x}, {self.y})"
    
    def __hash__(self):
        return hash((self.x, self.y)) # node must be hashable

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


class GridWorld:

     def __init__(self):
      configur.read(default_path)
      #get the variables from the config.ini file
      self.w = int(configur.get("CONFIGURATION",'width'))
      self.h = int(configur.get("CONFIGURATION",'height'))
      self.f = json.loads(configur.get("CONFIGURATION",'forbidden'))
      self.s = json.loads(configur.get("CONFIGURATION",'start'))
      self.g = json.loads(configur.get("CONFIGURATION",'goal'))

      #set the variables to create gridworld
      self.grid = []
      
      #self.make_grid()  #make the grid
      #self.set_start()  #set start coords
      #self.set_goal()   #set goal coords
      #self.set_forbid() #set forbidden coords
      
      self.reset()
      self.disply_board() #display the grid
      
      
      #reset the grid
     def reset(self):
        self.grid = []
        self.make_grid()
        for row in self.grid:
           for n in row:
              n.parent = None
              n.path = False
              n.visit = False
              n.in_fringe = False
              n.visit = False
        self.set_start()
        self.set_goal()       
        self.set_forbid()
      
      # Then, you need to define 4 functions declared above 
     def make_grid(self):
      # use 2 for loops to create 2D grid
       for i in range(self.h):
         column = []
         self.grid.append(column)
         for j in range(self.w):
           column.append(node(j,i))
         
      
      #start coords
     def set_start(self):
       x=self.s[0]
       y=self.s[1]
       #print ('Start:', x, y)
       self.get_node(x, y).s=True     

        #goal coords
     def set_goal(self):
       x=self.g[0]
       y=self.g[1]
       #print ('Goal:', x, y)
       self.get_node(x, y).g=True      

     
     def set_forbid(self):
        # use loop go through every one from self.f, to get each x-y, and do self.grid[x][y].f=True


        for i in range(len(self.f)):
            x=self.f[i][0]
            y=self.f[i][1]
            #print ('Forbidden:', x, y)
            self.get_node(x, y).f=True   

         
              
     def get_node(self, x, y): #return a node
        return self.grid[y][x]
    
     #display grid with X for forbidden, S for start node, G for goal node, _ for available node
     def disply_board(self):
         for i in range(self.h):
          print('|',end='')
          for j in range(self.w):
              g=self.grid[i][j].g
              s=self.grid[i][j].s
              f= self.grid[i][j].f
              if g: 
                print('G|', end='')      
              elif s:                  
                print('S|', end='')      
              elif f:
                print('X|', end='')
              elif self.grid[i][j].path:
                 print("o|", end="")
              elif self.grid[i][j].visit:
                 print("-|", end="")
              else:               
                print('_|', end='')
          print()
          
     def get_path(self, node): #return a node
        #sx=self.s[0]
        #sy=self.s[1]    
        path=[]
        n=node
        while n is not None:        
            path.append(n)
            n=n.parent
        path.reverse()    
        return path
     

     def neighbours(self, x, y):
        # define the four directions
        moves = [(0,1),(0,-1),(1,0),(-1,0)]
        result = []

        # for each direction
        for dx, dy in moves:
           # compute the new coordinate
           nx, ny = x + dx, y + dy

           # if the coordinate is inside the ground bounds
           if 0 <= nx < self.w and 0 <= ny < self.h:
              # and the cell is not forbidden
              if [nx, ny] not in self.f:
                 result.append(self.get_node(nx, ny))
               
        return result
     
    
    
grid = GridWorld()

# BFS
def bfs(grid, start_node, goal_node):
   expanded = 0

   # use a queue bc BFS explores in layers (FIFO)
   queue = deque([start_node])

   # track discovered nodes so we don't revisit them
   discovered = {start_node}
   start_node.parent = None

   while queue:
      # remove the next node to explore
      current = queue.popleft() # popleft is deque
      current.visit = True
      expanded += 1

      if current == goal_node:
         return current, expanded
      
      # look at all valid neighbours (implicit graph)
      for neighbour in grid.neighbours(current.x, current.y):
         # if we have not seen this node before
         if neighbour not in discovered:
            discovered.add(neighbour)
            neighbour.parent = current
            queue.append(neighbour)
   return None, expanded
   
# DFS
def dfs(grid, start_node, goal_node):
   expanded = 0
   # use stack, opposite of bfs (LIFO)
   stack = deque([start_node])
   discovered = {start_node}
   start_node.parent = None

   while stack:
      current = stack.pop() # pop instead of popleft() bc stack
      current.visit = True
      expanded += 1

      if current == goal_node:
         return current, expanded
      
      for neighbour in grid.neighbours(current.x, current.y):
         if neighbour not in discovered:
            discovered.add(neighbour)
            neighbour.visit = True
            neighbour.parent = current
            stack.append(neighbour)
   return None, expanded


# annotated for clarity; used GeeksForGeeks and Microsoft CoPilot for help
# ------------------------------------------------------------------------------------------------------------------
# A*; utilize manhattan distance as h(n)
# WHERE f(n) = total estimated cost
# h(n) = estimated distance to goal; heuristic used (manhattan distance, which is h(n) = |xn - xgoal| + |yn - ygoal)
# g(n) = distance from start to current node; number of steps taken so far
# ------------------------------------------------------------------------------------------------------------------
def a_star(grid, start_node, goal_node):
    expanded = 0
    # Priority queue: (f_score, tie_breaker, node)
    frontier = []
    counter = 0

    # Compute initial heuristic for start node
    h0 = abs(start_node.x - goal_node.x) + abs(start_node.y - goal_node.y)
    heapq.heappush(frontier, (h0, counter, start_node))
    counter += 1

    # g-cost dictionary
    g_cost = {start_node: 0}
    discovered = {start_node} # not accessed

    closed = set()

    # start node has no parent
    start_node.parent = None

    while frontier:

        # pop node with lowest f
        f, _, current = heapq.heappop(frontier)
        current.visit = True
        expanded += 1

        if current in closed:
            continue

        closed.add(current)

        # goal reached
        if current == goal_node:
            return current, expanded

        # explore neighbors
        for neighbour in grid.neighbours(current.x, current.y):

            if neighbour in closed:
                continue

            tentative_g = g_cost[current] + 1

            # if new or better path
            if neighbour not in g_cost or tentative_g < g_cost[neighbour]:

                g_cost[neighbour] = tentative_g

                # Manhattan heuristic
                h = abs(neighbour.x - goal_node.x) + abs(neighbour.y - goal_node.y)
                f = tentative_g + h

                neighbour.parent = current

                heapq.heappush(frontier, (f, counter, neighbour))
                counter += 1

    return None, expanded


# print the path
def mark_path(grid, path):
   for n in path:
      # don't overwrite start or goal
      if not n.s and not n.g:
         n.path = True

# run the searches
print("BFS")
grid.reset() # make sure to reset grid each time so no overlapping happens
start_node = grid.get_node(grid.s[0], grid.s[1])
goal_node  = grid.get_node(grid.g[0], grid.g[1])
goal, expanded = bfs(grid, start_node, goal_node)
path = grid.get_path(goal)
print("Number of Nodes Visited:", expanded)
print("Solution Path:", " → ".join(f"({n.x},{n.y})" for n in path)) # copy-pasted arrow from google
mark_path(grid, path)
grid.disply_board()

print("DFS")
grid.reset()
start_node = grid.get_node(grid.s[0], grid.s[1])
goal_node  = grid.get_node(grid.g[0], grid.g[1])
goal, expanded = dfs(grid, start_node, goal_node)
path = grid.get_path(goal)
print("Number of Nodes Visited:", expanded)
print("Solution Path:", " → ".join(f"({n.x},{n.y})" for n in path))
mark_path(grid, path)
grid.disply_board()

print("A*") # A* is the winner, it visited the least amount of nodes! to be expected
grid.reset()
start_node = grid.get_node(grid.s[0], grid.s[1])
goal_node  = grid.get_node(grid.g[0], grid.g[1])
goal, expanded = a_star(grid, start_node, goal_node)
path = grid.get_path(goal)
print("Number of Nodes Visited:", expanded)
print("Solution Path:", " → ".join(f"({n.x},{n.y})" for n in path))
mark_path(grid, path)
grid.disply_board()




    
