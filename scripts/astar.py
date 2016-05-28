#!/usr/bin/env python
import copy
import rospy
from read_config import read_config
from Queue import Queue, PriorityQueue


config      = read_config()
start       = config["start"]
goal        = config["goal"]
walls       = config["walls"]
pits        = config["pits"]
map_siz_arr = config["map_size"]
map_height  = map_siz_arr[0]
map_width   = map_siz_arr[1]
step_cost   = config["reward_for_each_step"]

def get_heuristic():
   heuristic = [] 
   for i in range(0, map_height):
      heuristic.append([])
      for j in range(0, map_width):
         heuristic[i].append(aStarNode(9999))

   heuristic[start[0]][start[1]].isStart = True
   heuristic[start[0]][start[1]].set_coords(start)
   heuristic[goal[0]][goal[1]].isGoal = True
   heuristic[goal[0]][goal[1]].set_coords(goal)

   for w in range(0, len(walls)):
      wall = walls[w]
      heuristic[wall[0]][wall[1]].isWall = True
      
   for p in range(0, len(pits)):
      pit = pits[p]
      heuristic[pit[0]][pit[1]].isPit = True

   #init the coords
   for h in range(0, map_height):
      for w in range(0, map_width):
         heuristic[h][w].set_coords([h,w])


   for h in range(0, map_height):
      for w in range(0, map_width):
         heuristic[h][w].value = abs(goal[0] - h) + abs(goal[1] - w)

   return heuristic

def uniform_cost_search(grid, start_node):

   pq = Queue(1000)
   start_node.value = 0
   pq.put(start_node)

   while not pq.empty():
      
      node = pq.get()
      h = node.coords[0]
      w = node.coords[1]

      if (node.coords[0] - 1) >= 0:
         up = grid[h-1][w]
         if up.value > node.value + 1:
            up.parent = node
            up.value = node.value + 1
            pq.put(up)

      if (node.coords[0]  + 1) < map_height:
         down = grid[h+1][w]
         if down.value > node.value + 1:
            down.parent = node
            down.value = node.value + 1
            pq.put(down)

      if (node.coords[1]  - 1) >= 0:
         left = grid[h][w-1]
         if left.value > node.value + 1:
            left.parent = node
            left.value = node.value + 1
            pq.put(left)

      if (node.coords[1]  + 1) < map_width:
         right = grid[h][w+1]
         if right.value > node.value + 1:
            right.parent = node
            right.value = node.value + 1
            pq.put(right)

def get_backward_cost():
   
   bw_cost = [] 
   for i in range(0, map_height):
      bw_cost.append([])
      for j in range(0, map_width):
         bw_cost[i].append(aStarNode(9999))

   bw_cost[start[0]][start[1]].isStart = True
   bw_cost[start[0]][start[1]].set_coords(start)
   bw_cost[goal[0]][goal[1]].isGoal = True
   bw_cost[goal[0]][goal[1]].set_coords(goal)

   for w in range(0, len(walls)):
      wall = walls[w]
      bw_cost[wall[0]][wall[1]].isWall = True
      
   for p in range(0, len(pits)):
      pit = pits[p]
      bw_cost[pit[0]][pit[1]].isPit = True

   #init the coords
   for h in range(0, map_height):
      for w in range(0, map_width):
         bw_cost[h][w].set_coords([h,w])

   uniform_cost_search(bw_cost, bw_cost[start[0]][start[1]])
   return bw_cost


def compute_path(grid):
  open_ls   = PriorityQueue()
  closed_ls = []

  open_ls.put(grid[start[0]][start[1]], 0)

  while not open_ls.empty():
     node = open_ls.get() 
     
     closed_ls.append(node.coords)
     #push all neighbors of node that are not in closed list into open list
     if (node.coords[0] - 1) >= 0:
        neighbor = grid[node.coords[0] -1][node.coords[1]]
        if neighbor.coords not in closed_ls and not neighbor.isWall and not neighbor.isPit:
           if (neighbor.parent is None or 
              (neighbor.parent is not None and neighbor.parent.value > node.value)):
              open_ls.put(neighbor, neighbor.value)
              neighbor.parent = node

     if (node.coords[0]  + 1) < map_height:
        neighbor = grid[node.coords[0] +1][node.coords[1]]
        if neighbor.coords not in closed_ls and not neighbor.isWall and not neighbor.isPit:
           if (neighbor.parent is None or 
              (neighbor.parent is not None and neighbor.parent.value > node.value)):
              open_ls.put(neighbor, neighbor.value)
              neighbor.parent = node
        
     if (node.coords[1]  - 1) >= 0:
        neighbor = grid[node.coords[0]][node.coords[1]-1]
        if neighbor.coords not in closed_ls and not neighbor.isWall and not neighbor.isPit:
           if (neighbor.parent is None or 
              (neighbor.parent is not None and neighbor.parent.value > node.value)):
              open_ls.put(neighbor, neighbor.value)
              neighbor.parent = node
 
     if (node.coords[1]  + 1) < map_width:
        neighbor = grid[node.coords[0]][node.coords[1]+1]
        if neighbor.coords not in closed_ls and not neighbor.isWall and not neighbor.isPit:
           if (neighbor.parent is None or 
              (neighbor.parent is not None and neighbor.parent.value > node.value)):
              open_ls.put(neighbor, neighbor.value)
              neighbor.parent = node

def find_astar_path():
   heur = get_heuristic()
   bw = get_backward_cost()

   astar_map = copy.deepcopy(heur)

   for h in range(0, map_height):
      for w in range(0, map_width):
         astar_map[h][w].value = heur[h][w].value + bw[h][w].value
         astar_map[h][w].parent = None

   compute_path(astar_map)

   node = astar_map[goal[0]][goal[1]]
   path_ls = []
   while node.parent is not None:
      path_ls = [node.coords] + path_ls
      node = node.parent
   path_ls = [start] + path_ls

   return path_ls

class aStarNode():
   def __init__(self, value):
      self.coords = None
      self.parent = None
      self.value  = value
      self.isPit = False
      self.isWall = False
      self.isStart = False
      self.isGoal = False


   def set_coords(self, coords):
      self.coords = coords

