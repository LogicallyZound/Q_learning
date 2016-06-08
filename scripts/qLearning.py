#!/usr/bin/env python
import copy
import rospy
import random
from read_config import read_config


config = read_config()
start       = config["start"]
goal        = config["goal"]
walls       = config["walls"]
pits        = config["pits"]
map_siz_arr = config["map_size"]
map_height  = map_siz_arr[0]
map_width   = map_siz_arr[1]
step_cost   = config["reward_for_each_step"]

max_iter       = config["max_iterations"]
threshold_diff = config["threshold_difference"]
reward_step    = config["reward_for_each_step"]
reward_wall    = config["reward_for_hitting_wall"]
reward_goal    = config["reward_for_reaching_goal"]
reward_pit     = config["reward_for_falling_in_pit"]
discount_fact  = config["discount_factor"]
learning_rate  = config["learning_rate"]

prob_forward   = config["prob_move_forward"]
prob_back      = config["prob_move_backward"]
prob_left      = config["prob_move_left"]
prob_right     = config["prob_move_right"]

robot_loc      = start
grid           = []
moves          = ['N', 'S', 'W', 'E']

class qLearning():
   def __init__(self):
      init = 1     

   def start(self):
      self.init_grid()  

      self.take_a_step()

   def take_a_step(self):
      #TODO: add in the weighs
      
      total_change = 0
      isAbsorbed = False 
      while True:
         #random if all the values are equal
         if grid[robot_loc[0]][robot_loc[1]].all_equal():
            move_n    = random.randint(0, 3)
            move = moves[move_n]
            old_val = grid[robot_loc[0]][robot_loc[1]].N_val
         else:
            move, old_val = grid[robot_loc[0]][robot_loc[1]].max_move_qval()

         old_loc = copy.deepcopy(robot_loc)
         moved = self.make_move(move)

         move_ls = []
         step_cost = 0
         if moved:
            move_ls = self.move_to_ls(move)
            step_cost = reward_step
         else:
            move_ls = [0,0]
            step_cost = reward_wall

         if grid[robot_loc[0]][robot_loc[1]].isGoal:
            step_cost = reward_goal
            isAbsorbed = True
         if grid[robot_loc[0]][robot_loc[1]].isPit:
            step_cost = reward_pit
            isAbsorbed = True

         Qold    = grid[old_loc[0]][old_loc[1]].get_move_val(move)

         mov, m_val = grid[robot_loc[0]][robot_loc[1]].max_move_qval()

         Ui      = step_cost + discount_fact*m_val
         Qnow    = (1-learning_rate)*Qold + learning_rate*Ui

         grid[old_loc[0]][old_loc[1]].set_val(move, Qnow)

         if isAbsorbed:
            return True

   def move_to_ls(self, move):
      if move == 'N':
         return [1, 0]
      if move == 'S':
         return [-1,0]
      if move == 'W':
         return [-1, 0]
      if move == 'E':
         return [1, 0]




   def make_move(self, move):
      h = robot_loc[0]
      w = robot_loc[1]
      moved = False
      if move == 'N' and h-1 >= 0 and not grid[h-1][w].isWall:
         robot_loc[0] = robot_loc[0]-1
         moved = True
      if move == 'S' and h+1 < map_height and not grid[h+1][w].isWall:
         robot_loc[0] = robot_loc[0]+1
         moved = True
      if move == 'W' and w-1 >= 0 and not grid[h][w-1].isWall:
         robot_loc[1] = robot_loc[1]-1
         moved = True
      if move == 'E' and w+1 >= map_width and not grid[h][w+1].isWall:
         robot_loc[1] = robot_loc[1]+1
         moved = True
      return moved
   
   def init_grid(self):
      for i in range(0, map_height):
         grid.append([])
         for j in range(0, map_width):
            grid[i].append(qNode(0))

      grid[start[0]][start[1]].isStart = True
      grid[start[0]][start[1]].set_coords(start)

      grid[goal[0]][goal[1]].isGoal = True
      grid[goal[0]][goal[1]].obs = 'GOAL'
      grid[goal[0]][goal[1]].set_coords(goal)
      grid[goal[0]][goal[1]].value = reward_goal

      for w in range(0, len(walls)):
         wall = walls[w]
         grid[wall[0]][wall[1]].isWall = True
         grid[wall[0]][wall[1]].obs = 'WALL'
      
      for p in range(0, len(pits)):
         pit = pits[p]
         grid[pit[0]][pit[1]].isPit = True
         grid[pit[0]][pit[1]].obs = 'PIT'
         grid[pit[0]][pit[1]].value = reward_pit

      #init the coords
      for h in range(0, map_height):
         for w in range(0, map_width):
            grid[h][w].set_coords([h,w])


class qNode():
   def __init__(self, value):
      self.N_val = 0
      self.S_val = 0
      self.W_val = 0
      self.E_val = 0

      N_n = 0
      S_n = 0
      W_n = 0
      E_n = 0

      self.coords = None
      self.parent = None
      self.value  = value
      self.isPit = False
      self.isWall = False
      self.isStart = False
      self.isGoal = False
      self.obs = None
      self.move   = None



   def get_move_val(self, move):
      if move == 'N':
         return self.N_val
      if move == 'S':
         return self.S_val
      if move == 'W':
         return self.W_val
      if move == 'E':
         return self.E_val

   def set_coords(self, coords):
      self.coords = coords

   def max_move_qval(self):
      m = self.N_val
      mov = 'N'
      if self.S_val > m:
         m   = self.S_val
         mov = 'S'
      if self.W_val > m:
         m   = self.W_val
         mov = 'W'
      if self.E_val > m:
         m   = self.E_val
         mov = 'E'

      return mov, m

   def all_equal(self):
      return (self.N_val == self.S_val and self.S_val == self.W_val and self.W_val == self.E_val)

   def set_val(self, move, val):
      if move == 'N':
         self.N_val = val
      if move == 'S':
         self.S_val = val
      if move == 'W':
         self.W_val = val
      if move == 'E':
         self.E_val = val
         
