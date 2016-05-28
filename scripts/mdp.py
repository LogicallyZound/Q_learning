# mdp implementation needs to go here
from read_config import read_config
import copy
import itertools
import rospy

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

prob_forward   = config["prob_move_forward"]
prob_back      = config["prob_move_backward"]
prob_left      = config["prob_move_left"]
prob_right     = config["prob_move_right"]

def construct_grid():    
   grid = [] 
   for i in range(0, map_height):
      grid.append([])
      for j in range(0, map_width):
         grid[i].append(aStarNode(0))

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

   return grid


def get_max_move(h, w, grid):
   canMoveUp    = 0
   canMoveDown  = 0
   canMoveLeft  = 0
   canMoveRight = 0

   wallAbove = 0
   wallBelow = 0
   wallLeft = 0
   wallRight = 0

   upVal    = 0
   downVal  = 0
   leftVal  = 0
   rightVal = 0

   upMoveReward    = 0
   downMoveReward  = 0
   leftMoveReward  = 0
   rightMoveReward = 0
   step_Val  = 0

   
   if (h - 1) >= 0 and not grid[h-1][w].isWall:
      canMoveUp = 1
      upVal     = grid[h-1][w].value
      step_val     = reward_step
   else:
      wallAbove = 1
      upVal     = grid[h][w].value

   if (h + 1) < map_height and not grid[h+1][w].isWall:
      canMoveDown = 1
      downVal     = grid[h+1][w].value
      step_val     = reward_step
   else:
      wallBelow = 1
      downVal     = grid[h][w].value

   if (w - 1) >= 0 and not grid[h][w-1].isWall: 
      canMoveLeft = 1
      leftVal     = grid[h][w-1].value
      step_val     = reward_step
   else:
      wallLeft = 1
      leftVal     = grid[h][w].value

   if (w + 1) < map_width and not grid[h][w+1].isWall:
      canMoveRight = 1
      rightVal     = grid[h][w+1].value
      step_val     = reward_step
   else:
      wallRight = 1
      rightVal     = grid[h][w].value
   
   moveN = 0
   moveS = 0
   moveW = 0
   moveE = 0

   if wallAbove:
      moveN += prob_forward * (reward_wall + discount_fact* (upVal))
      moveW += prob_right   * (reward_wall + discount_fact* (upVal))
      moveE += prob_left    * (reward_wall + discount_fact* (upVal))
      moveS += prob_back    * (reward_wall + discount_fact* (upVal))
   else:
      moveN += prob_forward * (reward_step + discount_fact* (upVal))
      moveW += prob_right   * (reward_step + discount_fact* (upVal))
      moveE += prob_left    * (reward_step + discount_fact* (upVal))
      moveS += prob_back    * (reward_step + discount_fact* (upVal))

   if wallBelow:
      moveN += prob_back    * (reward_wall + discount_fact* (downVal))
      moveW += prob_left    * (reward_wall + discount_fact* (downVal))
      moveE += prob_right   * (reward_wall + discount_fact* (downVal))
      moveS += prob_forward * (reward_wall + discount_fact* (downVal))
   else:
      moveN += prob_back    * (reward_step + discount_fact* (downVal))
      moveW += prob_left    * (reward_step + discount_fact* (downVal))
      moveE += prob_right   * (reward_step + discount_fact* (downVal))
      moveS += prob_forward * (reward_step + discount_fact* (downVal))

   if wallLeft:
      moveN += prob_left    * (reward_wall + discount_fact* (leftVal))
      moveW += prob_forward * (reward_wall + discount_fact* (leftVal))
      moveE += prob_back    * (reward_wall + discount_fact* (leftVal))
      moveS += prob_right   * (reward_wall + discount_fact* (leftVal))
   else:
      moveN += prob_left    * (reward_step + discount_fact* (leftVal))
      moveW += prob_forward * (reward_step + discount_fact* (leftVal))
      moveE += prob_back    * (reward_step + discount_fact* (leftVal))
      moveS += prob_right   * (reward_step + discount_fact* (leftVal))

   if wallRight:
      moveN += prob_right  * (reward_wall + discount_fact* (rightVal))
      moveW += prob_back   * (reward_wall + discount_fact* (rightVal))
      moveE += prob_forward * (reward_wall + discount_fact* (rightVal))
      moveS += prob_left   * (reward_wall + discount_fact* (rightVal))
   else:
      moveN += prob_right  * (reward_step + discount_fact* (rightVal))
      moveW += prob_back   * (reward_step + discount_fact* (rightVal))
      moveE += prob_forward * (reward_step + discount_fact* (rightVal))
      moveS += prob_left   * (reward_step + discount_fact* (rightVal))


   move_vals = [('N', moveN), ('S', moveS), ('W', moveW), ('E', moveE)]
   
   ma = max(move_vals, key=lambda item:item[1])

   return ma

def get_mdp_policy(pub):
   model_grid   = construct_grid()
   t_grid       = copy.deepcopy(model_grid)
   t_plus1_grid = copy.deepcopy(model_grid)
   su1 = 0
   su2 = 0

   for t in range (0, max_iter):
      for h in range(0, map_height):
         for w in range(0, map_width):
            if (not t_grid[h][w].isWall and not t_grid[h][w].isPit
                and not t_grid[h][w].isGoal ):
               move, val = get_max_move(h, w, t_grid)
               t_plus1_grid[h][w].move = move
               t_plus1_grid[h][w].value = val

               su2 += abs(t_grid[h][w].value-t_plus1_grid[h][w].value)     

      t_grid = copy.deepcopy(t_plus1_grid)

      return_grid = [] 
      for i in range(0, map_height):
         return_grid.append(['u']*map_width)

      for h in range(0, map_height):
         for w in range(0, map_width):
            return_grid[h][w] = t_grid[h][w].move if t_grid[h][w].move is not None else t_grid[h][w].obs
   


      flat_grid = itertools.chain(*return_grid)
      rospy.sleep(1) 
      pub.publish(list(flat_grid))

      print su2
      if su2 < threshold_diff:
         return return_grid
      else:
         su2 = 0



   return return_grid

class aStarNode():
   def __init__(self, value):
      self.coords = None
      self.parent = None
      self.value  = value
      self.isPit = False
      self.isWall = False
      self.isStart = False
      self.isGoal = False
      self.obs = None
      self.move   = None


   def set_coords(self, coords):
      self.coords = coords

