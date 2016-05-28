#!/usr/bin/env python
import rospy
import astar as ast
import mdp   as mdp

from read_config import read_config
from cse_190_assi_3.msg import AStarPath, PolicyList
from std_msgs.msg import Bool

#robot.py implementation goes here

class robot():
   def __init__(self):
      self.mdp_done = False
      self.aStar_done = False

      rospy.init_node('robot_node', anonymous = True)
      self.init_config()
      self.init_pubs()

      rospy.sleep(2)
      self.compute_aStar_map() 
      self.compute_mdp_path()

      if self.mdp_done and self.aStar_done:
         self.sim_publisher.publish(True)
         rospy.sleep(2)
         rospy.signal_shutdown('done')

      rospy.spin()


   def compute_mdp_path(self):
      policy = mdp.get_mdp_policy(self.mdp_publisher) 
      self.mdp_done = True

   def compute_aStar_map(self):
      path = ast.find_astar_path()
      for p in path:
         rospy.sleep(1)
         self.path_list_publisher.publish(p)
      self.aStar_done = True

      

   def init_config(self):
      self.config       = read_config()
      self.move_list    = self.config["move_list"] 
      map_size_arr      = self.config["map_size"]
      self.map_height   = map_size_arr[0]
      self.map_width    = map_size_arr[1]
      self.start        = self.config["start"]
      self.goal         = self.config["goal"]
      self.walls        = self.config["walls"]
      self.pits         = self.config["pits"]

   def init_pubs(self):
      self.path_list_publisher = rospy.Publisher('/results/path_list', AStarPath,
                                          queue_size = 10)

      self.mdp_publisher = rospy.Publisher('/results/policy_list',PolicyList ,
                                          queue_size = 10)
      self.sim_publisher = rospy.Publisher('/map_node/sim_complete',Bool ,
                                          queue_size = 10)

if __name__ == "__main__":
   robot_node = robot()
