import uav

import numpy as np

class pair_timeline:
    def __init__(self, i, j, step, dis):
        self.i    = i 
        self.j    = j 
        self.step = step 
        self.dis  = dis       
class dead_timeline:
    def __init__(self, i, j, step, dis, pi, pj, agent_i, agent_j):
        self.i          = i 
        self.j          = j 
        self.step       = step 
        self.dis        = dis 
        self.pi:list          = pi 
        self.pj:list          = pj 
        self.agent_i:uav.uav2D    = agent_i 
        self.agent_j:uav.uav2D    = agent_j 
from typing import List

g_p_list:List[pair_timeline]    = []
g_dead_list:List[pair_timeline] = []

