"""
ENPM 661 Path Planning for Autonomous Robots
Probabilistic RoadMap
Author: Akhilrajan (v.akhilrajan@gmail.com)
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
import heapq as hp

# PARAMETERS #
NUM_SAMPLES = 100
SEARCH_RADIUS = 100
EDGE_DISTANCE = 20

### For testing###
c_Map = np.ones((360, 480))
np.seed(40)

class PRM:
    def __init__(self):
        self.m_map = c_Map
        self.m_num_samples = NUM_SAMPLES
        self.m_search_rad = SEARCH_RADIUS
        self.m_nodes = []
        hp.heapify(self.m_nodes)

    def get_nodes(self):
        for i in range(self.m_num_samples):
            self.x = np.random.randint(0, self.m_map[0])
            self.y = np.random.randint(0, self.m_map[1])

            if self.check_node():
                np.append(self.m_nodes, [self.x, self. y])

            elif self.check_node() is False:
                continue

    def check_node(self):
        if self.m_map[self.x] !=0 and self.m_map[self.y] !=0:
            return False
        else:
            return True

    def connect_edge(self):
        for i in range(0, len(self.m_nodes)):
            self.x1 = self.m_nodes[0][0]
            self.y1 = self.m_nodes[0][1]
            self.x2 = self.m_nodes[i][0]
            self.y2 = self.m_nodes[i][1]

            if self.edge_check():
                self.edge = self.create_edge_node(self.x1, self.y1, self.x2, self.y2, i)

            elif self.edge_check() is False: continue

    # Distance between a Point and a Line formula
    def edge_check(self):
        numerator = np.abs(((self.x2 - self.x1)*(self.y1 - self.y0)) - ((self.x1 - self.x0)*(self.y2 - self.y1)))
        denominator = np.sqrt((self.x2 - self.x1)**2 + (self.y2 - self.y1)**2)
        dist = numerator/denominator
        if dist <= 0.3: return False
        else: return True

    # Create a node with parent and child links
    def create_edge_node(self, px1, py1, px2, py2, idx):



        return gen_node



def main():



if __name__ == '__main__':
    main()


