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

### For testing ###
c_Map = np.ones((360, 480))
np.seed(40)


class PRM:
    def __init__(self):
        self.m_map = c_Map
        self.m_num_samples = NUM_SAMPLES
        self.m_search_rad = SEARCH_RADIUS
        self.m_nodes = []
        # hp.heapify(self.m_nodes)

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

            if i == len(self.m_nodes)-1:
                self.m_nodes.pop(0)

    # Distance between a Point and a Line formula
    """ ## ADD YO XO Obstacle space """
    def edge_check(self):
        numerator = np.abs(((self.x2 - self.x1)*(self.y1 - self.y0)) - ((self.x1 - self.x0)*(self.y2 - self.y1)))
        denominator = np.sqrt((self.x2 - self.x1)**2 + (self.y2 - self.y1)**2)
        dist = numerator/denominator
        if dist <= 0.3: return False
        else: return True

    # Create a node with parent and child links
    def create_edge_node(self, px1, py1, px2, py2, idx):

        self.pr_idx = -1
        self.cur_idx = idx

        # (parent node index, child node index, (x, y))

        parent_node = (self.pr_idx, self.cur_idx, (px1, py1))
        child_node = (self.cur_idx, )

        # modify this to establish link


        return gen_node


def map_gen():
    map = np.zeros((250,400))
    c=15
    for y in range(1,map.shape[0]+1):
        for x in range(1, map.shape[1]+1):
            if x>=165-c and x<=235+c and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(235+c/1.414)))*(x-(235+c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(165-c/1.414)))*(x-(165-c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((165-c/1.414)-(200)))*(x-(200))+map.shape[0]-(60-c)>=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((235+c/1.414)-200))*(x-200)+map.shape[0]-(60-c)>=y:
                map[y-1][x-1]=10000
            if ((map.shape[0]-(210+c/1.414)-map.shape[0]+(185))/((115+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185<=y and ((map.shape[0]-(100-c/1.414)-map.shape[0]+185)/((105+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185>=y and ((map.shape[0]-(210+c/1.414)-map.shape[0]+180)/((115+c/1.414)-(75+c))*(x-(75+c))+map.shape[0]-180>=y or ((map.shape[0]-180-map.shape[0]+(100-c/1.414))/((75+c)-(105+c/1.414)))*(x-(105+c/1.414))+map.shape[0]-(100-c/1.414)<=y):
                map[y-1][x-1]=10000
            if (x-300)**2+(y-map.shape[0]+185)**2<=(40+c)**2:
                map[y-1][x-1]=10000
# if x>250 and x<275:
#	map[y-1][x-1]=1
    		if x>0 and x<=c:
                map[y - 1][x - 1] = 10000
            if x>400-c and x<=400:
                map[y - 1][x - 1] = 10000
            if y>0 and y<=c:
                map[y - 1][x - 1] = 10000
            if y>250-c and y<=250:
                map[y - 1][x - 1] = 10000
    return map


def main():



if __name__ == '__main__':
    main()


