#!/usr/bin/env python

"""
ENPM661: Project 2

Akhilrajan Vethirajan (v.akhilrajan@gmail.com)
Vishaal Kanna Sivakumar (vishaal@terpmail.umd.edu)
M.Eng. Student, Robotics
University of Maryland, College Park

"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
import cv2
import math
import random


def map_gen():
    map = np.zeros((250,400))
    c=0
    for y in range(1,map.shape[0]+1):
        for x in range(1, map.shape[1]+1):
            # if x>=165-c and x<=235+c and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(235+c/1.414)))*(x-(235+c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(165-c/1.414)))*(x-(165-c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((165-c/1.414)-(200)))*(x-(200))+map.shape[0]-(60-c)>=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((235+c/1.414)-200))*(x-200)+map.shape[0]-(60-c)>=y:
            #     map[y-1][x-1]=10000
            # if ((map.shape[0]-(210+c/1.414)-map.shape[0]+(185))/((115+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185<=y and ((map.shape[0]-(100-c/1.414)-map.shape[0]+185)/((105+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185>=y and ((map.shape[0]-(210+c/1.414)-map.shape[0]+180)/((115+c/1.414)-(75+c))*(x-(75+c))+map.shape[0]-180>=y or ((map.shape[0]-180-map.shape[0]+(100-c/1.414))/((75+c)-(105+c/1.414)))*(x-(105+c/1.414))+map.shape[0]-(100-c/1.414)<=y):
            #     map[y-1][x-1]=10000
            # if (x-300)**2+(y-map.shape[0]+185)**2<=(40+c)**2:
            #     map[y-1][x-1]=10000
            #if x>250 and x<275:
            #	map[y-1][x-1]=1
            if x>=40+20 and x<=120+20 and y>=30 and y<=100:
                map[y - 1][x - 1] = 10000
            if x>=200+20 and x<=280+20 and y>=30 and y<=100:
                map[y - 1][x - 1] = 10000
            # if x>=200+20 and x<=240+20 and y>=30 and y<=100:
            #     map[y - 1][x - 1] = 10000
            # if x>=280+20 and x<=320+20 and y>=30 and y<=100:
            #     map[y - 1][x - 1] = 10000
            if x>=40+20 and x<=80+20 and y>=140 and y<=220:
                map[y - 1][x - 1] = 10000
            # if x>=120+20 and x<=160+20 and y>=140 and y<=220:
            #     map[y - 1][x - 1] = 10000
            if x>=200+20 and x<=240+20 and y>=140 and y<=220:
                map[y - 1][x - 1] = 10000
            if x>=280+20 and x<=320+20 and y>=140 and y<=220:
                map[y - 1][x - 1] = 10000
            if x>0 and x<=c:
                map[y - 1][x - 1] = 10000
            if x>400-c and x<=400:
                map[y - 1][x - 1] = 10000
            if y>0 and y<=c:
                map[y - 1][x - 1] = 10000
            if y>250-c and y<=250:
                map[y - 1][x - 1] = 10000
    return map

class Graph:
    def __init__(self, No_nodes, Search_radius):
        self.nodes = []
        self.r = Search_radius
        self.adj_list = {i:[] for i in range(No_nodes)}
        self.N = No_nodes

    def sample(self, start_node, Goal_nodes, map1, map_color):
        self.nodes.append([0, start_node])
        cv2.circle(map_color, (start_node[0], start_node[1]), 5, [255, 255, 255], 10)
        for i in range(len(Goal_nodes)):
            # d = ((start_node[0] - Goal_nodes[i][0]) ** 2 + (start_node[1] - Goal_nodes[i][1]) ** 2) ** 0.5
            self.nodes.append([i, Goal_nodes[i]])
            cv2.circle(map_color, (Goal_nodes[i][0], Goal_nodes[i][1]), 5, [255, 255, 255], 10)
        i = 0
        random.seed(10)
        while i < self.N-len(Goal_nodes)-1:
            x = random.randint(0, map1.shape[1] - 1)
            y = random.randint(0, map1.shape[0] - 1)
            if map1[y][x] == 10000:
                continue
            cv2.circle(map1, (x, y), 2, [255, 0, 0], 2)
            d = ((start_node[0]-x)**2+(start_node[1]-y)**2)**0.5
            self.nodes.append([d,[x, y]])
            i += 1
        self.nodes.sort()
        # cv2.imshow('Map with Nodes', map1)
        # cv2.waitKey(0)

    def roadmap(self, map1, map_color):
        for i in range(0, self.N):
            for j in range(i+1,self.N):
                if i==j:
                    continue
                d = ((self.nodes[i][1][0]-self.nodes[j][1][0])**2 + (self.nodes[i][1][1]-self.nodes[j][1][1])**2)**0.5
                if d < self.r:
                    x1 = self.nodes[i][1][0]
                    y1 = self.nodes[i][1][1]
                    x2 = self.nodes[j][1][0]
                    y2 = self.nodes[j][1][1]
                    l = x1
                    k = y1
                    in_obstacle = 0
                    x_sign = 1
                    y_sign = 1
                    # print(x1,x2,y1,y2)
                    if y2 == y1:
                        theta = (x2-x1)*np.pi / 2
                        if x2 > x1:
                            x_sign = 1
                            y_sign = 1
                        if x2 < x1:
                            x_sign = -1
                            y_sign = 1
                    elif x1 == x2:
                        theta = (x2-x1)*np.pi
                    else:
                        if x2>x1 and y2>y1:
                            m = (y2 - y1) / (x2 - x1)
                            theta = abs(math.atan(m))
                            x_sign = 1
                            y_sign = 1
                        elif x2<x1 and y2>y1:
                            m = (y2 - y1) / (x2 - x1)
                            theta = abs(math.atan(m))
                            x_sign = -1
                            y_sign = 1
                        elif x2<x1 and y2<y1:
                            m = (y2 - y1) / (x2 - x1)
                            theta = abs(math.atan(m))
                            x_sign = -1
                            y_sign = -1
                        else:
                            m = (y2 - y1) / (x2 - x1)
                            theta = abs(math.atan(m))
                            x_sign = 1
                            y_sign = -1
                    while int(l)!=x2 and int(k)!=y2 and int(l)>0 and int(k)>0 and int(l)<map1.shape[1]-1 and int(k)<map1.shape[0]-1:
                        l += 0.1*x_sign*math.cos(theta)
                        k += 0.1*y_sign*math.sin(theta)
                        # print(l,k)
                        if map1[int(k)][int(l)]==10000:
                            in_obstacle = 1
                            break
                    if in_obstacle==0 and len(self.adj_list[i])<10:
                        self.adj_list[i].append(j)
                        self.adj_list[j].append(i)
                        cv2.line(map_color, (self.nodes[i][1][0],self.nodes[i][1][1]) , (self.nodes[j][1][0],self.nodes[j][1][1]), [255,255,255], 1)
        # map_color = cv2.resize(map_color, (800,500))
        cv2.imshow('Map with Nodes', map_color)
        cv2.waitKey(0)

    def generate_path(self, lst, ClosedList, idx):
        for i in range(0, len(ClosedList)):
            if ClosedList[i][1] == idx:
                idx = i
                break
        if ClosedList[idx][2] == -1:
            lst.append(ClosedList[idx][3])
            return lst
        else:
            lst.append(ClosedList[idx][3])
            self.generate_path(lst, ClosedList, ClosedList[idx][2])

    def Astar(self, start_idx, goal_idx):
        # total cost, current index, parent index, (x, y, theta), cost to come
        start = (0, 0, -1, start_idx, 0)
        Open_list = [start]
        heapq.heapify(Open_list)
        Closed_list = []
        goal_reached = 0
        index = 0
        while len(Open_list) and not goal_reached:
            M = heapq.heappop(Open_list)
            Closed_list.append(M)
            idx = M[3]
            if idx == goal_idx:
                goal_reached = 1
                break
            # print(idx)
            # print(self.adj_list[idx])
            if not self.adj_list[idx]:
                continue
            for i in range(0,len(self.adj_list[idx])):
                count = 0
                for j in range(0,len(Closed_list)):
                    if self.adj_list[idx][i] == Closed_list[j][3]:
                        count=1
                if count==1:
                    continue
                index+=1
                C2C = M[4] + ((self.nodes[idx][1][0] - self.nodes[self.adj_list[idx][i]][1][0])**2 + (self.nodes[idx][1][1] - self.nodes[self.adj_list[idx][i]][1][1])**2)**0.5
                total_cost = C2C + ((self.nodes[goal_idx][1][0] - self.nodes[self.adj_list[idx][i]][1][0])**2 + (self.nodes[goal_idx][1][1] - self.nodes[self.adj_list[idx][i]][1][1])**2)**0.5
                count1=0
                for j in range(0,len(Open_list)):
                    if self.adj_list[idx][i] == Open_list[j][3]:
                        if total_cost < Open_list[j][0]:
                            Open_list[j] = (total_cost, Open_list[j][1], M[1], self.adj_list[idx][i], C2C)
                            heapq.heapify(Open_list) #heapq.heapreplace(Open_list, new_node)
                            count1=1
                if count1==0:
                    new_node = (total_cost, index, M[1], self.adj_list[idx][i], C2C)
                    heapq.heappush(Open_list, new_node)
            if len(Open_list) == 0 and not goal_reached:
                print('Solution Not Found')
                quit()
        return Closed_list

def main():
    No_nodes = 70
    start_node = [20,20]
    Goal_nodes = [[120,120],[250,240],[150,20],[350,50]]
    map = map_gen()
    map1 = map.copy()
    map_obstacle = np.zeros((250, 400))
    map_obstacle[map == 10000] = 1
    map_color1 = np.zeros((250, 400, 3))
    map_color1[:, :, 2] = map_obstacle * 255
    PRM = Graph(No_nodes, 100)
    PRM.sample(start_node, Goal_nodes,map1,map_color1)
    PRM.roadmap(map1,map_color1)
    all_paths = []
    for i in range(0,len(Goal_nodes)+1):
        for j in range(i+1, len(Goal_nodes)+1):
            Closed_list = PRM.Astar(i, j)
            paths = []
            paths.append(Closed_list[len(Closed_list) - 1][3])
            PRM.generate_path(paths, Closed_list, Closed_list[len(Closed_list) - 1][2])
            all_paths.append(paths)

    for n,path in enumerate(all_paths):
        map_obstacle = np.zeros((250, 400))
        map_obstacle[map == 10000] = 1
        map_color1 = np.zeros((250, 400, 3))
        map_color1[:, :, 2] = map_obstacle * 255
        for i in range(0,len(path)-1):
            cv2.line(map_color1, (PRM.nodes[path[i]][1][0], PRM.nodes[path[i]][1][1]),(PRM.nodes[path[i+1]][1][0], PRM.nodes[path[i+1]][1][1]), [255,0,0], 1)
        cv2.imshow('Map',map_color1)
        cv2.waitKey(0)

if __name__ == '__main__':
    main()

