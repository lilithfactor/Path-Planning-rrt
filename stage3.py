'''
STAGE 3
no max distance
no closest node check
no tree like structure
'''
import matplotlib.pyplot as plt
# import shapely
from shapely.geometry import LineString
from shapely.geometry import Polygon
# from shapely.geometry import MultiPolygon as mpg
from shapely.geometry import Point
import random

class Node:
    # for parent node
    parent = None
    # for new random node
    newNode = None 
    # for position
    position = None
    # accepts parent node and new random node of type Point
    def __init__(self, parent, newNode, position):
        self.parent = parent
        self.newNode = newNode
        self.position = position

# function to return True if new Node isnt Present in any obstacle, 
# else False
def isNodeOkay(newNode):
    isOkay = True
    for obs in obsList:
        if newNode.within(obs):
            isOkay = False
            # print('Node is Not Okay')
            return isOkay
    return isOkay

# function to check if line crosses another Obstacle
# returns true if no cross
def isLineOkay(newLine):
    isOkay = True
    for obs in obsList:
        if newLine.crosses(obs):
            isOkay = False
            # print('Line is Not Okay')
            return isOkay
    return isOkay

def plotter(path):
    pass

# sample goal after x interval
def goalSampler(itr):
    pass

# function to repeat procedure 5 times
def driver(n, start, goal):
    plt.scatter(start.x, start.y, marker='x', color='yellow')
    plt.scatter(goal.x, goal.y, marker='x', color='green')
    # parent = Node()
    # newnode = Node(parent);

    path = []
    path.append(start)
    # node to calculate distance
    prev = start
    # keep track of number of nodes placed
    nodeCtr=0
    itr=0
    goalSampleCtr = 0
    while nodeCtr<n:
        itr+=1
        # increment goal sample ctr to check if 25 iterations are complete
        goalSampleCtr+=1

        # add class object here to sample nodes
        
        # sampling goal after 20 iterations
        if goalSampleCtr==20:
            (x, y) = (goal.x, goal.y)
            # taking new node as the goal
            newNode = Point(x, y)
            # reinitializing goalSampleCtr
            goalSampleCtr = 0
        else:
            # choosing random point within given range
            (x,y) = (random.uniform(0, 10), random.uniform(0, 10))
            newNode = Point(x, y)

        # if Node isnt present inside any Obstacle
        if isNodeOkay(newNode):
            # making a new line that joins new node to current node
            newLine = LineString((prev, newNode))
            # if line joining it doesnt cross Obstacle
            if isLineOkay(newLine):
                # add lines !!
                # line = LineString((prev, newNode))
                # plt.plot(newLine)
                plt.plot([prev.x, newNode.x], [prev.y, newNode.y])

                # incrementing number of nodes
                nodeCtr+=1
                # reinitializing prev as the new node
                prev = newNode
                # adding node to List
                path.append(newNode)
                plt.scatter(newNode.x, newNode.y, s=1.1)
                # print(path)
                
                if newNode == goal:
                    print(f'Goal Reached in {itr} iterations using {nodeCtr} nodes.')
                    return path
            
    print(f'Couldnt reach goal in {itr} iterations, using {nodeCtr} nodes')
    return path

# global obstacles
# Given Obstacle List (consists of multiple polygons)
# Obstacle list from Assignment
obstacle_list = [
    [(2, 10), (7, 10), (6, 7), (4, 7), (4, 9), (2, 9)],
    [(3, 1), (3, 6), (4, 6), (4, 1)],
    [(7, 3), (7, 8), (9, 8), (9, 3)]
]
# preparing polygon List for checking 
obsList = []
for obs in obstacle_list:
    obsList.append(Polygon(obs))
    
plt.plot(*obsList[0].exterior.xy, label='obstacle 1')
plt.plot(*obsList[1].exterior.xy, label='obstacle 2')
plt.plot(*obsList[2].exterior.xy, label='obstacle 3')

# start node and goal Node
start = Point(1, 1)
goal = Point(10, 10)

# setting range of graph
plt.xlim(0, 11)
plt.ylim(0, 11)

# n = int(input('Enter number of Nodes: '))
# D = float(input('Enter Max Distance between new node and Current Node: '))
n = 200
pathTrajectory = driver(n, start, goal)
plotter(pathTrajectory)
plt.show()
