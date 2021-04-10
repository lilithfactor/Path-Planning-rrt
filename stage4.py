'''
STAGE 4
REQD:
    - use class for creating nodes and for stack structure
    - add max distance to choose node in direction of random node
    - choose closest node in path to random node by traversing through path
    - traverse through whole path at end to print final path
'''
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point
import random
import numpy as np

# class for tracking tree
class Node:
    # for parent node
    parent = None
    # for position
    position = None
    # accepts new random node of type Point and parent node
    def __init__(self, position, parent=None):
        self.parent = parent
        self.position = position

# function to return True if randomPoint isnt present in any obstacle, 
def isNodeOkay(randomPoint):
    isOkay = True
    # traversing through Obstacle List
    for obs in obsList:
        # change isOkay to false if randomPoint is within any obstacle and return
        if point.within(obs):
            isOkay = False
            return isOkay

    return isOkay

# function to return True if line doesnt cross any obstacle
def isLineOkay(line):
    isOkay = True
    # traversing through Obstacle List
    for obs in obsList:
        # change isOkay to false if line crosses any obstacle, and return
        if line.crosses(obs):
            isOkay = False
            return isOkay

    return isOkay

# function to return point at distance d from head towards tail
def nodeonVectoratD(head, tail, d):
    (b,a) = (np.array(head, dtype=float), np.array(tail, dtype=float))
    n = b - a
    n /= np.linalg.norm(n, 2)
    point = b - d * n
    # returning point of type Point 
    return Point(point)

# function to traverse back to root of tree and plot path
def traverseBack(node):
    # while we havent reached the root of tree
    while(node!=start):
        # plot line segments joining node and its Parent
        plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='green')
        # decrementing node, moving towards root
        node = node.parent
        
def driver(n, start, goal, d):
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
        # increment goal sample ctr to check if 20 iterations are complete
        goalSampleCtr+=1

        # sampling goal after 20 iterations
        if goalSampleCtr==20:
            line = LineString([(prev.x, prev.y), (goal.x, goal.y)])
            if isLineOkay(line): # !!
                (x, y) = (goal.x, goal.y)
                # taking new node as the goal
                newPoint = Point(x, y)

                newNode = Node(newPoint)
                # adding parent node to tree
                newNode.parent = prev

                # reinitializing goalSampleCtr
                goalSampleCtr = 0

        else:
            (x,y) = (random.uniform(0, 10), random.uniform(0, 10))
            newPoint = Point(x, y)
            # checking if point is obstacle free
            if isNodeOkay(newPoint):
                # checking if line joining this point and 
                # prev is obstacle free so that it can be 
                # added to path
                line = LineString([(prev.x, prev.y), (x, y)])
                if isLineOkay(line):
                    # node at distance d on vector prev->newNode
                    newPoint = nodeonVectoratD(prev, newPoint, d)
                    # changing newNode to type Node()
                    newNode = Node(newPoint)
                    # adding parent node to tree
                    newNode.parent = prev
        
        # if Node isnt present inside any Obstacle
        if isNodeOkay(newPoint):
            # making a new line that joins new node to current node
            newLine = LineString((prev, newPoint))
            # if line joining it doesnt cross Obstacle
            if isLineOkay(newLine):
                # plot line segments
                plt.plot([prev.x, newPoint.x], [prev.y, newPoint.y], color='red')
                # incrementing number of nodes
                nodeCtr+=1
                # reinitializing prev as the new node
                prev = newPoint
                # adding node to List
                # path.append(newNode)
                # plot nodes
                plt.scatter(newPoint.x, newPoint.y, s=1.1)
            
                if newPoint == goal:
                    print(f'Goal Reached in {itr} iterations using {nodeCtr} nodes.')
                    print('\nTraversing Back.')
                    traverseBack(newNode);
                    # return path   
    
    else:
        print(f'\nCouldnt Reach Goal. (iterations: {itr}, nodes: {nodeCtr})')
    # return path

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

n = 2000
D = 1
driver(n, start, goal, D)
plt.show()
