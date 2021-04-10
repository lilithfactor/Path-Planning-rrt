'''
STAGE 5 - FINAL STAGE
ERROR SOLVING
'''
'''
PROCEDURE: UPDATE REQUIRED
    choosing random point in given range

    convert point into node using class, pass parent and new node
    checking if it lies inside given obstacle list
    checking if line crosses any obstacle
        adding it to path
        traversing to start of 
    plotting path
'''
'''
############		IMPORT STATEMENTS
'''
import math
import matplotlib.pyplot as plt
import numpy as np
import random
from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.geometry import Polygon

# class for tracking tree
class Node:
    # for parent node
    parent = None
    # for position of random point (child)
    position = None
    # accepts new random node of type Point and parent node
    def __init__(self, position, parent=None):
        self.parent = parent
        self.position = position

'''
############		FUNCTION DEFINITIONS
'''
# function to return True if randomPoint isnt present in any obstacle, 
def isNodeOkay(randomPoint, obsList):
    isOkay = True
    # traversing through Obstacle List
    for obs in obsList:
        # change isOkay to false if randomPoint is within any obstacle and return
        if randomPoint.within(obs):
            isOkay = False
            return isOkay

    return isOkay

# function to return True if line doesnt cross any obstacle
def isLineOkay(line, obsList):
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
def visualize(node):
    # while we havent reached the root of tree
    while(node!=start):
        # plot line segments joining node and its Parent
        plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='green')
        # decrementing node, moving towards root
        node = node.parent

# function to find distance between 2 points
def distanceBetween(point1,point2):
     dist = math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)

     return dist

# function to return node at least distance from point in tree
def minDistNode(point, start):
    # assuming tail node is the closest
    leastNode = Node(point)
    temp = Node(point)
    
    leastDist = distanceBetween(leastNode.position, point)
    # print(leastNode.position, point, leastDist)

    while temp!=start:
        if distanceBetween(temp.position, point)<leastDist:
            leastNode=temp;
        # decrementing node, moving towards root
        temp=temp.parent
        # print(temp.position)

    return leastNode

# function to implement rrt algorithm
def rrt(n, start, goal, d, obsList):
	# plotting Start and Goal
    plt.scatter(start.x, start.y, marker='x', color='yellow')
    plt.scatter(goal.x, goal.y, marker='x', color='green')

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

        '''
        CODE FOR SELECTION OF RANDOM NODES
        GOAL SAMPLED AFTER 20 ITERATIONS
        '''
        '''
            1. choose random point (newPoint)
            2. call minDistNode(newPoint) to return node closest(closestNode)
			3. create new node (branchNode) and set position as newPoint
			4. set parent of branchNode as closestNode
        '''
        # sampling goal after 20 iterations
        if goalSampleCtr==20:
            line = LineString([(prev.x, prev.y), (goal.x, goal.y)])
            if isLineOkay(line, obsList):
                (x, y) = (goal.x, goal.y)
                # taking new node as the goal
                newPoint = Point(x, y)
                closestNode = minDistNode(newPoint)
                branchNode = Node(newPoint)
                # adding parent node to tree
                branchNode.parent = closestNode
                # reinitializing goalSampleCtr
                goalSampleCtr = 0;
        else:
            (x,y) = (random.uniform(0, 10), random.uniform(0, 10))
            newPoint = Point(x, y)
            # checking if point is obstacle free
            if isNodeOkay(newPoint, obsList):
            	# line joining current node to random point
                line = LineString([(prev.x, prev.y), (x, y)])
                # checking if line joining this point and 
                # prev is obstacle free so that it can be 
                # added to tree
                if isLineOkay(line, obsList):
                	# node at distance d on vector prev->newNode
                    newPoint=nodeonVectoratD(prev, newPoint, d)
                    # getting closestNode from newPoint in tree
                    closestNode=minDistNode(newPoint, start);
                    # creating new branchNode which points to newPoint
                    branchNode=Node(newPoint);
	                # adding branchNode to tree with parent closestNode
                    branchNode.parent=closestNode;
        # if Node isnt present inside any Obstacle
        if isNodeOkay(newPoint, obsList):
            # making a new line that joins new node to current node
            newLine = LineString((prev, newPoint))
            # if line joining it doesnt cross Obstacle
            if isLineOkay(newLine, obsList):
                # plot line segments
                plt.plot([prev.x, newPoint.x], [prev.y, newPoint.y], color='red')
                # incrementing number of nodes
                nodeCtr+=1
                # reinitializing prev as the new node
                prev = newPoint
                # adding node to List
                path.append(newNode)
                # plot nodes
                plt.scatter(newPoint.x, newPoint.y, s=1.1)
                if newPoint == goal:
                    print(f'Goal Reached in {itr} iterations using {nodeCtr} nodes.')
                    print('\nTraversing Back.')
                    # returning end node for visualize()
                    return newNode   

    # if goal isnt reached and while loop terminates normally
    print(f'\nCouldnt Reach Goal. (iterations: {itr}, nodes: {nodeCtr})')

# driver function to run rrt(), visualize()
def test_rrt():
    # setting range of graph
    plt.xlim(0, 11)
    plt.ylim(0, 11)

    # Given Obstacle List
    obstacle_list = [
        [(2, 10), (7, 10), (6, 7), (4, 7), (4, 9), (2, 9)],
        [(3, 1), (3, 6), (4, 6), (4, 1)],
        [(7, 3), (7, 8), (9, 8), (9, 3)]
    ]

	# preparing list of polygon obstacles 
    obsList = []
    for obs in obstacle_list:
        plt.plot(*Polygon(obs).exterior.xy)
        obsList.append(Polygon(obs));
	    
	# plt.plot(*obsList[0].exterior.xy, label='obstacle 1')
	# plt.plot(*obsList[1].exterior.xy, label='obstacle 2')
	# plt.plot(*obsList[2].exterior.xy, label='obstacle 3')

    # start node and goal node
    start = Point(1, 1)
    goal = Point(10, 10)

    '''
    ############ TEMPORARY INPUT REGION
    '''
    # n = 2000 # define number of nodes
    # D = 0.5 # define max distance between parent and child
    n = int(input('number of Nodes: '))
    D = float(input(' max distance between child and parent: '))
    child = rrt(n, start, goal, D, obsList)
    visualize(child)
    plt.show()

test_rrt()