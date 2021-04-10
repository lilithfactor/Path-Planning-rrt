'''
STAGE 5 - Version 2 Branch
ERROR SOLVING
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
def isPointOkay(randomPoint, obsList):
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
def pointonVectoratD(head, tail, d):
    (b,a) = (np.array(head, dtype=float), np.array(tail, dtype=float))
    n = b - a
    n /= np.linalg.norm(n, 2)
    point = b - d * n
    # returning point of type Point 
    return Point(point)

# function to find distance between 2 points
def distanceBetween(point1,point2):
     dist = math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)

     return dist

# function to return node at least distance from point in tree
# traversing from start till position is null
def minDistNode(point, start):
    # assuming tail node is the closest
    leastNode = Node(point)
    temp = Node(point)
    
    leastDist = distanceBetween(leastNode.position, point)

    while temp.parent!=None:
        if distanceBetween(temp.position, point)<leastDist:
            leastNode=temp;
        # decrementing node, moving towards root
        temp=temp.parent
    return leastNode

# function to traverse back to root of tree and plot path
def visualize(node, start):
    # while we havent reached the root of tree
    while node.parent!=None:
        # plot line segments joining node and its Parent
        plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='green')
        # decrementing node, moving towards root
        node = node.parent

# function to implement rrt algorithm
def rrt(n, start, goal, d, obsList):
	# plotting Start and Goal
    plt.scatter(start.x, start.y, marker='x', color='yellow')
    plt.scatter(goal.x, goal.y, marker='x', color='green')
    # to store the final path
    path = []
    path.append(start)
    # currNode is the current point in the tree
    currPoint = start
    # keep track of number of nodes placed
    nodeCtr=0
    # counter for number of iterations
    itr=0
    # counter for goal sampling (resets after 15)
    goalSampleCtr = 0
    # while the number of nodes placed is <= n
    while nodeCtr<=n:
        # total number of iterations
        # increment goal sample ctr to check if 15 iterations are complete
        (itr, goalSampleCtr) = (itr+1, goalSampleCtr+1)
        # sampling goal, set nextPoint as Goal after 15 iterations
        if goalSampleCtr==15:
            (x, y) = (goal.x, goal.y)
            # getting node closest to the goal from the tree
            # closestNode = minDistNode(newPoint)
            # creating a line joining goal and current point
            line = LineString([(currPoint.x, currPoint.y), (x, y)]) # check definition of currPoint
            # creating nextPoint as the goal if lineOkay and nodeOkay
            nextPoint = Point(goal.x, goal.y)
            # checking if goal can be reached
            if isLineOkay(line, obsList):    
                # updating nextNode as goal
                nextNode = Node(nextPoint)
                # setting nextNode parent as currNode
                nextNode.parent = currNode
                # resetting goalSampleCtr
                goalSampleCtr = 0;

        # sampling random points
        else:
            # choosing random coordinates
            (x,y) = (random.uniform(0, 10), random.uniform(0, 10))
            # creating a newPoint with random coordinates
            newPoint = Point(x, y)
            # updating newPoint as point at distance d on vector currPoint->newPoint
            newPoint = pointonVectoratD(currPoint, newPoint, d)             
            # checking if newPoint is obstacle free
            if isPointOkay(newPoint, obsList):
            	# line joining current node to random point
                line = LineString([(currPoint.x, currPoint.y), (x, y)])
                # checking if line is obstacle free 
                if isLineOkay(line, obsList):
                    # setting nextPoint as newPoint
                    nextPoint = newPoint
                    # getting closestNode from newPoint in tree which is collision free
                    closestNode=minDistNode(nextPoint, start);
                    # updating nextNode with newPoint
                    nextNode=Node(nextPoint);
	                # adding nextNode to tree with parent closestNode
                    nextNode.parent=closestNode;
                    # print(closestNode.parent, closestNode.position, 'hey')

        # if nextNode isnt present inside any Obstacle
        if isPointOkay(nextPoint, obsList):
            # making a new line that joins nextPoint to currPoint
            line = LineString((currPoint, nextPoint))
            # if line joining it doesnt cross Obstacle
            if isLineOkay(line, obsList):
                # plot line segments
                plt.plot([currPoint.x, nextPoint.x], [currPoint.y, nextPoint.y], color='red')
                # plot point
                plt.scatter(nextPoint.x, nextPoint.y, s=1., color = 'black')
                # incrementing number of nodes
                nodeCtr+=1
                # reinitializing currNode as the new node
                currPoint = nextPoint
                # adding node to List
                path.append(nextPoint)
                
                if nextPoint == goal:
                    print(f'Goal Reached in {itr} iterations using {nodeCtr} nodes.')
                    print('\nTraversing Back.')
                    # returning end node for visualize()
                    visualize(nextNode, start)
                    return nextNode   

    # if goal isnt reached and while loop terminates normally
    print(f'\nCouldnt Reach Goal in iterations: {itr}, nodes: {nodeCtr}')

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
    n = 1000 # define number of nodes
    D = 1 # define max distance between parent and child
    # n = int(input('number of Nodes: '))
    # D = float(input(' max distance between child and parent: '))
    child = rrt(n, start, goal, D, obsList)
    plt.show()


# calling main function
test_rrt()