'''
STAGE 6 
- changed basic structure of while loop
ERROR SOLVING
'''
'''
            		IMPORT STATEMENTS
'''
import math
import matplotlib.pyplot as plt
import numpy as np
import random
from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.geometry import Polygon

''' 
                    CLASS DEFINITION
'''
'''
for Tracking Tree and Path
stores 2 things:
    1. points position (type 'Point')
    2. Parent node (by default None)
''' 
class Node:
    # for parent node
    # parent = Point(1, 1)
    parent = None
    # for position of random point (child)
    position = None
    # accepts new random node of type Point and parent node
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
'''
            		FUNCTION DEFINITIONS
'''
'''
                    STATUS - WORKING
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
'''
                    STATUS - WORKING
'''
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
'''
                    STATUS - WORKING
'''
# function to return point at distance d from head towards tail
def pointonVectoratD(head, tail, d):
    (b,a) = (np.array(head, dtype=float), np.array(tail, dtype=float))
    n = b - a
    n /= np.linalg.norm(n, 2)
    point = b - d * n
    # returning point of type Point 
    return Point(point)
'''
                    STATUS - WORKING
'''
# function that samples and returns goal after x iterations and
# other times samples and returns nextPoint at distance d from currPoint
def pointSampler(goalSampleCtr, currPoint, goal, d):
    # goalSampleCtr is x, sampling goal 
    if goalSampleCtr==15:
        # 
        point = Point(goal.x, goal.y)
        print('using goal')
    # sampling random point
    else:
        # taking random point
        point = Point(random.uniform(0, 10), random.uniform(0, 10))
        # updating point as a point on distance d in same direction
        point = pointonVectoratD(currPoint, point, d)
    return point
'''
                    STATUS - WORKING
'''
# function to return distance between 2 points
def distanceBetween(point1,point2):
     dist = math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)
     return dist
'''
                    STATUS - PENDING
'''
'''
    PROCEDURE:
    1. we have the newPoint, we have the starting point, currNode and obsList
    2. let leastDis = distance between currNode.position and newPoint
    3. let tempNode = currNode
    4. while tempNode.position!=start
    5. distance should be less than leastDist and isLineOkay = True
    6. if follows, then store as closestNode, let new leastDist be this, keep updating
    7. at end return this
'''
# function to return node at least distance from point in tree
# traversing from start till position is null
def minDistNode(newPoint, start, currNode, obsList):
    # closestNode to return, assuming currNode as closestNode
    closestNode = currNode
    # let tempNode = currNode, will be for while loop
    tempNode = currNode
    # let least distance be the distance between the currNode and newPoint
    leastDist = distanceBetween(currNode.position, newPoint)
    # while we havent reached the root
    while tempNode.parent!=None:
        # line joining newPoint and tempNode
        line = LineString([(tempNode.position), (newPoint)])
        # if line doesnt cross any obstacle
        if isLineOkay(line, obsList):
            # if distance btwn newPoint and tempNode.position is lesser than leastNode
            if distanceBetween(tempNode.position, newPoint)<leastDist:
                # updating leastDist
                leastDist = distanceBetween(tempNode.position, newPoint)
                # updating closestNode
                closestNode = tempNode
                print('closestNode: ', closestNode.position, closestNode.parent)
        # traversing towards the root
        tempNode = tempNode.parent
    # returning closestNode to newPoint
    return closestNode
'''
                    STATUS - PENDING
'''
# function to traverse back to root of tree, plot and return path
def visualize(node, start, path):
    # to store the final path
    path = []
    path.append(start)
    # while we havent reached the root of tree
    while node.parent!=None:
        # adding point to path
        path.append(node.position)
        # plot line segments joining node and its Parent
        plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='green')
        # print(node.parent, cdnode.position)
        # decrementing node, moving towards root
        node = node.parent
    return path
'''
                    STATUS - PENDING
'''
# function to implement rrt algorithm
def rrt(n, start, goal, d, obsList):
	# plotting Start and Goal
    plt.scatter(start.x, start.y, marker='x', color='yellow')
    plt.scatter(goal.x, goal.y, marker='x', color='green')
    # adding first node to the tree, currNode
    # parent of first node will be None
    currNode = Node(start)
    # currNode is the current point in the tree
    currPoint = start
    # keep track of number of nodes placed
    nodeCtr=0
    # counter for number of iterations
    itr=0
    # counter for goal sampling (resets after 15)
    goalSampleCtr = 0
    '''
        PROCEDURE:
        1. update goalSampleCtr, itr
        2. get a newPoint from pointSampler()
        3. check if it lies in obstacle
        4. check if line crosses any obstacle
        5. find closest node in tree to newPoint 
        6. get position of closestNode, plot line and point
        7. add point to tree, update variables
    '''
    # while n nodes havent been placed
    while nodeCtr<n:
        # incrementing iterations and goalSampleCtr
        (itr, goalSampleCtr) = (itr+1, goalSampleCtr+1)
        # function returns either the goal or a random point at distance d
        # nextPoint updates to newPoint if newPoint follows conditions
        newPoint = pointSampler(goalSampleCtr, currPoint, goal, d)
        # if point lies inside obstacle continue
        if not isPointOkay(newPoint, obsList):
            continue
        # line joining the currPoint to the newPoint
        line = LineString([(currPoint), (newPoint)])
        # if line crosses an obstacle
        if not isLineOkay(line, obsList):
            continue
        # getting closestNode to newPoint in tree
        closestNode = minDistNode(newPoint, start, currNode, obsList)
        # line connecting currPoint to newPoint
        line = LineString([(currPoint), (newPoint)]) # not needed?
        # plot line segment
        plt.plot([currPoint.x, newPoint.x], [currPoint.y, newPoint.y], color='red', linewidth=2)
        # plot newPoint
        plt.scatter(newPoint.x, newPoint.y, s=1.1, marker='*', color = 'black')
        # adding newPoint as newNode to tree
        newNode = Node(newPoint)
        # setting closestNode as newNodes parent
        newNode.parent = closestNode
        currNode = newNode
        # updating currNode as newNode
        # currNode = newNode # do i need this?
        # updating currPoint as newPoint
        currPoint = newPoint
        # updating number of nodes
        nodeCtr+=1
        # goal check
        if newPoint==goal:
            print(f'Goal reached!\nIterations: {itr},\nNodes: {nodeCtr}\n')
            print('Traversing Path.')
            path = visualize(currNode, start)
            return path
    # if goal isnt reached and while loop terminates normally
    print(f'\nCouldnt Reach Goal in iterations: {itr}, nodes: {nodeCtr}')
'''
                    WORKING
'''
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
        # plotting obstacles
        plt.plot(*Polygon(obs).exterior.xy)
        obsList.append(Polygon(obs));

    # start point and goal point
    start = Point(1, 1)
    goal = Point(10, 10)

    n = 2000 # define number of nodes
    D = 1 # define max distance between parent and child
    # n = int(input('number of Nodes: '))
    # D = float(input(' max distance between child and parent: '))
    rrt(n, start, goal, D, obsList)
    plt.show()

# calling main function
test_rrt()
