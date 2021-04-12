'''
STAGE 6 
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

# function that samples and returns goal after x iterations and
# other times samples and returns nextPoint at distance d from currPoint
def pointSampler(itr, currPoint, goal, d):
    # if itr is multiple of 10 sample point within goal region (goal-10)
    if itr%10==0:
        point = Point(random.uniform(8, 10), random.uniform(8, 10))
    else:
        # if iterations are multiple of 15, sample goal 
        if itr%15==0:
            point = Point(goal.x, goal.y)
        # sampling random point
        else:
            # taking random point
            point = Point(random.uniform(0, 10), random.uniform(0, 10))
            # if distance between point is greater than d then get point at distance d
            if distanceBetween(point, currPoint)>d:
                # updating point as a point on distance d in same direction
                point = pointonVectoratD(currPoint, point, d)
    return point

# function to return distance between 2 points
def distanceBetween(point1,point2):
     dist = math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)
     return dist

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
        # traversing towards the root
        tempNode = tempNode.parent
    # plotting branch with different color
    plt.plot([closestNode.position.x, newPoint.x], [closestNode.position.y, newPoint.y], color='orange', linewidth=1.2)
    # returning closestNode to newPoint
    return closestNode

# function to traverse back to root of tree, plot and return path
def visualize(endNode, start):
    tempNode = endNode
    # to store the final path
    path = []
    # while we havent reached the root of tree
    while tempNode.parent!=None:
        # adding point to path
        path.append(tempNode.position)
        # plotting the Path
        plt.plot([tempNode.parent.position.x, tempNode.position.x], [tempNode.parent.position.y, tempNode.position.y], color='green', linewidth=2)
        # decrementing node, moving towards root
        tempNode = tempNode.parent
    return path

# function to implement rrt algorithm
def rrt(n, start, goal, d, obsList):
	# plotting Start and Goal
    plt.scatter(start.x, start.y, marker='x', color='green')
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
    
    # while n nodes havent been placed
    while nodeCtr<n:
        # to track range explored
        maxX = -1
        maxY = -1
        # incrementing iterations and goalSampleCtr
        itr+=1
        # function returns either the goal or a random point at distance d
        # nextPoint updates to newPoint if newPoint follows conditions
        newPoint = pointSampler(itr, currPoint, goal, d)
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
        plt.plot([currPoint.x, newPoint.x], [currPoint.y, newPoint.y], color='red', linewidth=0.5)
        # plot newPoint
        plt.scatter(newPoint.x, newPoint.y, s=1, color = 'black')
        if newPoint.x>maxX:
            maxX = newPoint.x
        if newPoint.y>maxY:
            maxY = newPoint.y
        # adding newPoint as newNode to tree
        newNode = Node(newPoint)
        # setting closestNode as newNodes parent
        newNode.parent = closestNode
        currNode = newNode
        # updating currNode as newNode
        currNode = newNode # do i need this?
        # updating currPoint as newPoint
        currPoint = newPoint
        # updating number of nodes
        nodeCtr+=1
        # goal check
        if newPoint==goal:
            print('\n==================================================\n')
            print(f'Goal reached!\nIterations: {itr},\nNodes: {nodeCtr}\n')
            print('\n==================================================\n')
            print('Traversing Path.')
            path = visualize(currNode, start)
            path.append(start)
            print('Path:\n')
            for i in path:
                print(f'({i.x}, {i.y})')
            print('\n==================================================\n')
            return path
    print('\n==================================================\n')
    # if goal isnt reached 
    print(f'\nCouldnt Reach Goal.\nIterations: {itr}\nnodes: {nodeCtr}\n')
    print('\n==================================================\n')
    print(f'\nRange Explored:\nx: ({0, maxX}), \ny: ({0, maxY:})\n')
    print('\n==================================================\n')

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
        plt.plot(*Polygon(obs).exterior.xy, color='black')
        obsList.append(Polygon(obs));

    # start point and goal point
    start = Point(1, 1)
    goal = Point(10, 10)

    n = 5000 # define number of nodes
    D = 1 # define max distance between parent and child
    path = rrt(n, start, goal, D, obsList)
    # print(path)
    plt.show()

# calling main function
test_rrt()
