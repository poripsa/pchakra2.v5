#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## This program generates an optimum path from start to goal
## It uses motion planning algorithm RRT* to compute the optimum path
##
## Author : Poripsa Chakrabarty
##

import pygame, random, math
from pygame.locals import *
from math import sqrt, cos, sin, atan2, tan

xdimension = 640
ydimension = 480
WINSIZE = [xdimension, ydimension]  #This is the dimension of the pygame screen which displayes the edges
eps = 20.0 #This is the step size contraint for new node (a new node at a distance of eps gets generated if the random
num = 2000
xnear = []
xinit = (320, 240)

#The function below computes the distance between two nodes given as an input to it using the formulae:
# distance = square-root (x^2 + y^2)
def distance(dnode, drand):
    x1 = dnode[0]
    y1 = dnode[1]
    x2 = drand[0]
    y2 = drand[1]
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

#The function below computes the radius of the circle which determines the nearness of nodes
#whose origin is taken as input which is the node whose neighboring nodes is to be determined
def radius(nnode, len):
    D = 2  # Dimention of C-space
    freeSpaceArea = xdimension * ydimension
    radius = 2 * sqrt(1 + (1 / D)) * ((freeSpaceArea / 3.14) ** (1 / D)) * (((math.log(len)) / len) ** (1 / D)) #Given formulae
    return radius


#This function compares the distance between the random node and its nearest neighbor
#It returns the random node if its under distance of epsilon
#If not then its return a node which is in the direction of the random node but at a distance of epsilon
def steer(nnearest, nrand):
    if distance(nnearest, nrand) < eps:
        return nrand
    else:
        x = nrand[0] - nnearest[0]
        y = nrand[1] - nnearest[1]
        theta = atan2(y, x)
        stepped = [(nnearest[0] + eps * cos(theta), nnearest[1] + eps * sin(theta))]  # This propagates node towards epsilon in the direction of nrand
        #print stepped, "this is stepped"
        return stepped[0]
        """if Collision(stepped[0], obs) == 0:
            return stepped[0]
            elif Collision(stepped[0], obs) == 1:
            #while Collision(stepped[0], obs) == 1:
            print "in looking"
            #stepped = CollisionAvoidance(stepped)
            return 1 #the code following this will not be executed
            print stepped, "collision avoided"
            return stepped[0]"""


        #print stepped, "this is new stepped"
            #return stepped[0]

def CollisionAvoidance(pointINobs):
    x = pointINobs[0][0]
    y = pointINobs[0][1]
    x0 = obs[0]
    x1 = obs[0]+obs[2]/2
    x2 = obs[2]
    y0 = obs[1]
    y1 = obs[1]+obs[3]/2
    y2 = obs[3]
    if x0 <= x <= x1 or y0 <= y <= y1:
        x = obs[0]
        y = obs[1]
        nodeOutsideObs = [(x,y)]
        return nodeOutsideObs
    if x1 <= x <= x2 or y0 <= y <= y1:
        x = obs[2]
        y = obs[1]
        nodeOutsideObs = [(x, y)]
        return nodeOutsideObs
    if x1 <= x <= x2 or y1 <= y <= y2:
        x = obs[2]
        y = obs[3]
        nodeOutsideObs = [(x, y)]
        return nodeOutsideObs
    if x0 <= x <= x1 or y1 <= y <= y2:
        x = obs[0]
        y = obs[3]
        nodeOutsideObs = [(x, y)]
        return nodeOutsideObs
    return [(300, 300)]



#This function returns a list of nodes near any given node(passed as parameter newn)
#Nodes which fall under the radius(computed by radius function) are returned
#Center of circle is at the passed parameter input newn
"""In the following function computed radius is too small to return neighbors hence an arbitary number is used"""
def NearNodes(node, newn):
    xnear = []
    length = len(node)
    for n in node:
        if distance(n, newn) < 25: #radius(n, length):
            xnear.append(n)
            #pygame.draw.line(screen, pink, [n[0], n[1]], [newn[0], newn[1]])
    return xnear #Returns a list of nodes with distance 25 of input parameter newn

#The following function computes the distance(cost) of the path to a node from initial point
def cost(node0, singlenode):
    cost = 0
    while singlenode[0] != node0[0] and singlenode[1] != node0[0]: #Continue iteration till initial node is reached
        cost = cost + distance(singlenode, parentDict[singlenode]) #calculates the distance between child node and parent node
        singlenode = parentDict[singlenode] #parent node is now child node
    return cost

#This function draws an obstacle using pygame
def obstacle():
    obx = 300
    oby = 300
    obl = 50
    obh = 20
    #pygame.draw.rect(screen, blue, (obx,oby,obl,obh))
    return (obx, oby, obl, obh)

#This function checks for collision, if x coordinate of any node lies within the x coordinate of the obstacle
#or y coordinates lies within y then 1 is returned
# otherwise 0 is returned
def Collision(node, obs):
    #for n in node:
    #print obs[0], obs[1], obs[2], obs[3], node
    if (obs[0] < node[0] < (obs[0] + obs[2])) and (obs[1] < node[1] < (obs[1] + obs[3])):
        return 1
    return 0
    #return 0

#This function draws the final path (in pink) between the start node and the goal node
#It first finds the nearest node to the goal and then parses through its parents to the start goal
def drawMotionPath(goal, nodes, pygame, screen):
    nn = nodes[0]
    i = 0
    for p in nodes:
        if distance([p[0], p[1]], [goal[0], goal[1]]) < distance([nn[0], nn[1]], [goal[0], goal[1]]): #finds the nearest node
            nn = p
    #print nn[0], nodes[0][0], "printed"
    while nn[0] != nodes[0][0] and nn[1] != nodes [0][1]:
        parent = parentDict[nn]
        pygame.draw.line(screen, pink, [nn[0], nn[1]], [parent[0], parent[1]], 5)
        nn = parent
        i = i +1
        if i == num:
            return

    pygame.display.update()


if __name__ == '__main__':
    #xinit = [(320, 240)]
    pygame.init() #initialises screen
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('Poripsa RRT*')
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)
    darkBlue = (0, 0, 128)
    white = (255, 255, 255)
    black = (0, 0, 0)
    pink = (255, 200, 200)
    screen.fill(white) #Gives background as white of the screen
    #pygame.draw.rect(screen, blue, (300,300,50,20))
    pygame.display.update()

    node = [(xdimension/2, ydimension/2)] #Initialises root node variable
    # xparent = [(320,240)]
    xrand = [(0, 0)] #Initialises random node variable
    #d = []
    xnearest = [(0, 0)] #Initialises nearest node variable
    parentDict = {(node[0]): (node[0])} #Initialises dictionary

    #parentDict = {(320, 240): (320, 240)} #Initialises dictionary
    #ndist = []
    edge = []
    obs = obstacle()
    goal = (400, 400)
    pygame.draw.circle(screen, red, [int(goal[0]), int(goal[1])], 4, 0) #Draws a circle at goal
    pygame.draw.circle(screen, blue, [int(node[0][0]), int(node[0][1])], 3, 0) #Draws a circle at start

    for i in range(num): #Loops through desired number of iterations set in num
        xrand[0] = (random.random() * xdimension, random.random() * ydimension)
        #if Collision(xrand[0], obs) == 0:
        #   xrand[0] = xrand[0]
        if Collision(xrand[0], obs) == 1:
            #print "collision"
            while Collision(xrand[0], obs) == 1:
                xrand[0] = (random.random() * xdimension, random.random() * ydimension)

        #if (obs[0] < xrand[0][0] < (obs[0] + obs[2])) or (obs[1] < xrand[0][1] < (obs[1] + obs[3])):
            #print xrand, "rand node in obs", i

        cnode = node[0]
        for a in node:
            if distance(a, xrand[0]) < distance(cnode, xrand[0]): #finds the min distance to another node from xrand and hence finds its nearest node
                cnode = a
                xnearest = a
            xnearest = cnode

        #if steer(xnearest, xrand[0]) == 1:
            #print "looped to next iteration"
            #continue

        newn = steer(xnearest, xrand[0]) #Call function to find new node at distance epsilon
        #print newn, xnearest, xrand[0], "this is newn"
        pygame.draw.circle(screen, green, [int(newn[0]), int(newn[1])], 1, 0)  # Draws a circle(black dot) at new node

        #newn = xrand[0]
        xmin = xnearest
        cmin = cost(node[0], xnearest) + distance(xnearest, newn) #finds the cost of the newnode to root node via xnearest

        xnear = NearNodes(node, newn) #call the function NearNodes which finds the nearest node to the new node
        #print xnear, "near node exists"

        #the following loop parses through all near nodes to new node and checkes whether distance to it from root is minimized if travelled via near nodes
        for xn in xnear:
            # if noCollision
            if cost(node[0], xn) + distance(xn, newn) < cmin: #Checks if cost of near nodes is less than cost via nearest node
                xmin = xn
                pygame.display.update()
                cmin = cost(node[0], xn) + distance(xn, newn)
                #print xn, xmin, cmin, "xn xmin cmin"

        edge.append((xmin, newn))  # appends the node to an edge list

        pygame.draw.line(screen, black, [xmin[0], xmin[1]], [newn[0], newn[1]])
        parentDict[newn] = xmin #Changes the parent if cost from root node via near node to new node is cheaper

        #the following for loop checks to see if the cost of traversing to near nodes is minimised by path from new node
        for pn in xnear:  #takes all near nodes
            if cost(node[0], newn) + distance(newn, pn) < cost(node[0], pn): #checks if new cost is less that original cost
                xparent = parentDict[pn]
                #print "in here"
                parentDict[pn] = newn #if its less append dictionary
                #edge.remove((xparent, pn))
                pygame.draw.line(screen, white, [xparent[0], xparent[1]], [pn[0], pn[1]])
                edge.append((newn, pn))
                pygame.draw.line(screen, black, [newn[0], newn[1]], [pn[0], pn[1]])
        pygame.display.update()

        node.append((newn[0], newn[1]))
        # when we have the new node, all the nodes within the feasable radius will be considered for connection
        # the connection will be made only when the cost from root node(0,0) to the newnode is minimum i.e
        # the near node chosen will be one through which cost from root to newnode is minimum

        pygame.display.update()


    drawMotionPath(goal, node, pygame, screen) #Calls function to draw the final path
    #print goal
    """for g in node:
        if g == goal:
            finalpath = 1
        finalpath = 0

    if finalpath != 1:
        pygame.display.set_caption('Goal Could not be reached')
    elif finalpath == 1:
        pygame.display.set_caption('Goal reached!')"""
    # print edge

    pygame.display.update()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
