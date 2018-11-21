#Artificial Intelligence Package
#Real-time Motion Planning with Applications to Autonomous Urban Driving using crrt* algorithm
import sys, random, math, pygame
from pygame.locals import *
from math import*
from decimal import Decimal
from math import sqrt,cos,sin,atan2
from ClosedLoopRRT import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d
pygame.init()
fps = pygame.time.Clock()

eps = 8.0
tnode = 4000
rad = 5
delta = 5.0
level = 2

white = 255, 255, 255
black = 0,0,0
red = 254, 160, 30
blue = 0,0,255
green = 0,255,0
cyan = 40,220,250
pvalue=5

def nroot(value, nroot):
    rootvalue = 1/float(nroot)
    return round (Decimal(value) ** Decimal(rootvalue),3)

def dist1(x,y):
    return nroot(sum(pow(abs(a-b),pvalue) for a,b in zip(x, y)),pvalue)

def voronoiRegion():#calculated node forms voronoi region
    vor = Voronoi(p1)
    voronoi_plot_2d(vor)
    for region in vor.regions:
        if not -1 in region:
            sample = [vor.vertices[i] for i in region]
            plt.fill(*zip(*sample))
    plt.show()
#check collision and clears that node in random
#flag=0
xwin = 800
ywin = 600
win = [xwin, ywin]
count = 0
envobs = []
poly=[]
p1=[]
screen = pygame.display.set_mode(win)
pygame.display.set_caption('Closed loop Rapidly Exploring Random Tree')
def checkFesibility(p):
    for obs in envobs:
        if obs.collidepoint(p) == True:
            return True
    #        flag=1
    #for r in poly:
     #   if poly.collidepoint(p)==True:
      #      flag=1
  #  if flag==1:
   #     return True
    return False

#to get random sample
def randomSample():
    return random.random()*xwin, random.random()*ywin
#obstacles in environment
def setEnvConstr():
    global envobs
    global poly
    envobs = []
    poly=[]
    print("Environment constraints Specified");
    envobs.append(pygame.Rect((78,50),(100,100)))
    envobs.append(pygame.Rect((450,200),(490,240)))
    envobs.append(pygame.Rect((530,530),(700,700)))
    envobs.append(pygame.Rect((220,250),(240,300)))
    for obs in envobs:
        pygame.draw.rect(screen, red, obs)
        #pygame.draw.polygon(screen,green,((25,75),(76,125),(250,375),(400,25),(60,540)))
        #pygame.draw.circle(screen,red,(150,150),75)
        poly.append((25,75))
        poly.append((76,125))
        poly.append((250,375))
        poly.append((400,25))
        poly.append((60,540))

def nearestNeighbor(p1,p2):
    if dist(p1,p2) < eps:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])#calculate next trajectory path
        return p1[0] + eps*cos(theta), p1[1] + eps*sin(theta)

def reset():
    global count
    screen.fill(black)
    setEnvConstr()#Update the current vehicle states x0 and environmental constraints Xfree(t)
    count = 0
def removeInfeasible():
    while True:
        p = randomSample()
        col = checkFesibility(p)
        if col == False:
            return p
def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


def CLRRT():
    global count
    global p1
    p1=[]
    initialstate = False#initial node 
    initialPoint = Node(None, None)
    goalstate = False#goal node
    goalnode = Node(None, None)
    position = 'init'

    nodes = []#Take a sample s for input to controller.
    reset()
    #1: repeat
    while True:
        if position == 'init':
            fps.tick(10)
        elif position == 'goalFound':#13: Trace the best reference path r to controller
            cnode = gnode.parent
            print("Total nodes explored"+str(count));
            print("PATH:")
            while cnode.parent != None:
                pygame.draw.line(screen,cyan,cnode.point,cnode.parent.point)
                cnode = cnode.parent
                p1.append(cnode.point)
                print(cnode.point)
            tracePath = True
            voronoiRegion()
        elif position == 'optimize':
            fps.tick(0.5)
            pass
        elif position == 'buildTree':#5: Expand tree()
            count = count+1
            if count < tnode:
                found = False
                while found == False:#4: repeat
                    rand = removeInfeasible()
                    # print("random num = " + str(rand))
                    pnode = nodes[0]

                    for p in nodes:  #6: until time limit t is reached
                        if dist(p.point,rand) <= dist(pnode.point,rand):#7: Choose the best safe node sequence in the tree
                            nextsample = nearestNeighbor(p.point,rand)
                            if checkFesibility(nextsample) == False:#8,9,10
                                pnode = p 
                                found = True

                newnode = nearestNeighbor(pnode.point,rand)
                nodes.append(Node(newnode, pnode))#10: Add intermediate nodes to tree and mark them unsafe.Break.
                pygame.draw.line(screen,white,pnode.point,newnode)#trace all the intermediate nodes

                if checkgoalreached(newnode, goalnode.point, rad):#17: Add the goal node to tree.
                    position = 'goalFound'
                    gnode = nodes[len(nodes)-1]
                    #vor()

                
            else:
                print("Too  much Far!!Out of range!!!hence stopped.........")
                return;
    

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                #print('mouse down')
                if position == 'init':
                    if initialstate == False:
                        nodes = []
                        if checkFesibility(e.pos) == False:
                            print('Initial State: '+str(e.pos))
                            initialPoint = Node(e.pos, None)
                            nodes.append(initialPoint) 
                            initialstate = True
                            pygame.draw.circle(screen, blue, initialPoint.point, rad)
                    elif goalstate == False:
                        print('Goal State: '+str(e.pos))
                        if checkFesibility(e.pos) == False:
                            goalnode = Node(e.pos,None)
                            goalstate = True
                            pygame.draw.circle(screen, green, goalnode.point, rad)
                            position = 'buildTree'
                else:
                    position = 'init'
                    initialstate = False
                    goalstate = False
                    reset()

        pygame.display.update()
        fps.tick(10000)

CLRRT()
#Algorithm 2 Execution loop of RRT.
#1: repeat
#2: Update the current vehicle states x0 and environmental constraints Xfree(t)
#3: Propagate the states by the computation time and obtain x(t0 + t)
#4: repeat
#5: Expand tree()
#6: until time limit t is reached
#7: Choose the best safe node sequence in the tree
#8: if No such sequence exists then
#9: Send E-Stop to controller and goto line 2
#10: end if
#11: Repropagate from the latest states x(t0+t) using ther associated with the best node sequence, and obtainx(t); t 2 [t1; t2]
#12: if x(t) 2 Xfree(t) 8t 2 [t1; t2] then
#13: Send the best reference path r to controller
#14: else
#15: Remove the infeasible portion and its children from the tree, and goto line 7
#16: end if
#17: until the vehicle reaches goal

#Algorithm 1 Expand tree()
#1: Take a sample s for input to controller.
#2: Sort the nodes in the tree using heuristics.
#3: for each node q in the tree, in the sorted order do
#4: Form a reference command to the controller, by connecting the controller input at q and the sample s.
#5: Use the reference command and propagate from q until vehicle stops. Obtain a trajectory x(t); t 2 [t1; t2].
#6: Add intermediate nodes qi on the propagated trajectory.
#7: if x(t) 2 Xfree(t); 8t 2 [t1; t2] then
#8: Add sample and intermediate nodes to tree. Break.
#9: else if all intermediate nodes are feasible then
#10: Add intermediate nodes to tree and mark them unsafe.Break.
#11: end if
#12: end for
#13: for each newly added node q do
#14: Form a reference command to the controller, by connecting the controller input at q and the goal location.
#15: Use the reference command and propagate to obtain trajectory x(t); t 2 [t3; t4].
#16: if x(t) 2 Xfree(t); 8t 2 [t3; t4] then
#17: Add the goal node to tree.
#18: Set cost of the propagated trajectory as an upper bound CUB of cost-to-go at q.
#19: end if
#20: end for
