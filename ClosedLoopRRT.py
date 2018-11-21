
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent


def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def checkgoalreached(p1, p2, radius):
    distance = dist(p1,p2)
    if (distance <= radius):
        return True
    return False
