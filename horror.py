#imports
import math
import matplotlib.pyplot as plt
import numpy as np

import tkinter as tk
from tkinter import ttk, font
#simpy to use for the simulation part of the equations
#W is the only changing var the rest are provided by user


"""
Cases go
1: - - -
2: - + -
3: + - +
4: + + +
cofa = cofc so cofc is dead
"""

def boo(a, b, c, d, RtoFind):
    cofa = 0
    cofb = 0
    match RtoFind: #default is a minus sign
        case 1:
            cofa = 1
            cofb = 1
        case 2:
            cofa = 1
            cofb = -1
        case 3:
            cofa = -1
            cofb = 1
        case 4:
            cofa = -1
            cofb = -1

    return () - cofa * (1/2) * 0 #TODO the actual hard part

#variables to function with
xSLower = -10
xSUpper = 10
ySLower = -10
ySUpper = 10

nS = 1
rS = 0
wS = 0
cS = 0
bS = 0
kS = 0


#a-d for the horror function.
aZS = 0
bZS = 0
cZS = 0
dZS = 0

rCase = 1 #rCase is the case that needs to be found. it should be an int between 1 & 4

zS = boo() #this is the scary part


sinkageX = np.arrage(xSLower, xSUpper, ySLower, ySUpper)
sinkageY = 

xRLower = -10
xRUpper = 10
yRLower = -10
yRUpper = 10

kR = 0
bR = 0
muR = 0
wR = wS #making this one the same because I doubt the robots weight is gonna change between these two
zR = 0 #should be found in the above function
nR=1 #might need to change

resisX = numpy.arrage(xRLower, xRUpper, yRLower, yRUpper)
resisY = kR * bR * (zR**(nR + 1))/(nR + 1) * muR * (wR)/(4)


#functions to plot with

fig, ax = plt.subplots()
ax.plot()


#sources for script

# https://matplotlib.org/stable/gallery/lines_bars_and_markers/simple_plot.html#sphx-glr-gallery-lines-bars-and-markers-simple-plot-py