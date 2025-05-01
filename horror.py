#imports
import math
import matplotlib.pyplot as plt
import numpy as np

"""
Cases go
1: - - -
2: - + -
3: + - +
4: + + +
cofa = cofc, so cofc is dead
"""

def boo(a, b, c, d, RtoFind):
    cofa = 0
    cofb = 0
    match RtoFind: #default is a minus sign so - = + and + = - easy right?
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
        case default:
            print("ERROR: PLEASE ENTER A VALID R SCENARIO :)")
            exit(-1) #err

    bCubedabc = (2*(b**3)) - (9*a*b*c) + (27*(c**2)) + (27*(a**2)*d) - (72*b*d)

    repeatedNum =  bCubedabc + (((-4*(((b**2)-3*a*c+12*d)**3)) + ((bCubedabc) ** 2)) ** (1/2)) #This sequence is repeated in the formula so I am doing those calculations here
        
    #cool we should have all that we need to do this
    #also making the square roots into fractional exponents for ease of change (especially with the 1/3 exponents)
    return (-a/4) - cofa * (1/2) *  (((a**2)/4 - (2*b)/3 + ((2**(1/3)) * (b**2 - 3*a*c + 12*d))/(3 * (repeatedNum**(1/3))) + (((repeatedNum)/(54))**(1/3)))**(1/2)) - cofb * 1/2 * ((((a**2)/2) - ((4*b)/3) - (((2**(1/3))*((b**2)-3*a*c+12*d))/(3*((repeatedNum)**(1/3)))) - (((repeatedNum)/(54))**(1/3)) - cofa * ((-a**3 + 4*a*b - 8*c) / (4*((( (a**2/4) - ((2*b)/3) + (((2**(1/3)) * ((b**2) - (3*a*c) + (12*d))) / (3*((repeatedNum)**(1/3)))) + (((repeatedNum) / (54))**(1/3))))**(1/2)))))**(1/2)) #the hard part

nS = 1
rS = 0.1
wS = 135*9.81
cS = 0.37 #not the same for boo function
bS = 0.05 #not the same for boo function
kS = 40000000
rCase = 1 #rCase is the case that needs to be found. it should be an int between 1 & 4
listOfRoots = {}

def getSlippagePoint(xValue):
    a = -2 * rS
    b = 0
    c = 0
    d = (((xValue)**2) / (64*(kS**2)*(bS**2)*(cS**2)))
    try:
        temp = boo(a, b, c, d, 3) #this is the scary part
        return temp
    except:
        print("ERROR: 3 didn't work :(")

#sinkage as Y with respect to Weight (given one is an estimate)
xSLower = 20*9.81
xSUpper = 150*9.81
ySLower = 0
ySUpper = 5
slipX = np.arange(xSLower, xSUpper, 5)
slipY = getSlippagePoint(slipX)

#rolling resistance is graphed 
xRLower = 20*9.81
xRUpper = 150*9.81
yRLower = 0
yRUpper = 150

muR = 0.03

def resistance(weight):
    a = -2 * rS
    b = 0
    c = 0
    d = (((weight)**2) / (64*(kS**2)*(bS**2)*(cS**2)))
    temp = 0
    try:
        temp = boo(a, b, c, d, 3) #this is the scary part
    except:
        print("ERROR: 3 didn't work :(")
    return (kS * bS * ((temp**(nS + 1))/(nS + 1)) + muR * (weight/4))

resisX = np.arange(xRLower, xRUpper, 5)
resisY = resistance(resisX)

#functions to plot with

#graph wheel resistance
fige, axe = plt.subplots()
axe.plot(resisX, resisY)
axe.set(xlabel="Robot Weight (N)", ylabel="Wheel Resistance (N)", title="Robot Wheel Resistance with Respect to Robot Weight")
axe.grid()
fige.savefig("resistance.png")


#graph wheel sinkage (I start calling it wheel slippage internally at some point in writing this script)
fig, ax = plt.subplots()
ax.plot(slipX, slipY)
ax.set(xlabel="Robot Weight (N)", ylabel="Sinkage (m)", title="Robot Sinkage with Respect to Robot Weight")
ax.grid()
fig.savefig("slip.png")

plt.show()
plt.show()#here just in case

#sources of information for script
# https://matplotlib.org/stable/gallery/lines_bars_and_markers/simple_plot.html#sphx-glr-gallery-lines-bars-and-markers-simple-plot-py