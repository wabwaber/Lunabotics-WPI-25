import math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import rclpy
from random import randrange as randydandy
typeStore = get_typestore(Stores.ROS2_GALACTIC)

class cluster():
    def __init__(self, point: list):
        self.data = [point]
        self.thetas = []
        self.size = 1
        self.xTotal = point[1]
        self.yTotal = point[2]
        self.zTotal = point[3]
        self.xAvg = self.xTotal / self.size
        self.yAvg = self.yTotal / self.size
        self.zAvg = self.zTotal / self.size
        self.startTime = point[0]
        self.endTime = -1
    
    def add(self, toAdd: list):
        self.data.append(toAdd)
        self.size += 1
        if self.size == 2: #if it is the second data point
            firstTou = toAdd[0] - self.data[0][0] #do this calculation for the first tou
            self.thetas[0] = firstTou * self.xTotal #
        self.xTotal += toAdd[1]
        self.yTotal += toAdd[2]
        self.zTotal += toAdd[3]
        self.xAvg = self.xTotal / self.size
        self.yAvg = self.yTotal / self.size
        self.zAvg = self.zTotal / self.size
        tou = toAdd[0] - self.data[self.size-2][0]
        self.thetas[self.size-1] = [tou*self.xTotal, tou*self.yTotal, tou*self.zTotal]
    def setEnd(self, time: float):
        self.endTime = time


#TODO ENSURE THAT THE LAST DATA POINT ISN'T ALONE IN A CLUSTER IF IT IS TAKE DEFAULT TOU AND USE THAT TO GET ITS THETA
def processData(data: list):
    maxIndex = len(data) - 1 #if I had a nickel for each time I used this I would have 2,  but its weird how I have to use this twice right?
    maxM = (len(data) - 1)//2 #calculate the m maximum
    m = int(input("What is the Averaging factor you want to use (-1 for random)?(" + "1->" + str(maxM) + "): "))
    if m > maxM:
        #I hate you if you do this, (its me. I hate me)
        print("ERROR: m provided higher than the allowed maximum. Restarting...")
        processData(data) #call this function again to restart it
        exit() #exit as we shouldn't continue
    elif m == -1:
        m = randydandy(1, maxM, 1) #choose a random whole number between 1 and the maximum
    #for clarity m is the number of strides in a cluster so there are m gaps in a cluster or m+1 data points in a cluster
    timePeriod = data[maxIndex][0] -  data[0][0] #the total time period of the recording
    stride = timePeriod / m #striding right along...
    respon = input("Show data plots? (y/n): ")
    if respon == 'y':
        showData(data)
    currIndex = 0
    endOfCluster = currIndex + m + 1
    currCluster = cluster(data[currIndex])
    while currIndex <= maxIndex: #while we haven't gone through the entire list
        if currIndex > maxIndex: #if for whatever reason we go over the max index
            break #break out of the while loop

        if currIndex > endOfCluster:
            currCluster = cluster(data[currIndex])
            currIndex += 1
            continue
        
        currCluster.add(data[currIndex])

        if currIndex == endOfCluster:
            currCluster.setEnd(data[currIndex][0])
        
        currIndex += 1
    if currCluster.size == 1:
        currCluster.thetas[0] = 0 #set it to zero because its the only point so there is no variance!
    #at this point we have thetas for all clusters, wait... heck, I think I misunderstood what the paper I was reading was talking about, I did find another paper thou that is perfect for this and goes in detail for IMUs, Haiying Hou 2004 btw

    
def showData(data: list):
    times = []
    xData = []
    yData = []
    zData = []
    for p in data:
        times.append(p[0] - data[0][0])
        xData.append(p[1])
        yData.append(p[2])
        zData.append(p[3])
    plt.plot(times, xData, 'r', times, yData, 'b', times, zData, 'g')
    plt.axis(0, times[len(times)-1], 0, max(max(xData), max(yData), max(zData))+0.5)
    plt.show()
    

def makeTextFileOfBagData(name: str, path: str):
    file = open(name, "w")
    with Reader(path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = typeStore.deserialize_cdr(rawdata, connection.msgtype)
            toWrite = str(msg.header.stamp.sec + (msg.header.stamp.nanosec * (10 ** (-8)))) + ","
            toWrite += str(msg.linear_acceleration.x) + ','
            toWrite += str(msg.linear_acceleration.y) + ','
            toWrite += str(msg.linear_acceleration.z) + '\n'
            file.write(toWrite)
    file.close()
        

def main():
    print("Program starting...")
    response = input("Would you like to read through a bag?(y/n): ")
    fileName = None
    if response.lower() == 'y':
        bagPath = input("Please provide the full path to the bag: ")
        if bagPath == 'n': #default because I can
            bagPath = "./imudata2"
        fileName = input("What would you like the output file to be called (include .txt): ")
        makeTextFileOfBagData(fileName, bagPath)
        respons = input("Do you want to exit now that the file has been made or continue (y/n): ")
        if respons.lower() == 'y':
            exit() #early exit
    
    if fileName == None:
        fileName = input("Please provide the input file name (include extension): ")
    
    data = [] #list of each data point
    file = open(fileName, "r") #open file as read only
    for line in file: #for each line of the file
        newPoint = line.split(',') #split the line by commas
        data.append([float(newPoint[0]), float(newPoint[1]), float(newPoint[2]), float(newPoint[3])]) #I hate this but now its a list of floats in a list
    file.close() #close file after data has been read in

    processData(data)
