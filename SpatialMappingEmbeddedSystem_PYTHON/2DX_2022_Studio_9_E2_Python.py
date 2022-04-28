from cmath import cos, sin
from os import truncate
from turtle import color
import serial
import math
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Asks user if they want to delete the previous data that is currently stored in the text file
#If they want to delete it, (i.e. they enter in Y) it will open the file in write mode, delete everything then close the file
deletePreviousData = input("Do you want to erase the previous graph data? (Y/N)")
if(deletePreviousData == 'Y'):
    #Opens the file
    dataPlot = open('graphmeasurementsdata.txt','w')                                               
    #Erases the data
    truncate
    #Closes the file
    dataPlot.close() 

#Opening the serial port to start receiving data from the microcontroller
s = serial.Serial('COM4', baudrate = 115200, timeout = 4)
#Printing to the user what port was opened (COM4)
print("Opening: " + s.name)

#Resetting the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

#Preparing the 3D plot that will be used later
fig = plt.figure(figsize=(10,10))
PlotAxis = fig.add_subplot(111, projection='3d')
PlotAxis.set_xlabel('x')
PlotAxis.set_ylabel('y')
PlotAxis.set_zlabel('z')

#Flag to start receiving data
s.write('s'.encode())
  

count = 0
z = 0
#Creating an empty array of 64 elements that will hold the measurements of one rotation
x = [0]*64
#Open the file of measurements in append mode
dataPlot = open('graphmeasurementsdata.txt','a')
#While loop  that prints measurements on screen and stores them in a text file
while (count != 64):
    #Decode the measurement and put it in the array 
    x[count] = float(s.readline().decode())
    #Prints the measured value and the measurement number to the console
    print(x[count], count+1)
    
    #Convert the data to a string and put it in the text file
    #Since it is in append mode, it will add onto previous measurements if there are any
    #To differentiate between measured sets, this will be accounted for below
    dataPlot.write(str(x[count]))
    #Add a new line
    dataPlot.write('\n')
    #Increment count
    count+=1


#Close the text file
dataPlot.close()


#Open the text file again, this time in read mode
dataPlot = open("graphmeasurementsdata.txt","r")
#Take all the data from the text file and puth it in a string
dataStr = dataPlot.read()
#Split the data at the new lines
dataList = dataStr.split("\n")
#For loop that goes through each measurement and stores it in a list as a string
for i in range(0,len(dataList),1):                                              
    dataList[i] = str(dataList[i])
#Close the file
dataPlot.close()
#Remove the last element as it is a \n
dataList.pop(len(dataList)-1)
#Convert the data to float type
dataFloatList = [float(i) for i in dataList]


#Asks user if they want to plot the current data that is stored in the text file
#If they want to, (i.e. they enter in Y) it will 
plotCurrentData = input("Do you want to plot the data? (Y/N)")
if(plotCurrentData == 'Y'):
    

    count = 0
    #While loop that continously runs as long as we have data to graph
    #This while loop is used to plot the measured points on the plot
    while(count != len(dataFloatList)):

        #If statement to check if we are on the next set of measurements
        #If yes, then we increment z by 0.1
        if(count % 64 == 0 and count >= 64):
            z += 0.1

        #These three lines below define the points of where to plot the x,y, and z values from a defined meaurement
        Coordinate_Z = [(dataFloatList[count-1])*cos((count-1)*0.09817477),(dataFloatList[count])*cos(count*0.09817477)]
        Coordinate_Y = [(dataFloatList[count-1])*sin((count-1)*0.09817477),(dataFloatList[count])*sin(count*0.09817477)]
        Coordinate_X = [z,z] 

        #The lines below plot the data points in cartesian form
        PlotAxis.scatter( z,(dataFloatList[count])*sin(count*0.09817477),(dataFloatList[count])*cos(count*0.09817477), c = "black", s = 1)
        PlotAxis.plot(Coordinate_X ,Coordinate_Y,Coordinate_Z, color = 'black')
        
        #Increment the counter for the next iteration
        count += 1


    count = 0
    z = 0
    #While loop that continously runs as long as we have data to graph
    #This while loop is used to plot the lines connecting the points on the plot
    while(count < len(dataFloatList)-64):
        Coordinate_Z = [(dataFloatList[count])*cos((count)*0.09817477),(dataFloatList[count+64])*cos(count*0.09817477)]
        Coordinate_Y = [(dataFloatList[count])*sin((count)*0.09817477),(dataFloatList[count+64])*sin(count*0.09817477)]

        
        #If statement to check if we are on the next set of measurements
        #If yes, then we increment z by 0.1
        if(count % 64 == 0 and count >= 64):
            z += 0.1

        Coordinate_X = [z,z+0.1]
        PlotAxis.plot(Coordinate_X ,Coordinate_Y,Coordinate_Z, color = 'black')
        count += 1
    
    
    #Show the 3D graph
    plt.show()


#Printing to the user that the port is closing (COM4)
print("Closing: " + s.name )                          
#Close the port
s.close()
