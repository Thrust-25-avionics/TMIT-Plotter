
from vpython import *
from time import *
import numpy as np
import math
import serial
import csv
import atexit
import datetime

#change com port
ad = serial.Serial('/dev/cu.usbmodem1301',115200)
sleep(1)

display_x = 1350
display_y = 1080.0-40.0

#defining global variables for all parameters
roll=0
pitch=0
yaw=0
alt=0
press=0
vel=0
t=0


#Graph 1: altitude 
graph1=graph(align="left",title="altitude",xtitle="time",ytitle="altitude",width=display_x/3,height=display_y/4,fast=False)
g1 = gcurve(color=color.green,graph=graph1)
g1.plot(1,5)
g1.plot(2,6)

#Graph 2:velocity
graph2=graph(align="left",title="velocity",xtitle="time",ytitle="velocity",width=display_x/3,height=display_y/4,fast=False)
g2 = gcurve(color=color.green,graph=graph2)
g2.plot(1,5)
g2.plot(2,6)



#scene 1:sim
scene1 = canvas(align="right",title="",width=display_x/3.0,height=display_y/3.5)
scene1.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene1.forward=vector(-1,-1,-1)
scene1.select()
frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))
body = cylinder(length=6,radius=0.5,opacity=0.8,pos=vector(-3,0,0))
nosecone = cone(pos=vector(3,0,0),radius=0.5)
myObj=compound([body,nosecone])


#Graph 3:pressure
graph3=graph(align="left",title="pressure",xtitle="time",ytitle="pressure",width=display_x/3,height=display_y/4,fast=False)
g3 = gcurve(color=color.green,graph=graph3)
g3.plot(1,5)
g3.plot(2,6)

#Graph 4: gyro
graph4=graph(align="left",title="gyro",xtitle="time",ytitle="gyro",width=display_x/3,height=display_y/4,fast=False)
roll_curve = gcurve(color=color.green,graph=graph4)
pitch_curve = gcurve(color=color.green,graph=graph4)
yaw_curve = gcurve(color=color.green,graph=graph4)

roll_curve.plot(1,5)
pitch_curve.plot(2,6)


#scene 2: raw data
scene2 = canvas(align="right",title="",width=display_x/3.0,height=display_y/3.5)
scene2.select()
l = label(color=color.red,text=f"roll:{roll}\npitch:{pitch}\nyaw:{yaw}\naltitude:\ngps:\nvelocity:\nacceleration:",box=False,pos=vector(0,5,0),height=20,)

#to track whether csv is start or stop
isStart = False

def switchCSV():
    global isStart
    isStart = not isStart
    if(isStart):
        vb.text = "stop csv"
    else:
        vb.text = "start csv"
vb=button(bind=switchCSV,text="start csv")


def updateSim():

    k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
    y=vector(0,1,0)
    s=cross(k,y)
    v=cross(s,k)
    vrot=v*cos(roll)+cross(k,v)*sin(roll)
    
    frontArrow.axis=k
    sideArrow.axis=cross(k,vrot)
    upArrow.axis=vrot
    myObj.axis=k
    myObj.up=vrot
    sideArrow.length=2
    frontArrow.length=4
    upArrow.length=1


def drawGraph():  
    global t 
    roll_curve.plot(t,roll)
    pitch_curve.plot(t,pitch)
    yaw_curve.plot(t,yaw)
    t+=1

def updateText():
    global l
    l.text = f"roll:{roll}\npitch:{pitch}\nyaw:{yaw}"

def addCSV(data):
    # data.insert(0,datetime.now().time().strftime("%H:%M:%S")) #for timestamp
    with open("FlightData.csv", "a") as f: #csv
        writer = csv.writer(f, delimiter=",")
        writer.writerow(data)

def reset():
    #reset the animation
    #reset the datalogging
    pass

#functions to be executed at kill
def exit_func():
    #TODO:add functions to be executed at stop
    print("function stoppped")
atexit.register(exit_func)

while (True):
    rate(100)
    try:
        while (ad.inWaiting()==0):
            pass
        dataPacket=ad.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        if(len(splitPacket)==5):
            q0=float(splitPacket[1])
            q1=float(splitPacket[2])
            q2=float(splitPacket[3])
            q3=float(splitPacket[4])
            # alt=
            # press=
            # vel=

            roll=-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
            pitch=math.asin(2*(q0*q2-q3*q1))
            yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
            
            data=[alt,press,vel,roll,pitch,yaw]


            updateSim()

            #TODO: plot graphs in drawGraph() 
            drawGraph()
            #update raw data text
            updateText()
            #write csv
            addCSV(data)
            
            

     
    except:
        # print("exception")
        pass