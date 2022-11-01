
from vpython import *
from time import *
import numpy as np
import math
import serial
import csv
import datetime

# ad = serial.Serial('/dev/cu.usbmodem1301',115200)
sleep(1)

display_x = 1920.0
display_y = 1080.0-40.0

#scene 1
scene1 = canvas(align="left",title="",width=display_x/4.0,height=display_y/3.5, x=0,y=0)
scene1.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene1.forward=vector(-1,-1,-1)
scene1.select()
 
# xarrow=arrow(length=5, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
# xlabel=label(text="x", color=xarrow.color,pos=xarrow.pos+xarrow.axis )
# yarrow=arrow(length=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
# zarrow=arrow(length=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))
 
frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))
body = cylinder(length=6,radius=0.5,opacity=0.8,pos=vector(-3,0,0))
nosecone = cone(pos=vector(3,0,0),radius=0.5)
myObj=compound([body,nosecone])

roll=0
pitch=0
yaw=0
t=0

#scene 2
scene2 = canvas(align="left",title="",width=display_x/5.0,height=display_y/3.5)
scene2.select()
l = label(color=color.red,text=f"roll:{roll}\npitch:{pitch}\nyaw:{yaw}\naltitude:\ngps:\nvelocity:\nacceleration:",box=False,pos=vector(0,5,0),height=20,)

#scene 3
scene3 = canvas(align="left" , title="",width=display_x/4.0,height=display_y/3.5)
scene3.select()
scene.range = 13
yarrs = arrow(opacity=0.5,length=12,shaftwidth=0.3, axis=vector(0,1,0),pos=vector(-5,-5,0))
xarrs = arrow(length=12,shaftwidth=0.3, axis=vector(1,0,0),pos=vector(-5,-5,0))
for i in range(-4,6):
    label(text=f"{i+5}k",pos=vector(-5,i,0),box=False,xoffset=-10)
body2 = cylinder(color=color.red,length=1,radius=0.4,opacity=1,pos=vector(0,0,0),axis=vector(0,1,0))
nosecone2 = cone(color=color.red,pos=vector(0,1,0),radius=0.4,axis=vector(0,1,0))
myObj2=compound([body2,nosecone2])
myObj2.pos = vector(0,-5,0)

#Graph 1: 
graph1=graph(background=color.black,align="left",title="roll",xtitle="time",ytitle="velocity",width=display_x/4.5,height=display_y/4,fast=False)
g1 = gcurve(color=color.green,graph=graph1)
g1.plot(1,5)
g1.plot(2,6)

#Graph 2:
graph2=graph(align="left",title="pitch",xtitle="time",ytitle="acc",width=display_x/4.5,height=display_y/4,fast=False)
g2 = gcurve(color=color.green,graph=graph2)
g2.plot(1,5)
g2.plot(2,6)


#Graph 3:
graph3=graph(align="left",title="yaw",xtitle="time",ytitle="acc",width=display_x/4.5,height=display_y/4,fast=False)
g3 = gcurve(color=color.green,graph=graph3)
g3.plot(1,5)
g3.plot(2,6)




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
    # roll_curve.plot(time,yaw)  
    g1.plot(t,roll)
    g2.plot(t,pitch)
    g3.plot(t,yaw)
    t+=1

def updateText():
    global l
    l.text = f"roll:{roll}\npitch:{pitch}\nyaw:{yaw}"

def addCSV(data):
    print("c")
    data.insert(0,datetime.now().time().strftime("%H:%M:%S")) #for timestamp

    print(data)
    with open("FlightData.csv", "a") as f: #csv
        writer = csv.writer(f, delimiter=",")
        writer.writerow(data)


while (True):
    # pass
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
    
            roll=-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
            pitch=math.asin(2*(q0*q2-q3*q1))
            yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2

            rate(100)

            updateSim()

            #TODO: plot graphs in drawGraph() 
            drawGraph()
            print("a")
            #update raw data text
            updateText()
            print("b")
            
            data = ["roll","pitch","yaw"]
            print(data)
            #TODO: send data for csv file writing
            addCSV(data)

     
    except:
        pass