from __future__ import print_function
from cmath import cos

import time
import math
import numpy as np
from sr.robot import *

R = Robot()
a_th = 2.0
d_th = 0.4
global count
count=0
global i
i=0
silver = True
codeS_set=[0,0,0,0,0,0]
codeS_time=[0,0,0,0,0,0]
codeG_set=[0,0,0,0,0,0]
codeG_time=[0,0,0,0,0,0]
#codeS_set=np.array([0,0])
#codeG_set=np.array([0,0])
def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER:
            dist=token.dist
            rot_y=token.rot_y
            codeS=token.info.code
            t_silver=token.timestamp
    if dist==100:
       return -1, -1, 0, 0
    else:
       return dist, rot_y, codeS, t_silver

def find_golden_token():

    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            dist=token.dist
            rot_y=token.rot_y
            codeG=token.info.code
            t_golden=token.timestamp
    if dist==100:
        return -1, -1, 0, 0
    else:
        return dist, rot_y, codeG, t_golden
def move_to_silver_token(dist,rot_y,codeS,silver,pos,t_reach):
    #xs = np.where(codeS_set == codeS)
    #xs=codeS_set.index(codeS)
    #print(xs)
    if codeS not in codeS_set: 
    #if codeS_set[pos]!=codeS:
    #if xs==False:
        if dist <d_th: # if we are close to the token, we try grab it.
           #print("Found it!")
            #print(1)
            if R.grab(): # if we grab the token, we move the robot forward and on the right, we release the token, and we go back to the initial position
            #    print("Gotcha!")
                codeS_set[pos]=codeS
                codeS_time[pos]=t_reach
                #np.insert(codeS_set, codeS)
                print(codeS_time)
                pos=pos+1
                turn(30,1)
            #codeS_set.append(codeS)
            #print(codeS_set)
                silver = not silver # modify the silver value, to do the next task: release the token
           #else:
            #    print("Aww, I'm not close enough.")
        if -a_th<= rot_y <= a_th: # if the robot is well aligned with the token, we go forward
            #print("Ah, that'll do.")
            drive(40, 0.5)
        elif rot_y < -a_th: # if the robot is not well aligned with the token, we move it on the left or on the right
            #print("Left a bit...")
            turn(-2, 0.5)
        elif rot_y > a_th:
            #print("Right a bit...")
            turn(+2, 0.5)
        return pos, silver  
    #if codeS_set[pos]==codeS: 
    else:    
        turn(1, 1)
        #print(codeS_set)
        return pos, silver
    
        

def move_to_golden_token(dist,rot_y,codeG,silver,st):
    #count=0
    #dist, rot_y, codeG = find_golden_token()  #adquire the distance and rotation of the silver token detected
    #print(codeG)
        #print(dist)
    #xg = np.where(codeG_set == codeG)
    if codeG not in codeG_set: 
    #if codeG_set[st]!=codeG:    
        if dist <d_th*1.5: # the distance to release the token should be 1.5 greater than the distance to pick the token, to avoid push the tokens
           #print("Found it!")
            if R.release(): # if the token is released, move away from the token to look for the next silver token
            #    print("release token")
                codeG_set[st]=codeG
                #np.insert(codeG_set, codeG)
                #print(codeG_set)
                st=st+1
                drive(-20,2)
                turn(-50, 1)
            #codeG_set.append(codeG)
            #print(codeG_set)
        #    count=count+1# A counter is required to do the task 6 times
                silver = not silver #To go back to search the silver token, the silver value change again to the True state
           #else:
            # print("Aww, I'm not close enough.")
        if -a_th<= rot_y <= a_th: # if the robot is well aligned with the token, we go forward
            #print("Ah, that'll do.")
            drive(40, 0.5)
        elif rot_y < -a_th: # if the robot is not well aligned with the token, we move it on the left or on the right
            #print("Left a bit...")
            turn(-2, 0.5)
        elif rot_y > a_th:
            #print("Right a bit...")
            turn(+2, 0.5)
        if st==6:
            exit()
    #if count==6:#if the counter is equal to 6, it have to stop the simulation and exit the program
    #        exit()   
        return st, silver
    else:
    #if codeG_set[st]==codeG:    
        turn(-4, 1)
        #print(codeS_set)
        return st, silver

i=0
j=0
turn(-2,1)
drive(20,1)
while 1:
    if silver == True: # if silver is True, look for the silver token
        dist, rot_y, codeS, T_silver = find_silver_token() #adquire the distance and rotation of the silver token detected
        #print(dist,rot_y,codeS)
        i, silver=move_to_silver_token(dist,rot_y,codeS,silver,i,T_silver)
        #print(len(codeS_set))       
    #codeS_set[i]=codeS
    #print(codeS_set)
    #i=i+1
    if silver == False: # if silver is False, look for the golden token to release
        dist, rot_y, codeG, T_golden = find_golden_token()  #adquire the distance and rotation of the silver token detected
        #print("golden token:",codeG)
        j, silver=move_to_golden_token(dist,rot_y,codeG,silver,j)
        #print(dist)   
