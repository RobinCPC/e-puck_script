# Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# -------------------------------------------------------------------
# THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------

import math

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, epuck1 = vrep.simxGetObjectHandle(clientID, 'ePuck#', vrep.simx_opmode_oneshot_wait)
    
    # orientation
    res1, epuck1_base = vrep.simxGetObjectHandle(clientID, 'ePuck_base#', vrep.simx_opmode_oneshot_wait)
    
    if res != vrep.simx_return_ok and res1 != vrep.simx_return_ok:
        print 'Error: check object handlers in V-REP and here'
        vrep.simxFinish(clientID)
    else:
        time.sleep(2)
        # Initialization
        # ============= START MAIN LOOP =============
        # start epucks
        vrep.simxSetFloatSignal(clientID, 'startSignal', 0, vrep.simx_opmode_oneshot_wait)
        us = 0
        val1 = 1
        ind = 0
        # parameter for closed loop steering
        ga = 3
        h = 1
        k = 6
        xi = -1
        yi = 1
        phi = 0.75*math.pi  # degree

        while val1 > 0:
            ind += 1
            # Control Computation start -------------
            # Read position of the epuck position
            x, y, theta1 = 0, 0, 0
            res, pos1 = vrep.simxGetObjectPosition(clientID, epuck1, -1, vrep.simx_opmode_oneshot_wait)
            if res == vrep.simx_return_ok:
                x = pos1[0]  # (m)
                y = pos1[1]  # (m)
            # Read z-axis rotation of the epuck (radians)
            res, eularAngles1 = vrep.simxGetObjectOrientation(clientID, epuck1_base, -1, vrep.simx_opmode_oneshot_wait)
            if res == vrep.simx_return_ok:
                theta1 = eularAngles1[2]  # +90   # gamma=z-axis -> (degree)
            print 'x = {0:f}, y = {1:f}, theta1 = {2:f}'.format(x, y, theta1*(180/math.pi))
            # compute u (omega) and v (u in paper)
            rx = xi + x
            ry = yi + y
            rph = phi + theta1
            e = math.sqrt(rx**2+ry**2)
            th = math.atan2(-ry,-rx)
            al = th - rph

            u = k*al + (ga/al)*(math.cos(al)*math.sin(al))*(al+h*th)
            v = 1*ga*math.cos(al)*e
            RUD = 30.   # speed reducer to limit u,v below 1.0
            print 'rx= {0:f}, ry= {1:f}, rph= {2:f}'.format(rx, ry, rph)
            print 'e= {0:f}, th= {1:f}, al= {2:f}'.format(e, th, al)
            print 'u = {0:f}, v ={1:f}'.format(u/RUD, v/RUD)

            # u = 0.7
            # v = 0.1
            if ind > 1000:  # stop simulation set velocity to zero first
                val1 = 0
                u = 0
                v = 0
            # Control computation end -----------------
            us = u/RUD  # [-1]
            vs = v/RUD  # [-1]
            vrep.simxSetFloatSignal(clientID, 'matlabRef_omega', us, vrep.simx_opmode_oneshot)
            vrep.simxSetFloatSignal(clientID, 'matlabRef_v', vs, vrep.simx_opmode_oneshot)
        # stop epuck
        vrep.simxSetFloatSignal(clientID, 'matlabRef_omega', 0, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, 'matlabRef_v', 0, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, 'stopSignal', 0, vrep.simx_opmode_oneshot)
        # ================ END MAIN LOOP ===============
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive.
        vrep.simxGetPingTime(clientID)
        # Now close the connection to V-REP
        vrep.simxFinish(clientID)
else:
    print 'Failed connecting to remote API server'
print 'Program ended'
    

