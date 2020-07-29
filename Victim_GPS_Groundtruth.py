#Version = 1.0
#Author = haiquantran2897@gmail.com

#import lib
import airsim
import math
from math import pi
import cv2
import os
import sys
import tempfile
from airsim import MultirotorClient
import numpy as np 

with open('GPS_groundtruth.csv', 'w') as f:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    client.armDisarm(True)

    landed = client.getMultirotorState().landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
    else:
        print("already flying...")
        client.hoverAsync().join()

    #victim1
    client.moveToPositionAsync(-90, 0, -5, 4).join()
    client.hoverAsync().join()
    state = client.getMultirotorState()
    f.write(str(state.gps_location.latitude) + "," + str(state.gps_location.longitude) + '\n')

    #victim2
    client.moveToPositionAsync(10, -100, -5, 5).join()
    client.hoverAsync().join()
    state = client.getMultirotorState()
    f.write(str(state.gps_location.latitude) + "," + str(state.gps_location.longitude) + '\n')

    #victim3
    client.moveToPositionAsync(110, 0, -5, 5).join()
    client.hoverAsync().join()
    state = client.getMultirotorState()
    f.write(str(state.gps_location.latitude) + "," + str(state.gps_location.longitude) + '\n')

    #victim4
    client.moveToPositionAsync(10, 100, -5, 5).join()
    client.hoverAsync().join()
    state = client.getMultirotorState()
    client.hoverAsync().join()
    client.hoverAsync().join()
    client.hoverAsync().join()
    f.write(str(state.gps_location.latitude) + "," + str(state.gps_location.longitude) + '\n')

    #Landing
    print("landing...")
    client.landAsync().join()

    print("disarming.")
    client.armDisarm(False)