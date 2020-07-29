#NAME=ASE Ocean
#VERSION=1.0
#AUTHOR=TRAN HAI QUAN - haiquantran2897@gmail.com
#RUN CODE: python Get_Dataset.py --radius 10 --altitude 8 --speed 4 --center "10,0" --iterations 1
#-------------------------------------------
import setup_path 
import airsim
import os
import sys
import math
import time
import argparse
import cv2
import numpy as np 

# Khoi tao vi tri x, y, z
class Position:
    def __init__(self, pos):
        self.x = pos.x_val
        self.y = pos.y_val
        self.z = pos.z_val

# Make the drone fly in a circle.
class OrbitNavigator:
    def __init__(self, radius = 10, altitude = 10, speed = 2, iterations = 1, center = [10,0]):
        self.radius = radius #ban kinh R
        self.altitude = altitude # do cao
        self.speed = speed #toc do
        self.iterations = iterations #so vong lap
        self.z = None
        self.takeoff = False # whether we did a take off

        if self.iterations <= 0: #Neu vong lap nho hon 0 thi bay interation = 1
            self.iterations = 1

        if len(center) != 2: #Neu do dai center khac 2 
            raise Exception("Expecting '[x,y]' for the center direction vector")
        
        # center is just a direction vector, so normalize it to compute the actual cx,cy locations.
        # center chi la mot vector co huong, vi vay chuan ho no de tinh toan vi tri cx, cy thuc te 
        cx = float(center[0])
        cy = float(center[1])
        

        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        #lay vi tri hien tai cua drone
        self.home = self.client.getMultirotorState().kinematics_estimated.position

        # check that our home position is stable
        # kiem tra vi tri cua drone co on dinh khong
        start = time.time()
        count = 0
        while count < 100:
            pos = self.home
            if abs(pos.z_val - self.home.z_val) > 1:                                
                count = 0
                self.home = pos
                if time.time() - start > 10:
                    #drone dang bi tron, chung toi dang cho no on dinh truoc khi cat canh
                    print("Drone position is drifting, we are waiting for it to settle down...")
                    start = time
            else:
                count += 1

        # Gan center bang vi tri hien tai cua drone
        
        self.center_x_val = cx
        self.center_y_val = cy
    
    # Ham bat dau chay chuong trinh
    def start(self):
        print("arming the drone...")
        self.client.armDisarm(True)
        
        # AirSim uses NED coordinates so negative axis is up.
        # Airsim su dung toa do NED truc am (-z) la huong len tren (do cao am)
        start = self.client.getMultirotorState().kinematics_estimated.position #lay gia tri vi tri cua drone
        landed = self.client.getMultirotorState().landed_state #cat canh
        if not self.takeoff and landed == airsim.LandedState.Landed: 
            self.takeoff = True
            print("taking off...")
            self.client.takeoffAsync().join()
            start = self.client.getMultirotorState().kinematics_estimated.position
            z = -self.altitude + self.home.z_val
        else:
            print("already flying so we will orbit at current altitude {}".format(start.z_val))
            z = start.z_val # use current altitude then

        print("climbing to position: {},{},{}".format(start.x_val, start.y_val, z))
        self.client.moveToPositionAsync(start.x_val, start.y_val, z, self.speed).join()
        self.z = z
        
        print("ramping up to speed...")
        count = 0
        self.start_angle = None
        
        
        # ramp up time
        ramptime = self.radius / 10
        self.start_time = time.time()        
        victim_id = 0
        frame_rate = 0

        # # config video
        # frame_width = 512
        # frame_height = 288
        # out = cv2.VideoWriter('SAR.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

        with open('GPS.csv', 'w') as f:
            while count < self.iterations:

                # ramp up to full speed in smooth increments so we don't start too aggressively.
                now = time.time()
                speed = self.speed
                diff = now - self.start_time
                if diff < ramptime:
                    speed = self.speed * diff / ramptime
                elif ramptime > 0:
                    print("reached full speed...")
                    ramptime = 0
                    
                lookahead_angle = speed / self.radius            

                # compute current angle
                pos = self.client.getMultirotorState().kinematics_estimated.position
                print(pos)
                dx = pos.x_val - self.center_x_val
                dy = pos.y_val - self.center_y_val
                actual_radius = math.sqrt((dx*dx) + (dy*dy))
                angle_to_center = math.atan2(dy, dx)

                camera_heading = (angle_to_center - math.pi) * 180 / math.pi 

                # compute lookahead
                lookahead_x = self.center_x_val + self.radius * math.cos(angle_to_center + lookahead_angle)
                lookahead_y = self.center_y_val + self.radius * math.sin(angle_to_center + lookahead_angle)

                vx = lookahead_x - pos.x_val
                vy = lookahead_y - pos.y_val

                if self.track_orbits(angle_to_center * 180 / math.pi):
                    count += 1
                    self.radius += 5 #cu xong mot vong thi tang ban kinh len 5m
                    print("completed {} orbits".format(count))
                
                self.camera_heading = camera_heading
                self.client.moveByVelocityZAsync(vx, vy, z, 1, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, camera_heading))

                # video stream
                # phat luong video thu duoc tu camera
                CAMERA_NAME = '3'
                IMAGE_TYPE = airsim.ImageType.Scene
                raw_img = self.client.simGetImage(CAMERA_NAME, IMAGE_TYPE)
                decode_img = np.fromstring(raw_img, np.int8)
                img = cv2.imdecode(decode_img, cv2.IMREAD_UNCHANGED)
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)

                # # video write
                # out.write(img)
                cv2.imshow('frame',img)

                if frame_rate % 3 == 0:
                    gps_state = self.client.getMultirotorState()
                    cv2.imwrite('./datasetGPS/victims' + '_' + str(victim_id) + '.png', img)
                    f.write('ase' + '_' + str(victim_id) + '.png' + "," + str(gps_state.gps_location.latitude) + "," + str(gps_state.gps_location.longitude) + "," + str(gps_state.gps_location.altitude) + '\n')
                    victim_id += 1
                frame_rate += 1
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print('Stream Camera')

        if self.takeoff:            
            # if we did the takeoff then also do the landing.
            if z < self.home.z_val:
                print("descending")
                self.client.hoverAsync().join()

            print("landing...")
            self.client.landAsync().join()

            print("disarming.")
            self.client.armDisarm(False)


    def track_orbits(self, angle):
        # tracking # of completed orbits is surprisingly tricky to get right in order to handle random wobbles
        # about the starting point.  So we watch for complete 1/2 orbits to avoid that problem.
        if angle < 0:
            angle += 360

        if self.start_angle is None:
            self.start_angle = angle
            self.previous_angle = angle
            self.shifted = False
            self.previous_sign = None
            self.previous_diff = None            
            self.quarter = False
            return False

        # now we just have to watch for a smooth crossing from negative diff to positive diff
        if self.previous_angle is None:
            self.previous_angle = angle
            return False            

        diff = self.previous_angle - angle
        crossing = False
        self.previous_angle = angle


        diff = abs(angle - self.start_angle)
        if diff > 45:
            self.quarter = True

        if self.quarter and self.previous_diff is not None and diff != self.previous_diff:
            # watch direction this diff is moving if it switches from shrinking to growing
            # then we passed the starting point.
            direction = self.sign(self.previous_diff - diff)
            if self.previous_sign is None:
                self.previous_sign = direction
            elif self.previous_sign > 0 and direction < 0:
                if diff < 45:
                    self.quarter = False
                    crossing = True
            self.previous_sign = direction
        self.previous_diff = diff

        return crossing

    def sign(self, s):
        if s < 0: 
            return -1
        return 1

if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Orbit.py makes drone fly in a circle with camera pointed at the given center vector")
    arg_parser.add_argument("--radius", type=float, help="radius of the orbit", default=10)
    arg_parser.add_argument("--altitude", type=float, help="altitude of orbit (in positive meters)", default=20)
    arg_parser.add_argument("--speed", type=float, help="speed of orbit (in meters/second)", default=3)
    arg_parser.add_argument("--center", help="x,y direction vector pointing to center of orbit from current starting position (default 1,0)", default="1,0")
    arg_parser.add_argument("--iterations", type=float, help="number of 360 degree orbits (default 3)", default=3)    
    args = arg_parser.parse_args(args)    
    nav = OrbitNavigator(args.radius, args.altitude, args.speed, args.iterations, args.center.split(','))
    nav.start() 