#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import numpy as np
from PID import PIDController
from ColorDetector import ColorDetector


# Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#PID
pid = PIDController(proportional = 25.0, derivative_time = 0, integral_time=0)
pid.vmin, pid.vmax = -0.15, 0.15
pid.setpoint = 1.5   #aTargetAltitude(m)
TAltitude = pid.setpoint
baseThrust = 0.5

pidout = 0


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff_nogps(TAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6
    
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    if vehicle.mode.name == "INITIALISING":
        print ("Waiting for vehicle to initialise")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print ("Altitude left %s" % (TAltitude - current_altitude))
    
        pidout = int(pid.compute_output(TAltitude - current_altitude))
        pidout += baseThrust
        print ("Height = ", (TAltitude - current_altitude), "px, ", "Thrust = ", pidout)

        set_attitude(thrust = pidout)
        time.sleep(0.1)


def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                             0,
                                                             0,
                                                                 # Target system
                                                             0,
                                                                 # Target component
                                                             0b00000000,
                                                                 # Type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),
                                                                 # Quaternion
                                                             0,
                                                                 # Body roll rate in radian
                                                             0,
                                                                 # Body pitch rate in radian
                                                             math.radians(yaw_rate),
                                                                 # Body yaw rate in radian
                                                             thrust)
                                                                 # Thrust
    vehicle.send_mavlink(msg)
                                                             
    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)
        
        # Sleep for the fractional part
        time.sleep(modf[0])
        
        # Send command to vehicle on 1 Hz cycle
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]
initial_height=vehicle.location.global_relative_frame.alt


# Take off 2.5m in GUIDED_NOGPS mode.

arm_and_takeoff_nogps(TAltitude)

#change to position hold mode
vehicle.mode = VehicleMode("POSHOLD")

#define pidx and pidy as objects of PIDController
        pidx = PIDController(proportional = 25.0, derivative_time = 0, integral_time=0)
        pidx.vmin, pidx.vmax = -0.15, 0.15

        pidy = PIDController(proportional = 25.0, derivative_time = 0, integral_time=0)
        pidy.vmin, pidy.vmax = -0.15, 0.15
        #pid.setpoint??
#centre array
flag1 = True
while flag1:
    command=input("Enter destination ")
    obj = ColorDetector()
    WorkingArr = obj.method_detect()
    if command == 'r':
        xpos=0
        ypos=1
    elif command == 'b':
        xpos=2
        ypos=3
    elif command == 'g':
        xpos=4
        ypos=5
    elif command == 'k':
        xpos=6
        ypos=7
    elif command == 'l':
        #Land
        vehicle.mode = VehicleMode("LAND")
        flag1 = False
        

        # Shut down simulator if it was started.
        if sitl is not None:
            sitl.stop()
        

        print("Completed")
        flag1 = False
        
    flag2 = True
    vehicle.mode = VehicleMode("GUIDED_NOGPS")

    while flag2:

        pidxout = 0
        pidyout = 0
        
        xdes=WorkingArr[xpos]
        ydes=WorkingArr[ypos]

        #error in x and y; x0 and y0 are the central pixel coordinates which shall be calculated depending on the resolution of the camera
        
        errx=xdes-x0
        erry=ydes-y0

        pidxout = int(pidx.compute_output(errx))
        pidyout = int(pidy.compute_output(erry))

        #moving function
        set_attitude(pidxout,pidyout)#pitch and roll angles will be proportional to pidxout and pidyout
        time.sleep(0.1)
        if (errx < 0.5) & (erry <0.5) :#this condition is wrong! This will change mode as soon as error is 0 without letting it stabilise. Putting some delay should work
            vehicle.mode = VehicleMode("POSHOLD")
            print("Destination Reached")
            flag2 = False
    

    
    

