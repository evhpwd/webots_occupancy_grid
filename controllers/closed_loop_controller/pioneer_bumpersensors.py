# pioneer_simpleproxsensors Class Definition
# File: pioneer_simpleproxsensors.py
# Date: 7th Nov 2024
# Description: Proximity Sensor support for COMP329 Lab Tutorials (2024)
# Author: Terry Payne (trp@liv.ac.uk)
# Modifications: Based on pioneer_proxsensors.py (24th Jan 2022) used for
#                COMP329 Programming Assignment (2022)

import math
import pose

class PioneerBumperSensors:
    """ A custom class to manage the 16 proximity sensors on a Pioneer Adept robot """

    def __init__(self, robot):
        
        self.robot = robot
        timestep = int(robot.getBasicTimeStep())

        # Dimensions of the Robot
        # Note that the dimensions of the robot are not strictly circular, as 
        # according to the data sheet the length is 485mm, and width is 381mm
        # so we assume for now the aprox average of the two (i.e. 430mm), in meters
        self.radius = 0.215
        
    # ==================================================================================
    # External (Public) methods
    # ==================================================================================

    def get_value(self, i):
        if (i < len(self.ps)):
            return self.max_range - (self.max_range/self.max_value * self.ps[i].getValue())
        else:
            print("Out of range errorr in get_value()")
            return None

    def get_rawvalue(self, i):
        if (i < len(self.ps)):
            return self.ps[i].getValue()
        else:
            print("Out of range errorr in get_rawvalue()")
            return None
            
    def get_relative_sensor_pose(self, i):
        if (i < len(self.ps)):
            p = self.ps_pose[i]
            return pose.Pose(p.x, p.y, p.theta)
        else:
            print("Out of range errorr in get_relative_sensor_pose()")
            return pose.Pose(0, 0, 0)

    #   New method used by the COMP329 Programming Assignment
    def get_radius(self):
        return self.radius

