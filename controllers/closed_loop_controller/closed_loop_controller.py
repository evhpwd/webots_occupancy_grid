from controller import Supervisor

import pioneer_clnav as pn
import pioneer_simpleproxsensors as psps
import math
import pose
import time

def run_robot(robot):
        
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    pps = psps.PioneerSimpleProxSensors(robot)
    
    nav = pn.PioneerCLNav(robot, pps, timestep)

    nav.set_goal(pose.Pose(2.5,0,0))
    
    while robot.step(timestep) != -1:
        if (nav.update()):
            nav.stop()

if __name__ == "__main__":
    # create the Supervised Robot instance.
    my_robot = Supervisor()
    run_robot(my_robot)