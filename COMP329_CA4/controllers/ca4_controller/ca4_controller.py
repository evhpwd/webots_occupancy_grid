from controller import Supervisor

import pioneer_clnav as pn
import pioneer_simpleproxsensors as psps
import occupancy_grid as ogrid
import math
import pose
import time

def run_robot(robot):
        
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    pps = psps.PioneerSimpleProxSensors(robot)
    nav = pn.PioneerCLNav(robot, pps, timestep)
    
    # print(robot.arena_width, robot.arena_height)
    occupancy_grid = ogrid.OccupancyGrid(robot, 10, "display", nav.get_real_pose(), pps)
    arena_width = occupancy_grid.arena_width
    arena_height = occupancy_grid.arena_height

    explore_coords = [(1,1,0),(1,arena_height-1,0),(arena_width-1,1,0),(arena_width-1,arena_height-1,0)]
    ci = 0
    nav.set_goal(pose.Pose(explore_coords[ci][0],explore_coords[ci][1],explore_coords[ci][2]))
    
    coverage = occupancy_grid.paint()
    
    while coverage < 0.7:
        if (nav.update()):
            print("Goal", ci, "achieved.")
            ci += 1
            nav.set_goal(pose.Pose(explore_coords[ci][0],explore_coords[ci][1],explore_coords[ci][2]))
        
        occupancy_grid.map(nav.get_real_pose())
        coverage = occupancy_grid.paint()
        
    nav.stop()

if __name__ == "__main__":
    # create the Supervised Robot instance.
    my_robot = Supervisor()
    run_robot(my_robot)