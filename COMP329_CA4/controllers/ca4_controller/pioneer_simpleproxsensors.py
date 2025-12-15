import math
import pose

class PioneerSimpleProxSensors:
    """ A custom class to manage the 16 proximity sensors on a Pioneer Adept robot """

    MAX_NUM_SENSORS = 16

    def __init__(self, robot):
        
        self.robot = robot
        timestep = int(robot.getBasicTimeStep())

        self.radius = 0.215

        # set up proximity detectors
        self.ps = []
        for i in range(self.MAX_NUM_SENSORS):
            sensor_name = 'so' + str(i)
            self.ps.append(robot.getDevice(sensor_name))
            self.ps[i].enable(timestep)

        ps_degAngles = [
            90, 50, 30, 10, -10, -30, -50, -90,
            -90, -130, -150, -170, 170, 150, 130, 90
        ]
        ps_angles = [None] * len(ps_degAngles)
        for i in range(len(ps_angles)):
            ps_angles[i] = math.radians(ps_degAngles[i])
                
        # -------------------------------------------
        # Determine the poses of each of the sensors
        self.ps_pose = []
        for i in ps_angles:
            p = pose.Pose(math.cos(i) * self.radius, math.sin(i) * self.radius, i)
            self.ps_pose.append(p)

        # ------------------------------------------
        # Determine max range from lookup table
        lt = self.ps[0].getLookupTable()
        print(f"Lookup Table has {len(lt)} entries")
        self.max_range = 0.0
        for i in range(len(lt)):
            if ((i%3) == 0):
                self.max_range = lt[i]
            print(f" {lt[i]}", end='')
            if ((i%3) == 2):
                print("") # Newline

        self.max_value = self.ps[0].getMaxValue()
        print(f"Max Range: {self.max_range}")
        
    # ==================================================================================
    # External (Public) methods
    # ==================================================================================
  
    # Insert public methods here
    def get_maxRange(self):
        return self.max_range

    def get_number_of_sensors(self):
        return len(self.ps)

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

