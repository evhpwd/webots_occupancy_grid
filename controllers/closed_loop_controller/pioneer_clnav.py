from controller import Robot, TouchSensor
import math
import pose
import pioneer_simpleproxsensors as psps
import pioneer_bumpersensors as pbps

class PioneerCLNav:
    """ A custom class to demonstrate a closed loop controller with bug1 """

    WALLFOLLOWING = True
    GOALSEEKING = False
    WF_HIT = 0
    WF_SEARCH = 1
    WF_LEAVE = 2
    WF_ROTATE = 3

    BUG_RADIUS = 0.4;   # For wall following, allow a greater distance to hit/leave points
    GOAL_RADIUS = 0.1   # If the robot is this dist from goal, then success
    GOAL_VEL = 1        # min velocity when robot reaches goal before stopping
    START_VEL = 3       # Initial move velocity
    ACCEL_DIST = 0.4    # Accelerate up to distance away from the start  
    DECEL_DIST = 0.8    # Decelerate when this distance away from the goal  
    ACCEL_ANGLE = 0.524 # Decelerate rotation when this angle (rads) from goal
    WF_DIST = 0.3       # Distance when wall following
    WF_VEL = 5          # Default velocity when wall following

    # ==================================================================================
    # Constructor
    # ==================================================================================
    def __init__(self, robot, prox_sensors, timestep):
        
        self.robot = robot
        self.robot_node = self.robot.getSelf()    # reference to the robot node
        self.pps = prox_sensors                   # reference to the sensor model
        self.left_ts = self.robot.getDevice("left touch sensor")
        self.right_ts = self.robot.getDevice("right touch sensor")
        self.left_ts.enable(timestep)
        self.right_ts.enable(timestep)
        self.goal = self.get_real_pose()          # assume the goal is our current position
        self.start = self.get_real_pose()         # retain our initial position
        self.state = self.GOALSEEKING

        self.prev_error = 0              # Reset values used by the PID controller
        self.total_error = 0             # Reset values used by the PID controller
    
        # enable motors
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        self.max_vel = self.left_motor.getMaxVelocity()
                
        # Initialise motor velocity
        self.stop()  

    # ==================================================================================
    # Internal (helper) methods
    # ==================================================================================
    def set_velocity(self, lv, rv):
        self.left_motor.setVelocity(lv)
        self.right_motor.setVelocity(rv)       

    # returns the range to an obstacle in front of the robot
    # up to max_range. Used to trigger wall following
    def range_to_frontobstacle(self):
        range = min(self.pps.get_maxRange(),
                    self.pps.get_value(2),
                    self.pps.get_value(3),
                    self.pps.get_value(4),
                    self.pps.get_value(5))
        return range

    def range_to_frontcornerobstacle(self):
        range = min(self.pps.get_maxRange(),
                    self.pps.get_value(1),
                    self.pps.get_value(2),
                    self.pps.get_value(5),
                    self.pps.get_value(6))
        return range

    def range_to_leftobstacle(self):
        range = min(self.pps.get_maxRange(),
                    self.pps.get_value(0))
        return range

    # ==================================================================================
    # Internal (navigation) methods
    # ==================================================================================
    def pid(self, error):
        kp = 6.0   # proportional weight (may need tuning)
        kd = 40.0  # differential weight (may need tuning)
        ki = 0.0   # integral weight (may need tuning)
            
        prop = error
        diff = error - self.prev_error
        self.total_error += error
            
        if ((self.pid_counter % 4)==0):
            self.pid_diff = error - self.prev_error
            self.prev_error = error
        self.pid_counter+=1  # increment our counter
            
        return (kp * prop) + (ki * self.total_error) + (kd * self.pid_diff)

    def check_bumper(self):
        # if bumper not colliding, return false
        # otherwise, move back a bit
        left_bump = self.left_ts.getValue()
        right_bump = self.right_ts.getValue()
        if ((left_bump == 0 and right_bump == 0) or math.isnan(left_bump) or math.isnan(right_bump)):
            return False
            
        s = str(left_bump) + " " + str(right_bump)
        speed = 6
        right_mod = 1
        left_mod = 1
        if (left_bump):
            s += " left bump!"
            left_mod = -1/2
        if (right_bump):
            s += " right bump!"
            right_mod = -1/2
        print(s)
        
        current_time = self.robot.getTime()
        stop_time = current_time + 20
        while (self.robot.getTime() < stop_time):
            if (current_time < stop_time - 5):
                lv = -speed/2
                rv = -speed/2
            else:
                lv = speed * left_mod
                rv = speed * right_mod
            self.set_velocity(lv, rv)
        
        return True

    # ==================================================================================
    def adjust_velocity(self, bearing, velocity):
        rotate_vel = min(self.max_vel, abs(bearing*self.max_vel/self.ACCEL_ANGLE))
        if (bearing < 0):
            rotate_vel = -rotate_vel
            
        lv = min(self.max_vel, velocity-rotate_vel)
        rv = min(self.max_vel, velocity+rotate_vel)
        self.set_velocity(lv, rv)
        
    # ==================================================================================
    def rotate(self, bearing, velocity):
        # Check we are (approximately) at the correct
        # orientation (i.e. is the bearing small?)
        if (abs(bearing) < 0.1):
            return True
        self.adjust_velocity(bearing, velocity)
        return False

    # ==================================================================================
    def track_leave_point(self, p):
        dist = p.get_range(self.goal)  # distance to goal
        if (dist < self.leave_dist):
            self.leave_dist = dist
            self.leave_point = p
    # ==================================================================================
    # Main logic for the Bug 1 Algorithm
    # ==================================================================================
    # Note we don't handle unaccessible goals
    def bug1(self, p):
        hit_dist = p.get_range(self.hit_point)
        if (self.wf_state == self.WF_ROTATE):
            # About to leave, but need to rotate to the goal
            bearing = p.get_bearing(self.goal)
            print(f"State: {self.wf_state}, "+
                  f"goal dist {p.get_range(self.goal):.03f} "+
                  f"bearing: {bearing:.03f} ")
            if (self.rotate(bearing, self.WF_VEL)):
                self.state = self.GOALSEEKING
                self.start = self.get_real_pose()  # reset start point
            return
        
        if (self.wf_state == self.WF_LEAVE):
            print(f"State: {self.wf_state}, "+
                  f"goal dist {p.get_range(self.goal):.03f} "+
                  f"dist to leave: {p.get_range(self.leave_point):.03f} ")
            # Found Leave point; now need to rotate to goal
            if (p.get_range(self.leave_point) < self.BUG_RADIUS):
                self.wf_state = self.WF_ROTATE
                return
 
        if (self.wf_state == self.WF_SEARCH):
            print(f"State: {self.wf_state}, hit dist:{hit_dist:.3f} "+
                  f"goal dist {p.get_range(self.goal):.03f} "+
                  f"leave point: {self.leave_point} {self.leave_dist:.03f}")
            # Tracking distance to goal, and checking for hit point      
            if (hit_dist < self.BUG_RADIUS):
                self.wf_state = self.WF_LEAVE
            else:
                self.track_leave_point(p)

        if (self.wf_state == self.WF_HIT):
            print(f"State: {self.wf_state}, hit dist:{hit_dist:.3f} "+
                  f"goal dist {p.get_range(self.goal):.03f} "+
                  f"leave point: {self.leave_point} {self.leave_dist:.03f}")
            # Don't want to check for hit point until we have moved away      
            if (hit_dist > self.BUG_RADIUS):
                self.wf_state = self.WF_SEARCH
            self.track_leave_point(p)
        
        self.wall_following(p)              
    # ==================================================================================
    def wall_following(self, p):
        hit_dist = p.get_range(self.hit_point)
        
        # Is the obstacle in front of us?  If so, then rotate right,
        # around the right wheel
        front_range = min(self.range_to_frontobstacle(), 
                          self.range_to_frontcornerobstacle())
        if (front_range <= self.WF_DIST):
            self.set_velocity(self.WF_VEL, 0.0)
        else:
            left_range = self.range_to_leftobstacle()
            if (left_range < self.pps.get_maxRange()):
                error = left_range - self.WF_DIST
                control = self.pid(error);
        
                # Keep control in a set range
                control = min(control, self.WF_VEL)
                control = max(control, -self.WF_VEL)      
                self.set_velocity(self.WF_VEL, self.WF_VEL+control)
            else:
                # No wall, so turn
                self.set_velocity(self.WF_VEL*0.25, self.WF_VEL*0.75)

    # ==================================================================================
    def goal_seeking(self, p):
        accel_rate = (self.max_vel-self.START_VEL)/self.ACCEL_DIST
        decel_rate = (self.max_vel-self.GOAL_VEL)/(self.DECEL_DIST-self.GOAL_RADIUS)
        yintercept = self.max_vel - (decel_rate * self.DECEL_DIST)
        
        start_dist = p.get_range(self.start)    # distance to goal
        goal_dist = p.get_range(self.goal)     # distance to goal
        bearing = p.get_bearing(self.goal)     # bearing to goal
    
        obstacle_dist = self.range_to_frontobstacle()
        if (obstacle_dist <= self.WF_DIST):
            self.state = self.WALLFOLLOWING
            self.wf_state = self.WF_HIT
            self.prev_error = 0       # Reset PID controller values
            self.total_error = 0      # Reset PID controller values 
            self.pid_counter = 0      # update every loop
            self.pid_diff = 0.0
            self.hit_point = p
            self.leave_point = p
            self.leave_dist = goal_dist
        else:
            # Calculate acceleration velocity when leaving start
            accel_vel = min(self.max_vel,(start_dist*accel_rate) + self.START_VEL)
    
            # Calculate deceleration velocity when approaching goal      
            decel_vel = min(self.max_vel, (goal_dist*decel_rate) + yintercept);
            
            # Update deceleration velocity (e.g. when approaching obstacle
            if ((obstacle_dist - self.WF_DIST) < self.ACCEL_DIST):
                decel_vel = min(decel_vel, ((obstacle_dist - self.WF_DIST)*decel_rate) + yintercept)
                
            final_vel = min(decel_vel, accel_vel)
            self.adjust_velocity(bearing, final_vel)

    # ==================================================================================
    # External methods (Navigation)
    # ==================================================================================

    # ==================================================================================
    def stop(self):
        self.set_velocity(0.0, 0.0)

    # ==================================================================================
    # Get Real Pose - ask the supervisor where the robot is
    # ================================================================================== 
    def get_real_pose(self):
        if self.robot_node is None:
            return pose.Pose(0, 0, 0)
            
        real_pos = self.robot_node.getPosition()
        rot = self.robot_node.getOrientation()
        theta = math.atan2(-rot[0], rot[3])
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi):
            theta2 = -(3 * halfpi) + theta
        
        return pose.Pose(real_pos[0], real_pos[1], theta2)

    # ==================================================================================
    # Go to some pose
    # ==================================================================================
    def update(self):
        p = self.get_real_pose()
        if (p.get_range(self.goal) < self.GOAL_RADIUS):
            self.stop()
            return True
        
        if (self.check_bumper()):
            print("bump!")
        if (self.state == self.GOALSEEKING):
            self.goal_seeking(p)
        else:
            self.bug1(p)
        return False    # Not yet found goal
    
    def set_goal(self, p):
        self.goal = pose.Pose(p.x, p.y, p.theta)
        self.start = self.get_real_pose()
        self.update()
