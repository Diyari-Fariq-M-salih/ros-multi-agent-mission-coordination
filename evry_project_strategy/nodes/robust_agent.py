#!/usr/bin/env python
import rospy
import math
import tf
import random
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

class Robot:
    def __init__(self, robot_name, all_robot_names):
        self.robot_name = robot_name
        self.pose = None
        self.sonar = 5.0 
        self.other_poses = {} 

        self.sub_odom = rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_callback)
        self.sub_sonar = rospy.Subscriber(f"/{self.robot_name}/sensor/sonar_front", Range, self.sonar_callback)
        self.pub_vel = rospy.Publisher(f"/{self.robot_name}/cmd_vel", Twist, queue_size=10)

        for name in all_robot_names:
            if name != self.robot_name:
                rospy.Subscriber(f"/{name}/odom", Odometry, lambda data, n=name: self.other_odom_callback(data, n))

    def odom_callback(self, data):
        self.pose = data.pose.pose

    def other_odom_callback(self, data, name):
        self.other_poses[name] = data.pose.pose

    def sonar_callback(self, data):
        if data.range == float('inf'):
            self.sonar = 5.0
        else:
            self.sonar = data.range

    def get_pose(self):
        return self.pose

    def get_sonar(self):
        return self.sonar

    def get_yaw(self):
        if self.pose is None: return 0.0
        quaternion = (
            self.pose.orientation.x, self.pose.orientation.y,
            self.pose.orientation.z, self.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2] 

    def set_speed_angle(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_vel.publish(cmd)

def run_demo():
    rospy.init_node('robust_strategy_node', anonymous=True)
    my_name = rospy.get_param("~robot_name", "robot_1")
    
    # DEBUG: print name to be sure
    rospy.loginfo(f"My Name is: {my_name}")
    
    all_robots = ["robot_1", "robot_2", "robot_3"]
    robot = Robot(my_name, all_robots)

    flag_coords = {
        "robot_1": (-21.21, 21.21),
        "robot_2": (21.21, 21.21),
        "robot_3": (0.0, -30.0)
    }
    
    # --- SYMMETRY BREAKER ---
    # Robot 1 turns Left (+1), Robot 2 turns Right (-1)
    # This forces them to turn AWAY from each other.
    turn_preferences = {
        "robot_1": 1,  # Left
        "robot_2": -1, # Right
        "robot_3": 1   # Left
    }
    my_turn_dir = turn_preferences.get(my_name, 1)

    target = flag_coords.get(my_name, (0,0))
    rate = rospy.Rate(10)

    while robot.get_pose() is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # State Variables
    state = "MOVING"
    avoid_start_x = 0
    avoid_start_y = 0
    timer = 0
    
    while not rospy.is_shutdown():
        # 1. DATA
        my_pose = robot.get_pose()
        if my_pose is None: continue
        
        my_x = my_pose.position.x
        my_y = my_pose.position.y
        my_yaw = robot.get_yaw()
        sonar = robot.get_sonar()
        
        # 2. GOAL
        goal_x = target[0] - my_x
        goal_y = target[1] - my_y
        dist_to_goal = math.sqrt(goal_x**2 + goal_y**2)
        target_yaw = math.atan2(goal_y, goal_x)
        
        # 3. REPULSION (Keep robots apart)
        repulse_yaw = 0.0
        ROBOT_SAFE_DIST = 4.0
        closest_robot_dist = 99.0
        
        for name, pose in robot.other_poses.items():
            if pose is None: continue
            dx = my_x - pose.position.x
            dy = my_y - pose.position.y
            d = math.sqrt(dx**2 + dy**2)
            if d < closest_robot_dist: closest_robot_dist = d
            
            if d < ROBOT_SAFE_DIST:
                angle_to_robot = math.atan2(dy, dx)
                # Push opposite direction
                repulse_yaw += (angle_to_robot + math.pi)

        if closest_robot_dist < ROBOT_SAFE_DIST:
            final_desired_yaw = (target_yaw + repulse_yaw) / 2.0
        else:
            final_desired_yaw = target_yaw

        yaw_error = final_desired_yaw - my_yaw
        while yaw_error > math.pi: yaw_error -= 2*math.pi
        while yaw_error < -math.pi: yaw_error += 2*math.pi

        # 4. LOGIC
        PANIC_DIST = 0.8
        OBSTACLE_DIST = 2.0
        PATH_CLEAR_DIST = 3.5
        CLEARANCE_DIST = 2.5 

        linear_vel = 0.0
        angular_vel = 0.0

        if dist_to_goal < 1.0:
            linear_vel = 0.0
            angular_vel = 0.0
            rospy.loginfo_throttle(5, f"{my_name}: Arrived!")

        elif state == "PANIC_REVERSE":
            dist_backed = math.sqrt((my_x - avoid_start_x)**2 + (my_y - avoid_start_y)**2)
            if dist_backed < 1.0:
                linear_vel = -0.5
                angular_vel = 0.0
            else:
                state = "WAITING"
                timer = rospy.Time.now().to_sec()

        elif state == "WAITING":
            # Short random pause to desynchronize robots
            # Robot 1 waits 0.1s, Robot 2 waits 0.5s (randomly)
            if rospy.Time.now().to_sec() - timer < random.uniform(0.1, 0.8):
                linear_vel = 0.0
                angular_vel = 0.0
            else:
                state = "ROTATING"

        elif state == "ROTATING":
            if sonar > PATH_CLEAR_DIST:
                 state = "ROTATING_EXTRA"
                 timer = rospy.Time.now().to_sec()
            else:
                linear_vel = 0.0
                # USE FIXED DIRECTION HERE
                angular_vel = 0.8 * my_turn_dir 

        elif state == "ROTATING_EXTRA":
            if rospy.Time.now().to_sec() - timer < 0.5:
                linear_vel = 0.0
                angular_vel = 0.8 * my_turn_dir
            else:
                state = "DRIVING_CLEAR"
                avoid_start_x = my_x
                avoid_start_y = my_y
                rospy.loginfo(f"{my_name}: Path Clear. Driving Straight.")

        elif state == "DRIVING_CLEAR":
            dist_traveled = math.sqrt((my_x - avoid_start_x)**2 + (my_y - avoid_start_y)**2)
            
            if dist_traveled < CLEARANCE_DIST:
                linear_vel = 0.8
                angular_vel = 0.0 
                if sonar < PANIC_DIST:
                    state = "PANIC_REVERSE"
                    avoid_start_x = my_x
                    avoid_start_y = my_y
            else:
                state = "MOVING"

        elif sonar < PANIC_DIST:
            rospy.logerr(f"{my_name}: Panic!")
            state = "PANIC_REVERSE"
            avoid_start_x = my_x
            avoid_start_y = my_y

        elif sonar < OBSTACLE_DIST:
            rospy.logwarn(f"{my_name}: Obstacle. Avoiding.")
            state = "ROTATING"
            # Do NOT randomize here. Stick to the fixed direction.
            
        else:
            state = "MOVING"
            linear_vel = min(dist_to_goal, 0.8)
            angular_vel = 1.5 * yaw_error
            angular_vel = max(min(angular_vel, 1.2), -1.2)
            if abs(yaw_error) > 0.5: linear_vel = 0.0

        robot.set_speed_angle(linear_vel, angular_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        run_demo()
    except rospy.ROSInterruptException:
        pass