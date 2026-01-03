#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag


# -------------------------------------------------------
#                   ROBOT CLASS
# -------------------------------------------------------
class Robot:
    def __init__(self, robot_name):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        self.robot_name = robot_name

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)

        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        self.sonar = msg.range

    def callbackPose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        self.yaw = yaw

    def get_robot_pose(self):
        return self.x, self.y, self.yaw

    def constraint(self, val, min_val=-2.0, max_val=2.0):
        if val < min_val:
            return min_val
        if val > max_val:
            return max_val
        return val

    def set_speed_angle(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min_val=-1, max_val=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return 9999.0


# -------------------------------------------------------
#                   SIMPLE PID CLASS
# -------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()

    def compute(self, error):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0:
            dt = 1e-6

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.last_time = now

        return output


# -------------------------------------------------------
#                   MAIN CONTROL LOOP
# -------------------------------------------------------
def run_demo():
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot {robot_name} starting with FULL PID controller...")

    # PID controllers
    linear_pid = PID(kp=0.25, ki=0.0, kd=0.12)
    angular_pid = PID(kp=1.2, ki=0.0, kd=0.05)

    stop_threshold = 0.30  # meters

    # Fixed target: FLAG 1 position
    flag_x = -21.21320344
    flag_y =  21.21320344

    while not rospy.is_shutdown():

        x, y, yaw = robot.get_robot_pose()
        distance = robot.getDistanceToFlag()
        print("Distance to flag:", distance)

        # ------------- Stop if close enough ----------------
        if distance < stop_threshold:
            print("Flag reached. Stopping robot.")
            robot.set_speed_angle(0, 0)
            rospy.sleep(0.1)
            continue

        # ------------- PID LINEAR VELOCITY CONTROL ----------
        linear_cmd = linear_pid.compute(distance)
        max_speed = 10
        velocity = max(0.0, min(linear_cmd, max_speed))

        # ------------- PID ANGULAR CONTROL -----------------
        # compute desired heading toward flag
        target_angle = math.atan2(flag_y - y, flag_x - x)

        # angle error with wrap-around
        angle_error = target_angle - yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        angular_cmd = angular_pid.compute(angle_error)

        # publish PID outputs
        robot.set_speed_angle(velocity, angular_cmd)

        rospy.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("PID_Controller", anonymous=True)
    run_demo()
