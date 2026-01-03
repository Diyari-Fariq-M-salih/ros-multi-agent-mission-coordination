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
        self.robot_name = robot_name
        self.sonar = 0.0
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front", Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom", Odometry, self.callbackPose)

        self.cmd_vel_pub = rospy.Publisher(self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        self.sonar = msg.range

    def callbackPose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.yaw = yaw

    def get_robot_pose(self):
        return self.x, self.y, self.yaw

    def constraint(self, val, min_val=-2.0, max_val=2.0):
        return max(min_val, min(max_val, val))

    def set_speed_angle(self, linear, angular):
        msg = Twist()
        msg.linear.x = self.constraint(linear)
        msg.angular.z = self.constraint(angular, min_val=-1, max_val=1)
        self.cmd_vel_pub.publish(msg)

    def getDistanceToFlag(self):
        rospy.wait_for_service('/distanceToFlag')
        try:
            srv = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D(x=self.x, y=self.y)
            result = srv(pose, int(self.robot_name[-1]))
            return result.distance
        except:
            return 9999.0


# ------------------------------ PID CLASS ----------------------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev = 0.0
        self.integral = 0.0
        self.last = rospy.Time.now()

    def compute(self, error):
        now = rospy.Time.now()
        dt = (now - self.last).to_sec()
        if dt <= 0: dt = 1e-6
        self.integral += error * dt
        derivative = (error - self.prev) / dt
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev = error
        self.last = now
        return out


# --------------------------- FLAG POSITIONS --------------------------------
FLAG_POS = {
    1: (-21.21320344,  21.21320344),
    2: ( 21.21320344,  21.21320344),
    3: (  0.0,        -30.0)
}


# --------------------------- MAIN LOOP -------------------------------------
def run_demo():
    robot_name = rospy.get_param("~robot_name")
    robot_id = int(robot_name[-1])
    robot = Robot(robot_name)

    print(f"[{robot_name}] PID + Delay controller started.")

    # PID controllers
    linPID = PID(0.25, 0.0, 0.12)
    angPID = PID(1.2, 0.0, 0.05)

    # Timing Strategy: robot_i waits (i-1)*5 sec
    delay = (robot_id - 1) * 5.0
    start = rospy.Time.now().to_sec()

    # Target Flag
    flag_x, flag_y = FLAG_POS[robot_id]

    stop_dist = 0.30

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()

        # TIMING STRATEGY
        if now - start < delay:
            robot.set_speed_angle(0, 0)
            print(f"[{robot_name}] Waiting {delay - (now - start):.1f} sec...")
            rospy.sleep(0.1)
            continue

        # GET ROBOT STATE
        x, y, yaw = robot.get_robot_pose()
        dist = robot.getDistanceToFlag()
        print(f"[{robot_name}] dist={dist}")

        # STOP CONDITION
        if dist < stop_dist:
            robot.set_speed_angle(0, 0)
            print(f"[{robot_name}] Flag reached. Stopping.")
            rospy.sleep(0.1)
            continue

        # LINEAR PID
        v = linPID.compute(dist)
        v = max(0, min(v, 2.0))

        # ANGULAR PID
        desired_angle = math.atan2(flag_y - y, flag_x - x)
        angle_err = math.atan2(math.sin(desired_angle - yaw), math.cos(desired_angle - yaw))
        w = angPID.compute(angle_err)

        # SEND COMMAND
        robot.set_speed_angle(v, w)

        rospy.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("PID_multi_robot", anonymous=True)
    run_demo()
