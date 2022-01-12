import rclpy
from rclpy.node import Node

import math
import numpy as np

from std_msgs.msg import String, Float32, Int64
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3

import math
from enum import Enum


def dwa_control(x, config, goal, ob):

    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    rectangle = 1


class Config:

    def __init__(self):

        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 0.5  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([5.544445, 5.544445])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()

class dwa_planner(Node):

    def __init__(self):
        super().__init__('dwa_planner')
        self.turtle2_x = 0.0
        self.turtle2_y = 0.0
        self.turtle2_theta = 0.0
        self.turtle2_linear_velocity = 0.0
        self.turtle2_angular_velocity = 0.0

        self.twist = None
        self.linear = None
        self.angular = None

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        self.subscriber = self.create_subscription(Pose, '/turtle2/pose', self.subscribe_msg, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        timer_period = 1  
        self.timer = self.create_timer(timer_period, self.publish_msg)

    def subscribe_msg(self, msg):
        self.turtle2_x = msg.x
        self.turtle2_y = msg.y
        self.turtle2_theta = msg.theta
        self.turtle2_linear_velocity = msg.linear_velocity
        self.turtle2_angular_velocity = msg.angular_velocity

    def publish_msg(self):
        self.twist = Twist()
        self.linear = Vector3()
        self.angular = Vector3()

        self.linear.x = planner.turtle2_linear_velocity * math.cos(planner.turtle2_theta) * config.dt
        self.linear.y = planner.turtle2_linear_velocity * math.sin(planner.turtle2_theta) * config.dt
        self.linear.z = planner.linear_z

        self.angular.x = self.angular_x
        self.angular.y = self.angular_y
        self.angular.z = self.angular_z

        self.twist.linear = self.linear
        self.twist.angular = self.angular
    
        self.publisher.publish(self.twist)


planner = dwa_planner()

def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    
    
    return x



def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost



def main(gx=8.58, gy=2.5, robot_type=RobotType.rectangle, args=None):

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([planner.turtle2_x, planner.turtle2_y, planner.turtle2_theta, planner.turtle2_linear_velocity, planner.turtle2_angular_velocity])

    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob

    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = motion(x, u, config.dt)  # simulate robot

        # x[0] : x(m)
        # x[1] : y(m)
        # x[2] : theta(rad)
        # x[3] : linear_vel(m/s)
        # x[4] : angular_vel(rad/s)

        planner.turtle2_x = x[0]
        planner.turtle2_y = x[1]
        planner.turtle2_theta = x[2]
        planner.turtle2_linear_velocity = x[3]
        planner.turtle2_angular_velocity = x[4]


        trajectory = np.vstack((trajectory, x))  # store state history

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_width * 0.5:
            break

    rclpy.init(args=args)
    planner_start = dwa_planner()
    rclpy.spin(planner_start)
    planner_start.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

