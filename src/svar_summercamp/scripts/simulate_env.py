import math
import os
import svar
import random
import subprocess
import time
from os import path

import numpy as np

summercamp = svar.load("summercamp")
ros2 = svar.load("svar_messenger_ros2")
messenger = svar.load('svar_messenger').messenger

data_path = "/home/eric/桌面/summercamp/svar_summercamp/data"


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, map_file, environment_dim):
        self.environment_dim = environment_dim
        self.env = summercamp.Enviroment({
            "ranges_num": environment_dim,
            "map_file": map_file,
            "init_pose": [1, 1, 0, 0, 0, 0, 1],
            "scan2baselink": [0.0, 0, 0, 0, 0, 0, 1],
            "sim_t_factor": 1,
            "robot_radius": 0.3,
            "mesh": data_path + "/C3.stl",
            "min_scan_time_diff": -1.0
        })
        self.goal = self.env.random_free_pose()
        self.pub = messenger.advertise("/robot_goal", 0)
        self.last_stamp = None

    def pub_goal(self, pose, stamp):
        header = {"frame_id": "map", "stamp": stamp}
        t = pose.translation
        r = pose.rotation
        pose = {"orientation": {"x": r.x, "y": r.y, "z": r.z, "w": r.w}, "position": {"x": t.x, "y": t.y, "z": t.z}}
        msg = {"header": header, "pose": pose}

        self.pub.publish(msg)  # 发布定位信息

    # Perform an action and read a new state
    def step(self, dt, action):
        vel = {"linear": {"x": action[0], "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": action[1]}}
        pose, vel, scan = self.env.step(dt, vel)
        self.last_stamp = scan["header"]["stamp"]
        ranges = np.frombuffer(scan["ranges"], dtype="float32")

        done, collision, min_laser = self.observe_collision(ranges)
        v_state = []
        v_state[:] = ranges
        laser_state = [v_state]

        angle = pose.rotation.yaw
        goal2robot = pose.inverse() * self.goal
        distance = goal2robot.translation.norm()
        theta = math.atan2(goal2robot.translation.y, goal2robot.translation.x)

        # Detect if the goal has been reached and give a large positive reward
        target = False
        if distance < 0.2:
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser)
        return state, reward, done, target

    def reset(self):
        self.goal = self.env.random_free_pose()
        self.pub_goal(self.goal, self.last_stamp)
        pose, vel, scan = self.env.reset()
        self.last_stamp = scan["header"]["stamp"]
        ranges = np.frombuffer(scan["ranges"], dtype="float32")

        done, collision, min_laser = self.observe_collision(ranges)
        v_state = []
        v_state[:] = ranges
        laser_state = [v_state]

        angle = pose.rotation.yaw
        goal2robot = pose.inverse() * self.goal
        distance = goal2robot.translation.norm()
        theta = math.atan2(goal2robot.translation.y, goal2robot.translation.x)

        # Detect if the goal has been reached and give a large positive reward
        target = False
        if distance < 0.2:
            target = True
            done = True

        robot_state = [distance, theta, 0, 0]
        state = np.append(laser_state, robot_state)
        return state

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser <= 0.3:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        if target:
            return 100.0
        elif collision:
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2


transfer = ros2.Transfer({
    "node":
    "simulator",
    "subscriptions": [["/vel_mcu", "geometry_msgs/msg/Twist", 10]],  # 用于控制机器人运动
    "publishers": [
        ["/odom", "nav_msgs/msg/Odometry", 10],  # 里程计信息
        ["/map", "nav_msgs/msg/OccupancyGrid", 10],  # 栅格地图信息
        ["/map_cloud", "sensor_msgs/msg/PointCloud2", 10],  #用点云表达的墙面
        ["/markers", "visualization_msgs/msg/MarkerArray", 10],  # 用于显示机器人
        ["/tf", "tf2_msgs/msg/TFMessage", 10],  # 用于显示机器人
        ["/robot_goal", "geometry_msgs/msg/PoseStamped", 10],  # 用于显示机器人目标点
        ["/scan", "sensor_msgs/msg/LaserScan", 10]
    ]
})  # 激光雷达

if __name__ == "__main__":
    env = GazeboEnv(data_path + "/map", 360)

    for j in range(1, 100):
        env.reset()

        for i in range(1, 10000):
            env.step(0.1, [0.1, 0.1])

            time.sleep(0.001)
