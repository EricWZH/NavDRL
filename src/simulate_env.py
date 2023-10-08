import math
import os
import svar
import random
import subprocess
import time
from os import path

import numpy as np
import cv2
import torch

summercamp = svar.load("summercamp")
ros2 = svar.load("svar_messenger_ros2")
messenger = svar.load('svar_messenger').messenger

data_path = os.path.abspath(os.path.split(__file__)[0] + "/../data")

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.3


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, map_file, environment_dim):
        self.environment_dim = environment_dim
        self.env = summercamp.Enviroment({
            "ranges_num": environment_dim,
            "map_file": map_file,
            "init_pose": [1, 1, 0, 0, 0, 0, 1],
            "scan2baselink": [0.0, 0, 0, 0, 0, 0, 1],
            "sim_t_factor": 10,
            "max_vx": 0.4,
            "robot_radius": 0.3,
            "mesh": data_path + "/C3.stl",
            "min_scan_time_diff": -1.0
        })
        self.goal = self.env.random_free_pose()
        self.pub = messenger.advertise("/robot_goal", 0)
        self.last_stamp = None
        self.last_vel = None
        self.last_pose = None
        self.dises = []

    def pub_goal(self, pose, stamp):
        if stamp is None:
            return
        header = {"frame_id": "map", "stamp": stamp}
        t = pose.translation
        r = pose.rotation
        pose = {"orientation": {"x": r.x, "y": r.y, "z": r.z, "w": r.w}, "position": {"x": t.x, "y": t.y, "z": t.z}}
        msg = {"header": header, "pose": pose}

        self.pub.publish(msg)  # 发布定位信息

    # Perform an action and read a new state
    def step(self, action):
        vel = {
            "linear": {
                "x": float(action[0] + self.last_vel.translation.x),
                "y": 0,
                "z": 0
            },
            "angular": {
                "x": 0,
                "y": 0,
                "z": float(action[1] + self.last_vel.rotation.log().z)
            }
        }
        pose, vel, scan = self.env.step(0.1, vel)
        self.last_vel = vel
        self.last_pose = pose
        #time.sleep(0.04)
        self.last_stamp = scan["header"]["stamp"]
        ranges = np.frombuffer(scan["ranges"], dtype="float32")

        done, collision, min_laser = self.observe_collision(ranges)
        if collision:
            print("collision", min_laser)
        v_state = []
        v_state[:] = ranges
        laser_state = [v_state]

        #self.laser_processing(v_state)

        angle = pose.rotation.yaw
        goal2robot = pose.inverse() * self.goal
        distance = goal2robot.translation.norm()
        theta = math.atan2(goal2robot.translation.y, goal2robot.translation.x)

        # Detect if the goal has been reached and give a large positive reward
        target = False
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True
            print("arrived")

        robot_state = [distance, theta, vel.translation.x, vel.rotation.log().z]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, min_laser, distance, theta, vel, pose.translation)
        return state, reward, done, target

    def reset(self, goal_min=GOAL_REACHED_DIST, goal_radius=1.0, eval=False):
        d1, d2, d = 4.5, 4.7, 2.0
        if eval:
            # while 1:
            #     pose, vel, scan = self.env.reset()
            #     if d1 < max(abs(pose.translation.x), abs(pose.translation.y)) < d2:
            #         break
            if self.last_pose and not d1 - 0.3 < max(abs(self.last_pose.translation.x), abs(self.last_pose.translation.y)) < d2 + 0.3:
                self.last_pose = None
            while self.last_pose:
                pose, vel, scan = self.env.reset()
                if d1 < max(abs(pose.translation.x), abs(pose.translation.y)) < d2 and (pose.inverse() * self.last_pose).translation.norm() < 0.5:
                    self.last_pose = pose
                    break
            while not self.last_pose:
                pose, vel, scan = self.env.reset()
                if d1 < max(abs(pose.translation.x), abs(pose.translation.y)) < d2:
                    self.last_pose = pose
                    break

            self.last_vel = vel
            while 1:
                self.goal = self.env.random_free_pose()
                if d1 < max(abs(self.goal.translation.x), abs(self.goal.translation.y)) < d2:
                    if d1 < pose.translation.x < d2 and pose.translation.y <= d:
                        if d1 < self.goal.translation.x < d2 and d < self.goal.translation.y < d + 1:
                            break
                    if -d2 < pose.translation.x < -d1 and -d <= pose.translation.y:
                        if -d2 < self.goal.translation.x < -d1 and -d - 1 < self.goal.translation.y < -d:
                            break
                    if pose.translation.x <= d and -d2 < pose.translation.y < -d1:
                        if d < self.goal.translation.x < d + 1 and -d2 < self.goal.translation.y < -d1:
                            break
                    if -d <= pose.translation.x and d1 < pose.translation.y < d2:
                        if -d - 1 < self.goal.translation.x < -d and d1 < self.goal.translation.y < d2:
                            break
                    if d1 < pose.translation.x < d2 and d < pose.translation.y:
                        if d - 2 < self.goal.translation.x < d - 1 and d1 < self.goal.translation.y < d2:
                            break
                    if d < pose.translation.x and -d2 < pose.translation.y < -d1:
                        if d1 < self.goal.translation.x < d2 and -d + 1 < self.goal.translation.y < -d + 2:
                            break
                    if -d2 < pose.translation.x < -d1 and pose.translation.y < -d:
                        if -d + 1 < self.goal.translation.x < -d + 2 and -d2 < self.goal.translation.y < -d1:
                            break
                    if pose.translation.x < -d and d1 < pose.translation.y < d2:
                        if -d2 < self.goal.translation.x < -d1 and d - 2 < self.goal.translation.y < d - 1:
                            break
        else:
            d1, d2, d = 4.6, 4.7, 2.0
            while 1:
                pose, vel, scan = self.env.reset()
                if d1 < max(abs(pose.translation.x), abs(pose.translation.y)) < d2:
                    break
            self.last_vel = vel
            # while 1:
            #     self.goal = self.env.random_free_pose()
            #     dis = (pose.inverse() * self.goal).translation.norm()
            #     if goal_min < dis < goal_radius and d1 < max(abs(self.goal.translation.x), abs(self.goal.translation.y)) < d2:
            #         break
            while 1:
                self.goal = self.env.random_free_pose()
                dis = (pose.inverse() * self.goal).translation.norm()
                if goal_min < dis < goal_radius:
                    if d1 < pose.translation.x < d2 and -d <= pose.translation.y <= d:
                        if d1 < self.goal.translation.x < d2 and -d - 2 < self.goal.translation.y < d + 2:
                            break
                    if -d2 < pose.translation.x < -d1 and -d <= pose.translation.y <= d:
                        if -d2 < self.goal.translation.x < -d1 and -d - 2 < self.goal.translation.y < d + 2:
                            break
                    if -d <= pose.translation.x <= d and -d2 < pose.translation.y < -d1:
                        if -d - 2 < self.goal.translation.x < d + 2 and -d2 < self.goal.translation.y < -d1:
                            break
                    if -d <= pose.translation.x <= d and d1 < pose.translation.y < d2:
                        if -d - 2 < self.goal.translation.x < d + 2 and d1 < self.goal.translation.y < d2:
                            break
                    if d1 < pose.translation.x < d2 and d < pose.translation.y or d < pose.translation.x and d1 < pose.translation.y < d2:
                        if d1 < self.goal.translation.x < d2 and d - 2 < self.goal.translation.y < d:
                            break
                        if d - 2 < self.goal.translation.x < d and d1 < self.goal.translation.y < d2:
                            break
                    if d1 < pose.translation.x < d2 and pose.translation.y < -d or d < pose.translation.x and -d2 < pose.translation.y < -d1:
                        if d1 < self.goal.translation.x < d2 and -d < self.goal.translation.y < -d + 2:
                            break
                        if d - 2 < self.goal.translation.x < d and -d2 < self.goal.translation.y < -d1:
                            break
                    if -d2 < pose.translation.x < -d1 and pose.translation.y < -d or pose.translation.x < -d and -d2 < pose.translation.y < -d1:
                        if -d2 < self.goal.translation.x < -d1 and -d < self.goal.translation.y < -d + 2:
                            break
                        if -d < self.goal.translation.x < -d + 2 and -d2 < self.goal.translation.y < -d1:
                            break
                    if -d2 < pose.translation.x < -d1 and d < pose.translation.y or pose.translation.x < -d and d1 < pose.translation.y < d2:
                        if -d2 < self.goal.translation.x < -d1 and d - 2 < self.goal.translation.y < d:
                            break
                        if -d < self.goal.translation.x < -d + 2 and d1 < self.goal.translation.y < d2:
                            break

        # # 导航
        # pose, vel, scan = self.env.reset()
        # self.last_stamp = scan["header"]["stamp"]
        # self.last_vel = vel

        # while 1:
        #     self.goal = self.env.random_free_pose()
        #     dis = (pose.inverse() * self.goal).translation.norm()
        #     if dis < GOAL_REACHED_DIST:  # already arrived
        #         continue
        #     if dis < goal_radius:
        #         break

        self.last_stamp = scan["header"]["stamp"]
        self.pub_goal(self.goal, self.last_stamp)
        print("reset", self.last_stamp)
        ranges = np.frombuffer(scan["ranges"], dtype="float32")

        v_state = []
        v_state[:] = ranges
        laser_state = [v_state]

        angle = pose.rotation.yaw
        goal2robot = pose.inverse() * self.goal
        distance = goal2robot.translation.norm()
        theta = math.atan2(goal2robot.translation.y, goal2robot.translation.x)

        robot_state = [distance, theta, 0, 0]
        state = np.append(laser_state, robot_state)
        self.dises = []
        return state

    def observe_collision(self, laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser <= COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    def get_reward(self, target, collision, min_laser, distance, theta, vel, t):
        # if target:
        #     return 100.0
        # elif collision:
        #     return -50 - distance * 10
        # else:
        #     r3 = lambda x: 1 - x if x < 1 else 0.0
        #     return (vel.translation.x - 0.3) - abs(vel.rotation.log().z) / 3 - r3(min_laser) - abs(theta) / 3
        if target:
            r1 = lambda o: min(o) - 4.65 if o else 0
            return 100 + r1(self.dises) * 200
        elif collision:
            return -50 - distance * 10
            #return -100
        else:
            r3 = lambda o: o - 0.4 if o < 0.4 else 0.0
            r4 = lambda o: o - 4.65 if o > 4 else 0.0
            self.dises.append(max(abs(t.x), abs(t.y)))
            return (vel.translation.x - 0.3) / 4 + r3(min_laser) / 10 + r4(max(abs(t.x), abs(t.y))) / 2 - abs(theta) / 10


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
