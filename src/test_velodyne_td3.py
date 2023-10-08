import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from simulate_env import GazeboEnv, data_path


class Actor(nn.Module):

    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()

        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a


# TD3 network
class TD3(object):

    def __init__(self, state_dim, action_dim):
        # Initialize the Actor network
        self.actor = Actor(state_dim, action_dim).to(device)

    def get_action(self, state):
        # Function to get the action from the actor
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def load(self, filename, directory):
        # Function to load network parameters
        self.actor.load_state_dict(torch.load("%s/%s_actor.pth" % (directory, filename)))


# Set the parameters for the implementation
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
seed = 0  # Random seed number
max_ep = 500  # maximum number of steps per episode
file_name = "TD3_velodyne"  # name of the file to load the policy from

# Create the testing environment
environment_dim = 36
robot_dim = 4
env = GazeboEnv(data_path + "/map", environment_dim)
torch.manual_seed(seed)
np.random.seed(seed)
state_dim = environment_dim + robot_dim
action_dim = 2

# Create the network
network = TD3(state_dim, action_dim)
try:
    network.load(file_name, "./pytorch_models")
except:
    raise ValueError("Could not load the stored model parameters")

done = False
episode_timesteps = 0
goal_min = 2.0
goal_radius = 7.0
state = env.reset(goal_min=goal_min, goal_radius=goal_radius, eval=True)

# Begin the testing loop
while True:
    action = network.get_action(np.array(state))

    # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
    a_in = [action[0], action[1]]
    next_state, reward, done, target = env.step(a_in)
    time.sleep(0.02)
    done = 1 if episode_timesteps + 1 == max_ep else int(done)

    # On termination of episode
    if done:
        avg_dis = 4.7 - np.mean(env.dises)
        with open("./results/" + file_name + ".csv", "a+") as f:
            f.write(str(avg_dis) + "\n")
        print("..............................................")
        print("Average Distance: %f" % (avg_dis))
        print("..............................................")
        state = env.reset(goal_min=goal_min, goal_radius=goal_radius, eval=True)
        time.sleep(0.02)
        done = False
        episode_timesteps = 0
    else:
        state = next_state
        episode_timesteps += 1
