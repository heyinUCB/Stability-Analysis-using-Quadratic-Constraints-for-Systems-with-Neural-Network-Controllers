import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
from os import path

class LQR_Env(object):

    def __init__(self):

        self.viewer = None
        self.g = 10.0
        self.m = 0.15
        self.l = 0.5
        self.mu = 0.05
        self.dt = 0.02
        # self.max_torque = self.m*self.g*self.l*np.sin(np.pi/3)
        self.max_torque = 2
        self.max_speed = 8.0

        self.nx = 2
        self.nu = 1

        self.time = 0

        self.action_space = spaces.Box(low=-self.max_torque, high=self.max_torque, shape=(self.nu,))
        # observations are the two states
        xmax = np.array([np.pi/2, self.max_speed])
        self.observation_space = spaces.Box(low=-xmax, high=xmax)

        self.seed()


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self,u):
        th, thdot = self.state

        g = self.g
        m = self.m
        l = self.l
        mu = self.mu
        dt = self.dt

        u = np.clip(u, -self.max_torque, self.max_torque)[0]
        costs = th**2 + .1*thdot**2 + .001*(u**2) - 1
        # costs = 5*th**2 + .1*thdot**2 + .001*(u**2) - 4

        newthdot = thdot + (g/l*np.sin(th) - mu/(m*l**2)*thdot + 1/(m*l**2)*u) * dt
        newth = th + thdot * dt

        self.state = np.array([newth, newthdot])

        terminated = False
        if self.time > 200 or not self.observation_space.contains(self.state):
            terminated = True

        self.time += 1

        return self.get_obs(), -costs, terminated, {}

    def reset(self):
        high = np.array([np.pi/30, np.pi/20])
        self.state = self.np_random.uniform(low=-high, high=high)
        self.last_u = None
        self.time = 0

        return self.get_obs()

    def get_obs(self):
        return  self.state
