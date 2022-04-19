import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
from os import path

class LQR_Env(object):

    def __init__(self):

        self.viewer = None
        self.U = 28. # nominal speed, unit: m/s
        # Front cornering stiffness for one wheel.
        self.Ca1 = -61595. # unit: Newtons/rad
        # Rear cornering stiffness for one wheel.
        self.Ca3 = -52095. # unit: Newtons/rad
        self.Caf = 2*self.Ca1
        self.Car = 2*self.Ca3
        self.g = 10.0
        self.m = 1670. # vehicle mass
        self.Iz = 2100. # moment of inertia
        self.a = 0.99 # Distance from vehicle CG to front axle
        self.b = 1.7 # Distance from vehicle CG to rear axle
        self.mu = 0.8 # peak friction coefficient
        self.dt = 0.02
        self.max_torque = 2.
        self.max_speed = 8.0

        self.A =  np.array([[0., 1., 0., 0.],
                            [0., (self.Caf+self.Car)/(self.m*self.U), -(self.Caf+self.Car)/self.m, (self.a*self.Caf-self.b*self.Car)/(self.m*self.U)],
                            [0., 0., 0., 1.],
                            [0., (self.a*self.Caf-self.b*self.Car)/(self.Iz*self.U), -(self.a*self.Caf-self.b*self.Car)/(self.Iz), (self.a**2*self.Caf+self.b**2*self.Car)/(self.Iz*self.U)]])
        self.B1 = np.array([[0.],
                            [-self.Caf/self.m],
                            [0.],
                            [-self.a*self.Caf/self.Iz]])
        self.B2 = np.array([[0.],
                            [(self.a*self.Caf-self.b*self.Car)/self.m - self.U**2],
                            [0.],
                            [(self.a**2*self.Caf+self.b**2*self.Car)/(self.Iz)]])

        self.nx = 4
        self.nu = 1
        self.Q = np.zeros((self.nx, self.nx))
        self.Q[0, 0], self.Q[1, 1], self.Q[2, 2], self.Q[3, 3],  = 1., 5., 0.1, 0.5
        self.R = np.array([[0.1]])
        self.time = 0

        self.max_steering = 30/180*np.pi
        self.action_space = spaces.Box(low=-self.max_steering, high=self.max_steering, shape=(self.nu,))
        # observations are the four states
        xmax = np.array([2., 10., 60/180*np.pi, 5*60/180*np.pi])
        xmax = np.reshape(xmax,(self.nx,1))
        self.observation_space = spaces.Box(low=-xmax, high=xmax)

        self.seed()


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self,u):

        # e, edot, theta, thetadot = self.stat
        x = np.reshape(self.state,(self.nx,1))
        dt = self.dt
        curv = np.array([[self.np_random.uniform(low=-1/200, high=1/200)]])

        u = np.clip(u, -self.max_steering, self.max_steering)
        u = np.reshape(u,(self.nu,1))
        costs = np.matmul(np.matmul(x.T, self.Q),x) + np.matmul(np.matmul(u.T, self.R), u) - 1.7e3/25
        # costs = th**2 + .1*thdot**2 + .001*(u**2) - 1
        xdot = np.matmul(self.A, x) + np.matmul(self.B1, u) + np.matmul(self.B2, curv)
        new_x = x + xdot * dt

        self.state = new_x
        terminated = False

        if self.time > 200 or not self.observation_space.contains(self.state):
            terminated = True

        self.time += 1

        return self.get_obs(), -costs, terminated, {}

    def reset(self):
        high = np.array([0.01, 0.01, np.pi/180., np.pi/180.])
        self.state = self.np_random.uniform(low=-high, high=high)
        self.last_u = None
        self.time = 0

        return self.get_obs()

    def get_obs(self):
        return  np.reshape(self.state, (1, self.nx))[0]
