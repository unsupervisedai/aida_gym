import aida_env.aida_gym_env as e
import pybullet as p
import numpy as np


def test():
    env = e.AidaBulletEnv(render=True, on_rack=False)  #  use on_rack to remove gravity
    actions = -0.5 * np.ones(env.action_space.shape)
    for i in range(10):
        env.render()
        env.step(actions) # tak"e a random action
    action1 = -0.7 * np.ones(env.action_space.shape)
    action2 = -0.4 * np.ones(env.action_space.shape)
    for i in range(1000):
        env.render()
        if (i % 60 == 0):
            print("action 1")
            actions = action1
        elif (i % 60 == 30) :
            print("action 2")
            actions = action2
        env.step(actions) # tak"e a random action


def train():
    test()  # no difference
