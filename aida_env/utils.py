import aida_env.aida_gym_env as e
import pybullet as p
import numpy as np


def test_agent_on_env(agent):
    env = e.AidaBulletEnv(render=True, on_rack=False)  #  use on_rack to remove gravity
    for i in range(2000):
        env.step(agent.get_actions())

    reward = np.linalg.norm(env.aida.GetBasePosition()[0:2])
    p.disconnect()
    return reward

