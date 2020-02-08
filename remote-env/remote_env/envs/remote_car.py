import gym
import numpy as np
from gym import error, spaces, utils
from gym.utils import seeding


MAX_G = 16.0
MIN_G = -16.0
LOWER_REWARD_BOUND = -5
UPPER_REWARD_BOUND = 5

class RemoteCar(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(RemoteCar, self).__init__()
        
        self.reward_range = (LOWER_REWARD_BOUND, UPPER_REWARD_BOUND)
        # for image data
        # self.observation_space = spaces.Box(low=0, high=255, shape=(HEIGHT, WIDTH, N_CHANNELS), dtype=np.uint8) 
        # self.nested_observation_space = spaces.Dict({
        #     'sensors':  spaces.Dict({
        #         'position': spaces.Box(low=-100, high=100, shape=(3,)),
        #         'velocity': spaces.Box(low=-1, high=1, shape=(3,)),
        #         'front_cam': spaces.Tuple((
        #             spaces.Box(low=0, high=1, shape=(10, 10, 3)),
        #             spaces.Box(low=0, high=1, shape=(10, 10, 3))
        #         )),
        #         'rear_cam': spaces.Box(low=0, high=1, shape=(10, 10, 3)),
        #     })
        # })

        self.observation_space = spaces.Dict({
            "position": spaces.Box(low=0, high=float("inf"), shape=(1,), dtype=np.float32), 
            "acceleration": spaces.Box(low=MIN_G, high=MAX_G, shape=(3,), dtype=np.float32)
        })
        
    def step(self, action):
        pass
    def render(self, mode='human'):
        pass
    def close(self):
        pass