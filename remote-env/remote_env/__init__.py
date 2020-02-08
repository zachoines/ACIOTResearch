from gym.envs.registration import register

register(
    id='remote-car-v0',
    entry_point='remote_env.envs:RemoteCar',
)