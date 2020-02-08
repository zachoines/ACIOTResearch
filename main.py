import tensorflow as tf
import numpy as np

from PPO2.Train import Train
from PPO2.Test import Test
from PPO2.Config import config_pacman as config

from smbus2 import SMBus
import numpy as np
import time

# Open i2c bus 1 and read one byte from address 80, offset 0
def toFloat(byteArray):
    data_bytes = np.array(byteArray, dtype=np.uint8)
    return data_bytes.view(dtype=np.float32)

def split_list(a_list):
    half = len(a_list)//2
    return a_list[:half], a_list[half:]

def getSensorReadings():
    with SMBus(1) as bus:
        bus.write_byte(9, 1)
        a, b = split_list(bus.read_i2c_block_data(9, 0, 8))
        c = toFloat(a)
        d = toFloat(b)
        print(c)
        print(d)


def main():
    for i in range(1000):
        getSensorReadings()  
        time.sleep(1)

def PPO_Start():
    tf.keras.backend.set_floatx('float64')
    np.random.seed(42)
    tf.random.set_seed(42)

    train_session = Train(config)

    if (train_session.start()):
        Test(config)

if __name__ == '__main__':
    main()

    
    
    

