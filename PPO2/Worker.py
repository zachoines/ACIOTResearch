import time, random
from threading import Thread
import numpy as np
import tensorflow as tf

class WorkerThread(Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs={}, Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs)
        self._return = None
        self._target = target
        self._args = args
        self._kwargs = kwargs

    def run(self):
        if self._target is not None:
            self._return = self._target(*self._args, **self._kwargs)
    def join(self, *args):
        Thread.join(self, *args)
        return self._return

# Class to represent a worker in an environment. Call run() to generate a batch. 
class Worker():

    def __init__(self, network, env, batch_size = 32, render = True, exploration="Epsilon_greedy"):

        # Epsisode collected variables
        self._batch_buffer = []
        self._done = False
        self._render = render
        self.total_steps = 0
        self.network = network
        self.env = env
        self.batch_size = batch_size 
        self.s = self.env.reset()
        self.NUM_ACTIONS = env.action_space.n
        self.NONE_STATE = np.zeros(self.env.observation_space.shape)
        self.exploration = exploration
        
        
    # Reset worker and evironment variables in preperation for a new epoc
    def reset(self):
        self._batch_buffer = []
        self._done = False
        self.s = self.env.reset()


    # Generate an batch worth of observations. Return nothing.
    def run(self, keep_prob):
        batch = []
        while (len(batch) < self.batch_size):

            # Make a prediction and take a step if the epoc is not done
            if not self._done:
                self.total_steps += 1
                [logits], [action_dist], value = self.network.step(self.s, keep_p=keep_prob)
                
                action = self.action_select(logits, temperature=(1 - keep_prob))
                s_t, reward, d, stuff = self.env.step(action)
                self._done = d

                batch.append((self.s , s_t , reward, value, action, d, logits.tolist()))
                self.s = s_t

                # render the env
                if (self._render):
                    self.env.render()

            else:
                self._done = False
                self.s = self.env.reset()

        if batch == None or len(batch) < self.batch_size:
            print('ERROR: There was an issue with episode run!')
            exit(1)
        return batch

    

    # Get all the batches in this epoc
    def get_batches(self):
        return self._batch_buffer
            

    # Boltzmann Softmax style action selection
    def action_select(self, dist, exploration="", temperature=1.0, epsilon=.2):
        
        #  / ((temperature) * 10)
        if exploration == "boltzmann":        
            
            dist = tf.nn.softmax(dist / ((temperature) * 4 + 1)).numpy()

            [probas] = np.random.multinomial(1, dist, 1)
            a = np.argmax(probas)
            return a

        
        # Use with large dropout annealed over time
        elif exploration == "bayesian":
            
            return np.argmax(dist)
        
        elif exploration == "Epsilon_greedy":
            
            # Scale epsilon down to near 0.0 by multiplying .9 ... .1
            if random.random() < (epsilon * temperature):
                
                return random.randint(0, self.NUM_ACTIONS-1)

            else:
                dist = tf.nn.softmax(dist).numpy() 
                a = np.argmax(dist)
                return a

        # or use the Gumbel-Max Trick
        else:
            # Use this if you want to define a decayed temp to increase uncertainty

            # def sample_gumbel(shape, eps=1e-20): 
            #     U = tf.random.uniform(shape,minval=0,maxval=1)
            #     return -tf.math.log(-tf.math.log(U + eps) + eps)

            # y = dist + sample_gumbel(tf.shape(dist))
            # return tf.argmax(tf.nn.softmax( y / ( temperature * 4.0) + 1.0)).numpy()
            
            noise = tf.random.uniform(dist.shape)
            return tf.argmax(dist - tf.math.log(-tf.math.log(noise))).numpy()

