import numpy as np
import hyperparams as params
import dqn_train
import time
import tensorflow as tf
from environment import ship_environment

max_batch_train = 3

for batch_train in range(0, max_batch_train):
    params.model_name = 'model_' + '{:03}'.format(batch_train + 81)

    if batch_train < max_batch_train:
        params.duration = 160
        params.initial_learning_rate = 0.0005
        params.decay_steps = 50000
        params.decay_rate = 0.5
        params.fc_layer_params = (256,256)
        params.discount_factor = 0.98
        params.target_update_tau = 0.01
        params.num_of_obs = 1
        params.target_update_period = 1
        params.replay_buffer_max_length = 100000
        params.num_parallel_calls = 2
        params.sample_batch_size = 128
        params.num_steps = 1
        params.prefetch = 3
        params.max_episodes = 8001
        params.epsilon_greedy_episodes = 5000
        params.random_seed = np.random.randint(100000)
        params.DQN_update_time_steps = 5
        params.DQN_policy_store_frequency = 1000
        params.DQN_loss_avg_interval = 100
        params.destination_reward = 20
        params.collision_reward = -100

        params.TCPA = 1
        params.DCPA = 1

    tf.keras.utils.set_random_seed(params.random_seed)
    tf.config.experimental.enable_op_determinism()
    dqn_train.dqn_train(params)

print(f'Training Completed Successfully')


