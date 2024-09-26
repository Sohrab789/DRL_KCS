import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from tf_agents.environments import py_environment
from tf_agents.environments import tf_py_environment
from tf_agents.specs import array_spec
from tf_agents.trajectories import time_step as ts
from tf_agents.environments import wrappers
from scipy.integrate import solve_ivp
import os
from environment import ship_environment
import re

def wp_track(model_name, wind_flag=0, wind_speed=0,wind_dir=0,
             wave_flag=0, wave_height=0, wave_period=0, wave_dir=0,
             npoints=None, x_wp=None, y_wp=None, psi0=None,traj_str='',
             xdes=None, ydes=None,obs_x=None,obs_y=None,obsize=None):

    dir_list = os.listdir('trained_models/' + model_name)
    regex = re.compile(model_name + '_ep_')
    new_dir_list = list(filter(regex.match, dir_list))

    for policy_name in new_dir_list:

        env = wrappers.TimeLimit(ship_environment(train_test_flag=1,
                                                  wind_flag=wind_flag, wind_speed=wind_speed, wind_dir=wind_dir,
                                                  wave_flag=wave_flag, wave_height=wave_height,
                                                  wave_period=wave_period, wave_dir=wave_dir,
                                                  test_x_waypoints=x_wp,
                                                  test_y_waypoints=y_wp,
                                                  nwp=npoints,
                                                  obs_x=obs_x,obs_y=obs_y,obsize=obsize,
                                                  initial_obs_state=[1, 0, 0, x_wp[0], y_wp[0], psi0, 0]), duration=500)

        tf_env = tf_py_environment.TFPyEnvironment(env)

        train_step_counter = tf.Variable(0)

        # Reset the train step
        episodes = 1

        saved_policy = tf.compat.v2.saved_model.load('trained_models/'+ model_name + '/' + policy_name)

        for _ in range(episodes):

            time_step = tf_env.reset()
            # time = 0

            while not np.equal(time_step.step_type, 2):
                action_step = saved_policy.action(time_step)
                time_step = tf_env.step(action_step.action)

        plt.figure()
        X = env.x_traj
        Y = env.y_traj
        Psi = env.psi_traj
        x_ship = np.array([-0.5, -0.5, 0.25, 0.5, 0.25, -0.5, -0.5, 0.5, 0.25, 0, 0])
        y_ship = 16.1 / 230 * np.array([-1, 1, 1, 0, -1, -1, 0, 0, 1, 1, -1])

        plt.plot(X,Y,'y')

        # Waypoints
        x_wp = env.test_x_waypoints
        y_wp = env.test_y_waypoints

        plt.plot(xdes,ydes)

        plt.scatter(x_wp[0], y_wp[0], marker = 'o',c = 'm')

        obs_x,obs_y=env.obs_x,env.obs_y
        plt.plot(obs_x, obs_y, 'ro')

        plt.scatter(x_wp[1],y_wp[1],marker = 'o',c='g')

	
        # Plot ship geometry on path
        m = 9
        for i in range(1,m):
            m_indx = i * (len(X) // m)
            psi_new = Psi[m_indx]
            x_new_ship = X[m_indx] + x_ship * np.cos(psi_new) - y_ship * np.sin(psi_new)
            y_new_ship = Y[m_indx] + x_ship * np.sin(psi_new) + y_ship * np.cos(psi_new)
            plt.plot(x_new_ship, y_new_ship, 'saddlebrown',label=None)
            
        obs_x,obs_y=env.obs_x,env.obs_y
        plt.plot(obs_x, obs_y, 'ro', markersize=25)

        plt.xlabel("X/L")
        plt.ylabel("Y/L")
        plt.axis('equal')
        plt.legend(["Path Followed","Predefined Path","Start Waypoint","Obstacle","Destination Waypoint"],loc="lower left")
        ax = plt.gca()
        ax.invert_yaxis()
        plt.grid()
        if wind_flag == 0 and wave_flag == 0:
            plt.title(f'Obstacle size = {obsize}xShip Length')
            plt_str = f'trained_models/{model_name}/plots/{traj_str}_{obsize}_1_{policy_name}.png'
            plt.savefig(plt_str, dpi=600)
        elif wind_flag == 1:
            plt.title(f'V_w={wind_speed} \\beta_w={wind_dir}')
            plt_str = f'trained_models/{model_name}/plots/{traj_str}_{policy_name}_WS{wind_speed}_WD{wind_dir}.png'
            plt.savefig(plt_str, dpi=600)

