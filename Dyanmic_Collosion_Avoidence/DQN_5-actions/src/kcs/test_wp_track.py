import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from tf_agents.environments import tf_py_environment
from tf_agents.environments import wrappers
import os
from environment import ship_environment
import re
from utils import normalize
from matplotlib.animation import FuncAnimation
from matplotlib import animation


def wp_track(model_name, wind_flag=0, wind_speed=0,wind_dir=0,
             wave_flag=0, wave_height=0, wave_period=0, wave_dir=0,
             npoints=None, x_wp=None, y_wp=None, psi0=None,traj_str=''):

    dir_list = os.listdir('trained_models/' + model_name)
    regex = re.compile(model_name + '_ep_')
    new_dir_list = list(filter(regex.match, dir_list))

    for policy_name in new_dir_list:
        print('model loaded')
        env = wrappers.TimeLimit(ship_environment(train_test_flag=1,
                                                  wind_flag=wind_flag, wind_speed=wind_speed, wind_dir=wind_dir,
                                                  wave_flag=wave_flag, wave_height=wave_height,
                                                  wave_period=wave_period, wave_dir=wave_dir,
                                                  test_x_waypoints=x_wp,
                                                  test_y_waypoints=y_wp,
                                                  nwp=npoints,
                                                  initial_obs_state=[1, 0, 0, x_wp[0], y_wp[0], psi0, 0]), duration=10000)

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

        fig, ax = plt.subplots()
        
        ax.plot(x_wp, y_wp, 'k--')
        ax.plot(x_wp[0], y_wp[0], 'mo', markersize=10)
        
        # print(x_wp)       
        
        X = env.x_traj
        Y = env.y_traj

        obs_vel_arr = env.vel_obs
        obs_x_arr = env.obs_x
        print(f'obs_x_arr ={obs_x_arr} ')
        obs_y_arr = env.obs_y
        obs_size_arr = env.ob_size
        print(f'obs_size_arr ={obs_size_arr} ')

        ob_x_traj_list = np.zeros([len(obs_size_arr),len(X)])
        ob_y_traj_list = np.zeros([len(obs_size_arr),len(X)])

        scatter = []
        scatter = ax.scatter(ob_x_traj_list[:,0],ob_y_traj_list[:,0],marker = 'o', c = 'r', s = 10+obs_size_arr*100)
        for i in range(len(x_wp)-1):  
            plt.scatter(x_wp[i+1], y_wp[i+1],c= "lime",s=100)
        shiptraj_line, = ax.plot(X[0],Y[0])
        

        Psi = env.psi_traj
        x_ship = np.array([-0.5, -0.5, 0.25, 0.5, 0.25, -0.5, -0.5, 0.5, 0.25, 0, 0])
        y_ship = 16.1 / 230 * np.array([-1, 1, 1, 0, -1, -1, 0, 0, 1, 1, -1])

        psi_new = Psi[0]
        x_new_ship = X[0] + x_ship * np.cos(psi_new) - y_ship * np.sin(psi_new)
        y_new_ship = Y[0] + x_ship * np.sin(psi_new) + y_ship * np.cos(psi_new)

        shipshape_line, = ax.plot(x_new_ship, y_new_ship,'saddlebrown',label=None)

        
        traj_count = np.linspace(1,len(X),len(X))
        for i in range(len(obs_size_arr)):
            
            vec5_hat = normalize(np.array([obs_x_arr[(2*i)+1] - obs_x_arr[2*i], obs_y_arr[(2*i)+1] - obs_y_arr[2*i]]))  # Unit vector of motion direction of Obstacle
            ang_vec5 = np.arctan2(vec5_hat[1], vec5_hat[0])
            ob_x_traj_list[i,:] = obs_x_arr[2*i] + obs_vel_arr[i] * np.cos(ang_vec5) * traj_count
            ob_y_traj_list[i,:] = obs_y_arr[2*i] + obs_vel_arr[i] * np.sin(ang_vec5) * traj_count

        
        # scatter_crit = ax.scatter(ob_x_traj_list[env.crit_index_traj[0],0], ob_y_traj_list[env.crit_index_traj[0], 0], marker = 'o', c = 'b', s = 10 + obs_size_arr[env.crit_index_traj[0]]*25)


        plt.figure()

        ax.set_xlabel("X/L")
        ax.set_ylabel("Y/L")
        ax.axis('equal')
        ax.legend(["Predefined Path","Start Waypoint","Obstacle", "Destination Waypoint"],loc="lower left")
        ax.invert_yaxis()
        plt.grid()
        ax.set_xlim(-10, 15)
        ax.set_ylim(-10, 10)
        def animate(i):
            shiptraj_line.set_data(X[0:i+1],Y[0:i+1])

            psi_new = Psi[i]
            x_new_ship = X[i] + x_ship * np.cos(psi_new) - y_ship * np.sin(psi_new)
            y_new_ship = Y[i] + x_ship * np.sin(psi_new) + y_ship * np.cos(psi_new)
            shipshape_line.set_data(x_new_ship,y_new_ship)

            obs_data = np.transpose(np.vstack((ob_x_traj_list[:,i],ob_y_traj_list[:,i])))
            scatter.set_offsets(obs_data)

            crit_index = env.crit_index_traj[i]
            # obs_data_crit = [ob_x_traj_list[crit_index,i],ob_y_traj_list[crit_index,i]]
            # scatter_crit.set_offsets(obs_data_crit)

            return shiptraj_line, shipshape_line, scatter   #, scatter_crit

        ani = FuncAnimation(fig, animate, interval=100, blit=False, frames=len(traj_count))
        # FFwriter = animation.FFMpegWriter()

        ani.save(f'trained_models/{model_name}/plots/{traj_str}_{policy_name}.gif', writer='imagemagick')