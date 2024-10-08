import numpy as np
import tensorflow as tf
from tf_agents.environments import py_environment
from tf_agents.environments import tf_py_environment
from tf_agents.specs import array_spec
from tf_agents.utils import common
from tf_agents.trajectories import time_step as ts
from scipy.integrate import solve_ivp
import hyperparams as params
import kcs
import obstacle


class ship_environment(py_environment.PyEnvironment):

    def __init__(self, train_test_flag=0,
                 wind_flag=0,
                 wind_speed=0,
                 wind_dir=0,
                 wave_flag=0,
                 wave_height=0,
                 wave_period=0,
                 wave_dir=0,
                 test_x_waypoints=None,
                 test_y_waypoints=None,
                 nwp=None,
                 ob_size = None,
                 vel_obs = None,
                 initial_obs_state=None):

        self._action_spec = array_spec.BoundedArraySpec(shape=(), dtype=np.int64, minimum=0, maximum=4,name='action')
        self._observation_spec = array_spec.BoundedArraySpec(shape=(9,), dtype=np.float32, minimum=[-20, -np.pi, 0, -1, 0, -np.pi,-0.5, -0.5, 0],
                                                             maximum=[20, np.pi, 50, 1, 100, np.pi, 0.5, 0.5, 1], name='observation')
                    #cross track error; course angle error; dist to dest; yaw rate; dist to obstacle; angle to obs; speed towards ship; speed perp. to ship; size of obs

        self.episode_ended = False
        self.counter = 0
        self.x_goal = 0
        self.y_goal = 0
        self.distance = 0
        self.obs_state = [1, 0, 0, 0, 0, 0, 0]
        self.pos_x_obs = []
        self.pos_y_obs = []

        self.train_test_flag = train_test_flag
        self.wind_flag = wind_flag
        self.wind_speed = wind_speed
        self.wind_dir = wind_dir

        self.wave_flag = wave_flag
        self.wave_height = wave_height
        self.wave_period = wave_period
        self.wave_dir = wave_dir

        # Test waypoints
        self.test_x_waypoints = test_x_waypoints
        self.test_y_waypoints = test_y_waypoints
        self.nwp = nwp
        self.wp_counter = 1
        self.initial_obs_state = initial_obs_state

        # Trajectories of state variables
        # Only stored in testing
        self.x_traj = []
        self.y_traj = []
        self.psi_traj = []
        self.u_traj = []
        self.v_traj = []
        self.r_traj = []
        self.delta_traj = []
        self.crit_index_traj = []

        # Trajectories of observation states
        # Only stored in testing
        self.cross_trk_err_traj = []
        self.course_ang_err_traj =[]
        self.distance_traj = []

        # Trajectories of action
        # Only stored in testing
        self.action_traj = []

        # Reward in trajectory
        # Only stored in testing
        self.total_reward = []
        self.reward_01 = []    # Cross track error
        self.reward_02 = []    # Course angle error
        self.reward_03 = []    # Distance to goal

    def action_spec(self):
        return self._action_spec

    def observation_spec(self):
        return self._observation_spec

    

    def _step(self, action_no):

        # Action selection
        action_set = [-35*np.pi/180,-20*np.pi/180,0,20*np.pi/180,35*np.pi/180]
        delta_c = action_set[action_no]
        if self.train_test_flag == 1:
            self.action_traj.append(delta_c)

        tspan = (0, 0.3)
        yinit = self.obs_state

        sol = solve_ivp(lambda t,v: kcs.KCS_ode(t, v, delta_c, wind_flag=self.wind_flag,
                                                wind_speed=self.wind_speed, wind_dir=self.wind_dir,
                                                wave_flag=self.wave_flag, wave_height=self.wave_height,
                                                wave_period=self.wave_period, wave_dir=self.wave_dir),

                        tspan, yinit, t_eval=tspan, dense_output=True)

        # sol.y outputs 0)surge vel 1) sway vel 2) yaw vel 3) x_coord 4) y_coord 5) psi 6) delta

        u = sol.y[0][-1]
        v = sol.y[1][-1]
        r = sol.y[2][-1]
        x = sol.y[3][-1]
        y = sol.y[4][-1]
        psi_rad = sol.y[5][-1]  # psi
        psi = psi_rad % (2 * np.pi)
        delta = sol.y[6][-1]

        self.obs_state[0] = sol.y[0][-1]    # Surge velocity
        self.obs_state[1] = sol.y[1][-1]    # Sway velocity
        self.obs_state[2] = sol.y[2][-1]    # Yaw velocity
        self.obs_state[3] = sol.y[3][-1]    # X coordinate
        self.obs_state[4] = sol.y[4][-1]    # Y coordinate
        self.obs_state[5] = psi             # Heading angle
        self.obs_state[6] = sol.y[6][-1]    # Actual rudder angle

        if self.train_test_flag == 1:
            self.x_traj.append(x)
            self.y_traj.append(y)
            self.psi_traj.append(psi_rad)
            self.u_traj.append(u)
            self.v_traj.append(v)
            self.r_traj.append(r)
            self.delta_traj.append(delta)

        x_init = 0
        y_init = 0
        x_goal = self.x_goal
        y_goal = self.y_goal

        if self.train_test_flag == 1:
            x_init = self.test_x_waypoints[self.wp_counter-1]
            y_init = self.test_y_waypoints[self.wp_counter-1]
            x_goal = self.test_x_waypoints[self.wp_counter]
            y_goal = self.test_y_waypoints[self.wp_counter]

        # DISTANCE TO GOAL
        distance = ((x - x_goal) ** 2 + (y - y_goal) ** 2) ** 0.5
        self.distance = distance
        if self.train_test_flag == 1:
            self.distance_traj.append(distance)

        # CROSS TRACK ERROR
        vec1 = np.array([x_goal - x_init, y_goal - y_init])
        vec2 = np.array([x_goal - x, y_goal - y])
        vec1_hat = vec1 / np.linalg.norm(vec1)
        cross_track_error = np.cross(vec2, vec1_hat)

        if self.train_test_flag == 1:
            self.cross_trk_err_traj.append(cross_track_error)

        # COURSE ANGLE ERROR
        x_dot = u * np.cos(psi) - v * np.sin(psi)
        y_dot = u * np.sin(psi) + v * np.cos(psi)

        Uvec = np.array([x_dot, y_dot])
        Uvec_hat = Uvec / np.linalg.norm(Uvec)
        vec2_hat = vec2 / np.linalg.norm(vec2)

        course_angle = np.arctan2(Uvec[1], Uvec[0])
        psi_vec2 = np.arctan2(vec2[1], vec2[0])

        course_angle_err = course_angle - psi_vec2
        course_angle_err = (course_angle_err + np.pi) % (2 * np.pi) - np.pi

        if self.train_test_flag == 1:
            self.course_ang_err_traj.append(course_angle_err)

        
        dist_obs_cr,angle_obs,obs_x_vel,obs_y_vel,ob_size, crit_index,dist_obs_min, obs_size_min = obstacle.obstacle(self.obs_x, self.obs_y, x, y, x_dot, y_dot, Uvec_hat,
                                                                        course_angle, self.vel_obs, self.ob_size, self.counter)
        self.counter += 1  

        # print(dist_obs,angle_obs,obs_x_vel,obs_y_vel,ob_size)

        # REWARD FUNCTIONS
        R1 = -0.5 * abs(cross_track_error)
        R2 = -0.3 * abs(course_angle_err)
        R3 = -0.2 * distance
        reward = R1 + R2 + R3

        if self.train_test_flag == 1:
            self.reward_01.append(R1)
            self.reward_02.append(R2)
            self.reward_03.append(R3)
            self.total_reward.append(reward)
            self.crit_index_traj.append(crit_index)

        observation = [cross_track_error, course_angle_err, distance, r, dist_obs_cr, angle_obs, obs_x_vel, obs_y_vel,ob_size]
                # cross track error, course angle error, distance, yaw rate, distance to obstacle, angle to obstacle,
                #                               velocity along the ship, velocity perpendicular to ship, size of ship

       
        # DESTINATION CHECK
        if abs(distance) <= 0.5:
            if self.train_test_flag == 1:
                print(f"Destination {self.wp_counter} reached")
                if self.wp_counter == self.nwp - 1:
                    return ts.termination(np.array(observation, dtype=np.float32), reward)
                self.wp_counter += 1

            else:
                reward = params.destination_reward
                self.episode_ended = True
                print('Destination reached')
                return ts.termination(np.array(observation, dtype=np.float32), reward)

        # TERMINATION CHECK
        angle_btw23 = np.arccos(np.dot(vec2_hat, Uvec_hat))
        angle_btw12 = np.arccos(np.dot(vec1_hat, vec2_hat))
        if angle_btw12 > np.pi / 2 and angle_btw23 > np.pi / 2:
            self.episode_ended = True
            if self.train_test_flag == 1:
                print(f"Destination {self.wp_counter} missed")
            return ts.termination(np.array(observation, dtype=np.float32), reward)
            
        if (dist_obs_min - obs_size_min )<=0.5:
            self.episode_ended=True
            reward=params.collision_reward
            print("Collided")
            return ts.termination(np.array(observation, dtype=np.float32), reward)

        else:
            return ts.transition(np.array(observation, dtype=np.float32),reward)

    def _reset(self):

        # print("Next episode")

        if self.train_test_flag == 0:
            self.obs_state = [1, 0, 0, 0, 0, 0, 0]
            self.obs_x = []
            self.obs_y = []
            self.DCPA = []
            self.TCPA = []
            self.vel_obs = np.random.random_sample(size = params.num_of_obs)*0.5
            print(f'params.num_of_obs ={params.num_of_obs} ')
            print(f'self.vel_obs={self.vel_obs}')
            self.ob_size = np.random.random_sample(size = params.num_of_obs)
            print(f'self.ob_size={self.ob_size}')
            self.counter = 0

            radius = np.random.randint(8, 18)
            random_theta = 2 * np.pi * np.random.random()

            self.x_goal = radius * np.cos(random_theta)
            self.y_goal = radius * np.sin(random_theta)
		
            self.episode_ended = False

            print("GOAL COORD", self.x_goal, self.y_goal)
            x_goal = self.x_goal
            y_goal = self.y_goal
            
            
            for i in range(np.size(self.ob_size)):
                random_angle1 = 2 * np.pi * np.random.random()
                random_angle2 = 2 * np.pi * np.random.random()
                dist_obs = 15*np.random.random() + 5
                
                self.obs_x.append(dist_obs * np.cos(random_angle1))
                self.obs_x.append(dist_obs * np.cos(random_angle2))
                self.obs_y.append(dist_obs * np.sin(random_angle1))
                self.obs_y.append(dist_obs * np.sin(random_angle2))
               
            x_dot = 1
            y_dot = 0

            vec2 = np.array([x_goal, y_goal])
            Uvec = np.array([x_dot, y_dot])
            Uvec_hat = Uvec / np.linalg.norm(Uvec)

            course_angle = np.arctan2(Uvec[1], Uvec[0])
            psi_vec2 = np.arctan2(vec2[1], vec2[0])
            course_angle_err = course_angle - psi_vec2
            course_angle_err = (course_angle_err + np.pi) % (2 * np.pi) - np.pi

            obs_x_vel = 0
            obs_y_vel = 0

        
            dist_obs,angle_obs,obs_x_vel,obs_y_vel,ob_size,crit_index, dist_obs_min, obs_size_min  = obstacle.obstacle(self.obs_x, self.obs_y, 0, 0, x_dot, y_dot, Uvec_hat,
                                                                                 course_angle, self.vel_obs, self.ob_size, self.counter)
            
            
            self.counter += 1

            observation = np.array([0, course_angle_err, radius, 0,dist_obs, angle_obs, obs_x_vel, obs_y_vel, ob_size], dtype=np.float32)

        else:
            self.obs_state = self.initial_obs_state
            self.obs_x = []
            self.obs_y = []
            self.DCPA = []
            self.TCPA = []
            self.vel_obs = np.random.random_sample(size = 5)*0.25
            self.ob_size = np.random.random_sample(size = 5)
            self.counter = 0

            self.x_goal = self.test_x_waypoints[1]
            self.y_goal = self.test_y_waypoints[1]

            x = self.test_x_waypoints[0]
            y = self.test_y_waypoints[0]

            print("GOAL COORD", self.x_goal, self.y_goal)
            x_goal = self.x_goal
            y_goal = self.y_goal

            u = self.initial_obs_state[0]
            v = self.initial_obs_state[1]
            psi = self.initial_obs_state[5]
            x_dot = u * np.cos(psi) - v * np.sin(psi)
            y_dot = u * np.sin(psi) + v * np.cos(psi)
		
            vec2 = np.array([x_goal, y_goal])
            Uvec = np.array([x_dot, y_dot])
            Uvec_hat = Uvec / np.linalg.norm(Uvec)
            
            
            for i in range(np.size(self.ob_size)):
                random_angle1 = 2 * np.pi * np.random.random()
                random_angle2 = 2 * np.pi * np.random.random()
                dist_obs = 5*np.random.random()+5
                
                self.obs_x.append(dist_obs * np.cos(random_angle1))
                self.obs_x.append(dist_obs * np.cos(random_angle2))
                self.obs_y.append(dist_obs * np.sin(random_angle1))
                self.obs_y.append(dist_obs * np.sin(random_angle2))

                # self.obs_x.append(10)
                # self.obs_x.append(0)
                # self.obs_y.append(0)
                # self.obs_y.append(0)

            
            course_angle = np.arctan2(Uvec[1], Uvec[0])
            psi_vec2 = np.arctan2(vec2[1], vec2[0])
            course_angle_err = course_angle - psi_vec2
            course_angle_err = (course_angle_err + np.pi) % (2 * np.pi) - np.pi
            

            dist_to_goal = np.sqrt((x_goal - x) ** 2 + (y_goal - y) ** 2)

            dist_obs,angle_obs,obs_x_vel,obs_y_vel,ob_size, crit_index, dist_obs_min, obs_size_min  = obstacle.obstacle(self.obs_x, self.obs_y, 0, 0, x_dot, y_dot, Uvec_hat,
                                                                                 course_angle, self.vel_obs, self.ob_size, self.counter)
            self.counter += 1
            self.crit_index_traj.append(crit_index)
            observation = np.array([0, course_angle_err, dist_to_goal, 0, dist_obs, angle_obs, obs_x_vel, obs_y_vel, ob_size], dtype=np.float32)


        return ts.restart(observation)


# ENDS HERE