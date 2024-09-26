import numpy as np
import hyperparams as params

def obstacle(obs_x, obs_y, x, y, x_dot, y_dot, Uvec_hat, course_angle, vel_obs, ob_size, counter):

    R = []
    ob_x  = []
    ob_y = []
    vec_R  = []
    theta_t = []
    psi_ts = []
    DCPA = [] 
    TCPA  = []
    CR = []
    V_o = []
    V_r = []
    psi_r = []
    psi_os = []
    ang_vec5 = []
    vec5 = []
    vec5_hat = []
    obs_x_vel = []
    obs_y_vel = []
    alpha = params.TCPA
    beta = params.DCPA

    def angle_between(p1, p2):                                                              #Gives value between 0 to 360 in clockwise direction from p1 to p2
        ang1 = np.arctan2(*p1[::-1])
        ang2 = np.arctan2(*p2[::-1])
        return (ang1 - ang2) % (2 * np.pi)

    
    for i in range(0,np.size(ob_size)):
        
        vec5.append(np.array([obs_x[(2*i)+1] - obs_x[2*i] , obs_y[(2*i)+1] - obs_y[2*i]]))   # path line of the obstacle
        
        vec5_hat.append(vec5[-1] / np.linalg.norm(vec5[-1]))                                # Unit vector of motion direction of Obstacle
        
        ang_vec5.append(np.arctan2(vec5[i][1], vec5[i][0]))                                 #angle of the vec5 with x-axis(-pi,pi)

        ob_x.append(obs_x[2*i] + vel_obs[i] * np.cos(ang_vec5[i]) * counter)
        ob_y.append(obs_y[2*i] + vel_obs[i] * np.sin(ang_vec5[i]) * counter)               #Current point of obstacle

        R.append(np.sqrt((ob_y[i] - y) ** 2 + (ob_x[i] - x) ** 2))                          # Distance to obstacle from ship

        vec_R.append(np.array([ob_x[i] - x, ob_y[i] - y]))                                  # Vector joining the ship and obstacle

        theta_t.append(angle_between(Uvec_hat, vec_R[i]))                                   #angle between velocity of ship and line joining ship and obstacle

        psi_os.append(angle_between([0,1],Uvec_hat))                                        #angle between velocity of ship and y axis
        
        psi_ts.append(angle_between([0,1],vec5[i]))
        
        V_o.append([vel_obs[i]*np.sin(psi_ts[i]), vel_obs[i]*np.cos(psi_ts[i])])
        
        V_r.append(V_o[i] - np.array([x_dot,y_dot]))                                        #Relative velocity

        psi_r.append(angle_between([0,1],V_r[i]))
        
        DCPA.append(np.abs(R[i]*np.sin(psi_r[i]-psi_os[i]-theta_t[i]-np.pi)))               # Distance to closest point approach

        TCPA.append(R[i]*np.cos(psi_r[i]-psi_os[i]-theta_t[i]-np.pi) / np.linalg.norm(V_r)) # Time to closest point approach
        
        if TCPA[-1]>0:
            CR.append(np.exp(-(alpha*TCPA[-1]+beta*DCPA[-1])))
        else:
            CR.append(0)
        # CR.append(np.exp(-R[-1]))

            
    
    crit_index= CR.index(max(CR))                                                           #index of the critical object
    dist_obs_cr = R[crit_index]


    min_index = R.index(min(R))
    dist_obs_min = R[min_index]
    obs_size_min = ob_size[min_index]


    vec4  = np.array([x - ob_x[crit_index] , y - ob_y[crit_index]])                          # line joining obstacle's and ship's current locations
    vec4_hat = vec4 / np.linalg.norm(vec4)

    angle_btw54 = np.arccos(np.dot(vec5_hat[crit_index], vec4_hat))

    cross_btw_34 = np.sign(np.cross(Uvec_hat, vec4_hat))                                    # sign of sine value in cross product, gives -1 for obstacle in port, +1 for starboard


    obs_x_vel = vel_obs[crit_index] * np.cos(angle_btw54)                                   #Velocity of obstcale i in x direction
    obs_y_vel = np.abs(vel_obs[crit_index]* np.sin(angle_btw54)) * cross_btw_34             #Velocity of obstacle i in y direction

    psi_obs = np.arctan2(ob_y[crit_index]-y, ob_x[crit_index]-x)
    angle_obs = course_angle - psi_obs
    angle_obs = (angle_obs + np.pi) % (2*np.pi) - np.pi

    obs_size = ob_size[crit_index]
    
    obs_x_vel_cr = obs_x_vel
    obs_y_vel_cr = obs_y_vel

    return dist_obs_cr, angle_obs, obs_x_vel_cr, obs_y_vel_cr, obs_size, crit_index, dist_obs_min, obs_size_min 