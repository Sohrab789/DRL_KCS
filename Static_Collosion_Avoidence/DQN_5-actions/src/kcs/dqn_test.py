import numpy as np
import test_wp_track as wpt

model_name = 'model_211'

def single_wp(quadrant=1, len=10):
    npoints = 2
    psi0 = 0

    if quadrant == 1:
        x_wp = np.array([0, len])
        y_wp = np.array([0, len])
        #obs_x,obs_y = np.array([5]),np.array([5])
        obs_x,obs_y = np.array([0]),np.array([6])
    elif quadrant == 2:
        x_wp = np.array([0, -len])
        y_wp = np.array([0, len])
        #obs_x,obs_y = np.array([-5]),np.array([5])
        obs_x,obs_y = np.array([-4]),np.array([6])
    elif quadrant == 3:
        x_wp = np.array([0, -len])
        y_wp = np.array([0, -len])
        #obs_x,obs_y = np.array([-5]),np.array([-5])
        obs_x,obs_y = np.array([-4]),np.array([-6])
    elif quadrant == 4:
        x_wp = np.array([0, len])
        y_wp = np.array([0, -len])
        #obs_x,obs_y = np.array([5]),np.array([-5])
        obs_x,obs_y = np.array([6]),np.array([-4])
    elif quadrant == 5:
        x_wp = np.array([0, len+5])
        y_wp = np.array([0, 0])
        obs_x = np.array([len])
        obs_y = np.array([4])

    xdes = x_wp
    ydes = y_wp
    return npoints, x_wp, y_wp, psi0, xdes, ydes,obs_x,obs_y

# No wind condition

# Single waypoint tracking with single static obstacle of size 0.25L in four quadrants
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=1)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_01', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.25)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=2)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_02', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.25)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=3)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_03', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.25)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=4)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_04', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.25)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=5)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='Straight Line', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.25)



# Single waypoint tracking with single static obstacle of size 0.5L in four quadrants
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y= single_wp(quadrant=1)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_01', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.5)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=2)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_02', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.5)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=3)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_03', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.5)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=4)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_04', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.5)
n, xwp, ywp, psi0, xdes, ydes,obs_x,obs_y = single_wp(quadrant=5)
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='Straight Line', xdes=xdes, ydes=ydes,obs_x=obs_x,obs_y=obs_y,obsize=0.5)
