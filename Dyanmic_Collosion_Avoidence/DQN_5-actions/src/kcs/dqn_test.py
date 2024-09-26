import numpy as np
import test_wp_track as wpt

model_name = 'model_110'

def single_wp(len=15):
    npoints = 2
    psi0 = 0
    
    x_wp = np.array([-5, len-5])
    y_wp = np.array([0, 0])


    xdes = x_wp
    ydes = y_wp
    return npoints, x_wp, y_wp, psi0, xdes, ydes

# No wind condition

def square_wp():
    npoints = 6
    x_wp = [-15,-5,5,5,-5,-5]
    y_wp = [5,5,5,-5,-5,5]
    psi0 = np.arctan2(y_wp[1]-y_wp[0],x_wp[1]-x_wp[0])


    xdes = x_wp
    ydes = y_wp
    return npoints, x_wp, y_wp, psi0, xdes, ydes


def star_wp():
    npoints = 6
    x_wp = [0,5.878,-9.511,9.511,-5.878,0]
    y_wp = [10,-8.09,3.09,3.09,-8.09,10]
    psi0 = np.arctan2(y_wp[1]-y_wp[0],x_wp[1]-x_wp[0])

    xdes = x_wp
    ydes = y_wp
    return npoints, x_wp, y_wp, psi0, xdes, ydes

# Single waypoint tracking in four quadrants
n, xwp, ywp, psi0, xdes, ydes = star_wp()
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='star')

n, xwp, ywp, psi0, xdes, ydes = single_wp()
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='quadrant_01')

n, xwp, ywp, psi0, xdes, ydes = square_wp()
wpt.wp_track(model_name, wind_flag=0, wind_speed=0, wind_dir=0, npoints=n, x_wp=xwp, y_wp=ywp, psi0=psi0, traj_str='square')
