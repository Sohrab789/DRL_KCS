import numpy as np

def angle_between(p1, p2):   #Gives value between 0 to 360 in clockwise direction from p1 to p2
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return np.rad2deg((ang1 - ang2) % (2 * np.pi))

def ssa(angle):
    return (angle + np.pi)%(2*np.pi) - np.pi

def normalize(vec):
    return vec / np.linalg.norm(vec)

