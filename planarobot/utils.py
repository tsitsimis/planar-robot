import numpy as np
from scipy import interpolate


def interpolate_path(path, T, dt):
    path_len = path.shape[0]
    time = np.linspace(0, T, path_len)

    path_inter = interpolate.interp1d(time, path)
    y = np.array([path_inter(i * dt) for i in range(path_len)])
    return y


def calc_derivatives(path, dt):
    yd = np.gradient(path, dt)
    return yd
