import numpy as np
from scipy import interpolate


def interpolate_path(path, T, dt):
    path_len = path.shape[0]
    path_num = path.shape[1]
    time = np.linspace(0, T, path_len)

    y = np.zeros(path.shape)
    for j in range(path_num):
        path_inter = interpolate.interp1d(time, path[:, j])
        y[:, j] = np.array([path_inter(i * dt) for i in range(path_len)])
    return y


def calc_derivatives(path, dt):
    yd = np.gradient(path, dt, axis=0)
    return yd
