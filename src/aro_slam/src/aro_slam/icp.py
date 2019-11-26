from __future__ import absolute_import, division, print_function
import numpy as np
import numpy.ma as ma
import scipy.spatial as spatial
import math
import cv2
from sklearn.neighbors import NearestNeighbors
import random
import matplotlib.pyplot as plt


def icp(x, y, num_iters=10, inlier_ratio=1.0, inlier_dist_mult=1.0, tol=1e-3, y_index=None, R=None, t=None):
    """Iterative closest point algorithm, minimizing sum of squares of point to point distances.

    :param x: Points to align, D-by-N matrix.
    :param y: Reference points to align to, such that y[:, j] ~ R * x[:, i] + t for corresponding pair (i, j).
    :param num_iters: Number of iterations, consisting of NN search and transform update.
    :param inlier_ratio: Ratio of nearest points from which to compute inlier distance.
    :param inlier_dist_mult: Inlier distance multiplier deciding what is included in optimization.
    :param tol: Minimum improvement per iteration to continue.
    :param y_index: Index for NN search for y (cKDTree).
    :param R: Initial rotation.
    :param t: Initial translation.
    :return: Optimized rotation R, translation t, and corresponding criterion value on inliers (including multiplier).
    """
    #print(x.shape, y.shape)
    assert x.shape[0] == y.shape[0]
    if y_index is None:
        y_index = cKDTree(y.T)
    if R is None:
     # print("R is not set!")
        R = np.eye(x.shape[0])
    if t is None:
        t = np.zeros((x.shape[0], 1))
    prev_err_inl = float('inf')

    #err = float('inf')
    #tree = spatial.KDTree(y.T)
    Rl = R
    tl = t
    x = R.dot(x) + t
    #inlier_dist_mult=1.0


    for z in range(num_iters):

       nbs,ind = y_index.query(x.T)
       prev_err_inl = np.mean(nbs, dtype=np.float64)

    #if (prev_err_inl < tol):
    #   print("Returned earlier!")
    #   return Rl, tl, prev_err_inl

       outlier = (inlier_dist_mult*np.percentile(nbs, inlier_ratio*100))
       mask = nbs <= outlier
       x_tmp = x[:, mask]
       ind = ind[mask]
       y_tmp = y[:, ind]
    #y_tmp = np.array((np.array(y[0])[ind], np.array(y[1])[ind]))

       x1p = np.mean(x_tmp[0], dtype=np.float64)
       x2p = np.mean(x_tmp[1], dtype=np.float64)
       y1p = np.mean(y_tmp[0], dtype=np.float64)
       y2p = np.mean(y_tmp[1], dtype=np.float64)
    #blankArray =
       x_tmp -= np.array((np.ones(len(x_tmp[0]),)*x1p, np.ones(len(x_tmp[0]),)*x2p), np.float64)
       y_tmp -= np.array((np.ones(len(y_tmp[0]),)*y1p, np.ones(len(y_tmp[0]),)*y2p), np.float64)

       H = x_tmp.dot(y_tmp.T)
       U, S, V = np.linalg.svd(H, full_matrices=True)
       V = V.T
       R = V.dot(U.T)
       t = np.array((y1p, y2p), np.float64) - R.dot(np.array((x1p, x2p), np.float64))
       t = np.array((np.ones(1,)*t[0], np.ones(1,)*t[1]), np.float64)



       x = R.dot(x) + t

       Rl = R.dot(Rl)
       tl = R.dot(tl) + t



    return Rl, tl, prev_err_inl
