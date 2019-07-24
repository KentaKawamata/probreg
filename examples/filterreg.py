#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import numpy as np
import open3d as o3
import transformations as trans
from probreg import filterreg
from probreg import callbacks
import utils
import os
import glob


if __name__ == "__main__":

    origin_cloud = "26.ply"
    tf = "27"
    tf_cloud = tf + ".ply"

    # load source and target point cloud
    point_t = "/mnt/container-data/remove_plane/" + origin_cloud
    source_t = o3.read_point_cloud(point_t)

    point_t_plus_one = "/mnt/container-data/remove_plane/" + tf_cloud
    source_t_plus_one = o3.read_point_cloud(point_t_plus_one)

    #o3.draw_geometries([source_t])
    source_t = o3.voxel_down_sample(source_t, voxel_size=0.020)
    source_t_plus_one = o3.voxel_down_sample(source_t_plus_one, voxel_size=0.020)
    #o3.draw_geometries([source_t])

    cbs = [callbacks.Open3dVisualizerCallback(source_t_plus_one, source_t)]
    objective_type = 'pt2pt'
    

    """
    tf_param, _, _ = filterreg.registration_filterreg(source_t_plus_one, source_t,
                                                      objective_type=objective_type,
                                                      sigma2=0.02,
                                                      callbacks=cbs,
                                                      maxiter=1,
                                                      tol=0.01)
    """

    tf_param, _, _ = filterreg.registration_filterreg(source_t_plus_one, source_t,
                                                      objective_type=objective_type,
                                                      sigma2=55.7,
                                                      callbacks=cbs,
                                                      maxiter=5,
                                                      tol=4.5)

    rot = trans.identity_matrix()
    rot[:3, :3] = tf_param.rot

    path = '/mnt/container-data/rotation/' + tf + ".txt"

    with open(path, mode='w') as f:
        
        for i in range(4):
            for j in range(4):
                f.write(np.array2string(rot[j][i]) + "0" + "\n")



    result = copy.deepcopy(source_t_plus_one)
    result.points = tf_param.transform(result.points)

    # draw result
    source_t.paint_uniform_color([1, 0, 0])
    source_t_plus_one.paint_uniform_color([0, 1, 0])
    result.paint_uniform_color([0, 0, 1])

    origin_t_plus_one = "/mnt/container-data/ply_data/" + tf_cloud
    P_t_plus_one = o3.read_point_cloud(origin_t_plus_one)
    
    P_t_plus_one.points = tf_param.transform(P_t_plus_one.points)
    
    result_name = "/mnt/container-data/num_data/" + tf_cloud
    o3.io.write_point_cloud(result_name, P_t_plus_one)


    #o3.draw_geometries([source_t, source_t_plus_one, P_t_plus_one])

    
