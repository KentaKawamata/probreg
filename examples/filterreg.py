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
import sys

sig = None
ter = 1
ol = 0.00010


def main():

    argvs = sys.argv

    first = str(argvs[1])
    tf = str(argvs[2])

    origin_cloud = first + ".ply"
    tf_cloud = tf + ".ply"

    # load source and target point cloud
    point_t = "/mnt/container-data/remove_plane/" + origin_cloud
    source_t = o3.read_point_cloud(point_t)

    point_t_plus_one = "/mnt/container-data/remove_plane/" + tf_cloud
    source_t_plus_one = o3.read_point_cloud(point_t_plus_one)
    
    rot_in = trans.identity_matrix()
    rote_path = '/mnt/container-data/rotation/' + first + ".txt"
    
    with open(rote_path, mode='r') as f:
        
        l_strip = [s.strip() for s in f.readlines()]

        k=0
        for i in range(4):
            for j in range(4):
                rot_in[j][i] = np.array(l_strip[k])
                k=k+1

    source_t = o3.voxel_down_sample(source_t, voxel_size=0.020)
    source_t_plus_one = o3.voxel_down_sample(source_t_plus_one, voxel_size=0.020)

    # 基準とするpointcloud, 回転させたいpointcloud の順番 
    cbs = [callbacks.Open3dVisualizerCallback(source_t, source_t_plus_one)]
    objective_type = 'pt2pt'
    
    # 基準とするpointcloud, 回転させたいpointcloud の順番 
    tf_param, _, _ = filterreg.registration_filterreg(source_t, source_t_plus_one, 
                                                      source_t_plus_one.normals,
                                                      objective_type=objective_type,
                                                      sigma2=sig,
                                                      callbacks=cbs,
                                                      maxiter=ter,
                                                      tol=ol)

    # rot =
    #     | rot[00] rot[01] rot[02] t[0] |
    #     | rot[10] rot[11] rot[12] t[1] |
    #     | rot[20] rot[21] rot[22] t[2] |
    #     |    0       0       0     1   |
    #

    rot = trans.identity_matrix()
    rot[:3, :3] = tf_param.rot

    rot[0][3] = tf_param.t[0]
    rot[1][3] = tf_param.t[1]
    rot[2][3] = tf_param.t[2]


    print(rot)

    print("result: ", np.rad2deg(trans.euler_from_matrix(rot)),
          tf_param.scale, tf_param.t)
    #print("result: ", rot, tf_param.scale, tf_param.t)


    path = '/mnt/container-data/rotation/' + tf + ".txt"

    with open(path, mode='w') as f:
        
        for i in range(4):
            for j in range(4):
                f.write(np.array2string(rot[j][i]) + "0" + "\n")


    result = copy.deepcopy(source_t_plus_one)
    result.points = tf_param.transform(result.points)

    # draw result

if __name__ == "__main__":

    main()
