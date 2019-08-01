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

##################################
# FilterRegに用いるパラメータ 
##################################
sig = None
ter = 1
ol = 0.00010
##################################

def read_rotation(model_num):
    
    rot_in = trans.identity_matrix()
    rote_path = '/mnt/container-data/rotation/' + model_num + ".txt"
    
    with open(rote_path, mode='r') as f:
        
        l_strip = [s.strip() for s in f.readlines()]

        k=0
        for i in range(4):
            for j in range(4):
                rot_in[j][i] = np.array(l_strip[k])
                k=k+1
    
    return rot_in

def write_rotation(tf, scene_num):

    #
    # rot =
    #     | rot[00] rot[01] rot[02] t[0] |
    #     | rot[10] rot[11] rot[12] t[1] |
    #     | rot[20] rot[21] rot[22] t[2] |
    #     |    0       0       0     1   |
    #

    rot = trans.identity_matrix()
    rot[:3, :3] = tf.rot

    rot[0][3] = tf.t[0]
    rot[1][3] = tf.t[1]
    rot[2][3] = tf.t[2]

    print(rot)

    print("result: ", np.rad2deg(trans.euler_from_matrix(rot)),tf.scale, tf.t)

    path = '/mnt/container-data/rotation/' + scene_num + ".txt"

    with open(path, mode='w') as f:
        
        for i in range(4):
            for j in range(4):
                f.write(np.array2string(rot[j][i]) + "\n")


def main():

    argvs = sys.argv
    model_num = str(argvs[1])
    scene_num = str(argvs[2])
    cloud_dir = "/mnt/container-data/remove_plane/"

    model_path = cloud_dir + model_num + ".ply"
    model_cloud = o3.read_point_cloud(model_path)

    scene_path = cloud_dir + scene_num + ".ply"
    scene_cloud = o3.read_point_cloud(scene_path)
    
    rot_in = read_rotation(model_num)
    model_cloud.transform(rot_in)
    
    model_cloud = o3.voxel_down_sample(model_cloud, voxel_size=0.020)
    scene_cloud = o3.voxel_down_sample(scene_cloud, voxel_size=0.020)

    # 基準とするpointcloud, 回転させたいpointcloud の順番 
    cbs = [callbacks.Open3dVisualizerCallback(model_cloud, scene_cloud)]
    objective_type = 'pt2pt'
    
    # 基準とするpointcloud, 回転させたいpointcloud の順番 
    tf_param, _, _ = filterreg.registration_filterreg(model_cloud, scene_cloud, 
                                                      scene_cloud.normals,
                                                      objective_type=objective_type,
                                                      sigma2=sig,
                                                      callbacks=cbs,
                                                      maxiter=ter,
                                                      tol=ol)

    write_rotation(tf_param, scene_num)

    
if __name__ == "__main__":

    main()
