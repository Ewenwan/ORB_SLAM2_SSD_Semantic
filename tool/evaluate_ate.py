#!/usr/bin/python
# -*- coding:utf-8 -*-
# 评估 估计的estimated位姿轨迹与真实ground truth位姿轨迹的 绝对差值======
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.

# 依赖 Requirements: 
# sudo apt-get install python-argparse

# 用法 
# python evaluate_ate.py gt.txt est.txt
# --plot3D 画3D轨迹匹配图
# --verbose 显示所有误差信息 均方根 均值 中值 标准差 最大值最小值

import sys
import numpy
import argparse
import associate

# 相似变换误差==
def align_sim3(model, data):
    """Implementation of the paper: S. Umeyama, Least-Squares Estimation
       of Transformation Parameters Between Two Point Patterns,
       IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.
       Input:
           model -- first trajectory (3xn)
           data -- second trajectory (3xn)
       Output:
           s -- scale factor (scalar)
           R -- rotation matrix (3x3)
           t -- translation vector (3x1)
           t_error -- translational error per point (1xn)
    """
    # substract mean
    mu_M = model.mean(0).reshape(model.shape[0],1)
    mu_D = data.mean(0).reshape(data.shape[0],1)
    model_zerocentered = model - mu_M
    data_zerocentered = data - mu_D
    n = np.shape(model)[0]
    
    # correlation
    C = 1.0/n*np.dot(model_zerocentered.transpose(), data_zerocentered)
    sigma2 = 1.0/n*np.multiply(data_zerocentered,data_zerocentered).sum()
    U_svd,D_svd,V_svd = np.linalg.linalg.svd(C)
    D_svd = np.diag(D_svd)
    V_svd = np.transpose(V_svd)
    S = np.eye(3)

    if(np.linalg.det(U_svd)*np.linalg.det(V_svd) < 0):
    S[2,2] = -1

    R = np.dot(U_svd, np.dot(S, np.transpose(V_svd)))
    s = 1.0/sigma2*np.trace(np.dot(D_svd, S))
    t = mu_M-s*np.dot(R,mu_D)
    # TODO:
    model_aligned = s * R * model + t
    alignment_error = model_aligned - data
    t_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]

    #return s, R, t #, t_error
    return s, R, t, t_erro

# 欧式变换误差
"""
由于真实轨迹录制时的坐标系和算法一开始的坐标系存在差异，
所以算法估计的相机轨迹和真实轨迹之间存在一个欧式变换，
所以按照对估计值和真实值进行配准后，
需要求解真实值和匹配的估计值之间的一个欧式变换。

对估计值进行变换后再与真实值计算差值。

"""
def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).
    匹配误差计算====
    Input:
    model -- first trajectory (3xn)    估计值
    data -- second trajectory (3xn)    真值=
    
    Output:
    rot -- rotation matrix (3x3)    两数据的旋转平移矩阵
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn) 匹配误差
    
    """
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1) # 去均值===
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )# 
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
        # outer() 前一个参数表示 后一个参数扩大倍数
        # https://blog.csdn.net/hqh131360239/article/details/79064592
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())# 奇异值分解
    S = numpy.matrix(numpy.identity( 3 ))# 单位阵
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    trans = data.mean(1) - rot * model.mean(1)
    
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    # err = sqrt((x-x')^2 + (y-y')^2 + (z-z')^2) 
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error

def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    2D图
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)
 
def plot_traj3D(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    3D图
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    z = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
            z.append(traj[i][2])
        elif len(x)>0:
            ax.plot(x,y,z,style,color=color,label=label)
            
            label=""
            x=[]
            y=[]
            z=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,z,style,color=color,label=label)          

if __name__=="__main__":
    # parse command line
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--plot3D', help='plot the first and the aligned second trajectory to as interactive 3D plot (format: png)', action = 'store_true')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()
    
    # 读取数据
    first_list = associate.read_file_list(args.first_file)
    second_list = associate.read_file_list(args.second_file)
    
    # 按照时间戳进行匹配，最大差值不能超过 max_difference 0.02
    # 
    matches = associate.associate(first_list, second_list,float(args.offset),float(args.max_difference))    
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")
    
    # 按照匹配对 仅取出x、y、z位置信息
    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    
    # 对匹配后的位置坐标求解误差====
    rot,trans,trans_error = align(second_xyz,first_xyz)
    
    # 相互匹配误差线  
    second_xyz_aligned = rot * second_xyz + trans
    first_stamps = first_list.keys()
    first_stamps.sort()
    first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
    second_stamps = second_list.keys()
    second_stamps.sort()
    second_xyz_full = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    second_xyz_full_aligned = rot * second_xyz_full + trans
    
    if args.verbose:
        print "compared_pose_pairs %d pairs"%(len(trans_error))
        # 均方根误差 误差平方均值 再开根号
        print "absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        # 误差均值
        print "absolute_translational_error.mean %f m"%numpy.mean(trans_error)
        # 误差中值
        print "absolute_translational_error.median %f m"%numpy.median(trans_error)
        # 误差标准差
        print "absolute_translational_error.std %f m"%numpy.std(trans_error)
        # 误差最小值
        print "absolute_translational_error.min %f m"%numpy.min(trans_error)
        # 误差最大值
        print "absolute_translational_error.max %f m"%numpy.max(trans_error)
    else:
        print "%f"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        
    if args.save_associations:
        file = open(args.save_associations,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
        file.close()
        
    if args.save:
        file = open(args.save,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_full_aligned.transpose().A)]))
        file.close()

    if args.plot:
        # 绘制2D图=====
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.pylab as pylab
        from matplotlib.patches import Ellipse
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","ground truth")
        plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")
        
        # 误差线
        #label="difference"
        #for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
        #    ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
        #    label=""
            
        ax.legend()
            
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.savefig(args.plot,dpi=90)
        
    if args.plot3D:
        # 绘制3D图======
        import matplotlib as mpl
        mpl.use('Qt4Agg')
        from mpl_toolkits.mplot3d import Axes3D
        import numpy as np
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        #        ax = fig.add_subplot(111)
        plot_traj3D(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","groundTruth")
        plot_traj3D(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","orb-slam2-flow")
        
        # 误差线
        #label="difference"
        #for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
        #    ax.plot([x1,x2],[y1,y2],[z1,z2],'-',color="red",label=label)
        #    label=""            
        
        ax.legend()
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        print "Showing"
        plt.show(block=True)
        plt.savefig("./test.png",dpi=90)
        #answer = raw_input('Back to main and window visible? ')
        #if answer == 'y':
        #    print('Excellent')
        #else:
        #    print('Nope')

    #plt.savefig(args.plot,dpi=90)
