#!/usr/bin/env python
# coding=utf-8
# 绘制 相机3D轨迹
# 使用 python draw_groundtruth_Rt_associate.py
import numpy
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

import sys

# 数据匹配=======
import associate



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


# 读取数据=================================================================================
ground_truth_list = associate.read_file_list("./groundtruth.txt")   # 真实轨迹数据
src_list          = associate.read_file_list("./src.txt")           # 原 orb-slam2 预测
flow_list         = associate.read_file_list("./flow3.txt")         # flow 光流加持
geom_list         = associate.read_file_list("./geom.txt")          # geom 多视角几何加持
offset = 0.0 # 时间偏移量
max_difference = 0.02 # 最大时间差值
scale = 1.0 # 数据大小尺度 


# 真实值与 原版本算法预测值 进行对比==================================== 
# 按照时间戳进行匹配，最大差值不能超过 max_difference 0.02
matches_src = associate.associate(ground_truth_list, src_list,float(offset),float(max_difference))    
if len(matches_src)<2:
    sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

# 按照匹配对 仅取出x、y、z位置信息
gt_src_ass_xyz = numpy.matrix([[float(value) for value in ground_truth_list[a][0:3]] for a,b in matches_src]).transpose()
src_ass_xyz = numpy.matrix([[float(value) * float(scale) for value in src_list[b][0:3]] for a,b in matches_src]).transpose()
# 对匹配后的位置坐标求解误差====
rot,trans,trans_error = align(src_ass_xyz,gt_src_ass_xyz)
# 原数据的所有值
gt_stamps = ground_truth_list.keys()
gt_stamps.sort() # 按照时间先后顺序 排序
gt_xyz = numpy.matrix([[float(value) for value in ground_truth_list[b][0:3]] for b in gt_stamps]).transpose()
src_stamps = src_list.keys()
src_stamps.sort()
src_xyz = numpy.matrix([[float(value) * float(scale) for value in src_list[b][0:3]] for b in src_stamps]).transpose()
src_xyz_aligned = rot * src_xyz + trans # 欧式变换后的值
print "src: %d pairs"%(len(trans_error))
# 均方根误差 误差平方均值 再开根号
print "src rmse : %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))




# 真实值与 flow版本算法预测值 进行对比==================================== 
# 按照时间戳进行匹配，最大差值不能超过 max_difference 0.02
matches_flow = associate.associate(ground_truth_list, flow_list,float(offset),float(max_difference))    
if len(matches_flow)<2:
    sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

# 按照匹配对 仅取出x、y、z位置信息
gt_flow_ass_xyz = numpy.matrix([[float(value) for value in ground_truth_list[a][0:3]] for a,b in matches_flow]).transpose()
flow_ass_xyz = numpy.matrix([[float(value) * float(scale) for value in flow_list[b][0:3]] for a,b in matches_flow]).transpose()
# 对匹配后的位置坐标求解误差====
rot2,trans2,trans_error2 = align(flow_ass_xyz,gt_flow_ass_xyz)

# 原数据的所有值
flow_stamps = flow_list.keys()
flow_stamps.sort() # 按照时间先后顺序 排序
flow_xyz = numpy.matrix([[float(value) * float(scale) for value in flow_list[b][0:3]] for b in flow_stamps]).transpose()
flow_xyz_aligned = rot2 * flow_xyz + trans2 # 欧式变换后的值
print "flow: %d pairs"%(len(trans_error2))
# 均方根误差 误差平方均值 再开根号
print "flow rmse : %f m"%numpy.sqrt(numpy.dot(trans_error2,trans_error2) / len(trans_error2))





# 真实值与 geom版本算法预测值 进行对比==================================== 
# 按照时间戳进行匹配，最大差值不能超过 max_difference 0.02
matches_geom = associate.associate(ground_truth_list, geom_list,float(offset),float(max_difference))    
if len(matches_geom)<2:
    sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

# 按照匹配对 仅取出x、y、z位置信息
gt_geom_ass_xyz = numpy.matrix([[float(value) for value in ground_truth_list[a][0:3]] for a,b in matches_geom]).transpose()
geom_ass_xyz = numpy.matrix([[float(value) * float(scale) for value in geom_list[b][0:3]] for a,b in matches_geom]).transpose()
# 对匹配后的位置坐标求解误差====
rot3,trans3,trans_error3 = align(geom_ass_xyz,gt_geom_ass_xyz)

# 原数据的所有值
geom_stamps = geom_list.keys()
geom_stamps.sort() # 按照时间先后顺序 排序
geom_xyz = numpy.matrix([[float(value) * float(scale) for value in geom_list[b][0:3]] for b in geom_stamps]).transpose()
geom_xyz_aligned = rot3 * geom_xyz + trans3 # 欧式变换后的值
print "geom: %d pairs"%(len(trans_error3))
# 均方根误差 误差平方均值 再开根号
print "geom rmse : %f m"%numpy.sqrt(numpy.dot(trans_error3,trans_error3) / len(trans_error3))


# 3行×n列 转换成数组 array
gt_xyz = gt_xyz.A
src_xyz_aligned = src_xyz_aligned.A
flow_xyz_aligned = flow_xyz_aligned.A
geom_xyz_aligned = geom_xyz_aligned.A
# 
#print type(gt_xyz)
#print gt_xyz
#Sprint gt_xyz[0]

ax = plt.subplot( 111, projection='3d')
ax.plot(gt_xyz[0], gt_xyz[1], gt_xyz[2],'r',label='groundTruth')  # 红色线条
ax.plot( src_xyz_aligned[0], src_xyz_aligned[1],src_xyz_aligned[2], 'b',label='orb-slam2-src')  # 蓝色线条
ax.plot(flow_xyz_aligned[0],flow_xyz_aligned[1],flow_xyz_aligned[2],'g',label='orb-slam2-flow')  # 绿色线条
ax.plot(geom_xyz_aligned[0],geom_xyz_aligned[1],geom_xyz_aligned[2],'y',label='orb-slam2-geom')  # 黄色线条

#ax.plot(gt_xyz.transpose(),'r',label='groundTruth')  # 红色线条
#ax.plot(src_xyz_aligned.transpose(),'b',label='orb-slam2-src')  # 蓝色线条
#ax.plot(flow_xyz_aligned.transpose(),'g',label='orb-slam2-flow')  # 绿色线条
#ax.plot(geom_xyz_aligned.transpose(),'y',label='orb-slam2-geom')  # 黄色线条

ax.legend(loc='upper center')# 显示图例 label=' '  ‘center right'  best upper center
ax.set_zlabel('Z/m')  # 坐标轴
ax.set_ylabel('Y/m')
ax.set_xlabel('X/m')
ax.set_title('trajectory')#图标题  
plt.xlim( -1.6, 0.2 )
ax.set_zlim( 0.4, 2.6 )

plt.show()
