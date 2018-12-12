#!/usr/bin/env python
# coding=utf-8
# 绘制 相机3D轨迹
# 使用 python draw_groundtruth.py
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

f = open("./groundtruth.txt")
x = []
y = []
z = []
for line in f:
    if line[0] == '#':
        continue
    data = line.split()
    x.append( float(data[1] ) )
    y.append( float(data[2] ) )
    z.append( float(data[3] ) )
ax = plt.subplot( 111, projection='3d')
ax.plot(x,y,z,'r')  # 红色线条
ax.set_zlabel('Z')  # 坐标轴
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.set_title('GT trajectory')#图标题  
plt.show()
