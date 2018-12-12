#!/usr/bin/env python
# coding=utf-8
# 绘制 GT、SRC、FLOW 相机3D轨迹
# 使用 python draw_trajectory.py
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
# 起始位置 -0.6885 -3.1192 1.4248


f2 = open("./flow.txt")
x2 = []
y2 = []
z2 = []
for line in f2:
    if line[0] == '#':
        continue
    data = line.split()
    x2.append( float(data[1] ) -0.6885 )
    y2.append( float(data[2] ) -3.1192)
    z2.append( float(data[3] ) +1.4248)

f3 = open("./src.txt")
x3 = []
y3 = []
z3 = []
for line in f3:
    if line[0] == '#':
        continue
    data = line.split()
    x3.append( float(data[1] ) -0.6885)
    y3.append( float(data[2] ) -3.1192)
    z3.append( float(data[3] ) +1.4248)

f4 = open("./geom.txt")
x4 = []
y4 = []
z4 = []
for line in f4:
    if line[0] == '#':
        continue
    data = line.split()
    x4.append( float(data[1] ) -0.6885)
    y4.append( float(data[2] ) -3.1192)
    z4.append( float(data[3] ) +1.4248)
    
ax = plt.subplot( 111, projection='3d')
ax.plot(x,y,z,'r',label='groundtruth')  # 红色线条
ax.plot(x2,y2,z2,'g',label='flow')  # 绿色线条
ax.plot(x3,y3,z3,'b',label='src')  # 蓝色线条
ax.plot(x4,y4,z4,'y',label='geom')  # 黄色线条

ax.legend(loc='best')# 显示图例 label=' ' 
ax.set_zlabel('Z')  # 坐标轴
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.set_title('trajectory')#图标题  
plt.show()
