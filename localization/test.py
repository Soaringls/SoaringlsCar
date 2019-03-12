#Location直方图滤波算法实现与概率分布可视化 参考 https://zhuanlan.zhihu.com/p/46250526
import numpy as np
import matplotlib.pyplot as plt
#初始化概率，定义每个网格颜色，由此一维网格地图初始化完毕
p=[0.2,0.2,0.2,0.2,0.2]
world=['green','red','red','green','green']  #各个位置的特征
#加入传感器与运动
measurements=['red','green'] #测量值
motions     =[-1,1] #机器人移动方向,-1左 1右
pHit        =0.6    #测量值正确的概率
pMiss       =0.2    #测量值失败的概率
pExact      =0.8
pOvershoot  =0.1
pUndershoot =0.1
                                                            
def sense(p, Z):
    q = []
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i]*(hit*pHit+(1-hit)*pMiss))
    s = sum(q)
    #Normalized sense
    for i in range(len(q)):
        q[i] = q[i] / s
    return q
def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i - U)%len(p)]
        print("index1 is "+str((i - U)%len(p)))
        s = s + pOvershoot * p[(i-U-1)%len(p)]
        # print("index2 is------"+str((i-U-1)%len(p)))
        s = s + pUndershoot* p[(i-U+1)%len(p)]
        # print("index3 is------------ "+str((i-U+1)%len(p)))
        q.append(s)
    return q
def display_map(grid, bar_width = 1):
    if(len(grid) > 0):
        x_labels = range(len(grid))
        plt.bar(x_labels, height = grid, width = bar_width, color='b')
        plt.xlabel('Grid Cell')
        plt.ylabel('Probability')
        plt.ylim(0,1)
        plt.title('Probability of the robot being at each cell in the grid')
        plt.xticks(np.arange(min(x_labels), max(x_labels)+1, 1))
        plt.show()
    else:
        print('Grid is empty')
def main():
    global p
    for k in range(len(measurements)):
        p = sense(p, measurements[k])
        p = move(p, motions[k])
    print(p)
    display_map(p)

    # global p
    # plt.ion()
    # for k in range(len(measurements)):
    #     p = sense(p, measurements[k])
    #     for i in range(1000):
    #         p = move(p, motions[k])
    #         plt.pause(0.2)
    #         display_map(p, 0.9)
    # print(p)
if __name__=='__main__':
    main()