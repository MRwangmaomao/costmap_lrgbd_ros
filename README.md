# 代价地图与智能导航程序

## 1. 整个流程

深度图， 局部障碍物地图， RGBD局部代价地图， 局部代价地图可视化图

- step1 将深度图转化为点云
- step2 对点云进行xz平面的投影
- step3 利用射线法检测出可通行区域，蓝色为通行区域，绿色为障碍物，黑色为未知区域
- step4 障碍物膨胀
- step5 获取并生成一个waypoint，转换到机器人坐标系下
- step6 利用DWA算法计算机器人到达waypoint的速度，控制机器人移动

![cost_map](image/costmap.png)
左图为局部障碍物地图，右图为局部代价地图

![cost_map](image/tf_tree.png)

