oryxbot

本文只讲解oryxbot仿真部分的使用，实验可以对照实验指导书和使用说明手册一步一步进行，大部分步骤与真实机器人无异，当然关于前期基础的底盘、里程计等开发是不需要的

slam建图实验
#打开仿真与slam
~$ roslaunch oryxbot_slam oryxbot_slam_sim.launch 
#打开手柄节点
~$ roslaunch oryxbot_joy oryxbot_joy.launch 
#现在就可以了使用手柄控制机器人进行建图了


打开所有功能，定点抓取应用实验
#参考oryxTarm实验指导书.pdf：综合实验三、定点抓取应用实验
#首先打开机器人仿真及导航
~$ roslaunch oryxbot_navigation demo_nav_2d.launch 
#然后打开机械臂moveit控制 ar码识别、moveit编程节点、自主充电等应用
~$ roslaunch ar_pose_adjust ar_sim.launch 
#打开中心调度系统
~$ roslaunch oryxbot_apply oryxbot_center.launch 
#开始调度
~$ rosservice call /center_client "data: 0" 



###------------------------------------------------------------------------------------------------###

###------------------------------------------------------------------------------------------------###

#打开后的世界一般会有几个工作台及ar码物块，我们可以通过rviz 页面上方的2D Nav Goal箭头将机器人导航到工作台的正前方
#然后使用视觉二次定位导航服务将车体移动到工作台前面(#mode:工作台 ar 码所含信息(0~126),relative_move:最终距离 ar 码的距离(单位:m)):
~$ rosservice call /track_ar “relative_move: 0.02
mode: 0”
#抓取物块并放置到盒子里:(number：为抓取物块所贴ar码包含的信息；mode：0-抓取工作台物块，1-抓取现有位置下的物块；position：放置位置的x/y/z，全为0时不放下，盒子的坐标值(53.0,225.0,-3.0)后期会给出)
~$ rosservice call /pick_ar "number: 0
mode: 0
position:
- 53.0
- 225.0
- -3.0"
#最后需要将车体退出来(因为车离工作台太近,处于激光雷达的避障范围内)
#调用定向移动服务,后退 0.4m (参数说明: relative_move:车体坐标系下移动的距离(单位:m),mode:0-x 方向,1-y 方向,2-th 此时 relative_move 单位为弧度)。
~$ rosservice call /relative_move “relative_move: -0.4
mode: 0


###注意！！！由于仿真运行在gazebo7下，很多物理属性不太完善，有时会出现翻车或者抓取不到物块现象，这都很正常
#参考文件：oryxTarm实验指导书.pdf 、 oryxTarm使用手册.pdf
