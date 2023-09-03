<h1><center>SimOne平台智能网联车_开发手册</center></h1>





Created On 2023/9/3

> 调研合适的开源的自动驾驶平台：
>
> 0. MatLab Automated Driving Toolbox
> 0. Python-C Coder Matlab
>

> :clock1: 其他相关的开源参考项目
>
> *1.Carla (ubuntu)*
>
> *2.Apollo*
>
> *3.SUMO  CARSIM*
>
> 主流算法待开发和测试





****

得分手册主要冲击得分点

1. 单个主车规划车道场景
2. ADAS测试
3. 交通视觉流的规避



* 难度较大的是车辆超车交互以及路口行人测试

****



> >要解决的控制算法开发任务包括：
> >
> >0. 车辆动力学建模(关键词：轮胎参数，底盘机械参数，线和角速度，加速度，转动惯量，驾驶状态)
> >
> >1.环境感知技术(关键词：雷达radar，激光雷达lidar，图像视频流Streaming, )
> >
> >2.路径规划技术(关键词：路点waypoints, )
> >
> >3.全局地图定位技术 （关键词: 道路laneid,道路长宽,道路)
> >
> >4.决策与控制技术

## 第一步  API分工整合



已知PySimOneAPi的集成信息为：

* 传感器模块（`SimoneSensorApi`） 包括gps，imu, drive

  *  车体信息，包括视觉彩色图像，点云，雷达测距，以及动力学物理参数

  - 地图信息：车道、路点、（信息嵌入到global hdmap)
  - 交通信息：主车id、障碍物id、地图标识,，车道连接信息（规划算法）

* 通信服务模块

  基于c++的回调机制，提供了便捷的ros接口和matlab接口（未测试）

  

  ### 面向对象模块的数据类型

  * 车辆模型

    * 车体控制: **包括油门`throttle`，刹车`brake`, 档位`gear`,离合器`clutch`，转向角`steering`**(相关：转动比，转矩，角速度，轮胎角速度)

    核心的控制量为：
    $$
    <St,T,B,G,C> \Longleftarrow \mathcal{St}_i( <T_{data},B_{data}>)
    $$

    * 轮胎参数：轮胎线速度$V$，角速度，加速度；

    >1.位置控制 (转角xyz/rad,给定位置xyz坐标/m)
    >
    >2.电子发动机控制(停止距离，速度限制，转向矩，加速度限制容许，刹车和档位模式)

    此外，注意预定义的事件的优先级，包括不限于左转右转，道路变向，紧急刹车，超速，碰撞

    * IMU参数

      XYZ位置,XYZ线速度角速度，XYZ转角

    * 驾驶状态

      行驶 完成 未知

  * 地图

    * 车道：`Lane`:

      各种道路类型，包括不限于停车道，人行道，非机动车道`bike`，铁轨，电车，斜坡`ramp`，出口`entry` 

      边界类型和彩色信息，反光片`botts_dots` 固体 残缺 绿地 马路牙子`curb`

      道路的弯曲类型，弯曲参数包含,以及直线空间点坐标
      $$
      <C_0,C_1,C_2,C_3,St,Ed,L>
      $$

    * 路点

      四元数信息`quaternion` $x,y,z,w$ 路点坐标$X，Y$

    * GPS: 

      特别注意这里可以拿到车前后轮的左右速度

      里程计模型 `odometer`

      发动机转速`engineRom`

      GPS信号稳定`isGPSLost`

      关联到的imu数据

    * OBU车载单元《==》V2X车辆通信

  * 交通 

    * 信号灯
    * 障碍物`Obstacle` 长宽高,位置，角度，·id与·viewid，类型(碰撞与优先级)

  * 传感器模块：

    * 基本配置为 xyz,`roll`,`pitch`,`yaw`,传感器的频率`hz`

    * **视觉：**

      * 图像 RGB（HWC)
      * 点云的点测次数，点的存储size $\Longleftarrow$ xyz ,`rgbd`(深度信息)

    * **毫米波雷达:**

      角，位置，长宽高，范围，方位角`azimuth`，垂直角`vertical`(obstacle),相对范围位置和速度

      噪声比`snr`，散射截面`RCS`(反映目标对雷达敏感程度的度量)

      反馈障碍物数量和检测信息

    * **视觉传感器：**

       置信框,绝对位置，相对位置，检测物体长宽高

  * 人机控制接口
    * 硬件节点配置`Node` 雷达 相机 传感器融合 `GNSSINS`
    * 环境情景参数: 时间 光源可见性(环境光和人工光)  天气密度(如雨雪)，地面湿滑度

## 第二步  模块化分工设计



`任志恒` : 

* 传感器模块c api梳理
* 视觉模块c api梳理

`颜铭`：

* python api doc summary

* matlab 接口 和 c++相关测试开发

`任士伟`

* 车辆模型动力学模块 c api梳理

`茹翔宇`

* 交通模块c api开发



##  不同语言API的整理





### Python-C API Module



1. **HDMAP-全景地图**

>*TODO:*

2. **Sensor-传感器**

>*TODO:*

1. **Streaming-图像视频流**

> *TODO:*

1. **PNC-场景事件-动力学**

> *TODO:*



### Matlab API Module (Simulink)



### Cpp API Module (x64-VC)

