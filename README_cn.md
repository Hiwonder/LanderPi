# LanderPi

[English](https://github.com/Hiwonder/LanderPi/blob/main/README.md) | 中文

<p align="center">
  <img src="./sources/images/image.webp" alt="LanderPi Logo" width="400"/>
</p>

基于ROS2的复合型机器人系统，集成机械臂、激光雷达、3D深度相机等高性能组件，支持SLAM导航、自主抓取、语音交互等高级AI功能。

## 产品介绍

LanderPi是幻尔面向ROS教育场景开发的复合型机器人，它支持麦克纳姆轮、阿克曼、履带三种底盘配置，集成了树莓派5、高精度激光雷达、3D深度相机、3D深度视觉机械臂、赛事级轮胎等高性能配件，可以实现机器人运动控制、遥控通讯、SLAM建图导航、路径规划、动态避障、无人驾驶、体感控制、机械臂MoveIt仿真、手眼合一自主抓取、自主导航搬运、自然语言交互等应用。

LanderPi同时部署了多模态AI大模型，结合AI语音交互盒，它可以理解环境、规划行动并灵活执行任务，能实现更多高阶具身智能应用。

## 官方资源

### Hiwonder官方
- **官方网站**: [https://www.hiwonder.net/](https://www.hiwonder.net/)
- **产品页面**: [https://www.hiwonder.com/products/landerpi](https://www.hiwonder.com/products/landerpi)
- **官方文档**: [https://docs.hiwonder.com/projects/LanderPi/en/latest/](https://docs.hiwonder.com/projects/LanderPi/en/latest/)
- **技术支持**: support@hiwonder.com

## 主要功能

### AI视觉与导航
- **SLAM建图** - 实时同步定位与建图
- **路径规划** - 智能路线规划和导航
- **动态避障** - 实时障碍物检测与避障
- **无人驾驶** - 具备先进导航功能的自动驾驶能力
- **3D深度感知** - 深度相机集成的立体视觉

### 机械臂集成
- **3D深度视觉机械臂** - 先进的操作能力
- **MoveIt仿真** - 运动规划和控制仿真
- **手眼合一** - 基于视觉反馈的自主抓取
- **自主导航搬运** - 拾取和放置操作

### 智能控制
- **多底盘支持** - 麦克纳姆轮、阿克曼和履带
- **体感控制** - 直观的人机交互
- **遥控通讯** - 无线控制和监控
- **自然语言交互** - AI驱动的语音命令
- **多模态AI集成** - 先进的具身智能

### 编程接口
- **ROS2集成** - 完整的机器人操作系统2支持
- **Python编程** - 全面的Python SDK
- **AI语音交互** - 自然语言处理能力
- **仿真环境** - 完整的开发和测试平台

## 硬件配置
- **处理器**: 树莓派5
- **操作系统**: ROS2兼容Linux系统
- **视觉系统**: 3D深度相机 + 高精度激光雷达
- **操作系统**: 3D深度视觉机械臂
- **底盘**: 支持麦克纳姆轮、阿克曼和履带三种配置
- **轮胎**: 赛事级高性能轮胎
- **AI集成**: 多模态AI大模型配AI语音交互盒

## 项目结构

```
landerpi/
├── src/                    # 源代码模块
├── command                 # 命令参考和工具
└── sources/                # 资源和文档
    └── images/             # 产品图像和媒体
```

## 版本信息
- **当前版本**: LanderPi v1.0.0
- **支持平台**: 树莓派5

### 相关技术
- [ROS2](https://ros.org/) - 机器人操作系统2
- [MoveIt](https://moveit.ros.org/) - 运动规划框架
- [OpenCV](https://opencv.org/) - 计算机视觉库
- [PCL](https://pointclouds.org/) - 点云库

---

**注**: 所有程序已预装在LanderPi机器人系统中，可直接运行。详细使用教程请参考[官方文档](https://docs.hiwonder.com/projects/LanderPi/en/latest/)。