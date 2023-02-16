# xrrover_description

xacro 構造短且易讀的 robot urdf's XML 文件

## 重點檔案

```bash
├── meshes
│   ├── base
│   │   └── rover_base.dae
│   └── wheels
│       ├── rover_wheel_1.dae
│       └── rover_wheel_2.dae
├── rviz
│   └── xrrover.rviz
├── urdf
│   ├── base
│   │   └── rover_base.urdf.xacro
│   ├── gazebo
│   │   └── rover_gazebo.xacro
│   ├── laser
│   │   └── rover_laser.urdf.xacro
│   ├── wheels
│   │   └── rover_wheel.urdf.xacro
│   └── xrrover.urdf.xacro
└── worlds
    └── turtlebot3_world.world
```

## 功能概述
* xrrover.urdf.xacro: 利用 xacro 組裝機器人之各構件描述