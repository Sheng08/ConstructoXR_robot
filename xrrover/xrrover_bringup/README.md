# xrrover_bringup

使用 urdf 啟動機器人模型 (tf & rviz)  

## 重點檔案

```bash
└── launch
    ├── includes
    │   └── description.launch.xml
    ├── xrrover_model.launch
    └── xrrover_remote.launch
```

## 功能概述
* xrrover_remote.launch: tf  
* xrrover_model.launch: xrrover_remote + rviz  
* description.launch.xml: 啟動機器人描述