---
description: "分析一个 ROS2 包的结构、功能和关键接口。用法: analyze-ros-package <包路径或包名>"
---

# 分析 ROS2 包结构

分析 `$ARGUMENTS` 指定的 ROS2 包，输出结构化摘要。

## 步骤

1. **定位包根目录**：如果参数是包名，在 `src/` 下搜索 `package.xml` 匹配；如果是路径，直接使用。
2. **读取 package.xml**：提取包名、版本、构建类型（ament_cmake/ament_python/catkin）、依赖列表。
3. **读取 CMakeLists.txt 或 setup.py**：提取构建目标、可执行文件、库、安装规则。
4. **列出接口定义**：`msg/`、`srv/`、`action/` 下的所有 `.msg`、`.srv`、`.action` 文件，逐个读取内容。
5. **列出源文件**：`src/`、`scripts/` 下的 `.cpp`、`.py`、`.hpp`、`.h` 文件。
6. **列出 launch 文件**：`launch/` 下的 `.launch.py` 或 `.launch` 文件。
7. **列出配置文件**：`config/`、`param/`、`share/` 下的 `.yaml`、`.xml` 文件。
8. **分析节点**：对每个源文件，提取 ROS 节点名、订阅/发布的话题、服务客户端/服务器、参数。
9. **分析 launch 流程**：如果存在 launch 文件，描述启动顺序和参数。

## 输出格式

```
## 包概览
- 包名: xxx
- 构建类型: ament_cmake / ament_python / catkin
- ROS 版本: ROS1 / ROS2 Humble

## 节点列表
| 节点 | 类型 | 订阅 | 发布 | 服务 | 参数 |

## 接口定义
| 类型 | 名称 | 字段/方法 |

## Launch 文件
| 文件 | 启动节点 | 参数 |

## 依赖
- 构建依赖: ...
- 运行依赖: ...

## 关键发现
- ...
```

## 注意

- 如果包是 ROS1 catkin，额外标注需要迁移的部分。
- 如果包中有嵌套 `.git`，提醒用户可用 `remove-nested-git` 命令清理。
- 用户语言为中文，输出用中文。
