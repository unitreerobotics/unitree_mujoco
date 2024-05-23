
# 介绍
## Unitree mujoco
`unitree_mujoco` 是基于 `Unitree sdk2` 和 `mujoco` 开发的仿真器。用户使用 `Unitree_sdk2`、 `unitree_ros2` 和 `unitree_sdk2_python` 开发的控制程序可以方便地接入该仿真器，实现仿真到实物的开发流程。仓库别基于 c++ 和 python 实现了两个版本的仿真器， 其结构大致如下图所示:

![](./doc/func.png)

## 目录结构
- `simulate`: 基于 unitree_sdk2 和 mujoco (c++) 实现的仿真器
- `simulate_python`: 基于 unitree_sdk2py 和 mujoco (python) 实现的仿真器
- `unitree_robots`: unitree_sdk2 支持的机器人 mjcf 描述文件
- `terrain_tool`: 仿真场景地形生成工具
- `example`: 例程

## 支持的 Unitree sdk2 消息：
- `LowCmd`: 电机控制指令
- `LowState`：电机状态
- `SportModeState`：机器人位置和速度

注：
 1. 电机的编号与机器人实物一致，具体可参考 [Unitree 文档](https://support.unitree.com/home/zh/developer)
 2. 在机器人实物上关闭自带的运控服务后， `SportModeState` 消息是无法读取的。仿真中保留了这一消息，便于用户利用位置和速度信息分析所开发的控制程序。

## 相关链接
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree 文档](https://support.unitree.com/home/zh/developer)
- [mujoco doc](https://mujoco.readthedocs.io/en/stable/overview.html)

# 安装
## c++ 仿真器 (simulate)
### 1. 依赖
#### unitree_sdk2
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
chmod +x ./install.sh
sudo ./install.sh
```
详细见：https://github.com/unitreerobotics/unitree_sdk2
#### mujoco >= 3.0.0
```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```
```bash
git clone https://github.com/google-deepmind/mujoco.git
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
测试:
```bash
simulate
```
弹出 mujoco 仿真器表示安装成功。

#### yaml-cpp
yaml-cpp主要用于配置文件的读取：
```
sudo apt install libyaml-cpp-dev
```

### 2. 编译 unitree_mujoco
```
cd simulate/
mkdir build && cd build
cmake ..
make -j4
```

### 3. 测试:
运行：
```bash
./unitree_mujoco
```
可以看到加载了 Go2 机器人的 mujoco 仿真器。

在新的终端中运行：
```
./test
```
程序会输出机器人在仿真器中的姿态和位置信息，同时机器人的每个电机都会持续输出 1Nm 的转矩。

## Python 仿真器 (simulate_python)
### 1. 依赖
#### unitree_sdk2_python
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
如果遇到问题：
```bash
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```
参考: https://github.com/unitreerobotics/unitree_sdk2_python

#### mujoco-python
```bash
pip3 install mujoco
```
#### joystick
```bash
pip3 install pygame
```
### 2. 测试
```bash
cd ./simulate_python
python3 ./unitree_mujoco.py
```
在新终端运行
```bash
python3 ./test/test_unitree_sdk2.py
```
程序会输出机器人在仿真器中的姿态和位置信息，同时机器人的每个电机都会持续输出 1Nm 的转矩。


 

# 使用
## 1. 仿真配置
### c++ 仿真器
c++ 仿真器的配置文件位于 `/simulate/config.yaml` 中：
```yaml
# 仿真器加载的机器人名称
# "go2", "b2", "b2w", "h1"
robot: "go2"

# 机器人仿真仿真场景文件
# 以 go2 为例，指的是/unitree_robots/go2/文件夹下的 scene.xml 文件
robot_scene: "scene.xml" 

# dds domain id，最好与实物(实物上默认为 0)区分开
domain_id: 1 
# 网卡名称, 对于仿真建议使用本地回环 "lo"
interface: "lo"

# 是否输出机器人连杆、关节、传感器等信息，1为输出
print_scene_information: 1

# 是否使用虚拟挂带, 1 为启用
# 主要用于模拟 H1 机器人初始化挂起的过程 
enable_elastic_band: 0 # For H1 
```
### python 仿真器
python 仿真器的配置文件位于 `/simulate_python/config.py` 中：
```python
# 仿真器加载的机器人名称
# "go2", "b2", "b2w", "h1"
ROBOT = "go2" 

# 机器人仿真仿真场景文件
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene

# dds domain id，最好与实物(实物上默认为 0)区分开
DOMAIN_ID = 1 # Domain id
# 网卡名称, 对于仿真建议使用本地回环 "lo"
INTERFACE = "lo" # Interface 

# 是否输出机器人连杆、关节、传感器等信息，True 为输出
PRINT_SCENE_INFORMATION = True 

# 是否使用虚拟挂带, 1 为启用
# 主要用于模拟 H1 机器人初始化挂起的过程 
ENABLE_ELASTIC_BAND = False 

# 仿真步长 单位(s)
# 为保证仿真的可靠性，需要大于 viewer.sync() 渲染一次所需要的时间
SIMULATE_DT = 0.003  

# 可视化界面的运行步长，0.02 对应 50fps/s
VIEWER_DT = 0.02 
```
## 2. 地形生成工具
我们提供了一个在 mujcoc 仿真器中参数化创建简单地形的工具，支持添加楼梯、杂乱地面、高程图等地形。程序位于 `terrain_tool` 文件夹中。具体的使用方法见 `terrain_tool` 文件夹下的 readme 文件。
![](./doc/terrain.png)

## 3. sim to real
`example` 文件夹下提供了使用不同接口实现 Go2 机器人站起再趴下的简单例子。这些例子简演示了如何使用 Unitree 提供的接口实现仿真到实物的实现。下面是每个文件夹名称的解释：
- `cpp`: 基于 `C++`, 使用 `unitree_sdk2` 接口
- `python`: 基于 `python`，使用 `unitree_sdk2_python` 接口
- `ros2`: 基于`ros2`，使用 `unitree_ros2` 接口

### unitree_sdk2 
1. 编译运行
```bash
cd example/cpp
mkdir build && cd build
cmake ..
make -j4
```
运行：
```bash
./stand_go2 # 控制仿真中的机器人 (需确保 Go2 仿真场景已经加载)
./stand_go2 enp3s0 # 控制机器人实物，其中 enp3s0 为机器人所连接的网卡名称
```

2. sim to real
```C++
if (argc < 2)
{   
    // 如果没有输入网卡，使用仿真的 domian id 和 网卡(本地)
    ChannelFactory::Instance()->Init(1, "lo");
}
else
{   
    // 否则使用指定的网卡
    ChannelFactory::Instance()->Init(0, argv[1]);
}
```

### unitree_sdk2_python
1. 运行：
```bash
python3 ./stand_go2.py # 控制仿真中的机器人 (需确保 Go2 仿真场景已经加载)
python3 ./stand_go2.py enp3s0 # 控制机器人实物，其中 enp3s0 为机器人所连接的网卡名称
```
2. sim to real

```python
if len(sys.argv) <2:
    // 如果没有输入网卡，使用仿真的 domian id 和 网卡(本地)
    ChannelFactortyInitialize(1, "lo")
else:
    // 否则使用指定的网卡
    ChannelFactortyInitialize(0, sys.argv[1])
```
### unitree_ros2

1. 编译安装
首先确保已经正确配置好 unitree_ros2 环境，见 [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)。
```bash
source ~/unitree_ros2/setup.sh
cd example/ros2
colcon build
```

2. 运行仿真
```bash
source ~/unitree_ros2/setup_local.sh # 使用本地网卡
export ROS_DOMAIN_ID=1 # 修改domain id 与仿真一致
./install/stand_go2/bin/stand_go2 # 运行
```

3. 运行实物
```bash
source ~/unitree_ros2/setup.sh # 使用机器人连接的网卡
export ROS_DOMAIN_ID=0 # 使用默认的 domain id 
./install/stand_go2/bin/stand_go2 # 运行
```