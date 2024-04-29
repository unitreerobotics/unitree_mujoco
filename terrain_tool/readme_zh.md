#  地形生成工具

## 使用
1. 首先安装依赖
```bash
pip3 install noise opencv-python numpy 
```
2. 打开 terrain_generator.py 修改开头的配置，这里以 Go2 机器人为例
```python

# 机器人目录
ROBOT = "go2"

# 输入的场景文件 
INPUT_SCENE_PATH = "./scene.xml"

# 输出的
OUTPUT_SCENE_PATH = "../unitree_robots/" + ROBOT + "/scene_terrain.xml"
```
3. 运行
```bash
cd terrain_tool
python3 ./terrain_generator.py
```
程序将输出地形场景文件到`/unitree_robots/go2/scene_terrain.xml`。接着可以修改仿真器的配置文件`simulate/config.yaml`，将场景改为刚刚生成的 `scene_terrain.xml`:
```yaml
robot_scene: "scene_terrain.xml"
```
如果使用的是基于 python 的仿真器，则修改`simulate_python/config.py`：
```python
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene_terrain.xml" 
```
之后运行 unitree_mujoco 仿真器，可以看到所生成的地形。

# 函数解释
用户可以利用 `terrain_generator.py` 自行添加所需的地形，以下是函数解释。

##### 1. `AddBox`
添加立方体，参数：
``` python
position=[1.0, 0.0, 0.0] # 中心位置
euler=[0.0, 0.0, 0.0] # 姿态
size=[0.1, 0.1, 0.1] # 尺寸，长宽高
``` 
##### 2. `AddGeometry`
添加几何体, 参数：
``` python
position=[1.0, 0.0, 0.0] # 中心位置
euler=[0.0, 0.0, 0.0] # 姿态
size=[0.1, 0.1, 0.1] # 尺寸，部分几何体只需要用到前两个参数
geo_type="cylinder" # 几何体类型，支持"plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"
``` 

##### 3. `AddStairs`
添加楼梯，参数：
```python
init_pos=[1.0, 0.0, 0.0] # 靠近地面的台阶的位置
yaw=0.0 # 楼梯朝向
width=0.2 # 台阶宽度
height=0.15 # 台阶高度
length=1.5 # 台阶长度
stair_nums=10 #台阶数量
```

##### 4. `AddSuspendStairs`
添加悬浮楼梯，参数：
```python
init_pos=[1.0, 0.0, 0.0] # 靠近地面的台阶的位置
yaw=0.0 # 楼梯朝向
width=0.2 # 台阶宽度
height=0.15 # 台阶高度
length=1.5 # 台阶长度
gap=0.1 # 悬浮间隙
stair_nums=10 # 台阶数量
```

##### 5. `AddRoughGround`
添加杂乱地形，通过随机排列立方体实现杂乱的地形，参数：
```python
init_pos=[1.0, 0.0, 0.0] # 第一个立方体的位置
euler=[0.0, -0.0, 0.0], # 地形相对于世界的欧拉角
nums=[10, 10], # 在 x 和 y 方向上的立方体个数
box_size=[0.5, 0.5, 0.5], # 立方体的尺寸
box_euler=[0.0, 0.0, 0.0], # 立方体的姿态
separation=[0.2, 0.2], # 立方体在 x 和 y 方向上的间隔
box_size_rand=[0.05, 0.05, 0.05], # 立方体尺寸的随机增量
box_euler_rand=[0.2, 0.2, 0.2], # 立方体姿态的随机增量
separation_rand=[0.05, 0.05] # 立方体间隔的随机增量
```

##### 6.`AddPerlinHeighField`
基于 Perlin 噪声生成高程图地形，参数：
```python
position=[1.0, 0.0, 0.0],  # 地形中心位置
euler=[0.0, -0.0, 0.0],  # 地形相对于世界的欧拉角
size=[1.0, 1.0],  # 地形长宽
height_scale=0.2,  # 地形最大高度
negative_height=0.2,  # z 轴负方向高度
image_width=128,  # 地形高程图像素尺寸
img_height=128,
smooth=100.0,  # 噪声平滑度
perlin_octaves=6  # Perlin 噪声参数
perlin_persistence=0.5
perlin_lacunarity=2.0
output_hfield_image="height_field.png" # 输出的高程图名称
```

##### 7. `AddHeighFieldFromImage`
基于给定的图像生成地形
```python
position=[1.0, 0.0, 0.0] # 地形中心位置
euler=[0.0, -0.0, 0.0],  # 地形相对于世界的欧拉角
size=[2.0, 1.6],  # 地形长宽
height_scale=0.02,  # 地形最大高度
negative_height=0.1,  # z 轴负方向高度
input_img="./unitree_robot.jpeg" # 输入的图像路径
output_hfield_image="height_field.png", # 输出的高程图名称
image_scale=[1.0, 1.0],  # 缩放图像比例
invert_gray=False # 反转像素
```