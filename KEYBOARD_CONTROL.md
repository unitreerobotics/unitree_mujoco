# 键盘控制使用说明

## 概述

已为 unitree_mujoco 添加键盘控制功能，无需手柄即可控制机器人。

## 启用键盘控制

编辑 `simulate/config.yaml` 文件：

```yaml
use_joystick: 1  # 启用控制器
joystick_type: "keyboard"  # 使用键盘模式（原来是 "xbox" 或 "switch"）
```

## 键盘映射

### 基础按键对照表

| 键盘按键 | 手柄按键 | 说明 |
|---------|---------|------|
| **左Ctrl** | LB (L1) | 左肩键 |
| **左Shift** | LT (L2) | 左触发器 |
| **右Ctrl** | RB (R1) | 右肩键 |
| **右Shift** | RT (R2) | 右触发器 |
| **A** | A | A 按钮 |
| **B** | B | B 按钮 |
| **X** | X | X 按钮 |
| **Y** | Y | Y 按钮 |
| **1** | F1 | 功能键1 |
| **2** | F2 | 功能键2 |
| **Tab** | Start | 开始键 |
| **Esc** | Select | 选择键 |
| **↑/←/↓/→** | D-pad ↑/←/↓/→ | 方向键 |

### 摇杆控制

| 键盘按键 | 摇杆 | 说明 |
|---------|------|------|
| **I** | 左摇杆 ly +1.0 | 前进 |
| **K** | 左摇杆 ly -1.0 | 后退 |
| **J** | 左摇杆 lx -1.0 | 左移 |
| **L** | 左摇杆 lx +1.0 | 右移 |
| **U** | 右摇杆 rx -1.0 | 左转 |
| **O** | 右摇杆 rx +1.0 | 右转 |

### 组合键示例（对应 IOSDK 遥控器操作）

键盘支持同时按多个键，模拟遥控器组合键操作：

| 键盘组合 | 手柄组合 | UserCommand | 功能 |
|---------|---------|-------------|------|
| **左Ctrl + A** | L1 + A | L1_A | - |
| **左Ctrl + B** | L1 + B | L1_B | charleston_dance |
| **左Ctrl + X** | L1 + X | L1_X | - |
| **左Ctrl + Y** | L1 + Y | L1_Y | - |
| **左Shift + A** | L2 + A | L2_A | - |
| **左Shift + B** | L2 + B | L2_B | CMCC_GPC_dance1 |
| **左Shift + X** | L2 + X | L2_X | - |
| **左Shift + Y** | L2 + Y | L2_Y | CMCC_GPC_dance2 |
| **右Ctrl + A** | R1 + A | R1_A | dancekgswing |
| **右Ctrl + B** | R1 + B | R1_B | dancedzht |
| **右Ctrl + X** | R1 + X | R1_X | dancekaraoke |
| **右Ctrl + Y** | R1 + Y | R1_Y | danceydd |
| **右Shift + A** | R2 + A | R2_A | 行走模式 |
| **右Shift + B** | R2 + B | R2_B | dancebcgm |
| **右Shift + X** | R2 + X | R2_X | dancepower |
| **右Shift + Y** | R2 + Y | R2_Y | dancemojito |
| **左Shift + 1** | L2 + F1 | L2_F1 | 紧急停止(PASSIVE) |
| **1 + A** | F1 + A | F1_A | signal_debug |

### 弹性带控制（Elastic Band，用于吊起机器人）
需要在 `config.yaml` 中设置 `enable_elastic_band: 1`

- **9**: 启用/禁用弹性带
- **7**: 缩短弹性带（吊起机器人）
- **8**: 延长弹性带（释放机器人）

### 其他
- **Backspace**: 重置仿真

### 宏按键

| 按键 | 等效按下 | 说明 |
|------|----------|------|
| **3** | **L2 + D-pad Up** | 同时触发 L2 与 D-pad 上 |
| **4** | **R2 + A** | 同时触发 R2 与 A |

## 使用示例

1. 修改配置文件：
```bash
cd ~/Documents/Code/unitree/unitree_mujoco/simulate
vim config.yaml
```

2. 修改以下行：
```yaml
use_joystick: 1
joystick_type: "keyboard"
```

3. 编译并运行：
```bash
cd build
make -j4
./unitree_mujoco
```

4. 使用键盘控制机器人移动

## 技术实现

### 新增文件
- `simulate/src/keyboard_joystick.h`: 键盘控制类实现

### 修改文件
- `simulate/src/unitree_sdk2_bridge.h`: 添加键盘支持和 `setGLFWWindow()` 方法
- `simulate/src/main.cc`: 添加全局 bridge 指针和窗口设置逻辑

### 核心类
```cpp
class KeyboardJoystick : public unitree::common::UnitreeJoystick
```

该类继承自 `UnitreeJoystick`，通过 GLFW 键盘事件模拟手柄输入。

## 注意事项

1. 键盘控制是数字输入（0 或 1），不像手柄有模拟量
2. 对角线移动会自动归一化，保持速度一致
3. 可以同时按多个键实现组合控制
4. 键盘控制与原有手柄控制互不冲突，可通过配置文件切换

## 作者
Zhang Zhen (zhangzhen@cmhi.chinamobile.com)
