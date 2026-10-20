# 理论力学荣誉课程：平面三连杆机械臂静力学分析

使用虚功原理和四个雅可比矩阵，将三根连杆质心处的重力及末端负载重力转换为关节重力补偿力矩。考虑静摩擦时，计算电机力矩绝对值最小的静态保持解。

## 文件结构

- `model/`：几何、雅可比、静力矩和摩擦模型。
- `visualization/`：关节角扫描和自锁可视化。
- `task1.py`：交互入口。
- `assets/`：结果图片。
- [静力学分析报告](静力学分析报告.md)：模型、方法和图片分析。

## 运行

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python task1.py
```

物理参数可以通过命令行设置；省略时使用任务书基准值：

```bash
python task1.py --lengths 0.40 0.35 0.20 --masses 3.0 2.5 1.0 --load-mass 2.0 --gravity 9.81
```

函数接口同样支持 `lengths`、`masses`、`load_mass`、`gravity` 关键字参数。单位依次为 m、kg、kg、m/s²。函数接收弧度，交互输入使用度。最大静摩擦力矩 `Fc` 的单位为 N·m。

## 结果与分析

下列三幅图左侧为无摩擦力矩，右侧为各关节静摩擦上限为 2 N·m 时的最小保持力矩。扫描的是不同静止位形，不是运动过程。

### 固定关节 1、2，扫描关节 3

![力矩图](assets/torque-with-motor12-fixed.png)

三个关节力矩具有相同的角度变化项和不同的常数偏移。

### 固定关节 1、3，扫描关节 2

![力矩图](assets/torque-with-motor13-fixed.png)

第一、第二关节的无摩擦力矩之差保持不变。

### 固定关节 2、3，扫描关节 1

![力矩图](assets/torque-with-motor23-fixed.png)

整体姿态变化使三个关节的重力补偿力矩呈周期变化。改变一个关节相对角一般会影响多个关节的力矩。

### 自锁位形与静摩擦余量

![自锁空间图](assets/self-lock-3Nm.png)

各关节静摩擦上限均为 3 N·m。自锁指电机输出为零时，静摩擦足以维持当前位形。筛选条件为各关节重力补偿力矩绝对值均不超过静摩擦上限；颜色表示三关节最小余量 `min(Fc - abs(tau_g))`，其中 `tau_g` 是未扣除摩擦的力矩。

黄色表示余量较大，紫色表示接近摩擦极限。图采用每轴 40 点的离散采样，空白处不能直接解释为连续空间中完全不存在自锁位形。

选项 5 输入 `3` 可重建该图。`assets/self-lock.png` 和 `assets/self-lock-2.0Nm.png` 是旧算法留下的历史图片，未用于当前报告。
