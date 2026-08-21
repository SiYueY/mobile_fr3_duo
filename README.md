# Mobile FR3 Duo 原生 MuJoCo 模型设计与生成规范

## 当前正式入口

本仓库发布一个直接、完整的机器人模型和一个环境场景：
`models/mobile_fr3_duo.xml` 与 `models/scene.xml`。FR3、Hand、底盘和传感器
目录是可独立加载的组件资产，不参与 runtime attach 组合。

```bash
python tools/prepare_source.py --franka-root /path/to/franka_description --cache /path/to/cache
python tools/build.py
python tools/validate.py
python tools/render.py
```

唯一冻结 URDF 为 `source/generated/mobile_fr3_duo.urdf`（完整 self-collision）。

## 1. 项目目标与边界

`mobile_fr3_duo` 基于 Franka 官方机器人描述资源构建 Mobile FR3 Duo 的原生 MuJoCo MJCF 模型。

项目的机械数据真源为固定版本的 `frankarobotics/franka_description`。当前基线使用：

```text
franka_description 2.8.1
```

目标机器人组成：

```text
Mobile FR3 Duo v0.2
│
├── TMR v0.2
│   ├── Argo Drive
│   ├── Caster
│   └── Rocker Arm
│
├── Franka Spine v0.1
│
├── FR3 Duo Mount v0.3
│
├── Franka Head v0.2
│
├── Left FR3 v2.1
│   └── Franka Hand
│
└── Right FR3 v2.1
    └── Franka Hand
```

当前模型同时支持传感器增强版本，包括：

* RealSense D455 × 4；
* SICK nanoScan3 × 2；
* OLV-IMU01 × 1；
* ZED Mini × 1；
* FR3、Hand、Spine、TMR 和 Base 状态传感器。

项目目标不是生成一次性的 URDF 转换结果，而是维护一个：

* 可追溯；
* 可重新生成；
* 可自动验证；
* 可独立运行；
* 可持续同步官方模型；

的生产级 MJCF 模型。

当前模型可以描述为：

> 基于 Franka 官方机械资源构建的功能级和动力学级 Mobile FR3 Duo MuJoCo 模型。

当前阶段不声明为：

* 完整数字孪生；
* 已完成 TMR 系统辨识的模型；
* 与真实 Camera/LiDAR 内部算法完全一致的模型；
* 安全认证传感器模型。

---

## 2. 数据来源与模型架构

### 2.1 数据来源

模型数据分为以下类别：

| 类型                 | 含义                       |
| ------------------ | ------------------------ |
| `official-source`  | 与固定官方版本中的源文件完全一致         |
| `official-derived` | 从官方数据通过确定性转换得到           |
| `project-authored` | 本项目编写的模型、配置、脚本或场景        |
| `identified`       | 通过实机测试或系统辨识得到            |
| `estimated`        | 根据公开资料或工程经验估算            |
| `simulation-only`  | 仅服务于 MuJoCo 求解、控制或稳定性的参数 |

以下数据原则上必须以 Franka 官方资源为准：

```text
body/link hierarchy
joint hierarchy
joint origin
joint axis
joint range
velocity limit
effort limit
mass
center of mass
inertia tensor
Spine transform
Duo Mount transform
Head transform
FR3 transform
Hand geometry and joints
official sensor mounting frames
```

以下数据不得错误标记成官方机械参数：

```text
contact friction
solref / solimp
PD gain
sensor noise
simulation spawn clearance
TMR wheel-contact tuning
Hand gripping tuning
rendering parameters
```

项目通过：

```text
source/official_model_files.yaml
source/asset_manifest.yaml
source/parameter_sources.yaml
```

记录版本、SHA-256、资产和非官方参数来源。

### 2.2 Golden Reference

官方 Xacro 展开的完整 URDF 是模型的 **Golden Reference**。

当前生成链：

```text
franka_description
        │
        ▼
Official Xacro / YAML / Mesh
        │
        ▼
source/generated/*.urdf
        │
        ├──────────────┐
        ▼              ▼
URDF parsing        Mesh conversion
        │              │
        └───────┬──────┘
                ▼
          MJCF Builder
                │
                ▼
        Production MJCF
                │
                ▼
          Automated Tests
```

Golden URDF 用于验证：

* body hierarchy；
* joint transform；
* joint axis；
* joint limits；
* mass；
* center of mass；
* inertia；
* visual/collision transform；
* mounting frame。

它不是最终运行时模型。

### 2.3 Runtime 模型

正式 MJCF 必须独立于：

* ROS 2；
* Xacro；
* `package://`；
* 本机绝对路径；
* 用户机器中的第三方源码仓库；
* 运行时模型下载。

正式发布目录复制到另一台安装相同 MuJoCo 版本的机器后，应能直接加载：

```bash
simulate models/scene.xml
```

`scene.xml` 是薄环境层：它通过 `<include>` 引用同目录的
`mobile_fr3_duo.xml`，只增加 Menagerie 风格的渐变天空、棋盘地面、主方向光和
`preview` 相机。机器人、attach、执行器、传感器与 keyframe 始终由机器人模型定义。

---

## 3. 模型生成流程

### 3.1 官方资源固定

正式模型使用固定 Tag/Commit。

版本升级必须作为独立模型变更处理，并重新运行：

```text
official source validation
URDF generation
mesh generation
MJCF generation
kinematics tests
inertia tests
collision tests
sensor tests
stability tests
performance tests
```

不得让生产构建永久依赖：

```text
main
master
humble
jazzy
latest
```

等可变分支。

### 3.2 URDF 生成

`tools/prepare_source.py` 在内部调用官方 Xacro，生成：

```text
source/generated/
├── mobile_fr3_duo.urdf
└── collision_exclusions.yaml
```

生产 MJCF 构建阶段应优先消费这些被冻结的生成结果，而不是重新隐式访问开发者机器上的 `franka_description` checkout。
`collision_exclusions.yaml` 是由固定版本 SRDF 提取并按生成 URDF 过滤后的
disable-collision pair；正式 `python tools/build.py` 只读取这一仓库内输入。
`franka_description` checkout 仅属于 source preparation 阶段。
Source preparation 必须显式传入固定 checkout/cache 位置，例如：

```bash
python tools/prepare_source.py --franka-root /path/to/franka_description --cache /path/to/third-party-cache
```

或设置 `MOBILE_FR3_CACHE_DIR`。任何 production build 命令均不读取该路径。

如果后续 MJCF 构建还需要 SRDF、YAML 或其他官方语义数据，应在 source-generation 阶段将其：

* 固定复制；
* 转换成 manifest；
* 或保存必要的派生结果；

然后由生产 Builder 使用仓库内部输入。

因此推荐依赖关系为：

```text
franka_description
        │
        │ source preparation
        ▼
source/
        │
        │ production build
        ▼
MJCF
```

而不是：

```text
build_models.py
        │
        ├── source/generated/
        │
        └── /home/user/.../franka_description
```

### 3.3 Mesh

Visual 和 Collision 始终分离：

```text
models/<module>/assets/
├── visual/
└── collision/
```

Visual mesh 根据官方资产进行确定性转换。

Collision 优先使用官方 collision geometry，不使用高面数 visual mesh代替碰撞模型。

所有转换步骤记录：

```text
source file
source SHA-256
output file
output SHA-256
conversion tool
tool version
unit conversion
```

一个 link 对应多个 visual mesh 是允许的，不要求强制合并成单个 OBJ。

### 3.4 MJCF 生成

`tools/build.py` 从冻结的官方 URDF 构造 Canonical IR，一次发射可独立
加载的组件模块、唯一完整机器人与 scene。正式模型直接表达全部连接关系，
不包含子模型引用。

```text
Frozen official URDF → Canonical IR → independent component modules
                                      ↓
                         robot configuration → mobile_fr3_duo.xml + scene.xml
```

模块持有并发布自己的 assets；视觉 OBJ 与其多子网格映射仍以
`source/generated/asset_conversion.json` 为事实来源。

每个模块只包含自身的 body subtree、metadata 与 `assets/`；复制一个模块目录
即可加载，且不存在嵌套 `dependencies/`。正式 `mobile_fr3_duo.xml` 直接包含
整机连接关系，发布包保留各模块的 `models/<module>/assets/` 相对资源路径。

---

## 4. MJCF 模型设计

### 4.1 正式模型

当前正式输出位于 `models/`：

```text
mobile_fr3_duo.xml
scene.xml
```

其中：

**`mobile_fr3_duo.xml`**

完整数字孪生模型：

* 完整机械结构；
* 双 Franka Hand；
* motor actuator；
* collision；
* self-collision；
* mounting site；
* state sensors；
* keyframes。
* 4×D455、ZED Mini、IMU 与 2×nanoScan3。

position、reduced 与 planar 是临时构建 profile：使用
`python tools/build_robot.py --variant <profile>` 生成到被 Git 忽略的 `build/`，
不会出现在正式 `models/` 目录。

### 4.2 Robot 与 Scene 分离

机器人文件不得包含：

* ground；
* room；
* table；
* task objects；
* 抓取辅助约束；
* 场景灯光；
* viewer environment。

这些内容属于：

```text
scene*.xml
```

因此直接加载机器人本体时发生自由落体是正常行为，不应通过在 robot XML 中加入隐藏地面解决。

### 4.3 Visual 与 Collision

建议保持 class 分层：

```text
visual
collision
wheel
finger_pad
sensor_collision
```

Visual geom：

```text
contype = 0
conaffinity = 0
```

不得参与物理 collision。

Physical collision 和 self-collision 必须具有明确来源和用途。

相邻 body、官方 SRDF disable-collision pair 和必要的 self-collision shell exclusion 应通过 `<contact>` 显式表达。

不得通过关闭全部 self-collision 解决模型稳定性问题。

### 4.4 Inertia

所有官方 rigid-body：

```text
mass
CoM
inertia tensor
```

均应由 Golden URDF 转换得到。

URDF inertia frame 和 MuJoCo body frame 不一致时，应执行完整 tensor rotation：

```text
I_body = R * I_inertial * R^T
```

测试应比较等价完整 inertia tensor，而不是 XML 字符串是否完全相同。

---

## 5. Actuator、Hand 与移动底盘

### 5.1 Actuator 分层

基础模型使用：

```text
motor
```

而 Position 变体使用：

```text
position
```

机械臂正式控制模型应尽量保持接近：

```text
tau =
    kp * (q_des - q)
  + kd * (dq_des - dq)
  + tau_ff
```

因此生产模型本身不应把所有控制策略永久固化成高增益 `<position>` actuator。

### 5.2 FR3 rotor inertia

Franka 官方提供：

```text
motor_inertia
gear_ratio
```

若采用 joint-side reflected inertia：

```text
J_reflected = J_motor * N²
```

可以作为 MuJoCo armature 建模依据。

MuJoCo 3.9.0 同时支持 joint-level 和 actuator-level armature。

本项目如果采用：

```xml
<motor gear="1" armature="J_reflected"/>
```

则必须在参数来源中明确：

```text
official source:
J_motor
N

official-derived:
J_reflected = J_motor * N²
```

未来如果改为：

```xml
<motor gear="N" armature="J_motor"/>
```

必须同步重新检查 actuator torque semantics，不能只修改 armature。

### 5.3 Franka Hand

双 Franka Hand 来自官方描述。

Finger 联动由 MuJoCo equality/tendon 等机制表达。

上层接口应优先暴露：

```text
gripper width
```

而不是要求调用者分别协调两个 finger。

Hand 中的：

```text
contact friction
actuator force limit
finger-pad solver parameters
```

如果没有官方依据，应明确分类为：

```text
simulation-only
```

或：

```text
identified
```

### 5.4 TMR

正式 Mobile FR3 Duo 模型使用真实机械 wheel/steering/contact 结构。

禁止通过每个周期直接修改：

```text
qpos
qvel
```

伪造正式底盘运动。

Planar proxy 仅存在于 debug variant。

TMR 的：

```text
wheel friction
steering actuator range
wheel actuator range
contact parameters
```

若没有官方实机依据，需要保持来源标记，不得表述成 TMR 官方控制参数。

---

## 6. Sensor 模型

### 6.1 Mechanical Frame 与 Sensor Simulation 分离

来自 Franka 官方的：

```text
camera mounting point
LiDAR mounting point
IMU mounting point
FR3 accelerometer frame
```

属于机器人机械结构。

这些 frame 应保留为：

```xml
<site .../>
```

或者等价 body/frame。

真正的：

```text
camera FOV
resolution
scan pattern
IMU noise
sensor update rate
```

属于传感器模型。

二者不得混为一谈。

### 6.2 当前传感器

当前增强模型包括：

| Sensor         | 数量 |
| -------------- | -: |
| RealSense D455 |  4 |
| nanoScan3      |  2 |
| OLV-IMU01      |  1 |
| ZED Mini       |  1 |

同时提供：

* joint position；
* joint velocity；
* actuator force；
* base pose；
* base linear velocity；
* base angular velocity。

Camera / LiDAR / IMU 的更新频率不与 Physics timestep 强制一致。

例如：

```text
Physics     1000 Hz
D455          30 Hz
nanoScan3     50 Hz
ZED Mini      30 Hz
```

Renderer 或 sensor publisher 应负责调度。

### 6.3 Sensor Transform 验证

传感器安装验证最终不应只使用：

```text
distance < 某个较大阈值
```

而应比较完整：

```text
T_parent_sensor
```

即：

```text
translation
+
rotation
```

官方 mounting frame 应达到 Golden Reference 精度。

当前 P2 对 8 个官方 mounting point（IMU、四个 D455、两个 LiDAR、Head
camera）在每个 keyframe 上，使用冻结的 Golden URDF FK 比较完整世界
`T_parent_sensor`。平移与旋转误差均要求不超过 `1e-6`；旋转通过相对旋转矩阵的
角度计算，不直接比较 quaternion 分量。

项目自行增加的 sensor hardware body 则验证：

```text
official mounting frame
        ×
documented sensor installation transform
```

---

## 7. 生成器、配置与仓库结构

### 7.1 Builder 模块化

`tools/build_modules.py` 负责从 Canonical IR 发射结构模块；
`tools/build_robot.py` 负责完整机器人层的 contact、equality、actuator、sensor、
keyframe 与 variant：

* URDF parsing；
* body generation；
* inertia；
* visual；
* collision；
* self-collision；
* contact exclusion；
* actuator；
* sensor；
* keyframe；
* model variants。

运行时模块目录为：

```text
models/
├── franka_tmr/
├── franka_spine/
├── franka_head/
├── franka_fr3/
├── franka_hand/
├── realsense_d455/
├── imu/
├── nanoscan3/
└── zed_mini/
```

其中：

```text
build_models.py
```

只承担：

```text
CLI
load configuration
construct builder
select variant
write output
```

每个模块同时提供 `*.metadata.yaml`，声明 root body、可挂接接口与来源版本；
组合 YAML 会在生成前校验未知模块、重复 prefix 和错误 mounting point。

### 7.2 配置与算法分离

下列内容不应长期硬编码在 Builder Python 源码：

```text
TMR simulation actuator limits
Hand simulation actuator limits
keyframe qpos
sensor rates
sensor installation parameters
simulation-only contact tuning
```

推荐逐步转移到：

```text
config/
├── control/{motor.yaml,position.yaml}
├── contacts.yaml
├── keyframes.yaml
├── sensor/mobile_fr3_duo.yaml
└── simulation/{default.yaml,reduced.yaml}
```

其中官方参数和项目参数必须继续保持来源分类。

`config/actuator.yaml` 的 `ctrlrange` 是 Builder 写入 MJCF 的 actuator 硬上限。

配置化的目的不是把所有常量都移动到 YAML，而是区分：

```text
生成算法
```

和：

```text
模型数据 / 仿真标定数据
```

### 7.3 Source preparation 与 Production build 分离

推荐形成两个明确阶段：

```text
prepare-source
    ↓
source/generated + manifest

build-model
    ↓
production MJCF
```

`build-model` 不应该需要知道开发机器上：

```text
franka_description
realsense_ros
zed_ros2_description
...
```

分别 clone 在哪里。

`tools/prepare_source.py` 是需要固定 `franka_description@2.8.1` checkout 的 source
preparation 入口；它同时生成 URDF 和 `collision_exclusions.yaml`。mesh conversion、
sensor asset import 与官方文件校验也只在 preparation 阶段使用显式指定的固定 cache。
之后可在没有第三方源码 checkout 的环境中运行：

```bash
python tools/build.py
```

这使得：

```text
CI build
release build
其他开发机器 build
```

拥有完全一致的输入。

### 7.4 仓库卫生

正式根目录只保留有明确用途的：

```text
MJCF
scene
documentation
preview
configuration
source manifests
tools
tests
release files
```

临时 XML、MuJoCo 调试日志等文件不得进入正式 Git 历史。

---

## 8. 自动验证与变更规则

当前模型的可信度主要依赖自动验证，而不是 XML 人工目视检查。

### 8.1 必须保留的测试

至少覆盖：

```text
model load
structure
joint limits
FK
Jacobian
mass/inertia
FR3
Hand
Spine
TMR
contact
initial penetration
odometry
sensor
stability
performance
```

### 8.2 Golden Reference 对比

以下数据必须自动与 Golden URDF 比较：

```text
joint hierarchy
joint origin
joint axis
joint range
mass
CoM
inertia
critical mounting transforms
TCP
```

FK 推荐继续使用多个随机合法姿态，而不是只测试 Home Pose。

### 8.3 Collision

至少验证：

* Home Pose 无深穿透；
* 所有 Keyframe 无非法穿透；
* visual 不参与 contact；
* 官方需要排除的 pair 已排除；
* physical collision 仍保持启用；
* sensor collision 不影响关键机器人运动。

### 8.4 Stability

模型应在无每周期 `qpos/qvel` 修正的情况下稳定运行。

正式动力学测试禁止使用：

```text
hidden weld
planar proxy
unbounded actuator
automatic pose reset
```

来掩盖物理问题。

### 8.5 Performance

目标 timestep：

```text
0.001 s
```

即：

```text
1000 Hz Physics
```

但性能验收应分别记录：

```text
physics only
physics + contacts
physics + sensor scheduling
offscreen camera rendering
LiDAR ray casting
```

避免将 Rendering 开销错误归因于 Physics。

### 8.6 模型变更

以下变化必须重新运行完整验证：

* 更新 `franka_description`；
* 更新 MuJoCo；
* 修改 inertia；
* 修改 collision；
* 修改 contact exclusions；
* 修改 actuator semantics；
* 修改 armature；
* 修改 Hand coupling；
* 修改 TMR contact；
* 修改 sensor transform；
* 修改 model hierarchy。

---

## 9. 当前实施路线

本项目不重新创建新的 Mobile FR3 Duo 模型工程。

现有模型、Golden URDF、转换资产、测试和来源清单均作为当前基线保留。

后续按照以下顺序渐进重构：

```text
现有 v1.0.0
    │
    ▼
移除构建环境耦合
    │
    ▼
清理临时文件
    │
    ▼
冻结 Production Builder 输入
    │
    ▼
拆分 ModelBuilder
    │
    ▼
配置与算法分离
    │
    ▼
加强 Sensor SE(3) 验证
    │
    ▼
继续调整 TMR / Hand / contact 参数
    │
    ▼
实机辨识参数逐步替换 simulation-only 参数
```

第一原则：

> 重构过程中不得以“重新生成一个看起来相同的模型”作为成功标准，必须以现有 Golden Reference 和自动测试作为回归标准。

第二原则：

> XML 文件是否拆成多个模块不是重点；生成代码、数据来源和验证责任是否模块化才是重点。

第三原则：

> Franka 官方数据决定机器人的机械事实，MuJoCo Builder 决定如何表达这些事实，项目配置负责非官方仿真参数，测试负责证明转换没有改变模型语义。
