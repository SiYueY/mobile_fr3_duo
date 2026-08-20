# Mobile FR3 Duo 原生 MuJoCo 高质量仿真模型设计方案

## 1. 文档信息

| 项目           | 内容                                         |
| ------------ | ------------------------------------------ |
| 项目名称         | Mobile FR3 Duo MuJoCo Model                |
| 模型名称         | `mobile_fr3_duo`                           |
| MJCF 根模型名    | `<mujoco model="mobile_fr3_duo">`          |
| 当前机械基础       | Franka 官方 `mobile_fr3_duo_v0_2`            |
| 当前默认末端执行器    | 左右两个 Franka Hand                           |
| 目标 MuJoCo 版本 | MuJoCo 3.9.0 及以上                           |
| 当前目标         | 建立可复用、可维护、稳定且可信的 Mobile FR3 Duo 原生 MJCF 模型 |
| 长期目标         | 尽可能完整实现官方 Vision and Manipulation Kit      |
| 目标质量         | 对齐 `mujoco_menagerie` 的模型组织、质量、文档、许可证和测试标准 |
| 计划首个正式版本     | `v1.0.0`                                   |
| 文档日期         | 2026 年 8 月 7 日                             |
| 适用对象         | MuJoCo 模型、机器人控制、ROS 2、感知、测试和技术评审人员         |

---

# 2. 项目背景

Franka 官方 `franka_description` 已提供 Mobile FR3 Duo 的机械描述基础，包括：

* `mobile_fr3_duo_v0_2` 整机组合；
* `tmrv0_2` 移动底盘；
* `franka_spine_v0_1` 升降机构；
* `fr3_duo_mount_v0_3` 双臂安装架；
* `franka_head_v0_2` 头部安装架；
* 左右两台 `fr3v2_1`；
* `franka_hand`；
* visual mesh；
* collision mesh；
* self-collision geometry；
* 质量、质心和惯性；
* 关节轴、关节限制和安装位姿。

`franka_description 2.8.1` 的官方 Release 包含 `mobile_fr3_duo_v0_2`、`fr3_duo` 和 `fr3v2_1` 等生成模型。官方 `mobile_fr3_duo_v0_2.xacro` 将 TMR、Franka Spine 和 FR3 Duo 组合，并将双臂系统连接到 `franka_spine_mounting_point`。

官方整机组合关系为：

```text
tmrv0_2
└── franka_spine_v0_1
    └── franka_spine_mounting_point
        └── fr3_duo
            ├── fr3_duo_mount_v0_3
            ├── franka_head_v0_2
            ├── left fr3v2_1
            │   └── left Franka Hand
            └── right fr3v2_1
                └── right Franka Hand
```

`fr3_duo.xacro` 固定实例化 FR3 Duo Mount、Franka Head Bracket 和左右两台机械臂，因此 Head Bracket 属于当前官方机械结构的一部分，不应被视为项目自行增加的附件。

本项目不将 URDF 自动转换结果直接作为正式交付物，而是将官方 Xacro、URDF、YAML 和 mesh 作为权威输入，构建经过人工审查和自动测试的原生 MJCF。

---

# 3. 项目定位

## 3.1 当前目标

当前目标是：

> 以 Franka 官方 `franka_description` 中的 `mobile_fr3_duo_v0_2` 为机械基础，尽可能完善其在 MuJoCo 中的结构、动力学、碰撞、控制和传感器能力，形成可复用、可维护、稳定且可信的 Mobile FR3 Duo 原生 MJCF 模型。

当前阶段重点完成：

1. 官方机械结构的完整复现；
2. URDF 到原生 MJCF 的可靠重构；
3. 显式质量和完整惯性；
4. TMR 真实轮地接触；
5. Spine、双 FR3 和双 Franka Hand 动力学；
6. visual、collision 和 self-collision；
7. 官方预留传感器位置；
8. TMR 官方传感器套件及头部相机的 nominal 仿真；
9. 模型来源、版本和许可证管理；
10. Menagerie 风格测试和独立交付。

## 3.2 长期目标

长期目标是：

> 在稳定的 `mobile_fr3_duo` 基础模型之上，逐步实现尽可能接近 Franka 官方 Mobile FR3 Duo Vision and Manipulation Kit 的完整模型变体。

长期扩展主要包括：

* 两个 Robotiq 2F-85；
* 两台 RealSense D405 腕部相机；
* 两套 D405 腕部支架；
* FR3 与 Robotiq 适配结构；
* Vision Kit 配套附件；
* 更真实的相机、LiDAR 和 IMU 误差模型；
* 与真实整机 ROS 2 接口和标定数据的对齐。

长期模型文件规划为：

```text
mobile_fr3_duo_vision_kit.xml
```

Vision and Manipulation Kit 不属于基础模型 `v1.0.0` 的完成条件。

## 3.3 模型能力边界

当前模型应被描述为：

> 基于 Franka 官方机械资源的高质量功能级和动力学级 MuJoCo 仿真模型。

不得描述为：

* 完整实机数字孪生；
* 已完成系统辨识的 TMR 模型；
* 完整 Vision and Manipulation Kit；
* 可验证 nanoScan3 安全认证功能的模型；
* 与 RealSense 或 ZED 内部深度算法完全等价的视觉模型。

---

# 4. 设计原则

## 4.1 官方模型是机械参数真源

以下数据必须来自固定版本的 Franka 官方文件：

* body/link 层次；
* joint 层次；
* joint origin；
* joint axis；
* joint range；
* effort limit；
* velocity limit；
* mass；
* center of mass；
* inertia tensor；
* Spine、Mount、Head 和机械臂安装位姿；
* Franka Hand 几何和关节；
* TMR 传感器 mounting point。

不得根据图片、效果图或第三方模型重新估算这些参数。

## 4.2 最终模型必须是原生 MJCF

正式模型不得依赖：

* 运行时 Xacro；
* 运行时 URDF 转换；
* ROS 2；
* `package://`；
* 本机绝对路径；
* 用户机器中的第三方仓库；
* 运行时下载模型资产。

正式交付目录复制到其他机器后，应可以直接执行：

```bash
simulate scene.xml
```

## 4.3 参数来源必须分类

| 分类                 | 含义                    |
| ------------------ | --------------------- |
| `official-source`  | 与指定官方 Tag 中的文件完全一致    |
| `official-derived` | 从官方文件经过确定性转换得到        |
| `project-authored` | 项目自行编写的 MJCF、配置、场景或脚本 |
| `identified`       | 通过实机测试或系统辨识获得         |
| `estimated`        | 根据公开资料或工程经验估算         |
| `simulation-only`  | 仅用于求解器、接触、控制或调度的仿真参数  |

以下参数不得被误标为官方参数：

* 接触摩擦；
* `solref`、`solimp`；
* PD 增益；
* actuator armature；
* 传感器噪声；
* 网络或控制延迟；
* 轮胎滑移参数；
* 渲染参数。

## 4.4 机器人与任务场景分离

机器人本体文件不得包含：

* 地面；
* 房间；
* 桌子；
* 操作对象；
* 电缆；
* 任务约束；
* 抓取辅助弹簧；
* 吸附；
* 物体跟随；
* 遥操作逻辑。

Menagerie 的模型目录将机器人定义放在 `<model>.xml` 中，地面、灯光和额外对象放在 `scene.xml` 中。

## 4.5 禁止非物理稳定手段

正式模型禁止：

* 每个仿真周期覆盖 `qpos`；
* 每个仿真周期清零 `qvel`；
* 隐藏 weld 锁定活动机构；
* 使用无力限制的超高增益执行器；
* 使用抓取吸附；
* 使用被抓物体跟随；
* 使用外加弹簧维持抓取；
* 使用 X/Y/Yaw 虚拟关节驱动正式底盘；
* 为消除接触问题而关闭全部自碰撞。

---

# 5. 当前模型范围

## 5.1 机械系统

当前基础机械模型包含：

```text
Mobile FR3 Duo
├── TMR v0.2
│   ├── Front Argo Drive
│   ├── Rear Argo Drive
│   ├── Front Caster
│   ├── Rear Caster
│   └── Rocker Arm
├── Franka Spine v0.1
├── FR3 Duo Mount v0.3
├── Franka Head Bracket v0.2
├── Left FR3 v2.1
│   └── Franka Hand
└── Right FR3 v2.1
    └── Franka Hand
```

## 5.2 当前传感器范围

TMR 官方模型提供四个相机 mounting point、两个 LiDAR mounting point 和一个 IMU mounting point。`franka_ros2 v2.5.1` 的 `sensor_suite.xacro` 在这些位置安装四台 D455、一台 OLV-IMU01 和两台 nanoScan3。

当前项目包含：

| 设备                |     数量 | 当前实现目标                           |
| ----------------- | -----: | -------------------------------- |
| RealSense D455    |      4 | 几何、frame、RGB 和理想深度接口             |
| SICK nanoScan3    |      2 | 几何、scan frame 和二维射线扫描            |
| OLV-IMU01         |      1 | 安装 frame、简化几何和理想 IMU             |
| ZED Mini          |      1 | 左右目 frame、nominal baseline 和同步渲染 |
| FR3 joint state   |   14 组 | 位置、速度和 actuator force            |
| Franka Hand state |    4 组 | 位置、速度、force 和开口宽度                |
| TMR joint state   | 全部活动关节 | 转向、轮速和被动机构状态                     |
| Spine state       |    1 组 | 位置、速度和 actuator force            |
| Base state        |    1 组 | 位姿、线速度和角速度                       |

## 5.3 当前不包含

基础 `v1.0.0` 不包含：

* Robotiq 2F-85；
* D405 腕部相机；
* D405 支架；
* Robotiq 适配结构；
* Vision Kit 线缆和供电部件；
* 充电站；
* Franka GELLO Duo。

---

# 6. 模型分层与文件变体

为避免机械基础、传感器和长期 Vision Kit 相互耦合，模型按以下层级维护。

## 6.1 机械基础模型

```text
mobile_fr3_duo.xml
```

包含：

* 完整机械结构；
* 双 Franka Hand；
* actuator；
* collision；
* self-collision；
* mounting site；
* joint 和 base state sensor；
* keyframe。

不实例化外部相机、LiDAR 和 IMU 实体。

## 6.2 当前传感器增强模型

```text
mobile_fr3_duo_with_sensors.xml
```

在基础模型上增加：

* D455 × 4；
* nanoScan3 × 2；
* OLV-IMU01 × 1；
* ZED Mini × 1；
* optical frame；
* scan frame；
* nominal sensor 参数。

该文件属于当前 `v1.0.0` 正式交付物，不是长期 Vision Kit 文件。

## 6.3 位置控制变体

```text
mobile_fr3_duo_position.xml
```

用于：

* Viewer 调试；
* 轨迹回放；
* MoveIt 轨迹验证；
* 功能测试。

## 6.4 Reduced 变体

```text
mobile_fr3_duo_reduced.xml
```

采用官方 reduced TMR，用于性能受限场景，不用于完整底盘动力学验证。

## 6.5 Planar Debug 变体

```text
mobile_fr3_duo_planar_debug.xml
```

仅用于上层软件调试，可使用平面代理关节，但不得纳入正式动力学质量评级。

## 6.6 长期 Vision Kit 变体

```text
mobile_fr3_duo_vision_kit.xml
```

替换双 Franka Hand，增加 Robotiq、D405 和专用支架，作为后续 `v2.0.0` 目标。

---

# 7. 固定官方资源版本

截至 2026 年 8 月 7 日，本项目固定使用：

| 仓库                                  | 固定 Tag   | 用途                                |
| ----------------------------------- | -------- | --------------------------------- |
| `frankarobotics/franka_description` | `2.8.1`  | 整机机械模型和资产                         |
| `frankarobotics/franka_ros2`        | `v2.5.1` | ROS 2 Humble 的 TMR 传感器和控制参考       |
| `realsenseai/realsense-ros`         | `4.58.3` | D455 description、mesh 和 frame     |
| `stereolabs/zed-ros2-description`   | `0.1.5`  | ZED Mini description、mesh 和 frame |
| `SICKAG/sick_safetyscanners2`       | `1.0.5`  | nanoScan3 description 和 mesh      |
| MuJoCo                              | `3.9.0`  | 仿真引擎基线                            |

`franka_description 2.8.1` 是当前最新 Release；`franka_ros2` 同时维护 Jazzy 和 Humble 版本线，本项目固定 Humble 对应的 `v2.5.1`。RealSense、ZED 和 SICK 的对应最新 Tag 分别为 `4.58.3`、`0.1.5` 和 `1.0.5`。

## 7.1 Olive IMU 资源处理

`olive-robotics/olvx_descriptions_module` 当前没有正式 Tag，因此不作为本项目固定 Tag 资源。

当前策略：

1. 通过 `franka_ros2 v2.5.1` 确认设备类型、父 frame 和安装语义；
2. 使用 `franka_description` 中的官方 IMU mounting point；
3. 第一版使用项目自行编写的简化 box visual/collision；
4. IMU 几何分类为 `project-authored`；
5. Olive 发布正式 Tag 后，再独立评估是否替换。

## 7.2 禁止使用可变分支

正式资源不得直接使用：

```text
main
master
humble
jazzy
ros2-development
latest
```

版本升级必须通过独立变更完成，并重新运行全部模型测试。

---

# 8. 官方模型文件验证

## 8.1 验证目的

项目提供：

```text
tools/verify_official_model_files.py
```

该脚本只回答：

> 项目保存或引用的某个源模型文件，是否与固定官方 Tag 中的对应文件完全一致？

## 8.2 验证对象

只验证项目直接复制或引用的官方：

* Xacro；
* URDF；
* YAML；
* DAE；
* STL；
* OBJ；
* 材质文件。

## 8.3 清单文件

```text
source/official_model_files.yaml
```

示例：

```yaml
repositories:
  franka_description:
    repository: https://github.com/frankarobotics/franka_description.git
    tag: "2.8.1"

  franka_ros2:
    repository: https://github.com/frankarobotics/franka_ros2.git
    tag: "v2.5.1"

  realsense_ros:
    repository: https://github.com/realsenseai/realsense-ros.git
    tag: "4.58.3"

  zed_ros2_description:
    repository: https://github.com/stereolabs/zed-ros2-description.git
    tag: "0.1.5"

  sick_safetyscanners2:
    repository: https://github.com/SICKAG/sick_safetyscanners2.git
    tag: "1.0.5"

files:
  - repository: franka_description
    path: robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.xacro
    sha256: "<sha256>"
    component: mobile_fr3_duo_v0_2
```

## 8.4 检查内容

脚本只检查：

1. 仓库地址；
2. Tag 是否存在；
3. 文件是否存在于该 Tag；
4. Tag 中官方文件的 SHA-256；
5. 项目实际使用文件的 SHA-256；
6. 两者是否相同。

状态定义：

| 状态                 | 含义             |
| ------------------ | -------------- |
| `official`         | 与固定官方 Tag 完全一致 |
| `modified`         | 项目文件内容被修改      |
| `missing`          | 项目或官方 Tag 中不存在 |
| `wrong-tag`        | 文件来自其他版本       |
| `wrong-repository` | 来源仓库错误         |
| `manifest-error`   | 清单信息错误         |

该脚本不验证：

* MJCF 是否正确；
* URDF 到 MJCF 转换是否正确；
* 派生 mesh 是否正确；
* 惯性和运动学是否一致；
* 模型是否稳定。

项目编写的 `mobile_fr3_duo.xml` 和派生 mesh 不得标记为官方原文件。

---

# 9. Menagerie 质量标准

MuJoCo Menagerie 的单模型目录通常包含：

```text
assets/
LICENSE
README.md
CHANGELOG.md
<model>.xml
scene.xml
<model>.png
```

Menagerie 要求模型文件仅描述模型本体，并采用统一 XML 风格，包括两空格缩进、双引号、自闭合空元素、合理使用 default class 和提供 `scene.xml`。其检查流程包括格式、许可证、XML、结构和仿真测试。

Menagerie 的模型质量等级为：

| 等级 | 定义                |
| -- | ----------------- |
| A+ | 参数来自正式系统辨识        |
| A  | 参数现实可信，但未完成正式系统辨识 |
| B  | 模型稳定，但部分参数不够真实    |
| C  | 仅条件稳定，仍需显著改进      |

本项目目标：

| 子系统         | `v1.0.0` 目标 | 长期目标 |
| ----------- | ----------: | ---: |
| 几何与运动学      |           A |    A |
| 官方惯性        |           A |    A |
| FR3 关节动力学   |          B+ |    A |
| Franka Hand |           B |    A |
| Spine       |           B |    A |
| TMR 轮地动力学   |           B |    A |
| 理想传感器       |          A- |    A |
| 真实传感器误差     |       C/不评级 |  B/A |
| 整体模型        |           B |    A |

没有实机系统辨识数据时，不声明 A+。

---

# 10. 推荐目录结构

```text
mobile_fr3_duo/
├── assets/
│   ├── tmrv0_2/
│   ├── franka_spine_v0_1/
│   ├── fr3_duo_mount_v0_3/
│   ├── franka_head_v0_2/
│   ├── fr3v2_1/
│   ├── franka_hand/
│   └── sensors/
│       ├── realsense_d455/
│       ├── sick_nanoscan3/
│       ├── olv_imu01/
│       └── zed_mini/
│
├── config/
│   ├── control/
│   │   ├── tmr_control.yaml
│   │   ├── spine_control.yaml
│   │   ├── arm_control.yaml
│   │   └── hand_control.yaml
│   └── sensors/
│       ├── simulation_default.yaml
│       ├── official_runtime_reference.yaml
│       ├── navigation.yaml
│       ├── full_perception.yaml
│       └── profiles/
│           ├── d455.yaml
│           ├── nanoscan3.yaml
│           ├── imu.yaml
│           └── zed_mini.yaml
│
├── source/
│   ├── official_model_files.yaml
│   ├── parameter_sources.yaml
│   ├── link_manifest.yaml
│   ├── joint_manifest.yaml
│   ├── inertial_manifest.yaml
│   ├── frame_manifest.yaml
│   ├── asset_manifest.yaml
│   ├── name_mapping.yaml
│   └── generated/
│       ├── mobile_fr3_duo_visual.urdf
│       └── mobile_fr3_duo_self_collision.urdf
│
├── tools/
│   ├── generate_urdf.sh
│   ├── extract_urdf_parameters.py
│   ├── convert_visual_meshes.py
│   ├── convert_collision_meshes.py
│   ├── build_model.py
│   ├── generate_contact_excludes.py
│   ├── verify_official_model_files.py
│   ├── validate_assets.py
│   ├── format_xml.py
│   └── render_preview.py
│
├── examples/
│   ├── tmr_swerve_controller.py
│   ├── spine_position_controller.py
│   ├── dual_arm_joint_controller.py
│   ├── hand_width_controller.py
│   └── sensor_viewer.py
│
├── tests/
│   ├── test_load.py
│   ├── test_names.py
│   ├── test_structure.py
│   ├── test_frames.py
│   ├── test_kinematics.py
│   ├── test_jacobian.py
│   ├── test_inertials.py
│   ├── test_joint_limits.py
│   ├── test_contacts.py
│   ├── test_self_collision.py
│   ├── test_mobile_base.py
│   ├── test_odometry.py
│   ├── test_spine.py
│   ├── test_arms.py
│   ├── test_hands.py
│   ├── test_sensor_mounts.py
│   ├── test_cameras.py
│   ├── test_lidars.py
│   ├── test_imu.py
│   ├── test_sensor_timing.py
│   ├── test_stability.py
│   └── test_performance.py
│
├── CHANGELOG.md
├── LICENSE
├── README.md
├── pyproject.toml
├── Makefile
├── mobile_fr3_duo.png
├── mobile_fr3_duo.xml
├── mobile_fr3_duo_with_sensors.xml
├── mobile_fr3_duo_position.xml
├── mobile_fr3_duo_reduced.xml
├── mobile_fr3_duo_planar_debug.xml
├── scene.xml
└── scene_with_sensors.xml
```

正式交付目录不得依赖开发期 `third_party/` 路径。

---

# 11. 模型构建流程

## 11.1 获取固定官方资源

所有资源从第 7 章指定的 Tag 获取。

## 11.2 生成官方 URDF

生成：

```text
source/generated/mobile_fr3_duo_visual.urdf
source/generated/mobile_fr3_duo_self_collision.urdf
```

配置要求：

```text
robot_types = [tmrv0_2, fr3v2_1, fr3v2_1]
hand = true
reduced_version = false
gazebo = false
with_sc = false / true
```

## 11.3 提取模型参数

提取：

* link；
* joint；
* parent-child；
* origin；
* axis；
* range；
* effort；
* velocity；
* damping；
* mass；
* COM；
* inertia；
* visual；
* collision；
* frame；
* mounting point。

## 11.4 原生 MJCF 重构

正式模型需要重新组织：

* `compiler`；
* `option`；
* `default`；
* `asset`；
* `worldbody`；
* `contact`；
* `equality`；
* `actuator`；
* `sensor`；
* `keyframe`。

## 11.6 自动验证

正式模型更新前必须通过：

* 加载；
* 结构；
* FK；
* Jacobian；
* 惯性；
* 关节限制；
* 碰撞；
* 底盘；
* Spine；
* 双臂；
* Hand；
* 传感器；
* 稳定性；
* 性能测试。

---

# 12. MJCF 顶层结构与默认配置

```xml
<mujoco model="mobile_fr3_duo">
  <compiler/>
  <option/>
  <size/>
  <visual/>
  <statistic/>

  <default>
    ...
  </default>

  <asset>
    ...
  </asset>

  <worldbody>
    ...
  </worldbody>

  <contact>
    ...
  </contact>

  <equality>
    ...
  </equality>

  <actuator>
    ...
  </actuator>

  <sensor>
    ...
  </sensor>

  <keyframe>
    ...
  </keyframe>
</mujoco>
```

初始配置：

```xml
<compiler
  angle="radian"
  meshdir="assets"
  autolimits="true"
  inertiafromgeom="false"
  fusestatic="false"/>

<option
  timestep="0.001"
  integrator="implicitfast"
  solver="Newton"
  iterations="50"
  ls_iterations="10"
  gravity="0 0 -9.81"/>
```

原则：

1. 所有有质量 body 显式提供 inertial；
2. 不使用 `balanceinertia` 掩盖错误；
3. 默认物理周期为 1 ms；
4. Viewer 和传感器周期与物理周期解耦；
5. solver 参数由稳定性和性能测试确定；
6. XML 格式遵循 Menagerie 风格。

MuJoCo 原生支持显式惯性、motor、position actuator、camera、关节传感器和 IMU 传感器。

---

# 13. TMR v0.2 移动底盘设计

## 13.1 官方结构

官方 TMR 包含：

* `base_link`；
* `base_inertia`；
* 前后 Argo Drive；
* Caster；
* Rocker Arm；
* 完整版和 reduced 版；
* 传感器 mounting point。

官方文件给出了底盘主体质量和完整惯性，并定义 wheel、caster 和 rocker 的基础几何关系。

正式模型采用：

```text
reduced_version = false
```

## 13.2 根节点

```xml
<body name="base_link">
  <freejoint name="base_freejoint"/>
  ...
</body>
```

正式底盘必须通过轮地接触运动。

## 13.3 主动关节

* Argo steering joint；
* Argo drive wheel joint。

## 13.4 被动关节

* Caster steering；
* Caster rolling；
* Rocker Arm；
* 官方定义的其他被动机构。

被动关节不得配置主动 position actuator。

## 13.5 执行器

正式模型使用 motor：

```xml
<motor
  name="argo_drive_front_steering_motor"
  joint="argo_drive_front_steering_joint"
  gear="1"
  ctrllimited="true"
  ctrlrange="..."/>

<motor
  name="argo_drive_front_wheel_motor"
  joint="argo_drive_front_wheel_joint"
  gear="1"
  ctrllimited="true"
  ctrlrange="..."/>
```

控制链：

```text
vx、vy、wz
   ↓
Swerve / Argo inverse kinematics
   ↓
steering target + wheel target
   ↓
MuJoCo actuator
   ↓
wheel-ground contact
```

## 13.6 轮地接触

要求：

* wheel visual 与 collision 分离；
* collision 使用 cylinder 或简化凸体；
* 不使用高面数 visual mesh 碰撞；
* 验证轮半径、轮轴和滚动方向；
* 分别标定纵向牵引和侧向滑移；
* 不使用异常大驱动力掩盖摩擦问题；
* 不使用平面代理驱动正式模型。

轮地参数第一版分类为 `simulation-only`，获得实机数据后转为 `identified`。

---

# 14. Franka Spine 设计

Spine 使用 Z 轴 prismatic joint：

```xml
<joint
  name="franka_spine_vertical_joint"
  type="slide"
  axis="0 0 1"
  range="0 0.85"
  limited="true"/>
```

正式执行器：

```xml
<motor
  name="franka_spine_motor"
  joint="franka_spine_vertical_joint"
  gear="1"
  ctrllimited="true"
  ctrlrange="-100 100"/>
```

位置控制在外部控制器实现。

验收要求：

* 双臂负载下可升降；
* 全行程可控；
* 不超过速度限制；
* 上下限无持续振荡；
* 任意合法高度可保持；
* 不覆盖 `qpos`；
* 不依赖异常大的 damping。

---

# 15. FR3 Duo Mount 与 Head Bracket

要求：

* 保留 Mount 和 Cover；
* 保留官方 inertial；
* 保留左右机械臂安装位姿；
* 保留 Head Bracket；
* 保留 `head_camera_mounting_point`；
* 不手工改变双臂夹角；
* 头部相机必须相对 Head Bracket 安装。

---

# 16. 双 FR3 v2.1 设计

## 16.1 运动学

必须保留：

* 左右前缀；
* 14 个 revolute joint；
* joint origin；
* joint axis；
* joint range；
* effort limit；
* velocity limit；
* flange；
* hand attachment；
* TCP；
* link inertial。

## 16.2 惯性

```xml
<inertial
  pos="..."
  quat="..."
  mass="..."
  fullinertia="ixx iyy izz ixy ixz iyz"/>
```

禁止：

* 丢弃非对角项；
* 使用 Panda 参数替换 FR3 v2.1；
* 从 visual mesh 推导官方 link 惯性；
* 为消除 warning 随意增加质量。

## 16.3 执行器

```xml
<motor
  name="left_fr3v2_1_joint1_motor"
  joint="left_fr3v2_1_joint1"
  gear="1"
  ctrllimited="true"
  ctrlrange="..."/>
```

位置、速度、阻抗、重力补偿和 whole-body 控制在外部实现。

---

# 17. 双 Franka Hand 设计

每个 Hand 包含：

* hand body；
* hand TCP；
* 左右 finger；
* 两个 prismatic joint；
* visual；
* collision；
* self-collision；
* finger inertial。

## 17.1 两指联动

```xml
<equality>
  <joint
    name="left_hand_finger_coupling"
    joint1="left_fr3v2_1_finger_joint1"
    joint2="left_fr3v2_1_finger_joint2"
    polycoef="0 1 0 0 0"/>
</equality>
```

具体符号必须根据关节轴验证。

## 17.2 控制接口

```text
left_hand_width
right_hand_width
```

定义：

```text
width = q_finger1 + q_finger2
```

开口范围：

```text
0～0.08 m
```

## 17.3 指尖接触

```xml
<default class="finger_pad">
  <geom
    condim="4"
    friction="..."
    solref="..."
    solimp="..."/>
</default>
```

禁止使用：

* 吸附；
* hidden weld；
* 物体跟随；
* 抓取辅助弹簧。

---

# 18. 传感器安装基准

TMR 官方 mounting point 如下。

| 名称                           | XYZ                       | RPY        |
| ---------------------------- | ------------------------- | ---------- |
| `imu_mounting_point`         | `0.260 0 0.1478`          | `π 0 0`    |
| `front_mounting_point`       | `0.380705 0 0.2345`       | `π 0 0`    |
| `rear_mounting_point`        | `-0.380705 0 0.2345`      | `π 0 π`    |
| `right_mounting_point`       | `0 -0.272712 0.1145`      | `π 0 -π/2` |
| `left_mounting_point`        | `0 0.272712 0.1145`       | `π 0 π/2`  |
| `lidar_front_mounting_point` | `0.3275 0.2175 0.19065`   | `0 π 3π/4` |
| `lidar_rear_mounting_point`  | `-0.3275 -0.2175 0.19065` | `0 π 7π/4` |

无质量、无几何的 mounting link 可以转换为 site，但必须：

* 保留官方名称；
* 保持相同变换；
* 支持自动比较；
* 不再手写另一套外参。

---

# 19. RealSense D455 设计

实例名称：

```text
camera_front
camera_rear
camera_left
camera_right
```

每台 D455 至少保留：

* device link；
* color frame；
* color optical frame；
* depth frame；
* depth optical frame；
* infra1 frame；
* infra1 optical frame；
* infra2 frame；
* infra2 optical frame。

示例：

```xml
<body name="camera_front_link" pos="..." quat="...">
  <inertial .../>
  <geom class="visual" mesh="d455_visual"/>
  <geom class="sensor_collision" type="box" size="..."/>

  <site name="camera_front_color_optical_frame" .../>
  <site name="camera_front_depth_optical_frame" .../>
  <site name="camera_front_infra1_optical_frame" .../>
  <site name="camera_front_infra2_optical_frame" .../>

  <camera name="camera_front_color" .../>
  <camera name="camera_front_depth" .../>
</body>
```

项目区分：

```text
official_runtime_reference.yaml
simulation_default.yaml
```

前者记录 Franka ROS 2 参考配置，后者定义 MuJoCo RGB、depth、分辨率和更新率。

当前模型不复现：

* RealSense 主动红外双目算法；
* 深度空洞；
* 自动曝光；
* 镜面和透明材质失效；
* 设备逐台标定差异。

---

# 20. SICK nanoScan3 设计

实例：

```text
lidar_front
lidar_rear
```

实现内容：

* device body；
* visual；
* collision；
* scan origin；
* scan frame；
* 批量 raycast；
* 时间戳；
* `ranges`；
* 可配置噪声。

运行时使用 `mj_multiRay` 批量发射扫描射线；MuJoCo 官方 API 提供 `mj_ray` 和 `mj_multiRay`。

项目默认参考参数：

```yaml
update_rate: 50.0
angle_min: -0.7853981634
angle_max: 3.9269908170
range_min: 0.05
range_max: 10.0
gaussian_noise: 0.006
max_beams: 500
```

模型不复现：

* 安全认证逻辑；
* protective field；
* 多回波；
* 玻璃和镜面行为；
* 雨雾；
* 内部安全滤波。

该模型不得用于安全认证验证。

---

# 21. OLV-IMU01 设计

当前实现使用：

* 官方 TMR mounting point；
* `franka_ros2` 中的 OLV-IMU01 安装语义；
* 项目简化 visual/collision；
* MuJoCo 原生 IMU sensor。

```xml
<body name="imu_link" pos="..." quat="...">
  <geom class="visual" type="box" size="..."/>
  <geom class="sensor_collision" type="box" size="..."/>
  <site name="imu_sensor_frame" size="0.002"/>
</body>

<sensor>
  <gyro
    name="imu_angular_velocity"
    site="imu_sensor_frame"/>
  <accelerometer
    name="imu_linear_acceleration"
    site="imu_sensor_frame"/>
  <framequat
    name="imu_orientation"
    objtype="site"
    objname="imu_sensor_frame"/>
</sensor>
```

提供：

```text
ideal
estimated_realistic
```

在没有实机测试数据时，bias、random walk 和温漂必须标记为 `estimated`。

---

# 22. ZED Mini 设计

ZED Mini 安装在：

```text
head_camera_mounting_point
```

保留：

* ZED body；
* left camera frame；
* left optical frame；
* right camera frame；
* right optical frame；
* center frame；
* nominal stereo baseline。

MJCF 使用两个独立相机：

```xml
<camera name="head_zed_left" .../>
<camera name="head_zed_right" .../>
```

当前实现目标：

* 几何正确；
* 左右 frame 正确；
* nominal baseline 正确；
* 同一仿真快照同步渲染；
* 支持左右 RGB；
* 支持外部理想 stereo depth。

当前不实现：

* ZED SDK 深度算法；
* 神经网络深度；
* 真实置信度；
* 自动曝光；
* 镜头畸变；
* 逐台标定差异。

---

# 23. 状态传感器

每个活动关节至少定义：

```xml
<jointpos name="..." joint="..."/>
<jointvel name="..." joint="..."/>
```

主动关节可以增加：

```xml
<jointactuatorfrc name="..." joint="..."/>
```

对外状态语义必须区分：

```text
tau_command
tau_actuator
tau_constraint
tau_external_estimate
tau_bias
```

`jointactuatorfrc` 不得直接解释为真实 FR3 的完整外部力矩测量。

---

# 24. Visual、Collision 与 Inertial

## 24.1 Visual

官方 DAE 使用固定脚本转换为 OBJ，记录：

* 官方仓库和 Tag；
* 源路径；
* 输入 SHA-256；
* 工具版本；
* 单位；
* 坐标轴；
* 变换操作；
* 输出 SHA-256。

## 24.2 Collision 优先级

1. 官方 primitive；
2. 官方 self-collision capsule；
3. 官方 collision STL；
4. 对官方 mesh 进行凸分解；
5. 经验证的 box、capsule、sphere 近似。

禁止：

* 用完整底盘 visual mesh 直接碰撞；
* 用 FR3 高面数 visual mesh 自碰撞；
* 使用未经检查的非封闭 mesh；
* 全局放大 collision 解决穿透。

## 24.3 Default Class

```xml
<default class="visual">
  <geom
    contype="0"
    conaffinity="0"
    group="2"
    density="0"/>
</default>

<default class="collision">
  <geom
    group="3"
    contype="1"
    conaffinity="1"
    condim="4"
    friction="..."/>
</default>

<default class="sensor_collision">
  <geom
    group="4"
    contype="1"
    conaffinity="1"
    density="0"/>
</default>
```

---

# 25. 自碰撞与接触过滤

自碰撞来源优先级：

1. 官方 `--with-sc` 模型；
2. 官方 SRDF；
3. direct parent-child 关系；
4. 随机姿态碰撞扫描；
5. 人工审查。

默认可以排除：

```text
direct parent ↔ direct child
```

不得全局排除：

* 左右机械臂；
* 左右 Hand；
* Hand 与对侧机械臂；
* 机械臂与 Spine；
* 机械臂与 Mount；
* 机械臂与 TMR；
* 机械臂与传感器；
* wheel 与 floor。

所有 keyframe 必须无非预期初始穿透。

---

# 26. Keyframe

至少提供：

```text
home
transport
manipulation
wide_workspace
spine_min
spine_max
```

要求：

* 无自碰撞；
* 无 joint limit violation；
* steering 初值合理；
* Hand 开口明确；
* actuator ctrl 与 qpos 相匹配；
* 加载后无大幅控制阶跃。

---

# 27. 传感器运行时架构

```text
Physics thread
└── 独占主 mjData
    └── 生成 SimulationSnapshot

Camera workers
└── 独立 mjData 和渲染上下文

LiDAR worker
└── 基于快照执行 mj_multiRay

State sensor worker
└── IMU、joint、base state

Publisher/API layer
└── 输出数据
```

`SimulationSnapshot` 建议包含：

```text
simulation_time
qpos
qvel
actuator_force
body_pose
mocap_state
sensor_reference_state
```

禁止：

* Camera、Viewer 和 Physics 线程并发写同一个 `mjData`；
* 每个物理步渲染全部相机；
* 相机和 LiDAR阻塞物理线程；
* 将传感器频率绑定到 1 kHz。

建议频率：

| 子系统              |        建议频率 |
| ---------------- | ----------: |
| Physics          |     1000 Hz |
| FR3 controller   |     1000 Hz |
| TMR controller   | 500～1000 Hz |
| Spine controller | 500～1000 Hz |
| Joint state      | 500～1000 Hz |
| LiDAR            |       50 Hz |
| D455             |       30 Hz |
| ZED Mini         |  30 或 60 Hz |
| Viewer           |    30～60 Hz |

---

# 28. 自动化测试体系

## 28.1 加载测试

验证：

```text
mobile_fr3_duo.xml
mobile_fr3_duo_with_sensors.xml
mobile_fr3_duo_position.xml
mobile_fr3_duo_reduced.xml
mobile_fr3_duo_planar_debug.xml
scene.xml
scene_with_sensors.xml
```

要求：

* 无缺失资产；
* 无重复名称；
* 无非法 inertia；
* 无不存在的 joint、site 或 body 引用；
* 无 keyframe 长度错误。

## 28.2 结构一致性

比较 URDF 与 MJCF：

* joint 数量；
* 名称；
* 类型；
* parent-child；
* axis；
* range；
* effort；
* velocity；
* TCP；
* mounting point。

## 28.3 正向运动学

随机至少 1000 组合法状态，对比 Pinocchio/KDL 与 MuJoCo：

| 项目          |       最大误差 |
| ----------- | ---------: |
| 位置          |   `1e-6 m` |
| 姿态          | `1e-6 rad` |
| joint axis  |     `1e-9` |
| joint range |    `1e-10` |

## 28.4 Jacobian

比较：

* MuJoCo Jacobian；
* Pinocchio/KDL Jacobian；
* 中心差分 Jacobian。

建议阈值：

```text
max absolute error < 1e-5
```

## 28.5 惯性

逐 body 比较：

* mass；
* COM；
* full inertia；
* eigenvalue；
* total mass。

要求：

* 惯性矩阵对称；
* 正定；
* 满足刚体惯性约束；
* 不依赖自动平衡修正。

## 28.6 TMR 测试

* 前进；
* 后退；
* 横移；
* 斜移；
* 原地旋转；
* 圆周；
* steering 连续性；
* Caster 跟随；
* Rocker 接地；
* 静止漂移；
* 制动；
* 斜坡；
* wheel slip。

必须证明运动来自 wheel-ground contact。

## 28.7 Spine 测试

* 空载；
* 双臂负载；
* 最大高度；
* 最低高度；
* 速度限制；
* 上下限；
* 多种双臂姿态；
* 60 秒高度保持。

## 28.8 双臂测试

* home；
* 随机合法姿态；
* 重力补偿；
* joint-space PD；
* torque limit；
* 双臂同步运动；
* 双臂互碰；
* 与 Mount、Spine 和底盘碰撞。

## 28.9 Franka Hand 测试

* 开合；
* 两指同步；
* 开口宽度；
* force limit；
* 方块抓取；
* 圆柱抓取；
* 薄片抓取；
* 抓取后升降；
* 抓取后底盘移动；
* 滑移；
* 过载释放。

## 28.10 传感器测试

数量：

```text
D455 = 4
nanoScan3 = 2
IMU = 1
ZED stereo pair = 1
FR3 joints = 14
finger joints = 4
```

验证：

* mounting transform；
* optical frame；
* camera direction；
* RGB；
* metric depth；
* stereo baseline；
* LiDAR FoV；
* LiDAR range；
* IMU 三轴方向；
* 时间戳；
* 更新周期。

## 28.11 稳定性测试

以下 keyframe 分别运行至少 60 秒：

* home；
* transport；
* manipulation；
* spine_min；
* spine_max。

完整系统另执行至少 10 分钟集成测试。

要求：

* 无 NaN；
* 无 Inf；
* 无无界能量增长；
* 无接触爆炸；
* 无异常底盘漂移；
* 无持续 joint limit 振荡。

## 28.12 性能测试

记录：

* `nbody`；
* `njnt`；
* `nv`；
* `nu`；
* `ngeom`；
* 平均 step 时间；
* P95 step 时间；
* 实时因子；
* Camera render 时间；
* LiDAR raycast 时间；
* snapshot 复制时间。

性能回退超过 10% 时触发 CI 警告。

---

# 29. CI 与质量门禁

建议 Makefile：

```makefile
install:
	uv sync
	pre-commit install

verify-official-files:
	uv run python tools/verify_official_model_files.py

check:
	uv run ruff check .
	uv run python tools/format_xml.py --check
	uv run python tools/validate_assets.py
	uv run python tools/verify_official_model_files.py

test:
	uv run pytest -v

all: check test
```

CI 失败条件：

* 官方文件验证失败；
* 缺少许可证；
* XML 格式错误；
* 重复名称；
* FK 超差；
* Jacobian 超差；
* 惯性非法；
* keyframe 穿透；
* 仿真出现 NaN；
* 传感器数量错误；
* mounting point 错误；
* 正式底盘依赖 planar joint；
* 存在状态覆盖或抓取辅助；
* 性能严重回退。

---

# 30. README 与发布文档要求

README 必须包含：

1. 项目定位；
2. 当前范围；
3. 长期 Vision Kit 目标；
4. 固定官方版本；
5. MuJoCo 最低版本；
6. 官方源文件列表；
7. URDF 生成方法；
8. mesh 转换方法；
9. collision 处理；
10. actuator 类型；
11. sensor 列表；
12. keyframe；
13. 运行方法；
14. 测试方法；
15. 已知限制；
16. 许可证；
17. 参数来源分类；
18. 与真实机器人的差异；
19. 模型质量等级。

还需提供：

```text
CHANGELOG.md
LICENSE
parameter_sources.yaml
官方资源清单
性能测试报告
稳定性测试报告
```

---

# 31. 版本规划

| 版本       | 内容                          |
| -------- | --------------------------- |
| `v0.1.0` | 官方资源、URDF 和来源清单            |
| `v0.2.0` | 完整机械结构和 visual              |
| `v0.3.0` | 惯性、限制、actuator 和运动学测试       |
| `v0.4.0` | Collision 和 self-collision  |
| `v0.5.0` | TMR 动力学                     |
| `v0.6.0` | Spine、双 FR3 和双 Hand         |
| `v0.7.0` | TMR 传感器和 ZED Mini           |
| `v0.9.0` | Release Candidate           |
| `v1.0.0` | 当前范围正式发布                    |
| `v2.0.0` | Vision and Manipulation Kit |

---

# 32. 开发阶段详细说明

## 32.1 阶段总览

```text
阶段 0：项目初始化与官方资源审计
阶段 1：官方模型展开与中间数据基线
阶段 2：原生 MJCF 机械结构
阶段 3：运动学、惯性、限制与执行器
阶段 4：Collision、自碰撞与接触
阶段 5：TMR 移动底盘动力学
阶段 6：Spine、双 FR3 与双 Franka Hand
阶段 7：传感器实体、接口与运行时
阶段 8：系统集成与 Menagerie 级交付
阶段 9：Vision and Manipulation Kit 长期扩展
```

阶段 0～8 构成 `v1.0.0` 的完整开发范围。

---

## 32.2 阶段 0：项目初始化与官方资源审计

### 目标

建立可重复、可追溯的开发环境，固定官方资源版本，确认模型文件和许可证。

### 任务

1. 创建仓库和目录；
2. 配置 Python、MuJoCo、pytest、ruff、pre-commit；
3. 获取所有固定 Tag；
4. 建立 `official_model_files.yaml`；
5. 实现官方文件验证脚本；
6. 完成许可证审计；
7. 建立初始 CI；
8. 建立项目 README 和 CHANGELOG。

### 交付物

```text
official_model_files.yaml
verify_official_model_files.py
LICENSE
README.md
CHANGELOG.md
Makefile
pyproject.toml
CI 配置
```

### 验收

* 固定 Tag 均可获取；
* 官方文件验证通过；
* 许可证明确；
* 开发依赖可安装；
* CI 可运行；
* 无来源不明资产。

---

## 32.3 阶段 1：官方模型展开与数据基线

### 目标

生成完整 URDF，并固定后续开发所需的结构、命名和参数基线。

### 任务

1. 生成 visual URDF；
2. 生成 self-collision URDF；
3. 提取 link、joint、inertial 和 frame；
4. 建立命名映射；
5. 记录模型统计；
6. 建立参数和资产清单。

### 交付物

```text
mobile_fr3_duo_visual.urdf
mobile_fr3_duo_self_collision.urdf
link_manifest.yaml
joint_manifest.yaml
inertial_manifest.yaml
frame_manifest.yaml
asset_manifest.yaml
name_mapping.yaml
```

### 验收

* URDF 可解析；
* 所有 mesh 存在；
* 所有 joint 名称唯一；
* 所有活动 joint 有轴和范围；
* 所有有质量 link 有惯性；

---

## 32.4 阶段 2：原生 MJCF 机械结构

### 目标

完成结构和外观正确的原生 MJCF。

### 任务

1. 建立 MJCF 顶层结构；
2. 重建 TMR body tree；
3. 重建 Spine；
4. 重建 Duo Mount 和 Head；
5. 重建双 FR3；
6. 重建双 Franka Hand；
7. 转换 visual mesh；
8. 将纯 frame 转换为 site；
9. 建立初始 keyframe；
10. 创建 scene 和预览。

### 交付物

```text
mobile_fr3_duo.xml 初始版本
assets/
scene.xml
mobile_fr3_duo.png
test_structure.py
test_names.py
test_frames.py
```

### 验收

* MJCF 无错误加载；
* body tree 与 URDF 一致；
* 全部活动 joint 存在；
* 左右机械臂方向正确；
* Spine、Mount、Head 和 Hand 位姿正确；
* mounting point 正确；
* visual 与官方模型一致。

---

## 32.5 阶段 3：运动学、惯性、限制与执行器

### 目标

使模型在运动学、惯性和控制边界上与官方描述一致。

### 任务

1. 转换所有 mass、COM 和 full inertia；
2. 配置 joint range、effort、velocity 和 damping；
3. 为主动关节增加 motor；
4. 建立 position 变体；
5. 配置 Hand equality；
6. 增加 joint state sensor；
7. 完成 FK 测试；
8. 完成 Jacobian 测试；
9. 完成惯性和限制测试。

### 交付物

```text
mobile_fr3_duo.xml 动力学基线
mobile_fr3_duo_position.xml
parameter_sources.yaml
test_kinematics.py
test_jacobian.py
test_inertials.py
test_joint_limits.py
```

### 验收

* 1000 组随机姿态 FK 通过；
* Jacobian 通过；
* 惯性正定；
* 总质量一致；
* joint range 一致；
* actuator 有明确限幅；
* Hand 两指同步；
* 无 NaN。

---

## 32.6 阶段 4：Collision、自碰撞与接触

### 目标

建立稳定、合理且计算量可控的碰撞系统。

### 任务

1. 建立 visual、collision、self-collision default class；
2. 转换 collision mesh；
3. 检查 mesh 闭合和退化三角形；
4. 使用官方 self-collision geometry；
5. 自动生成 contact exclude 候选；
6. 人工审查 exclude；
7. 检查 keyframe 初始穿透；
8. 执行随机姿态碰撞扫描；
9. 标定 finger pad 接触；
10. 建立 wheel collision。

### 交付物

```text
collision assets
contact excludes
collision 参数说明
test_contacts.py
test_self_collision.py
test_initial_penetration.py
```

### 验收

* keyframe 无非预期穿透；
* 双臂互碰有效；
* Hand 与机械臂碰撞有效；
* 机械臂与底盘、Spine 和 Mount 碰撞有效；
* wheel-floor 接触有效；
* 无全局关闭自碰撞；
* 碰撞性能可接受。

---

## 32.7 阶段 5：TMR 移动底盘动力学

### 目标

实现由轮组和轮地接触驱动的 TMR。

### 任务

1. 配置 `base_freejoint`；
2. 配置 steering 和 drive joint；
3. 配置 Caster 与 Rocker；
4. 实现 Swerve/Argo 逆运动学；
5. 建立 torque/velocity 控制示例；
6. 标定轮地摩擦；
7. 建立 ground-truth odometry；
8. 建立 wheel odometry；
9. 完成前进、横移、旋转、斜移和斜坡测试；
10. 建立 planar debug 变体。

### 交付物

```text
tmr_swerve_controller.py
tmr_control.yaml
test_mobile_base.py
test_wheel_contacts.py
test_odometry.py
```

### 验收

* 正式运动来自轮地接触；
* 不依赖 planar joint；
* 前进、横移和旋转可实现；
* steering 连续；
* Caster 与 Rocker 运动合理；
* 静止漂移在阈值内；
* 无异常大 actuator force；
* 60 秒运行无 NaN。

---

## 32.8 阶段 6：Spine、双 FR3 与双 Franka Hand

### 目标

完成整机操作机构控制和抓取能力。

### 任务

1. 实现 Spine torque 和 position controller；
2. 实现双 FR3 torque actuator；
3. 实现重力补偿；
4. 实现 joint-space PD；
5. 实现双臂同步轨迹；
6. 实现 Hand width 控制；
7. 建立标准抓取对象；
8. 执行方块、圆柱和薄片抓取；
9. 执行抓取后 Spine 升降；
10. 执行抓取后底盘移动；
11. 执行整机耦合稳定性测试。

### 交付物

```text
spine_position_controller.py
dual_arm_joint_controller.py
hand_width_controller.py
标准抓取场景
test_spine.py
test_arms.py
test_hands.py
test_whole_body_stability.py
```

### 验收

* Spine 全行程可控；
* 双臂重力补偿稳定；
* 双 Hand 同步正确；
* 标准抓取通过；
* 抓取后升降和移动通过；
* 无吸附或辅助弹簧；
* 整机耦合运行无 NaN。

---

## 32.9 阶段 7：传感器实体、接口与运行时

### 目标

补齐 TMR 官方传感器和头部 ZED Mini，建立独立调度的传感器运行时。

### 任务

1. 实现四台 D455；
2. 实现两台 nanoScan3；
3. 实现 OLV-IMU01 nominal 模型；
4. 实现 ZED Mini 左右相机；
5. 建立 optical 和 scan frame；
6. 实现 RGB 和 metric depth；
7. 实现 `mj_multiRay`；
8. 实现 gyro、accelerometer 和 orientation；
9. 定义 `SimulationSnapshot`；
10. 实现物理、相机、LiDAR 和状态线程；
11. 建立传感器 profile；
12. 完成传感器安装、方向、时间和频率测试。

### 交付物

```text
mobile_fr3_duo_with_sensors.xml
scene_with_sensors.xml
sensor profiles
camera renderer
LiDAR raycast module
IMU interface
SimulationSnapshot
test_sensor_mounts.py
test_cameras.py
test_lidars.py
test_imu.py
test_sensor_timing.py
```

### 验收

* D455 数量为 4；
* nanoScan3 数量为 2；
* IMU 数量为 1；
* ZED 左右目完整；
* mounting point 正确；
* optical frame 正确；
* LiDAR FoV 和方向正确；
* IMU 三轴正确；
* 更新率独立；
* 关闭传感器不影响动力学；
* 不并发修改主 `mjData`。

---

## 32.10 阶段 8：系统集成与正式交付

### 目标

完成性能优化、CI、文档、许可证和 `v1.0.0` 发布。

### 任务

1. 完成全部模型变体；
2. 优化 collision 和传感器性能；
3. 完成 60 秒和 10 分钟稳定性测试；
4. 完成所有 CI；
5. 完成 README、LICENSE 和 CHANGELOG；
6. 生成固定预览图；
7. 生成性能和测试报告；
8. 检查独立加载能力；
9. 打包正式模型；
10. 发布 `v1.0.0`。

### 交付物

```text
完整模型目录
全部正式变体
README.md
LICENSE
CHANGELOG.md
mobile_fr3_duo.png
CI
性能报告
测试报告
v1.0.0 发布包
```

### 验收

* 所有正式 XML 独立加载；
* 全部测试通过；
* 10 分钟集成运行无 NaN；
* 性能达到目标；
* 来源和许可证清晰；
* 无任务专用辅助；
* 目录和格式符合 Menagerie 风格；
* 质量等级表述准确。

---

## 32.11 阶段 9：Vision and Manipulation Kit

该阶段不属于 `v1.0.0`。

主要任务：

* 确认 Robotiq 2F-85 版本；
* 获取并审查官方 CAD；
* 建立 Robotiq 连杆和 equality；
* 建立 actuator 和抓取接触；
* 获取 D405 模型；
* 获取官方腕部支架；
* 建立 D405 外参；
* 使用官方 Vision Kit USD 验证装配；
* 增加 Vision Kit collision；
* 进行双 Robotiq 抓取验证；
* 增加独立许可证；
* 完成实机对比。

约束：

* 不修改基础模型的默认 Franka Hand；
* 以独立变体维护；
* 不以未经验证的社区参数覆盖官方数据；
* 不因该阶段延迟基础模型发布。

---

# 33. 项目开发计划

## 33.1 人员配置假设

推荐配置：

| 角色           |       投入 |
| ------------ | -------: |
| MuJoCo 模型工程师 |    1 人全职 |
| 控制与动力学工程师    |  0.5～1 人 |
| 传感器与渲染工程师    |    0.5 人 |
| 测试与工具工程师     |    0.5 人 |
| 技术负责人        | 按里程碑参与评审 |

推荐并行开发周期为 **24 周**。

单人串行开发建议预留 **36～44 周**。

## 33.2 里程碑

| 里程碑       |     时间 | 阶段   | 结果                  |
| --------- | -----: | ---- | ------------------- |
| M0：项目可构建  |  第 2 周 | 阶段 0 | 版本、资源、CI 和验证完成      |
| M1：官方基线冻结 |  第 4 周 | 阶段 1 | URDF 和清单完成 |
| M2：整机可视化  |  第 7 周 | 阶段 2 | 完整机械模型可加载           |
| M3：运动学可信  | 第 10 周 | 阶段 3 | FK、Jacobian 和惯性通过   |
| M4：碰撞可信   | 第 13 周 | 阶段 4 | Collision 和自碰撞稳定    |
| M5：底盘可控   | 第 17 周 | 阶段 5 | TMR 轮地驱动通过          |
| M6：操作系统可控 | 第 19 周 | 阶段 6 | Spine、双臂和 Hand 通过   |
| M7：传感器完整  | 第 22 周 | 阶段 7 | TMR 传感器和 ZED 完成     |
| M8：正式发布   | 第 24 周 | 阶段 8 | `v1.0.0` 发布         |

## 33.3 24 周排期

### 第 1～2 周：项目初始化

* 创建仓库；
* 固定官方版本；
* 获取资源；
* 建立文件清单；
* 实现验证脚本；
* 配置依赖和 CI；
* 完成许可证初审。

### 第 3～4 周：官方模型基线

* 生成两种 URDF；
* 提取 link、joint、inertial 和 frame；
* 建立命名映射；
* 固定统计基线。

### 第 5～7 周：原生机械 MJCF

* TMR；
* Spine；
* Duo Mount；
* Head；
* 双 FR3；
* 双 Hand；
* visual mesh；
* frame site；
* keyframe；
* scene 和预览。

### 第 8～10 周：运动学与惯性

* explicit inertial；
* joint limit；
* motor；
* position 变体；
* Hand equality；
* joint sensor；
* FK；
* Jacobian；
* 惯性和限制测试。

### 第 11～13 周：碰撞

* visual/collision 分层；
* collision mesh；
* self-collision；
* contact exclude；
* 随机碰撞扫描；
* wheel collision；
* finger pad；
* 初始穿透修复。

### 第 14～17 周：TMR 底盘

* freejoint；
* steering 和 drive；
* Caster 与 Rocker；
* Swerve IK；
* wheel-ground contact；
* odometry；
* 标准运动测试；
* 长时间稳定性。

### 第 18～19 周：Spine、双臂和 Hand

* Spine 控制；
* 双臂重力补偿；
* 双臂 PD；
* Hand width；
* 标准抓取；
* 抓取后升降和移动；
* 整机耦合测试。

### 第 20～22 周：传感器

* D455 × 4；
* nanoScan3 × 2；
* IMU × 1；
* ZED Mini；
* optical 和 scan frame；
* snapshot；
* 线程和调度；
* profile；
* 传感器测试。

### 第 23～24 周：集成与发布

* 完成模型变体；
* 性能优化；
* 10 分钟稳定测试；
* 完整 CI；
* README；
* LICENSE；
* CHANGELOG；
* 预览；
* 性能报告；
* `v1.0.0` 发布。

---

# 34. 项目管理规范

## 34.1 分支管理

```text
main
├── feature/source-audit
├── feature/native-mjcf
├── feature/collision
├── feature/tmr-dynamics
├── feature/dual-arm
├── feature/sensors
└── release/v1.0.0
```

每个分支必须：

* 对应明确 Issue；
* 包含实现和测试；
* 更新参数来源；
* 更新 CHANGELOG；
* 通过 CI；
* 完成评审后合并。

## 34.2 Issue 模板

每个任务包含：

```text
背景
目标
输入文件
修改范围
参数来源
实现步骤
测试方法
验收条件
风险
交付文件
```

建议单个 Issue 工作量为 1～5 个工作日。

## 34.3 阶段评审

评审状态：

| 状态                  | 含义          |
| ------------------- | ----------- |
| `PASS`              | 可进入下一阶段     |
| `PASS WITH ACTIONS` | 非阻塞问题转为后续任务 |
| `FAIL`              | 修复后重新评审     |

以下问题不得作为非阻塞项：

* 运动学错误；
* 惯性错误；
* joint axis 错误；
* 初始穿透；
* NaN；
* 正式底盘依赖代理关节；
* 状态覆盖；
* 来源不明资产；
* 许可证不明确。

## 34.4 参数变更

所有非官方参数记录在：

```text
source/parameter_sources.yaml
```

示例：

```yaml
parameter: wheel_floor_friction
component: tmr
value: "..."
classification: simulation-only
reason: "..."
test_reference: test_mobile_base.py
introduced_in: "v0.5.0"
```

参数修改必须说明：

* 修改原因；
* 影响的测试；
* 是否改变质量等级；
* 是否需要更新性能基线。

## 34.5 Definition of Done

任务完成必须满足：

* 实现已提交；
* 模型可加载；
* 测试已新增或更新；
* 测试通过；
* 无新增 warning；
* 参数来源已记录；
* 文档已更新；
* CHANGELOG 已更新；
* CI 通过；
* 完成评审；
* 无隐藏临时修复。

不能以“Viewer 中看起来正常”作为完成标准。

---

# 35. 主要风险

| 风险              | 影响        | 应对策略                      |
| --------------- | --------- | ------------------------- |
| TMR 轮地参数未公开     | 滑移和移动不真实  | 先建立稳定 B 级模型，后续实机辨识        |
| Argo Drive 结构复杂 | 接触不稳定     | 保留运动学，简化 collision        |
| Spine 负载较大      | 振荡或下滑     | torque actuator、闭环和限幅     |
| 双臂碰撞数量大         | 性能下降      | 使用官方简化自碰撞几何               |
| Hand 摩擦不足       | 抓取滑移      | 标准物体标定，不使用吸附              |
| 传感器 frame 复杂    | 图像和扫描方向错误 | 自动 frame 和方向测试            |
| Olive 无正式 Tag   | 资源无法固定    | 使用简化几何和官方 mounting point  |
| 自动转换丢失信息        | 结构或惯性错误   | URDF/MJCF 数值一致性测试         |
| 高分辨率相机过多        | 渲染性能不足    | 独立频率、按需启用和独立 worker       |
| Vision Kit 提前耦合 | 基础模型难维护   | 独立模型变体                    |
| 参数来源混乱          | 模型可信度下降   | 强制 parameter_sources.yaml |
| 非物理修复进入正式模型     | 动力学失真     | CI、代码评审和禁用项检查             |

---

# 36. 当前项目完成标准

## 36.1 结构

* [ ] 模型名称统一为 `mobile_fr3_duo`；
* [ ] 使用原生 MJCF；
* [ ] 不依赖运行时 URDF；
* [ ] robot 与 scene 分离；
* [ ] 无绝对路径；
* [ ] 无 `package://`；
* [ ] 所有名称唯一；
* [ ] 模型可独立加载。

## 36.2 官方资源

* [ ] 使用第 7 章固定 Tag；
* [ ] 官方文件清单完整；
* [ ] 官方文件验证通过；
* [ ] 许可证明确；
* [ ] 项目文件未冒充官方文件。

## 36.3 机械系统

* [ ] 完整 TMR；
* [ ] Argo Drive；
* [ ] Caster；
* [ ] Rocker；
* [ ] Spine；
* [ ] Duo Mount；
* [ ] Head Bracket；
* [ ] 双 FR3 v2.1；
* [ ] 双 Franka Hand。

## 36.4 动力学

* [ ] 全部显式惯性；
* [ ] 完整 joint limit；
* [ ] force limit；
* [ ] 底盘由轮地接触驱动；
* [ ] 不覆盖 `qpos/qvel`；
* [ ] 无抓取辅助；
* [ ] 无 NaN；
* [ ] 10 分钟集成测试通过。

## 36.5 碰撞

* [ ] visual/collision 分离；
* [ ] self-collision；
* [ ] 双臂互碰有效；
* [ ] Hand 与机械臂碰撞有效；
* [ ] 机械臂与底盘碰撞有效；
* [ ] 传感器碰撞合理；
* [ ] keyframe 无初始穿透。

## 36.6 传感器

* [ ] D455 × 4；
* [ ] nanoScan3 × 2；
* [ ] IMU × 1；
* [ ] ZED Mini × 1；
* [ ] mounting point 正确；
* [ ] optical frame 正确；
* [ ] LiDAR raycast；
* [ ] IMU 输出；
* [ ] 独立更新周期；
* [ ] 传感器可整体关闭。

## 36.7 测试和交付

* [ ] FK；
* [ ] Jacobian；
* [ ] 惯性；
* [ ] joint limit；
* [ ] TMR；
* [ ] Spine；
* [ ] 双臂；
* [ ] Hand；
* [ ] 传感器；
* [ ] 稳定性；
* [ ] 性能；
* [ ] CI；
* [ ] README；
* [ ] LICENSE；
* [ ] CHANGELOG；
* [ ] 预览图；
* [ ] `v1.0.0` 发布包。

---

# 37. 最终结论

项目实施路线为：

```text
Franka 官方 mobile_fr3_duo_v0_2
                ↓
固定官方 Tag
                ↓
验证官方源模型文件
                ↓
生成完整 URDF
                ↓
原生 MJCF 重构
                ↓
TMR + Spine + 双 FR3 + 双 Franka Hand
                ↓
补充 TMR 传感器和 nominal ZED Mini
                ↓
碰撞、执行器、状态和 keyframe
                ↓
结构、运动学、惯性和稳定性测试
                ↓
Menagerie 风格目录、许可证、文档和 CI
                ↓
稳定、可信、可复用的 mobile_fr3_duo v1.0
                ↓
未来独立扩展 Vision and Manipulation Kit
```

当前项目的核心成果不是立即完整复刻 Vision and Manipulation Kit，而是：

> 以 Franka 官方 `mobile_fr3_duo_v0_2` 为可靠机械基础，尽可能完善移动底盘、升降机构、双臂、双 Franka Hand、官方传感器预留位置、动力学、碰撞和仿真接口，建立一套能够长期维护和持续扩展的 Mobile FR3 Duo 原生 MuJoCo 模型体系。

完整 Vision and Manipulation Kit 是长期演进方向。基础 `mobile_fr3_duo` 模型必须在不依赖 Robotiq 和 D405 的情况下独立稳定、来源可信、结构清晰，并通过完整自动化测试。
