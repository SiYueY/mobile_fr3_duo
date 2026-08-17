# Mobile FR3 Duo 原生 MuJoCo 模型 v1.0.0

## 1. 项目定位

以 Franka 官方 `franka_description@2.8.1` 的 `mobile_fr3_duo_v0_2` 为机械真源，
构建可复用、可维护、稳定且可信的**原生 MJCF** 仿真模型，对齐
`mujoco_menagerie` 的组织、质量、文档与测试标准。设计文档见
[mobile_fr3_duo.md](mobile_fr3_duo.md)。

## 2. 当前范围（v1.0.0）

- TMR v0.2 移动底盘（4 轮 swerve、Caster、Rocker、Argo 驱动）
- Franka Spine v0.1（升降柱）
- FR3 Duo Mount 与 Head Bracket
- 双 FR3 v2.1（14 主动关节）
- 双 Franka Hand（两指联动，width = q1 + q2 ∈ [0, 0.08] m）
- 传感器实体：D455 × 4、nanoScan3 × 2、OLV-IMU01 × 1、ZED Mini 双目
- 六组 keyframe、position 变体、reduced 变体、planar debug 变体

## 3. 长期 Vision Kit 目标（v2.0.0，不在本版本）

双 Robotiq 2F-85、D405、官方腕部支架与 Vision Kit 装配（见设计文档 §32.11）。

## 4. 固定官方版本

| 组件 | 版本 | 提交 |
| --- | --- | --- |
| franka_description | 2.8.1 | `02afaae` |
| franka_ros2 | v2.5.1 | `9faaaaf` |
| realsense-ros | 4.58.3 | `60c8509` |
| zed-ros2-description | 0.1.5 | `449eef9` |
| sick_safetyscanners2 | 1.0.5 | `c8d787f` |

完整 SHA-256 清单见 `source/official_model_files.yaml`。

## 5. MuJoCo 最低版本

`mujoco >= 3.9.0`（锁定 `mujoco==3.9.0`，升级走独立变更）。

## 6. 官方源文件列表

见 `source/official_model_files.yaml`（5 个仓库的 tag、commit 与文件 SHA-256）
和 `source/asset_manifest.yaml`（mesh 清单）。

## 7. URDF 生成方法

`tools/generate_urdf.sh`：在临时 ament index 下调用 ROS Humble `xacro` 展开
`mobile_fr3_duo_v0_2.urdf.xacro`，参数
`robot_types=['tmrv0_2','fr3v2_1','fr3v2_1']`、`hand=true`、
`reduced_version=false`、`gazebo=false`，分别生成 visual 与 self-collision
URDF 到 `source/generated/`。

## 8. Mesh 转换方法

`tools/convert_visual_meshes.py`（DAE→OBJ，毫米单位按 DAE `<unit>` 归一为米）、
`tools/convert_collision_meshes.py`（STL 校验复制）、
`tools/import_sensor_assets.py`（传感器 mesh）。每次转换记录输入/输出
SHA-256、工具版本与单位换算，见 `source/generated/*_conversion.json`。

## 9. Collision 处理

- visual / collision / self-collision / sensor_collision 分层 default class；
- 官方 `--with-sc` 胶囊 + 官方 collision STL；
- contact exclude 来自官方 SRDF、父子对，以及对膨胀 SC 壳的 documented
  例外（肩部壳与腕部壳在双臂安装间距下必然重叠，仅排除壳-壳对，真实几何
  全部保持激活，见 `source/parameter_sources.yaml` 的
  `sc_shell_exclusions`）。

## 10. Actuator 类型

- 基础模型：`motor`（TMR 4、Spine 1、FR3 14、Hand 2）；
- `mobile_fr3_duo_position.xml`：`position` 变体；
- 关节 effort/velocity 取自官方 `joint_limits.yaml`；TMR/Hand 的 actuator
  力幅为仿真标定值（记录于参数来源）。

## 11. Sensor 列表

- D455 × 4（device/color/depth/infra1/infra2 + optical frame，30 Hz）
- nanoScan3 × 2（scan frame + `mj_multiRay`，50 Hz，±45°/225°，0.05–10 m，
  500 束）
- OLV-IMU01 × 1（gyro/accelerometer/framequat，官方 mounting point + 项目
  box 几何）
- ZED Mini 双目（左右相机 + 63 mm baseline）
- jointpos/jointvel/jointactuatorfrc、base pos/quat/vel

## 12. Keyframe

`home`、`transport`、`manipulation`、`wide_workspace`、`spine_min`、
`spine_max`。全部通过无自碰撞、joint limit 与初始穿透检查。

## 13. 运行方法

用 MuJoCo 官方 `simulate` 打开时，请加载**带地面的 scene 变体**（机器人本体
XML 按设计文档 §4.4 不含地面，单独打开会因无接触面而下坠）：

```bash
/opt/mujoco-3.9.0/bin/simulate ./scene_with_sensors.xml   # 完整传感器 + 地面
/opt/mujoco-3.9.0/bin/simulate ./scene.xml                # 基础模型 + 地面
uv run python examples/dual_arm_joint_controller.py --viewer  # home 位姿闭环保持
```

注意：motor 变体（`scene.xml` / `scene_with_sensors.xml`）启动时执行器
ctrl=0（无驱动力）。使用 MuJoCo `simulate` 时请先加载 `home` keyframe，或使用
上面的 `--viewer` 命令运行重力补偿与双臂 PD 控制；这样可从明确的安全初始姿态
开始仿真。

命令行/脚本加载：

```bash
make install
.venv/bin/python tools/build_model.py --all
.venv/bin/python tools/render_preview.py
.venv/bin/python -c "import mujoco; mujoco.MjModel.from_xml_path('scene.xml')"
```

机器人本体文件（`mobile_fr3_duo*.xml`）保留"无地面/无任务对象"的纯机器人
定义（§4.4 机器人与任务场景分离）；需要落地、驱动、抓取时统一使用
`scene*.xml`。

示例（`examples/`）：`simulation_snapshot.py`（多线程运行时骨架）、
`camera_renderer.py`（独立 mjData 离屏渲染）、`lidar_raycast.py`
（`mj_multiRay` 50 Hz）、`tmr_swerve_controller.py`、`spine_position_controller.py`、
`dual_arm_joint_controller.py`、`hand_width_controller.py`、`grasp_scenario.py`。

## 14. 测试方法

```bash
make check   # ruff + XML 格式 + 资产校验 + 官方文件校验
make test    # pytest（含 5×60 s keyframe 与 10 min 集成稳定性）
```

## 15. 已知限制

- 官方 SC 壳（半径 7–9 cm）在双肩 10 cm 间距下必然重叠，壳-壳对按
  documented 例外排除；膨胀壳使动态抓取示例易失败，`grasp_scenario.py`
  为尽力而为演示；
- TMR 官方底盘 mesh 底部与轮底接近同高，悬挂（Rocker/Caster）沉降后底盘
  可能轻微触地，轮地驱动仍由轮组接触产生（详见
  `source/parameter_sources.yaml` 的 `tmr_chassis_ground_contact`）；
- 官方 TMR 底盘 mesh 为简化壳体，包覆四轮上部；轮子视觉/碰撞几何均存在
  且参与驱动，但从常规视角大部分被壳体遮挡（贴近地面仅露出底部窄带），
  属官方几何外观；需要查看轮子时可从低角度近距离观察或隐藏 chassis
  group；
- 轮地摩擦、solref/solimp、PD 增益、传感器噪声为 `simulation-only` /
  `estimated` 参数，无实机辨识；
- OLV-IMU01 使用项目编写简化几何（上游无正式 tag）；
- 无实机数据，质量等级不声明 A+（见 §19）。

## 16. 许可证

仓库 Apache-2.0（见 [LICENSE](LICENSE)）。franka_description、franka_ros2
为 Apache-2.0；realsense-ros、zed-ros2-description、sick_safetyscanners2
各自携带独立许可证，仅以转换后的资产引用，未随本仓库分发原始文件。

## 17. 参数来源分类

所有非官方参数按 `official-source | official-derived | project-authored |
identified | estimated | simulation-only` 分类，完整清单见
`source/parameter_sources.yaml`。

## 18. 与真实机器人的差异

- 电机/传动、摩擦与阻抗参数为估计值；
- 传感器为理想模型（无真实噪声/畸变模型，除声明估计项）；
- 未做实机辨识；wheel slip、关节摩擦等以稳定性和测试通过为目标标定。

## 19. 模型质量等级

| 项目 | 等级 | 说明 |
| --- | --- | --- |
| 几何/运动学 | A | 官方 URDF/DH + PyKDL 交叉验证（≤1e-6，实测 5e-11） |
| 官方惯性 | A | 逐 link 质量/COM/惯量核对，总质量 178.184 kg |
| FR3 | B+ | 运动学/惯性官方，驱动参数估计 |
| Hand/Spine/TMR | B | 几何官方，动力学与控制参数仿真标定 |
| 理想传感器 | A- | 安装外参官方，成像模型理想化 |
| 整体 | B | 无实机辨识，不声明 A+ |

---

另见 [CHANGELOG.md](CHANGELOG.md)、`docs/reports/performance.md` 与
`docs/reports/stability.md`。
