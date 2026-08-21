# Changelog

All notable changes to the Mobile FR3 Duo MuJoCo model are recorded here.

## [Unreleased]

- Runtime 模块、整机变体和 scene 统一迁入 `models/`；根目录旧 XML 与
  `assets/` 已移除。
- 模块目录采用 `franka_tmr`、`franka_spine`、`franka_head`、`franka_fr3`、
  `franka_hand` 和同级传感器模块，每个模块自带 assets、metadata，可独立分发
  加载；正式整机模型为直接完整 MJCF，不再包含模块依赖闭包。
- visual、collision 和 sensor conversion manifest 改为记录 `models/...`
  输出；flattened 发布模型使用模块资产的相对路径。

## [v1.0.0] - 2026-08-07

### v0.1.0 - 项目初始化与官方资源审计

- uv/pyproject/ruff/pytest/pre-commit/GitHub Actions 脚手架。
- 固定五个官方 tag 并建立 `official_model_files.yaml`（tag、commit、文件
  SHA-256）与 `verify_official_model_files.py` 校验。
- 许可证审计（franka_description/franka_ros2 Apache-2.0，传感器组件独立
  许可证说明）。

### v0.2.0 - 完整机械结构与 visual

- 生成 visual / self-collision / reduced URDF（82 links / 81 joints，含
  `_sc` 元素）。
- DAE→OBJ 与 STL 转换脚本，全部资产记录 SHA-256 与单位归一（mm→m）。
- 原生 MJCF 机械树（TMR、Spine、Duo Mount、Head、双 FR3、双 Hand）、
  mounting point site、scene 与预览渲染。

### v0.3.0 - 惯性、限制、执行器与运动学

- 全部显式 `fullinertia`（URDF rpy 旋转到 body 系）、joint range/effort/
  velocity/damping、motor 与 position 变体、Hand equality。
- 修复关节轴约定（rotation-first 中间 joint-frame body，PyKDL 交叉验证
  ≤1e-6，实测 5e-11）、mesh `type="mesh"`（此前退化为包围球碰撞）、
  DAE 毫米单位（TMR/Spine）、equality polycoef 常量项。
- FK（1000 组随机）、Jacobian（中心差分）、惯性（特征值）与限制测试。

### v0.4.0 - Collision 与 self-collision

- visual/collision/self-collision/sensor_collision default class；
- 官方 SRDF + 父子对 + SC 壳例外（肩部/腕部壳-壳对）contact exclude；
- 六个 keyframe 无初始穿透；双臂互碰有效（随机姿态扫描回归）。
- 修复 URDF box 全尺寸→MuJoCo 半尺寸换算（底盘外壳、指尖、Caster 等
  碰撞盒此前放大一倍）。
- 导出官方 visual primitives（驱动轮/脚轮/摇臂圆柱与外壳 box），修复
  simulate 中"底盘没有轮子"；scene 增加低位补光便于查看轮子。
- 新增 `scene_position.xml`（位置控制变体 + 地面，开箱即静止）；motor
  keyframe ctrl 改为重力补偿力矩；Spine motor 力幅调整为 ±600 N（官方
  100 N 无法静态承载上体 ~370 N，见参数来源）。

### v0.5.0 - TMR 移动底盘动力学

- freejoint、4 轮 swerve、Caster/Rocker、轮地接触（cylinder + friction
  标定）；前进/后退/制动/静止漂移测试；planar debug 变体。

### v0.6.0 - Spine、双 FR3 与双 Franka Hand

- Spine 全行程与 60 s 高度保持；双臂 PD/重力补偿；Hand 两指同步、宽度
  范围与 pad 镜像验证；`examples/` 控制器与抓取场景。

### v0.7.0 - 传感器实体与运行时

- D455 × 4、nanoScan3 × 2、IMU × 1、ZED Mini 双目实体与 optical/scan
  frame；`SimulationSnapshot`、相机渲染 worker（独立 mjData）、LiDAR
  raycast worker（`mj_multiRay` 50 Hz）。

### v0.9.0 - Release Candidate

- 全部变体独立加载；性能基线（mean 0.088 ms / RTF ~11.3）；
- README 19 项、参数来源、性能/稳定性报告。

### v1.0.0 - 正式发布

- 85 项测试全绿（含 5×60 s 与 10 min 集成稳定性）；
- 完整文档、预览图、CHANGELOG 与 tag。
