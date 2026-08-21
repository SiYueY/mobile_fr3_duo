# TODO: FR3 Duo D435 腕部相机 optical frame 校准

## 当前状态（2026-08-21）

双侧 D435、Franka 专用支架与 Franka Hand 的机械层级已集成到
`models/mobile_fr3_duo.xml`：

```text
left/right_fr3v2_1_hand
  └─ left/right_d435_figure_fit_mount
       └─ left/right_d435_bottom_screw_frame
            └─ left/right_d435_link
```

已完成并保留的机械结论：

- 支架 STEP 的弧形法兰槽轴与当前 Franka Hand/TCP 轴同轴；槽半径为
  31.6 mm，与手部约 31.5 mm 的圆法兰匹配。
- D435 使用 Franka RealSense 专用支架的两颗 M3x10 螺钉安装；不使用
  D435 底部的 1/4”-20 孔。
- 当前 D435 位姿使用支架**前表面**的两处 M3 孔口，左右孔中心残差为
  浮点误差量级：`d435_link pos="0.0057 0.0175 0.011"`，identity rotation。
- 支架与 D435 为 visual-only；不参与 MuJoCo 接触或动力学。
- 基础模型可加载，相关 D435、visual mesh 和 sensor 测试共 26 项通过。

## 未完成问题

当前 `d435_*_rgb` / `d435_*_depth` 的 optical frame 未经过图像标定验证。
零位离屏 RGB 图主要看到机器人本体；将现有 camera quaternion 简单翻转
180 度后又会看向支架/手部。因此不能通过手工翻转 camera quaternion
解决。

这不影响机器人运动学、外观展示和非视觉任务；但会使腕部 RGB-D 图像的
方向、可见工作区和 RGB/depth 外参不可靠，不能用于视觉抓取、数据集生成、
相机标定或视觉策略训练。

## 下一次工作

- [ ] 锁定 D435 对应 `realsense-ros` Xacro/URDF 版本，并提取完整链：
  `d435_link → depth_frame/color_frame → *_optical_frame`。
- [ ] 在 D435 STEP 中识别 RGB 镜头中心、左右深度镜头中心、镜头面法线和
  顶部方向；记录其相对 D435 机械 frame 的 datum。
- [ ] 明确记录 STEP CAD → 当前 `d435.obj` 的坐标轴与手性变换。不能仅靠
  调整 MJCF quaternion 掩盖坐标系反射。
- [ ] 创建仅用于测试的临时 MJCF optical-calibration fixture（不提交到基础
  模型）：在每个 `hand_tcp` 前方 0.2–0.5 m 设置中心、左右、上方的高对比
  标记或棋盘格。
- [ ] 由 CAD/URDF 生成有限个候选 optical pose；对 RGB 和 depth 分别离屏
  渲染并筛选，而不是凭肉眼调整外壳位置。
- [ ] 选择满足以下条件的 pose：
  - 中心目标位于画面中心附近；
  - 左/右/上标记的像素方向正确，图像不镜像、不倒置；
  - 支架、手掌和基座不遮挡图像中心；
  - RGB 与 depth 分别指向工作区，并保留官方基线偏移；
  - 左右腕在对应工作空间均得到可用视野。
- [ ] 将验收后的 optical site 和 camera `pos/quat` 写入
  `models/mobile_fr3_duo.xml`，并同步更新
  `models/realsense_d435/realsense_d435.metadata.yaml`。
- [ ] 保存离屏 RGB/depth 验收图和数值结果到 `reports/`，记录源文件 URL、
  commit/SHA-256、坐标变换与最终外参。

## 验收标准

1. MuJoCo 3.9 可加载 `models/mobile_fr3_duo.xml`，无 XML 或 asset 错误。
2. 四个相机 `d435_left/right_{rgb,depth}` 都可离屏渲染。
3. 已知标定目标的投影中心误差不超过画面宽、高的 5%。
4. RGB/depth 目标方向一致，已知距离与理想 depth 输出一致。
5. 画面不包含支架或手部的大面积自遮挡，且工作区目标可见。
6. 完成后重新运行 D435、visual mesh 和 sensor 回归测试。

## 说明

MuJoCo 相机可提供理想 RGB 与 depth 渲染，但不自动模拟 RealSense 双目匹配
误差、IR 投影器、深度噪声或缺失像素；如任务需要这些效果，应在 optical
外参验收后单独实现传感器噪声模型。
