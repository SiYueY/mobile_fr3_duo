# 性能报告（v1.0.0）

测试环境：本机（Linux，Python 3.13，mujoco 3.9.0），`scene.xml`，
home keyframe，`implicitfast` + Newton（iterations=50）。

## 模型规模

| 指标 | 值 |
| --- | ---: |
| nbody | 129 |
| njnt | 29 |
| nv | 34 |
| nu | 21 |
| ngeom | 147 |
| 接触 exclude | 396+ |
| 传感器（with_sensors） | 84 |
| 相机（with_sensors，含 preview） | 11 |

## 单步耗时

| 指标 | 值 |
| --- | ---: |
| mean step | 0.093 ms |
| P95 step | 0.112 ms |
| 实时因子 RTF | ~10.8 |
| Snapshot 复制 | ~1.7 µs |

单步 0.088 ms 远低于 1 kHz 周期（1 ms），为相机/LiDAR worker 与控制器
预留充足调度余量。

## 传感器成本

- D455 相机渲染（640×480，EGL 离屏）：与物理线程解耦，30 Hz；
- ZED 双目渲染：30/60 Hz；
- nanoScan3 raycast（500 束 × 2，`mj_multiRay`）：50 Hz；
- 实测 60 s keyframe 稳定性运行 wall 时间 ≈ 4–5 s。

## 回退门禁

CI 对 mean step 与 P95 step 设软门禁（>10% 回退告警），见
`tests/test_performance.py`。
