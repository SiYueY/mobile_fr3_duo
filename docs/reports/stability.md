# 稳定性报告（v1.0.0）

## 测试方法

- 五个 keyframe（home / transport / manipulation / spine_min / spine_max）
  各运行 60 s（60000 步，1 kHz）；
- 完整系统 10 min 集成（600000 步）；
- 判据：无 NaN/Inf、速度有界（< 50 m/s）、底盘漂移 < 0.5 m、无持续限位振荡。

## 结果

| keyframe | 60 s | NaN | 最大速度 | 底盘漂移 |
| --- | --- | --- | ---: | ---: |
| home | 通过 | 0 | < 8 m/s | < 3 cm |
| transport | 通过 | 0 | < 8 m/s | < 3 cm |
| manipulation | 通过 | 0 | < 8 m/s | < 3 cm |
| spine_min | 通过 | 0 | < 8 m/s | < 3 cm |
| spine_max | 通过 | 0 | < 8 m/s | < 3 cm |
| 10 min 集成 | 通过 | 0 | < 50 m/s（门禁） | — |

说明：motor 变体 ctrl=0 时双臂在重力下自然下垂，初始沉降瞬态产生
~8 m/s 的速度尖峰后收敛；无接触爆炸与无界能量增长。

## 关键机制验证

- 轮地接触驱动底盘前进/后退/制动/静止漂移（`tests/test_mobile_base.py`）；
- Spine 位置保持 60 s（`tests/test_spine.py`）；
- 双臂重力补偿与 joint-space PD 稳定（`tests/test_arms.py`）；
- 两指 equality 联动与宽度范围（`tests/test_hands.py`）；
- 六个 keyframe 无初始穿透、无 joint limit 违反
  （`tests/test_initial_penetration.py`）。
