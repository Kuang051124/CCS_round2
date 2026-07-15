# AprilTag 单目测距 + 舵机翻转

基于 MaixCAM 的 AprilTag 视觉标签检测、单目测距，集成 SG90 舵机串口遥控翻转。

## 硬件需求

- MaixCAM (Pro) 开发板
- GC4653 摄像头（或兼容型号）
- AprilTag 标签（推荐 TAG36H11，10cm 边长）
- SG90 舵机（可选，A19 引脚）

## 接线

| SG90 | MaixCAM |
|------|---------|
| 橙(信号) | A19 |
| 红(VCC) | 5V |
| 棕(GND) | GND |

## 文件结构

```
apriltag_ranging/
├── main.py                  # 主程序 — AprilTag 检测 + 串口输出 + flip 命令
├── apriltag_detector.py     # AprilTag 检测 + 测距引擎
├── apriltag_config.py       # 共享参数管理（加载/保存/默认值）
├── apriltag_editor.py       # 触摸屏参数编辑器
├── sg90.py                  # SG90 舵机 PWM 驱动
├── calibrate.py             # 焦距标定工具
└── README.md
```

## 快速开始

```bash
cd /root/apriltag_ranging
python main.py
```

- **检测模式**：实时 AprilTag 检测 + 测距 + 串口输出
- **板载按键**：切换进入参数编辑器（触摸屏）
- **编辑器**：调节 k1 畸变系数，Save/Load/Reset/Exit
- **串口命令**：发送 `flip` 控制舵机 0°↔180° 翻转
- **一键标定**：编辑器中点 Auto Cal 自动计算焦距

## 串口通信

波特率 115200，端口 `/dev/ttyS0`。

### 输出帧（AprilTag 检测结果）

```
$31,04,DDD,HHH,5,III,AAA,SSS#
```

| 字段 | 内容 |
|------|------|
| DDD | 距离 mm 低 3 位 |
| HHH | 距离 mm 千位（最大 9999mm） |
| F=5 | AprilTag 标志 |
| III | 标签 ID |
| AAA | 偏航角 ×10（相对画面中心） |
| SSS | 偏航方向（0=右 1=左） |

示例：`$31,04,350,000,5,000,015,000#` → 距离 350mm，ID=0，偏航 1.5° 右

### 输入命令（控制舵机）

| 命令 | 作用 |
|------|------|
| `flip\n` | 翻转舵机：0°→180° 或 180°→0° |

## 测距原理

```
distance = (focal_length × real_diagonal) / pixel_diagonal
```

针孔相机模型。焦距通过 Auto Cal 自动标定。支持 k1 畸变校正。

## 参数说明

配置文件 `/root/apriltag_config.txt`

| 参数 | 默认 | 说明 |
|------|------|------|
| tag_family | TAG36H11 | 标签族 |
| tag_size_mm | 130.0 | 标签边长 (mm) |
| focal_length | 300.0 | 焦距 (px) |
| min_area | 100 | 最小检测面积 |
| distort_k1 | 0.0 | 桶形畸变系数 |
| output_mode | nearest | nearest=最近标签, all=全部 |

## 颜色标注

| 框色 | 距离 |
|------|------|
| 🟢 绿 | < 30 cm |
| 🟡 黄 | 30 ~ 80 cm |
| 🔴 红 | > 80 cm |
