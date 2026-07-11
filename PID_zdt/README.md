# CameraPID 摄像头坐标 PID 跟踪控制 — 使用文档

## 概述

CameraPID 是一个基于增量式 PID 的**双轴摄像头跟踪控制器**，用于闭环控制两台 ZDT_X42S 步进电机，使激光光斑跟踪摄像头画面中的目标坐标。

**核心能力:**

| 功能 | 说明 |
|------|------|
| 实时跟踪 | 摄像头坐标 → PID 计算 → 电机脉冲修正，闭环完成 |
| 迷宫路径 | 录制/预设路径点序列，自动逐点跟踪 |
| 识别采集 | 解析摄像头协议中的数字/图形/动物识别结果 |

---

## 协议格式

摄像头通过 UART 发送以下帧格式:

```
$31,04,XXX,YYY,FFF,000,000,000#
```

| 字段 | 含义 | 示例 |
|------|------|------|
| `$` | 帧起始符 | — |
| `31` | 协议号 | 固定 |
| `04` | 子协议号 | 固定 |
| `XXX` | X 坐标 | 000–999 (像素) |
| `YYY` | Y 坐标 | 000–999 (像素) |
| `FFF` | 功能码 | 见下表 |
| 后续字段 | 预留 | 根据 FFF 不同有不同含义 |

**功能码 FFF:**

| FFF | 含义 | 额外字段 |
|-----|------|----------|
| `0` | 目标坐标 (要去的位置) | 无 |
| `1` | 激光坐标 (当前位置) | 无 |
| `2` | 数字识别 | `NNN` = 数字值 |
| `3` | 图形识别 | `AAA`预留, `BBB`颜色(1红2黄3绿4蓝), `CCC`形状(1三角2方3圆) |
| `4` | 终点目标 | 到达终点, 写入 target_x/y 并置 `g_endpoint_ready` |

---

## 快速开始

### 1. 初始化

```c
#include "pid.h"

int main(void) {
    HAL_Init();
    // ... 其他硬件初始化 ...

    CameraPID_Init();                        // 初始化 PID 控制器
    CameraPID_AttachMotors(&motor_x, &motor_y);  // 绑定 X/Y 轴电机
    CameraPID_SetMotorParams(500, 5);        // 速度 500 RPM, 加速度档位 5
    CameraPID_SetTunings(0.6f, 0.01f, 0.2f,  // X 轴 Kp/Ki/Kd
                         0.6f, 0.01f, 0.2f); // Y 轴 Kp/Ki/Kd
    CameraPID_SetScale(10.0f, 10.0f, 200.0f); // 标定比例, 输出上限
    CameraPID_SetDeadband(3);                // 3 像素死区

    while (1) {
        CameraPID_Track();  // 一键跟踪
        HAL_Delay(5);       // ~200Hz
    }
}
```

### 2. PID 调参

**调参顺序建议: 先 Kp, 再加 Kd, 最后补一点 Ki**

```c
CameraPID_SetTunings(kp_x, ki_x, kd_x, kp_y, ki_y, kd_y);
```

| 参数 | 作用 | 偏大症状 | 偏小症状 |
|------|------|----------|----------|
| Kp | 比例增益 | 超调震荡 | 响应慢, 跟不上 |
| Ki | 积分增益 | 低频振荡, 过冲 | 有稳态误差 |
| Kd | 微分增益 | 高频抖动, 对噪声敏感 | 阻尼不足 |

**典型初值 (需根据实际机械标定):**

| 场景 | Kp | Ki | Kd |
|------|-----|-----|-----|
| 保守 (先跑通) | 0.3 | 0.0 | 0.0 |
| 标准 | 0.6 | 0.01 | 0.2 |
| 激进 (快速响应) | 1.0 | 0.02 | 0.5 |

### 3. 标定 scale

`scale` = 每像素对应多少电机脉冲，需实测标定:

```
1. 电机发 100 脉冲
2. 观察激光在摄像头画面中移动了多少像素
3. scale = 100 / 移动像素数
```

例如: 100 脉冲移动了 10 像素 → scale = 10.0

---

## API 参考

### 初始化类

```c
void CameraPID_Init(void);
```
清零所有状态，默认 `scale=10.0`, `deadband=3`，启动 UART1 中断接收。调用后必须再调 `SetTunings` 设置 PID 参数。

```c
void CameraPID_AttachMotors(ZDT_HandleTypeDef *motor_x, ZDT_HandleTypeDef *motor_y);
```
绑定 X/Y 轴电机句柄，必须在 `Init` 后调用。

```c
void CameraPID_SetMotorParams(uint16_t speed_rpm, uint8_t acc);
```
设置电机速度 (RPM) 和加速度档位 (0–255)。

```c
void CameraPID_SetTunings(float kp_x, float ki_x, float kd_x,
                          float kp_y, float ki_y, float kd_y);
```
设置两轴 PID 参数，可独立配置。

```c
void CameraPID_SetScale(float sx, float sy, float out_max);
```
设置像素→脉冲换算比例 (`sx`, `sy`) 和单次最大修正脉冲数 (`out_max`)。

```c
void CameraPID_SetDeadband(uint16_t pixels);
```
设置死区 (像素)，误差在此范围内时不输出修正，防止电机微抖。

### 协议解析类

```c
void CameraPID_FeedByte(uint8_t byte);
```
逐字节喂入解析器，典型用法: 在 UART 中断回调中调用。

```c
void CameraPID_PollUART(void);
```
在 main 循环中轮询 UART，非阻塞 (timeout=1ms)。

### PID 计算类

```c
uint8_t CameraPID_Compute(int32_t *corr_x, int32_t *corr_y);
```
执行一次增量 PID 计算，返回电机脉冲修正量。返回 0 = 无新数据或误差在死区内，返回 1 = 有新的修正量。

### 一键跟踪

```c
void CameraPID_Track(void);
```
主循环中调用这一个即可完成: 轮询 + PID 计算 + 驱动电机。

### 迷宫路径

```c
void CameraPID_StartRecord(void);     // 开始录制路径
void CameraPID_FinishRecord(void);    // 结束录制并锁定
void CameraPID_LoadPath(const MazePoint *points, uint8_t count);  // 加载预设路径
uint8_t CameraPID_MazeTrack(void);    // 迷宫跟踪主函数
void CameraPID_GetMazeProgress(uint8_t *cur, uint8_t *total);  // 查询进度
```

**MazeTrack 返回值:**
- `0` — 正在跟踪当前点
- `1` — 到达当前点 (应蜂鸣 0.5s)
- `2` — 全部完成 (应蜂鸣 2s)

### 识别数据

```c
extern RecogItem g_recog;              // 最新识别结果
extern RecogItem g_items[12];          // 收集列表
extern uint8_t  g_item_count;          // 已收集数量
extern uint8_t  g_item_new;            // 有新条目
extern uint8_t  g_endpoint_ready;      // FFF=4 终点已收到

void CameraPID_ClearItems(void);       // 清空收集列表
```

---

## 增量式 PID 原理

```
公式: Δu = Kp×[e(k)-e(k-1)] + Ki×e(k) + Kd×[e(k)-2·e(k-1)+e(k-2)]
```

- **输出 Δu**: 本次修正**变化量** (非绝对位置)
- **误差 e(k)**: `target - laser`，正 = 激光偏左/偏下
- **直接发给电机**: 修正量配合 `ZDT_MODE_REL_CURRENT` 模式

**为什么用增量式而非位置式:**
1. 输出天然是变化量，配合相对位置脉冲指令
2. 无积分饱和风险
3. 摄像头丢帧/断连也不至于输出突变

---

## 方向校准

如果实际运动方向与预期相反，将对应轴的 `scale` 取负即可:

```c
CameraPID_SetScale(-10.0f, 10.0f, 200.0f);  // X 轴反向
```

---

## 典型工作流程

### 跟踪模式

```
┌──────────┐    UART中断     ┌──────────────┐
│  摄像头   │ ─────────────→ │ FeedByte解析  │
│ $31,04.. │  逐字节喂入     │ 更新target/laser│
└──────────┘                └──────┬───────┘
                                   │
                            CameraPID_Track()
                                   │
                    ┌──────────────┼──────────────┐
                    │  Compute()   │  像素→脉冲    │
                    │  增量PID     │  乘以scale    │
                    └──────────────┴──────┬───────┘
                                          │
                                   ZDT_FastPosMove()
                                          │
                                   ┌──────┴──────┐
                                   │  X轴电机     │
                                   │  Y轴电机     │
                                   └─────────────┘
```

### 迷宫模式

```
StartRecord() → 摄像头自动追加路径点 → 激光出现时自动 FinishRecord()
     │
     ▼
MazeTrack() 循环调用:
  当前目标点 → Track() → 到达判断 → 切换下一个点
     │
     ▼
全部完成 → 返回 2
```

---

## 注意事项

1. **must both_ready**: 目标和激光坐标都至少收到过一次，PID 才开始计算
2. **坐标刷新**: Compute 每次只处理最新的一对坐标，处理完清零标志，防止在相同误差上反复跑 PID
3. **电机参数**: SetMotorParams 后 `s_params_sent` 自动清零，下次 Track 会重发 SetParams
4. **UART 缓冲**: 解析器使用 `$` 作为帧起始，乱码自动丢弃；帧结束符支持 `#` 或 `\n`
5. **录制终点**: 激光出现时自动结束录制，或手动调 `FinishRecord()`
