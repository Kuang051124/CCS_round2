# yb_protocol 协议模块与摄像头串口通信使用文档

## 概述

`yb_protocol` 是运行在 TI MSPM0G3507 微控制器上的自定义串口协议模块，用于与 **K230 AI 摄像头模块**进行双向通信。该协议通过 UART 串口接收摄像头识别到的**激光笔坐标**和**目标靶坐标**，供主控程序驱动步进电机完成自动瞄准。

---

## 1. 系统架构

```
┌─────────────────┐     UART (自定义协议)      ┌──────────────────┐
│   K230 摄像头    │ ◄──────────────────────► │  MSPM0G3507 主控  │
│  (AI 视觉识别)   │      $...帧格式...#        │                  │
└─────────────────┘                            └────┬──┬──┬──┬────┘
                                                    │  │  │  │
                                          ┌─────────┘  │  │  └──────────┐
                                          ▼             ▼  ▼              ▼
                                    ┌──────────┐  ┌──────────┐   ┌──────────────┐
                                    │ 步进电机  │  │ OLED显示  │   │ 蜂鸣器/LED    │
                                    │ (云台控制) │  │ (调试界面) │   │ (声光提示)    │
                                    └──────────┘  └──────────┘   └──────────────┘
```

### 串口资源分配

| 串口实例 | 外设 | 用途 | 波特率 |
|---------|------|------|--------|
| `K230_INST` | UART (Main) | 与 K230 摄像头通信，接收目标坐标 | 取决于 SysConfig 配置 |
| `UART_WIT` | UART + DMA | 维特智能姿态传感器 (yaw/pitch/roll) | 取决于模块配置 |
| `UART_BNO08X` | UART + DMA | BNO08X 姿态传感器 | 115200 |
| `UART_ZDT_INST` | UART3 Extend | ZDT_X42S 步进电机1 (PA26/PA13) | 115200 |
| `UART_ZDT2_INST` | UART1 Main | ZDT_X42S 步进电机2 (PB4/PB5) | 115200 |

---

## 2. yb_protocol 协议详解

### 2.1 文件结构

| 文件 | 说明 |
|------|------|
| [yb_protocol.h](yb_protocol.h) | 协议头文件：常量定义、全局变量声明、API 声明 |
| [yb_protocol.c](yb_protocol.c) | 协议实现：接收状态机、帧解析 |

### 2.2 协议帧格式

每帧数据以 **`$`** (0x24) 开头，**`#`** (0x23) 结尾，中间字段用逗号分隔：

```
$<len>,<id>,<x>,<y>,<w>,<h>,<target_id>,<degrees>#
 │   │    │    │  │  │  │     │         │
 │   │    │    │  │  │  │     │         └── 角度/朝向信息
 │   │    │    │  │  │  │     └── 目标ID
 │   │    │    │  │  │  └── 目标高度 (target_h)
 │   │    │    │  │  └── 目标宽度 (target_w)
 │   │    │    │  └── 激光笔 Y 坐标 (laser_y)
 │   │    │    └── 激光笔 X 坐标 (laser_x)
 │   │    └── 协议功能ID (固定为 4)
 │   └── 帧总长度 (包含 $ 和 #)
```

**示例帧**: `$12,4,320,240,100,100,1,45#`
- 帧长 18 字节 → 十进制 18（含 `$` 和 `#`）
- 协议ID = 4
- 激光坐标: (320, 240)
- 目标坐标: (100, 100)
- 目标ID = 1
- 角度 = 45°

### 2.3 两种帧格式

协议共用 `$` 帧头 `#` 帧尾 + 逗号分隔的格式，通过第 5 个字段（`format_flag`）区分数据类型：

#### 格式一：激光与目标坐标帧（format_flag = 4）

```
$<len>,4,<laser_x>,<laser_y>,<target_w>,<target_h>,<id>,<degrees>#
 │   │    │         │         │         │         │    │
 │   │    │         │         │         │         │    └── 角度
 │   │    │         │         │         │         └── 目标编号
 │   │    │         │         │         └── 目标高度 (像素)
 │   │    │         │         └── 目标宽度 (像素)
 │   │    │         └── 激光笔 Y 坐标 (像素)
 │   │    └── 激光笔 X 坐标 (像素)
 │   └── 协议ID (固定为 4)
 └── 帧总长度
```

**示例**: `$18,4,320,240,100,100,1,45#`
- 激光坐标: (320, 240)，目标坐标: (100, 100)，目标ID=1，角度=45°

#### 格式二：AprilTag 距离与角度帧（format_flag = 5）

```
$<len>,4,<dist_mm>,<tag_id>,5,<reserved>,<angle_raw>,<dir>#
 │   │    │         │       │   │          │         │
 │   │    │         │       │   │          │         └── 方向: 000=偏右, 001=偏左
 │   │    │         │       │   │          └── 角度原始值 (单位 0.1°)
 │   │    │         │       │   └── 保留字段 (如 443)
 │   │    │         │       └── 格式标志位 (5=AprilTag)
 │   │    │         └── Tag 编号 (1-255)
 │   │    └── 距离 (单位 mm)
 │   └── 协议ID (固定为 4)
 └── 帧总长度
```

**示例**: `$31,04,147,001,5,443,326,000#`
| 字段 | 值 | 含义 |
|------|-----|------|
| dist_mm | 147 | 距离 = 147 mm |
| tag_id | 001 | Tag 编号 = 1 |
| format_flag | 5 | AprilTag 格式 |
| reserved | 443 | 保留字段 |
| angle_raw | 326 | 角度 = 32.6° (326 × 0.1) |
| dir | 000 | 方向 = 偏右 |

### 2.4 常量定义

```c
#define PTO_BUF_LEN_MAX         (50)    // 接收缓冲区最大长度
#define PTO_ATAG_LEN_MAX        (32)    // 标签最大长度
#define PTO_HEAD                (0x24)  // 帧头 '$'
#define PTO_TAIL                (0x23)  // 帧尾 '#'
#define PTO_FUNC_ID             (4)     // 协议功能ID
#define PTO_FUNC_ID_APRILTAG    (4)     // AprilTag 协议ID (同 4)
#define PTO_APRILTAG_DIR_RIGHT  (0)     // 角度偏右
#define PTO_APRILTAG_DIR_LEFT   (1)     // 角度偏左
#define PTO_FORMAT_COORD        (4)     // 坐标模式
#define PTO_FORMAT_APRILTAG     (5)     // AprilTag 模式
```

### 2.4 全局变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `RxBuffer[50]` | `uint8_t[]` | 协议接收缓冲区，存储当前帧原始数据 |
| `RxIndex` | `uint8_t` | 接收缓冲区写入下标 |
| `RxFlag` | `uint8_t` | 接收状态机状态：0=等待帧头, 1=接收数据中 |
| `New_CMD_flag` | `uint8_t` | 新命令就绪标志，1 表示完整帧已收到 |
| `New_CMD_length` | `uint8_t` | 当前帧的数据长度 |
| `rx_temp` | `uint8_t` | 临时接收字节 |

### 2.6 API 说明

#### 基础 API（兼容旧代码）

##### `void Pto_Data_Receive(uint8_t Rx_Temp)`

**接收状态机**，在 UART 接收中断中逐字节调用。

**状态转换**:
```
State 0 (等待帧头):
  ├── 收到 '$' → 写入 RxBuffer[0], 切换到 State 1
  └── 收到其他 → 忽略

State 1 (接收数据):
  ├── 收到 '#' → 帧结束，置 New_CMD_flag=1，切回 State 0
  ├── 缓冲区溢出 (≥50字节) → 丢弃本帧，切回 State 0
  └── 收到其他 → 存入 RxBuffer, RxIndex++
```

##### `void Pto_Data_Parse(uint8_t *data_buf, uint8_t num, int *recv_data)`

原始解析函数，返回 `int` 数组（向后兼容）。

##### `void Pto_Clear_CMD_Flag(void)`

清除命令标志。

---

#### 结构化 API（推荐使用）

##### `int8_t Pto_Data_Parse_Coord(uint8_t *data_buf, uint8_t num, Pto_Coord_Data_t *coord)`

解析**激光与目标坐标**帧（格式标志位 = 4），将结果填充到结构体。

```c
Pto_Coord_Data_t coord;
int8_t ret = Pto_Data_Parse_Coord(RxBuffer, New_CMD_length, &coord);
if (ret == 0) {
    // coord.laser_x,  coord.laser_y   — 激光坐标
    // coord.target_w, coord.target_h  — 目标宽高
    // coord.target_id                 — 目标编号
    // coord.degrees                   — 角度
}
```

| 返回值 | 含义 |
|--------|------|
| `0` | 解析成功 |
| `-1` | 帧头 `$` 或帧尾 `#` 校验失败 |
| `-2` | 帧长度校验失败 |
| `-3` | 协议 ID 校验失败 |

##### `int8_t Pto_Data_Parse_Apriltag(uint8_t *data_buf, uint8_t num, Pto_Apriltag_Data_t *apriltag)`

解析 **AprilTag 距离与角度**帧（格式标志位 = 5）。

```c
Pto_Apriltag_Data_t tag;
int8_t ret = Pto_Data_Parse_Apriltag(RxBuffer, New_CMD_length, &tag);
if (ret == 0) {
    // tag.distance_mm  — 距离 (mm),  如 147
    // tag.tag_id       — Tag 编号,   如 1
    // tag.format_flag  — 格式标志位, 应为 5
    // tag.reserved     — 保留字段
    // tag.angle_raw    — 角度原始值 (0.1°单位), 如 326
    // tag.direction    — 方向, 0=偏右, 1=偏左

    // 转换为实际角度:
    float angle = Pto_Apriltag_Angle_To_Deg(tag.angle_raw, tag.direction);
    // angle = +32.6° (偏右) 或 -32.6° (偏左)
}
```

##### `int8_t Pto_Data_Parse_Auto(uint8_t *data_buf, uint8_t num, Pto_Coord_Data_t *coord, Pto_Apriltag_Data_t *apriltag)`

**自动识别帧格式**并解析。根据第 5 字段自动判断是坐标帧还是 AprilTag 帧。

```c
Pto_Coord_Data_t coord;
Pto_Apriltag_Data_t tag;
int8_t ret = Pto_Data_Parse_Auto(RxBuffer, New_CMD_length, &coord, &tag);
if (ret == 1) {
    // AprilTag 帧 → 使用 tag 结构体
} else if (ret == 0) {
    // 坐标帧 → 使用 coord 结构体
}
```

| 返回值 | 含义 |
|--------|------|
| `1` | AprilTag 帧，数据在 `apriltag` 中 |
| `0` | 坐标帧，数据在 `coord` 中 |
| `-1` | 帧头尾校验失败 |
| `-2` | 长度校验失败 |
| `-3` | 协议 ID 校验失败 |

##### `float Pto_Apriltag_Angle_To_Deg(int16_t angle_raw, uint8_t direction)`

将 AprilTag 角度原始值（0.1° 单位）转换为带符号的实际度数。

```c
float deg = Pto_Apriltag_Angle_To_Deg(326, PTO_APRILTAG_DIR_RIGHT);  // +32.6°
float deg = Pto_Apriltag_Angle_To_Deg(326, PTO_APRILTAG_DIR_LEFT);   // -32.6°
```

---

## 3. 摄像头串口通信

### 3.1 K230 摄像头

K230 是 Kendryte 的 AI 视觉处理模块，通过 UART 与 MSPM0 通信：

- **连接方式**: UART Main (K230_INST)
- **通信方向**: 双向（MSPM0 可发送控制命令给 K230）
- **摄像头功能**: 识别激光笔光斑和目标靶的位置

#### 停止摄像头命令

```c
void Stop_Camera() {
    sprintf(oled_buffer, "$P#%%");   // 发送停止命令
    len = strlen(oled_buffer);
    for (int i = 0; i < len; ++i) {
        DL_UART_Main_transmitData(K230_INST, oled_buffer[i]);
        mspm0_delay_ms(5);
    }
}
```

### 3.2 典型数据流

```
1. K230 摄像头采集图像 → AI 识别激光点 & 目标靶
2. K230 通过 UART 发送帧 → "$18,4,320,240,100,100,1,45#"
3. MSPM0 UART RX 中断触发
4. Pto_Data_Receive() 逐字节接收，检测帧头帧尾
5. 完整帧收到 → New_CMD_flag = 1
6. 主循环检测 New_CMD_flag → Pto_Data_Parse() 解析坐标
7. 提取 recv_data[0..5] → 激光坐标 / 目标坐标 / ID / 角度
8. PID 控制步进电机 → 将激光笔对准目标
9. Pto_Clear_CMD_Flag() → 等待下一帧
```

### 3.3 主循环中的完整使用示例

#### 仅处理坐标帧

```c
// 初始化
Pto_Clear_CMD_Flag();
DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

Pto_Coord_Data_t coord;

for (;;) {
    // 1. 超时检查
    mspm0_get_clock_ms(&curr_time);
    if (curr_time - last_time > 4000) break;

    // 2. 等待新帧
    if (!New_CMD_flag) continue;

    // 3. 解析坐标帧
    int8_t ret = Pto_Data_Parse_Coord(RxBuffer, New_CMD_length, &coord);
    Pto_Clear_CMD_Flag();

    if (ret != 0) continue;  // 校验失败

    // 4. 过滤无效数据
    if (coord.laser_x == 0 || coord.laser_y == 0 ||
        coord.target_w == 0 || coord.target_h == 0) continue;

    // 5. PID 控制步进电机对准
    SMotor_Turn_To(&smotor_hor,
        PID_Loc(&hori_laser_pid, coord.target_w, coord.laser_x, 1));
    SMotor_Turn_To(&smotor_ver,
        PID_Loc(&vert_laser_pid, coord.target_h, coord.laser_y, 1));

    // 6. 判断是否已对准
    if (Distance((Point_t){coord.target_w, coord.target_h},
                 (Point_t){coord.laser_x, coord.laser_y}) < 5) break;

    DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);
    mspm0_delay_ms(50);
}
```

#### 自动识别坐标帧 + AprilTag 帧

```c
Pto_Clear_CMD_Flag();
DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);

Pto_Coord_Data_t coord;
Pto_Apriltag_Data_t tag;

for (;;) {
    if (!New_CMD_flag) continue;

    int8_t ret = Pto_Data_Parse_Auto(RxBuffer, New_CMD_length, &coord, &tag);
    Pto_Clear_CMD_Flag();

    if (ret == 1) {
        // AprilTag 帧: 获取距离和角度
        float angle_deg = Pto_Apriltag_Angle_To_Deg(tag.angle_raw, tag.direction);
        float dist_m = tag.distance_mm / 1000.0f;

        // 显示
        sprintf(oled_buffer, "Dist:%4d mm", tag.distance_mm);
        sprintf(oled_buffer, "Ang :%+5.1f deg", angle_deg);
        sprintf(oled_buffer, "Tag :%3d Dir:%s", tag.tag_id,
                tag.direction == PTO_APRILTAG_DIR_RIGHT ? "R" : "L");

        // 根据距离和角度调整云台
        SMotor_Turn_Degree(&smotor_hor, angle_deg * 0.1f);

    } else if (ret == 0) {
        // 坐标帧: 激光瞄准逻辑
        SMotor_Turn_To(&smotor_hor,
            PID_Loc(&hori_laser_pid, coord.target_w, coord.laser_x, 1));
        SMotor_Turn_To(&smotor_ver,
            PID_Loc(&vert_laser_pid, coord.target_h, coord.laser_y, 1));

        if (Distance((Point_t){coord.target_w, coord.target_h},
                     (Point_t){coord.laser_x, coord.laser_y}) < 5) break;
    }
    // ret < 0 → 校验失败，跳过

    DL_UART_Main_getErrorStatus(K230_INST, DL_UART_ERROR_OVERRUN);
    mspm0_delay_ms(50);
}
```

---

## 4. 扫描搜索模式

当摄像头视野中没有目标时（`recv_data` 全为 0），系统会自动旋转云台搜索：

```c
// 逆时针搜索
if (recv_data[0] == 0 || recv_data[1] == 0 ||
    recv_data[2] == 0 || recv_data[3] == 0) {
    SMotor_Turn_Degree(&smotor_hor, -5);   // 逆时针转 5 度
    mspm0_delay_ms(60);                     // 等待电机响应
}

// 或顺时针搜索（不同任务方向不同）
if (recv_data[0] == 0 || recv_data[1] == 0 ||
    recv_data[2] == 0 || recv_data[3] == 0) {
    SMotor_Turn_Degree(&smotor_hor, 5);    // 顺时针转 5 度
    mspm0_delay_ms(60);
}
```

---

## 5. 其他串口传感器

### 5.1 WIT 维特智能姿态传感器 (UART_WIT)

通过 UART + DMA 接收姿态数据，中断中自动解析：

```c
// wit.h - 数据结构
typedef struct {
    float pitch;        // 俯仰角 (°)
    float roll;         // 横滚角 (°)
    float yaw;          // 偏航角 (°)
    float temperature;  // 温度 (°C)
    int16_t ax, ay, az; // 加速度 (mg)
    int16_t gx, gy, gz; // 角速度 (°/s)
    int16_t version;    // 固件版本
} WIT_Data_t;

extern WIT_Data_t wit_data;

// 初始化
WIT_Init();

// 读取数据 (DMA 自动更新 wit_data)
float current_yaw = wit_data.yaw;
```

### 5.2 BNO08X 姿态传感器 (UART_BNO08X)

```c
// bno08x_uart_rvc.h - 数据结构
typedef struct {
    uint8_t index;
    float pitch;        // 俯仰角 (°)
    float roll;         // 横滚角 (°)
    float yaw;          // 偏航角 (°)
    int16_t ax, ay, az; // 加速度
} BNO08X_Data_t;

extern BNO08X_Data_t bno08x_data;

// 初始化
BNO08X_Init();
```

### 5.3 ZDT_X42S 步进电机 (UART_ZDT)

闭环步进电机驱动，通过 RS485/UART 通信：

```c
#include "ZDT_X42S/zdt_x42s.h"

ZDT_HandleTypeDef motor1, motor2;

// 初始化
ZDT_Init(&motor1, UART_ZDT_INST, 1);   // 电机1, 地址1
ZDT_Init(&motor2, UART_ZDT2_INST, 1);  // 电机2, 地址1

// 角度移动（推荐使用快速位置模式）
ZDT_FastPosMoveAngle(&motor1, 45.0f, 500, 10, ZDT_MODE_REL_CURRENT);
//                        角度    转速 加速度 模式

// 速度控制
ZDT_SpeedCtrl(&motor1, ZDT_DIR_CW, 300, 20);
//                     方向       转速  加速度

// 回零操作
ZDT_SetHomeZero(&motor1, 1);               // 设置当前位置为零点并存储
ZDT_TriggerHome(&motor1, ZDT_HOME_NEAREST); // 触发就近回零
uint8_t status = ZDT_ReadHomeStatus(&motor1); // 读取回零状态
```

---

## 6. 中断配置要点（SysConfig）

### 6.1 K230 摄像头 UART 配置

在 TI SysConfig 中配置 `K230_INST` UART：
- 启用 RX 中断
- 根据摄像头波特率设置 Baud Rate
- 在 `interrupt.c` 中添加 `K230_INST_IRQHandler` 处理函数

### 6.2 WIT/BNO08X DMA UART 配置

参照头文件注释，关键步骤：
1. 添加 UART 模块并命名
2. 设置波特率（BNO08X: 115200）
3. 方向设为 "RX only"
4. 启用 FIFO，RX Timeout 设为 1
5. 配置 DMA：Fixed addr → Block addr，字节传输
6. 在中断中处理校验和与数据解析

---

## 7. 常见的调试场景

### 7.1 验证摄像头通信

在 Debug 页面（`page_state = 2`）中，系统会自动显示收到的摄像头数据：

```c
// slave_control.c - Page_Debug()
if (New_CMD_flag) {
    Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data);
    Pto_Clear_CMD_Flag();
    // OLED 显示目标坐标和激光坐标
    sprintf(oled_buffer, "Tpos:%4d,%4d", recv_data[2], recv_data[3]);
    sprintf(oled_buffer, "Lpos:%4d,%4d", recv_data[0], recv_data[1]);
}
```

### 7.2 常见问题排查

| 问题 | 可能原因 | 排查方法 |
|------|---------|---------|
| 收不到数据 | 波特率不匹配 | 确认 K230 与 MSPM0 波特率一致 |
| 帧解析失败 | 帧格式错误 | 检查帧头 `$` 帧尾 `#` 是否正确 |
| 协议 ID 不匹配 | `PTO_FUNC_ID` 不对 | 确认 K230 发送的协议 ID 是否为 4 |
| 缓冲区溢出 | 帧长超过 50 字节 | 增大 `PTO_BUF_LEN_MAX` 或减小帧大小 |
| 串口错误累积 | Overrun Error | 定期调用 `DL_UART_Main_getErrorStatus()` 清除 |
| 第一个数据混乱 | 上电残余数据 | 代码中 `flag=true` 跳过第一帧 |

---

## 8. 依赖关系

```
yb_protocol
├── stdint.h    (标准整数类型)
├── stdio.h     (sprintf 调试输出)
├── stdlib.h    (atoi 字符串转整数)
└── string.h    (内存操作)

调用方:
├── slave_control.c  (主控制逻辑)
├── main.c           (系统初始化)
└── interrupt.c      (K230_INST_IRQHandler)
```

---

## 9. K230 摄像头端协议实现（Python/MaixPy）

K230 端运行 MaixPy v4 脚本，通过 `YbProtocol` 类打包数据并通过 UART 发送给 MSPM0。

### 9.1 YbProtocol 发送端实现

```python
# K230 端 - 协议类定义
class YbProtocol:
    def package_circle(self, circle_id, x, y, r):
        """打包圆形目标信息"""
        return f"$31,04,{int(x):03d},{int(y):03d},000,000,{circle_id:03d},{int(r):03d}#\n"

    def package_number(self, num, x, y):
        """打包数字识别结果"""
        return f"$31,04,{int(x):03d},{int(y):03d},000,000,{num:03d},000#\n"

    def package_gap(self, x, y, w, h):
        """打包缺口识别结果"""
        return f"$31,04,{int(x):03d},{int(y):03d},{int(w):03d},{int(h):03d},000,000#\n"

    def package_laser(self, x, y):
        """打包激光点坐标"""
        return f"$31,04,{int(x):03d},{int(y):03d},000,000,000,000#\n"

    def package_maze(self, x, y, flag):
        """打包迷宫路径信息"""
        return f"$31,04,{int(x):03d},{int(y):03d},{int(flag):03d},000,000,000#\n"

yb_protocol = YbProtocol()

# UART 发送（逐字符发送以保证与下位机协议兼容）
def uart_send_char_by_char(data: str, delay_ms: int = 1):
    for ch in data:
        serial.write_str(ch)
        sleep_ms(delay_ms)
```

### 9.2 协议帧字段对照表

| 帧格式 | C 端 `recv_data` 索引 | 字段含义 | 示例场景 |
|--------|----------------------|---------|---------|
| `$31,04,xxx,yyy,000,000,id,rrr#` | `[0]=x, [1]=y, [4]=id, [5]=r` | 圆形靶目标 | 圆形识别 |
| `$31,04,xxx,yyy,000,000,num,000#` | `[0]=x, [1]=y, [4]=num` | 数字识别 | 数字目标 |
| `$31,04,xxx,yyy,www,hhh,000,000#` | `[0]=x, [1]=y, [2]=w, [3]=h` | 缺口/矩形目标 | 缺口检测 |
| `$31,04,xxx,yyy,000,000,000,000#` | `[0]=x, [1]=y` | 激光点坐标 | 激光瞄准 |
| `$31,04,xxx,yyy,flag,000,000,000#` | `[0]=x, [1]=y, [2]=flag` | 迷宫路径 | 迷宫导航 |

> **注意**: K230 端帧长度固定为 31，坐标值用 3 位零填充格式（如 `003`），帧尾带 `\n` 换行符。C 端解析时不依赖 `\n`，仅依赖 `$` 和 `#` 边界符。

### 9.3 激光检测实现

```python
# laser_detector.py - 激光光斑检测
class LaserDetector:
    def __init__(self, camera, display, serial):
        self.cam = camera
        self.thresholds = [(90, 105, -20, 20, -20, 20)]  # LAB 颜色阈值

    def process_frame(self, img):
        blobs = img.find_blobs(self.thresholds, ...)
        results = []
        for blob in blobs:
            results.append({'x': blob.cx(), 'y': blob.cy(), 'area': blob.area()})
        # 只保留面积最大的光斑
        if results:
            return max(results, key=lambda r: r['area'])
        return None
```

---

## 10. 硬件引脚分配

### 10.1 K230 摄像头串口

| 功能 | 引脚 | 方向 |
|------|------|------|
| K230 TX → MSPM0 RX | **PB18** | INPUT |
| MSPM0 TX → K230 RX | **PB17** | OUTPUT |
| 波特率 | 115200 | — |
| UART 实例 | UART2 (Main) | — |

### 10.2 完整串口引脚图

```
MSPM0G3507 (LQFP-64) UART 引脚布局:

  UART0 (Main):   PA11 (RX) ← WIT 陀螺仪
  UART1 (Main):   PB4  (TX) → 电机2 ZDT,  PB5  (RX) ← 电机2 ZDT
  UART2 (Main):   PB17 (TX) → K230,       PB18 (RX) ← K230
  UART3 (Extend): PA26 (TX) → 电机1 ZDT,  PA13 (RX) ← 电机1 ZDT
```

> ⚠️ **重要**: UART3 是 Extend 类型，初始化必须用 `DL_UART_Extend_*` API；UART0/1/2 是 Main 类型，用 `DL_UART_Main_*` API。

---

## 11. 与项目其他模块的交互

### 11.1 主状态机中的协议使用

```
main.c 主循环
  └── page_state 状态机
        ├── 0: Page_Home()     — 菜单导航
        ├── 1: Page_Calib()    — PID 参数校准
        ├── 2: Page_Debug()    — 显示 K230 实时坐标
        ├── 3: Page_Test()     — 核心任务（使用协议数据）
        │     ├── Task 1: 循迹绕圈
        │     ├── Task 2: 固定位置激光瞄准
        │     ├── Task 3: 旋转 + 激光瞄准
        │     ├── Task 4: 逆时针扫描瞄准
        │     ├── Task 5: 顺时针扫描瞄准
        │     └── Task 6: 循迹 + 实时目标跟踪
        ├── 4: Page_Debug_Step() — 步进电机调试
        └── 5: Page_Debug_Gyro() — 陀螺仪调试
```

### 11.2 PID 瞄准控制回路

```
K230 摄像头                    MSPM0 主控
  │                              │
  ├─ 采集图像 ──────────────────►│
  ├─ AI 识别激光/目标              │
  ├─ 打包帧 ────────────────────►│ Pto_Data_Receive() (ISR)
  │                              │ Pto_Data_Parse() → (lx,ly, tx,ty)
  │                              │
  │                              ├─ 水平 PID: err = tx - lx
  │                              │   └─ SMotor_Turn_To(&smotor_hor, PID_out)
  │                              │
  │                              ├─ 垂直 PID: err = ty - ly
  │                              │   └─ SMotor_Turn_To(&smotor_ver, PID_out)
  │                              │
  │                              ├─ 判断: Distance(target, laser) < 5 ?
  │                              │   ├─ YES → 触发激光, 蜂鸣提示, 任务完成
  │                              │   └─ NO  → 等待下一帧继续
  │                              │
  │◄────── 下一帧 ──────────────┤ (循环)
```

---

## 12. 快速集成指南

1. **复制文件**: 将 `yb_protocol.h` 和 `yb_protocol.c` 添加到工程
2. **SysConfig 配置**: 创建名为 `K230_INST` 的 UART 实例（UART2 Main），TX=PB17, RX=PB18，波特率 115200，启用 RX 中断
3. **中断注册**: 在 `slave_control.c` 中实现 `K230_INST_IRQHandler`，调用 `Pto_Data_Receive()`
4. **主循环轮询**:
   ```c
   if (New_CMD_flag) {
       Pto_Data_Parse(RxBuffer, New_CMD_length, recv_data);
       Pto_Clear_CMD_Flag();
       // recv_data[0]=laser_x, [1]=laser_y, [2]=target_w, [3]=target_h
       // recv_data[4]=target_id, [5]=degrees
   }
   ```
5. **K230 端配合**: 确保 K230 脚本中的 `YbProtocol` 类格式与 C 端 `PTO_FUNC_ID=4` 一致
6. **共地**: K230 与 MSPM0 必须共地（GND 连接）
7. **根据实际摄像头协议调整**: 修改 `PTO_FUNC_ID` 和字段映射关系

---

## 附录 A: 项目文件索引

| 路径 | 说明 |
|------|------|
| [yb_protocol/yb_protocol.h](yb_protocol.h) | 协议头文件 |
| [yb_protocol/yb_protocol.c](yb_protocol.c) | 协议实现 |
| [Slave_Control/slave_control.c](../Slave_Control/slave_control.c) | 主控逻辑（含 K230 ISR） |
| [main.c](../main.c) | 程序入口与初始化 |
| [Drivers/WIT/wit.c](../Drivers/WIT/wit.c) | WIT 陀螺仪 UART+DMA 驱动 |
| [Drivers/BNO08X_UART_RVC/bno08x_uart_rvc.c](../Drivers/BNO08X_UART_RVC/bno08x_uart_rvc.c) | BNO08X UART 驱动 |
| [ZDT_X42S/zdt_x42s.c](../ZDT_X42S/zdt_x42s.c) | 步进电机 RS485/UART 驱动 |
| [PROJECT_GUIDE.md](../PROJECT_GUIDE.md) | 项目总体指引 |
| [ZDT_X42S/PINMAP.md](../ZDT_X42S/PINMAP.md) | 完整引脚分配表 |
| [First round of visuals/](../../First%20round%20of%20visuals/) | K230 端 Python 视觉脚本 |
| [demo3/test2/](../../demo3/test2/) | K230 端检测模块（激光/迷宫/形状等） |
