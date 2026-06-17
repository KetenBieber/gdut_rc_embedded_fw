# gdut_rc_embedded_fw

[English](./README.md)

> 面向 Robocon 竞赛的嵌入式实时固件框架 | STM32H7 + FreeRTOS + 现代 C++

> **注意**: 本仓库前身为 `General_Framework`，如果你在旧文档/代码中看到该名称，指的就是这里。

## 项目简介

`gdut_rc_embedded_fw` 是一套运行在 **STM32H723** 上的机器人嵌入式固件框架，以 **FreeRTOS** 为操作系统内核，采用 **CMake + st-armclang** 工具链构建。项目在裸机驱动和 HAL 库之上构建了完整的模块化层级，并引入无锁数据结构、依赖注入、编译期优化等现代软件工程实践。

## 架构分层

```
┌─────────────────────────────┐
│  APP 层  │  chassis / control / debug / watchdog / robot_com  │
├─────────────────────────────┤
│  Algorithm 层 │  lockfree_queue / double_buffer / ringbuffer  │
├─────────────────────────────┤
│  Module 层  │  电机驱动 / 传感器驱动 / 通信协议                │
├─────────────────────────────┤
│  BSP 层     │  Canbus / UartPort / UsbPort / bsp_dwt          │
├─────────────────────────────┤
│  HAL 层     │  STM32CubeMX 生成的外设驱动                      │
└─────────────────────────────┘
```

## 设计原则

### 1. 无锁并发 — CAS 原子操作

在单核 MCU 中，竞态条件来源于高优先级中断打断低优先级代码。框架采用基于 `LDREX`/`STREX` 的 CAS 原子操作实现无锁队列（MPSC），避免在 ISR 中使用互斥锁可能导致的死锁问题。

```cpp
// 多生产者安全入队，发生竞争时自旋重试
queue.push(data);  // 内部使用 CAS 保证原子性
```

### 2. 编译期确定性

- 全局静态分配内存，避免运行时动态分配导致的内存碎片
- 不推荐使用 STL 中动态分配的数据结构（`std::queue`、`std::deque`）
- 所有数据结构大小在编译期确定，运行时状态可预测

### 3. 硬件就绪时刻感知

CAN 发送采用双触发机制——定时轮询 + 发送完成中断续发。发送完成 ISR 中立即从队列取下一帧发送，将 1ms 只能发一帧提升为数帧，充分利用总线带宽。

### 4. 依赖注入 & 接口抽象

HAL 句柄通过构造函数参数注入到 BSP 类，初始化延后到 `Init()` 接口，避免 HAL 未就绪时访问外设：
```
HAL 初始化 → BSP 初始化 → 设备初始化 → OS 启动 → 任务内初始化
```

### 5. 话题通信解耦

Task 之间通过发布/订阅模式交换数据，避免全局变量直接读写，保障线程安全同时明确业务逻辑边界。

## 任务列表

| 任务 | 文件 | 功能 |
|------|------|------|
| chassis_task | `APP/chassis_task/` | 底盘运动执行 |
| control_task | `APP/control_task/` | 核心控制逻辑 |
| debug_task | `APP/debug_task/` | 调试数据输出 |
| robot_com | `APP/robot_com/` | 机器人间通信配置 |
| watchdog_task | `APP/watchdog_task/` | 系统看门狗 |

## 工具链

| 组件 | 选型 | 说明 |
|------|------|------|
| 构建系统 | CMake + CMakePresets | 跨平台，统一嵌入式与 ROS 工作流 |
| 编译器 | st-armclang (ARM Compiler 6) | 支持现代 C++17 特性 |
| 调试器 | Ozone (SEGGER) | 比 Keil 更强的调试体验 |
| 烧录 | J-Flash / OpenOCD | 灵活选择 |
| MCU | STM32H723VGT6 | Cortex-M7 @ 550MHz |

## 快速开始

```shell
# 配置 CMakePresets
cmake --preset stm32h7

# 编译
cmake --build --preset stm32h7

# 烧录（J-Link）
JFlash -openprj project.jflash -open firmware.hex -auto -exit
```

## 相关仓库

- [rc_ecat_controls](https://github.com/KetenBieber/rc_ecat_controls) — 上位机 ROS EtherCAT 控制器
- [easy_ecatslave_hw](https://github.com/KetenBieber/easy_ecatslave_hw) — EtherCAT 从站硬件固件
- [rc25_path](https://github.com/KetenBieber/rc25_path) — 路径规划与 Gazebo 仿真
