# gdut_rc_embedded_fw

> Real-time embedded firmware framework for Robocon | STM32H7 + FreeRTOS + Modern C++

> **Note**: This repository was formerly named `General_Framework`. If you encounter that name in old documentation or code, it refers to this project.

[中文文档](./README_CN.md)

## Overview

`gdut_rc_embedded_fw` is a robot embedded firmware framework running on **STM32H723**. It uses **FreeRTOS** as the RTOS kernel and **CMake + st-armclang** as the build toolchain. The project builds a complete modular hierarchy atop bare-metal drivers and HAL, incorporating lock-free data structures, dependency injection, and compile-time optimization — bringing modern software engineering practices to embedded development.

## Layer Architecture

```
┌─────────────────────────────┐
│  APP Layer   │  chassis / control / debug / watchdog / robot_com  │
├─────────────────────────────┤
│  Algorithm   │  lockfree_queue / double_buffer / ringbuffer       │
├─────────────────────────────┤
│  Module      │  Motor drivers / sensors / communication protocols │
├─────────────────────────────┤
│  BSP         │  Canbus / UartPort / UsbPort / bsp_dwt            │
├─────────────────────────────┤
│  HAL         │  STM32CubeMX-generated peripheral drivers          │
└─────────────────────────────┘
```

## Design Principles

### 1. Lock-Free Concurrency — CAS Operations

On single-core MCUs, race conditions arise from high-priority ISRs preempting lower-priority code. This framework uses CAS (Compare-And-Swap) atomic operations based on `LDREX`/`STREX` to implement lock-free queues (MPSC), avoiding potential deadlocks from mutex usage inside ISRs.

```cpp
// Multi-producer safe enqueue — spins on CAS conflict
queue.push(data);
```

### 2. Compile-Time Determinism

- Static global memory allocation to avoid runtime fragmentation
- STL containers with dynamic allocation (`std::queue`, `std::deque`) are discouraged
- All data structure sizes fixed at compile time for predictable runtime behavior

### 3. Hardware-Ready Moment Awareness

CAN transmission uses dual-trigger: periodic polling + TX-complete ISR chaining. The TX-complete ISR immediately dequeues the next frame, allowing multiple frames per 1 ms cycle instead of just one.

### 4. Dependency Injection & Interface Abstraction

HAL handles are injected into BSP classes via constructor parameters. Initialization is deferred to an `Init()` interface to ensure proper ordering:
```
HAL init → BSP init → Device init → OS boot → Per-task init
```

### 5. Topic-Based Task Decoupling

Tasks exchange data through a publish/subscribe model, avoiding direct global variable access and clarifying business logic boundaries while maintaining thread safety.

## Task List

| Task | Source | Function |
|------|--------|----------|
| chassis_task | `APP/chassis_task/` | Chassis motion execution |
| control_task | `APP/control_task/` | Core control logic |
| debug_task | `APP/debug_task/` | Debug data output |
| robot_com | `APP/robot_com/` | Inter-robot communication |
| watchdog_task | `APP/watchdog_task/` | System watchdog |

## Toolchain

| Component | Selection | Notes |
|-----------|-----------|-------|
| Build | CMake + CMakePresets | Cross-platform, unified embedded & ROS workflow |
| Compiler | st-armclang (ARM Compiler 6) | C++17 support |
| Debugger | Ozone (SEGGER) | Superior debugging experience |
| Flashing | J-Flash / OpenOCD | Flexible options |
| MCU | STM32H723VGT6 | Cortex-M7 @ 550MHz |

## Quick Start

```shell
# Configure
cmake --preset stm32h7

# Build
cmake --build --preset stm32h7

# Flash (J-Link)
JFlash -openprj project.jflash -open firmware.hex -auto -exit
```

## Related Repos

- [rc_ecat_controls](https://github.com/KetenBieber/rc_ecat_controls) — ROS EtherCAT controllers
- [easy_ecatslave_hw](https://github.com/KetenBieber/easy_ecatslave_hw) — EtherCAT slave hardware
- [rc25_path](https://github.com/KetenBieber/rc25_path) — Path planning & Gazebo simulation
