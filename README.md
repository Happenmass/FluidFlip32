# M5StackFluidSimulate

> 本 README 主要面向 AI Agent，用于在最短时间内建立项目心智模型并开始动手。
> 人类读者也可读，但行文以"够用即止、信息密度优先"为原则。

## 一、项目是什么

在 **M5StickC S3**（ESP32-S3 + 240×135 ST7789 LCD + BMI270 IMU + 两个按键）上跑一个实时**流体模拟**，把屏幕当作一个装着水的小容器：晃动设备 → IMU 读取重力方向 → 模拟出水的晃动、波纹、液面光影。

![实机效果参考](assets/example.jpg)

灵感与算法参考自 mitxela 的 FLIP 流体挂坠以及 Matthias Müller "Ten Minute Physics"。本仓库的 [flip-card/](flip-card/) 子目录是上游硬件参考工程（RP2350 + 自制 PCB），**只读参考、不要改动**。

## 二、目录结构（你只需要关心两个固件目录）

| 路径 | 状态 | 说明 |
|------|------|------|
| [firmware/](firmware/) | 旧版（PIC FLIP，当前 main 分支有改动） | 40×22 网格，单核，opaque/blob 双渲染风格 |
| [firmware_flip/](firmware_flip/) | **新版（FLIP，重写中、推荐主攻方向）** | 40×19 网格、1000 粒子、双核（core1 物理 / core0 渲染+IMU），8 级密度 LUT |
| [flip-card/](flip-card/) | 上游参考（RP2350） | **只读**，不要修改 |
| [docs/superpowers/](docs/superpowers/) | 设计 spec 与 plan | 排错或扩功能前先翻一下 |

> **不确定该改哪一套时，默认改 `firmware_flip/`。** 如果用户提到 "PIC FLIP"、"opaque rim+sparkle"、"renderer.cpp 里的 alpha blending" 等才是在说 `firmware/`。

## 三、硬件 / 工具链

- **目标板**：M5StickC S3（PlatformIO board id：`m5sticks3`，自带 JSON 在各固件的 `boards/` 下）
- **MCU**：ESP32-S3（双核 Xtensa LX7，8 MB Flash，512 KB SRAM + 8 MB PSRAM 视型号）
- **框架**：Arduino + ESP-IDF（通过 platformio）
- **关键依赖**：
  - `m5stack/M5Unified@^0.2.0`（按键、IMU、屏幕初始化）
  - `lovyan03/LovyanGFX@^1.1.16`（仅 `firmware_flip/` 用，做精灵双缓冲）
- **构建工具**：PlatformIO Core（已安装在 `/opt/homebrew/bin/pio`，Mac 上用 `pio` 即可）
- **C++ 标准**：`-std=gnu++17`（`build_unflags = -std=gnu++11` 是必需的，移除会编译失败）

## 四、编译 & 烧录命令

> **重要前置**：M5StickC S3 通过板载 USB-C 直连即可烧录，不需要按住任何键进入下载模式（ESP32-S3 的 USB-JTAG 自动复位）。如果烧录失败再考虑手动 reset。

### `firmware_flip/`（推荐主攻）

```bash
# 进入目录（pio 命令默认在当前工作目录找 platformio.ini）
cd firmware_flip

# 仅编译（不烧录）—— Agent 自检语法时用这个，最快
pio run -e m5sticks3

# 编译 + 烧录到设备
pio run -e m5sticks3 -t upload

# 编译 + 烧录 + 打开串口监视器（115200 baud）
pio run -e m5sticks3 -t upload -t monitor

# 仅打开串口监视器（看 [boot] / [fps] / [imu] 日志）
pio device monitor -b 115200
```

### `firmware/`（旧版）

```bash
cd firmware
pio run -e m5sticks3 -t upload      # 烧录
pio run -e native -t test           # 跑单元测试（仅 fluid_sim.cpp，主机端 Unity）
```

`firmware/` 多一个 `[env:native]` 用于 host-side 单测——它**不依赖硬件**，用 `pio test -e native` 即可在 Mac 上跑。`firmware_flip/` 目前没有 native 测试环境。

### 串口设备路径

烧录前先确认设备已插好：

```bash
ls /dev/cu.usbserial-* /dev/cu.usbmodem* 2>/dev/null
```

PlatformIO 会自动选择第一个 ESP32-S3 USB JTAG 设备；如果有多个 USB 串口干扰，可以在命令里加 `--upload-port /dev/cu.usbmodem...`。

## 五、运行时交互（写代码前要知道）

| 输入 | `firmware/` 行为 | `firmware_flip/` 行为 |
|------|------------------|------------------------|
| 倾斜设备 | IMU → 重力向量驱动模拟 | 同左 |
| BtnA 短按 | 反重力速度脉冲（"爆开"） | —— |
| BtnA 长按 | —— | 10 Hz 从顶部注入新粒子（最多到 `kMaxParticles`） |
| BtnB 短按 | 增加 100 粒子 | 重置粒子到 `kInitParticles` |
| BtnB 长按（≥1s） | 重置场景 | —— |

启动后串口会打印 `[boot]`、`[cfg]`、`[fps]`，`firmware_flip/` 头 5 秒还会打印 `[imu]` 校准窗口数据。

## 六、改代码时的关键路径

### `firmware_flip/`

- 物理：[src/fluid/flip_solver.cpp](firmware_flip/src/fluid/flip_solver.cpp) + [mac_grid.h](firmware_flip/src/fluid/mac_grid.h)
- 渲染：[src/render/renderer.cpp](firmware_flip/src/render/renderer.cpp)（LovyanGFX 精灵 → push 到 LCD）
- IMU：[src/imu/imu_reader.cpp](firmware_flip/src/imu/imu_reader.cpp)
- **所有可调参数集中在** [src/config.h](firmware_flip/src/config.h)（网格大小、粒子数、重力、FLIP 比例、密度 LUT、抗闪烁 EMA、注入节奏、双核任务栈/优先级）。改行为优先改这里。
- 双核线程模型在 [src/main.cpp](firmware_flip/src/main.cpp) 顶部注释里讲得很清楚——动它之前先读那段。

### `firmware/`

- 物理：[src/fluid_sim.cpp](firmware/src/fluid_sim.cpp)（host-portable，可被 native test 链接）
- 渲染：[src/renderer.cpp](firmware/src/renderer.cpp)（两套渲染风格由 `kEnableAlphaBlending` 切换）
- 输入：[src/input.cpp](firmware/src/input.cpp)
- 配置：[src/fluid_sim_config.h](firmware/src/fluid_sim_config.h)

## 七、code-review-graph 知识图谱

仓库已构建 code-review-graph（见 `.code-review-graph/`），覆盖两套固件的 C/C++ 源码。在做任何"找调用者 / 找影响范围 / 改动评审"类操作时，**先用 MCP 工具 `mcp__code-review-graph__*` 而不是 Grep**——更快、更省 token、附带结构上下文。详见仓库根目录 `CLAUDE.md`。

## 八、常见坑位

1. **编译报 `std::atomic` 相关错误** → 检查 `build_unflags = -std=gnu++11` 是否被误删，必须用 gnu++17。
2. **烧录卡在 `Connecting...`** → USB-C 数据线只有充电功能 / 系统占用了串口（关掉其他 monitor 进程）。
3. **屏幕黑屏但串口正常** → `firmware_flip/` 渲染依赖 LovyanGFX 1.1.16+，先 `pio pkg update`。
4. **IMU 读数全 0** → 确认 `cfg.internal_imu = true`，`firmware_flip` 在 `FLIP_USE_IMU=0`（Stage<2）时会强制关 IMU。
5. **粒子瞬移 / 爆炸** → 多半是 `kFixedDt` 或 `kGravityMagnitude` 改过了，先用 git diff 比对 `config.h`。
6. **不要 `--no-verify` 跳过 hook**，不要在主分支强推。
