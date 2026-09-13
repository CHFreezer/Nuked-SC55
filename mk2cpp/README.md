# mk2cpp — SC-55mk2 固件 C++ 语义级翻译工程

目标：**把 SC-55mk2 的固件行为和声音合成逻辑，恢复为人能直接理解、修改和维护的
原生 C++ 程序**。源码应像为现代 x86/amd64 编写的软件合成器，以完整算法、命名状态和
正常的函数/分支/循环表达 MIDI、音符、voice、包络及声音生成；保持 MK2 原有逻辑和
可观察行为，先完成原版 28 复音兼容，再扩展到 256 个发声单元。

终态原生后端的启动、复位和正常播放直接执行上述算法，不运行 H8/M37450 指令、
逐 PC 生成码或解释器回退。原解释器与逐指令翻译保留为参考后端和迁移工具。
**透明 bank 模式零原 ROM 依赖**：离线提取采样元数据和波形，保存为 JSON + WAV，
支持独立试听、编辑、替换并重新加载生效；未编辑原厂素材保持无可闻差异，不要求 bit 一致。
原生后端也支持显式加载原 wave ROM，在同配置 stock 28 下保持 bit 对照。
两种音源选择都使用提取后的音色/参数表，不运行原固件。

算法需要的定宽整数、定点、截断和饱和可以保留。
产物以库集成到原 Nuked SC-55 程序，**保留并复用 nukeykt 的 PCM 芯片仿真、LCD、
RtMidi、SDL、资源加载及其他可用设施**。PCM 仿真是两个分支共同的声音基础，
M4.5 通过控制接口与时钟适配驱动它，保留其合成行为。

**终态入口（M4.5 待实现）**：不加 `-mk2cpp`，维持原模拟器的 H8/子 MCU 与设备
运行路径；加 `-mk2cpp`，进入原生 C++ 控制分支，继续使用上述设施与 PCM 仿真。
不新增程序、不更换默认后端。现有 `-mk2cpp` 仍是 M1–M4 的逐指令混合翻译入口，
尚未实现这个原生分支；M4.5 将沿用同一 flag 完成切换。

**进度更新（2026-09-13）**：M1–M3 指令翻译已完成。M4 以 `97a0ae5` 为
**hand 逐指令整理与 stock 28 行为回归检查点**收口，保留其全部成果，不回退。
当前 hand 覆盖 4239 个 PC 入口，已消除 `case 0x`，但主体仍是操作寄存器和 PC 的
`step_*` 指令函数，尚未达到完整算法表达。既有 n=28 音频 null 与状态产物一致；
提交另记录回归曲、boot/定点窗口和用户试听通过，证据边界见 [复核](docs/12_polyphony_reassessment.md)。

新增 **M4.5：原生算法重建（stock 28）**，承接原 M4 尚未完成的人类语义目标。
验收要求算法可读、控制行为等价、脱离 MCU 指令执行；透明 bank 模式全生命周期无需原 ROM。
**M5：真 256 复音扩展**仍暂缓，待 M4.5 完成后在原生状态与声音内核上实施。
目标全文与验收标准见 [M4.5 规格](docs/13_m45_native_reconstruction.md)。

**M4.5 施工入口**：[14 架构](docs/14_m45_native_architecture.md) 定义模块、状态、
设备接口与时序边界；[15 施工规程](docs/15_m45_construction_guide.md) 定义算法契约、
阶段交付和首个完整算法样板；[16 透明 bank](docs/16_transparent_bank.md) 定义
元数据/波形提取、JSON + WAV 存储与编辑使用。旧 07–11 用作证据，不直接照其 L0 草案施工。

命名说明：`mk2cpp` = **MK2 ROM → C++**（准确）。不使用 `h8cpp` 这类名字——
GT 是可跑多种固件的 H8 模拟器，本工程翻译的是 **MK2 的 ROM 代码**，不是重写 CPU 核；
集成开关/符号统一 `mk2cpp`/`MK2CPP_`/`mk2c_` 前缀。

参考与证据：

- ROM 字节及既有反汇编基线用于恢复固件算法（本地资产，**不入 git**）。
- GT 仿真器 `../src/` 是重建等价性的运行参照，固定 ROM、输入时序和 PCM/滤波配置比较。
- PCM 开盖逆向来源与 MK2 滤波的已知模型差异分别记录；GT 对照通过不等于实机差异已解决，
  见 [PCM 来源与滤波边界](docs/12_polyphony_reassessment.md)。

## 目录

```
mk2cpp/
  README.md            本文件（目标/Ground Truth/政策）
  include/mk2cpp.h     集成 ABI（MK2CPP_Init/CanStep/Step、开关、诊断；入库）
  src/mk2cpp.cpp       集成胶水：分派表、回退统计、版本（入库）
  src/gen/             自动翻译产物（**ROM 派生，不入 git**，本地生成，CMake 可选编译）
                       顶层 = 主 H8（mk2c_r1/r2 + init）；sm/ = 子 MCU（mk2c_sm + init，M3）
  src/hand/            M4 手工逐指令参考实现（voice/PCM；M4.5 逐步重建算法）
  tools/h8lift/        ROM → C++ 反译器（入库）
  tools/tracediff/     两模式 trace/状态差分（入库）
  tools/cover/         覆盖率仪表盘（入库）
  docs/                00–02 计划/架构/约定；03–11 旧研究；12 复评；13 目标；14 架构；15 施工规程；16 透明 bank
  tests/               oracle 脚本（入库）；语料/快照本地
  out/                 运行/中间产物（不入 git）
```

## 构建（GT 集成）

本节命令描述当前 M1–M4 构建与对照方式；M4.5 的原生后端接入尚待实施。

GT 用 Ninja + clang-cl 构建，SDL2 用本地 SDL2 目录（`SDL2_DIR` 指向其 cmake 目录）：

```
cmake -S . -B build -G Ninja ^
  -DCMAKE_C_COMPILER="C:/Program Files/LLVM/bin/clang-cl.exe" ^
  -DCMAKE_CXX_COMPILER="C:/Program Files/LLVM/bin/clang-cl.exe" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DSDL2_DIR="<本地 SDL2 目录>/cmake"
cmake --build build
```

- 默认**不含自动生成码**；已编译并注册的 hand 入口仍可参与 `-mk2cpp`，其余入口回退。
  `no translated code linked` 描述 gen 链接状态，不能单凭它判定整个运行都在解释器中。
- 本地启用生成码：追加 `-DMK2CPP_GEN_DIR=<repo>/mk2cpp/src/gen`（生成物不入 git）。
- 验证（两模式零回归）：
  `nuked-sc55.exe -mk2 -demo -tracepc <out> 200000000 202000000` vs
  `nuked-sc55.exe -mk2 -mk2cpp -demo -tracepc <out> 200000000 202000000`，
  再与 `tools/baselines` 的基准 trace 比对。

## 硬性规则

1. **研究先行**：遇到说不清的阻碍，先停实现、补证据文档，再恢复
   （见 `../tools/docs/evidence_protocol.md` 与 `polyphony_256_todo.md` 施工规则）。
2. **只提交代码与文档**：ROM 及派生产物（反汇编、trace、快照、`src/gen/` 输出）
   一律本地。`mk2cpp/` 是**已入库的公开目录**，其内部派生路径的忽略规则写在
   公开 `.gitignore` 的 `mk2cpp:` 段，随仓库共享；**不得在任何公开文件（md、
   `.gitignore` 注释等）中写出本地私有路径名或本地目录布局**，本地资料的
   忽略方式不入库文档。
   提交前需确认：`mk2cpp/out/`、`mk2cpp/build/`、`mk2cpp/src/gen/`、
   `mk2cpp/tests/*.bin|*.txt` 均被 `.gitignore` 命中（`git check-ignore` 验证；
   `mk2cpp/build/` 由通用 `build/` 规则命中）。
3. **等价性**：M1–M3 期间，同一输入下 `-mk2cpp` 模式与默认解释器模式必须逐指令/逐状态一致
   （同一 GT 二进制两模式对照）；任何简化都必须标注并给出 oracle。M4 起才允许语义化改写
   改变实现方式，且必须通过音频/行为对照。所有 oracle GT 运行必须加 `-nomidi`
   （否则宿主 MIDI 端口的外部字节会被注入、造成 hash 漂移）。
4. **测试预算与范围**：默认回归只跑回归曲（`python3 mk2cpp/tests/m4_quick_gate.py`，
   约 65 s）；demo 60 s 预算（`--demo`，`[144M,1440M)`，约 61 s）；boot/demo200
   两模式（`two_mode_check.py`）仅在主链路改动时跑。**只测回归曲，不再默认/半默认
   测其他曲子**；全量语料（规模见 local.md）与压力矩阵仅用户明确要求时运行，且必须
   分批、汇报耗时。单次前台预算 ≤3 分钟，超时后台/分片并先告知。
5. **测试脚本跨平台**：测试一律用 Python 3 标准库（`tests/*.py`），不写平台路径，
   GT 可执行文件按平台自动探测（`build/nuked-sc55[.exe]`）。**不得把嵌入式解释器
   提交进仓库**；各用户使用自己机器的解释器（本机环境因人而异，见仓库根
   `local.md`，不入库）。门禁已无 `.ps1`（旧脚本全部移植/删除）；
   `m4_stress_voices` 随 M5 暂缓，重启时以 Python 重写。
6. **目录卫生**：调试产物写 `mk2cpp/out/` 或 `%TEMP%\opencode\`，任务收尾清空；
   `../build/` 只留运行资产。

## 里程碑

| 阶段 | 交付 | 验收 oracle |
|---|---|---|
| M1 ✅ | `mk2cpp.h` 集成（`-mk2cpp` + 混合回退）+ h8lift（h8dec/h8part/h8emit）+ tracediff + cover + hashdump | **已达成（2026-09-11）**：9217 PC 注册；boot 0–3M 与 demo 200–202M 两模式 trace + 状态哈希 + 基准 trace 全部一致 |
| M2 ✅ | 主固件全执行面翻译（含未执行可达路径） | **已达成（2026-09-11）**：15999 PC（9217 执行集 + 6782 可达新增）全译、0 stub（13 处 `TODO(gt)`）；boot+demo200 两模式 trace + 状态哈希与 M1 基线一致 |
| M3 ✅ | 子 MCU 固件翻译（`smemit` 全译 rom_sm 4KB）+ SM/主 CPU 5× 时序对接 | **已达成（2026-09-11）**：4096 SM PC 全译；boot+demo200 两模式 `s` 行 SM trace 逐条一致、hashdump 与 M1 基线同哈希（含 `hash.sm`/`sm_ram`/`hash.lcd_state`）；执行集（45/251 PC）全落翻译区间 |
| M4 ✅（范围重划） | hand 逐指令整理、命名与模块划分，stock 28 回归检查点（`97a0ae5`） | 4239 PC；既有音频 null/状态产物一致，其他回归与试听记录见 12 §1.2；原人类语义目标移交 M4.5 |
| **M4.5（待实施）** | **原生算法重建（stock 28）**：完整算法/状态/调度、透明 bank 与 WAV 替换 | 可读性 + wave ROM 模式 bit 对照 + 透明 bank 试听/编辑 + 无 MCU 执行；透明 bank 零原 ROM 依赖 |
| M5（暂缓） | 原生 voice 池与 PCM 内核的真 256 复音扩展 | 依赖 M4.5；验证容量边界、效果槽隔离、混音与实时性能；旧 255 上限不是最终验收 |

详细设计见 [计划](docs/00_plan.md)、[架构](docs/01_architecture.md)、[约定](docs/02_conventions.md) 与 [M4.5 目标及验收](docs/13_m45_native_reconstruction.md)。
