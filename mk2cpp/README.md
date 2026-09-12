# mk2cpp — SC-55mk2 固件 C++ 语义级翻译工程

目标：把 **SC-55mk2 固件（rom1/rom2/rom_sm）**语义级翻译为 C++，以 **`mk2cpp.h`
库形式集成回 GT，替换原 H8 解释器行为**（开关控制、未翻译 PC 可回退），
设备/调度/音频全部复用 GT。当前聚焦**原版 28 复音固件的 C++ 翻译**：M1–M3 直译
（主 H8 + 子 MCU 全执行面）已达成，`-mk2cpp` 与默认解释器逐指令/逐状态等价。
**M4 = voice/PCM 语义化改写（stock 28），闭包翻译完成**：主链路与 voice 闭包例程
（池/分配释放、note 物化与描述符链、P0 分派簇、ts_scan/C9、interp、0x4DE7 族、
init/reset/d1ac、共享 helper、interp 间接目标、pool 续段、命令环 handler 等）
已全部手写 C++ 并逐指令接入，累计 **4239 个 PC 入口**，静态可达闭包
**closure−gen−hand = 0**（2026-09-12）；**82/82 唯一 MIDI 曲目两模式逐字节
MATCH**（含 037/040 全曲），实时试听通过；剩 185 个闭包外动态 PC（事件环/part/
主循环等非 voice 子系统）与死字节/竞态不可达臂由 mixed 回退；快照审查确认
hand 无私有状态，无需 `MK2CPP_StateSave/Load`；
**M5 = 256 复音扩展**（`pcm_ext_*`/`-voices:`/0xE800 窗口/page6-7/`PCM_MAX_VOICE`）
是翻译目标之外的独立能力研究，已于 **2026-09-11 回滚暂缓**。
**不做独立可执行程序。**

命名说明：`mk2cpp` = **MK2 ROM → C++**（准确）。不使用 `h8cpp` 这类名字——
GT 是可跑多种固件的 H8 模拟器，本工程翻译的是 **MK2 的 ROM 代码**，不是重写 CPU 核；
集成开关/符号统一 `mk2cpp`/`MK2CPP_`/`mk2c_` 前缀。

Ground Truth（不可怀疑）：
- ROM 字节（本地资产目录或 `../build/`，**不入 git**）
- GT 仿真器 `../src/`（H8 语义、PCM、定时器、SM）
- 既有反汇编基线 `../tools/baselines/`（本地 fixture）

## 目录

```
mk2cpp/
  README.md            本文件（目标/Ground Truth/政策）
  include/mk2cpp.h     集成 ABI（MK2CPP_Init/CanStep/Step、开关、诊断；入库）
  src/mk2cpp.cpp       集成胶水：分派表、回退统计、版本（入库）
  src/gen/             自动翻译产物（**ROM 派生，不入 git**，本地生成，CMake 可选编译）
                       顶层 = 主 H8（mk2c_r1/r2 + init）；sm/ = 子 MCU（mk2c_sm + init，M3）
  src/hand/            人工语义化改写（voice/PCM 子系统；入库策略见 02）
  tools/h8lift/        ROM → C++ 反译器（入库）
  tools/tracediff/     两模式 trace/状态差分（入库）
  tools/cover/         覆盖率仪表盘（入库）
  docs/                设计与约定（00–11：00_plan / 01_architecture / 02_conventions / 03–06 研究 / 07–11 M4/M5 设计）
  tests/               oracle 脚本（入库）；语料/快照本地
  out/                 运行/中间产物（不入 git）
```

## 构建（GT 集成）

GT 用 Ninja + clang-cl 构建，SDL2 用本地 SDL2 目录（`SDL2_DIR` 指向其 cmake 目录）：

```
cmake -S . -B build -G Ninja ^
  -DCMAKE_C_COMPILER="C:/Program Files/LLVM/bin/clang-cl.exe" ^
  -DCMAKE_CXX_COMPILER="C:/Program Files/LLVM/bin/clang-cl.exe" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DSDL2_DIR="<本地 SDL2 目录>/cmake"
cmake --build build
```

- 默认**不含**生成码：`-mk2cpp` 会打印 `no translated code linked` 并整体回退解释器。
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
   改变实现方式，且必须通过音频/行为对照。
4. **目录卫生**：调试产物写 `mk2cpp/out/` 或 `%TEMP%\opencode\`，任务收尾清空；
   `../build/` 只留运行资产。

## 里程碑

| 阶段 | 交付 | 验收 oracle |
|---|---|---|
| M1 ✅ | `mk2cpp.h` 集成（`-mk2cpp` + 混合回退）+ h8lift（h8dec/h8part/h8emit）+ tracediff + cover + hashdump | **已达成（2026-09-11）**：9217 PC 注册；boot 0–3M 与 demo 200–202M 两模式 trace + 状态哈希 + 基准 trace 全部一致 |
| M2 ✅ | 主固件全执行面翻译（含未执行可达路径） | **已达成（2026-09-11）**：15999 PC（9217 执行集 + 6782 可达新增）全译、0 stub（13 处 `TODO(gt)`）；boot+demo200 两模式 trace + 状态哈希与 M1 基线一致 |
| M3 ✅ | 子 MCU 固件翻译（`smemit` 全译 rom_sm 4KB）+ SM/主 CPU 5× 时序对接 | **已达成（2026-09-11）**：4096 SM PC 全译；boot+demo200 两模式 `s` 行 SM trace 逐条一致、hashdump 与 M1 基线同哈希（含 `hash.sm`/`sm_ram`/`hash.lcd_state`）；执行集（45/251 PC）全落翻译区间 |
| M4（进行中） | voice/PCM 语义化改写（stock 28）：主链 `native_pool`/`native_allocfree`/`voice_materialize`/`mask_acc`/`pcm_enable`（1260 PC）已手写 C++；闭包剩余例程待翻译（见 `out/m4/18_closure_gap.md`，非可选） | 主链已达成（2026-09-12）：55s MIDI 整曲与 stock 逐字节一致（pcmdiff 0/7,282,760）、boot/demo trace+hash 与基线一致、试听通过；全量翻译进行中 |
| M5（暂缓） | 256 复音扩展（**目标 256 声**；`pcm_ext_*`/`-voices:`/0xE800/page6-7；原 M4c 容量优化）——独立于翻译目标的能力研究 | 已回滚（2026-09-11）；`-voices:255` 压力矩阵（n=32/64/128/255，255 = 0xff 哨兵妥协上限，非目标）随扩展暂缓；设计记录 `docs/07–11` |

详细设计见 `docs/00_plan.md`、`docs/01_architecture.md`、`docs/02_conventions.md`。
