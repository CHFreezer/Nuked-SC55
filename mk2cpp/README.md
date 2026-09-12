# mk2cpp — SC-55mk2 固件 C++ 语义级翻译工程

目标：把 **SC-55mk2 固件（rom1/rom2/rom_sm）**语义级翻译为 C++，以 **`mk2cpp.h`
库形式集成回 GT，替换原 H8 解释器行为**（开关控制、未翻译 PC 可回退），
设备/调度/音频全部复用 GT。当前聚焦**原版 28 复音固件的 C++ 翻译**：M1–M3 直译
（主 H8 + 子 MCU 全执行面）已达成，`-mk2cpp` 与默认解释器逐指令/逐状态等价。
**M4 = voice/PCM 语义化改写（stock 28），覆盖完成、语义化进行中**：主链路与
voice 闭包例程（池/分配释放、note 物化与描述符链、P0 分派簇、ts_scan/C9、interp、
0x4DE7 族、init/reset/d1ac、共享 helper、interp 间接目标、pool 续段、命令环
handler 等）已接入 **4239 个 PC 入口**，静态可达闭包 **closure−gen−hand = 0**
（2026-09-12）；**指定回归曲**（外部 MIDI，曲目见本机 local.md，覆盖绝大部分行为），扩展覆盖
（代理侧一次性，非默认）为全量唯一 MIDI 曲目（数量见 local.md）两模式逐字节 MATCH（见
`out/m4/41_full_regression.md`）；
剩 185 个闭包外动态 PC（事件环/part/主循环等非 voice 子系统）与死字节/竞态
不可达臂由 mixed 回退。**覆盖 ≠ 语义化**：当前 `src/hand/` 主体是
`switch (mcu.pc)` + `case 0x....` 的逐指令直译（32.4k 行 / 4,232 case，
可核对等价、不可当语义代码阅读），尚未达到本工程"人类可读语义 C++"的目标；
M4 收口还需：(1) 宿主指令边界回调（支撑整例程语义写法且不破坏中断/MIDI
时序）；(2) 语义重写（`pcm_misc.cpp` 13.9k 行垃圾桶已于 2026-09-12 拆分为
`pcm_irq_service`/`pcm_fraction_div`/`note_on_setup`/`pitch_env`/`voice_param`/
`ts_scan`/`cmd_ring` 七个 ROM 例程模块；其余模块命名审计与全量语义重写待做）；
(3) 正式验收 oracle（n=28 音频 null + 用户在场试听，未执行）。
快照审查确认 hand 无私有状态，无需 `MK2CPP_StateSave/Load`；
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
| M4（进行中） | voice/PCM 语义化改写（stock 28）：手写 C++ 覆盖 `native_pool`/`native_allocfree`/`voice_materialize`/`mask_acc`/`pcm_enable` + 闭包例程（`pcm_dispatch`/`note_path`/`note_chain_tail`/`pcm_interp`/`dsp_rate`/`reset_init` + 原 `pcm_misc` 拆分出的 7 模块 + 原 `shared_misc` 拆分出的 `dsp_rate_common`/`maint_fill_d1de`/`maint_merge_d1cd`/`maint_counter_d1d5`/`maint_voice_gate_d1ff`/`maint_table_walk_d1d6`/`maint_bit_scan_d1cc`/`shared_nop_rts` + `hand_prims.h`）共 4239 PC | 覆盖达成（2026-09-12）：closure−gen−hand=0、全量整曲逐字节一致（数量见 local.md）、boot/demo trace+hash 与基线一致；**语义化未完成**：hand 主体仍为逐指令 case 表（非可读语义 C++），模块需按 ROM 例程拆分重写；n=28 音频 null + 听感验收未执行 |
| M5（暂缓） | 256 复音扩展（**目标 256 声**；`pcm_ext_*`/`-voices:`/0xE800/page6-7；原 M4c 容量优化）——独立于翻译目标的能力研究 | 已回滚（2026-09-11）；`-voices:255` 压力矩阵（n=32/64/128/255，255 = 0xff 哨兵妥协上限，非目标）随扩展暂缓；设计记录 `docs/07–11` |

详细设计见 `docs/00_plan.md`、`docs/01_architecture.md`、`docs/02_conventions.md`。
