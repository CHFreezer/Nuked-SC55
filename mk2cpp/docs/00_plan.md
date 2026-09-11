# mk2cpp 计划（M1–M4）

状态：M1 设计定稿，开始实现。
依据：`../tools/docs/`（证据协议、voice_memory_map、voice_bounds_inventory、
task_irq_map、polyphony_256_todo）。

## 0. 范围

- **纳入**：主 H8/532 固件（rom1/rom2 代码面）、子 MCU M37450 固件（rom_sm，4KB）。
- **不翻译**：采样/波形 ROM（数据引擎输入）、GT 设备模型（PCM/定时器/LCD/ADC 等，
  以宿主 API 复用，不重写行为）。
- **终态**：GT 的 `-mk2cpp` 模式（`mk2cpp.h` 库）用翻译后的 C++ 跑同一 ROM 场景，
  行为与默认解释器模式等价；之后在 M4 对 voice/PCM 做语义化改写并支持 256。
  **不产出独立可执行程序。**

## 1. 方法与边界

翻译分两层，均以 GT 为 oracle：

1. **直译层（M1–M2，`src/gen/`）**：按基本块生成 C++，内存模型/标志/周期与 GT
   完全一致（`uint8_t mem`、页映射、`cycles += 12`、SR 位语义照抄
   `../src/mcu_opcodes.cpp`）。目标是 **0 分歧**，不是"好看"。
2. **语义层（M4，`src/hand/`）**：对 voice 子系统与 PCM 编程做可读重写
   （真正的结构体/256 数组/原生 API），并用 co-sim 保证可观测行为等价。

**混合运行**：未翻译的 PC 由 GT 解释器兜底（同一进程内），保证始终可运行、
覆盖率可逐步提升；到达 100% 执行集后再移除兜底。

## 2. 里程碑与验收

### M1 基础设施 — ✅ 达成（2026-09-11）
达成记录（命令与产物见 `tests/two_mode_check.ps1`、`out/twomode/`）：
- `h8dec`：`--check-decode` 9235 条 0 mismatch；`h8part`：9217 指令 → 2576 块（与 §06 一致），确定性哈希稳定；`h8emit`：9217 函数、0 stub、7 处 `TODO(gt)`（GT FIXME 尾巴），切片/全量 clang-cl 编译通过。
- 集成：`-mk2cpp` + `mk2cpp/src/gen`（本地）链接；启动打印 `9217 translated PCs registered`。
- 验证：boot 0–3M（trace 375,116 行 + hashdump SHA256 91CE3BF8…）与 demo 200–202M（trace 375,001 行 + hash 4B445776…）两模式与基线全部一致；`hashdump` 本身三种运行（h1/h2/h3）同哈希。

### M1 基础设施（原始设计）
交付：
- `mk2cpp/mk2cpp.h` + `mk2cpp.cpp`：GT 集成 ABI（`MK2CPP_Init/CanStep/Step`）、
  `-mk2cpp` 开关、未翻译 PC 回退解释器、fallback 统计。
- `tools/h8lift`：输入 rom1/rom2 + PC 集 + flow 边，输出 GT 可直接编译的
  C++ 翻译实现 + 映射表（`src/gen/`，本地不入 git）。
- `tools/tracediff`：比较**同一 GT 二进制的两模式**（默认 vs `-mk2cpp`）的 trace/
  快照/哈希，逐指令报告首个分歧及最近历史（取代独立 cosim 进程方案）。
- 覆盖率仪表盘：执行 PC 集（`../tools/baselines/pc_main.txt`）+ 静态可达集，
  统计已翻译/兜底/缺失。
oracle：
- 从 reset 跑 3M 指令（对照 `../tools/baselines/trace_boot3m_base.txt`）两模式 0 分歧；
- 从快照跑 200M–202M 窗口 0 分歧（对照 `trace_200m_base.txt`）。

### M2 主固件全执行面
交付：执行集 100% 翻译 + 未执行但静态可达路径补齐（引用 r16 静态扫描清单）。
oracle：demo/mocknote/长跑（≥300M 周期）0 分歧；复位后任意窗口哈希一致。

### M3 子 MCU
交付：`src/gen/sm/`（rom_sm 4KB 全译）+ SM 与主 CPU 的时序对接（5× 时钟）。
oracle：`s` 行 SM trace 与 GT 逐条一致；LCD 渲染与 GT 相同帧序列。

### M4 语义化 + 256
交付：`src/hand/` 中 voice/PCM 原生实现；`-voices:<n>` 走新引擎。
oracle：
- n=28：与 stock 音频输出 null 测试（容差 0 或明确量化误差）；
- n=32/64/128/256：长跑稳定、无复位、PCM 激活、CPU 占空比合理；
- 与 `../tools/docs/task_irq_map.md` §4 的 O1–O10 验收矩阵一致（256 目标）。

## 3. 风险与对策

| 风险 | 对策 |
|---|---|
| 数据/代码边界（跳转表嵌在代码区） | 以执行集为权威；静态可达用 r16 扫描 + 人工抽检；兜底解释器 |
| 间接跳转（`jmp @rN`） | 分发表由运行时目标集合动态建立（首次遇到某目标即登记），避免静态枚举错误 |
| 中断/异常时序 | 与 GT 同点调度：每步先 `handle_interrupts()` 再执行块；周期记账一致 |
| 固定点 DSP 语义 | M1–M3 直译保真；M4 改写时保留量化步骤并用音频对照 |
| 生成物版权 | `src/gen/` 不入 git（ROM 派生）；只提交工具/运行时/文档 |
| 范围蔓延 | 里程碑验收不通过不得进下一阶段；阻塞即补研究文档 |

## 4. 立即可执行的第一步（M1）

1. `tools/h8lift` v0：吃 `pc_main.txt` + `flow_main.txt` + rom 字节，线性切分基本块
   （用 `../tools/disasm/h8dasm.c` 的 decoder 语义），输出：
   - `src/gen/blocks_rom1.cpp` / `blocks_rom2.cpp`：`case pc: ... goto next;`
   - `src/gen/map.csv`：pc → 块函数名/长度
2. `mk2cpp.h` v0：在 GT `work_thread` 的取指处加开关（默认关闭时零改动）：
   `if (mk2cpp_enabled && MK2CPP_CanStep(flat)) MK2CPP_Step(); else MCU_ReadInstruction();`
   翻译实现直接调用 GT 现有 `MCU_*`/设备函数（无独立运行时）。
3. `tools/tracediff` v0：两模式各跑一段并对比（先 boot 3M，再快照窗口），
   首个分歧输出最近 64 步历史。
4. 用 boot 3M 指令集做第一轮 0 分歧验证；再扩到快照窗口。
