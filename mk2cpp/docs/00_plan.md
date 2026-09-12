# mk2cpp 计划（M1–M5）

状态：M1 ✅ M2 ✅ M3 ✅（2026-09-11，直译）；M4 语义化改写（stock 28）进行中
（2026-09-12 覆盖翻译完成并逐字节验证：4239 PC、closure−gen−hand=0、全量
曲目一致（数量见本机 local.md）；**语义化与模块化未完成**：hand 主体仍为逐指令 case 表）；M5（256
复音扩展）已回滚暂缓（2026-09-11）。里程碑口径：M4 = ROM 全量 C++ 翻译的
语义层收尾；M5 = 在翻译目标之外新增 256 复音能力的研究。
依据：`../tools/docs/`（证据协议、voice_memory_map、voice_bounds_inventory、
task_irq_map、polyphony_256_todo）。

## 0. 范围

- **纳入**：主 H8/532 固件（rom1/rom2 代码面）、子 MCU M37450 固件（rom_sm，4KB）。
- **不翻译**：采样/波形 ROM（数据引擎输入）、GT 设备模型（PCM/定时器/LCD/ADC 等，
  以宿主 API 复用，不重写行为）。
- **终态**：GT 的 `-mk2cpp` 模式（`mk2cpp.h` 库）用翻译后的 C++ 跑同一 ROM 场景，
  行为与默认解释器模式等价。**当前聚焦原版 28 复音固件的 C++ 翻译**：
  M1–M3 直译已达成；**M4** 对 voice/PCM 做语义化改写（stock 28）进行中
  （2026-09-12 覆盖完成；语义化重写与模块拆分待做）；**M5** 的 256 复音扩展是在此之上的独立能力研究，
  已于 2026-09-11 回滚暂缓（见 §2）。
  **不产出独立可执行程序。**

## 1. 方法与边界

翻译分两层，均以 GT 为 oracle：

1. **直译层（M1–M2，`src/gen/`）**：按基本块生成 C++，内存模型/标志/周期与 GT
   完全一致（`uint8_t mem`、页映射、`cycles += 12`、SR 位语义照抄
   `../src/mcu_opcodes.cpp`）。目标是 **0 分歧**，不是"好看"。
2. **语义层（M4，`src/hand/`）**：对 stock 28 的 voice 子系统与 PCM 编程做可读重写
   （原生结构体/API），并用 co-sim 保证可观测行为等价；256 数组/扩展 API 属 M5
   （已回滚暂缓）。

**混合运行**：未翻译的 PC 由 GT 解释器兜底（同一进程内），保证始终可运行、
覆盖率可逐步提升；到达 100% 执行集后再移除兜底。

## 2. 里程碑与验收

### M1 基础设施 — ✅ 达成（2026-09-11）
达成记录（命令与产物见 `tests/two_mode_check.py`、`out/twomode/`）：
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

### M2 主固件全执行面 — ✅ 达成（2026-09-11）
达成记录：
- `h8reach`：静态可达分析，15999 总 PC（9217 exec + 18 vec + 6764 static + 206 jump-table）；10 个跳转表站点、1015 条表项。
- `h8part --vec --static`：15999 指令 → 9463 块，223 overlap。
- `h8emit`：15999 函数生成，13 处 `TODO(gt)`，0 stub。
- 验证：boot 0–3M + demo 200–202M 两模式 trace/hash 全部 PASS（与 M1 基线一致）。
交付：执行集 100% 翻译 + 未执行但静态可达路径补齐（r16 扫描 + 向量表 + 跳转表展开）。
oracle：demo 200M 窗口 0 分歧；boot 3M 0 分歧。

### M3 子 MCU — ✅ 达成（2026-09-11）
达成记录：
- `smemit`（`tools/h8lift/smemit.c`）：rom_sm 4KB 全译，逐 SM PC 发射 C++（`src/gen/sm/`，
  本地不入 git）。opcode 模型（`sm_op_impl`/`sm_op_len` 表 + 各寻址式表达式）由
  `tools/h8lift/sm_parse.py` 从已入库来源生成：`impl` 取 GT `src/submcu.cpp` 的
  `SM_Opcode_Table`（GT 只实现 165/256），`len` 取 GT handler 路径的取指字节数，
  并逐条与 `tools/disasm/smdasm.c` 的完整 M37450 表交叉校验（165/165 一致），
  非手算；`sm_parse.py --check` 可复验 `smemit.c` 内嵌表。
- 地址模型：SM 固件执行于 `sm.pc ∈ [0xf000,0xffff]`（14 位形式），GT `SM_Read` 经
  `& 0x1fff` 选 ROM、`& 0xfff` 索引，故 smemit 全译该 4096 区间（`g_rom[pc&0xfff]` 仅用于
  解码；发射体调用 GT 运行时 helper，行为与运行期字节无关）。10 个向量目标（reset=0xf003）
  与 VM 校验的 251-PC 执行集全部落在该区间（Python 验证：区间外 0 个）。
- 集成：`mk2cpp.h` 增 `MK2CPP_SM_Register/CanStep/Step` + 统计；`mk2cpp.cpp` 增
  `smk2` 分派表（按 `sm.pc` 索引）；`src/submcu.cpp` `SM_Update` 取指处分派
  （`-mk2cpp` 且已翻译 → `MK2CPP_SM_Step`，否则 `SM_ExecuteOneInstruction` 回退），
  5× 时钟/`cycles+=48`/timer/UART 时序原样不变；CMake 用 `GLOB sm/` + `MK2CPP_HAS_SM_GEN`。
- 验证（`tests/two_mode_check.py`，同一 GT 二进制两模式）：
  - boot [0,3M)：trace 375,116 行（含 125,117 条 `s` 行 SM 指令）两模式逐条一致，
    hashdump SHA256 `91CE3BF8…`（= M1 基线），baseline `trace_boot3m_base.txt` 一致。
  - demo200 [200M,202M)：trace 375,001 行两模式一致，SHA256 `4B445776…`（= M1 基线），
    baseline `trace_200m_base.txt` 一致；hash 含 `hash.sm`/`sm_ram`/`sm_shared_ram`/
    `sm_device_mode`/`hash.lcd_state` 全同 → SM 状态与 LCD 帧序列一致；`sm.cycles≈202M×5`
    印证 5× 时钟。demo200 窗口执行 45 个唯一 SM PC（均在翻译集内）。
oracle：`s` 行 SM trace 与 GT 逐条一致（两模式 + 基线）；LCD 渲染与 GT 相同帧序列
（`hash.lcd_state` 一致）。**已达成。**

### M4 语义化改写（stock 28）— 覆盖完成，语义化进行中（2026-09-12）

**进度（2026-09-12）**：主链路与 voice 闭包例程已全部改写为手写 C++ 并逐指令接入，
累计约 4100 个 PC 入口：pool-init/alloc-free（`native_pool`/`native_allocfree`）、
note 物化链（`voice_materialize`）、mask_acc、PCM enable/disable（`pcm_enable`）、
P0 分派簇（`pcm_dispatch`：IRQ0/voice_search/pcm_dispatcher）、note 链（`note_path`/
`note_chain_tail`）、P1 杂项（原 `pcm_misc`，2026-09-12 已按 ROM 例程拆分为
`pcm_irq_service`/`pcm_fraction_div`/`note_on_setup`/`pitch_env`/`voice_param`/
`ts_scan`/`cmd_ring` 七个模块）、
interp（`pcm_interp`）、0x4DE7 族（`dsp_rate`）、init/reset+d1ac（`reset_init`）、
共享原语与碎片尾（原 `shared_misc`，2026-09-12 已按 ROM 例程拆分为
`dsp_rate_common`/`maint_fill_d1de`/`maint_merge_d1cd`/`maint_counter_d1d5`/
`maint_voice_gate_d1ff`/`maint_table_walk_d1d6`/`maint_bit_scan_d1cc`/
`shared_nop_rts` 八个模块 + 共享原语 `hand_prims.h`）。
验证：55 秒 MIDI 整曲与 stock 逐字节一致（`-midiseq`，pcmdiff 0/7,282,760）；
boot/demo 各窗口 trace+hash 与基线一致；多场景压力与休眠臂对拍一致；
用户实时试听通过（2026-09-11 早期构建；最终覆盖版听感验收见下）。

**覆盖与剩余（2026-09-12）**：静态可达闭包全部 hand 化（`closure−gen−hand = 0`，
hand 共 4239 PC）；闭包内 3 簇动态间接目标（interp `0x3A5A-0x3A9D`、pool 续段
`0x4062B-0x406E2`、命令环 handler `0x625-0x673`，共 99 PC）已补译；
**全量唯一 MIDI 曲目两模式逐字节 MATCH**（含两首指定整曲与超长曲尾部，见
`out/m4/29/31/32/37_*.md`；数量与曲目见本机 local.md）；剩 185 个闭包外动态 PC（14 簇：事件环 `a4d0/a7d0`、
part 复位/参数、主事件循环回边等非 voice 子系统）与死字节/竞态不可达臂由 mixed 回退，
既有 trace 两模式逐行一致（`out/m4/33_dynamic_class.md`）。快照审查
（`out/m4/27_snapshot_status.md`）确认 hand 层无私有仿真状态，GT `state_save/load`
已完整覆盖，无需 `MK2CPP_StateSave/Load`。

**语义化差距（2026-09-12 复核）**：覆盖翻译只保证行为等价，不代表达到本工程
"人类可读语义 C++"的目标。当前 `src/hand/` 共 32,413 行、4,232 个
`case 0x....`，主体是 `switch (mcu.pc)` 逐指令直译（十六进制 PC 控制流、
裸寄存器读写），与 `../src/` 原作者风格（命名操作/结构体/算法控制流）差距明显；
只有早期模块（`native_allocfree`/`mask_acc`/`pcm_enable`）有命名原语。
M4 收口三步：(1) 宿主指令边界回调（每步中断轮询/cycles/trace/MIDI 轮询的
可重入点，支撑整例程语义写法而不破坏时序）；(2) 语义重写（`pcm_misc.cpp`
13,950 行/1,130 case 垃圾桶已于 2026-09-12 拆分为 `pcm_irq_service`/
`pcm_fraction_div`/`note_on_setup`/`pitch_env`/`voice_param`/`ts_scan`/
`cmd_ring` 七个例程模块，拆分后指定整曲与拆分前逐字节一致；`shared_misc.cpp`
同日拆分为 `dsp_rate_common`/`maint_fill_d1de`/`maint_merge_d1cd`/
`maint_counter_d1d5`/`maint_voice_gate_d1ff`/`maint_table_walk_d1d6`/
`maint_bit_scan_d1cc`/`shared_nop_rts` 八个例程模块，共享 H8 原语提取为
`hand_prims.h`，指定整曲同样逐字节一致；其余模块做
命名审计与全量语义重写）；(3) 正式 oracle：`tests/m4_audio_null.py --execute --user-present
-CheckBaseline`（含用户在场听感，尚未执行）。

> **里程碑拆分（2026-09-11）**：原 M4 把「voice/PCM 语义化改写」与「256 复音扩展」
> 捆在一起。现拆分：**M4 = stock 28 的语义化改写，属 ROM 全量 C++ 翻译的收尾**；
> **M5 = 256 扩展，属翻译目标之外的独立能力研究**（已回滚暂缓）。`docs/07–11`
> 为混编记录：不依赖扩展的 stock-28 语义分析仍属 M4，涉及 N>28、256-bit mask、
> `0xE800`、page6 的扩展设计随 M5 暂缓。当前 `src/hand/pcm_enable.cpp` 的 6 个 L0
> 覆盖只翻译 stock 28 的 PCM enable/disable flush（不依赖任何 256 扩展），
> `native_pool.cpp` 固定用 `kLegacy=28`。

交付分层：
- **M4a 控制语义化**：`src/hand/` 把 voice/PCM 控制路径原生化（固件寄存器写 →
  typed API/原生状态），DSP 仍用 GT `PCM_Update`；n=28 音频 null 先通过。
- **M4b DSP 移植**（已作废）：DSP 本身就是 GT `src/pcm.cpp` 的 C++ 芯片模型，
  不是固件 ROM；"移植"只是早期 256/原生引擎路线下想把宿主模型搬进 hand，
  现无必要（2026-09-12 标注）。

切片顺序（详见 `09_m4_integration.md` §5.5；重启时按 stock 28 复核）：PCM enable/disable
flush → pool-init → alloc/free（**slice-2**）→ per-voice materialize/PCM 写路径 → …；
前期限定 L0（一次 `MK2CPP_Step` = 恰好一条 H8 指令），整例程 hook（L1）在 n=28 null
通过前不得启用。slice-2 规格见 `11_slice2_pool_spec.md`：pool-init 可被中断（boot 实测
2 次 FRT2）**必须 L0**；L1 首个用例只用全 IML=7 的 `mask_acc 0x1ad3`；命名勘误——
`0x1823/0x187e` 是 release 而非 alloc（分配器 = `pool_pop 0x19ad` + `link 0x194c`），
`a3a0 0x94` 是 free 标记、`a368` 是 slot→part。

依赖（Wave 0）：
- 集成骨架：hand 独立稀疏 override 表 + `MK2CPP_Configure/PostReset`（`09`）；
- 音频 tap：`-wav:`/`-audiowin`/`-audiohash` 与 `-snapinfo`（`08`/`10`）；
- 音频基线冻结（stock 300M–320M）（**待办，需用户**，`10` §6）；≥255 同时音 MIDI
  素材属 M5。

oracle：
- n=28：与 stock 音频输出 null 测试（容差 0 或书面量化误差）——**仍有效**；
- 与 `../tools/docs/task_irq_map.md` §4 的 O1–O10 验收矩阵一致（扩展相关项随 M5 暂缓）；
- 音频/听感窗口 ≥300M，状态 ≥200M；超时 = ceil(终点/24e6×2)+15s。

### M5 256 复音扩展（目标 256 声；现行妥协上限 255）— 暂缓（2026-09-11 回滚）

> **回滚说明（2026-09-11）**：256 复音扩展（`pcm_ext_*` 全局、`-voices:<n>`、
> 0xE800 扩展窗口、page6/7 backing、`PCM_MAX_VOICE`/`PCM_EFF_BASE`/`PCM_SLOTS`、
> 效果槽外置）已从 `src/` 回滚到原版 28 复音实现（对齐 `9c98ab9` 的 PCM 语义）。
> `docs/07–11` 描述的是**扩展模式的设计**，现为**暂缓记录**；重启须基于原版
> 28 复音重新评估范围。

口径：**目标 = 256 声同时发音**；`-voices:255` 是 `0xff` 哨兵 + 8-bit 池计数
妥协下的阶段性上限，**非最终目标**（真 256 所需变更见 `10` D1）。

交付：
- **M5a 容量扩展**：按 `docs/07–11` 落地 `pcm_ext_*`/`-voices:n`/0xE800 窗口/
  page6-7/256-bit mask 等设计。
- **M5b 容量 256 优化**：inactive voice 跳过/批处理（须证明与全跑等价）；阶段验收在
  妥协上限 `-voices:255` 实时达标，**最终目标 256 声**。

oracle（随扩展暂缓）：
- `-voices:255`（目标 256 声；255 = 妥协上限）：长跑稳定、无复位、PCM 激活、
  CPU 占空比合理；压力矩阵 n=32/64/128/255（`tests/m4_stress_voices.ps1`（已删除；M5 重启时以 Python 重写））；
- ≥255 同时音 MIDI 素材与注入通道（**待办，需用户**，`10` §6）。

设计文档：`07_m4_voice_spec.md`（voice 语义）、`08_m4_pcm_api.md`（PCM/音频）、
`09_m4_integration.md`（覆盖表/开关矩阵）、`10_m4_oracle.md`（验收 oracle）、
`11_slice2_pool_spec.md`（slice-2 pool-init/release/free 规格）。

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
