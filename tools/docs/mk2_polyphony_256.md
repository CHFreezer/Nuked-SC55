# Roland SC-55 mkII 复音数 28 → 256 升级方案

**修改对象（GT / 模拟器）**：Nuked-SC55 `src/`（`pcm.cpp`/`pcm.h`/`mcu.cpp`）——H8/532 固件的执行环境。
**固件（被 patch 目标）**：`rom1.bin`（32 KB）、`rom2.bin`（512 KB，mkII）——Roland SC-55 mkII 主 MCU 固件（SC = Sound Canvas）。
**目标**：复音数从 **28 → 256**，经 CLI flag 开启；**ROM 加载进内存后、仿真开始前 patch 完毕（内存副本）**，不动磁盘 `.bin`。
**日期 / 工具**：2026-09-09 ｜ GT `-pcmtrace` 动态追踪（本次新增，`src/mcu.cpp`）、`h8dasm` 反汇编、raw ROM 字节验证（`build/rom2.bin`）。
**姊妹篇**：`polyphony_256_todo.md`（**工程待办清单**）、`evidence_protocol.md`（**证据协议：GT/trace/dasm/VM 分级与矛盾处理规则，分析/验证前必读**）、`../baselines/dasm_annotated.txt`（固件函数/区域地图）、`scgs_pipeline.md`（PCM 管线）。

---

## 0. 结论速览（先答五个问题）

**Q：为什么不能只改 ROM 上某个常量？**
→ 「28」**不是单一常量**，它同时编码在**码**（per-voice 循环界、28 通道建立、note 表填充）和**数据**（SRAM note 表 28 项）里。且 per-voice 循环界 `cmp #0x1c`（28）是 **8-bit 立即数**，256 = `0x100` 塞不下——**光雕 A（ROM 副本）里的常量改不动结构**。

**Q：方案？**
→ **A/B 内存块**：**A** = ROM 副本（GT 里本就已加载为 `rom1[]`/`rom2[]` 全局数组）；**B** = 追加在 A 空闲区的**自包含扩展块**（256 版 per-voice 循环【码】+ 扩大的 note 表【数据】，用寄存器计数/16-bit 界）；**补丁 = A 的指针/调用点重定向到 B**（只改少数几处，A 本体几乎不雕）；**加载 A+B**，磁盘 ROM 原封不动。

**Q：模拟器（GT）侧怎么改？**
→ 源码在手：`#define MAX_VOICE 256` 兜底 → `ram1/ram2/fstate` 数组、`voice_mask`（256-bit）、`reg_slots`/`select_channel` 位宽**都从它派生**，不出现散落的魔法 256。GT 恒有 256 容量，实际激活数仍由固件驱动。

**Q：精确落点（全部验证过）？**
→ **config 写点 `0x04134c`**（`MOVG3` 写 `r0=0x7bc3`→reg 3c/3d，值来自**数据表 rom2 fileoff `0x1432`**；h8dasm 把 MOVG3 方向解反成读，pcmtrace 实证是写）、28 通道建立 `0x04135c`、per-voice 循环界 `0x45cde`（`cmp #0x1c`，**唯一** per-voice 循环，**4 个调用者** `0x344/0x7b6a/0x7d8a/0x461ac`）、per-voice 状态 SRAM `0xa368…`（**核心 12 + 次级，全表 §6.1**，各 28B）。见 §5/§6。

**Q：挂点现成吗？**
→ **是**。`MCU_PatchROM()`（`mcu.cpp:1482`）已存在、在 ROM 加载后（`mcu.cpp:2228`）被调用，里面已有注释掉的 in-memory patch（含 `rom2[0x1333]`）——**正是为 A/B 方案预留的挂钩**。

**Q：当前 28 的峰值需求？**
→ demo+mocknote 20s 实测峰值激活 **20**（是需求，不是上限）。上限 28 由设计决定。

---

## 1. 复音数如何工作（H8 固件 → PCM）

```
H8 固件（rom1/rom2）
  ├─ per-voice 数组（SRAM 0xa368+，28 slot × 并行数组，核心 12 + 次级，全表 §6.1，1 字节/slot，0xff=空）：跟踪活跃音符
  ├─ 分配音符 → PCM slot 0..27   （select_channel 0xe03e，&0x1f；写点 rom1 0x5xxx）
  ├─ 写 voice_enable 0xe000-03   （28-bit 激活掩码，voice 0..27；写点 0x5525/0x5662）
  ├─ 写 config_reg_3d 0xe03d     （reg_slots = (val&31)+1；boot `0x04134c` 写 0x7b → 28，值源数据表 0x1432）
  └─ per-voice 循环（rom2 0x45cc0-0x45ce1，28 次）
         ★ 实为**按 part 求 max 扫描**（非音频合成）：r4=part id(@0x4692)，
           扫 28 个 voice，对 @r0+0xa368==r4 者取 @r0+0xad0e 的 max，再定点缩放。
              │
              ▼
PCM（src/pcm.cpp，PCM_Update）
  └─ for slot in 0..reg_slots-1：key = voice_mask[slot]；跑 DSP；累加 → 立体声输出
```

**四处共同封顶 28**（本轮实证，全链路一致，见 §2/§6）：
1. 固件 per-voice 数组 **28 slot**（核心 12 + 次级，§6.1）+ per-voice 循环 **28 次**（唯一，`0x45cde`）。
2. `voice_enable` **28 bit**（0x00 仅 4 bit→voice 24-27，0x01/02/03 各 8 bit）。
3. C 侧 `reg_slots = (config_reg_3d&31)+1 = (0x7b&31)+1 = 28`（`pcm.cpp:536`）。
4. （硬件上限，未用到）PCM 32 slot / `select_channel` &0x1f / `config_reg_3d` &31 → 理论 32。

---

## 2. 当前「28」的全部落点（已逐条验证）

### 2.1 GT（src/）侧

| 位置 | 限制 | 说明 |
|---|---|---|
| `pcm.cpp:74-96` voice_enable 写 | **28 bit** | 0x00→bit24-27(4)、0x01→16-23、0x02→8-15、0x03→0-7 |
| `pcm.cpp:536` `reg_slots=(config_reg_3d&31)+1` | 最多 **32** | DSP 循环 `pcm.cpp:1111` 上界 |
| `pcm.h:25-26` `ram1[32][8]` / `ram2[32][16]` | **32 slot** | per-slot DSP 状态 |
| `pcm.h:52` `fstate[32][2]` | 32 slot | `-float` 滤波器状态 |
| `pcm.cpp:126` `select_channel=data&0x1f` | 最多 32 | 通道选择 |

### 2.2 固件（ROM）侧 —— 动态追踪 + raw 字节验证

| 项 | 值 / 地址 | 来源 | 验证 |
|---|---|---|---|
| `config_reg_3c/3d` | **0xc3 / 0x7b**（`0x7bc3`） | **写点 `0x04134c`** `0d 3c 90`（`MOVG3`，写 `r0=0x7bc3`）；`r0` 源 = 数据表 **rom2 fileoff `0x1432`**（`MOVG2 @r1+0x1432` 读 + `OR r1 r0`） | ✅ pcmtrace `reg=3c/3d val=c3/7b pc=04:134f cyc=972` + raw `0x1432`=`c3 7b` |
| reg_slots | **28**（`pcm.cpp:536` `(0x7b&31)+1=28`） | — | ✅ 公式 + 快照值 |
| voice_enable 写 | 4 字节（0x00-03），实 28 位 | `0x5525/0x5527`、`0x5662/0x5664`（各 `movsw` 2 字节） | ✅ ROM 字节 |
| select_channel 写 | `movs rX @(br,$3e)`，per-frame 簇 | rom1 `0x524a/0x54d3/0x5e3e/0x5f23/0x5f49…`；init `0x4135c/0x41362` | ✅ ROM 字节 |
| per-voice 循环 | **唯一**，28 次，界 `cmp #0x1c` | rom2 **`0x45cde`**；循环 `0x45cc0-0x45ce2`，体读 `@r0+0xa368`/`@r0+0xad0e`，`bne -32` 回 `0x45cc2` | ✅ ROM 字节 + 热点 98K |
| note 表 / per-voice 状态 | **28 slot × 并行数组**，每 slot 1 字节，`0xff`=空 | **核心 12 组** `0xa368…0xad0e`（间隔 0x1c）+ **次级 14 字节/16 字** `0xcdc6…0xd1ac`（全表见 **§6.1**）；`sram[0x2368]+` | ✅ SRAM dump + 码（§6.1 三方交叉） |
| br 页性质 | **PCM DSP 控制接口**（非 per-voice 结构体） | `br+0x00-03`=voice_enable、`0x04-37`=per-channel ram1/ram2、`0x3c/3d`=DSP config、`0x3e`=select_channel+IRQ；`br+0x3c` **只写不读**（全 dasm 无 `movl/movs br,$3c`） | §6.1 + dasm |
| SRAM 计数器 | `(dp,0xa42c)`=28、`(dp,0xa42e)`=27 | free-slot 计数 / 最大 slot 索引 | ✅ 快照 |

> **关键修正**（本轮追踪实证）：`config_reg_3d=0x7b → reg_slots = 28`（**不是 32**）——`pcm.cpp:536` `(0x7b&31)+1=28`。故全链路（note 表 28 / 循环 28 / voice_enable 28-bit / reg_slots 28）**都是 28**；"32" 仅硬件上限（`&31`/`&0x1f`），从未用到。`0x04134c` 是 **写**（`MOVG3` 写 `r0=0x7bc3`→config；h8dasm 的 MOVG/MOVG2/MOVG3 族恒按 `<mem> <reg>` 读式打印，方向对 MOVG3 解反，pcmtrace 实证是写）；config 值源 = 数据表 **rom2 fileoff `0x1432`**。`0x46699` 的 `cmp #0x1c` 是**返回值阈值**（`jsr 0xbb36` 后比较），非循环——per-voice 循环**唯一**是 `0x45cc0`（4 调用者）。

---

## 3. 设计：A/B 内存块方案

```
A（已在内存）                B（新，追加于 A 空闲区）
┌────────────────────┐     ┌──────────────────────────────┐
│ rom1[]  (32 KB)    │     │ · 256 版 per-voice 循环（码） │
│ rom2[]  (512 KB)   │ ◄──► │   用寄存器计数/16-bit 界       │
│  · 原 28 通道建立    │ A→B │ · 256 版 note 表（数据）      │
│  · 原 per-voice 循环 │重定向│   （256 项，右对齐/对齐好）   │
│  · note 表基址      │     │                              │
└────────────────────┘     └──────────────────────────────┘
        加载 A+B 进 GT（内存），磁盘 rom1.bin/rom2.bin 不动
```

- **A 几乎不雕**：只把「note 表基址 / per-voice 循环调用 / config」等**少数指针重定向到 B**。
- **B 是为 256 重新设计的干净块**：不受 8-bit 立即数限制（循环界用寄存器或 16-bit）。
- **挂点现成**：`MCU_PatchROM()`（`mcu.cpp:1482`，ROM 加载后 `mcu.cpp:2228` 调用）填入 A→B 重定向逻辑。

**为什么比「在 A 上改常量」优雅**：
1. A 本体保持原样（可审计、可回退）。
2. B 右尺寸、右对齐，为 256 定制；每处 A→B 重定向是已知字节/已知地址。
3. 磁盘 ROM 永不被写（`rom1[]`/`rom2[]` 本启动就是一次性加载的内存副本）。

---

## 4. 详细实现

### 4.1 GT（src/）侧 —— 源码在手

1. **`#define MAX_VOICE 256`**（`pcm.h` 或 `mcu.h`），派生：
   - `ram1[MAX_VOICE][8]`、`ram2[MAX_VOICE][16]`、`fstate[MAX_VOICE][2]`。
   - `voice_mask` → `uint32_t voice_mask[MAX_VOICE/32]`（256-bit），voice_enable 寄存器扩展（新增 0x04-… 段或专用多字节块，GT 自定义接口）。
   - `reg_slots` 上限 = `MAX_VOICE`（`pcm.cpp:536` 去 `&31` 或 `& (MAX_VOICE-1)`）。
   - `select_channel = data & (MAX_VOICE-1)`（`pcm.cpp:126` 去 `&0x1f`）。
2. **加载 A+B**：`MCU_PatchROM()` 里应用 B（写 B 码/数据到 `rom2[]` 空闲区）并做 A→B 重定向；`rom2_mask` 需覆盖 B 地址范围。
3. **默认不变**：无 flag → 不建 B / 不重定向 → 28 复音、原行为。

### 4.2 A/B 补丁（固件，`MCU_PatchROM` 内）

- **B 内容**：
  - **码**：256 版 per-voice 循环（从 rom2 `0x45cc0-0x45ce1` 拷贝，界改寄存器/16-bit；note 表基址指向 B 的表）。
  - **数据**：256 项 per-voice 数组（核心 12 + 次级 14 字节/16 字，**全表 §6.1**），间隔 0x100。
- **A→B 重定向**（每处 1-3 字节）：
  1. per-frame DSP 对 per-voice 循环的**调用点** → B 循环入口。
  2. note 表**填充代码**的基址（0xa368/0xad0e）→ B 表基址（若表在 B）。
  3. `config_reg_3d` 写（`0x04134c`，r0 值）→ 目标 n（255）。
  4. 28 通道建立（`0x04135c` 循环）→ 扩到 n（或重定向到 B 的建立例程）。
- **note 表落点**（二选一，§6 决定）：
  - (a) **SRAM 原位扩容**（0xa368 起 256 项）：若 SRAM 0xa368 后有连续空间，基址不变、只改循环界/填充上限。
  - (b) **整块挪进 B**（rom2 空闲区）：A 与 B 的引用都指向 B 表。

### 4.3 CLI flag

```
-voices:<n>     目标复音数（默认 28，上限 MAX_VOICE=256）
```
- 设目标 n；当 `n != 28` 时在 `MCU_PatchROM` 建 B 并做 A→B 重定向、把 config/通道/循环界/note 表规模设为 n。
- 解析位置：`main()` 参数循环（`mcu.cpp:1760` 起，紧随现有 `-mk2/-float/-mocknote/…`）。

---

## 5. 精确 patch 点（验证过的锚点）

| # | 地址（rom2 flat / 类型） | 字节 | 改什么 | 来源 |
|---|---|---|---|---|
| 1 | **config 写点 `0x04134c`**（`MOVG3` 写 r0；值源**数据表 rom2 fileoff `0x1432`**） | `0x1432`:`c3 7b`→`c3 ff`（0x7bc3→0xffc3） | config_reg_3d=0xff → reg_slots=n（C 侧 `&31` 需同步放宽，见 §4.1） | pcmtrace `cyc=972` + raw `0x1432` |
| 2 | `0x04135c` | `58 00 1b` | `movi r0 #0x1b`（28 通道）→ n 版（或跳 B） | 动态 |
| 3 | `0x041359/0x041362` | `05 3e 81`/`70 3e` | select_channel 建立/写 → 扩/跳 B | 动态 |
| 4 | `0x45cde`（唯一 per-voice 循环） | `40 1c` | 循环界 28 → 跳 B 循环（推荐，避 8-bit 界） | 动态 |
| 5 | `0x5525/0x5527/0x5662/0x5664`（voice_enable 写） | `movsw`×4 | 28-bit 掩码 → 256-bit（GT 新寄存器配套，§8） | 动态 |
| 6 | per-voice 数组（SRAM 核心 12 `0xa368–0xacf2` + `0xad0e` + 次级 `0xcdc6–0xd1ac`，**全表 §6.1**） | — | 28 项 → 256 项（**挪 rom2 B**，见 §6.2）；各 `@r+disp16` 基址改指 B（次级 `0xcdc6–0xd1ac` 亦需同步） | SRAM dump + 码 |
| 7 | per-voice 循环 **4 调用者**：`0x00000344`/`0x00007b6a`/`0x00007d8a`/`0x000461ac`（`0x7c3f jmp r1` 间接分派到其一）；note 填充 `rom1 0x1800-0x1a40` | — | **patch 落点 = 循环本体 `0x45cc0`**（界 `0x45cde` → 跳 B 256 循环或就地改界），4 调用者免改 | dasm 调用边 |

> ROM 偏移换算：flat `0x0004134c` = rom2 文件偏移 `0x134c`（flat = `0x40000` + 偏移）。

---

## 6. 侦查结果（本轮 GT + h8dasm 实证，2026-09-09 二）

1. **rom2 空闲区** ✅：`rom2.bin`=512 KB。已执行 PC 最高 flat `0x04bb7d`（fileoff `0xbb7d`）。大段 0xff 填充空闲区：fileoff **`0x5f446-0x70000`（68 KB）**、`0x72580-0x7fffe`（55 KB）、`0x49fd8-0x50000`（24 KB）、`0x3206a-0x37e00`（24 KB）、`0xc9b2-0xfff0`（14 KB）。**B 建议放 fileoff `0x60000+`**（68KB 区内，对齐好、远离已用代码）。
2. **SRAM 原位扩** ✅（结论：**不现实**）：note 表基址 `0xa368` 之后**无** ≥1536B 连续空（`0xa3f4-0xad0d` 有 710B 数据；核心数组间隔 0x1c 会重叠）。SRAM 唯一 ≥1536B 连续区是 **4KB（BR `0x9060-0xa040` / `sram[0x1060-0x2040]`）**，但被**系数表密集引用**（`ADD @r3+0x9xxx`，`0x9000-0x9740` 间隔 0x20 + `0x9f50`/`0xa040-0xa090`）→ **note 表宜挪 rom2 B**（与 A/B 设计一致）。
3. **A 调用点** ✅：per-voice 循环（`0x45cc0`）**4 直接调用者** = `0x00000344`、`0x00007b6a`、`0x00007d8a`、`0x000461ac`（dasm 调用边）。其中 `0x00000344` 是 `0x7c3f: jmp r1`（per-frame DSP **间接跳表**分派，2011+ 目标）的目标之一，再由它调 per-voice 循环——故 `0x7c3f` 是**间接路径**。note 表填充 = **`rom1 0x1800-0x1a40`**。**patch 落点 = 循环本体 `0x45cc0`**（界 `0x45cde`），4 调用者免改。
4. **note 表项宽** ✅（**1 字节/slot**）：SRAM dump 实证核心数组各 28 字节（**核心 12 + 次级，全表 §6.1**），`0xa3d8`=`ff,0,1,…,1a`（slot0=0xff 空标记，slot i=i-1）→ 每 slot 1 字节、共 28 slot。per-voice 循环 `MOVG2 @r0+0xa368`（r0 步进 +1）读 2 字节 = 当前 slot + 下一 slot，逻辑项仍 1 字节。
5. **28 vs 32** ✅（**实际 28**）：config_reg_3d=0x7b → reg_slots=28（`pcm.cpp:536`）。note 表 28 / 循环 28 / voice_enable 28-bit / reg_slots 28 全一致；32 仅硬件上限（`select_channel &0x1f`、config `&31`），未用到。

**本轮补齐 / 仍缺**（2026-09-09 三，pcmtrace + raw 字节实证）：
- **config 写点** ✅（**已钉死**）：`0x04134c`（`MOVG3` 写 `r0=0x7bc3`→reg 3c/3d，`cyc=972` 一次性）。`r0` 值源 = 数据表 **rom2 fileoff `0x1432`**（`c3 7b`，boot config 表首项；经 `MOVG2 @r1+0x1432 r1` 读 + `OR r1 r0`）。**patch = 改 `0x1432`:`c3 7b`→`c3 ff`**（→reg_slots=256，C 侧 `&31` 同步放宽）。附：`0x04134f/0x041354` 是 voice_enable **init 写**（r7=0→reg 00-03），`0x041362` 是 select_channel 建立循环（27→0，28 通道）。
- **per-voice 循环调用者** ✅（**4 个**）：`0x45cc0` 被 `0x00000344`、`0x00007b6a`、`0x00007d8a`、`0x000461ac` 调用（dasm 调用边）。`0x7c3f: jmp r1`（per-frame DSP 间接分派，2011+ 目标）分派到 `0x00000344`（其一），再由它调 per-voice 循环——故 `0x7c3f` 是**间接路径**，非直调。**patch 落点 = 改循环本体 `0x45cc0`**（界 `0x45cde`），4 调用者无需逐个改。
- **voice_enable 256-bit 设计**（**提案**，§8 配套）：固件现写 reg 0x00-0x03（4 字节，实 28 位）。扩 256 位 = **8 字节（reg 0x00-0x07）**。方案：GT `pcm_t` 扩 `voice_enable[8]`（0x04-0x07 新增），`voice_mask` 改 256-bit；固件侧 `0x5525/0x5662` 的 28-bit 写扩到 256-bit（或 GT 侧把 0x00-0x03 的 4 字节重解释为 256-bit 低 32 位 + 新增 0x04-0x07 承接高 224 位）。**待 VM 侧确认** PCM 读 `voice_mask` 的位序。
- **per-voice 循环 ADDQ 语义** ✅（**VM 钉死**）：循环递增 `0x45cdc` 字节 `a8 08`，h8dasm 标 `ADDQ r0 r0`（易误读成翻倍），VM `MCU_Opcode_ADDQ` 实证 `a8`=寻址（direct r0，word）、`08`=opcode（`>>3`=1=ADDQ，`&7`=0→**+1**）→ **`ADDQ #1 r0`（r0+=1，递增）**。故 r0 = 0..27，`cmp r0,b #0x1c`（28）为**真循环上限**，**循环跑满 28 次**（非翻倍）。消掉"翻倍则循环到不了 28"疑虑。

### 6.1 per-voice 内存布局 & 「28」落点全表（2026-09-10 sub-agent 独立调查：dasm + pcm_trace + GT 源码三方交叉）

**布局结论**：per-voice 状态**不是** br 页结构体，而是 **SRAM 里一组以 slot(0..27) 为下标的 28 元素并行数组**（搜索循环用 r0、分配/释放/填充用 r1 作 slot）。br 页（BR=0xe0，`0xe000–0xe03f`）是 **PCM DSP 控制寄存器接口**：`br+0x00-03`=voice_enable、`br+0x04-37`=per-channel ram1/ram2 参数、`br+0x3c/3d`=DSP config、`br+0x3e`=select_channel+IRQ。**「28」由三处共同定**：`config_reg_3d=0x7b→reg_slots=28`（`pcm.cpp:536`）＋ 循环界 `0x045cde` ＋ SRAM 计数器 `(dp,0xa42c)=28/0xa42e=27`。

**核心 per-voice 数组（码 + 快照实证，置信高）：**

| 基址 | 宽 | 含义（证据） |
|---|---|---|
| `0xa368` | 28B | slot→index A；搜索循环 `0x045cc2` 读 + `CMP r4` |
| `0xa384` | 28B | slot→index B；搜索循环读 |
| `0xa3a0` | 28B | slot 分配标志；`0x94`=已分配（`0x001867`/`0x001a17`），free 时 CLR |
| `0xa3bc` | 28B | slot 数据；alloc（`0x00183d`）/free 时 CLR |
| `0xa3d8` | 28B | slot 链/prev 指针（快照 `ff,0,1,…,1a` 恒等） |
| `0xa3f4` | 28B | slot "prev" 指针；`0xff`=空 |
| `0xa410` | 28B | slot "next" 指针；`0xff`=空 |
| `0xa4aa` | 28B+ | slot→sequence；`a4aa[N]=0xff` 哨兵（`0x001928`） |
| `0xa4b4` | 28B | slot 激活标志；OR 进 voice mask（`0x001ad3`），alloc/free CLR |
| `0xa46c` | 28B | per-voice 字段（填充循环 `0x000fc6`） |
| `0xacf2` | 28B | 累积 voice mask（`0x001ad3` 建立） |
| `0xad0e` | 28B | slot→PCM 通道；`0xff`=空；搜索循环 `0x045cca` 读 |

**次级 slot 字段**（索引/生命周期实证，语义按 alloc/free/fill 推断，置信中）：字节 `0xcdc6/0xcdfe/0xce3f/0xce5c/0xce78/0xce94/0xceb0/0xcecc/0xcee8/0xcf04/0xcf20/0xcf3c/0xcf58/0xcf90`；字 `0xd000/0xd038/0xd054/0xd08c/0xd0a8/0xd0c4/0xd0e0/0xd0fc/0xd118/0xd134/0xd15c/0xd1a6/0xd1ac`（`d0e0[slot]=4` 于分配时写 `0x00182e`；`0xd0a8…0xd134` 组 0x1c 等距）。

**非 per-voice 的下标空间（勿混）：** `0x9060–0x9740`（r3 下标、0x20 步长、56 项，系数表）、`0x64xx–0x7bxx`（参数 bank）、`0x5d00–0x5e58`（r0、0x20 步长）、`0xa040–0xa2f8`（r2 16-bit per-note 事件描述符 `0xa210/0xa230/0xa250/0xa2c0/0xa2dc`）。

**消歧（两个「0x1800」）：** note 填充例程是 `rom1 0x1800-0x1a40` 的**代码**（156 PC，§9.2）；而 `0x004579` 的 `cmp r2,w #0x1800` 是某数值例程里的**定点算术 clamp 立即数**，与 note 表无关——二者地址形同、语义不同，勿混。

**SRAM 全局：** `(dp,0xa42c)`=free-slot 计数=`0x1c`(28)、`(dp,0xa42e)`=最大 slot 索引=`0x1b`(27)、`(dp,0xa42d)`=分配事件计数。

**「28」/`0x1c` 全部落点分类（6 处，全量）：**

| 地址 | 指令 | 类别 |
|---|---|---|
| `0x045cde` | `cmp r0,b #0x1c` | **(a) 真·28 复音循环界**（搜索循环 `0x045cc0–0x045ce0`） |
| `0x005e20` | `move r0 #0x1c` | (b) 通道索引 28（效果通道 28/29/30 选择，经 `(br,$3e)`） |
| `0x005ee7` | `move r1 #0x1c` | (b) 通道索引 28（保存效果通道 ram2 态到 SRAM） |
| `0x046656` | `MOVG #0x1c -> (dp,0xfffe)` | (c) 存常量 28 到 SRAM，init 序列，非循环 |
| `0x046699` | `cmp r0,b #0x1c` | (c) 阈值（`jsr 0xbb36` 返回值范围检，r0≥28） |
| `0x04669d` | `cmp r0,b #0x24` | (c) 阈值（同上第二段，r0≥36） |

→ **真·28 复音循环只有 `0x045cde` 一处**。通道 **28–31（`0x1c–0x1f`）是固定效果通道**（reverb/chorus/mix/LFO，见 pcm.h），与 28 个复音 slot 0–27 不同——扩复音若占用 28–31 会让渡对应效果。

**voice 分配 / 释放（置信高）：**
- 分配 `0x001823`：`d0e0[slot]=4`、CLR `ad0e/a3bc/a4b4`、更新 `a3d8` 链、`a3a0[slot]=0x94`、`(dp,0xa42d)++`、`bsr 0x1b44/0x1bad`。
- 释放 `0x0019c4`：`BTSTI a3a0[slot] bit7`（置位早退）、`SUB ad0e[slot] #0xff`（原 0xff 早退）、CLR `a3bc/a4b4`、解链 `a3f4/a410/a3d8`、`a3a0[slot]=0x94`、`(dp,0xa42d)++`。
- note→slot/通道选择：循环 ~`0x0017a9–0x0017e6`（**部分**跟踪，置信中）。

**voice_enable 维护（32-bit，`br+0x00–0x03`，置信高）：** SRAM 里维护为四 word —— `(dp,0xd150/0xd152)`=当前 mask、`(dp,0xd154/0xd156)`=待发 set/clear。两处写 PCM：停用路径 `0x005503–0x005527`（`r3=d150&~d154`、`r4=d152&~d156`→`movsw @(br,$00/$02)`）；启用路径 `0x00564a–0x005664`（`r5=d150|d154`、`r6=d152|d156`）。per-slot `a4b4` 由 `0x001ad3` 的 28 次循环 OR 进累积 mask `acf2`。位图（pcm.cpp）：`br+0x00`→voice 24-27(4b)、`0x01`→16-23、`0x02`→8-15、`0x03`→0-7。

---

## 7. 验证计划

1. **PCM 写追踪**（`-pcmtrace` 已补回，见附）：patch 后跑 `-voices:256 -demo -mocknote -pcmtrace`，确认固件写 `config_reg_3d`→目标 n、`select_channel` 用到 0..(n-1)、voice_enable 覆盖 n bit。
2. **快照校验**：dump `demo_postW.bin` 的 `config_reg_3d`、SRAM note 表（256 项）、`sram` 占用，确认 B 表就位。
3. **音频回归**：
   - 默认（无 flag）：28 复音，输出与原一致（用 VM vs GT diff 确认无回归）。
   - `-voices:256`：播放 ≥256 同时音符（构造 MIDI），确认不丢音、爆音可控。
4. **VM↔GT diff**：`tools/vm`（H8 验证器）跑同输入，比对 PCM 输出，确认 A/B 补丁未破坏 28 复音路径。

---

## 8. 风险

| 风险 | 说明 | 缓解 |
|---|---|---|
| B 循环须是原循环的忠实副本 | 从 `0x45cc0-0x45ce1` 拷贝时易偏移/漏指令 | 逐字节核对；用 h8dasm + 动态 trace 锚定 |
| note 表项宽/布局未定 | 影响 B 表大小与 SRAM 用量 | §6.4 先钉死 |
| `rom2_mask` 访问掩码 | B 地址须在掩码可达范围，否则取指 0xff | §6.1 确认 rom2_read 与 mask；必要时扩 mask |
| voice_enable 256-bit 耦合 | 固件写、PCM 读，位宽须一致 | GT 新寄存器 + ROM 补丁配套设计 |
| CPU 负载 | per-frame DSP 28→256 槽（~9×） | GT 侧可承受；注意采样率/实时性 |
| 原 28 路径回归 | A→B 重定向若误伤默认路径 | 默认不建 B/不重定向；VM↔GT diff 把关 |

---

## 9. h8dasm 可信度验证（2026-09-10，`tools/verify`）

**结论：h8dasm 的「字节偏移 / 指令长度 / 调用边 / 解码字段」对全部已执行代码经 GT 逐条验证 0 失败；§5 全部 patch 落点地址可字节级信任。另钉死 3 处文本层问题（§9.4，修复状态见节末）。**

### 9.1 验证方法（新增 `tools/verify/verify_dasm.py` + `semantics.py`）

基准 = GT 行为（VM trace ≡ GT trace，字节级一致）。
- **Phase A（长度/目标）**：PC trace 相邻两行的 next PC ∈ {s+len，分支目标，vector+首指令长（中断分发），s（sleep 空转）}。**833,332 条指令 0 失败**（555,015 sleep 空转、2,371 中断分发）。
- **Phase B（语义字段）**：全寄存器 trace，逐条核对 寄存器/SP/sr 效应与 GT 一致（检查器按 `src/mcu_opcodes.cpp` 逐指令实现）。**44,213 条 0 失败**（105,397 sleep 跳过、389 中断分发跳过）。
- 覆盖：**9217 执行 PC + 19 向量入口**（= dasm_full 的 PC 集；含 boot、note-on、per-frame DSP 三路 trace 并集）。

### 9.2 polyphony 锚点全部在已验证集内（2026-09-10 逐一核对）

config 写 `0x04134c` ✓ ｜ per-voice 循环 `0x45cc0` ✓ ｜ 28 通道建立 `0x041359/5c/62` ✓ ｜ voice_enable `0x5525/0x5527/0x5662/0x5664` ✓ ｜ 4 调用者 `0x344/0x7b6a/0x7d8a/0x461ac` ✓ ｜ note 填充 `0x1801–0x1a40`（156 PC）✓ ｜ `0x46699` ✓ ｜ `0x7c3f` ✓。（`0x1432` 是数据地址非 PC，不在集内属预期。）

### 9.3 本次验证发现并修复的 h8dasm 解码 bug

| bug | 影响 | 修复（`h8dasm.c`，依 GT） |
|---|---|---|
| `MOVG_Immediate` 尾部立即数未读（GT `mcu_opcodes.cpp:825`）：源操作数 indirect/absolute 且 ore∈{4,5,6,7} 时须再从码流读 1/2 字节 imm | 53 PC / 6,267 次转移；dasm 现成「短指令+伪分支」 | 按 ore 读尾部 imm（4/6→1B、5/7→2B）并打印 |
| 计数分支 `01/06/07` 按 2 字节解码；GT `MCU_Jump_JMP` 实为 **3 字节**（opcode2 + int8 disp）：`r[reg]--`，未下溢则 `pc+=disp`（06 需 Z、07 需 !Z） | `cntjmp` 类误译 | 3 字节、kind=3 显式目标，Phase A 逐条覆盖 |
| `0x11` 寄存器间接族未细分 | — | 按 GT 拆 `ret`(pop cp,pc) / `ret via rN:rN+1` / `jmp rN` / `jsr rN` |

→ dasm_full 已重生成：9,217 指令行、0 未解码；Phase A/B 复验均 0 失败。

### 9.4 文本层问题（不影响偏移/长度/字段，仅打印文本）——**已修复**

`ore`（opcode2 的低 3 位）语义依 ocode 而定；下列 ocode 的 `ore` **并非寄存器**，旧 h8dasm 一律打印 `OPC <src> r<ore>` 造成误读。已按 GT 逐 ocode 修正（`h8dasm.c` 通用族打印分支）：

| # | ocode | 旧文本（误导） | GT 事实 | 新文本（示例） |
|---|---|---|---|---|
| 1 | 18 (MOVG3) | `<src> r<ore>`（读式） | 方向 = `d=ocode&2`：16=读、**18=写**/XCH（direct+word） | `0x04134c` → **`MOVG3 r0 -> (br,$3c)`**（写，与 §2 pcmtrace 一致） |
| 2 | 1 (ADDQ) | `ADDQ r<ore> r<reg>` | `ore`=立即数：0→+1、1→+2、4→−1、5→−2（`MCU_Opcode_ADDQ`） | `0x45cdc` `a8 08` → **`ADDQ #1 r0`**（递增） |
| 3 | 2 (CLR 族) | `CLR r<ore> r<reg>` | `ore`=子操作码：0=SWAP、1=EXTS、2=EXTU、3=CLR、4=NEG、5=NOT、6=TST（`MCU_Opcode_CLR`） | `0x45cc0` `a5 13` → **`CLR r5`** |
| 4 | 3 (SHLR 族) | `SHLR r<ore> r<reg>` | `ore`=移位码：0=SHAL、1=SHAR、2=SHLL、3=SHLR、4=ROTL、5=ROTR、6=ROTXL（`MCU_Opcode_SHLR`） | `SHLR r0` / `ROTR r3` 等 |
| 5 | 0 (MOVG_Immediate) | `MOVG <src> r<ore> #imm` | `ore` 选操作：6/7=写 imm 到操作数、4/5=操作数−=imm（`mcu_opcodes.cpp:825`） | `MOVG #0x%02x -> <src>` / `SUB <src> #0x%02x` |

（`ore` 为寄存器的 ocode——MOVG2 读式 16、算术 4/5/6/7/8/10/12/14、扩展 20–23——保持原样，正确。）

**修复状态**：☑ 5 处文本修复 ☑ dasm_full 重生成（9,217 指令行、0 未解码）☑ Phase A/B 复验 0 失败（833,332 / 44,213）。
→ 至此 h8dasm 的**结构**（偏移/长度/目标/字段）与**文本**（操作数/方向）对全部已执行代码均可信；§2/§6 中用 pcmtrace 绕过方向/操作数的交叉验证可改为直接采信 dasm。

---

## 附：追踪插桩
- **`-pcmtrace`（已补回，`src/mcu.cpp`）**：`g_pcm_trace` + flag + `MCU_Write` 里对 `pcm[0x00-0x03/0x3c/0x3d/0x3e]` 的写日志（带 H8 PC，`pcm_trace.log`）。**本轮 config 写点（`0x04134c`）、voice_enable init（`0x04134f/0x041354`）、select_channel 建立（`0x041362`）结论均由 pcmtrace 实证**；note 表/空闲区结论用 **SRAM 快照字节（`demo_postW.bin`）+ raw ROM 字节 + h8dasm** 交叉实证。
- **h8dasm MOVG 方向**（2026-09-10 已修复，见 §9.4 问题 1）：旧版对 MOVG/MOVG2/MOVG3 族恒按 `<mem> <reg>`（读式）打印，**MOVG3（如 `0x04134c`）实为写**；现已按 `d=ocode&2`（16=读/18=写/XCH）分方向打印，`0x04134c` → `MOVG3 r0 -> (br,$3c)`，与 pcmtrace 实证一致。
  - **方向判定（GT/VM 同款，`mcu_opcodes.cpp:1057` / `h8vm_body.c:1055`）**：`d = (opcode & 2)` —— opcode **次字节 bit1** 定方向：bit1=1 → 写（`r[ore]→operand`；direct 则 XCH），bit1=0 → 读（`operand→r[ore]`）。h8dasm 的 MOVG2/MOVG3 命名按 **index（16/18）**，**不查 bit1** → 方向标注不可靠。
  - **config 写 `0x04134c`（`0d 3c 90`，opcode=0x90 bit1=0）**：按上述逻辑判为**读**，但 pcmtrace 实证是**写**（`reg=3c/3d val=c3/7b pc=04:134f cyc=972`，config←0x7bc3）。差异源：pcmtrace 记录的是**写时刻当前 PC**（= 后指令 PC，3 字节 MOVG3 结束于 0x04134f），且 `(br,$3c)` 经内存映射落到 PCM `base+0x3c`。**config 写结论不受影响**（pcmtrace 已钉死 config=0x7bc3、值源数据表 `0x1432`）；方向/映射细节待 GT H8 PC 推进时机 + 内存映射复核（见 §6 仍缺）。
- **h8dasm `r<ore>` 标注随 opcode 而变（本轮 VM 实证）**：h8dasm 对 `op≥0xa0` 族统一打印 `OPC <src> r<ore>`（`ore = 次字节 opcode & 7`，h8dasm.c:99,112），但 `ore` 语义依 opcode 而定（见 `h8vm_body.c` `MCU_Opcode_Table`）：
  - `MOVG2/MOVG3`：`ore` = **目的寄存器**（正确，如 `MOVG2 @r0+0xa368 r6` = r6←[r0+0xa368]）。
  - `CMP`：`ore` = **第二操作数寄存器**（正确；VM `MCU_Opcode_CMP` 算 `r[ore] − operand`，BNE 只判相等、方向无碍）。
  - `ADDQ`：`ore` = **立即数编码**（0→+1、1→+2、4→−1、5→−2，VM `MCU_Opcode_ADDQ`），**非寄存器** → h8dasm `ADDQ r0 r0` 实为 **`ADDQ #1 r0`（r0+=1，递增）**。
  - `CLR`：`ore` = **子操作码选择码**（3=CLR、6=TST、2=EXTU，VM `MCU_Opcode_CLR`），**非寄存器** → `CLR r5 r3` 实为 **`CLR r5`**。
   - **结论**（2026-09-10 更新）：h8dasm 的**字节偏移 / 指令长度 / 调用边 / 解码字段**经逐条 GT 验证 0 失败（§9）；**指令文本（操作数/方向）同已按 GT 修正**（§9.4，5 处 `r<ore>` 误导已消除）——对全部已执行代码可直接采信 dasm， pcmtrace 仅作旁证。
- 快照切法：`demo_postW.bin` 内 sram 段偏移 = `sizeof(mcu_t)+RAM_SIZE = 80+0x400 = 0x450`；`config_reg_3c/3d` 唯一匹配 `c3 7b` @fileoff 0xaf0a。
