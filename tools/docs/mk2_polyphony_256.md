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
→ 源码在手：`#define MAX_VOICE 255`（容量上限）兜底 → `ram1/ram2/fstate` 数组、`voice_mask`（256-bit）、`reg_slots`/`select_channel` 位宽**都从它派生**。GT 恒有 255 容量，实际激活数由 **`-voices:<n>`（28–255，真生效的参数化）** + 固件 config 驱动。

**Q：精确落点（全部验证过）？**
→ **config 写点 `0x04134c`**（`MOVG3` 写 `r0=0x7bc3`→reg 3c/3d，值来自**数据表 rom2 fileoff `0x1432`**；h8dasm 把 MOVG3 方向解反成读，pcmtrace 实证是写）、28 通道建立 `0x041333`（立即数 28 循环，§6.5 R12）、per-voice 搜索循环 `0x45cc0`（`0x45cbe` fallthrough 进入；4 origin 上下文 `0344/7b6a/7d8a/461ac`）、per-voice 状态 SRAM `0xa368…`（**核心 12 + 次级，全表 §6.1**，各 28 字节）。**⚠ 28 界不止一处**：R3/R4 实证全量 **~15 处界（13×28 + 2×32）** 分布在 pool sizing/build/reset/scan/init 例程（见 **§6.4 ③ 全表**）；256 化须全改，目标 **N=255**，B 例程集 = **26 逻辑/41 碎片**（§6.4 ④ → §6.5 R11/R12）。见 §5/§6/§6.5。

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
1. 固件 per-voice 数组 **28 slot**（核心 12 + 次级，§6.1）+ per-voice **搜索**循环 **28 次**（`0x45cde`；⚠ 28 界不止一处，全量 ~15 处见 §6.4 ③）。
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
| per-voice 搜索循环 | 28 次，界 `cmp #0x1c`（⚠ **非唯一** 28 界，全量 ~15 处见 **§6.4 ③**） | rom2 **`0x45cde`**；循环 `0x45cc0-0x45ce2`，体读 `@r0+0xa368`/`@r0+0xad0e`，`bne -32` 回 `0x45cc2` | ✅ ROM 字节 + 热点 98K |
| note 表 / per-voice 状态 | **28 slot × 并行数组**，每 slot 1 字节，`0xff`=空 | **核心 12 组** `0xa368…0xad0e`（间隔 0x1c）+ **次级 14 字节/16 字** `0xcdc6…0xd1ac`（全表见 **§6.1**）；`sram[0x2368]+` | ✅ SRAM dump + 码（§6.1 三方交叉） |
| br 页性质 | **PCM DSP 控制接口**（非 per-voice 结构体） | `br+0x00-03`=voice_enable、`0x04-37`=per-channel ram1/ram2、`0x3c/3d`=DSP config、`0x3e`=select_channel+IRQ；`br+0x3c` **只写不读**（全 dasm 无 `movl/movs br,$3c`） | §6.1 + dasm |
| SRAM 簿记 | `(dp,0xa42d)`=28（池 free 计数）、`(dp,0xa42f)`=27（a3d8 链头）、a42e=0、a42c=0 | ⚠ 旧表 a42c/a42e 错位；订正见 **§6.5 R12**（快照 file `0x287c`=`00 1c 00 1b 00`） | ✅ 快照 |

> **关键修正**（本轮追踪实证）：`config_reg_3d=0x7b → reg_slots = 28`（**不是 32**）——`pcm.cpp:536` `(0x7b&31)+1=28`。故全链路（note 表 28 / 循环 28 / voice_enable 28-bit / reg_slots 28）**都是 28**；"32" 仅硬件上限（`&31`/`&0x1f`），从未用到。`0x04134c` 是 **写**（`MOVG3` 写 `r0=0x7bc3`→config；h8dasm 的 MOVG/MOVG2/MOVG3 族恒按 `<mem> <reg>` 读式打印，方向对 MOVG3 解反，pcmtrace 实证是写）；config 值源 = 数据表 **rom2 fileoff `0x1432`**。`0x46699` 的 `cmp #0x1c` 是**返回值阈值**（`jsr 0xbb36` 后比较），非循环——per-voice **搜索**循环是 `0x45cc0`（`0x45cbe` fallthrough 进入，4 origin 上下文）。**⚠ 后续 R3/R4 修正：per-voice 28 界不止这一处**（全量 ~15 处 28/32 界见 §6.4 ③）；本行"唯一"为早期结论，已作废。

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

1. **`#define MAX_VOICE 255`**（`pcm.h` 或 `mcu.h`；容量，非运行时值），派生：
   - `ram1[MAX_VOICE+4][8]`、`ram2[MAX_VOICE+4][16]`、`fstate[MAX_VOICE+4][2]`（+4 = 效果槽外置 EFF_BASE=256）。
   - `voice_mask`/`voice_mask_pending` → 256-bit（32 字节），`PCM_WriteExt/ReadExt` 经 `0xe800` 窗口；latch 整 32B。
   - `reg_slots`：**默认路径保持 `(config_reg_3d&31)+1` 不变**；扩展模式（`-voices` 生效时）用 `config_reg_3d+1`（→ 任意 N）。⚠ 勿用"第 7 位"判别式（N<128 会错）。
   - `select_channel` 去 `&0x1f`（扩展模式 0..N-1）+ 新增"效果扩展通道"寄存器（效果槽 256..259）。
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
-voices:<n>     set polyphony (default 28, range 28-255; ROM patched in memory only)
```
- **语义（已定，2026-09-11）**：单横线+冒号，与 `-gain:N` 风格一致。**n 在 28–255 间真实生效**：当 `n != 28` 时 `MCU_PatchROM` 在内存副本里建 B，并把 **n 印进 B 的所有循环界与 config 字节**（`config_reg_3d = n-1`）——生成器（C）参数化，无需运行时传参。
- 数组容量固定 255 宽（page6 布局不变），n 只是用前 n 槽；效果槽固定 256..259，与 n 无关。
- 解析位置：`main()` 参数循环（`mcu.cpp:1797` 起，紧随现有 `-gain:/-p:/-mk2/-float/…`）。
- 验证矩阵：28（逐字节回归）＋255（压力）必测；另抽 1–2 个中间值（如 100）验证参数化。

---

## 5. 精确 patch 点（验证过的锚点）

> **⚠ 本节为 (i) 时代初版锚点表，最终落地以 §6.3/§6.4 为准**（(ii) 方向：18 例程搬 B + ~15 处界重写 + A→B 重定向）。下表锚点（config 写点/voice_enable 写簇/per-voice 数组地址）仍有效；但 **#4「唯一 per-voice 循环」已被 R3/R4 推翻**（28 界全量 ~15 处见 §6.4 ③），**#7「4 调用者 / `0x7c3f` jmp r1 间接分派」已纠错**（`0x7c3f` 是 `ret`，非 `jmp r1`；`0x45cc0` 经 `0x45cbe` fallthrough 进入，见 §6.4 ①）。

| # | 地址（rom2 flat / 类型） | 字节 | 改什么 | 来源 |
|---|---|---|---|---|
| 1 | **config 写点 `0x04134c`**（`MOVG3` 写 r0；值源**数据表 rom2 fileoff `0x1432`**） | `0x1432`:`c3 7b`→`c3 <n-1>`（255 时 `c3 fe`；`0x7bc3`→`0x(n-1)c3`） | config_reg_3d=n-1 → 扩展模式 reg_slots=n（§4.1/§6.5 R10） | pcmtrace `cyc=972` + raw `0x1432` |
| 2 | `0x04135c` | `58 00 1b` | `movi r0 #0x1b`（28 通道）→ n 版（或跳 B） | 动态 |
| 3 | `0x041359/0x041362` | `05 3e 81`/`70 3e` | select_channel 建立/写 → 扩/跳 B | 动态 |
| 4 | `0x45cde`（唯一 per-voice 循环） | `40 1c` | 循环界 28 → 跳 B 循环（推荐，避 8-bit 界） | 动态 |
| 5 | `0x005525/0x005527`+`0x005662/0x005664`（voice_enable 写，**2 簇**） | `movsw`×2/簇 | 28-bit 掩码 → 256-bit（**32 字节**；落点依 R2 方案 B/C，`br+0x04+` 撞 ram1 不可用） | 动态+R2 |
| 6 | per-voice 数组（SRAM 核心 12 `0xa368–0xacf2` + `0xad0e` + 次级 `0xcdc6–0xd1ac`，**全表 §6.1**） | — | 28 项 → 256 项（**挪 rom2 B**，见 §6.2）；各 `@r+disp16` 基址改指 B（次级 `0xcdc6–0xd1ac` 亦需同步） | SRAM dump + 码 |
| 7 | per-voice 循环 **4 调用者**：`0x00000344`/`0x00007b6a`/`0x00007d8a`/`0x000461ac`（`0x7c3f jmp r1` 间接分派到其一）；note 填充 `rom1 0x1800-0x1a40` | — | **patch 落点 = 循环本体 `0x45cc0`**（界 `0x45cde` → 跳 B 256 循环或就地改界），4 调用者免改 | dasm 调用边 |

> ROM 偏移换算：flat `0x0004134c` = rom2 文件偏移 `0x134c`（flat = `0x40000` + 偏移）。

---

## 6. 侦查结果（本轮 GT + h8dasm 实证，2026-09-09 二）

1. **rom2 空闲区** ✅：`rom2.bin`=512 KB。已执行 PC 最高 flat `0x04bb7d`（fileoff `0xbb7d`）。大段 0xff 填充空闲区：fileoff **`0x5f446-0x70000`（68 KB）**、`0x72580-0x7fffe`（55 KB）、`0x49fd8-0x50000`（24 KB）、`0x3206a-0x37e00`（24 KB）、`0xc9b2-0xfff0`（14 KB）。**B 建议放 fileoff `0x60000+`**（68KB 区内，对齐好、远离已用代码）。
2. **SRAM 原位扩** ✅（结论：**不现实**）：note 表基址 `0xa368` 之后**无** ≥1536 字节 连续空（`0xa3f4-0xad0d` 有 710 字节 数据；核心数组间隔 0x1c 会重叠）。SRAM 唯一 ≥1536 字节 连续区是 **4KB（BR `0x9060-0xa040` / `sram[0x1060-0x2040]`）**，但被**系数表密集引用**（`ADD @r3+0x9xxx`，`0x9000-0x9740` 间隔 0x20 + `0x9f50`/`0xa040-0xa090`）→ **note 表宜挪 rom2 B**（与 A/B 设计一致）。
3. **A 调用点**（⚠ 部分被 §6.3/§6.4 修正）：per-voice **搜索**循环（`0x45cc0`）经 **`0x45cbe` fallthrough** 进入，在 4 个 origin 上下文（`0x00000344`、`0x00007b6a`、`0x00007d8a`、`0x000461ac`）下到达。**纠错（§6.4 ①）**：`0x7c3f` 是 **`ret`**（字节 `11 19`），**非 `jmp r1`**；"2011+ 目标"是 ret 返回扇出，非间接跳表。note 表填充 = **`rom1 0x1800-0x1a40`**。
4. **note 表项宽** ✅（**1 字节/slot**）：SRAM dump 实证核心数组各 28 字节（**核心 12 + 次级，全表 §6.1**），`0xa3d8`=`ff,0,1,…,1a`（slot0=0xff 空标记，slot i=i-1）→ 每 slot 1 字节、共 28 slot。per-voice 循环 `MOVG2 @r0+0xa368`（r0 步进 +1）读 2 字节 = 当前 slot + 下一 slot，逻辑项仍 1 字节。
5. **28 vs 32** ✅（**实际 28**）：config_reg_3d=0x7b → reg_slots=28（`pcm.cpp:536`）。note 表 28 / 循环 28 / voice_enable 28-bit / reg_slots 28 全一致；32 仅硬件上限（`select_channel &0x1f`、config `&31`），未用到。

**本轮补齐 / 仍缺**（2026-09-09 三，pcmtrace + raw 字节实证）：
- **config 写点** ✅（**已钉死**）：`0x04134c`（`MOVG3` 写 `r0=0x7bc3`→reg 3c/3d，`cyc=972` 一次性）。`r0` 值源 = 数据表 **rom2 fileoff `0x1432`**（`c3 7b`，boot config 表首项；经 `MOVG2 @r1+0x1432 r1` 读 + `OR r1 r0`）。**patch = 改 `0x1432`:`c3 7b`→`c3 fe`**（→reg_slots=255；GT 公式分支见 §6.5 R12）。附：`0x04134f/0x041354` 是 voice_enable **init 写**（r7=0→reg 00-03），`0x041362` 是 select_channel 建立循环（27→0，28 通道）。
- **per-voice 循环调用者**（⚠ 已按 §6.3/§6.4 修正）：`0x45cc0` 经 `0x45cbe` fallthrough 进入，4 个 origin 上下文可达（`0344/7b6a/7d8a/461ac`）。**纠错**：`0x7c3f` 是 `ret` 非 `jmp r1`（见 §6.4 ①）。**(ii) 下 patch 落点 = 各 18 例程 A 入口 + `0x45cbe` 边界**（§6.3/§6.4 ④），非单改 `0x45cc0`。
- **voice_enable 256-bit 设计**（**位序已钉死，方案已定：路线 C @ `0xe800`**，2026-09-10 R2 研究）：
  - **位序（Confirmed，GT T0 端到端）**：消费侧 `pcm.cpp:1116` `key=(voice_active>>slot)&1` → **voice_mask 第 N 位 = slot N = voice N**（1:1）。写侧 `pcm.cpp:74-95`：`br+0x00`→bit24-27(4b)、`0x01`→16-23、`0x02`→8-15、`0x03`→0-7。组装 `mcu.cpp:1040 MCU_Write16` 大端（高字节→低地址）：`movsw r3@(br,$00)`+`movsw r4@(br,$02)` ⇒ **`voice_mask(32b)=(r3<<16)|r4`**，r3=voice16-31、r4=voice0-15，bit N=voice N。
  - **纠错（原提案 3 处）**：① "扩 256 位 = 8 字节" **错**——256 bit = **32 字节**（=8×32-bit 字），8 字节仅 64 bit（=64 voice）。② PCM 控制区仅 **64 寄存器**（`address&=0x3f`；H8 窗口 `0xe000-0xe3ff` 整 1KB 别名到这 64 字节），`br+0x04-0x0F`=per-channel **ram1**（`pcm.cpp:128`）→ **"reg 0x00-0x07" 撞 ram1，不可行**。③ "4 个 movsw 写点" 实为 **2 簇**：停用 `0x005525/0x005527`、启用 `0x005662/0x005664`（各 2×`movsw`）；`0x005529/0x005666` 的 **`movl`(0x60-0x6f=`MCU_Opcode_Short_MOVL`) 是读**（`r[reg]=MCU_Read`，br+0x00 回读），非写。
  - **方案（已定：路线 C，2026-09-10；默认 28 路径零改动）**：**C** = 第二 PCM 窗口 **`0xe800-0xe83f`（br=0xe8）**，64 字节块（页0 空闲，读/写均 Unknown 未映射）→GT `voice_mask` 扩 256-bit，保 PCM chip 抽象。（备选 B = page6 SRAM 直读，未选。）**布局注意（2026-09-10 核实回滚）**：`0xe400-0xe7ff` **不是**空闲——是 **GA/LCD/IRQ 设备块**（`mcu.cpp:888` `base|0x400..base|0x800`；e402=IRQ、e404/5=LCD），紧挨 PCM 窗口，硬扩必撞；故选其后的 `0xe800`。
  - **落点细节（C）**：**低 32 bit（voice 0-31）留在主窗口 `br+0x00-0x03`（不动，默认路径字节级一致）**；**高 224 bit（voice 32-255，28 字节）经新窗口 `0xe800+0x00-0x1b`（br=0xe8）**。GT `PCM_Write` 增 `0xe800` 分支（`PCM_WriteExt`）拼 bit32-255；固件 256 路径在原有 2×movsw（低 32 bit）后**追加 14 次字写**（28 字节）到 br=0xe8。
  - **连带扩展（R1b/R4 联动）**：`d150-d156`(4 字节→32 字节)、`a4b4`/`acf2`(28 字节→256 字节，随 R1b 落 page6)、per-slot 累积循环 **`0x001ada`**（`r1` 0x1b→0，28 次 `acf2[r1]|=a4b4[r1]`）28→256。
- **per-voice 循环 ADDQ 语义** ✅（**VM 钉死**）：循环递增 `0x45cdc` 字节 `a8 08`，h8dasm 标 `ADDQ r0 r0`（易误读成翻倍），VM `MCU_Opcode_ADDQ` 实证 `a8`=寻址（direct r0，word）、`08`=opcode（`>>3`=1=ADDQ，`&7`=0→**+1**）→ **`ADDQ #1 r0`（r0+=1，递增）**。故 r0 = 0..27，`cmp r0,b #0x1c`（28）为**真循环上限**，**循环跑满 28 次**（非翻倍）。消掉"翻倍则循环到不了 28"疑虑。

### 6.1 per-voice 内存布局 & 「28」落点全表（2026-09-10 sub-agent 独立调查：dasm + pcm_trace + GT 源码三方交叉）

**布局结论**：per-voice 状态**不是** br 页结构体，而是 **SRAM 里一组以 slot(0..27) 为下标的 28 元素并行数组**（搜索循环用 r0、分配/释放/填充用 r1 作 slot）。br 页（BR=0xe0，`0xe000–0xe03f`）是 **PCM DSP 控制寄存器接口**：`br+0x00-03`=voice_enable、`br+0x04-37`=per-channel ram1/ram2 参数、`br+0x3c/3d`=DSP config、`br+0x3e`=select_channel+IRQ。**「28」由三处共同定**：`config_reg_3d=0x7b→reg_slots=28`（`pcm.cpp:536`）＋ 循环界 `0x045cde` ＋ SRAM 计数器 `(dp,0xa42c)=28/0xa42e=27`。

**核心 per-voice 数组（码 + 快照实证，置信高）：**

| 基址 | 宽 | 含义（证据） |
|---|---|---|
| `0xa368` | 28 字节 | slot→index A；搜索循环 `0x045cc2` 读 + `CMP r4` |
| `0xa384` | 28 字节 | slot→index B；搜索循环读 |
| `0xa3a0` | 28 字节 | slot 分配标志；`0x94`=已分配（`0x001867`/`0x001a17`），free 时 CLR |
| `0xa3bc` | 28 字节 | slot 数据；alloc（`0x00183d`）/free 时 CLR |
| `0xa3d8` | 28 字节 | slot 链/prev 指针（快照 `ff,0,1,…,1a` 恒等） |
| `0xa3f4` | 28 字节 | slot "prev" 指针；`0xff`=空 |
| `0xa410` | 28 字节 | slot "next" 指针；`0xff`=空 |
| `0xa4aa` | 28 字节+ | slot→sequence；`a4aa[N]=0xff` 哨兵（`0x001928`） |
| `0xa4b4` | 28 字节 | slot 激活标志；OR 进 voice mask（`0x001ad3`），alloc/free CLR |
| `0xa46c` | 28 字节 | per-voice 字段（填充循环 `0x000fc6`） |
| `0xacf2` | 28 字节 | 累积 voice mask（`0x001ad3` 建立） |
| `0xad0e` | 28 字节 | slot→PCM 通道；`0xff`=空；搜索循环 `0x045cca` 读 |

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

→ ~~**真·28 复音循环只有 `0x045cde` 一处**~~。**⚠ 此结论已被 R3/R4 推翻（2026-09-10，见 §6.4）**：上表只覆盖了**显式 8-bit 立即数** `#0x1c`；大量 per-voice 循环用 **寄存器界** `movi rN #0x001b`（27）+ `cntjmp`（27→0 = **28 次**），grep `#0x1c` 找不到。**全量 28/32 界清单见 §6.4**（~14 处 28 界 + 2 处 32 界）。通道 **28–31（`0x1c–0x1f`）是固定效果通道**（reverb/chorus/mix/LFO，见 pcm.h），与 28 个复音 slot 0–27 不同——扩复音若占用 28–31 会让渡对应效果（0x005e20/0x005ee7 属此类，非复音界）。

**voice 分配 / 释放（置信高）：**
- 分配 `0x001823`：`d0e0[slot]=4`、CLR `ad0e/a3bc/a4b4`、更新 `a3d8` 链、`a3a0[slot]=0x94`、`(dp,0xa42d)++`、`bsr 0x1b44/0x1bad`。
- 释放 `0x0019c4`：`BTSTI a3a0[slot] bit7`（置位早退）、`SUB ad0e[slot] #0xff`（原 0xff 早退）、CLR `a3bc/a4b4`、解链 `a3f4/a410/a3d8`、`a3a0[slot]=0x94`、`(dp,0xa42d)++`。
- note→slot/通道选择：循环 ~`0x0017a9–0x0017e6`（**部分**跟踪，置信中）。

**voice_enable 维护（32-bit，`br+0x00–0x03`，置信高）：** SRAM 里维护为四 word —— `(dp,0xd150/0xd152)`=当前 mask、`(dp,0xd154/0xd156)`=待发 set/clear。**2 簇**写 PCM（各 2×`movsw`）：停用路径 `0x005525/0x005527`（`r3=d150&~d154`、`r4=d152&~d156`→`movsw @(br,$00/$02)`）；启用路径 `0x005662/0x005664`（`r5=d150|d154`、`r6=d152|d156`）；其后 `0x005529/0x005666` 的 `movl`(0x60-0x6f) 是 **br+0x00 回读（非写）**。per-slot `a4b4` 由 `0x001ada` 的 28 次循环（`r1` 0x1b→0）OR 进累积 mask `acf2`。位图（pcm.cpp，**bit N=voice N**）：`br+0x00`→voice 24-27(4b)、`0x01`→16-23、`0x02`→8-15、`0x03`→0-7。

### 6.2 R1 研究：per-voice 状态落 page6 的**页寄存器/基址改点**（2026-09-10，GT 源码 + dasm 9217 PC 实证）

**为什么不是"改基址寄存器 dp/tp=6"一行**（纠正 R1b 原设想）：

1. **全部 per-voice 访问走 page=dp（Confirmed）**：dasm 逐条枚举 **245 个独立指令点**（`@r0`:16、`@r1`:186、`@r2`:4、`@r3`:11、`(dp,disp16)`≈28），覆盖核心 12 + 次级 27 数组（§6.1 全表），横跨搜索循环 `0x045cc0`、alloc `0x001823`、free `0x0019c4`、fill（rom1 `0x001800+`/`0x000fc6`）、mask 累积 `0x001ada`、rom2 DSP（`0x404xx`/`0x45e0`/`0x4668`…）。
2. **页寄存器映射（Confirmed，GT `MCU_GetPageForRegister` `mcu.h:208`）**：`r0-r3→dp`、`r4-r5→ep`、`r6-r7→tp`。`@rN+disp16` 有效地址 = `(page<<16)|((rN+disp16)&0xffff)`（`mcu_opcodes.cpp:631/646`）——**16-bit disp 加不出 page6 的 bit16**，故到 page6 **只能**把某个页寄存器置 6（H8 无 32-bit 数据寻址；`MOVF`/`@r6+disp8` 亦走 tp，`mcu_opcodes.cpp:719`）。
3. **三个页寄存器中只有 tp 可独占（Confirmed）**：
   - **dp 不可独占**：188 次 `LDC #0 r5`（全 0x00，防御式写 0）+ ~2800 处引用（含 §6.4 全局变量表 `0xfb80`/`0xfdc0`）→ 置 6 会打断所有其它 dp 访问。
   - **ep 不可独占**：`LDC #imm r4` 值域 **0/1/2/3/4/10**（36 次），固件**动态换页**经 `@r4/@r5` 取 rom2 数据表（如 `LDC #4 r4`→`@r4+0x0542`）→ ep 不是恒 0，不可独占。
   - **tp 唯一可独占**：全固件**仅 1 次写**（boot `0x00022b: LDC #0 r7`→tp=0）之后恒 0；仅 **~20 处** `@r6/@r7` 用户（均执行、以 tp=0 落 page0：全局变量表 `@r6+0xfdXX`、`@r6+0x1bfe/0x05f4/0x1310` 等，含 `MOVF` 形式 `@r6+disp8`）。→ 独占 tp=6 时这 ~20 处需搬迁到 dp 基。
4. **方向已定 = (ii)（2026-09-10 拍板）**：per-voice 例程**整体搬 B 重写**（256 + page6 + tp=6），**A 的 28 例程零改动**（默认路径逐字节一致 → 回归最安全，与 A/B 设计初衷一致）。代价：B 需含 per-voice 例程完整副本、代码量大。
   - **B 内容（重写副本，page6/256/tp=6）**：per-voice 例程集（见 §6.3 清单）里每处的 ~245 站点访问全部改为：基址寄存器 `r1/r0/r3`→**`r6`**（tp=6）、slot 索引移入 r6、`disp`→page6 偏移（0x0000/0x0100/…，256 字节间距）；B 例程入口 `LDC #6 r7` 建 tp=6（自包含，不改 A 的 boot）。
    - **寄存器分配（B 内逐例程核）**：搜索循环 `r6` 是 temp（读 slot 数据）→该处索引须换寄存器（**r7=SP，不能当索引**，PushStack 用；§6.5 R8）；alloc/free/fill 用 `stm #0x3e` 保存 r1-r6 → 索引用 `r6` 安全。
    - **A→B 重定向机制（已定 (a)，§6.3 研究闭环）**：因 A 例程**零改动**，256 模式下 per-voice 操作须导向 B。选 **(a) 每个 A 例程入口首字节 patch `jmp B_entry`**（调用者全免改）。原"关键待核：是否有中途 branch target"已在 §6.3 答清——**全 entry-only**，仅 2 例外：① 搜索循环 `0x45cc0` 是 `0x45cbe` fallthrough 进入→patch `0x45cbe` 边界；② PCM write1/2 簇模块内→随 `0x516c` 顶重定向。故 (a) 可行。
   - **~20 处 `@r6/@r7`（tp=0/page0）**：A 不动→保持 tp=0 行为不变（默认路径安全）；**B 内**若需 tp=6 又用 r6/r7 基址，须自洽（B 例程内 r6/r7 归 page6 用，不与 A 的全局变量表 `@r6+0xfdXX` 混用——B 用 r6 时不再依赖 tp=0 的全局表，或 B 内先存 r6 改指）。
5. **b_ram 初值（gap2，定）**：page6 `b_ram`(64KB) **零初始化**（GT 静态数组天然 0）；256 slot 初值由 **B 的重写 fill 例程**计算填充（位置相关初值：链/恒等/0xff 空标记/`d0e0[slot]=4` 等）。**无需从 rom2 载入静态初值表**。
6. **(ii) 下 tp 无需全局独占 A**：tp=6 只发生在 **B 例程内**（自 `LDC #6 r7` 起）；A 的 boot `0x00022b: LDC #0 r7` 与 ~20 处 `@r6/@r7`（tp=0）**全不动** → 默认路径 page6 相关零触碰。

### 6.3 R1(ii) 研究：per-voice 例程集合 + 调用图 + A→B 重定向点（2026-09-10，dasm + flow_main 前驱实证）

**⚠ 数据解读修正（Confirmed）**：`dasm_full.txt` 行尾 `;-> XXXXXXXX` **不是直接调用者**，是"哪个顶层 IRQ 上下文可达该块"（origin label）。**直接调用者 = `flow_main.txt` 的前驱（src→dst 边）**。例：`flow_main` 无 `00000344→00000f86` 边，但 `0x00000f86` 行带 `;-> 00000344`。→ 重定向点只能用 flow_main 前驱找。

**顶层控制流骨架（5 个 IRQ/向量入口 + 5 个 return 点）**：入口 `0x00000344`/`0x00000524`/`0x00007b6a`/`0x00007d8a`/`0x000461ac` → 汇流 `0x000003db→0x000003fd→0x000004af`（事件出队到 `(dp,0xfdc0/0xfdc2)`，rte `0x00000508`）。return 点 `0x00007c3f`(ret)/`0x00007dc3`/`0x000003ba`/`0x000003fc`/`0x00000508`(rte) 是**中断恢复**扇出，非逻辑调用，不重定向。

**per-voice 例程集合（13 逻辑例程 + 2 共享 helper，全单一入口）：**

| # | 例程 | 区域 | 入口 | 直接调用者（flow_main） | 备注 |
|---|---|---|---|---|---|
| 1 | note-fill | rom1 `0x0f86–0x1188` | `0x0f86` | note 路径内部 | 自环；`jsr 0x516c` |
| 2 | note dispatcher | rom1 `0x15e0–0x173d` | `0x15e0` | `0x15de` fallthrough | steal 循环 |
| 3 | note-on/off handler | rom1 `0x173e–0x17ec` | `0x173e` | `0x15f3` | → slot-select |
| 4a | slot-select | rom1 `0x179c–0x17ec` | `0x179c` | `bsr` `0x1744/0x175f/0x1781` | 触 acf2 |
| 4b | note helper | rom1 `0x17ed–0x1801` | `0x17ed` | 7 处（`0x150e/1667/1701/1720/174c/1768/178a`） | → alloc |
| 5 | alloc A | rom1 `0x1823–0x187d` | `0x1823` | `bsr` `0x17f9` | jsr H1/H2/0x516c |
| 6 | alloc B | rom1 `0x187e–0x18cd` | `0x187e` | `bsr16` `0x1815` | jsr H1/H2 |
| 7 | free | rom1 `0x19c4–0x1a23` | `0x19c4` | `bsr16` `0x0587`（cleanup `0x055c`） | jsr H1/H2 |
| 8 | mask-acc | rom1 `0x1ad3–0x1af1` | `0x1ad3` | `bsr` `0x0621/0x0827` | acf2 循环 |
| 9 | PCM voice-enable | rom1 `0x516c→…` | `0x516c` | `jsr` `0x10d5/1157/1829/1886` | → write 簇 |
| 10 | PCM write1 簇 | rom1 `0x54fb–0x552b` | `0x54fb` | 模块内 `0x5366/0x53cf` | `movsw r3 @(br,$00)` |
| 11 | PCM write2 簇 | rom1 `0x564a–0x5670` | `0x564a` | 模块内 `0x5373/0x53d5` | `movsw r5 @(br,$00)` |
| 12 | **voice 搜索循环** | rom2 `0x45cc0–0x45ce0` | **`0x45cbe` fallthrough**（**非 jsr**） | 4 origin：`0344/7b6a/7d8a/461ac` | **唯一结构例外** |
| 13 | rom2 voice init | rom2 `0x40462–0x40586` | `0x40462` | `pjsr` `0x0565` | 建 free list |

**共享 helper（必须连同搬，否则 jsr 落回旧 28 版静默失效）：** `H1 0x00001b44`（rts→`0x1b8f`，被 alloc A `0x1847`/alloc B `0x18a2`/free `0x19f7` 调）、`H2 0x00001bad`（rts→`0x1bfd`，被 alloc A `0x1870`/alloc B `0x18c0`/free `0x1a20` 调）。

**(ii) 重定向可行性（Confirmed）：结构干净。** 除 2 例外全部 **entry-only**（外部控制流只在例程顶进入）：
- **搜索循环 `0x45cc0`**：fallthrough 进入（从 `0x45cbe` 掉入）→ 重定向须 **patch `0x45cbe` 边界**（或连 `0x45cbe…` 前导一起搬）。
- **PCM write1/2 簇**：模块内部 → 随 R9 一起搬，在 `0x516c` 顶重定向即可。
- "假阳性外部跳入"（如 `0x184a←0x1b8f`）全是 **helper rts 返回**，非分支 → 搬 helper 后 rts 目标自动落在 B 内。

**(ii) 重定向点全清单（阶段 2 落地用，**R3/R4 后已扩充，最终以 §6.4 ④ 为准**）**：§6.3 初判 14 处（13 例程顶 + `0x45cbe` 边界）→ §6.4 扩充后 B 例程集 = **18**，A 侧重定向 ≈15 入口 + `0x45cbe`（新增：`0x5869` 区/`0x041220`/`0x0433xx`/`0x041333`）。每处 A 首字节 patch `jmp B_entry`（仅 `-voices≠28` 时），A 本体其余字节不动。

**⚠ 真正的 256 工作量在数据结构，不在控制流（(ii) B 重写核心）**：
1. **0x1c/0x1b 循环界**：搜索循环 `cmp r0,b #0x1c`、`movi r1 #0x1b`，及 rom2 init/alloc/free/mask-acc 同款 → 改 0x100/0xff + 数组重排。**全量界清单见 §6.4**（~14 处 28 界 + 2 处 32 界）。
2. **64-bit PCM voice-enable mask（`d150–d156`）**：现跟踪 ≤64 voice → 256-bit 重建 + write1/2 簇位寻址重写（与 §6.2 R2 路线 C `0xe800` 联动）。
3. **`acf2` 占用累积器**：28→256 扩容 + 饱和算术复查。

### 6.4 R3/R4 研究：28/32 界全量清单 + 例程集扩充 + `0x7c3f` 纠错（2026-09-10，dasm + flow_main + raw 字节实证）

**① `0x7c3f` 纠错（Previous assumption invalid → Rollback，协议 §10）**：
- **Previous assumption invalid**：`0x00007c3f` = `jmp r1` 间接 per-frame 分派（"2011+ 目标"）。
- **Correct GT result**：`rom1[0x7c3f]=0x11, [0x7c40]=0x19` → 指令 **`[11 19]` = `ret`**（pop cp,pc）。opcode 族校准（dasm 全样本 `[11 XX]`）：`19`=ret、`D1`=jmp r1、`D2/D3/D6`=jmp r2/r3/r6、`D9/DE`=jsr r1/r6。`0x7c3f` 后字节是 `19` 非 `D1` → **是 `ret` 非 `jmp r1`**。
- "2015/2011 目标"真相：`flow_main` 从 `0x00007c3f` 有 **2015 条出边** = ret 弹出的**返回地址扇出**（各 jsr/中断上下文遗留的返回点），非跳表。dasm 的 `;+2011 more` = 4 显示 + 2011 = 2015 个**可达 origin 上下文**计数，**非跳表长度**。
- **真实 per-frame DSP 路径（Confirmed）**：`0x00007d8a`(IRQ) → `pjsr #0x04:14d8` → `0x0414d8: jmp #0x04aca9` → **`0x04aca9` = per-frame DSP 体**（遍历**动态活跃 voice 链表**，头 `(dp,0xd68a)`，每节点调 `0x04043a5`）→ `0x04adce: jmp #0x0414db` → `0x0414db: ret` → `0x00007dab` → `0x00007dc3: rte`。**无跳表**，单条直接 `jmp`。

**② 关键结构发现（R3，Confirmed）**：**28 不在 per-note 路径**。note→slot/alloc A/B/free **无 28 循环**——它们走**哨兵终止的 per-channel 链表**（头 `(perch,0xa220)`、next `+0xa250`）+ 在 pool 栈 `(dp,0xa430)` 上**单 voice 压/弹**。**这些自动扩展到 256**（只要 pool 建成 256 大、数组在 page6）。**28 硬编码在 pool 的 sizing/build/reset/scan/init 例程**（下表）。→ (ii) 的 B 须重写这些例程的界；per-note 路径因访问数组也须 page6 重定向，但其**循环无需改界**。

**③ 全量 28/32 界清单（R3+R4 合并，grep `#0x1c` 漏掉的寄存器界 `movi #0x001b`+`cntjmp`=28 次、`movi #0x001f`+`cntjmp`=32 次）：**

| PC | 例程 | 做什么 | 界 | conf |
|---|---|---|---|---|
| `0x040508`/`04050d` | rom2 pool-init `0x040462`(loop C) | **free-list BUILD（pool 尺寸=28，关键）** | `movi r1 #0x001b` | Confirmed |
| `0x040462`/`040469` | 同上 (loop A) | per-voice clear（21 字段 ce5c…a46c） | `movi r1 #0x001b` | Confirmed |
| `0x0404d9`/`0404dc` | 同上 (loop B) | sentinel 置位（a3f4/a410/a3bc/a3d8） | `movi r1 #0x001b` | Confirmed |
| `0x04053b` | 同上 (loop D) | **per-channel** reset（a250/a288/a26c/a314/a2c0/a2dc） | `movi r2 #0x001b` | Confirmed |
| `0x040565` | 同上 (loop E) | **per-channel** reset（a210/a220/a230/a240/a200） | `BPL` 0…N | Confirmed |
| `0x001ad7` | rom1 mask-acc `0x1ad3` | a4b4/acf2 OR 累积 | `movi r1 #0x001b` | Confirmed |
| `0x0051d9` | rom1 pool-scan | **PCM IRQ pending 扫描**（`TST @r1+0xd15c`；**非零**命中→`0x51ef CLR`→`0x25f0` 服务；⚠ 旧"find-free==0"标签作废，见 §6.6 R15） | `movi r1 #0x001b` | Confirmed |
| `0x0051fc` | rom1 pool-scan | find-free（`[r1+0xd0e0]==0`） | `movi r1 #0x001b` | Confirmed |
| `0x005886` | rom1 pool-scan | per-voice 表 scan（`0x64d6`） | `movi r1 #0x001b` | Confirmed |
| `0x0058d7` | rom1 pool-scan | per-voice 表 clear（`0x64d6`） | `movi r1 #0x001b` | Confirmed |
| `0x041239`/`04123c` | rom2 per-voice init | init（ad0e/d15c/表 0x64d6） | `movi r0 #0x001b` | Confirmed |
| `0x041305` | rom2 per-voice init | init（`@r1*4+0x64d6`） | `movi r1 #0x001b` | Confirmed |
| `0x04135c`/`041362` | rom2 28-通道建立 `0x041333` | **立即数循环（⚠ 旧"死码"结论 2026-09-11 作废）**：`0x135c movi r0 #0x1b`（r0=27）→ `0x1362 movs r0 @(br,$3e)`（`MOVS`=**写** select_channel=r0）→ `0x1374 cntjmp r0,-21`（28 次，通道 0..27）；每迭代建 `(br,$12/$14)=0`、`(br,$16/$18/$1a)=0xba` | `movi #0x1b`（256 化改 `#0xfe`） | Confirmed（T0 raw `58 00 1b` + `Short_MOVS`=store） |
| `0x045cde` | rom2 DSP search | 已知"真·"搜索循环 | `cmp r0,b #0x1c` | Confirmed |
| `0x04331b` | rom2 reset | `dp,0xd435[0..31]`（**32 元素**） | `movi r0 #0x001f`（**32**） | Confirmed（旧记 0x4331e=循环体） |
| `0x043326` | rom2 reset | `dp,0xd1ac[0..31]`（d1ac=per-voice 次级，**32 宽**） | `movi r0 #0x001f`（**32**） | Confirmed（旧记 0x433323） |

**可变界（自动扩展，无需改界）：** `0x04aca9`（per-frame DSP 体，活跃 voice 链表遍历，0..28 可变）；per-note 路径（per-channel 链表 + 单 voice push/pop）。

**SM 侧 27/28 常量（已核，Strongly supported）**：`0x0404cd/d1/d5`→a432=15/a434=27/a436=27；`0x044835`/`0x044d1d`→d37c=27。**SM 不消费**——SM 不能直接寻址 main SRAM `0x8000-0xe000`；main→SM 唯一通道是共享 RAM（main `0xec00-0xecbf`↔`sm_shared_ram[0x00-0xbf]`↔SM `0x0200-0x02bf`，`mcu.cpp:679/929`、`submcu.cpp:218-299`），而 main 写入共享区的全部值只有：8 字节 DSP 系数 `0xb0-0xb7`、标志位 `0xb9`、读 `0x20/0x28`，**无一携带 27/28/15**；SM 侧读共享区也只 0x20-0x2a/0x2b0-0x2ba。→ **这 4 槽位是 main 内部 write-only/参考用，256 patch 无须保 SM 一致性**（B 重写时顺手 256 化或原样皆可）。

**Red herring（非复音界）：** `0x005e20`/`0x005ee7`（效果通道索引 28）、`0x046699`/`0x04669d`（返回值阈值 28/36）、`0x005869`（`trapa #0x1b`）、`0x046656`（init 常量）。

**④ 对 (ii) 的影响 — B 例程集最终版（研究闭合，2026-09-10）：**
- **B 须含重写的例程 = 18（⚠ 2026-09-11 R11 推翻：实为 26 逻辑例程/41 碎片，见 §6.5 R11）**：per-note 8（note-fill `0x0f86`/dispatcher `0x15e0`/handler `0x173e`/slot-select `0x179c`/note-helper `0x17ed`/allocA `0x1823`/allocB `0x187e`/free `0x19c4`）+ mask-acc `0x1ad3` + PCM enable `0x516c`（内含 find-free scan `0x51d9/51fc`，随模块搬）+ PCM write1/2 簇（模块内）+ 表 scan/clear `0x5869` 区（`0x5886/58d7`）+ **pool-init/reset `0x040462`（5 子循环，含 pool BUILD `0x040508`）** + per-voice init `0x041220`（含 0x41239 循环体；另一例程顶 `0x041266`，含 0x41305）+ **32 宽 reset 真顶 `0x04329f`**（32 宽循环 movi 在 `0x4331b/0x43326`）+ **通道建立 `0x041333`**（立即数 28 循环，§6.5 R12 订正）+ DSP search `0x45cc0`（经 `0x45cbe` 边界）+ helper H1 `0x1b44`/H2 `0x1bad`。
- **不入 B（已核 Confirmed）**：`0x04aca9`/`0x043a5`（per-frame per-node）——只触活跃 voice 链表 `0xd68a/0xd68e` + 事件 ring `0xa4d0/0xacd0`（256 深 FIFO，**非** voice-indexed 数组），**与 per-voice 数组 page6 搬迁独立**，无须重定向。（例外：若 patch 日后也搬 `0xd68a` 链表或 `0xa4d0` ring，须同步其常量。）
- **真正工作量 = 重写界表（→ N=255，见 §6.5 R12）+ per-voice 数组 255 宽落 page6 + 闭包 26 例程 + 跨页 pjmp/wrapper + 中断 wrapper**（§6.5 R5/R7/R11）。混宽照旧（`0xd435`/`0xd1ac` 也 255 化）。per-note 循环界**不用改**（自动扩展）。
- **8-bit 界**：`0x045cde` `cmp r0,b #0x1c` → **`#0xff`**（do-while，0..254）；寄存器界 `movi #0x001b` → **`#0x00fe`**（255 次）；a432/a434/a436/d37c 原样。
- **通道建立 `0x041333`**：立即数 28 循环（非 `(br,$3e)` 界，§6.5 R12）→ 改 `#0xfe` + GT 侧 `select_channel` 去 `&0x1f`（§6.5 R10）。

**⑤ 三个 blocker 消项（研究闭合，可进实现）**：
1. ✅ **a432/a434/a436/d37c 非 SM 消费**（Strongly supported，~85-90%）：SM 寻址不到 main SRAM；main→SM 共享区只走 `0xec00-0xecbf`↔`0x0200-0x02bf`，全部值已枚举（8 字节系数 `0xb0-0xb7`/标志 `0xb9`/读 `0x20/0x28`），无一携带 27/28/15。残余风险仅"未执行 main 路径可能读"（低）。
2. ⚠ **`0x04135c` = 立即数 28 循环计数（2026-09-11 订正，旧"死码"作废）**：`58 00 1b`=movi r0,#27；`70 3e`=movs r0→(br,$3e)（`Short_MOVS`=**写**，不覆盖 r0）。→ 256 化改 `#0xfe`（255 次）+ GT `select_channel` 去 `&0x1f`。
3. ✅ **`0x04aca9`/`0x043a5` 不触 per-voice 数组**（Confirmed，全指令枚举）→ 不入 B。

**⑥ 剩余研究缺口 → 2026-09-11 已全部研究（6 子代理并行，全部只读复核）；索引：**
- R5 tp=6 中断安全 → **§6.5 R5**（不安全；B 入口/出口 SR wrapper 方案已定）
- R6 SM 侧容量 → **§6.5 R6**（Closed：SM 无 voice 层，不需 patch；旧「SM 做 DSP」作废）
- R7 A→B 入口/跨页 → **§6.5 R7**（必须 `pjmp #0x0E` + wrapper/蹦床；19 入口字节表）
- R8 B 布局 → **§6.5 R8**（cp=0x0E 换算；page6 表 13,312B）
- R9 256-bit mask → **§6.5 R9**（MHI/PHI + 14×movsw + 提交序）
- R10 GT 规格 → **§6.5 R10**（file:line 全表；`MCU_PatchROM` 实为 `mcu.cpp:1512`，调用 `:2291`）
- R11 闭包 → **§6.5 R11**（26 逻辑/41 碎片；闭包外残留 8 处）
- R12 上限 → **§6.5 R12**（**N=255**；a42d 是计数；`0x04135c` 是真循环）
- R13 DSP 硬编码 → **§6.5 R13**（效果槽外置 `EFF_BASE`）
- R14 取舍 → **§6.5 R14**（建议 (ii)/N=255；(D) 不改 A 时仅到 128）
- 已解除：`a432/a434/a436/d37c` 只写无读且 SM 不消费 → 原样保留。

---

### 6.5 R5-R14 并行研究结果（2026-09-11，6 个独立只读子代理）

**总览**：R5 不安全但可修（SR wrapper）；R6 关闭（SM 无此层）；R7/R8/R9 机制与布局已定；R10 有 file:line 全表；R11 闭包 26 例程/41 碎片；R12 上限 **255**；R13 效果槽外置；R14 建议继续 (ii)。

**R5 tp=6 中断重入安全（Confirmed：当前形态不安全；最小补丁已定）**
- 固件**确有软件中断屏蔽**：`BSET_ORC #0x0700 r0`（SR IML=7；执行集 55 处）与 `BCLR_ANDC #0xf8ff r0`（IML=0；63 处）——**旧"未见软件关中断"作废**；中断入口由硬件置 IML。
- tp 全 ROM 仅 2 次写：`0x16c`/`0x22b` 的 `LDC #0 r7`；无其它。
- B 候选例程本体大多 mask=0（仅 PCM `jsr 0x516c` 段与 `0x54fb/0x564a` 小段被 ORC 包裹）。
- **唯一带 @r6 表访问的 handler = `0x342`（FRT1_OCIA）**（0x0369/0x0378/0x03c5 → 0xFDCD/0xFDE2/0xFE18+idx）；T1 实测其 rte 恢复点落入 B 区间（0x0f86–0x1a40 ×240、0x516c–0x59ff ×95、0x45cc0 区 ×77）。
- **最小补丁（不改 A）**：B 顶层入口 `STC r0 -> --r7`（存 SR，tp 仍 0）→ `BSET_ORC #0x0700 r0`（屏蔽）→ `LDC #6 r7`；出口 `LDC #0 r7` → `LDC r7++ r0`（恢复）。代价：B 全程 IML=7，中断延迟=B 例程时长（与现 `0x1ad3` 同级）。
- **连带（必须同做）**：B 闭包内 `0x5ec/0x15d5/0x1217/0x1f02` 等 `@r6+disp` 访问 **page0 表**（0x05f4/0x1bfe/0x1310）→ tp=6 后自动改页，B 重写须搬表到 page6 或改 dp 基。

**R6 SM 侧（Closed；旧 T3 前提作废）**
- **SM 不是 DSP 合成器**：GT/VM opcode 表无 ADC/SBC/EOR/移位/乘法（全 NotImplemented）；sm_ram 128B（含硬件栈）+ shared 192B 全量引用枚举**无 per-voice 数组、无 27/28 常量**。真实合成 = main SRAM 数组 + GT `pcm.cpp`。
- main 每 note **向 SM 写 0 字节**；SM 仅转发外部 MIDI 字节（≈10KB/s 上限 vs 线速 3.125KB/s）。**255 voice 不影响 SM；SM 无需 patch。**
- 行动：`dasm_annotated.txt` §0/§3「SM core DSP loop / 负责 DSP 合成」为 T3 误判，已加勘误。

**R7 A→B 重定向（Confirmed：必须跨页）**
- **`10 hi lo`（jmp abs）不改 cp** → 到不了 B（**cp=0x0E**）；必须 `13 0E hi lo`（pjmp）或 `03 0E hi lo`（pjsr）。**`rts` 只弹 pc** → 跨页返回需 wrapper+蹦床。
- 19 个 A 入口首指令均容 4B patch（4B 内无分支目标）；真例程顶更正：`0x4329f`（非 0x4331e/323）、`0x41220`/`0x41266`（非 0x41239/0x41305）；`0x5869`=`trapa #0x1b`（handler 0x2ac 不触 tp，安全）。
- 三机制：bsr/jsr 入口 → `pjmp`→B wrapper（`bsr16` body）→`pjmp` A 蹦床（`rts`）；pjsr 入口 → wrapper（`pjsr` body）→`ret`；贯穿/分支入口（0x15e0/0x45cbe/0x5869/0x4329f）直接 pjmp + B 内回跳。H1/H2 无 A 侧重定向（仅 B 内调用）。

**R8 B 块布局（空间充裕）**
- **B 执行页 = cp 0x0E**：GT `mcu.cpp` case 14 → rom2 fileoff `0x60000`（`rom2_mask` 覆盖）；`0x60000-0x6ffff` 空闲 66.9KB。
- 代码量：原 18 例程 ≈2.6KB；含已证缺口 ≈4.2KB；R11 全闭包 6–10KB。
- **per-voice 基址用 r6**（r7=SP）；`@r0-3=dp / @r4-5=ep / @r6-7=tp`。
- page6 数据布局（每数组 256B 对齐；**勘误**：a46c/cdc6/cdfe/cf58/cfac/d000/d054/64d6 为**字**数组=512B；d1ac/d15c/d1a6/d0e0 为字节；补漏 cfac/cfe4）：

| off | 数组 | off | 数组 | off | 数组 | off | 数组 |
|---|---|---|---|---|---|---|---|
| 0000 a368 B | 0100 a384 B | 0200 a3a0 B | 0300 a3bc B |
| 0400 a3d8 B | 0500 a3f4 B | 0600 a410 B | 0700 a46c W |
| 0900 a4aa B | 0A00 a4b4 B | 0B00 acf2 B | 0C00 ad0e B |
| 0D00 ce3f B | 0E00 ce5c B | 0F00 ce78 B | 1000 ce94 B |
| 1100 ceb0 B | 1200 cecc B | 1300 cee8 B | 1400 cf04 B |
| 1500 cf20 B | 1600 cf3c B | 1700 cf90 B | 1800 cfe4 B |
| 1900 d038 B | 1A00 d08c B | 1B00 d0a8 B | 1C00 d0c4 B |
| 1D00 d0e0 B | 1E00 d0fc B | 1F00 d118 B | 2000 d134 B |
| 2100 d15c B | 2200 d1a6 B | 2300 d1ac B | 2400 d435 B |
| 2500 64d6 W | 2700 cdc6 W | 2900 cdfe W | 2B00 cf58 W |
| 2D00 cfac W | 2F00 d000 W | 3100 d054 W | 3300 MHI(32)/3320 PHI(32) |

合计 **0x3400 = 13,312B**；page6 余 ~50KB。

**R9 256-bit voice_enable（设计已定）**
- 低 32bit 原样（`d150/d152` 当前、`d154/d156` 待发 + 2×movsw + `movl` 提交）；高 224bit = page6 `MHI`/`PHI` 各 32B，经 br=0xe8 **14×movsw（$00..$1a）**；**提交（回读 br+0x00）必须晚于 ext 写**。
- 待发置位 `0x546e` 扩展：slot≤15→d156、16–31→d154（原样），≥32→`PHI[(slot-32)>>3]` BSET。
- GT 需 `PCM_WriteExt/ReadExt` + latch 整 32B；**主窗口 reg0 现只写 `data&0xf`（bits24-27）→ voice28-31 置不上**，扩展模式须允许全字节。
- ⚠ VM 差异：`h8vm_main.c` 现把 0xe800 落到 `sram[0x6800]`；新分支必须先加，避免污染。

**R10 GT 256 化规格（要点；file:line 全表在 todo R10）**
- **耦合包**：config/`reg_slots`（**默认公式 `(val&31)+1` 不动；扩展模式 `val+1`**，模式由 `-voices` 标志判定，勿用"第 7 位"判别式）｜mask 三件套+ext+latch｜效果槽外置 EFF_BASE（默认 28；扩展 256）｜select_channel 去掩码 + 效果通道扩展选择（8-bit 通道 vs 4 效果槽不可兼得）｜IRQ 编码（irq_channel≥32 不可直接 OR status bit5）｜**cycles 必须钳 28**（保 `spec.freq≈66207Hz`；用 256 则 ≈7.5kHz 音频饿死）｜快照 `pcm_t` 不兼容。
- **地址漂移**：`MCU_PatchROM` 实际 `mcu.cpp:1512`、调用 `:2291`（旧文档 1482/2228 作废）。
- 实施顺序：数组扩宽→mask 256→ext 窗口→reg_slots/cycles→EFF_BASE→select/IRQ→固件 B；每步默认路径 0-diff。

**R11 B 闭包（Confirmed 26 逻辑/41 碎片）**
- 全量 **222** 个数组访问点；rom1 0x2000–0x4fff 的 **45 处与旧清单逐条一致**。
- 新增外部例程 7 个：`0x25f0/0x272e/0x2d95/0x2e83/0x3615/0x3a9e/0x4de7`。
- 闭包外残留 **8 处/6 例程**：`0x531`（`select_channel&0x1f` 越写 d15c，>31 通道时**高危**）、`0x7cf`、`0x2052`、`0x406e4`、`0x436b5`（d1ac 32 宽）、`0x46621` → 单独立项。
- 勘误：`0x50ee/0x516b` 是 `rts`（返回扇出），非外调。

**R12 上限 255（Confirmed；旧解读作废）**
- 快照实测 `a42c/a42d/a42e/a42f/a430 = 00 1c 00 1b 00`：**a42d=28=池计数、a42f=27=a3d8 链头**、a42c=`a4a8−a42d` 有符号短缺量。旧 §6.1「a42c=28/a42e=27」错位。
- `0xff` 哨兵（20+ 字段）与 slot 255 冲突 + a42d 回绕 → **N=255（slot 0..254）**；若 A 的 BPL/BMI 符号测试不改则仅 **N≤128**（B 重写修）。
- **`0x04135c` 死码结论作废**（T0 复核：`MOVS`=写、`58 00 1b` 是真计数；`0x1362` 起 28 次写 select_channel 0..27）。
- 32 宽 reset 真 movi = `0x04331b/0x043326`；loop E（part 级）16 次。
- 补丁：14 处 `movi #0x1b→#<n-1>`（255 时 `#0xfe`）+ search `0x45cde #0x1c→#n`（255 时 `#0xff`）+ config `c3 <n-1>`（255 时 `c3 fe`）；全部由生成器按 n 印入（清单见 todo R12）。

**R13 DSP 硬编码（并入 R10）**
- `cases 17/18/21/22/23/31` 是**固定效果路由**（rcadd 来自循环外固定块）；reg_slots>32 会让 28–31 被当 voice（总线 `ram1[31][1]/[3]` 互毁、case31 双注入、key-off 清效果态、IRQ 污染）→ **效果槽外置 EFF_BASE**；`:1520` slot2 加 last 门控；cycles 钳 28。

**R14 取舍（建议：继续 (ii)，N=255）**
- (D) 影子映射不改 A → 受 ~21 处符号测试限制，实际 **N≤128**；到 255 需就地扩指令（2B BPL/BMI→4B 比较，多数无空间）→ 侵入度与 (ii) 相当。
- (ii) 成本已量化：闭包 26 例程/8–12KB + 跨页 wrapper + 中断 wrapper + page6 13.3KB + R9 mask。**建议 (ii)/N=255**。

**本轮对既有章节的勘误汇总**：§0/§1（SM 不做 DSP；通道建立立即数循环）、§2.2/§6.1（簿记错位；数组宽度/cfac/cfe4/a3d8=next）、§6.2（r7=SP；tp 需中断 wrapper）、§6.3（B=18→26/41；H1/H2 无 A 侧重定向）、§6.4③⑤（`0x04135c` 非死码；32 宽 movi 地址）、§5（config `c3 fe`）、§9.1（MCU_PatchROM 行号）。

### 6.6 阶段 0 研究：R15-R17（2026-09-11，3 子代理并行）

**R15 PCM 中断通道号（Confirmed；方案已定）**
- **d15c = "PCM IRQ 待处理标志"数组**（非 find-free；旧标签作废）：全 ROM 仅 6 处字面访问（`0x531` 写 0xff；`0x516c/0x553a/0x41257` CLR；`0x51e0 TST`+`0x51ef CLR`，**非零**命中→`0x25f0` 服务）。
- **固件不需要完整声部号**：`0x536 MOVE #2 -> r0` 立刻覆盖；channel 唯一用途 = d15c 下标 → 唯一功能故障是 `AND #0x1f` 截断。
- PCM status 消费者仅 3 处（`0x529` IRQ；`0x41359/0x413b1` 纯 ack）；无人测 bit5；IRQ0 仅接 PCM（GA 走 IRQ1）；IPRA=0x77 → IRQ0 level=7 不嵌套。
- 协议：GT ext `0xe820`=完整 slot（`PCM_ReadExt`）；`status |= irq_channel & 0x1f`（0-diff）；B handler：**先读 ext 再读 status(ack)** → 写 page6 `d15c[slot]` → `MOVE #2/#1` → `pjmp 0:0x3db`。
- 落地二选一：(a-2) `0x529` 原地 4B `13 0E hi lo`→B 续体（22B，最小）；(a-1) IRQ0 向量 `rom1[0x80..0x83]`=`00 0E hi lo`→B 全 handler（29B）。
- 连带：B 的 IRQ dispatcher（`0x51d9` 区）改 page6 全宽扫描（已在闭包）。

**R16 闭包完整性（Confirmed/Strongly：基本闭合，补 d1ac helper 族）**
- 全 ROM 静态扫描（disp16 **+ 立即数基址** `movi/ADD #0xd435/#0xd1ac`）：**d435/d1ac 无 disp16 引用**，必须按立即数扫。
- 新增 15 个"已执行但 222 未计"点（`64d6`/`cfac`/`cfe4` 漏项）——全在既有闭包例程内。
- 69 个"未执行但解码合法"点：~60 在既有例程的**未执行臂**内；**唯一新缺口 = d1ac helper 族 4 入口** `0x43695/0x436a3/0x436c3/0x436d5`（置/清 d1ac bit2/bit1，被 jsr 调用）→ 并入 B。
- 实现要求：**整例程本体搬运**（含未执行臂），不是只搬 trace 片段。

**R17 效果通道（Confirmed；方案改优：PCM 0x3f 别名，不需 0xe800 扩通道）**
- 全量 66 写入点（执行 42）+ 3 status 读；分类 V=16 / E=42（含 6 处"借槽 30 读回"）/ C=8 测试。效果编程簇 `0x5ddc-0x64c3` **不在闭包**（不碰 per-voice 数组）。
- **最小方案**：GT 扩展模式把 **PCM reg 0x3f**（全 ROM 无写、现无分支）定义为效果选择：写 v → `select_channel = EFF_BASE + (v & 3)`；**所有 E 点原地 1 字节 patch `0x3e→0x3f`**（含 6 个读回点），值 0x1c-0x1f 自动映射 0..3；默认模式 0x3f 仍忽略、0x3e 仍 `&0x1f` → 0-diff。
- 勘误：① 数组需 **`[MAX_VOICE+5]`**（EFF_BASE=256..259 → ≥260 项）；② 效果**读回** 6 点必须与写点同改（R17 原只提写）；③ R13 硬编码补 `pcm.cpp:1044`/`:1520`；④ `0x5e20/0x5ee7` = `move r0/r1 #0x1c` 值源，非写入点。

---

## 7. 验证计划

1. **PCM 写追踪**（`-pcmtrace` 已补回，见附）：patch 后跑 `-voices:<n> -demo -mocknote -pcmtrace`，确认固件写 `config_reg_3d`→n-1、`select_channel` 用到 0..(n-1)、voice_enable 覆盖 n bit。
2. **快照校验**：dump `demo_postW.bin` 的 `config_reg_3d`、SRAM note 表（256 项）、`sram` 占用，确认 B 表就位。
3. **音频回归**：
   - 默认（无 flag）：28 复音，输出与原一致（用 VM vs GT diff 确认无回归）。
   - `-voices:255`：播放 ≥255 同时音符（构造 MIDI），确认不丢音、爆音可控；另抽测 1–2 个中间值（如 `-voices:100`）验证参数化。
4. **VM↔GT diff**：`tools/vm`（H8 验证器）跑同输入，比对 PCM 输出，确认 A/B 补丁未破坏 28 复音路径。

---

## 8. 风险

| 风险 | 说明 | 缓解 |
|---|---|---|
| B 块须是原 18 例程的忠实副本 | 重写（256/page6/tp=6）时易偏移/漏指令/错寄存器 | 逐字节核对；用 h8dasm + 动态 trace 锚定 + `tools/verify` Phase A/B 复验 |
| ~15 处界重写漏改 | 漏一处 → 该例程仍按 28/32 迭代，256 静默错 | 界清单已钉死（§6.4 ③ 全表）；实现时逐条对照 + pcmtrace 验证 |
| 混宽数组（28 计数 + 32 宽 `0xd435`/`0xd1ac`） | 两类宽度都要 256 化，易漏 32 宽侧 | §6.4 ③ 单列 2 处 32 界；B 内 `0xd435`/`0xd1ac` 256 宽落 page6 |
| `rom2_mask` 访问掩码 | B 地址须在掩码可达范围，否则取指 0xff | 确认 rom2_read 与 mask；必要时扩 mask |
| voice_enable 256-bit 耦合 | 固件写、PCM 读，位宽须一致 | GT 新寄存器 + ROM 补丁配套设计 |
| CPU 负载 | per-frame DSP 28→256 槽（~9×） | GT 侧可承受；注意采样率/实时性 |
| 原 28 路径回归 | A→B 重定向若误伤默认路径 | 默认不建 B/不重定向；VM↔GT diff 把关 |

---

## 9. h8dasm 可信度验证（2026-09-10，`tools/verify`）

**结论：h8dasm 的「字节偏移 / 指令长度 / 调用边 / 解码字段」对全部已执行代码经 GT 逐条验证 0 失败；§5 全部 patch 落点地址可字节级信任。另钉死 3 处文本层问题（§9.4）与 5 族剩余文本 bug + smdasm 1 处（§9.5，2026-09-11 二次审计），**均已修复并重生成**。**

### 9.1 验证方法（现 `tools/verify/verify_dasm.c`；原 Python 版已于 2026-09-11 迁移，见 §9.5）

基准 = GT 行为（VM trace ≡ GT trace，字节级一致）。
- **Phase A（长度/目标）**：PC trace 相邻两行的 next PC ∈ {s+len，分支目标，vector+首指令长（中断分发），s（sleep 空转）}。**833,332 条指令 0 失败**（555,015 sleep 空转、2,371 中断分发）。
- **Phase B（语义字段）**：全寄存器 trace，逐条核对 寄存器/SP/sr 效应与 GT 一致（检查器按 `src/mcu_opcodes.cpp` 逐指令实现）。**44,213 条 0 失败**（105,397 sleep 跳过、389 中断分发跳过）。
- 覆盖：**9217 执行 PC + 19 向量入口**（= dasm_full 的 PC 集；含 boot、note-on、per-frame DSP 三路 trace 并集）。

### 9.2 polyphony 锚点全部在已验证集内（2026-09-10 逐一核对）

config 写 `0x04134c` ✓ ｜ per-voice 循环 `0x45cc0` ✓ ｜ 28 通道建立 `0x041359/5c/62` ✓ ｜ voice_enable `0x5525/0x5527/0x5662/0x5664` ✓ ｜ 4 调用者 `0x344/0x7b6a/0x7d8a/0x461ac` ✓ ｜ note 填充 `0x1801–0x1a40`（156 PC）✓ ｜ `0x46699` ✓ ｜ `0x7c3f` ✓。（`0x1432` 是数据地址非 PC，不在集内属预期。）

### 9.3 本次验证发现并修复的 h8dasm 解码 bug

| bug | 影响 | 修复（`h8dasm.c`，依 GT） |
|---|---|---|
| `MOVG_Immediate` 尾部立即数未读（GT `mcu_opcodes.cpp:825`）：源操作数 indirect/absolute 且 ore∈{4,5,6,7} 时须再从码流读 1/2 字节 imm | 53 PC / 6,267 次转移；dasm 现成「短指令+伪分支」 | 按 ore 读尾部 imm（4/6→1 字节、5/7→2 字节）并打印 |
| 计数分支 `01/06/07` 按 2 字节解码；GT `MCU_Jump_JMP` 实为 **3 字节**（opcode2 + int8 disp）：`r[reg]--`，未下溢则 `pc+=disp`（06 需 Z、07 需 !Z） | `cntjmp` 类误译 | 3 字节、kind=3 显式目标，Phase A 逐条覆盖 |
| `0x11` 寄存器间接族未细分 | — | 按 GT 拆 `ret`(pop cp,pc) / `jsr via rN:rN+1`（push pc/cp，call 语义）/ `jmp rN` / `jsr rN` |

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

### 9.5 2026-09-11 二次审计：剩余文本 bug（H1-H5）+ smdasm + verify 迁 C

**Phase A/B 的盲区**：只查**结构**（长度/目标）与**寄存器/sr 语义**，不查 h8dasm 打印文本；§9.4 修复时漏掉了 4 族依赖 `ore` 语义的文本。二次审计发现并修复：

| # | bug（旧文本） | GT 事实 | 证据（已执行） | 修复 |
|---|---|---|---|---|
| H1 | ocodes 9/11 恒印 `BSET_ORC`/`BCLR_ANDC` | 仅**立即数**操作数是 ORC/ANDC；否则走 BSET/BCLR（`bit=r[ore]&0xf`，写回操作数） | 24 处，如 `0x3e4 [d2 49]`→`BSET @r2 r1`；`0x547a BSET (dp,0xd154) r1`（voice_enable 待发掩码区） | 条件助记符 |
| H2 | ocodes 24-31 印 `r<ore>` | 位号 = `ore\|((ocode&1)<<3)`；奇数 index 值少 8 | `0x1e45 BTSTI @r2+2 #9 [ea 02 f9]`、`0x3caf BTSTI r2 #15 [aa ff]` | 印 `#bit` |
| H3 | MOVF `0x90-0x9f`（写）与读同文 | 写方向 `r -> @r6+disp` | 基线未执行（潜伏） | 写加 `->` |
| H4 | `ret via rN:rN+1` | GT 为 push pc/cp 后经 r 对跳转=**call**；且奇数 reg 打印对错（`reg+1` 应为 `(reg&~1)+1`） | 基线未执行（潜伏） | 改 `jsr via` + 修 pair |
| H5 | MULXU/DIVXU 字模式不示寄存器对；STC 无方向 | 目的/源 = `r{ore&~1}:r{ore&~1\|1}`；STC=控制寄存器→操作数 | `0x68c MULXU #0xd8 r4:r5` 等已执行 | 印 `rA:rB` / `STC rN -> ea` |

**smdasm**：`0x22` 模板 `"jsr \\{sp}"` 输出多余反斜杠 → 改 `"jsr {sp}"`（值 `0xff00|imm` 本就符合 GT `SM_Opcode_JSR`；基线未解码 0x22，潜伏）。`sm_full.txt`/`sm_disasm.txt` 重生成 **0 diff**。

**验证工具迁 C（`tools/verify/verify_dasm.c`，.py 已删）**：Phase A+B 与 Python 版语义对齐，另增强 ① **向量入口补解码**（`pc_vec.txt`→`mach_vec.txt`：向量目标地址不作为 trace 行 PC 出现，旧版会漏判派发）② **补全 ocodes 9/11/24-31 语义**（Python 版跳过，且会对真实寄存器修改误报）。

**复验（VM trace，boot+快照两段）**：boot 200K instr → Phase A 0 失败（385 派发识别）/ Phase B 0 失败（80,447 条）；demo 快照 400K instr → Phase A 0 失败（1,079 派发识别）/ Phase B 0 失败（115,568 条）。

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

---

## 附2：VM page6 扩展 + 内存探针实测（2026-09-10）

**目的**：把 R1d「扩展后 page6 可读写」从 C 读取升级为 **VM 实测**；顺带钉死 VM 的短指令字节/字语义（探针正确性前置）。

**VM 改动（`tools/vm/h8vm_main.c`，方案 A 验证器侧）**：
- `static uint8_t b_ram[0x10000]`（64KB，方案 A 扩展 RAM，backing page6）。
- `MCU_Read_impl` 加 `case 6: return b_ram[off];`（原 `default: return 0xff`）。
- `MCU_Write` 加 `else if (page==6) b_ram[off]=value;`。
- page6 原 = unbacked（读 0xff / 写丢弃），现零重叠变为真 RAM。**未**加入 `VM_SaveState/LoadState`（保持快照字节兼容）；跨 savesnap 持久化需后补。
- GT（`src/mcu.cpp`）同款改动 = **阶段 1 待做**（见 todo R1b-方案A）。

**探针（`tools/probe/memprobe_gen.c` → `build/probe_rom1.bin`，590 字节码）**：16 页×{0x0000,0x8000} 矩阵 + **page6 专用块**（字节精确 `0x80`/`0x98`，5 偏移 0x0000/0x8000/0xC000/0xFFFE 写不同 sentinel 再读回）。SM 用 `probe_smrom.bin`（`STP` 上电即睡，零噪音）。解码 `memprobe_read.c`。

**结果（`build/probe_p6.snap`）**：
- 矩阵 page6 off0/off1 均 `pre=00 rb=a5` → **WRITABLE**（扩展前 `ff ff` UNBACKED）。
- 其余 15 页**回归不变**：10/11 sram 可写、5/7/12/13 仍 unbacked、ROM 页只读、page0 off0 仍 rom1。
- page6 detail d1–d7 **7/7 PASS**：`(6,0x0000)=A5`、`(6,0x8000)=5A`、回读 `(6,0x0000)` 仍 `A5`（**无别名**）、`(6,0xC000)` `00→3C`、`(6,0xFFFE)` `00→C3` → page6 = 真 64KB 可读写窗口、字节精确、无别名。

**钉死的 VM 短指令语义（`g_verbose` 读日志逐字节核对，T2 验证，供探针/后续 patch 参照）**：
- `0x88/0x90` **是字操作**（bit3 `0x08` = siz）：`0x88` = 字读（`r = Read16`：`b[off]<<8|b[off+1]`）、`0x90` = 写走 `MCU_Write16`（高字节→off、低字节→off+1）。**字节精确**操作用 `0x80`（byte read）/ `0x98`（single-byte write）。
- **字读只清低字节**：`r[reg] &= ~0xff; r[reg] |= data`（GT `mcu_opcodes.cpp:724` / VM `h8vm_body.c:724`）——**保留寄存器原高字节**。故旧矩阵用 `0x90` 写 0xA5 时 r0 高字节泄漏（上一轮读残留），落到目标 off 处；`pre`/`rb` 记录的是低字节 = off+1 处值。**page6 detail 块改用 `0x80/0x98` 规避此坑**，d0 的 0xFF 即矩阵 `0x90` 的高字节泄漏（非 page6 属性）。
- `MOVS [br+d],rN`（`0x70|N`）= 字节精确（`addr=(br<<8)|d`，`MCU_Write(addr, rN&0xff)`），探针 scratch 写入用它，可靠。

**已知弱点（潜在，非现行 bug）**：legacy 16 页 matrix 的可写判定 `WRITABLE iff rb==0xA5` 是单 sentinel——若某**只读 ROM 页**的 off+1 处字节恰为 0xA5，会被误判为可写。当前 rom1/rom2 无此情形（无任何 ROM 页 rb=0xA5，`build/probe_p6.snap` 实测），本轮结论不受影响；**换 ROM 或复用探针时**需加固（第二 sentinel 复核，或 matrix 也改 0x80/0x98 字节精确）。page6 判定不依赖此弱点（detail 块 7 项独立校验）。
