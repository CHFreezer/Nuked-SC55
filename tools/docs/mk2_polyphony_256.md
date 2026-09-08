# SC-55 mk2 复音数 28 → 256 升级方案

**修改对象（GT / 模拟器）**：Nuked-SC55 `src/`（`pcm.cpp`/`pcm.h`/`mcu.cpp`）——H8/532 固件的执行环境。
**固件（被 patch 目标）**：`rom1.bin`（32 KB）、`rom2.bin`（512 KB，mk2）——Yamaha mk2 主 MCU 固件。
**目标**：复音数从 **28 → 256**，经 CLI flag 开启；**运行时 patch ROM（内存副本）**，不动磁盘 `.bin`。
**日期 / 工具**：2026-09-09 ｜ GT `-pcmtrace` 动态追踪（本次新增，`src/mcu.cpp`）、`h8dasm` 反汇编、raw ROM 字节验证（`build/rom2.bin`）。
**姊妹篇**：`../baselines/dasm_annotated.txt`（固件函数/区域地图）、`scgs_pipeline.md`（PCM 管线）。

---

## 0. 结论速览（先答五个问题）

**Q：为什么不能只改 ROM 上某个常量？**
→ 「28」**不是单一常量**，它同时编码在**码**（per-voice 循环界、28 通道建立、note 表填充）和**数据**（SRAM note 表 28 项）里。且 per-voice 循环界 `cmp #0x1c`（28）是 **8-bit 立即数**，256 = `0x100` 塞不下——**光雕 A（ROM 副本）里的常量改不动结构**。

**Q：方案？**
→ **A/B 内存块**：**A** = ROM 副本（GT 里本就已加载为 `rom1[]`/`rom2[]` 全局数组）；**B** = 追加在 A 空闲区的**自包含扩展块**（256 版 per-voice 循环【码】+ 扩大的 note 表【数据】，用寄存器计数/16-bit 界）；**补丁 = A 的指针/调用点重定向到 B**（只改少数几处，A 本体几乎不雕）；**加载 A+B**，磁盘 ROM 原封不动。

**Q：模拟器（GT）侧怎么改？**
→ 源码在手：`#define MAX_VOICE 256` 兜底 → `ram1/ram2/fstate` 数组、`voice_mask`（256-bit）、`reg_slots`/`select_channel` 位宽**都从它派生**，不出现散落的魔法 256。GT 恒有 256 容量，实际激活数仍由固件驱动。

**Q：精确落点（全部验证过）？**
→ config `0x04134c`（`0x7b`→reg_slots 32）、28 通道建立 `0x04135c`、per-voice 循环界 `0x45cde`（`cmp #0x1c`）、note 表 SRAM `0xa368+`。见 §5。

**Q：挂点现成吗？**
→ **是**。`MCU_PatchROM()`（`mcu.cpp:1482`）已存在、在 ROM 加载后（`mcu.cpp:2228`）被调用，里面已有注释掉的 in-memory patch（含 `rom2[0x1333]`）——**正是为 A/B 方案预留的挂钩**。

**Q：当前 28 的峰值需求？**
→ demo+mocknote 20s 实测峰值激活 **20**（是需求，不是上限）。上限 28 由设计决定。

---

## 1. 复音数如何工作（H8 固件 → PCM）

```
H8 固件（rom1/rom2）
  ├─ note 表（SRAM 0xa368+，28 项）：跟踪活跃音符
  ├─ 分配音符 → PCM slot 0..27   （select_channel 0xe03e，&0x1f）
  ├─ 写 voice_enable 0xe000-03   （28-bit 激活掩码，voice 0..27）
  ├─ 写 config_reg_3d 0xe03d     （reg_slots = (val&31)+1）
  └─ per-voice 循环（rom2 0x45cc0-0x45ce1，28 次）
              │
              ▼
PCM（src/pcm.cpp，PCM_Update）
  └─ for slot in 0..reg_slots-1：key = voice_mask[slot]；跑 DSP；累加 → 立体声输出
```

**三处共同封顶 28**（`config_reg_3d` 的 32 是冗余，见 §2）：
1. 固件 note 表 **28 项** + per-voice 循环 **28 次**。
2. `voice_enable` **28 bit**（0x00 仅 4 bit→voice 24-27，0x01/02/03 各 8 bit）。
3. （硬件）PCM 32 slot / `select_channel` &0x1f / `config_reg_3d` &31 → 上限 32。

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
| `config_reg_3d` | **0x7b** → reg_slots 32 | rom2 flat **`0x04134c`** `0d 3c 90`（`MOVG3 (br,$3c) r0`，r0=`0x7bc3`） | ✅ GT `-pcmtrace` + ROM 字节 |
| `config_reg_3c` | 0xc3 | 同上（16-bit 写 `0x7bc3`） | ✅ |
| voice_enable init | 0（0x00-0x03） | `0x04134f`/`0x041354` | ✅ trace |
| select_channel init | **0x1b→0x00（28 通道）** | `0x04135c` `58 00 1b`（`movi r0 #0x1b`）+ `05 3e 81` | ✅ trace |
| select_channel 运行 | 0..0x1b（偶尔 0x1c-0x1f） | rom1 `0x0000:2ea3`/`5688` | ✅ trace |
| per-voice 循环 | **28 次**，界 `cmp #0x1c` | rom2 flat **`0x45cde`** `40 1c`；循环体读 `@r0+0xa368`/`@r0+0xad0e`，`bne -32` 回 `0x45cc0` | ✅ ROM 字节 |
| note 表 | **28 项 × 5 数组**，间隔 0x1c | SRAM **`0xa368`/`0xa384`/`0xa3a0`/`0xa3bc`/`0xa3d8`** + `0xad0e`（值数组）＝ `sram[0x2368]+` | 🟡 explore+循环引用，尺寸待钉（§6） |

> **关键修正**（vs 纯静态）：`config_reg_3d=0x7b→32`（**不是 28**），故 reg_slots 有冗余；真正绑定是 note 表(28) + voice_enable(28-bit) + select_channel 范围(28)。权威 config 写点在 `0x04134c`（非静态猜的 `0x85d3f`）。

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
  - **数据**：256 项 note 表（5 数组 + 值数组），间隔 0x100。
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

| # | 地址（rom2 flat） | 字节 | 改什么 | 来源 |
|---|---|---|---|---|
| 1 | `0x04134c` | `0d 3c 90` | `MOVG3 (br,$3c) r0`：r0 从 `0x7bc3` → 目标（reg_slots=n、3c 依样） | 动态 |
| 2 | `0x04135c` | `58 00 1b` | `movi r0 #0x1b`（28 通道起点）→ n 版（或跳 B） | 动态 |
| 3 | `0x041359` | `05 3e 81` | `MOVG2 (br,$3e) r1`（select_channel 建立）→ 扩/跳 B | 动态 |
| 4 | `0x45cde` | `40 1c` | per-voice 循环界 `cmp #0x1c`（28）→ 跳 B 循环（推荐，避 8-bit 界） | 动态 |
| 5 | `0xa368+`（SRAM=`sram[0x2368]+`） | — | note 表 28 项 → 256 项（原位扩 或 挪 B） | 静态+循环引用 |
| — | A 调用点（待定位，§6） | — | per-frame DSP → B 循环入口 | 待侦查 |

> ROM 偏移换算：flat `0x0004134c` = rom2 文件偏移 `0x134c`（flat = `0x40000` + 偏移）。

---

## 6. 待侦查（决定 B 怎么摆 —— 落地前必做）

1. **rom2 使用范围 & 空闲区**：mk2 `rom2.bin`=512 KB（`build/rom2.bin` 实测 0x80000），`rom2_mask` 覆盖全部；扫固件最高使用偏移，把 **B 放进尾部空闲区**（并确认 `rom2_mask`/访问掩码可达 B 地址，`mcu.cpp:762` 起）。
2. **SRAM note 表能否原位扩**：dump 快照（`demo_postW.bin`）或加 SRAM 追踪，看 `sram[0x2368]`（=0xa368）之后是否有 ≥256×(项宽) 连续空间；有→原位扩（更省 redirect），无→挪 B。
3. **A 的调用点**：定位 per-frame DSP **调用 per-voice 循环（0x45cc0）的调用点**、note 表**填充代码**（写 0xa368/0xad0e 的地方），做 A→B 重定向。
4. **note 表项宽（1 vs 2 字节）**：per-voice 循环用 `MOVG2`（读 2 字节）但索引 +1——需确认每项 1 还是 2 字节，定 B 表布局与 SRAM 用量。
5. **note 表 28 vs 32**：select_channel 运行到 0x1f 说明能编 32 通道，但 per-voice 循环是 28；确认固件实际音符上限（影响 B 表规模与「28→256」的语义）。

---

## 7. 验证计划

1. **`-pcmtrace`（本次新增，已验证可用）**：patch 后跑 `-voices:256 -demo -mocknote`，确认固件写 `config_reg_3d`→目标 n、`select_channel` 用到 0..(n-1)、voice_enable 覆盖 n bit。
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

## 附：本次新增的追踪插桩（暂留，供后续验证复用）
- `src/mcu.cpp`：`g_pcm_trace`（全局）+ `-pcmtrace` flag + `MCU_Write` 里对 `pcm[0x00-0x03/0x3c/0x3d/0x3e]` 的写日志（带 H8 PC）。
- 证据：`build/pcm_trace.log`（126 KB，demo+mocknote 20 s，3424 行 PCM 控制写）。
