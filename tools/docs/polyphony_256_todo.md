# 复音 256 工程 — Todo List

> 配套：`mk2_polyphony_256.md`（方案 + 证据）、`evidence_protocol.md`（**动手前必读**的分析规则）。
> 状态图例：`[ ]` 未开始 ｜ `[~]` 进行中 ｜ `[x]` 完成 ｜ `[!]` 阻塞/待决策
> 更新：2026-09-10

**总原则**：阶段 0（研究）未定稿前，不进阶段 1/2 实现。每勾一项，在 `mk2_polyphony_256.md` 对应章节补证据（地址/字节/GT 行号），本表只记状态。

---

## 阶段 0 — 阻塞研究（决定阶段 1/2 怎么做）

**R1【头号】256-slot 可变 per-voice 状态落点 —— 走"扩展模拟器"路线（方案 A）**
> **镜头纠正（2026-09-10）**：模拟器本体我们说了算，目的 = 跑真实 dump ROM + 我们的扩展（同游戏模拟器）。**不把 GT 当前未 backing 的页当"硬件墙"**。GT 的 H8 寻址 `(page<<16)+offset`，但 `MCU_Read/Write` 对 page 取 `&0xf`（**4-bit**）→ **16 页 = 1MB**（非 16MB，我先前说错）。可寻址空间内未 backing 的页 = **5/6/7/12/13（5×64KB=320KB 可扩）**，page6 在其中 → 扩内存**无需新操作码**。
> **事实（Confirmed，GT `mcu.cpp` + VM `h8vm_main.c` 双源 C）**：当前可写 RAM = 32KB SRAM + 1KB ram；page6 现**无 backing**（读 0xff/写丢弃）——是**现状**，非"不可"。256-note 状态 ~15KB 在现有 32KB 内**无连续空区**（见 R1a）→ 需给 H8 **扩内存**。
- [x] R1a 完整 **SRAM 地图**：32KB 中 **110/128 块(0x100)在用**，**最大连续空闲仅 2KB**（窗口 0x9800–0x9f00）；高密度引用（难搬移）：per-voice 核心 `0x2100–0x2d00`、次级 `0x4d00–0x5400`。数据源：`tools/analyze_sram.py`（快照+引用密度）
- [x] R1c **容量决策**：现有 32KB 装不下 256-note(~15KB)+现有数据 → **给模拟 H8 扩内存**（方案 A），非"不可行"
- [ ] **R1b-方案A（扩展内存，选定路线）**：
  - GT：`MCU_Read/Write` 给 **page6** 加 case → `b_ram[128KB]`（page6 现无 backing，零重叠；read/write 都走 b_ram）
  - 固件：per-voice 基址寄存器改 `dp/tp=6`、基址 `0x60000`（H8 原生可寻址 page6，**无需新操作码**）
  - 效果：模拟 H8 = 32KB SRAM + 128KB 扩展 RAM，per-voice 状态住 page6（= 游戏模拟器加内存扩展，干净可审计可回退）
- [ ] R1b-方案B（**备用**，仅当 1MB 内 320KB 扩内存仍不够时才用）：GT/VM opcode 表加自定义宽地址操作码 + patch ROM 用新码。侵入深（改指令流）
- [ ] R1d **经验探针**（**C 实现**，见证据协议 §14）：写 H8 探针 ROM 跑 `h8vm.exe … savesnap`，读快照 `sram[]/rom[]` 实测**扩展前**"可写=32KB+1KB"、**扩展后**"page6 可读写 128KB"，把结论从 C 读取得升级为 VM 实测
- [ ] **R1b 细节待钉**：b_ram 大小（≥15KB，取 64/128KB 对齐）、per-voice 基址寄存器改点全清单（per-voice 循环 + alloc/free/fill 的 `dp/tp` 建立处）、b_ram 初值（是否从 rom2 空闲区载入初值表）

**R2 voice_enable 256-bit 位序 + 扩展**（文档明标"待确认"）
- [ ] 确认 `pcm.cpp` 读 `voice_mask` 位序（br+0x00→voice24-27、0x01→16-23、0x02→8-15、0x03→0-7，及 256 化后扩展段）
- [ ] 定 8 字节方案（reg 0x00–0x07；GT `voice_mask` 256-bit）
- [ ] trace 启用/停用写点 `0x005503–0x005527` / `0x00564a–0x005664`，确认 4 个 `movsw` 簇可 patch 扩到 8

**R3 分配/释放路径全 trace（清隐藏 28）**
- [ ] 全 trace `0x0017a9–0x0017e6`（note→slot/通道选择，**目前仅部分跟踪**）
- [ ] 复核 `0x001823`(alloc) / `0x0019c4`(free) 全量读写
- [ ] 确认分配扫描循环**无第二处 28 界**（寄存器界循环）→ 更新 §6.1 六处分类

**R4 per-frame DSP 无第二套 per-voice/channel 迭代**
- [ ] 枚举 `0x7c3f` 跳表 2011+ 目标，标出每个是否迭代 voice/channel
- [ ] 确认除 C 侧 `reg_slots` 循环(pcm.cpp:1111) + `0x45cc0` 搜索循环外，无其它 28 迭代

---

## 阶段 1 — GT（src/）实现
- [ ] `#define MAX_VOICE 256` 派生 `ram1/ram2/fstate`（pcm.h:25-26,52）
- [ ] `voice_mask` → 256-bit（`uint32_t[MAX_VOICE/32]`）+ 读写 + 位序（依 R2）
- [ ] `reg_slots` 上限去 `&31`（pcm.cpp:536）；`select_channel` 去 `&0x1f`（pcm.cpp:126）
- [ ] （依 R1b）SRAM 布局处理
- [ ] `MCU_PatchROM()`（mcu.cpp:1482，加载后 mcu.cpp:2228 调）：建 B（rom2 fileoff `0x60000+`）+ A→B 重定向 + `rom2_mask` 覆盖
- [ ] CLI `-voices:<n>`（mcu.cpp:1760+ 参数循环；默认 28，上限 MAX_VOICE）

## 阶段 2 — ROM patch（`MCU_PatchROM` 内，§5 七点）
- [ ] `0x1432`:`c3 7b`→`c3 ff`（config_reg_3d→n）
- [ ] `0x04135c` 通道建立扩 n（或跳 B）
- [ ] `0x45cde` 界 → 跳 B 256 循环（推荐，避 8-bit 界）
- [ ] voice_enable 写点 4→8 word（`0x5525/0x5527/0x5662/0x5664`）
- [ ] per-voice 数组基址 → R1b 新布局（核心 12 + `0xad0e` + 次级 `0xcdc6–0xd1ac` 同步）
- [ ] B 码：256 版 per-voice 循环（寄存器/16-bit 界，**忠实复制** `0x45cc0–0x45ce1`，逐字节核对 + h8dasm 复验）
- [ ] B 数据：只读初值表可留 rom2；**可变部分依 R1b 落 SRAM**

## 阶段 3 — 验证（§7 + evidence_protocol）
- [ ] **【最高优先】默认回归**：无 flag，28 复音输出与原**逐字节一致**（VM↔GT diff）——防 A→B 误伤默认路径
- [ ] `-voices:256`：构造 ≥256 同时音符 MIDI，不丢音 / 爆音可控
- [ ] `-pcmtrace`：`config_reg_3d→n`、`select_channel` 0..n-1、voice_enable n bit
- [ ] 快照：`config_reg_3d` / B 表就位 / sram 占用
- [ ] **patch 后 ROM 重跑 `tools/verify`（Phase A/B 须 0 失败）**——依协议，patched dasm 同样过 GT 验证

---

## 非阻塞（Unresolved but non-blocking）
- [ ] 次级字段 `0xcdc6–0xd1ac` 语义命名（Inferred 置信，用到再定）
- [ ] note→slot 通道选择精确逻辑（R3 顺带）

## 约定
- 遇矛盾按 `evidence_protocol.md`：先列假设 → 机械位展开 → 以 ROM+src 为准；VM/dasm/旧文档不作证明。
- 结论用三档置信度（Confirmed / Strongly supported / Inferred），不合并表述。
- **自制工具优先 C**（与 GT/VM 类型零 gap、静态类型；见 `evidence_protocol.md` §14），不用 Python。
- 每阶段结束：更新本表状态 + 同步 `mk2_polyphony_256.md`。
