# 复音 256 工程 — Todo List

> 配套：`mk2_polyphony_256.md`（方案 + 证据）、`evidence_protocol.md`（**动手前必读**的分析规则）。
> 状态图例：`[ ]` 未开始 ｜ `[~]` 进行中 ｜ `[x]` 完成 ｜ `[!]` 阻塞/待决策
> 更新：2026-09-11

**总原则**：阶段 0（研究）未定稿前，不进阶段 1/2 实现。每勾一项，在 `mk2_polyphony_256.md` 对应章节补证据（地址/字节/GT 行号），本表只记状态。

**施工规则（2026-09-11，用户指示）**：一旦遇到"明确阻碍"（模型不清、行为无法解释、需要靠二分/试错定位），**立即停止实现，退回研究**：写清问题 → 产出带证据的文档（新 research 文件或补进现有文档）→ 定稿后才恢复实现。禁止在文档不完整时继续盲改。

---

## R21-R23 研究补全（2026-09-11）— 三份权威文档，施工前必读
- `voice_memory_map.md`：**28 个 AoS 语音结构体，每个 0x12a 字节，SRAM [0xad2e,0xcdc6)**；ROM1 `0x64d6` 为 28 项 big-endian 指针表（`P-2`=槽号，v 覆盖 `[P-0x80,P+0xAA)`）。容量 28 的硬原因：槽 28 起于 `0xcdc6` 正好压到 SoA 数组。`r1=0xcc9c@0x3970` = `P(27)-0x80`。
- `voice_bounds_inventory.md`：逐站点语义。**现有界值表错误项**：`0x0404c8`（descriptor-pool 种子，非循环界）、`0x04135c`（PCM select 硬件循环，仅 ≤31 有效）；**缺失项**：`0x045cde cmp r0,b #0x1c`（应=N=0xff）、`0x4331b/0x43326/0x433ed/0x43644`（32 宽循环，N>32 才改）。`0x64d6` 表止于 0x650d（pitch 表紧随）→ 解释了"只放宽界值"时 `r0=0x25ad` 崩溃。
- `task_irq_map.md`：mask 契约（regs0..3=32 位；态 d150/d152+d154/d156；置位 0x546E；提交 0x54FB/0x564A）；GT 扩展窗口 ext 0x00..0x1B→mask 字节 4..31（32..63 走 0xE800+0..3）。**>32 必须改固件 mask 路径**（≤64 逐字节补丁：7 调用点 + 0x7DC4/0x7E12/0x7E63 三例程 + 0x9D80~0x9D87 状态）。验收 oracle：LCDEN≈4.22M、isr≈1382/20M、cfg3d=n-1、有音符驱动的 mask 写、215M 不复位。

### 施工顺序（机械执行，不再二分）
1. 修 `tools/gen_reloc.c` 界值表：删 `0x0404c8`；`0x04135c` 仅 N≤31；加 `0x045cde`=N；`0x4331b/0x43326/0x433ed/0x43644` 仅 N>32。
2. N=32：重定位 32 个 AoS 结构体（stride 0x12a，起 0xad2e）+ 冲突 SoA 数组；建 32 项指针表并补丁所有 `0x64d6` 读取点；config=0x1f。
3. N∈(32,64]：叠加 mask 路径补丁；结构体扩到 N。
4. 4 项 oracle 全过才算完成。

---

## 阶段 0 — 阻塞研究（决定阶段 1/2 怎么做）

**R1【头号】256-slot 可变 per-voice 状态落点 —— 走"扩展模拟器"路线（方案 A）**
> **镜头纠正（2026-09-10）**：模拟器本体我们说了算，目的 = 跑真实 dump ROM + 我们的扩展（同游戏模拟器）。**不把 GT 当前未 backing 的页当"硬件墙"**。GT 的 H8 寻址 `(page<<16)+offset`，但 `MCU_Read/Write` 对 page 取 `&0xf`（**4-bit**）→ **16 页 = 1MB**（非 16MB，我先前说错）。可寻址空间内未 backing 的页 = **5/6/7/12/13（5×64KB=320KB 可扩）**，page6 在其中 → 扩内存**无需新操作码**。
> **事实（Confirmed，GT `mcu.cpp` + VM `h8vm_main.c` 双源 C）**：当前可写 RAM = 32KB SRAM + 1KB ram；page6 现**无 backing**（读 0xff/写丢弃）——是**现状**，非"不可"。256-note 状态 ~15KB 在现有 32KB 内**无连续空区**（见 R1a）→ 需给 H8 **扩内存**。
- [x] R1a 完整 **SRAM 地图**：32KB 中 **110/128 块(0x100)在用**，**最大连续空闲仅 2KB**（窗口 0x9800–0x9f00）；高密度引用（难搬移）：per-voice 核心 `0x2100–0x2d00`、次级 `0x4d00–0x5400`。数据源：`tools/analyze_sram.py`（快照+引用密度）
- [x] R1c **容量决策**：现有 32KB 装不下 256-note(~15KB)+现有数据 → **给模拟 H8 扩内存**（方案 A），非"不可行"
- [ ] **R1b-方案A（扩展内存，选定路线）**：
  - **VM（验证器 `tools/vm/h8vm_main.c`）✅ 2026-09-10**：加 `b_ram[0x10000]`(64KB) + `MCU_Read_impl case 6` / `MCU_Write page6`，page6 由 unbacked 变真 RAM；`memprobe` VM 实测 page6 满窗可读写、无别名（见 R1d + 附2）。**注意：`b_ram` 未加入 `VM_SaveState/LoadState`（保持快照字节兼容），跨 savesnap 持久化需后补。**
  - GT（`src/mcu.cpp`）：**待做（阶段 1）** —— `MCU_Read/Write` 给 **page6** 加同款 case → `b_ram`（与 VM 改动一致）
  - 固件：per-voice 状态改指 **page6**（`0x60000+`）。**⚠ 研究结论（2026-09-10，见下方 R1b-研究 + R1b-(ii)）**：page6 只能经页寄存器（tp）到达；**已定 (ii)** = per-voice 例程整体搬 B（B 内 tp=6）+ 14 处 A→B 重定向，A 零改动。
  - 效果：模拟 H8 = 32KB SRAM + 64KB(page6) 扩展 RAM（要 128KB 再挂 page7），per-voice 状态住 page6
- [ ] R1b-方案B（**备用**，仅当 1MB 内 320KB 扩内存仍不够时才用）：GT/VM opcode 表加自定义宽地址操作码 + patch ROM 用新码。侵入深（改指令流）
- [x] **R1d 经验探针**（**C 实现**，见证据协议 §14）✅ 2026-09-10：H8 探针 ROM（`tools/probe/memprobe_gen.c`）跑 `h8vm.exe … savesnap`，`memprobe_read.c` 解码。实测**扩展前**可写=32KB SRAM+1KB ram（page0 off1 / 10 / 11）、unbacked=page5/6/7/12/13；**扩展后** page6 满窗（0x0000/0x8000/0xC000/0xFFFE）可读写、无别名（`build/probe_p6.snap`，7/7 校验 PASS）。→ 结论由 C 读取升级为 **VM 实测**
- [x] **R1b-研究（基址改点全清单 + b_ram 初值）✅ 2026-09-10**（GT `mcu.h`/`mcu.cpp` + dasm 9217 PC 实证，见 mk2 §6.2）：
  - **寻址事实（Confirmed）**：per-voice 全部 **245 个独立访问点**走 **page=dp**（@r0:16 / @r1:186 / @r2:4 / @r3:11 / `(dp,disp)`≈28；核心 12 + 次级 27 数组，全表 mk2 §6.1）。页寄存器映射（`MCU_GetPageForRegister` mcu.h:208）：**r0-r3→dp、r4-r5→ep、r6-r7→tp**；`@rN+disp16` = `(page<<16)|((rN+disp16)&0xffff)`，**16-bit disp 无法产生 page6 的 bit16** → 到 page6 **必须**经某页寄存器=6。
  - **页寄存器可独占性（Confirmed）**：**dp 不可独占**（188 次 `LDC #0 r5` 全 0x00 防御式 + ~2800 引用，含 §6.4 全局变量表 0xfb80/0xfdc0）；**ep 不可独占**（`LDC #imm r4` 值域 0/1/2/3/4/10 共 36 次，动态页取 rom2 数据表）；**tp 唯一可独占**（仅 1 次写：boot `0x00022b: LDC #0 r7`，之后恒 0；仅 ~20 处 @r6/@r7 用户，均执行、以 tp=0 落 page0，独占时需搬迁到 dp 基）。
  - **b_ram 初值（gap2，定）**：**零初始化**（GT 静态数组天然 0）+ 重写 fill 例程填充 256 slot 初值（位置相关初值如链/恒等/0xff 空标记由重写 fill 计算）。**无需 rom2 静态初值表**。
  - **方向已定 = (ii)（2026-09-10 拍板）**：per-voice 例程**整体搬 B 重写**（256+page6+tp=6），A 零改动（默认回归最安全）。tp=6 只在 B 例程内（自 `LDC #6 r7` 起），A 的 boot `0x00022b` 与 ~20 处 `@r6/@r7`（tp=0）全不动。
- [x] **R1b-(ii) 研究：per-voice 例程集合 + 调用图 + 重定向点 ✅ 2026-09-10**（dasm + flow_main 前驱实证，见 mk2 §6.3）：
  - **⚠ 解读修正（Confirmed）**：dasm 行尾 `;-> XXXXXXXX` **非直接调用者**（是"哪个 IRQ 上下文可达"的 origin label）；**直接调用者 = flow_main 前驱**。
  - **例程集合（§6.3 初判 = 13 逻辑例程 + 2 共享 helper；⚠ §6.4 R3/R4 后扩到 18，最终见 §6.4 ④）**，全单一入口（entry-only）。13 例程顶：`0x0f86`(note-fill)/`0x15e0`(dispatcher)/`0x173e`(note on/off)/`0x179c`(slot-select)/`0x17ed`(note helper)/`0x1823`(alloc A)/`0x187e`(alloc B)/`0x19c4`(free)/`0x1ad3`(mask-acc)/`0x516c`(PCM enable)/`0x54fb`+`0x564a`(PCM write1/2 簇，模块内)/`0x40462`(rom2 init)。helper：`0x1b44`(H1)/`0x1bad`(H2) **必须连同搬**。§6.4 新增：表 scan/clear `0x5869`、pool-init 5 循环 `0x040462`、per-voice init `0x041220`、32 宽 reset `0x0433xx`、通道建立 `0x041333`。
  - **2 个结构例外**：① 搜索循环 `0x45cc0` 是 **fallthrough 进入**（非 jsr，从 `0x45cbe` 掉入）→ 重定向 patch **`0x45cbe` 边界**；② PCM write1/2 簇随 `0x516c` 一起搬。
  - **(ii) 重定向点全清单（§6.3 初判 14 处 → §6.4 扩充 ≈15 A 入口 + `0x45cbe`）**：每处 A 首字节 patch `jmp B_entry`（仅 `-voices≠28` 时）。
  - **⚠ 真正工作量在数据结构非控制流（B 重写核心）**：0x1c/0x1b 循环界→0x100/0xff、64-bit `d150-d156` PCM mask→256-bit（联动 R2 路线 C `0xe800`）、`acf2` 28→256 扩容。
- [ ] **R1b 实现（阶段 1/2，(ii) 已定，研究已闭合）**：GT `mcu.cpp` page6 `b_ram`（零初始化）；ROM patch = 建 B（**最终 18 例程**的 256/page6 重写，清单见 mk2 §6.4 ④：per-note 8 + mask-acc + PCM enable(含 write 簇/find-free) + 表 scan/clear `0x5869` + pool-init `0x040462`(5 循环) + per-voice init `0x041220` + 32 宽 reset `0x0433xx` + 通道建立 `0x041333` + search `0x45cc0` + H1/H2）+ A→B `jmp` 重定向（各例程顶 + `0x45cbe`）+ B 数据 256 slot；数据结构重写（~15 处界 13×28+2×32→256、256-bit mask、acf2、混宽 0xd435/0xd1ac）

**R2 voice_enable 256-bit 位序 + 扩展**（2026-09-10 研究：位序钉死，方案已定：路线 C @ `0xe800`）
- [x] 确认 `pcm.cpp` 读 `voice_mask` 位序（**Confirmed GT T0**：消费侧 `pcm.cpp:1116` `key=(voice_active>>slot)&1` → **bit N=voice N**；写侧 `br+0x00`→bit24-27(4b)、`0x01`→16-23、`0x02`→8-15、`0x03`→0-7；`MCU_Write16` 大端 ⇒ `voice_mask=(r3<<16)|r4`）
- [x] 定扩展方案（**2026-09-10 已定：路线 C**）：第二 PCM 窗口 **`0xe800-0xe83f`（br=0xe8）**，64 字节块（⚠ `0xe400-0xe7ff` 是 GA/LCD/IRQ 设备块非空闲，勿用）；GT `voice_mask`→256-bit。低 32 bit 留主窗口 `br+0x00-0x03`（默认路径不变），高 224 bit（voice32-255，28字节）经 `0xe800`。（备选 B=page6 SRAM 直读，未选；原「8 字节 reg 0x00–0x07」错：256bit=32 字节，且 `br+0x04-0x0F` 撞 ram1）
- [x] trace 写点（**实为 2 簇**，非 4：停用 `0x005525/0x005527`、启用 `0x005662/0x005664`，各 2×`movsw`；`0x005529/0x005666` 的 `movl`(0x60-0x6f) 是**读**非写）；可 patch 每簇 2×movsw→写 32 字节；连带扩 `d150-d156`(4 字节→32 字节)、`a4b4`/`acf2`(28 字节→256 字节)、累积循环 `0x001ada`(28→256，**R4 重叠**)

**R3 分配/释放路径全 trace（清隐藏 28）✅ 2026-09-10**（dasm + flow_main 实证，见 mk2 §6.4）
- [x] 全 trace note→slot/通道选择（`0x15e0–0x1801` + slot-select `0x179c–0x17e6`）：**无 28 循环**——走哨兵 per-channel 链表（头 `+0xa220`/next `+0xa250`）+ pool 栈单 voice 压/弹，**自动扩展到 256**
- [x] 复核 alloc A `0x1823` / alloc B `0x187e` / free `0x19c4` 全量读写：单 voice 操作（r1=1 voice base），触 a3a0/a3bc/a4b4/a3d8/a368/a384/ad0e/d0e0 + pool 计数 a430/a42f/a42d/a42c；**无 28 循环**
- [x] 隐藏 28 界（**推翻 §6.1「只有一处」**）：per-note 路径无界，但 **pool sizing/build/reset/scan/init 有 ~14 处 28 界**（`movi #0x001b`+`cntjmp`=28 次，grep `#0x1c` 漏掉）：`0x040508`(pool BUILD 关键)/`0x040462`(A)/`0x0404d9`(B)/`0x04053b`(D,perch)/`0x040565`(E,perch)/`0x001ad7`(mask-acc)/`0x0051d9`/`0x0051fc`(find-free)/`0x005886`/`0x0058d7`(表 scan/clear)/`0x041239`/`0x041305`(init)/`0x04135c`(**已核**：界=`(br,$3e)` 通道寄存器，`movi #0x1b` 死码) + 已知 `0x045cde` + 2×32 宽 `0x04331e`/`0x043323`

**R4 per-frame DSP 无第二套 per-voice/channel 迭代 ✅ 2026-09-10**（raw 字节 + dasm + flow_main，见 mk2 §6.4）
- [x] **`0x7c3f` 纠错（rollback）**：`rom1[0x7c3f]=11,19` → **`[11 19]`=`ret`**，非 `jmp r1`。"2015/2011 目标"= ret 返回扇出 + origin 计数，**非跳表**。旧文档「jmp r1 间接分派」作废
- [x] 真实 per-frame DSP：`0x07d8a→pjsr 0x414d8→jmp 0x4aca9`（**无跳表**）；`0x4aca9`=活跃 voice **链表遍历**（可变 0..28，非固定界）
- [x] **推翻「除 0x45cc0 + reg_slots 外无其它 28 迭代」**：另有 `0x04123c`(init 28)、`0x040462` reset(5×28)、**`0x04331e`/`0x043323`(2×32 宽 reset，`0xd435`/`0xd1ac` 是 32 元素)**。→ per-voice 数组**混宽**（28 计数 + 32 宽），256 化须全改
- [x] **3 blocker 消项 ✅ 2026-09-10**（⚠ ② 于 2026-09-11 订正，见 mk2 §6.5 R12）：① a432/a434/a436/d37c **非 SM 消费**（main→SM 共享区值全枚举，无 27/28/15；Strong）② ~~`0x04135c` 界=`(br,$3e)`、`movi #0x1b` 死码~~ **作废**：`0x04135c` 是立即数 28 循环计数（`MOVS`=写，不覆盖 r0）→ 改 `#0xfe` + GT select_channel 去 `&0x1f` ③ `0x04aca9`/`0x043a5` **不触 per-voice 数组**（只触 0xd68a 链表 + 0xa4d0 事件 ring）→ **不入 B**

**R5 tp=6 中断重入安全 ✅ 2026-09-11 研究闭合（mk2 §6.5 R5）**
- [x] 固件确有软件关/开中断：`BSET_ORC #0x0700 r0`（IML=7，55 处）/ `BCLR_ANDC #0xf8ff r0`（IML=0，63 处）；旧"未见软件关中断"作废
- [x] 唯一带 @r6 表访问的 handler = `0x342`（FRT1_OCIA）；其 rte 恢复点落入 B 区间 240/95/77 次
- [x] 结论：B(tp=6) 当前不安全；最小补丁（不改 A）= B 顶层入口 `STC r0 -> --r7` + `BSET_ORC #0x0700 r0` + `LDC #6 r7`，出口 `LDC #0 r7` + `LDC r7++ r0`
- [ ] 实现：B wrapper（阶段 2）；B 内 page0 表访问（`0x5ec/0x15d5/0x1217/0x1f02`）搬迁

**R6 SM 侧 ✅ Closed（mk2 §6.5 R6；旧前提作废）**
- [x] SM 非 DSP 合成器（opcode 表无 ADC/SBC/EOR/移位/乘法）；无 per-voice 数组/27/28 常量；main 每 note 向 SM 写 0 字节
- [x] 结论：255 voice 不影响 SM；**SM 无需 patch**
- [x] 文档勘误：dasm_annotated §0/§3「SM 做 DSP」已加更正

**R7 A→B 重定向 ✅ 2026-09-11 机制定（mk2 §6.5 R7）**
- [x] `10 hi lo` 不改 cp → 必须 `13 0E hi lo`（pjmp）跨页；`rts` 不回 cp → wrapper+蹦床
- [x] 19 入口字节表（全部容 4B）；真顶订正 `0x4329f`/`0x41220`/`0x41266`；`0x5869`=trapa（安全）
- [x] 三机制已定：bsr/jsr 入口 → pjmp→wrapper(bsr16 body)→pjmp A 蹦床；pjsr 入口 → wrapper(pjsr body)→ret；贯穿/分支入口直 pjmp+B 内回跳
- [ ] 实现：wrapper/TRAMP（rom1 尾空闲 `0x7dc4+` 放 `rts` 蹦床）

**R8 B 块布局 ✅ 2026-09-11（mk2 §6.5 R8）**
- [x] B 执行页 = **cp=0x0E**（rom2 fileoff `0x60000`）；空间 66.9KB；代码 ~2.6KB(原18)/4.2KB(含已证缺口)/6–10KB(全闭包)
- [x] page6 布局表 13,312B（宽度勘误：a46c/cdc6/cdfe/cf58/cfac/d000/d054/64d6 为**字**；补 cfac/cfe4；a3d8=next）
- [x] per-voice 基址用 r6（r7=SP 不可当索引）
- [ ] 实现：B 代码搬运+重写

**R9 256-bit mask ✅ 2026-09-11 设计定（mk2 §6.5 R9）**
- [x] `MHI`(0x3300)/`PHI`(0x3320) 各 32B；14×movsw 表（`$00..$1a`）；提交（回读 br+0x00）晚于 ext 写
- [x] 待发置位 `0x546e` 扩展；GT 主窗口 reg0 现只写 `data&0xf` → voice28-31 缺口
- [ ] 实现：B write 簇 + GT `PCM_WriteExt/ReadExt`

**R10 GT 256 化 ✅ 2026-09-11 规格产出（mk2 §6.5 R10；file:line 全表见研究产物）**
- [x] 耦合包：config/reg_slots 公式分支（`val<0x80?(val&31)+1:val+1`，0xfe→255）、mask 三件套+ext+latch、EFF_BASE 效果外置、select_channel+效果通道扩展、IRQ 编码、**cycles 钳 28**（保 66207Hz）、快照不兼容
- [x] 地址漂移：`MCU_PatchROM` 实为 `mcu.cpp:1512`、调用 `:2291`（旧 1482/2228 作废）
- [ ] 实现：阶段 1（顺序：数组扩宽→mask→ext→公式→EFF_BASE→select/IRQ）

**R11 闭包 ✅ 2026-09-11（mk2 §6.5 R11）**
- [x] 222 访问点；45 处逐条一致；新外部例程 7 个（`0x25f0/0x272e/0x2d95/0x2e83/0x3615/0x3a9e/0x4de7`）；闭包 **26 逻辑/41 碎片**
- [x] 闭包外残留 8 处/6 例程（`0x531` `select_channel&0x1f` 越写 d15c 高危、`0x7cf/0x2052/0x406e4/0x436b5/0x46621`）→ 单独立项
- [x] 勘误：`0x50ee/0x516b` 是 `rts`（返回扇出）
- [ ] 实现：闭包并入 B（重定向类型 J/C/S 见 mk2 §6.5/R11 表）

**R12 上限 ✅ 2026-09-11：N=255（mk2 §6.5 R12）**
- [x] 快照订正：`a42d=28`=池计数、`a42f=27`=a3d8 链头、`a42c=a4a8−a42d` 短缺量
- [x] `0x04135c` 死码作废（立即数 28 循环）；32 宽 movi = `0x04331b/0x043326`；loop E=16(part)
- [x] 结论：**N=255**（0xff 哨兵 + a42d 回绕）；A 符号测试不改则 N≤128（B 重写修）
- [ ] 实现：14 处 `movi #0x1b→#<n-1>` + `0x45cde #0x1c→#n` + config `c3 <n-1>`（全部由生成器按 n 印入；255 时 `#0xfe`/`#0xff`/`fe`）

**R13 GT DSP 硬编码 ✅ 并入 R10（mk2 §6.5 R13）**
- [x] cases 17/18/21/22/23/31 = 固定效果路由；效果槽外置 EFF_BASE（默认 28→扩展 256）；`:1520` slot2 last 门控；cycles 钳 28

**R14 设计取舍 ✅ 结论：继续 (ii)；容量 255；`-voices:<n>` 28–255 真生效（决策 2026-09-11）**
- [x] (D) 不改 A → 受 ~21 处符号测试限制，实际 N≤128；到 255 需就地扩指令（无空间）→ 侵入度与 (ii) 相当
- [x] (ii) 成本量化：闭包 26 例程/8–12KB + 跨页 wrapper + 中断 wrapper + page6 13.3KB + R9 mask
- [x] **决策**：flag=`-voices:<n>`（单横线冒号，28–255）；B 由 C 生成器按 n 参数化（界/config 印入）；GT `reg_slots` 用模式开关（默认公式不动，扩展 `val+1`）；验证 28/255 必测 + 中间值抽测

**R15 PCM 中断通道号 ✅ 2026-09-11 闭合（mk2 §6.6；方案已定）**
- [x] d15c = **PCM IRQ pending 标志**（非 find-free；旧标签作废）；全 ROM 仅 6 处字面访问；固件不需完整声部号（`0x536 MOVE #2` 覆盖）；唯一故障 = `AND #0x1f` 截断下标
- [x] PCM status 消费者仅 3 处（`0x529`/`0x41359`/`0x413b1`）；IRQ0 仅 PCM（level=7 不嵌套）
- [x] 方案：ext `0xe820`=完整 slot + `status&0x1f`；B handler 先读 ext 再 ack → 写 page6 d15c → `pjmp 0:0x3db`
- [ ] 实现二选一：(a-2) `0x529` 原地 4B pjmp（22B B 续体，推荐最小）｜(a-1) IRQ0 向量 `rom1[0x80]`→B 全 handler（29B）

**R16 闭包完整性 ✅ 2026-09-11 闭合（mk2 §6.6；补 d1ac helper 族）**
- [x] 全 ROM 静态扫描（disp16 + 立即数基址）：`d435/d1ac` **无 disp16 引用**（只能按 `movi/ADD #base` 扫）
- [x] 15 个"已执行漏计"点（`64d6/cfac/cfe4`）全在既有闭包内；69 个"未执行"点 ~60 在既有例程未执行臂内
- [x] **唯一新缺口 = d1ac helper 族 4 入口** `0x43695/0x436a3/0x436c3/0x436d5` → 并入 B
- [ ] 实现要求：**整例程本体搬运**（含未执行臂），不是只搬 trace 片段

**R17 效果通道 ✅ 2026-09-11 闭合（mk2 §6.6；方案改优：0x3f 别名）**
- [x] 全量 66 写入点（执行 42）+ 3 status 读；V=16/E=42（含 6 读回）/C=8；效果簇 `0x5ddc-0x64c3` **不在闭包**
- [x] 方案：GT 扩展模式 reg **0x3f** 写 = `select_channel=EFF_BASE+(v&3)`；**E 点原地 1B patch `3e→3f`**（含 6 读回点），不搬效果簇、不需 0xe800 扩通道
- [x] 勘误：数组需 **`[MAX_VOICE+5]`**；R13 硬编码补 `pcm.cpp:1044/:1520`；`0x5e20/0x5ee7`=值源非写点
- [ ] 实现：生成 0x3f patch 清单（E 点）+ GT 0x3f 分支

**R18 B 内 `@r6/@r7` page0 表访问全扫 ✅ 2026-09-11（mk2 §6.6 补充）**
- [x] 闭包（41 片段 + d1ac 族）内**没有** `@r6/@r7` 访存（会被 tp=6 改页）；全 ROM 的 20 处 `@r6/@r7` 访存全部在闭包外（0x23b-0x1f02：boot/handler/其它例程）→ **无需搬表/镜像**
- [x] `r6` 在闭包内被大量当 temp 用（审计 512 行）→ B 重写策略：**按片段寄存器重映射**（数组索引寄存器 → r6，原 r6 用途按生存期改派到空闲 dp 寄存器）；需带 decode/encode 的 C 重写器 `tools/bgen`
- [x] 数组搬/留决策（修订 R8）：**`d1a6`、`d1ac` 留在 SRAM 不搬**（访问者全在闭包外；d1ac helper 族因此**不进 B**，R16 缺口消项）；`d435`/`64d6` 及 R8 其余数组随 B 搬 page6（R8 布局表中 d1a6/d1ac 两行释放）
- [x] **搬移表最终版**（闭包内 `@rN+disp16` 全量枚举 = 205 地址）：
  - **搬 page6（per-voice）**：`a34c`（**新发现，旧表漏**，字节）、`a368/a384/a3a0/a3bc/a3d8/a3f4/a410`、`a46c(W)`、`a4aa/a4b4/acf2/ad0e`、`cdc6(W)/cdfe(W)/ce3f/ce5c/ce78/ce94/ceb0/cecc/cee8/cf04/cf20/cf3c/cf58(W)/cf90/cfac(W)/cfe4`、`d000(W)/d038/d054(W)/d08c/d0a8/d0c4/d0e0/d0fc/d118/d134/d15c`、`64d6(W)`
  - **搬 page6（per-voice 描述符池，2026-09-11 修正）**：`a250/a26c/a288/a2a4/a2c0/a2dc/a2f8/a314/a330`（loop D 以 28 次初始化、索引走 `a42e` 链 → 随复音数扩到 255；⚠ 旧"a2xx 全留 SRAM"有误）。page6 布局：`0x3500 a250 / 0x3600 a26c / 0x3700 a288 / 0x3800 a2a4 / 0x3900 a2c0 / 0x3a00 a2dc / 0x3b00 a2f8 / 0x3c00 a314 / 0x3d00 a330`（各 256B）
  - **留 SRAM（保持 dp）**：`a040-a240` + `a200/a210/a220/a230/a240`（loop E 以 **16 次**初始化 = per-part）、`a43c/a44c`(per-part 16)、`a1b0`(part)、`a1d0/a1df/a1f0/a1f4`(全局标量)、`abde/ac0e/ac4e`、`dc76-dd7c/de4c/de6c`、`8018`、`ffc5`(设备)
  - **ROM 表（`<0x8000`，必须保持 dp；B 内不得改基址）**：`650e/652e/653a`、`673a/683c/6854/686c/687a/6883/6903/6a03/6a84/6c86-7aee`、`9060-9740`(56×0x20 系数)、`522c/1432/0100/0280/0300/0380/0004/0080-00a8`
  - `d1ac` 属 d1ac helper 族（留 SRAM）→ 该族**不入 B**；`d435` 经 `movi/ADD #imm` 形式（reset）单独处理
  - 布局补：`a34c → page6 0x3340`（256B）；其余按 mk2 §6.5 R8 表
- [x] **重写器 v1 落地（GT 内置 `src/patch_256.cpp`）**：H8 子集 decode + `@rN+disp16`（f0–ff，bit3=size）→ `MOVG2 rX->r6` + page6 disp；相对分支（cntjmp/Bcc/bsr/bsr16）两遍定址修正；入口 wrapper（`bf 98 0c 07 00 48 04 06 8f`）+ 顶层 ret 内联 epilogue；界印 n；pool-init 片段（`0x40462-0x4062b`）已重写、`b_dump.bin` 反汇编核对通过；默认 200-202M trace 逐字节一致
- [ ] 把其余 40 个片段纳入 `kFrags`（含 entry/exit 类型）并逐批启用；`g_all_enabled=1` 后安装/重定向/置 `pcm_ext_active`
- [ ] 特别项：`0x45cbe` 搜索循环、IRQ0 handler（R15）、效果 `3e→3f`（R17）、`d435` 立即数形态、混合基址（r6 被占用的片段需暂存/改派）

**R19 中断屏蔽窗口时序影响（实现时，可实验）**
- [ ] B 全程 IML=7 → FRT tick/MIDI 延迟=例程时长；VM 实验量化，确认固件无时序依赖

**R20 性能与验收（阶段 3 前置）**
- [ ] 模拟器 CPU ×9 实测；≥255 音符 MIDI 测试素材与判定标准；默认 28 的 VM↔GT diff 工具链搭建
- [ ] **GT 运行规范（用户要求，2026-09-11）**：验证/诊断时 **GT 不得无头运行**——必须正常启动（**LCD 窗口 + 声音输出**）+ **超时自动 kill**；用户现场观察协助诊断。禁止 headless/静默模式替代（细节见 plan_256.md §6）

---

## 阶段 1 — GT（src/）实现 ✅ 2026-09-11 完成（默认路径 200M-202M trace 与基准逐字节一致）
- [x] `MAX_VOICE 255` + 效果槽外置（`PCM_SLOTS 260`、`PCM_EFF_BASE 256`；pcm.h）
- [x] 数组扩宽 `ram1/ram2/fstate` = PCM_SLOTS（pcm.h）
- [x] `voice_mask/pending` → `uint8_t[32]` + latch `memcpy` 32B；主窗口 reg0 默认 `&0xf`（扩展模式全字节）
- [x] `mcu.cpp`：`0xe800-0xe83f` → `PCM_WriteExt/ReadExt`（`pcm_ext_active` 守护）；**page6 `b_ram` 64KB**（`pcm_ext_enabled` 守护）
- [x] `reg_slots` 模式开关（active `config+1`；默认 `(config&31)+1`）；**`cycles` 钳 28**；`select_channel` 扩展 0..255 / 库存 28..31→EFF 重映射；**reg 0x3f 别名** `EFF_BASE+(v&3)`
- [x] 效果槽外置（~217 处 `ram1/2[28..31]`→`PCM_EFF_BASE+0..3`；`slot2`/`case` = EFF+3；**新增 `pcm_mod_slot()`**：两处 `&31` 调制源索引映射到 EFF 行——实现时发现的真实差异点）
- [x] status `irq_channel & 0x1f`；`PCM_ReadExt(0x20)`=完整 slot、（0x21）=active
- [x] CLI `-voices:<n>`（28–255 校验，n≠28 置 enabled）；`MCU_PatchROM` 挂点（当前打印 note，`pcm_ext_active` 保持 0 直到 B 补丁落地）
- [x] 快照不兼容记录（`pcm_t` 尺寸变，旧 `demo_postW.bin` 弃用）
- 证据：步骤 1-4 默认 trace 140-150M 逐字节一致；步骤 5-7 默认 trace **200-202M 逐字节一致**；**EFF=256 vs EFF=28 的 200M 快照强等价**（差异仅在效果槽且配对完全相等，eram/accum/rcsum/fstate 零差异）

## 阶段 2 — ROM patch（`MCU_PatchROM` 内，**(ii) 最终版**，清单 mk2 §6.3/§6.4）
- [ ] `0x1432`:`c3 7b`→`c3 <n-1>`（扩展模式 reg_slots=n；GT 模式开关驱动）
- [ ] **B 块**（fileoff `0x60000+`，cp=0x0E）：闭包 **26 逻辑/41 碎片 + d1ac helper 族 4 入口**（§6.5 R11/§6.6 R16；**整例程本体搬运含未执行臂**）255/page6/tp=6 重写；基址寄存器→**r6**；入口/出口 wrapper（`STC r0 -> --r7`+`BSET_ORC #0x0700`+`LDC #6 r7` / `LDC #0`+`LDC r7++ r0`）；跨页返回 wrapper+TRAMP（rom1 尾 `0x7dc4+`）
- [ ] **IRQ0 改造**（§6.6 R15）：(a-2) `0x529` 原地 4B `13 0E hi lo` + B 续体 22B（先读 `0xe820` 再 ack → page6 d15c → `pjmp 0:0x3db`）｜(a-1) 向量 `rom1[0x80]`→B 全 handler；B 内 dispatcher `0x51d9` 区改 page6 全宽扫描
- [ ] **效果 E 点 patch**（§6.6 R17）：全部 E/读回点 `3e→3f` 1B（含 6 读回；清单见 R17 研究产物）
- [ ] **B 内数据结构重写**：界表（14× `movi #0x1b→#<n-1>` + `0x45cde → #n`，按 n 印入；§6.5 R12）、`MHI/PHI` 256-bit mask（§6.5 R9）、`acf2`、B 内 page0 表访问（`@r6+disp`）搬迁
- [ ] **A→B 重定向**（仅 `-voices≠28`）：每入口 4B `13 0E hi lo`（19 点首指令均容 4B；§6.5 R7）+ `0x45cbe` 边界；H1/H2 无 A 侧
- [ ] B 数据：255 slot 初值由 B pool-init 副本计算落 page6（零初始化+fill；无静态初值表）
- [ ] 闭包外残留 8 处/6 例程单独立项（`0x531` 高危等，§6.5 R11）

## 阶段 3 — 验证（§7 + evidence_protocol）
- [ ] **【最高优先】默认回归**：无 flag，28 复音输出与原**逐字节一致**（VM↔GT diff）——防 A→B 误伤默认路径
- [ ] `-voices:255`：构造 ≥255 同时音符 MIDI，不丢音 / 爆音可控；另抽测中间值（如 `-voices:100`）验证参数化
- [ ] `-pcmtrace`：`config_reg_3d→n`、`select_channel` 0..n-1、voice_enable n bit
- [ ] 快照：`config_reg_3d` / B 表就位 / sram 占用
- [ ] **patch 后 ROM 重跑 `tools/verify`（Phase A/B 须 0 失败）**——依协议，patched dasm 同样过 GT 验证

---

## 非阻塞（Unresolved but non-blocking）
- [ ] 次级字段 `0xcdc6–0xd1ac` 语义命名（Inferred 置信，用到再定）
- [ ] note→slot 通道选择精确逻辑（R3 顺带）
- [ ] **探针 legacy matrix 单 sentinel 弱点**：`WRITABLE iff rb==0xA5` 对只读 ROM 页 off+1 处恰为 0xA5 的字节会误判（当前 rom1/rom2 无此情形，见 `mk2_polyphony_256.md` 附2）；**换 ROM 或复用探针时**加固（第二 sentinel 或 matrix 改 0x80/0x98 字节精确）

## 约定
- **GT 运行规范（用户要求，2026-09-11）**：验证/诊断 GT 时 **不得无头运行**——正常启动（**LCD 窗口 + 声音输出**）+ **超时自动 kill**；用户现场观察协助诊断（详见 `plan_256.md` §6）。禁止 headless/静默模式替代。**超时按 24MHz 换算**（`cycles/24e6` 秒 ×≈2，如 200M→20s），不要固定长等待。
- **启动时间窗（用户提示，2026-09-11）**：初次上电 LCD 动画 ≈5s（≈120M cycles）**期间不接受 note/MIDI**；开关电源一次后再开机**无动画**；`-demo` 按键 ≈6s 开始、8s 结束（144M–192M）；**`-demo` 在 200M 仍未开始发演示曲目声音，声音验证取样 ≥300M（≈12.5s，必有 PCM 输出）**。⇒ 状态对比 ≥200M，音频/听感 ≥300M（详见 `plan_256.md` §6）。
- 遇矛盾按 `evidence_protocol.md`：先列假设 → 机械位展开 → 以 ROM+src 为准；VM/dasm/旧文档不作证明。
- 结论用三档置信度（Confirmed / Strongly supported / Inferred），不合并表述。
- **自制工具优先 C**（与 GT/VM 类型零 gap、静态类型；见 `evidence_protocol.md` §14），不用 Python。
- 每阶段结束：更新本表状态 + 同步 `mk2_polyphony_256.md`。
