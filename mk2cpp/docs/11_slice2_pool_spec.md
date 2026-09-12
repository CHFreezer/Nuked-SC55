# 11 M4 slice-2 规格：voice pool-init + release/free 原生化

> **暂缓（2026-09-11）**：本文的 N=255 妥协上限口径（`pcm_ext_voices`、slot 255 哨兵、
> 256-bit mask）随 256 扩展回滚**暂缓**（里程碑拆分后归 M5）。`src/hand/native_pool.cpp`
> 当前固定 `kLegacy=28`（`pool_post_reset` 不再读取 `pcm_ext_voices`）；
> pool-init/release/free 的原生语义分析仍有效（stock-28 部分属 M4），
> 重启须基于原版 28 复音。

状态：已评审 v1，2026-09-11。
配套文档：[07 voice 语义](07_m4_voice_spec.md) · [09 hand 覆盖表与集成](09_m4_integration.md) · [10 验收 oracle](10_m4_oracle.md) · [00 计划](00_plan.md)。
口径：目标 = **256 声同时发音**；验收上限 `-voices:255` 是 `0xff` 哨兵 + 8-bit 池计数妥协下的**阶段性上限，非最终目标**（07 §0.5）；
slice-2 限定 **L0**（一次 `MK2CPP_Step` = 恰好一条 H8 指令）；整例程 hook（L1）在 n=28
null 通过前不得启用，且首个 L1 用例只能是全 IML=7 的 `mask_acc 0x1ad3`（09 §4.3）。
基线：HEAD `84d3e51`；L0 首切片 `mk2cpp/src/hand/pcm_enable.cpp` 与 oracle
`mk2cpp/tests/m4_audio_null.py` 已落地并 n=28 null PASS。
地址约定：`PC` 为 flat（`0000xxxx`=rom1，`0004xxxx`=rom2）；`dasm` 行号指
`tools/baselines/dasm_full.txt`；置信度：**C**=ROM+src 证实、**S**=强证据、**I**=推断。

---

## 0. 结论摘要（先读）

1. **Q4/Q5 闭合**：`d1a6` **不是 per-voice**，索引是 `(dp,0xd1a4)&1`（2 项缓冲，C）；`d1ac` 是固定 **32 项**数组，helper 族在已执行路径上只用常量下标 0/4/5/6，未观察到 slot 索引（S，残留下标出处待全量审计）。两者都**不随 N 扩**，256 下不因 N 别名；`d1a6/d1ac` 继续留 SRAM（与 R18 一致）。→ 不是 slice-2 blocker。
2. **Q11 澄清（含 07 勘误）**：`(dp,0xa1e2)` 持有的是**per-part 0x70 字节记录指针**（`0x8048+0x70*part`，表在 **ROM1 0x1bfe**），不是 note 描述符指针。全 ROM 读点 8 处、写点 2 处已枚举（C）。因此需要钉在页 0 的是 **part 记录区 `[0x8048,0x8718)`**，不是描述符池 `a2xx`；`a2xx` 仅按字面基址+索引访问，可原生化/迁 page6。
3. **Q7 闭合**：`0x2f17 TST @r1+0xacf2` 位于例程 **0x2e85（C9，入口蹦床 0x2e83）**，由 **ts_scan 0x5869 经 `0x591c/0x5980 bsr16 0x2e83`** 调用（`dasm:5667,5737`）。acf2 是 per-voice 累积位，写者 mask_acc `0x1ad3`、读者/清者 C9。slice-2 不动 acf2；C9 与 mask_acc 必须成对原生化（记入后续切片约束）。
4. **Q3 闭合**：`0x42b8a`（rom2）清 `P-0x6c`（=S+0x14=REC0.w[10]）与 `P-0x4a`（=S+0x36=REC1.w[10]）以及两个全局 `ce36/ce38`；由 init_params 循环 `0x41315` 带 `r0=P` 调用（C）。**判 H**（per-voice AoS 解引用；n>28 无 SRAM P 可解）。
5. **Q1 闭合**：`0x15d1` 的 `@r6+0x1bfe`（`dasm:2142-2148`）读的是 **ROM1 0x1bfe 的 16 项 per-part 指针表**（tp=0，`0x1bfe<0x8000` → rom1，`src/mcu.cpp:707-709`），不是 SRAM。R18 的 41 碎片闭包从 **0x15e0** 才开始（feasibility §3.1 表），所以 `0x15d1-0x15de` 在闭包外、tp=0、**不矛盾**；R5 旧注中的 `0x15d5` 已被 R18 的最终边界取代（S；41 碎片原始清单已随 `src/patch_256.cpp` 清理出树，属**证据在树外**）。
6. **slice-2 blocker**：B1（`0x1823/0x187e` 命名与语义反向：实为 release）、B2（`a3a0 0x94=free`、`a368`=slot→part）、B3（pool-init 例程内实测 2 次中断 → 必须 L0）、B4（hand 表 256 容量不足）、B5（n>28 不得把 ≥28 的 slot 交回 ROM）、B6/B7（残余研究项）——逐条证据与处置见 §1.6。
7. **本规格推荐路线**：slice-2 用 **L0 表驱动派发（每 PC 一个 descriptor，同一 dispatcher 注册）**，而非整例程 L1；pool-init 必须 L0（可中断），alloc/free 在 L1 资格探针通过前也走 L0。L1 机制可以先行实现，但先在**全 IML=7 的 mask_acc 0x1ad3** 上验证，不拿 slice-2 试刀。

---

## 1. 前置 blocker 清算（逐条，带证据与置信度）

### 1.1 Q4/Q5 — `d1a6`/`d1ac` 是否 per-voice、256 下是否别名

**d1a6：Confirmed，不是 per-voice。**
- 写点 `0x1da4 MOVG3 r0 -> @r1+0xd1a6 [f1 d1 a6 90]`、清点 `0x1ddf CLR @r1+0xd1a6 [f1 d1 a6 13]`（`dasm:3240/3279`）；两处前面的索引计算是
  `MOVG2 (dp,0xd1a4) r1; AND #0x01 r1; EXTU r1`（`0x1d9b-0x1da2`、`0x1dd6-0x1ddd`）。
  ⇒ 数组只有 **2 项**，索引 = `(dp,0xd1a4) & 1`（双缓冲/半区选择），与 slot 无关。
- 另有直接基址清零 `(dp,0xd1a6)`/`(dp,0xd1a7)`/`(dp,0xd1a8)`：`0x406fb/0x406ff/0x40703`（`dasm:12799-12803`）与 `0x46640`（`dasm:16555`），说明它是若干字节的全局小缓冲。
- 结论：`polyphony_256_feasibility.md:83` 的「[I] per-voice」被索引计算直接推翻；N=255 不会把 slot 别名进该数组。

**d1ac：Strongly supported，固定 32 项、非 slot 索引；残留下标出处待审计。**
- 复位：`0x43318 movi r1 #0xd435; 0x4331b movi r0 #0x1f; 0x4331e CLR r1++; 0x43320 cntjmp` 清 `d435[32]`（写-only）；`0x43323 movi r1 #0xd1ac; 0x43326 movi r0 #0x1f; 0x43329 MOVG #4 -> r1++; cntjmp` 填 `d1ac[32]`（`dasm:13864-13876`）。
- 扫描：`0x433ea movi r1 #0xd1cb; 0x433ed movi r0 #0x1f` 对 `d1cb..d1ac` 查 bit2/bit1（`dasm:13994-`）；`0x43641 movi r1 #0xd1cc; 0x43644 movi r0 #0x1f; 0x43647 BTSTI --r1 #0; cntjmp` 扫 32 项 bit0（`dasm:14111-14116`）。
- helper 族（静态反汇编，rom2 fileoff，全部 `EXTU r0` 索引）：
  | 入口 | 行为 | 证据 |
  |---|---|---|
  | `0x43695` | `d1ac[r0] |= 0x04`（置 bit2） | `rom2[0x3695]=a0 12 f0 d1 ac 81 04 04 41 f0 d1 ac 91 19` |
  | `0x436a3` | `movi r0 #0x1f` 循环置全部 32 项 bit2 | `rom2[0x36a3]=58 00 1f ... 01 b8 f2 19` |
  | `0x436b5` | `d1ac[r0] &= 0xfb`（清 bit2） | `dasm:14129-14135`（已执行） |
  | `0x436c3` | 循环清全部 32 项 bit2 | `rom2[0x36c3]=58 00 1f ... 01 b8 f2 19` |
  | `0x436d5` | `BCLR @r0+0xd1ac #1`（清 bit1） | `rom2[0x36d5]=a0 12 f0 d1 ac d1 19` |
- 调用点：已执行 `0x448de/e3/e8`（r0=0/6/5）、`0x4528f/294/29f/2a4/2a6`（r0=6/5）、`0x4587c/881/886/888`（r0=4/6/5）、`0x46960`（`r1=0x0c; MOVG2 @r1+0x694b r0` → 从 ROM1 0x6957 取定值；需按数据核对）。原始字节扫描另捕到 `0x4b6xx/0x4b7xx` 一批 `jsr #0x3695`；其中部分是数据误命中或未解码 r0 出处。
- 结论：**已证实的索引域是 0..6 常量或固定 ROM 值**，数组语义宽度 32；`EXTU` 不会在 32 处自然回绕，理论下标 0..255，但无证据它承载 voice slot。对 >32 的风险 = 某个未执行调用点若传 slot≥32 会写错项（不是 32 别名，而是越界语义）；**列为 B6 残余研究项**（见 §6）。

### 1.2 Q11 — note 描述符池是否留页 0、`(dp,0xa1e2)` 全读点枚举

**`a1e2` 的真实目标：per-part 0x70 记录（Confirmed）。**
- 写点：`0x7e1 MOVG3 r2 -> (dp,0xa1e2) [1d a1 e2 92]`（`dasm:600`）与 `0x9e8`（`dasm:692`），两处前置都是
  `MOVG2 r3 r6; SHLL r6; MOVG2 @r6+0x1bfe r2`（`0x7d9-0x7dd`、`0x9e2-0x9e4`）。
- ROM1 `0x1bfe` 表（`build/rom1.bin[0x1bfe..0x1c1d]`）：`8048 80b8 8128 8198 8208 8278 82e8 8358 83c8 8438 84a8 8518 8588 85f8 8668 86d8`，**16 项 word，stride 0x70**，落 page-0 SRAM `[0x8048,0x8718)`（part 记录区，不是 `a2xx`）。
- `tp=0`（r7 仅 boot `0x22b LDC #0 r7` 写过，之后恒 0）；`0x1bfe<0x8000` 按 `src/mcu.cpp:707-709` 走 rom1。

**全读点枚举（原始 ROM 扫描 `1d a1 e2`，两个 ROM 只 rom1 命中；C）：**

| 类型 | PC | 读取寄存器 | 证据 |
|---|---|---|---|
| W | `0x007e1` | 写 r2 | `dasm:600` |
| W | `0x009e8` | 写 r2 | `dasm:692` |
| R | `0x01034` | r2 | `dasm:1368` |
| R | `0x0104d` | r2 | 仅原始字节（未执行臂） |
| R | `0x01063` | r2 | `dasm:1392` |
| R | `0x01283` | r3 | `dasm:1814` |
| R | `0x01605` | r2 | `dasm:2193` |
| R | `0x0160f` | r2 | `dasm:2202` |
| R | `0x016b1` | r2 | 仅原始字节（未执行臂） |
| R | `0x01a6e` | r2 | 仅原始字节（未执行臂） |

**含义与决策**：
- 旧 07 §4.2 的「note 池须留页 0，因为 part 代码持 `(dp,0xa1e2)` 指针」**前提说反了（07 已勘误）**：该指针指向 part 记录；描述符池 `a250/a26c/a288/a2a4/a2c0/a2dc/a2f8/a314/a330` 只按「字面基址 + 8 位索引」访问（`dasm` 多证据，`voice_memory_map.md` §6）。
- 因此：**part 记录区 `[0x8048,0x8718)` 必须留页 0 SRAM（C，强约束）**；描述符池可在消费者全部替换后原生化/迁 page6（与 R18 把 `a2xx` 搬 page6 的决定一致）。slice-2 只需要产出这条结论，不需要动描述符池的驻留。

### 1.3 Q7 — `acf2` 读者 `0x2f17` 定位

- `0x2f14 MOVG2 @r0+-2 r1`（slot 回写）→ `0x2f17 TST @r1+0xacf2 [f1 ac f2 16]` → `0x2f1b BEQ 0x2fee`；非零则 `0x2f1e CLR @r1+0xacf2`（`dasm:5737-5739`）。
- 所在例程：**0x2e85**（0x2e83 是 `bsr -3 -> 0x2e82(rts)` 蹦床，`dasm:5664/5667`），r0=P 入口，`0x2e85 BSET_ORC` 起 IML=7、`0x2fee BCLR_ANDC`。
- 调用链：ts_scan `0x5869` 内 `0x591c bsr16 -> 0x2e83`（r0=`(dp,0xd17a)` 一个 live P；`dasm:11341`）与 `0x5980`（r0=`(dp,0xd17a)`；`dasm:11412`）；`0x2e85` 也被 `0x2865/0x3001` 路径进入（07 §1.3）。
- 判定：C9 按 07 §3.1 是 **H（保守）**，且在 feasibility §3.1 的碎片表内（`2e83 0x2e83-0x30f0`，13 站点）。acf2 = per-voice「待启动」累积位，写者 `mask_acc 0x1ad3`。
- 对 slice-2 的约束：alloc/free/pool-init 都不碰 acf2；但 **C9（读者）与 mask_acc（写者）必须同一批原生化，或 acf2 保持 SRAM 且 native 写者同步镜像**。列入 §6 依赖顺序（B7）。

### 1.4 Q3 — `0x42b8a` 的 H/R 归类

- 代码（rom2，`dasm:13478-13482`）：
  ```
  0x42b8a CLR @r0+-108    ; P-0x6c = S+0x14 = REC0.w[10]
  0x42b8d CLR @r0+-74     ; P-0x4a = S+0x36 = REC1.w[10]
  0x42b90 CLR (dp,0xce36) ; 全局 word
  0x42b94 CLR (dp,0xce38) ; 全局 word
  0x42b98 rts
  ```
- 调用者：init_params `0x41266` 的 per-voice 循环 `0x41308-0x41318`，`0x4130c r0 = word[0x64d6+2*r1]` 后 `0x41315 bsr16 -> 0x2b8a`（`dasm:13294`）。
- **判 H（S）**：2/4 写点是 AoS 解引用（P-相对）；n>28 不存在 SRAM P。保留为 R 的唯一前提是全时维护 AoS SRAM 代理，而这与「native 权威」目标冲突；该例程仅 5 条指令，原生成本可忽略。
- native 映射：`Voice.rec0.w[10]`、`Voice.rec1.w[10]`（07 §4.1 的 `Rec.w[17]` 覆盖 S+0x00/0x22 起）；`ce36/ce38` 为全局 word，与 `ce3f` 数组相邻但**不是 per-voice**。

### 1.5 Q1 — `0x15d1 @r6+0x1bfe` 与 R18 矛盾复核

- 直接证据：`0x15d1 MOVG2 r3 r6 [ab 86]; 0x15d3 SHLL r6; 0x15d5 MOVG2 @r6+0x1bfe r2 [fe 1b fe 82]`（`dasm:2142-2148`，已执行）。
- `tp=0` 恒定（boot `0x22b` 后无写）；`(tp<<16)|((r6+0x1bfe)&0xffff) = 0x1bfe+2*part`，落在 **ROM1**。
- `0x1bfe` 表 = 16 个 per-part 指针（见 §1.2），供 `0x7e1/0x9e8/0x15d5` 三处共用同一 `MOVG2 r3 r6; SHLL r6; MOVG2 @r6+0x1bfe` 模式——**这是一个 page0 ROM 表访问**，不是 SRAM 表。
- R18 的闭包范围：feasibility §3.1 的碎片表在该地址区间只有 `dispatcher 0x15e0-0x15fb`；`0x15d1-0x15de` 不在其中。B 重定向点在 `0x15e0`（分支/贯穿入口），`0x15d1` 段在 A 执行、tp=0，故 R18 的「闭包内无 @r6/@r7 访存」成立。R5 旧注列出 `0x15d5` 属早期粗闭包，已被 R18 取代。
- 置信：**S**（碎片表 + dasm + 表字节；41 碎片原始清单在 `src/patch_256.cpp` 清理后不在树内，属唯一残留证据缺口）。
- 对 M4：native `note_disp` 直接 `MCU_Read16(0x1bfe + 2*part)` 即可（flat 0x1bfe 走 rom1，`src/mcu.cpp:707-709`）。

### 1.6 slice-2 前置 blocker 清单

| # | blocker | 证据 | 处置 |
|---|---|---|---|
| B1 | `0x1823/0x187e` 命名与语义反向（release，不是 alloc） | `d0e0=4` → jump table `rom1[0x522c+4]=0x5238 pcm_stop`（raw `52 32 52 cb 52 38`；`dasm:10686-10693`）；`a42d++`（`0x186c/0x18bc`）对 `pool_pop` 的 `a42d--`（`0x19bf`） | 07 已勘误（2026-09-11）；规格按下文语义实现 |
| B2 | `a3a0 0x94=free`（07 写 0x94=allocated）；`a368=part`（07 写 note index） | `0xf86 CLR a3a0`；`0x19c4 BTSTI a3a0 #7` 早退；`0x45cc2 CMP a368[i] r4`（r4=part） | 07 已勘误；规格按修正值（`a384`=slot→desc） |
| B3 | L1 资格：pool-init 例程内实测 2 次中断 | `trace_boot3m_base.txt:28651`（前 `04:04b4`@c322068）、`:30656`（前 `04:0624`@c346128） | slice-2 走 L0；L1 先在 `0x1ad3` 验证 |
| B4 | `MK2CPP_HAND_MAX=256` 不够 L0 全覆盖 | `mk2cpp.cpp:31,103-108`；pool-init 实测 127 个 PC + 其余片段 ≈300+ | 扩容（≥1024） |
| B5 | n>28 时 ROM 消费者（pool_pop/SoA/描述符）不能见 slot≥28 | `src/mcu_opcodes.cpp:571-580`（disp16 无 page 位）；`voice_memory_map.md` §8 | slice-2 保持「legacy 窗口 0..27 仍由 SRAM 真值」；native 扩展位 dormant |
| B6 | `d1ac` 未执行调用点的 r0 出处 | §1.1 表；`0x46960` 从 ROM1 取定值 | 插桩/审计；不影响 slice-2 |
| B7 | `acf2`（C9/mask_acc）与 S 区字段的成对性 | §1.3 | 后续切片顺序约束 |

---

## 2. 数据模型定稿

### 2.1 双轨原则（slice-2 版）

- **SRAM 页 0 数组仍是互操作真值（interop truth）**：slice-2 结束时，凡未被 native 替换的 ROM/gen PC 仍按字面基址+索引读写页 0；因此 n=28 下所有写入必须在**同一条指令**（L0）或**同一 block 出口**（L1）落成与 stock 逐字节相同的 SRAM。
- **native `Pool` 是权威副本 + 扩展位容器**：为 n>28 保存 slot/desc ≥28 的状态；n=28 时它与 SRAM 镜像完全一致。
- **提交时机**：L0 = 每指令立即双写；L1 block = 入口 import（可能被 ROM 改过的字段：free scalars + chain）→ 执行 → 出口 commit（本 block 触及字段）。禁止延迟到 step 结束才提交（未替换 ROM 代码在 block 返回后立即可能读）。
- **n>28 安全阀（B5）**：slice-2 期间 ROM 的 `pool_pop 0x19ad`/`desc_setup 0x18ce` 未替换，**native 不得把 ≥28 的 slot 放回 ROM 可弹出位置**。做法：pool_init 的 SRAM 镜像链只写 0..27（stock 结构），native 扩展链单独持有 28..N-1 且 `pool.legacy_limit=28`；诊断可查 native 全链。

### 2.2 native 结构（slice-2 子集；字段→H8）

```cpp
namespace mk2c {

constexpr int kVoicesMax = 256;   // 容量；slot 255 = 0xff 哨兵，活性 255（妥协上限）
constexpr int kLegacy    = 28;    // 页 0 镜像窗口（stock ROM 可见域）

struct Pool {
    uint8_t  voices;              // N = pcm_ext_voices（28..255），PostReset 捕获
    uint8_t  legacy_limit;        // slice-2 固定 28；ROM pool_pop 可见上限
    // --- free 栈/链（a3d8 + 5 个标量）---
    uint8_t  free_next[kVoicesMax]; // a3d8；0xff=尾
    uint8_t  free_head;             // a42f：pop 端（0x19ad 取 head）
    uint8_t  free_tail;             // a430：push 端（0x184a/0x19fa 追加）
    uint8_t  count;                 // a42d：free 计数（pool_init=N；pop--；push++）
    int8_t   shortfall;             // a42c：a4a8-a42d 有符号短缺

    struct Slot {                   // 每个 0x12a 结构的 SoA 视图（slice-2 用到的）
        uint8_t  flags;             // a3a0：0x94=free（bit7=1）；note_fill 分配时清 0
        uint8_t  pcm_ch;            // ad0e：0xff=无
        uint8_t  st_a3bc;           // a3bc
        uint8_t  active;            // a4b4
        uint8_t  cmd;               // d0e0：0/2/4（4=stop，2=play）
        uint8_t  irq_pend;          // d15c
        uint8_t  part;              // a368（修正：part，不是 note index）
        uint8_t  desc;              // a384（描述符索引）
        uint8_t  prev, next;        // a3f4/a410（desc 内 voice 链；语义按 dasm 原样复刻）
        uint8_t  start_prev, start_next; // d0a8/d0c4（并行链）
        uint8_t  ce5c, ce78, ce94, d118, d134, ceb0, cf20, cf3c,
                 cf90, d0fc, ce3f;  // loop A 字节初值（含义多未证，按偏移命名）
        uint16_t cfac, d000, d054, a46c; // loop A word 初值
    } v[kVoicesMax];

    struct Desc {                   // 描述符池（a2xx）
        uint8_t  next, prev;        // a250 / a26c
        uint8_t  state;             // a288（0x94）
        uint8_t  vhead, vtail;      // a2c0 / a2dc
        uint8_t  part_age;          // a314
        uint8_t  f2f8;              // a2f8（desc_setup 写 a4a5；语义 I）
    } d[kVoicesMax];
};

} // namespace mk2c
```

**修正标注（相对 07 §4.1）：**
- `a3a0`：`0x94` 是 **free/idle**（bit7=1）；native 名用 `flags`，注释写 `// 0x94=free`。
- `a368`：`slot→part`；`a384`：`slot→desc`。
- `a2c0/a2dc`：desc 内 voice 链的首/尾指针（`link 0x194c` 追加到 a2dc，`H1 0x1b44` 摘除并改 a2c0/a2dc）。
- `a34c` 的索引域在 loop A 是 slot、在 `free_voice` 是 desc（`0x19e1/0x19eb`）→ **语义未定，slice-2 留 SRAM，不进 native**（I）。

### 2.3 数组归属表（slice-2）

| 类别 | 数组 | 归属 | 依据 |
|---|---|---|---|
| free 池 | `a3d8/a42f/a430/a42d/a42c` | native 权威 + SRAM 镜像（0..27） | §3.1/§3.2；n>28 扩展位仅 native |
| slot 状态 | `a3a0/ad0e/a3bc/a4b4/d0e0/d15c` | native + 镜像 | loop A/B、release/free |
| 链 | `a3f4/a410/d0a8/d0c4/a368/a384` | native + 镜像 | loop B、link/H1/H2 |
| 描述符池 | `a250/a26c/a288/a314/a2c0/a2dc/a2f8` | native + 镜像 | loop D、H2（`a2f8` 语义 I） |
| loop A 其余字节/字 | `ce5c/ce78/ce94/d118/d134/ceb0/cf20/cf3c/cf90/d0fc/ce3f` + `cfac/d000/d054/a46c` | native + 镜像（语义 TODO） | `0x40469-0x404c2` |
| part 记录 | `[0x8048,0x8718)` 16×0x70 | **留 SRAM** | `a1e2` 指针（§1.2） |
| part 表 | `a010..a240`、`a040/a050/a060/a070/a080/a190/a1b0/a1d0/a1f0/a1f4/a43c/a44c/a200/a210/a220/a230/a240/a432/a434/a436` | **留 SRAM** | loop E/helper；16 项，无容量问题 |
| 全局 | `ce36/ce38`（0x42b8a） | native 标量 + 镜像 | §1.4 |
| 小缓冲 | `d1a6[2]`、`d1ac[32]`、`d435[32]` | **留 SRAM** | §1.1 |
| 掩码 | `d150..d157`、`acf2`、`a4b4` | `a4b4` native+镜像；`d150..d157/acf2` 留 SRAM 到对应切片 | 首切片已处理 flush PCs；acf2 见 B7 |
| PCM/AoS 深水 | AoS `[0xad2e,0xcdc6)`、SoA 语义未证组 | 留 SRAM（n=28）；n>28 由后续切片接管 | `voice_memory_map.md` §9 |
| PCM 设备 | `0xE000..` | GT `PCM_Write/WriteExt`（不复制） | 08 §4 |

### 2.4 n 参数入口

- **复用 `extern int pcm_ext_voices;`（不新增全局）**：`MK2CPP_PostReset()`（`mk2cpp.cpp:279`，已在 `PCM_Reset()` 后由 `src/mcu.cpp:3069` 调用）里一次性：
  `g_pool.voices = (uint8_t)pcm_ext_voices;` 断言 28..255；`-mk2cpp` 无 `-voices` 时为 28。
- 理由：`-voices` 已经是 GT/PCM 与 native 的共同容量来源，双全局会漂移；`pcm_ext_active` 仍按 09 §3.2 只在 n≠28 且原生引擎能驱动时置 1（slice-2 不置）。
- 只读约束：hand 不得写 `pcm_ext_*`；`MK2CPP_PostReset` 是唯一捕获点。

---

## 3. 逐例程重写规格（伪代码级）

> 所有访问通过 GT `MCU_Read/Write(M,M16)`（页 0 flat `0xdxxx` 直通 SRAM，PCM 窗保留 trace），禁止直连未导出的 `sram[]`（`src/mcu.cpp:657` 为 static）。
> 下表伪代码是 `dasm_full.txt` 的**逐指令转写**，不重构语义；`native` 表示同时更新 `Pool`。

### 3.1 pool_init `0x40462-0x4062a`（入口 pjsr from `0x565`）

- 调用：`0x565 pjsr #0x04:0462`（`dasm:693`）；返回 `0x40586 ret` → `0x569 BCLR_ANDC`。
- 实测：一次 boot 调用共 **2868 条指令**（`trace_boot3m_base.txt` 行 28028-30927 计 2900 行，减去 32 条 FRT2 handler 行），**127 个唯一 PC**；期间 2 次中断（c=322068 前 `04:04b4`；c=346128 前 `04:0624`）。
- 5 个循环 + 1 个 bsr helper（`0x40588-0x4062a`）：

**A. per-slot 清零（`0x40469-0x404c2`，r1=27..0，body 23 条）**
```
for r1 = N-1 down to 0:            // stock 0x1b；镜像时只跑 27..0
  r3=0xff: ce5c[r1]=ce78[r1]=ce94[r1]=d118[r1]=d134[r1]=ceb0[r1]=cf20[r1]=cf90[r1]=d0fc[r1]=d0a8[r1]=d0c4[r1]=0xff
  cf3c[r1]=0x3c
  r2=0:  ad0e[r1]=ce3f[r1]=d0e0[r1]=a4b4[r1]=a34c[r1]=0
  cfac[2*r1]=d000[2*r1]=d054[2*r1]=a46c[2*r1]=0
```
顺序证据：`dasm:12635-12656`（写序即上表；word 数组先 `SHLL r1` 两次再 `SHLR`）。

**B. 哨兵（`0x404dc-0x404ef`，r1=27..0，body 4 条）**
```
a3f4[r1]=0xff; a410[r1]=0xff; a3bc[r1]=0; a3d8[r1]=0xff
```

**C. free 链构建（`0x40508-0x40536`，r1=27..0）**
```
a1f0=0; a1df=0; a42f=0xff; a430=0xff; a42d=0
for r1 = N-1 down to 0:
  r0 = a430
  if (r0 >= 0) { a3d8[r0] = r1; } else { a42f = r1; a3d8[r1] = 0xff; }
  a430 = r1; a3a0[r1] = 0x94; a42d++
a42e = 0xff
```
⇒ 完成后链 `a42f=N-1 → … → 0`，`a430=0`，`a42d=N`。0x404c8 处（`movi r2 #0x1b`）不是循环，只做 `r3=0x0f; a432=0x0f; a434=0x1b; a436=0x1b` 与把 r2 种子交给 loop D（`dasm:12658-12666`；`voice_bounds_inventory.md` §4 的 MISCLASSIFIED 已记录）。

**D. 描述符池（`0x4053b-0x40562`，r2=27..0）**
```
for r2 = 27 down to 0:
  r0 = a42e; a250[r2] = r0; a42e = r2
  a288[r2] = 0x94; a26c[r2]=0xff; a314[r2]=0xff; a2c0[r2]=0xff; a2dc[r2]=0xff
```
n>28 时 native 版本应循环 N 次（R18 将 `a2xx` 定 255 宽）；slice-2 的 SRAM 镜像仍只写 28 项。

**E. part 表（`0x40565-0x40581`，r3=15..0，per-part 16）**
```
a210[r3]=0; a220[r3]=0xff; a230[r3]=0xff; BCLR a240[r3].0; a200[r3]=0x00ff
```
（`BPL` 终止 ⇒ 16 次；PART-16，不随 N。）

**F. helper `0x40588`（per-part 16；`dasm:12728-12788`）** —— 逐指令转写（只影响 part 数组，留 SRAM）：
```
r3=15..0:
  r4 = word[abde + 2*r3]; a1b0[2*r3] = r4
  if (r4 < 0xe0) { r4=1; a1f4=1 } else { r4=2; a1f4=2; r4-=0xe0 }
  a43c[r3] = a1f4
  a44c[2*r3] = low((uint32)r4 * 0xd8)        // MULXU #0xd8
  r5 = low((uint32)r4 * 0xd8)
  r0 = (byte[r5+13] & 1) ? 2 : 0; a040[r3]=r0; a080[r3]=0x40; a190[2*r3]=0x3c3c
然后 r3=15..0:
  a060[r3]=0; a050[r3]=0xff; a070[r3]=0x3c
  r0 = (r3<<4) + 0x9f50; 8×CLR --r0
  r2(0xa090 base) 起 16 字节 = 0xff
```

**n=28 位等价要求点**：A/B/C/D/E/F 的写序、值、索引缩放（word 数组 ×2）、`cntjmp` 下溢终止（28 次）、`a1f4` 分支、`MULXU` 16 位回绕（`ADD #0` 只取低 word）全部逐字节一致；中断不在 block 内被吞（L0）。
**PC 停止点与中断**：**整例程每个指令边界都可能被中断**（§1.6 B3 实证 2 次）；若用 L1 block，必须把 block 边界放在 5 个循环头/迭代末，让宿主在边界轮询（见 §4.3）。

---

### 3.2 release A `0x1823-0x187d` / release B `0x187e-0x18cd`

> 名字勘误见 B1：这是**释放一条 voice 并压回 free 栈**；调用者 note_helper/aux_alloc（`0x17ed/0x180d`）沿描述符的 voice 链循环调用，直到 `a2dc[desc] < 0`。

**release A（`dasm:2524-2575`）：**
```
entry: r1=slot, r2=desc, r3=part, r5=next-desc
0x1823 BSET_ORC #0x0700            // IML=7
0x1827 stm #0x3e; 0x1829 jsr 0x516c (pcm_start, §3.4)
0x182c ldm #0x3e
0x182e d0e0[slot] = 4              // → dispatcher 走 pcm_stop
0x1833 BCLR_ANDC #0xf8ff           // IML=0, ex_ignore=1
0x1837 EXTU r1
0x1839 ad0e[slot] = 0
0x183d a3bc[slot] = 0
0x1841 a4b4[slot] = 0
0x1845 r0=0; 0x1847 bsr H1(slot,desc)
0x184a..0x1863 push slot to free tail:
     r0=a430; if (r0>=0) a3d8[r0]=slot else a42f=slot; a3d8[slot]=0xff; a430=slot
0x1867 a3a0[slot]=0x94
0x186c a42d++
0x1870 bsr H2(slot,desc,part)
0x1873 a42c--; if <0 a42c=0
```
**release B（`dasm:2577-2623`）** 同构，差异：`0x187e EXTU r1` 在 ORC 前；IML 窗 `0x1880-0x1890`；push 用 free **head 端**判定：
```
0x18a5 r0=a42f; if (r0>=0) a3d8[slot]=r0 else a430=slot; a42f=slot
```
其余（`d0e0=4`，清 ad0e/a3bc/a4b4，H1/H2，a42d++，a42c--）相同。

**副作用/寄存器清单（两例程一致）**：入口 r1/r2/r3 由调用者给；出口不返回值（r0 被 H2 用）；clobber r0/r3（H1 保存恢复 r3）；写 PCM 设备 `select_channel`、reg 0x16/0x18（pcm_start 内，走 `MCU_Write`，`-pcmtrace` 记录）；断言 `pcm_start` 的两分支（0x12 vs 0x14）由所选通道 reg0x32/0x34 比较决定。

**中断窗口**：
- `0x1823-0x1833`（A）/`0x187e-0x1890`（B）为 IML=7，block 内不轮询在该窗内**无观察差**；
- 尾段（A: `0x1837-0x187d`，约 15 条；B: `0x1894-0x18cd`）无保护 → 若整段 L1，中断最多延迟 ~15×12=180 cycles（相对 FRT2 周期 ~14.5k cycles 约 1.2%）；风险低但**违反 09 §4.3 的硬规则**。
- 入口 IML 未证（调用链 note_helper 来自 note 事件路径，未见 SR 证据）→ **探针项**（§5.4）。

---

### 3.3 cleanup free `0x19c4-0x1a23`（`dasm:2801-2866`）

```
entry: r1 = slot
0x19c4 if (a3a0[slot] & 0x80) rts        // 已 free，防重复
0x19ca if (ad0e[slot] == 0xff) rts       // 未分配通道
0x19d1 r2 = a384[slot]                    // desc
0x19d5 r3 = a368[slot]                    // part（修正）
0x19d9 a3bc[slot]=0; 0x19dd a4b4[slot]=0
0x19e1 if (a2c0[desc] == slot) BCLR a34c[desc].1
0x19eb if (a2dc[desc] == slot) BCLR a34c[desc].0
0x19f5 r0=0; 0x19f7 bsr H1(slot,desc)
0x19fa..0x1a13 push slot to free tail (0x184a 同构)
0x1a17 a3a0[slot]=0x94
0x1a1c a42d++
0x1a20 bsr H2(slot,desc,part)
0x1a23 rts
```
- 无 IML 包裹；调用者 `0x587 bsr16 -> 0x19c4`（cleanup 任务，与 pool-init 同一 `0x55c` 上下文）。
- 与 release A/B 的差别：不收 `d0e0=4`、不调 pcm_start；desc/part 从 `a384/a368` 反查；维护 `a34c` 两位。
- **`a34c` 语义未定**（slot/desc 混用）→ 留 SRAM，native 不做权威（I）。

---

### 3.4 子例程（native 同 block 内联或独立 L0）

**`pcm_start 0x516c-0x51b5`（21 条，`dasm:10604-10618`）**
```
d15c[slot]=0
MCU_Write(0xe03e, slot)                    // select_channel（保持 pcmtrace）
r4 = MCU_Read(0xe032); r4 = MCU_Read(0xe03a)   // read_latch 组合
r5 = MCU_Read(0xe034); r5 = MCU_Read(0xe03a)
r2 = word[0x64d6 + 2*slot]                 // P
if (r4 >= r5):  { r6=0x14; MCU_Write(0xe018,0xb5); P+30=0xb5 }   // S+0x6e
else:           { r6=0x12; MCU_Write(0xe016,0xb5); P+26=0xb5 }   // S+0x6a
P+0=P+2=P+4 = r6
```
（`CMP r4 r5; BCC 0x519e`（`dasm:10610-10612`）在 GT 语义下 C=r4<r5、BCC 取 r4≥r5 支；r4/r5 是读 latch 的 4/8 位组合，需照 `MCU_Read` 语义。n>28 时 `P` 不存在 → native 直接写 `Slot` 的 pcm 镜像字段。`select_channel` 的扩展解耦见 08 R2/F5。）

**`H1 0x1b44-0x1b8f`（21 条，`dasm:2970-3038`）** —— 原样转写：
```
push r3; r3=0; r3=a3f4[r1]
if (r3 >= 0) { a2c0[r2]=r3; a410[r3]=0xff; d0a8[r3]=0xff; a3f4[r1]=0xff; d0c4[r1]=0xff }
else { r3=a410[r1];
       if (r3 >= 0) { a410[r1]=0xff; d0a8[r1]=0xff; a3f4[r3]=0xff; d0c4[r3]=0xff }
       else { a2c0[r2]=0xff } }
if (r3 >= 0 分支汇合) a2dc[r2]=r3
pop r3
```
（第一分支是否覆盖 `a2dc` 见 `dasm:1b89` 汇合点；按 listing 原样实现，不做语义优化。）

**`H2 0x1bad-0x1bfd`（26 条，`dasm:3040-3076`）**
```
if (a2c0[desc] >= 0) goto tail
bsr unlink(desc)                            // §下方
a250[desc]=a42e; a42e=desc; a288[desc]=0x94
if (a200[part] < 0) goto tail
if (a200[part] != desc) goto tail
r0=0x7f
for (d=a220[part]; d>=0; d=a250[d]):
    if (a314[d] > r0) { r0=a314[d]; a200[part]=d }
tail: a210[part]--
```
（比较为无符号 `BLS` 跳过；保留 `0x7f` 初值。`a210` = part 内活跃 voice 计数。）

**`unlink 0x1b23-0x1b43`（11 条，`dasm:2941-2967`）**：沿 `a250/a26c` 与 `a220/a230` 双链摘除 desc（按 listing 原样）。

**`pool_pop 0x19ad-0x19c3`（7 条，`dasm:2784-2799`）**
```
r1 = a42f; r0 = a3d8[r1]; a42f = r0
if (r0 < 0) a430 = r0
a42d--
rts                                  // 返回 r1 = slot
```
（slice-2 仍 ROM；native 版在后续切片启用，且必须做 B5 的 legacy 上限。）

**`H3 0x1b90-0x1bac`（9 条，`dasm:3015-3038`）**：清 `a288/a2a4`，按 `a314` 维护 `a200[part]`（desc_setup 用，后续切片）。

---

## 4. L1 接口需求（`mk2cpp_hand_routine_fn`）

### 4.1 签名与宿主协议

```c
/* mk2cpp.h 拟新增（与 09 §4.3 一致） */
typedef uint32_t (*mk2cpp_hand_routine_fn)(void);   /* 返回本次执行 H8 指令数 n>=1 */
void MK2CPP_HandRegisterRoutine(uint32_t flat, mk2cpp_hand_routine_fn fn);
```

宿主 `MK2CPP_Step` 命中 routine 时：
```c
uint32_t n = rfn();
if (n == 0) { fprintf(stderr, "mk2cpp: hand routine %08x returned 0\n", flat); exit(1); }
mcu.cycles += 12u * (n - 1u);   /* 宿主随后统一 +12，合计 12n */
he->hits++; mk2cpp_hand_hits++;
mk2cpp_trace_check();           /* T 检查仍在块尾一次（与 gen 现状一致） */
```
- **n 定义**：stock 在该 PC 起会执行的**本块内** H8 指令数（含动态分支实际走的臂、含内联子例程；不含块内本应发生的中断 handler 指令）。返回值随每次调用可变。
- **cycles 验证点**：任何 checkpoint 的 `hash.mcu/mcu.cycles` 必须与 stock 一致；建议加 debug 断言 `old_cycles + 12n == new_cycles` 且每条目记录 n 的 min/max。
- **寄存器/栈**：块出口的 `mcu.r[0..7]`、栈内容、`mcu.pc/cp`、`mcu.sr`（含 IML）、`mcu.ex_ignore` 必须与 stock 同边界一致。
- **ex_ignore 硬约束**：块内出现 `ORC/ANDC/LDC` 时 stock 会在**该指令后的第一次宿主轮询**消耗 `ex_ignore`；L1 块内没有轮询。规则：块内每个虚拟轮询点必须自行 `ex_ignore=0` 模拟消耗，且块结束时的 `ex_ignore` 必须等于“stock 从块尾到块出口之间还会发生的轮询消耗后的值”。简化做法：**含 ORC/ANDC/LDC 的例程不整体 L1，而在这些 SR 边界拆块**（release A/B 即 `0x1823..0x1833` 与 `0x1837..0x187d` 两块）。

### 4.2 L1 资格门（建议写成硬规则）

**L1-eligible ⇔** 入口 IML=7 ∧ 块内无 `ORC/ANDC/LDC/RTE` ∧ 所有内联子例程同条件 ∧ 入口/出口栈帧与 stock 一致。
- 满足者：`mask_acc 0x1ad3`（`0x1ad3 BSET_ORC` … `0x1aed BCLR_ANDC`，无内部 SR 写，`dasm:2893-2906`）。
- slice-2 三例程均**不满足**：pool-init 实测可中断；release A/B 的 IML 窗不覆盖尾段；free 无保护且入口 IML 未证。→ **slice-2 不使用整体 L1**。

### 4.3 若坚持 L1 的替代形态（记录，不推荐）

- **L1-block（按循环迭代/子例程拆块）**：在 5 个循环头（`0x40469/0x404dc/0x4050d/0x4053b/0x40565`）与 helper 循环头（`0x4058b/0x405fa`）注册 block hook，每 block 返回本迭代条数；宿主在 block 间正常轮询，中断由硬件保存/恢复寄存器后 `rte` 回块头 PC，重新进入 block。**不需要边界回调**，是 L1 里最接近 stock 的形态；代价是 trace 只保留块首行。
- **边界回调（09 §4.3 逃生口）最小需求**（仅当整例程 L1 且例程可重入时）：
  ```c
  /* 语义 = 宿主每步轮询（src/mcu.cpp:1895-1898）：ex_ignore? 清 0 : MCU_Interrupt_Handle() */
  /* 返回 1 = 已开始一次中断/异常（mcu.pc/cp 指向向量），0 = 无 */
  int MK2CPP_HandBoundary(void);
  ```
  约束：不推进 cycles、不调设备；回调不能 unwind C++，例程必须在**可恢复的 ROM PC**（循环头）检查返回并立即收尾；若返回 1，例程不得再写 `pc/cp`，未完成状态必须落在 SRAM（native 侧无持久化则不可重入）。pool-init 的循环计数器在寄存器里、由中断帧保护，理论可行；实现复杂度高，本期不用。

### 4.4 slice-2 的推荐接口形态：L0 表驱动

- 每个 PC 注册同一个 `void l0_dispatch(void)`；`l0_dispatch` 用 `mcu.pc` 查本切片 descriptor 表，执行**恰好一条**指令（含操作数字节消耗、标志、pc/cp）。
- descriptor 需要按数组语义路由：`{base, width, elem, native_field}`；slot/desc < legacy 时 SRAM 写 + native 写，≥legacy 时仅 native（B5 安全阀在 allocator 侧）。
- 好处：cycles/中断/trace/`ex_ignore`/pcmtrace 逐字节保持（n=28 gate 全可用）；与 gen 完全同契约，不需要 §4.1/§4.2 的新协议。
- 容量：`MK2CPP_HAND_MAX` 由 256 提到 ≥1024（slice-2 约 300 条目；后续切片会继续增长，建议 4096 并保留注册期重复检测 `mk2cpp.cpp:98-102`）。

---

## 5. 验收计划

### 5.1 n=28 null（必过 gate；在现有 `m4_audio_null.py` 上加强）

命令与现状一致（`-mk2 -mk2cpp -voices:28 -demo -wav: -audiowin 300M 320M -audiohash 320M -hashdump 320M`；可选 `-HandOff`）。

| # | 判据 | 现状 | slice-2 目标 |
|---|---|---|---|
| 1 | WAV payload bit-exact（sha256）+ `-audiohash` 相等 | PASS | 保持 |
| 2 | **完整 state hash 逐行相等**（不只标量；含 `hash.sram/hash.pcm/hash.mcu`） | 已实测 stock==m4（`mk2cpp/out/m4/audio_null/*.state.hash` 无 diff） | 保持；这是 pool 镜像 byte-exact 的主判据 |
| 3 | `-mk2cpp-hand:0` A/B 与 hand-on 字节一致 | PASS | 保持 |
| 4 | `-pcmtrace`（若用 `-mocknote 144M`）逐行一致（含 cyc/pc 列） | 现切片 PASS | L0 下必须继续保持；release 路径的 `pcm_start` 设备写会出现在 note 事件窗口 |
| 5 | `-tracepc` boot 0–3M 与 `[200M,202M)` 对冻结 baseline 0 分歧 | two_mode_check 可用 | L0 下保持；**L1 后失效**（§5.3） |
| 6 | hand 命中可证：pool-init 条目命中 ≥1（boot c≈314580-349392），release/free 在 demo 声音段命中 >0 | 无 per-entry 打印 | 需加 `MK2CPP_DumpStats`（09 §2.6）或 `-mk2cpp-pool` 诊断 |
| 7 | 无 stall/复位：LCDEN 0 ≤2、`isr`/`snapinfo` 增长、320M 时 `pc` 正常 | PASS | 保持 |

### 5.2 n=64 冒烟（非 gate；明确不完整）

- 命令：`-mk2 -mk2cpp -voices:64 -demo -wav:... -audiowin 300M 320M -snapinfo <c> <f> -hashdump ...`（现场 + 超时 kill）。
- 预期：
  - boot 完成，pool 诊断可查 `voices=64, count=64, native chain 63→0`；
  - SRAM 镜像链仍只含 0..27（stock 结构），ROM `pool_pop` 只会在 0..27 内分配 → **听感与 n=28 相同（最多 28 声）**；
  - `cfg3d` 仍为 0x7b（slice-2 不置 `pcm_ext_active`，`-voices` 只开 page6 backing/reg 数）。方案 A 已定案（2026-09-11）：扩展模式 voice 数取 `pcm_ext_voices`、config 恒 stock `0x7b`，O4/S4 新期望 `cfg3d=7b` + `ext_voices=n`（见 `out/m4/12_cfg3d_voice_count.md` §4.3）；`isr` 增长，无 `00:037A` stall。
- **明确不覆盖**：slot≥28 的分配/发声/IRQ/mask；native 扩展链无游戏内消费者。此冒烟只证明「native pool 能按 N 初始化且不回归」。

### 5.3 trace/hash 可用性矩阵

| 模式 | `-tracepc` | `-pcmtrace` | 全 state hash | 音频 |
|---|---|---|---|---|
| L0（slice-2 推荐） | 可用（逐行 0-diff） | 可用（含 cyc/pc） | 可用（byte-exact） | 可用（gate） |
| L1-block/整例程 | **失效**（块内行丢失，行数/相位变） | **pc/cyc 列失真**（内层设备写只报块入口 pc） | 仅当 cycles/中断相位逐位一致才可用；否则只比标量 | 可用（权威 gate） |
| n>28 任意 | 只对未扩展路径可比 | ext 写可读（G2 已实现） | `hash.sram` 只覆盖 legacy 窗口；native 扩展位不可见 | 冒烟非 gate |

### 5.4 L1 资格探针（若将来要 L1 化 slice-2；一次性、现场）

1. **SR 探针**：在 hand 侧临时注册 L0 shim 在 `0x40462/0x1823/0x187e/0x19c4` 记录 `mcu.sr`（IML）与 `mcu.ex_ignore` 到文件（或加临时 `-regtrace`）；确认入口 IML。
2. **中断落点**：`-mocknote 144000000 -tracepc <f> 144000000 146000000`（或 demo `[300M,320M]`）抓含向量（`00:0344/00:0524/00:0342`）的 trace，检查 release/free 区间内是否发生中断。
3. 若两测均显示「入口 IML=7 且区间无中断」→ 允许 L1；否则保持 L0。

---

## 6. 风险与未决项

| # | 项 | 类型 | 处置 |
|---|---|---|---|
| R1 | B1/B2 命名/位义错位若照旧文档实现 | 设计 | 07 已勘误（2026-09-11）；voice_memory_map、09 §5.1 待同步 |
| R2 | pool-init 可中断（实证） | 实现 | L0 或 L1-block；禁止整例程 L1 |
| R3 | release A/B 尾段与 free 的入口 IML 未证 | 实现 | §5.4 探针后再定 L1；默认 L0 |
| R4 | `MK2CPP_HAND_MAX`/注册结构容量 | 集成 | 提到 ≥1024；查表 O(log n) 不变 |
| R5 | native 扩展位在 slot≥28 无消费者、快照不可见 | 验收 | slice-2 只做自检 + n=64 冒烟；不声称 n>28 功能 |
| R6 | `a34c/cf04/cf20/cf58/cfe4/...` 语义未证 | 数据 | 保留 `f_<offset>` 命名、留 SRAM；只做逐指令镜像 |
| R7 | `d1ac` 未执行调用点下标出处 | 研究 | B6：全量审计 `jsr #0x36xx` 调用点 r0 来源（含 0x4b6xx 数据误命中排除） |
| R8 | 41 碎片原始清单不在树内（`src/patch_256.cpp` 被清理） | 证据 | Q1 结论以 feasibility §3.1 表 + dasm 为准；如需硬证据从 git 历史/备份取 |
| R9 | 中断相位/周期：L1 下 `12*(n-1)` 只补条数，不补中断边界 | 时序 | L0 下无此问题；L1 用 §4.2 门 |
| R10 | `a2f8`（desc_setup 写 `(dp,0xa4a5)`）、`H2` 的 `a314>0x7f` 选择语义 | 数据 | 按 listing 转写，不命名；单测对拍 |

---

## 7. 实现工作量估计（slice-2，L0 表驱动路线）

| 工作项 | 内容 | 估量 |
|---|---|---|
| L0 派发基础设施 | `l0_dispatch` + descriptor 表 + 数组路由（native/SRAM 双写）；hand 表扩容 | ~0.5–1 人日 |
| `Pool`/`Slot`/`Desc` + import/commit | §2.2/§2.3；`MK2CPP_PostReset` 捕获 N | ~0.5–1 人日 |
| pool_init（5 循环 + helper，127 已执行 PC + 未执行臂） | §3.1 转写 + 逐字节对拍 | ~1.5–2 人日 |
| release A/B + free + pool_pop/H1/H2/unlink/H3（~160 PC） | §3.2–3.4 转写；`pcm_start` 设备写走 `MCU_Write` | ~2–3 人日 |
| 诊断 | per-entry hits + pool dump（`-mk2cpp-pool`/`MK2CPP_DumpStats`） | ~0.5 人日 |
| 验收 | n=28 null（全 hash+音频+pcmtrace+A/B）、n=64 冒烟、探针运行（现场） | ~0.5–1 人日 |
| **合计** | | **~5.5–8.5 人日** |

L1 机制若同期开发（host 协议 + mask_acc 验证）另计 ~1–1.5 人日；把它算进 slice-2 会显著增加中断语义风险，建议独立排期。

---

## 附录 A — 证据索引

- pool-init：`dasm:12630-12788`（`0x40462` 入口、`0x404c8` 种子、loop B/C/D/E、helper）；boot 行数与中断：`trace_boot3m_base.txt:28027-30927`（2868 条，中断点 `:28651`/`:30656`）。
- release A/B/free：`dasm:2524-2575 / 2577-2623 / 2801-2866`；H1/H2/unlink/pool_pop/H3：`dasm:2970/3040/2941/2784/3015`。
- `d0e0` 命令表：`rom1[0x522c..0x5233] = 52 32 52 cb 52 38`；`dasm:10686-10693,10747`。
- `parse a1e2`：`dasm:600/692/1368/1392/1814/2193/2202`；raw scan 追加 `0x104d/0x16b1/0x1a6e`；表 `build/rom1.bin[0x1bfe..0x1c1d]`。
- d1a6/d1ac：`dasm:3240/3279`；`dasm:13866-13876/13994-14116/14129-14135`；rom2 fileoff `0x3695/0x36a3/0x36c3/0x36d5`。
- acf2：`dasm:5667/5737`；写者 `dasm:2893-2906`；调用 `dasm:11341/11412`。
- 0x42b8a：`dasm:13478-13482`；调用 `dasm:13294`。
- 0x15d1：`dasm:2142-2148`；R18：`polyphony_256_todo.md:149-158`；碎片表 `polyphony_256_feasibility.md:198-228`。
- 地址模型：`src/mcu.cpp:657`（sram static）、`:707-709`（page0 <0x8000→rom1）、`src/mcu.h:208-215`（页寄存器）；`src/mcu_opcodes.cpp:571-580`（disp16）、`:887/911/1000`（ex_ignore）、`:779-800`（MOVS）。
- 表容量：`mk2cpp/src/mk2cpp.cpp:31,64-78,103-115`；T 检查：`:136-140`；PostReset 占位：`:279-284`。
- 验收：`mk2cpp/docs/10_m4_oracle.md` §2；`mk2cpp/tests/m4_audio_null.py:41-69`；现状 hash：`mk2cpp/out/m4/audio_null/*.state.hash`（stock==m4 无 diff）。

## 附录 B — 对既有文档的勘误（变更记录）

以下条目已于 2026-09-11 并入 [07](07_m4_voice_spec.md) 与索引；`voice_memory_map.md`、09 §5.1 待同步。

| 文档 | 原文 | 更正 | 证据 |
|---|---|---|---|
| 07 §4.1/§2.3 | `a3a0 0x94=allocated` | 0x94=**free**（bit7=1）；分配（note_fill）清 0 | `0xf86`, `0x19c4` |
| 07 §2.3 | `a368`=slot→note index A | `a368`=**slot→part**；`a384`=slot→desc | `0x45cc2` vs `r4=part`；`link 0x19a4/19a8` |
| 07 §4.2/R7 | note 池须留页 0（因 a1e2 指它） | a1e2 指 **per-part 0x70 记录 0x8048+n*0x70**；描述符池无指针持有 | §1.2 |
| 07 §1.2 B13/B14 | alloc A/B | 语义为 **release A/B**（`d0e0=4`→pcm_stop，`a42d++`）；分配器 = `pool_pop 0x19ad` + `link 0x194c` | `0x522c` 表、`0x186c` |
| 09 §5.1/§5.5 | alloc/free `0x1823/0x187e/0x19c4` | 0x1823/0x187e 为 **release A/B**；分配器 = `pool_pop` + `link` | 11 §1.6 B1 |
| feasibility §1.3 | d1a6 [I] per-voice | 索引=`(dp,0xd1a4)&1`，2 项 | §1.1 |
| R5 注（todo:82） | B 闭包含 `0x15d5` | 最终闭包从 `0x15e0` 起；`0x15d5` 在 A、tp=0 | §1.5 |
