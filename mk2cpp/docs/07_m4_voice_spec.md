# 07 M4 voice 子系统语义规格

> **暂缓（2026-09-11）**：本文描述的 256 复音扩展（`pcm_ext_*`/`-voices:`/0xE800 窗口/
> `PCM_MAX_VOICE`/效果槽外置）已从 `src/` 回滚到原版 28 复音实现。本文的 voice 语义
> 分析（slot/哨兵/pool 闭包）仍有效，但涉及 N>28、256-bit mask、`0xE800`、page6 的
> 设计**暂缓（里程碑拆分后归 M5，见 `00_plan.md` §2）**；不依赖扩展的 stock-28 语义
> 部分仍属 M4，重启时须基于原版 28 复音重新评估。

状态：已评审 v1，2026-09-11（勘误更新：slice-2 规格 [11](11_slice2_pool_spec.md) 修正
§0.4/§0.10/§1.2/§2.3/§2.4/§3.1/§3.2/§4.1/§4.2 的命名与归属，逐条见 §6）。
配套文档：[08 PCM 引擎与音频路径](08_m4_pcm_api.md) · [09 hand 覆盖表与集成](09_m4_integration.md) · [10 验收 oracle](10_m4_oracle.md) · [11 slice-2 pool 规格](11_slice2_pool_spec.md) · [00 计划](00_plan.md)。
阅读顺序：00_plan §M4 → **07（voice 语义）** → 08（PCM/音频）→ 09（集成/分派）→ 10（验收）。
口径：目标 = **256 声同时发音**；验收上限 `-voices:255` 是 `0xff` 哨兵 + 8-bit 池计数
妥协下的**阶段性上限，非最终目标**（§0.5）；
前期限定 L0（一次 `MK2CPP_Step` = 恰好一条 H8 指令），整例程 hook（L1）在 n=28 null 通过前不得启用（09 §4.3）。

范围：`mk2cpp/src/hand/` 原生 voice/PCM 子系统替换 `mk2cpp/src/gen/` 对应块。
行号口径：评审基线 HEAD `84d3e51`；引用 GT/mk2cpp 行号在实现推进后可能漂移。
只读来源：`tools/docs/`（voice_memory_map / voice_bounds_inventory / task_irq_map /
mk2_polyphony_256 / polyphony_256_feasibility / polyphony_256_todo）、
`tools/baselines/dasm_full.txt`（行号）、GT `src/`（行号）、mk2cpp 架构（README/01/02/03）。

置信度标记按 `tools/docs/evidence_protocol.md` §12：**C**=ROM+src 证实；
**S**=强证据（GT+多站点一致）；**I**=推断。本文所有未标 C 的语义命名都是待验证假设。

地址约定：PC `0000xxxx`=rom1 flat，`0004xxxx`=rom2 flat（=fileoff+0x40000）；
`sram[a] = sram[a & 0x7fff]`（`src/mcu.cpp:703,737,968` 页 0 映射）；
快照文件偏移 `sizeof(mcu_t)+RAM_SIZE`（`src/mcu.cpp:161-163,1229-1230`）。

---

## 0. 结论摘要（先读）

1. **可替换粒度**：mk2cpp 分派是「一 PC 一步」（`src/mcu.cpp:1451-1455`，
   `mk2cpp/src/mk2cpp.cpp:57-80`）。M4 前期锁定 **L0**：每个 hand 函数与 gen 相同，
   必须恰好执行一条 H8 指令并写回 `mcu.pc/cp`。§1 的例程/区间表是**设计单元与
   语义闭包视图**，不是"入口一次执行 N 条"的 hook；整例程 L1 的启用条件见 09 §4.3。
   L1 段内不轮询中断的时序风险见 §5 R1。
2. **必须原生的判定标准**：凡是「以 slot/note 索引访问 per-voice 数组」或「解引用
   AoS P 指针」的例程，在 N>28 时都无法由 ROM 解释器执行（8-bit disp16 到不了 page6，
   `src/mcu_opcodes.cpp:571-580`）。等价于 `mk2_polyphony_256.md` §6.5 R11/R18 的闭包
   （26 逻辑例程/41 碎片）+ 本文 §1.4 的搜索循环 + §1.5 的 IRQ0。
3. **可保留 ROM 的部分**：MIDI/UART 输入、part/patch/参数状态（`a1xx` 为主）、
   效果编程簇 `0x5ddc-0x64c3`、音色/系数 ROM 表、LCD/定时器/SM/事件队列、
   `0x4aca9` SM 队列消费者。前提是 §3.2 的镜像字段在边界上可见。
4. **状态双轨**：建议 **Voice（AoS+SoA）原生**，**part 记录区 `[0x8048,0x8718)` 留在
   页 0 SRAM**——part 代码通过 `(dp,0xa1e2)` 持有的是 per-part 0x70 记录指针
   （表 ROM1 `0x1bfe`，见 11 §1.2），**不是 note 描述符指针**；note 描述符池 `a2xx`
   可原生化/迁 page6（§4.2）。描述符池扩容到 N 的 9 个数组约 2.3 KB（§4.2）。
5. **目标 256 声与妥协上限 255**：工程目标始终是 **256 声同时发音**（项目名即 256 复音）。
   **255 不是目标数字**，而是两个实现约束的妥协产物：①所有 slot 哨兵是 `0xff`
   （voice_bounds_inventory §4.1 第 7 条；R12），**slot 255 必须保留为哨兵**；
   ②池计数 `a42d` 为 8-bit，回绕上限 255（`polyphony_256_todo.md` R12）。
   因此本设计数组可用 **256** 项但活性上限 255（`std::array<Voice,256>`，§5 R3）；
   GT `PCM_MAX_VOICE=255`、`PCM_SLOTS=260`（`src/pcm.h:27-29`）与之对齐。验收统一
   `-voices:255` 只是"验到当前妥协上限"，**非目标**；真 256 所需的设计变更
   （哨兵表示/计数加宽/数组 ≥264/EFF 260..263/CLI 放开）见 `10_m4_oracle.md` D1
   与 `08_m4_pcm_api.md` P2。
6. **PCM 边界**：voice 参数写继续走 GT `PCM_Write(0xE000|reg)` 可保 bit 级行为；
   但效果簇的 `select_channel=0x1c..0x1f` 与 voice slot 28..31 在扩展模式会冲撞
   （`src/pcm.cpp:131-152`）。建议新增**原生 PCM voice 参数 API**绕过 select 寄存器
   （§4.4、§5 R4；GT 侧对应关系见 08 §4.2）。
7. **mask 提交序**：pending 写入后必须有一次主窗口 `<4` 的读回才 latch
   （`src/pcm.cpp:215-220`）；256-bit 扩展经 `0xE800+0x00..0x1b`（task_irq_map §2.5），
   提交（读回）必须晚于 ext 写。原生引擎可直接写 `pcm.voice_mask/pending` 或用
   `PCM_WriteExt`，但**不得**省掉 readback 语义（O6/O7）。
8. **中断**：IRQ0 handler `0x0522-0x053D` 用 `status&0x1f` 作 `d15c` 下标
   （task_irq_map §1.3/§2.4）。N>32 时该截断是唯一功能性故障；hand 接管 0x0522
   后先读 ext `0xE820`（`PCM_ReadExt`, `src/pcm.cpp:303-304`）再 ack `0xE03E`。
9. **CPU 成本**：`PCM_Update` 的 per-cycle voice 循环是 `reg_slots`
   （`src/pcm.cpp:587,1164`），256 voice 是 ~9× DSP 负载；`cycle_slots` 已钳 28
   （`src/pcm.cpp:1672-1675`）只保采样率、不省 per-slot 计算。§5 R9。
10. **证据缺口与已闭合项**：note-fill `0x0f86` 的外部直接调用者仍未在 flow_main 静态边
    中出现（只有自递归 0x1106/0x1188 与 IRQ 返回扇出）→ 可能经运行时指针分派（Q9 未闭合）。
    已闭合（11 §1.1/§1.5）：`d1a6` 非 per-voice（索引 `(dp,0xd1a4)&1`，仅 2 项，Confirmed）、
    `d1ac/d435` 固定 32 项（Strongly supported；残余调用点下标出处 = 11 B6）、
    `0x15d1 @r6+0x1bfe` 是 ROM1 16 项 per-part 指针表（Strongly supported），与 R18 不矛盾。

---

## 1. 例程清单与调用图

列含义：`入口 PC`（hand 注册点）；`区间`；`调用方式`（进入该入口的 H8 方式）；
`直接调用者`（flow_main/dasm 证据）；`主要被调`；`作用`（置信度）。
「指令数」= `dasm_full.txt` 执行集内该区间行数（下界；未执行臂存在），周期=×12
（GT 每指令 `mcu.cycles += 12`，`src/mcu.cpp:1457`）。

### 1.1 组 A — 引导 / 初始化（rom2）

| # | 例程 | 入口 | 区间 | 调用方式 | 直接调用者（证据） | 主要被调 | 作用 | 指令/周期 |
|---|---|---|---|---|---|---|---|---|
| A1 | `pool_init` | `0x040462` | `0x040462-0x040586` | `pjsr #0x04:0462`（voice_bounds §1.1③；`0x00565`） | boot `0x0565` | `0x40588` | 5 个子循环：A 42 数组清零、B 哨兵、C free-list 构建（`a42d=N`）、D 描述符池、E part 表 | 78 / 936 |
| A2 | `init_desc` | `0x041220` | `0x041220-0x041263` | `pjsr`（`0x051cc`） | `0x0051cc`（task_irq_map:111） | `trapa #0x11` | 清 `d150/152/154/156`；循环 27..0：`ad0e[i]=0`、`word[P(i)+0/2/4]=0x16`、`d15c[i]=0` | 20 / 240 |
| A3 | `init_params` | `0x041266` | `0x041266-0x041324` | `pjsr`（`0x056b6`） | `0x0056b6`（task_irq_map:112） | `0x42b8a`、`trapa #0x18/#0x11` | 初始化 `d181..d18d`（从 `0x802b..`）、循环 27..0：`P-26=0xff` + `bsr 0x42b8a` | 54 / 648 |
| A4 | `pcm_chan_init` | `0x041333` | `0x041333-0x04142d` | `pjsr`（boot `0x0217`） | `0x000217` | `0x4142f` | 写 config `0x3c/0x3d`（源 `rom2[0x1432]=c3 7b`）；`select_channel=0..27` 循环，每通道 `reg12/14=0`、`reg16/18/1a=0xba`；再 `0x1e→0x3e`、`0x1f→0x3e` 效果/全局初始化；mask 全 1/全 0 测试 | 91 / 1092 |
| A5 | `maint_reset32` | `0x04329F` | `0x04329F-0x0433C2` | 经 `0x414A2` stub（task_irq_map:115） | `0x414A2` | `0x433C4/0x433D1/0x43641/0x434CB/0x4342A` | `d1d6..d1dd=0xff`、`d435[32]=0`、`d1ac[32]=4`、循环 trapa 节拍；`0x43370` 起 0x1e/3/4 计数 | 75 / 900 |

### 1.2 组 B — note/voice 分配、释放、链接（rom1）

| # | 例程 | 入口 | 区间 | 调用方式 | 直接调用者（dasm 行） | 主要被调 | 作用 | 指令/周期 |
|---|---|---|---|---|---|---|---|---|
| B1 | `note_fill` | `0x000F86` | `0x000F86-0x00118F` | `bsr16`（自调 `0x1106 [1e fe 7d]`/`0x1188 [1e fd fb]`） | 仅自递归 + IRQ 返回扇出（flow_main: `1106→f86`、`1188→f86`）★ | `0x516c`、自递归 | 从 part 状态（`dp,0xa1d0..a1f5`）物化一只 voice：写 SoA（`ce78/ce94/cf3c/d134/cf90/ceb0/d0fc/cecc/cfe4/d038/d08c/d0e0=2/a4b4/acf2/cfac/d000/d054/a46c`），`jsr 0x516c` 启动 PCM 通道 | 144 / 1728 |
| B2 | `scan_a` | `0x0013A9` | `0x0013A9-0x0013C9` | 分支落入（`0x13a5 BMI`） | `0x13a5→0x13a9` | `0x1459` | 沿 `a220/a250` 链查 note 描述符（`a288==0`、`a314`、`a330 bit0`），命中则 `bsr 0x1459` | 12 / 144 |
| B3 | `kill_a` | `0x001459` | `0x001459-0x0014A8` | `bsr16`（`0x13be`） | `0x13be` | — | `a288[desc]=2`；置 `a2a4 bit0`、`a3bc[slot]=1`、`a4b4[slot]=0xff`；沿 `a410` 链传播 | 24 / 288 |
| B4 | `scan_b` | `0x00151E` | `0x00151E-0x00157C` | 进入点未在 flow_main 定死 | part 路径 | — | 清 `a240[part] bit0`；沿 `a220/a2a4/a250` 链置 `a3bc/a4b4` | 28 / 336 |
| B5 | `alloc_scan` | `0x00157D` | `0x00157D-0x0015D0` | 进入点未定死 | — | `0x15d1` | `a42c = a4a8 - a42d`（短缺量）；按 part `r3=0x0f..0` 走 `a210/a8018`，逐 part 调 `0x15d1` 抢 voice | 19 / 228 |
| B6 | `note_disp` | `0x0015D1` | `0x0015D1-0x0015FA` | `bsr`（`0x15b3`） | `0x15b3` | `0x16f4`/`0x173e` | `r6=part`；读 `word[r6*2+0x1bfe]`（tp=0，`0x1bfe<0x8000` → **ROM1 表**，`src/mcu.cpp:707-709`；11 §1.5）与 `@r2+5 bit4`；按 `a040[part]` 分派 0=0x16f4、2=0x173e | 17 / 204 |
| B7 | `note_cmd` | `0x0015FB` | `0x0015FB-0x0016A1` | 进入点未定死 | — | `0x17ed`/`0x180d`/`0x18ce`/`0x157d` | note 指令循环：处理 `(dp,0xa1e2)` **part 记录**（per-part 0x70 指针，11 §1.2；`a220` 链、`a314/a2f8` 匹配），`bsr 0x18ce` 建描述符 | 56 / 672 |
| B8 | `steal` | `0x0016F4` | `0x0016F4-0x00173D` | `bsr16`（`0x15ea`） | `0x15ea` | `0x17ed` | 抢音：沿 `a220` 链找 `a288==0` 者，`bsr 0x17ed`；失败 `r0=1` | 19 / 228 |
| B9 | `key_on` | `0x00173E` | `0x00173E-0x00179B` | `bsr16`（`0x15f3`） | `0x15f3` | `0x179c`、`0x17ed` | `r4=a200[part]`；模式 `r6=0/1/2` 三轮调 `0x179c`+`0x17ed`（normal/effect/low）；返回 `r0` | 31 / 372 |
| B10 | `slot_select` | `0x00179C` | `0x00179C-0x0017EC` | `bsr`（`0x1744`）、`bsr16`（`0x175f/0x1781`） | `0x1744/0x175f/0x1781` | — | 沿 `a220` 链按 `ad0e` 最小者选 PCM 通道；结果经 `(dp,0xa438)` 返回 `r1` | 27 / 324 |
| B11 | `note_helper` | `0x0017ED` | `0x0017ED-0x001805` | `bsr16`×7（`0x150e/1667/1701/1720/174c/1768/178a`） | 同左 | `0x1823` | 沿 `a250/a2dc` 链循环调用 **release A**（0x1823，勘误见 11 §1.6 B1）；结束后 `TST (dp,0xa42c)` | 9 / 108 |
| B12 | `aux_alloc` | `0x00180D` | `0x00180D-0x001822` | `bsr16`（`0x1688`） | `0x1688` | `0x187e` | 同 B11 但走 **release B**（0x187e） | 7 / 84 |
| B13 | `release_a` | `0x001823` | `0x001823-0x00187D` | `bsr`（`0x17f9`） | `0x17f9` | `0x516c`、`0x1b44`、`0x1bad` | **释放一条 voice 并压回 free 栈**（名字勘误，11 §1.6 B1）：`jsr 0x516c`；`d0e0[slot]=4`→派发 `pcm_stop 0x5238`；清 `ad0e/a3bc/a4b4`；free 栈压入（H1）；`a42d++/a42c--` | 27 / 324 |
| B14 | `release_b` | `0x00187E` | `0x00187E-0x0018CD` | `bsr16`（`0x1815`） | `0x1815` | 同上 | release A 的 `a42f` 栈变体（free head 端） | 24 / 288 |
| B15 | `desc_setup` | `0x0018CE` | `0x0018CE-0x00194B` | `bsr16`（`0x169c`） | `0x169c` | `0x1b90/0x194c` | 在描述符池建 note：接 `a42e` 链、写 `a250..a330`、`a4aa` 链哨兵 `0xff`、`a210++`；内部 `bsr 0x192f`→`pool_pop`、`bsr 0x193f`→`link`（**真正分配器**，11 §1.6 B1） | 38 / 456 |
| B16 | `link` | `0x00194C` | `0x00194C-0x0019AC` | `bsr`（`0x193f`） | `0x193f` | — | 双链插入：`a2c0/a3f4/a410/d0a8/d0c4/a368/a384/a2dc` | 23 / 276 |
| B17 | `pool_pop` | `0x0019AD` | `0x0019AD-0x0019C3` | `bsr`（`0x192f`） | `0x192f` | — | free 栈弹出 `a42f→a430`，`a42d--` | 7 / 84 |
| B18 | `free_voice` | `0x0019C4` | `0x0019C4-0x001A23` | `bsr16`（cleanup `0x055c` 路径 `0x0587`） | `0x00587`（dasm:2801 注释链） | `0x1b44/0x1bad` | `a3a0 bit7` 早退；`ad0e==0xff` 早退；清 `a3bc/a4b4`；解 `a2c0/a2dc` 链；free 栈压回；`a42d++` | 28 / 336 |
| B19 | `cleanup_seq` | `0x001A4D` | `0x001A4D-0x001A6B` | 进入点未定死 | — | — | 对 `a4aa/a4ab` 链清 `ce3f=0xff` | 10 / 120 |
| B20 | `mask_acc` | `0x001AD3` | `0x001AD3-0x001AF1` | `bsr16`（`0x0621/0x0827`） | `0x00621/0x00827` | — | 循环 27..0：`acf2[i] &#124;= a4b4[i]; a4b4[i]=0`；IML=7 包裹 | 9 / 108 |
| B21 | `unlink` | `0x001B23` | `0x001B23-0x001B43` | `bsr16`（`0x1bb3`） | H2 `0x1bb3` | — | `a220/a250/a230/a26c` 双链摘除 | 11 / 132 |
| B22 | `H1` | `0x001B44` | `0x001B44-0x001B8F` | `bsr16`（`0x1847/0x18a2/0x19f7`） | B13/B14/B18 | — | free 栈压入：保存 r3、改 `a2c0/a3f4/a410/d0a8/d0c4` | 21 / 252 |
| B23 | `H3` | `0x001B90` | `0x001B90-0x001BAC` | `bsr16`（`0x191d`） | B15 | — | 清 `a288/a2a4`；按 `a314` 选 `a200[part]` 最小 | 9 / 108 |
| B24 | `H2` | `0x001BAD` | `0x001BAD-0x001BFD` | `bsr16`（`0x1870/0x18c0/0x1a20`） | B13/B14/B18 | `0x1b23` | free 栈摘除、`a288=0x94`、选最老 `a220`、`a210--` | 26 / 312 |

★ B1 的调用者不闭合：flow_main 没有外部 `bsr/jsr → 0x0f86` 的静态边（只有
`000003ba→0f86`、`00007c3f→0f86` 两个返回扇出），疑似经运行期函数指针/事件分派进入
（见 §5 Q4）。mk2 §6.3 标注为「note 路径内部」。

### 1.3 组 C — PCM 命令服务 / voice 参数 / DSP 耦合（rom1）

| # | 例程 | 入口 | 区间 | 调用方式 | 直接调用者（dasm 行） | 主要被调 | 作用 | 指令/周期 |
|---|---|---|---|---|---|---|---|---|
| C1 | `pcm_start` | `0x00516C` | `0x00516C-0x0051B5` | `jsr #0x516c` | `0x10d5`(dasm:1483)、`0x1157`(:1561)、`0x1829`(:2526)、`0x1886`(:2580) | — | `d15c[slot]=0`；写 `(dp,0xe03e)` select；读 PCM `e032/e03a/e034`；写 `P+0/2/4=0x12/0x14`、`P+26/30=0xb5`；`r1=slot` | 21 / 252 |
| C2 | `pcm_dispatcher` | `0x0051D0` | `0x0051CC-0x00522A`（0x51cc pjsr + 落入 0x51d0） | fallthrough（`0x51cc`） | event dispatch（`0x04af` 族） | `0x25F0`、`0x5238`、`0x52CB` | `trapa #0x10`；非 0→扫 `d15c`（C: `0x51e0 TST`，命中 `0x51ef CLR` → `BRA 0x25f0`）；0→扫 `d0e0`（`0x520b`），命中后 `@r2+0x522c` 跳表 | 29 / 348 |
| C3 | `pcm_stop` | `0x005238` | `0x005238-0x0052C8` | `jmp r2`（跳表项） | C2 | — | `CLR ad0e[slot]`；读 PCM `0x32/0x34` 得旧通道回写 `ad0e`；状态 `0x0e/0x10` 或 `0x16`；`trapa #0x12` | 40 / 480 |
| C4 | `pcm_play` | `0x0052CB` | `0x0052CB-0x005390` | `jmp r2`（跳表项） | C2 | `0x53EB/0x5998/0x5D6E/0x546E/0x54CC/0x54FB/0x5533/0x564A/0x5626` | 完整启动一条 voice：materialize→coeff→copy→mask set→参数写；kill 分支 `0x5390` | 59 / 708 |
| C5 | `irq_service` | `0x0025F0` | `0x0025F0-0x002666` | `BRA`（`0x51f3`） | C2 | `0x51F6`/`0x51F3` | 状态字判 `0x0e`；读 PCM `0x32/0x34`；写 `P+0/2/4`、`P+26/30`；走 `d0a8/d0c4` 链后回 dispatcher | 29 / 348 |
| C6 | `svc_math` | `0x002669` | `0x002669-0x00272D` | fallthrough（C5 `BEQ`） | C5 | — | 32/32 定点除法（`DIVXU #0x2ee0`）算频率分数；写 PCM `reg10`；查 `0x78ee/0x7aee` 表 | 53 / 636 |
| C7 | `note_on_setup` | `0x00272E` | `0x00272E-0x02874`（区间上界待精确） | `BRA`（`0x548e`） | `0x548E`（mask_set 且 `d0e0==0`） | — | 读 tone `r5[4]/[8]`；`cdfe[slot]=0`；写 `P+0xa2`、`P-81` 等；门控/重触发 | 57+ / 684+ |
| C8 | `r2d95` | `0x002D95` | `0x002D95-0x002E82` | 进入点未定死 | — | — | 与音高/包络相关（语义未确证，R11 外部例程之一） | 84 / 1008 |
| C9 | `r2e83` | `0x002E83`/`0x002E85` | `0x002E83-0x002F9F` | `bsr16`（`0x591c`） | `0x591C` | — | 0x2e83 本身是 `bsr -3 → 0x2e82(rts)` 蹦床；主体 `0x2e85-` 处理 voice 参数 | 81 / 972 |
| C10 | `tone_fields` | `0x003580` | `0x003580-0x003614` | `bsr16`（`0x5329/0x532d/0x53b3`） | `0x5329/0x532d/0x53b3` | — | 由 tone 指针构造 REC0/REC1 字段（`P-0x80..`、`P-92/-90`、`P+0xa8`）；`ret` 于 `0x3614` | 49 / 588 |
| C11 | `gate_setup` | `0x003615` | `0x003615-0x003708`（rts 待精确） | `bsr16`（`0x5334/0x53ba`） | `0x5334/0x53ba` | — | `ep=@P+0x98/0x99`；清 `cdc6[slot]`、`P-115`；由 tone `r5` 建 REC0 门控/系数 | 59 / 708 |
| C12 | `interp_entry` | `0x003709` | `0x003709-0x0037F1` | `bsr16`（`0x5911`） | `0x5911` | `0x38B0` | 读 P-115 门控；选 REC0 输入；`r1=P-0x80` 或 `P-0x5e` 落入 `0x38b0` | 33 / 396 |
| C13 | `interp` | `0x0038B0` | `0x0037F2-0x0039B4`（含入口 0x37f2/0x37fe） | fallthrough（`0x37f7/0x38ac`） | C12、`0x2865→0x37fe`、`0x3001→0x37fe`（voice_memory_map App.A） | — | 两条 0x22B 记录 REC0/REC1：`MULXU (dp,0xad2a)` 积分、clamp、输出 `r1+0x20`（REC0+0x20/P-0x60） | 105 / 1260 |
| C14 | `cross_copy` | `0x003A9E` | `0x003A9E-0x003BB1` | `bsr16`（`0x533f`） | `0x533F` | — | 交换两 slot 的 `cdc6` word，并拷贝 AoS 字段（`0x3aa1-0x3aac`） | 71 / 852 |
| C15 | `ts_scan` | `0x005869` | `0x005869-0x0058E8` | `trapa #0x1b` 后落入；`BEQ` 自 `0x56c0` | `0x56C0` | `0x5998/0x3709/0x2E83/0x5671` | 写 `(dp,0xad2a)`；循环 `r1=27..0` 过 `P(v)` 表找活跃 voice（`P+0<0x12`、`P-26==0`）；`d178/d17a` 暂存；`0x58d7` 循环清 `P-26` | 45 / 540 |
| C16 | `coeff_calc` | `0x005998` | `0x005998-0x005D6D` | `bsr16`（`0x530c/0x53a4/0x58fe`）、`bsr`（`0x5946`） | C4/C15 | — | **读 SoA** `ce78[slot]`(`0x599a`)、`d134[slot]`(`0x59b0`)，读 tone ROM `0x7218`、系数表 `0x9740/0x9060..`；写 `P+0x86/+0x8a/...` | 217 / 2604 |
| C17 | `coeff_copy` | `0x005D6E` | `0x005D6E-0x005DC2` | `bsr16`（`0x5317/0x5950`） | C4/C15 | — | 把 `r2` 指向的 P 的 `+0x86/+0x88/+0x8a/-80/-114/+0x96/…` 拷到 `r0` 指向的 P | 23 / 276 |

**C16 是关键边界**：它同时访问 SoA（`ce78/d134`，slot 索引）与 AoS（P 相对），
所以 256 下不能只做「AoS proxy」而必须让 `ce78/d134` 也按逻辑 slot 可见（§4）。

### 1.4 组 D — rom2 搜索 / per-voice 初始化

| # | 例程 | 入口 | 区间 | 调用方式 | 直接调用者 | 主要被调 | 作用 | 指令/周期 |
|---|---|---|---|---|---|---|---|
| D1 | `voice_search` | **`0x045CBE`（边界）** | `0x045C7A-0x045D9A`（循环体 `0x45CC0-0x45CE0`，15/180） | **fallthrough**（`0x45cbe` 从 `0x45cbc` 掉入/`0x45c94 BNE`） | 4 个 IRQ 上下文（`0x344/0x7b6a/0x7d8a/0x461ac`；mk2 §6.3 #12） | `0x45CE6` 之后 | 按 part `r4` 扫 `a368[i]`；匹配者取 `ad0e[i]` 的 max，输出给 part 混音 | 91 / 1092 |
| D2 | `pool_init` | 同 A1 | | | | | | |

（D1 是唯一非 entry-only 的 per-voice 例程；hand 必须在 `0x45cbe` 注册/接管，
否则 widening 后循环从 `P(v)` 表越界读到 pitch 表——voice_bounds §3.1。）

### 1.5 任务/IRQ 入口（与 voice 相关的）

| 入口 | 向量/来源 | 行为 | 证据 |
|---|---|---|---|
| `0x000522` | IRQ0 = PCM voice IRQ（vector 32） | `r0 = Read(0xE03E) & 0x1F; d15c[r0]=0xFF; r0=2,r1=1; jmp 0x03DB` | task_irq_map:69,89；dasm:375-377 |
| `0x000491` | 主 sleep 循环 | idle | `src/mcu.cpp` 调度；task_irq_map §1.1 |
| `0x000342` | FRT2 OCIA heartbeat | 事件队列扫描→dispatcher `0x4af`（`isr` oracle） | task_irq_map §1.3 |
| `0x007D8A` | FRT3 OCIB per-frame | `pjsr 0x414D8 → jmp 0x4ACA9`（SM 队列消费者，**不触 per-voice 数组**） | task_irq_map:76,98；mk2 §6.4①；dasm:17564 |
| `0x004ACA9` | per-frame body | 处理 `(dp,0xd68a)` SM 队列 + `jsr 0x43A5` 事件 ring | dasm:17564-17682 |

### 1.6 主调用图（S；来源：dasm 调用点 + voice_bounds/mk2 表）

```
MIDI/part 状态 (a1xx, ROM)
   │
   ├─(note 指令)────────────────────────────► B7 note_cmd 0x15fb
   │                                             ├─ B11 note_helper 0x17ed ─► B13 release_a 0x1823
   │                                             ├─ B12 aux_alloc 0x180d ───► B14 release_b 0x187e
   │                                             ├─ B15 desc_setup 0x18ce ──► B16 link 0x194c
   │                                             └─ B5 alloc_scan 0x157d ───► B6 note_disp 0x15d1
   │                                                                             ├─ B8 steal 0x16f4
   │                                                                             └─ B9 key_on 0x173e
   │                                                                                  ├─ B10 slot_select 0x179c
   │                                                                                  └─ B11 note_helper 0x17ed
   ├─ B1 note_fill 0x0f86 (part a1xx → SoA 物化) ──────────────────────────────► C1 pcm_start 0x516c
   │
   └─ B20 mask_acc 0x1ad3 (acf2 |= a4b4)   ← 每帧由 0x0621/0x0827 调用

事件 dispatcher (0x04af) ──► C2 pcm_dispatcher 0x51cc/0x51d0
       │                         ├─(d15c≠0)─► C5 irq_service 0x25f0 ──► C6 svc_math 0x2669
       │                         └─(d0e0≠0)─► C3 pcm_stop 0x5238 | C4 pcm_play 0x52cb
       │                                                                  ├─ C4 0x53eb materialize
       │                                                                  ├─ C16 coeff_calc 0x5998
       │                                                                  ├─ C17 coeff_copy 0x5d6e
       │                                                                  ├─ C10 tone_fields 0x3580
       │                                                                  ├─ C11 gate_setup 0x3615
       │                                                                  ├─ C14 cross_copy 0x3a9e
       │                                                                  ├─ 0x546e mask_set ─(d0e0==0)─► C7 0x272e
       │                                                                  ├─ 0x54cc param_ack
       │                                                                  ├─ 0x54fb flush_off ──► PCM regs 0/1/2/3
       │                                                                  ├─ 0x5533 param_write ─► PCM regs
       │                                                                  ├─ 0x564a flush_on  ──► PCM regs 0..3 + readback
       │                                                                  └─ 0x5626 loop_write
       └─ C15 ts_scan 0x5869 (trapa #0x1b; P(v) 扫描) ──► 0x5998 / 0x3709 / 0x2e83 / 0x5671

FRT3 OCIB 0x7d8a ──► 0x414d8 ──► 0x4aca9 (SM 队列, ROM 保留)
IRQ0 0x0522 ──► d15c[status&0x1f]=0xff ──► C2 下次扫描 (hand 接管点)
rom2 搜索 D1 0x45cbe ◄── 4 个 IRQ 上下文直接可达（无 jsr）
```

---

## 2. 数据结构

### 2.1 页模型与可寻址性

- `r0-r3→dp, r4-r5→ep, r6-r7→tp`（`src/mcu.h:208-215`）；`@rN+disp16` /
  `(dp,disp16)` 的地址 = `(page<<16)|((rN+disp16)&0xffff)`（`src/mcu_opcodes.cpp:571-580`），
  **disp16 加不出 page 位**。
- 页 0 SRAM 窗口只有 `0x8000-0xdfff`（24 KB，`src/mcu.cpp:703,737`）；
  page6 `b_ram` 64 KB 已在 GT（`src/mcu.cpp:658,843-846,1064`）。
- 结论：**N>28 时任何 ROM 侧的 per-voice 数组访问都不可能寻址到扩容区**；
    hand 原生化是唯一出路（polyphony_256_feasibility §2.1/2.3）。

### 2.2 AoS — 28 × 0x12a voice 结构（C）

- 区域 `[0xad2e, 0xcdc6)` = 0x2098 B = 28 × 0x12a。slot 0 `[0xad2e,0xaed8)`，
  slot 27 `[0xcc9c,0xcdc6)`。
- 指针表：`P(v) = rom1[0x64d6 + 2*v]`，28 个 big-endian word，`P(0)=0xadae`，
  步长 0x12a；下一 word `rom1[0x650e]=0x0006` 是 pitch 表（不是指针）。
  原始字节 voice_memory_map §2 line 76-79。
- `S = P-0x80`（子块），`M = [P, P+0xaa)`（主块）；`P-2` = slot 回写
  （write `0x53f3 [e8 fe 91]`，读 `0x361d/0x3aa7/0x531e/0x546e/0x5540/0x5626/...`）。
- 字段表（节选；完整证据见 voice_memory_map §4.1/§4.2）：

| 偏移 | 宽 | 候选语义 | 证据 PC（dasm 行） | 置信 |
|---|---|---|---|---|
| `P-2` | w | slot index 0..27 | `0x53f3`（:10921）；读 `0x361d`(:6756) | C |
| `S+0x00..+0x20` | 17w | REC0 插值记录（含 tone `r5[72]`/`r5[73]` 输入、`MULXU (dp,0xad2a)` 积分、`+0x20` 输出） | `0x35af`(:6654)、`0x38b0-0x39b4`(:6978-7107) | S |
| `S+0x22..+0x42` | 17w | REC1 记录（同构于 REC0，入口 `0x38ac`） | `0x359a/0x35cc/0x3608`、`0x38aa`(:6974) | S |
| `S+0x45` | b | bit7 = steal/cleanup pending | `0x5321/0x537a/0x53ae BTSTI [e0 c5 f7]` | C |
| `S+0x66` | w | 0 / `0xff` 哨兵（init、key-on 清） | `0x2e89/0x58e2/0x41310` | C |
| `S+0x68` | w | PCM `br+0x1a`（pitch 初值） | `0x55c3`(:11107) | C |
| `S+0x6a` | w | PCM `br+0x36` | `0x563d`(:11144) | C |
| `S+0x6c/6d/6e` | b | PCM `br+05/09/0d` | `0x5553/0x5583/0x556b` | C |
| `S+0x70/72/74/76` | w | PCM `br+06/0a/0e/1e` | `0x5556/0x5586/0x556e/0x5540` | C |
| `S+0x6f` | b | voice-busy/event | `0x2600 TST @r0-17` | S |
| `S+0x78..0x7d` | b | LFO/算法选择、flag、0/2 选择 | `0x2f83/0x3123/0x312e/0x3139/0x2fc3` | I |
| `S+0x7e` | w | slot（=P-2；两者同址，S+0x7e = P-2） | 同 `P-2` | C |
| `P+0x00/02/04` | 3w | 状态字：init `0x16`；`0x0e/0x10`（运行）、`0x12/0x14`（启停中）、`0x0c` | `0x41248/0x4124d/0x41252`(:13233-13235)、`0x262f/0x2632/0x2635`(:4284-4286)、`0x2f5d`、`0x55ed` | C |
| `P+0x06..0x2c` | 混合 | 计数器/命令/指针（多处 `(agg.)`） | voice_memory_map §4.2 | I |
| `P+0x2e` | w | tone 0x70B 记录指针（`rom1[0x7218+2*ce78]`） | `0x5442`(:10943)、`0x377a` | C |
| `P+0x30` | w | `0x8748 + cecc[slot]` | `0x545a/0x5469`(:10951-10961) | C |
| `P+0x86/88/8a/8c/8e/90/92/94/96` | 9w | DSP 系数（coeff_calc/copy 目标） | `0x5d72-0x5dbe`(:12063-12095)、`0x5a15` | C |
| `P+0x98/99/9a` | 3b | EP 页 for `+0x9c/+0x9e/+0xa0` | `0x3615`(:6752)、`0x3580`(:6615)、`0x460c` | C |
| `P+0x9b` | b | tone index（`ce78[slot]`） | `0x5438/0x543c` | C |
| `P+0x9c/9e/a0` | 3w | 样本/音色/DPCM 表指针 | `0x5412/0x541a/0x5422` | C |
| `P+0xa2/a3` | b | tone 字节 `r5[8]`、算子/类型 | `0x273f(:4370)/0x3bf9` | C |
| `P+0xa4` | b | ← 分数累加使能 | `0x2681 CLR [f0 00 a4 13]` | S |
| `P+0xa6` | w | 分数累加器 | `0x2685 MOVG2 [f8 00 a6 81]` | C |
| `P+0xa8` | w | 有符号音高/偏移（tone `r5[14]`） | `0x3610`(:6746)、`0x37a1` | C |

### 2.3 SoA — per-voice 并列数组（C：voice_memory_map §6 / kMoved）

字数组（16-bit）：`a46c, 64d6, cdc6, cdfe, cf58, cfac, d000, d054`；
字节数组（42，1B/slot）：
`a34c a368 a384 a3a0 a3bc a3d8 a3f4 a410 a4aa a4b4 acf2 ad0e`
`ce3f ce5c ce78 ce94 ceb0 cecc cee8 cf04 cf20 cf3c cf90 cfe4`
`d038 d08c d0a8 d0c4 d0e0 d0fc d118 d134 d15c`
+ 描述符池 `a250 a26c a288 a2a4 a2c0 a2dc a2f8 a314 a330`。
索引空间与语义（代表证据行）：

| Base | W | 语义（置信） | 证据 |
|---|---|---|---|
| `a368` | b | slot→part（`voice_search` 与 r4=part 比较；勘误见 11 §1.6 B2） | `0x45cc2`(:15908) |
| `a384` | b | slot→描述符索引（`link` 写入；勘误见 11 §1.6 B2） | `0x1748`(:2372)、`0x19a8` |
| `a3a0` | b | slot 空闲标记（**0x94=free**，bit7=1；分配（`note_fill 0xf86`）时清 0；勘误见 11 §1.6 B2） | `0xf86 CLR`、`0x19c4 BTSTI #7`、`0x1867`(:2560) |
| `a3bc` | b | 占用/状态字节 | `0x183d`(:2536)、`0x148c`(:2008) |
| `a3d8` | b | free-list next（哨兵 0xff；slot0 0xff） | `0x40517`(:12687)、`0x1854` |
| `a3f4/a410` | b | 链 prev/next（0xff=空） | `0x1b48`(:2976)、`0x19e7` |
| `a4aa` | b | slot→sequence；`a4aa[N]=0xff` 哨兵 | `0x1928`(:2689) |
| `a4b4` | b | 激活位（OR 进 acf2） | `0x1ada`(:2897) |
| `acf2` | b | mask 累积 | `0x1ade`(:2898)、`0x2f17`(:5737) |
| `ad0e` | b | slot→PCM 通道；0xff=空 | `0x17c0`(:2454)、`0x1839`(:2535) |
| `ce78` | b | tone index（→`rom1[0x7218]`） | `0xfb8`(:1274)、`0x5438` |
| `d0a8/d0c4` | b | 链 prev/next（voice start 链） | `0x2638`(:4287)、`0x58a6`(:11263) |
| `d0e0` | b | 命令种类（0=idle,2,4） | `0x1026`(:1358)、`0x521d`(:10687)、`0x182e`(:2530) |
| `d15c` | b | PCM IRQ pending（0xff） | `0x531`、`0x516c`(:10604)、`0x51e0`(:10652) |
| `a250/a26c` | b | note 描述符链 next/prev | `0x13c3`(:1961)、`0x18f0`(:2646) |
| `a288` | b | note 释放状态（`2`=releasing，`0x94`=idle） | `0x1459`(:1970)、`0x1bc2`(:3058) |
| `a2a4` | b | note 状态位（bit0/bit2） | `0x1475`(:1975)、`0x165f`(:2264) |
| `a2dc` | b | 描述符 slot→voice 索引 | `0x1488`(:2005)、`0x1996`(:2766) |
| `a314` | b | 描述符→part/年龄 | `0x13b2`(:1943) |
| `a330` | b | 描述符标志位 | `0x13b8`(:1949) |

**语义未证**：`a34c, cee8, cf04, cf20, cf3c, cf58, cf90, cfe4, d000, d038, d054, d08c, d0fc, d118, d134, cdc6, cdfe` 的精确含义（voice_memory_map §9、todo「非阻塞」）；实现时保留 `f_0xNN` 命名。

### 2.4 描述符池与链表

- note 描述符池 = `a200/a210/a220/a230/a240`（per-part，16 项）+ `a250/a26c/a288/a2a4/a2c0/a2dc/a2f8/a314/a330`（per-note，N 项）：
  loop E 16 次（`0x40565`，voice_bounds §1.5）、loop D N 次（`0x4053b`）。
- `(dp,0xa1e2)`（`0x1605 MOVG2 [1d a1 e2 82]`；另见 `0x101xx` 多处）持有的是
  **per-part 0x70 记录指针**（`0x8048+0x70*part`，表 ROM1 `0x1bfe`；全读点枚举见 11 §1.2），
  **不是 note 描述符指针**。因此须保持页 0 SRAM 可寻址的是 **part 记录区 `[0x8048,0x8718)`**；
  `a2xx` 描述符池只按字面基址+索引访问，可原生化/迁 page6（§4.2）。
- voice 侧三组链：
  1) 分配链 `a2c0/a3f4/a410/a3d8`（H1/H2/B16/B18）；
  2) 启动链 `d0a8/d0c4`（voice start 顺序，`0x2638/0x58a6`）；
  3) free 栈 `a3d8` + 头尾 `(dp,0xa42f/a430)` + 计数 `(dp,0xa42d)`；
     短缺 `(dp,0xa42c)`。
- `d158/d15a` = 当前启动中的两个 live P 指针（`0x52f7/0x5300`；kill 时
  `d15a=0xffff`，`0x5392/0x5396`）。

### 2.5 mask/IRQ 状态约定（对应 task_irq_map §2）

- 当前 mask：`d150`（bit i = voice 16+i，低字节 16-23/高字节 24-31）、
  `d152`（bit i = voice i）；待发：`d154/d156`。
- PCM 位映射：reg0=voice24-31、reg1=16-23、reg2=8-15、reg3=0-7
  （`src/pcm.cpp:89-99`；`src/pcm.h:35-37`）；GT `voice_mask` bit N = voice N。
- 写路径：`0x546e` 置 pending（`cmp r1,w #0x000f`，`0x5474 BLS`→d156，否则减 16→d154；
  `0x547a/0x5480`）→ `0x54fb` 清 flush（`d150 &= ~d154`，`0x5525/0x5527` movsw）
  或 `0x564a` 置 flush（`d150 |= d154`，`0x5662/0x5664`），随后 `movl r6 @(br,$00)`
  （`0x5529/0x5666`）**读回即 GT latch**（`src/pcm.cpp:215-220`）。
- 256-bit 扩展：主窗口低 32 bit 不变；voice 32..255 经 `0xE800+0x00..0x1b`
  （byte a = voices 32+8a..39+8a，task_irq_map §2.5）；ext 写后必须回到主窗口读回。
- IRQ：`d15c[slot]`；IRQ0 `0x0522` 用 5-bit 状态截断（`0x052D AND #0x1F`）。
  扩展下完整 slot 在 `PCM_ReadExt(0x20)`（`src/pcm.cpp:303-304`）。

---

## 3. 最小替换集

### 3.1 判定标准与分类

**必须 hand 原生（H）**：读写 per-voice 数组 / 解引用 AoS P / 编码 28/32 界的例程。
**可保留 ROM（R）**：只操作 part 全局标量、ROM 表、PCM 效果寄存器或描述符指针的例程。
**条件保留（M）**：只要 §3.2 镜像字段有效即可保留，否则一起原生。

| 组 | 例程 | 类别 | 理由 |
|---|---|---|---|
| A1 pool_init | `0x40462` | **H** | 28 次界、建 free 栈；256 必须重写（R11/R12） |
| A2 init_desc | `0x41220` | **H** | `P(v)` 表 + `d15c` 循环 |
| A3 init_params | `0x41266` | **H** | `P(v)` 表循环 + `0x42b8a` |
| A4 pcm_chan_init | `0x41333` | **H** | 立即数 28 循环（`0x4135c`）；config 写点 |
| A5 maint_reset32 | `0x4329f` | **H** | `d1ac[32]`/`d435[32]` **固定 32 项**（11 §1.1），按 R18 留 SRAM |
| B1 note_fill | `0x0f86` | **H** | 直接写 13+ SoA 数组 |
| B2..B24 note/release/free | 全部 | **H** | 全部按 slot 索引 SoA/AoS（R11 22x 站点；B13/B14 为 release，见 11 §1.6 B1） |
| C1 pcm_start | `0x516c` | **H** | 写 slot 索引的 `d15c` 与 AoS |
| C2 pcm_dispatcher | `0x51d0` | **H** | 扫描 `d15c/d0e0` 界 28；jump table 需接管 |
| C3 pcm_stop | `0x5238` | **H** | slot 索引 ad0e/P |
| C4 pcm_play | `0x52cb` | **H** | slot 全链路 |
| C5 irq_service | `0x25f0` | **H** | AoS P |
| C6 svc_math | `0x2669` | **H** | AoS P（`P+30/38`、PCM reg） |
| C7 note_on_setup | `0x272e` | **H** | AoS P + `cdfe[slot]` |
| C8 r2d95 | `0x2d95` | **H**（保守） | R11 新增外部例程；per-voice 访问待逐条确认 |
| C9 r2e83 | `0x2e85` | **H**（保守） | 同上 |
| C10 tone_fields | `0x3580` | **H** | AoS P-92/-90/+0xa8 |
| C11 gate_setup | `0x3615` | **H** | `cdc6[slot]` + AoS |
| C12/C13 interp | `0x3709/0x38b0` | **H** | AoS REC0/REC1 |
| C14 cross_copy | `0x3a9e` | **H** | 两 slot 的 `cdc6` |
| C15 ts_scan | `0x5869` | **H** | `P(v)` 表 28 循环 |
| C16 coeff_calc | `0x5998` | **H** | SoA `ce78/d134` + AoS |
| C17 coeff_copy | `0x5d6e` | **H** | AoS 两 P |
| D1 voice_search | `0x45cbe` | **H** | `a368/ad0e` 28 循环 |
| IRQ0 | `0x0522-0x053D` | **H** | `&0x1f` 截断 + `d15c` 写 |
| 0x43695 族 | `0x43695/a3/c3/d5` | **M** | `d1ac` 固定 32 项、已执行调用点索引 ≤6（11 §1.1），可留 SRAM；残余调用点出处见 11 B6 |
| effect cluster | `0x5ddc-0x64c3` | **R** | 只写 PCM 效果/全局寄存器（R17）；依赖 §4.4 的 select 方案 |
| per-frame SM | `0x4aca9` | **R** | 只碰 `d68a/d68e` SM 队列与 `0x43a5`（mk2 §6.4④） |
| part/MIDI/ROM 表/LCD/timer/SM | — | **R** | 无 per-voice 写入 |
| `0x42b8a` | `0x42b8a` | **H** | 清 AoS `P-108/P-74` + 2 个全局（`0x41315` 调用处带 r0=P）；per-voice AoS 解引用，n>28 无 SRAM P（已闭合，11 §1.4） |

### 3.2 部分替换时的 SRAM 镜像字段清单

以下字段在「hand 拥有、但仍有 ROM 读者/写者」时必须在边界同步。
同步格式：`镜像时机` = 写入后立即 / 每次边界 / 仅调试导出。
一致性要求：`byte-exact` = 逐字节一致（trace/hash 可对照）；`semantic` = 只要值域正确。

| SRAM(页0) | 宽/大小 | 语义 | 原生载体 | 未替换读者（若保留） | 同步 |
|---|---|---|---|---|---|
| `d150/d152` | 4 B | 当前 voice mask | `VoicePool::mask_cur[32]` 低 4B | 无（R17 确认效果簇不读）；状态哈希 | byte-exact，flush 后 |
| `d154/d156` | 4 B | 待发 mask | `mask_pend[32]` 低 4B | — | byte-exact |
| `d15c[N]` | N B | IRQ pending | `irq_pend[slot]` | IRQ0 0x522（若 M）；C2（若 R） | byte-exact，置/清后 |
| `d0e0[N]` | N B | 命令种类 | `cmd[slot]` | C2（若 R）；写点 `0x1026/0x182e` | byte-exact |
| `d158/d15a` | 4 B | live P 指针 | `Voice*`（≤28 时=SRAM 槽地址） | C16/C17（若 R） | 指针→SRAM 槽，每次启动 |
| `a42c` | 1 B | 短缺量 | `pool.shortfall` | B5（若 R） | byte-exact |
| `a42d` | 1 B | 活跃计数 | `pool.count` | 0x157d/0x1587/0x1873/0x18c3/0x19bf | byte-exact |
| `a42e/a42f/a430` | 3 B | 描述符链头、free 头/尾 | `pool`/`note_pool` 头尾 | B15/B17/B13/B18（若 R） | byte-exact |
| `a3d8[N]` | N B | free 链 | `pool.free_next[]` | B13/B14/B18（若 R） | byte-exact |
| `a2xx` 描述符池 | 9×N | note 描述符 | 可原生化/迁 page6（11 §1.2；`a1e2` 不指它） | note 路径 ROM（若保留） | 无需镜像 |
| AoS `[0xad2e,0xcdc6)` | 0x2098 | 28 个 voice | `Voice[0..27]` 的 SRAM 视图（n=28 模式） | C16/C13 等（若 R） | byte-exact（**仅 n=28 回归模式**） |
| `ce78/d134/cfe4/d038/d08c/cf90/cecc` | 7×N B | materialize 源 | `Voice` 字段 | C16 读 `ce78/d134` | byte-exact，materialize 前 |
| `cfac/d000/d054` | 3×2N B | 指针类 | `Voice` 字段 | 若 C16/C17 保留 | byte-exact |
| PCM 寄存器 0xE000.. | 64 B | 设备状态 | GT `pcm` | 全部 | 经 `PCM_Write/WriteExt`（不复制） |

**最小镜像集（推荐）**：若按 §3.1 全量原生 H 组，则只剩 `d150..d157`（状态哈希）、
`d15c`（IRQ0 若保留）、`a2xx`（part 指针兼容）需要处理；`Voice` 本体无需回写 SRAM。

### 3.3 分派接入与运行时契约

- **注册**：`MK2CPP_Init()`（`mk2cpp/src/mk2cpp.cpp:122-147`）在 gen 填充后调用
  `MK2CPP_HandFillTables()`（`MK2CPP_HAS_HAND`）；hand 用
  `MK2CPP_HandRegister(flat, fn)` 注册进独立稀疏 override 表，查询时 hand 优先
  （方案 B，契约与 API 见 09 §2）。
- **返回语义**：
  - `bsr/jsr` 进入的例程（尾指令 `rts [19]`）：`mcu.pc = MCU_PopStack()`（`src/mcu_opcodes.cpp:311-314`）；
  - `pjsr` 进入的例程（尾指令 `ret [11 19]`）：先 `mcu.cp=PopStack()`、再 `mcu.pc=PopStack()`（同文件 345-349）。
  - fallthrough/分支进入（如 `0x45cbe`、`0x51cc→0x51d0`、`0x548e→0x272e`）：
    hand 函数执行完整段并显式设 `mcu.pc`，且不得弹出未压入的帧。
- **周期**：L0 下宿主每步后统一 `mcu.cycles += 12`（`src/mcu.cpp:1457`），hand
  函数**不得**自改 cycles。§1 表的指令数/周期是整例程下界，仅在评估或启用 L1 时使用
  （L1 的 `12*(n-1)` 补账协议见 09 §4.3）。
- **状态标志**：边界处调用者常用 `BPL/BNE/BLE` 检查返回的 `r0/r1`。GT 的 `MOVG`
  等会更新状态（`src/mcu_opcodes.cpp:1077-1079`），所以 hand 必须用
  `MCU_SetStatus*` 写出与 ROM 段末指令相同的 N/Z/C/V（逐例程在 §4.3 列）。
- **中断窗口**：段内不轮询 `MCU_Interrupt_Handle`（`src/mcu.cpp:1444-1445`）。
  ROM 在这些例程内有 IML=7 包裹（`BSET_ORC #0x0700/BCLR_ANDC #0xf8ff`，
  如 `0x10cf-0x10da`、`0x535e-0x5389`、`0x1ad3/0x1aed`）。hand 段应保持 IML 语义：
  入口备份 SR、置 IML=7，出口恢复（对应 B 路线的 R5 wrapper）。**待验证**
  （§5 R1/R2）。

---

## 4. 原生设计

### 4.1 `Voice` 结构（AoS + SoA 合并；字段 ← H8 偏移）

```cpp
namespace mk2c {   // 与 08 §4.3 的 Engine 同一命名空间（单层，兼容 C++11）

constexpr int kMaxVoices = 256;   // 索引 0..255；255 保留为 0xff 哨兵（§0.5）
constexpr int kMaxNotes  = 256;   // 描述符池

struct Voice {                    // 一个 0x12a AoS + 其 SoA 视图（内存布局不要求等同）
    // ---- 身份 / 池 / 链（H8: P-2, a368/a384/a3a0/a3bc/a3d8/a3f4/a410/a4b4/acf2/ad0e/d0e0/d15c/d0a8/d0c4）----
    uint8_t  slot;                // P-2（0..254）
    uint8_t  part, desc_index;    // a368 slot→part / a384 slot→描述符索引（11 §1.6 B2）
    uint8_t  pcm_ch;              // ad0e：0xff=free
    uint8_t  free_flag;           // a3a0：0x94=free（bit7=1）；分配（note_fill）时清 0
    uint8_t  active;              // a4b4
    uint8_t  mask_acc;            // acf2
    uint8_t  cmd;                 // d0e0：0 idle / 2 / 4
    uint8_t  irq_pending;         // d15c
    uint8_t  st_a3bc;             // a3bc
    uint8_t  chain_prev, chain_next;   // a3f4 / a410
    uint8_t  start_prev, start_next;   // d0a8 / d0c4
    uint8_t  free_next;           // a3d8（仅 free 栈）
    // ---- 插值器 / 包络（S = P-0x80）----
    struct Rec { int16_t w[17]; } rec0, rec1;  // S+0x00 / S+0x22（0x22B 各）
    uint16_t f_s44;               // S+0x44
    uint8_t  flag_steal;          // S+0x45 bit7
    int16_t  f_s46, f_s48, f_s4a, f_s4c, f_s4e, f_s50, f_s52, f_s54, f_s56, f_s58,
             f_s5a, f_s5c, f_s5e, f_s60, f_s62, f_s64; // S+0x46..0x65 候选标量
    // ---- PCM 镜像字段（S+0x66..0x7e）----
    uint16_t f_s66;               // 0/0xffff 哨兵
    uint8_t  pcm_67;              // S+0x67
    uint16_t pcm_pitch;           // S+0x68 → br+0x1a
    uint16_t pcm_36;              // S+0x6a → br+0x36
    uint8_t  pcm_05, pcm_09, pcm_0d; // S+0x6c/6d/6e
    uint8_t  voice_busy;          // S+0x6f
    uint16_t pcm_06, pcm_0a, pcm_0e, pcm_1e; // S+0x70/72/74/76
    uint8_t  lfo_a, lfo_b, lfo_c, lfo_d;     // S+0x78..0x7b
    uint8_t  algo, sel02;         // S+0x7c / 0x7d
    // ---- 主块（M = P..P+0xaa）----
    uint16_t state[3];            // P+0/2/4：0x16 idle、0x0e/0x10 run、0x12/0x14 过渡、0x0c
    uint16_t f_06, f_08, f_0a;    // P+0x06/08/0a（计数器/step）
    uint16_t f_0c, f_0e, f_10, f_12, f_14, f_16, f_18;
    uint16_t f_1a, f_1c, f_1e;    // 含 0xb5 哨兵（P+0x1a/+0x1e）
    uint16_t f_20, f_22, loop_start, loop_end; // P+0x20../+0x24/+0x26
    uint8_t  dpcm_lo, f_29, f_2a, f_2b, f_2c, f_2d; // P+0x28..0x2d
    uint16_t tone_rec;            // P+0x2e（0x8048+n*0x70）
    uint16_t tone_val;            // P+0x30（0x8748+cecc）
    uint16_t f_34, f_36, f_38, ratio, f_3c, f_3e, f_40, f_42, f_44p, f_46p, f_48p;
    uint8_t  f_4a..f_53;          // P+0x4a..0x53 混合
    uint16_t filt_54, filt_56, f_58, f_5a, f_5c, f_5e;
    uint8_t  f_60..f_6f;          // P+0x60..0x6f（含 0x65 flag、0x66 mode=3、0x68=0x40）
    uint16_t f_70, f_72, f_74, f_76, f_78, f_7a, f_7c, f_7e;
    uint16_t coef80, coef82, coef84, coef86, coef88, coef8a, coef8c, coef8e,
             coef90, coef92, coef94, coef96;  // P+0x80..0x96（DSP 系数，C16/C17 目标）
    uint8_t  ep98, ep99, ep9a;    // P+0x98/99/9a（EP 页）
    uint8_t  tone_idx;            // P+0x9b（SoA ce78 的物化副本）
    uint16_t ptr9c, ptr9e, ptr_a0; // P+0x9c/9e/a0（sample/tone/dpcm 指针）
    uint8_t  f_a2, f_a3;          // P+0xa2/0xa3
    uint8_t  frac_en;             // P+0xa4
    uint16_t frac_acc;            // P+0xa6
    int16_t  pitch;               // P+0xa8
    // ---- materialize 源（SoA，0x53eb 拷入 P+0x98..0xa0）----
    uint8_t  cfe4, d038, d08c, ce78, cecc, cf90;
    uint16_t cfac, d000, d054;
    // ---- 搜索/混音辅助（SoA 语义未证字段，保留 TODO）----
    uint8_t  a34c, cee8, cf04, cf20, cf3c, cf58_lo; // cf58 是字数组：按需 16-bit
    uint8_t  d0fc, d118, d134;
};
```

字段命名约定按 `mk2cpp/docs/02_conventions.md`：已证实字段用语义名，未证实用
`f_<offset>` 并 `TODO`。`Voice` 结构建议 `static_assert(sizeof(Voice) <= 512)`，
但**不要求**与 0x12a 二进制兼容；兼容由边界镜像/代理完成。

### 4.2 `VoicePool` / `NotePool`

```cpp
struct NoteDesc {                 // 建议原生化/迁 page6（(dp,0xa1e2) 指 part 记录，不指它；11 §1.2）
    uint8_t  next, prev;          // a250 / a26c
    uint8_t  state;               // a288（0 idle / 2 release / 0x94）
    uint8_t  flags;               // a2a4
    uint8_t  voice;               // a2dc
    uint8_t  part_age;            // a314
    uint8_t  bits;                // a330
    uint8_t  f2f8;                // a2f8
    uint8_t  f2c0;                // a2c0
};

struct VoicePool {
    std::array<Voice, kMaxVoices> v;          // AoS+SoA 合并
    std::array<uint8_t, kMaxVoices> free_next;// a3d8
    uint16_t free_head, free_tail;            // a42f / a430
    uint16_t count;                           // a42d（0..255）
    int16_t  shortfall;                       // a42c（有符号字节）
    uint16_t live_cur, live_next;             // d158 / d15a（voice 索引，0xffff=无）
    uint8_t  mask_cur[32];                    // d150/d152 扩展（bit N=voice N）
    uint8_t  mask_pend[32];                   // d154/d156 扩展
    std::array<uint8_t, kMaxVoices> d1ac;     // 固定 32 项、非 slot 索引（11 §1.1；残余 B6）
    std::array<uint8_t, kMaxVoices> d435;     // 现为 write-only，可保留 32
    std::array<NoteDesc, kMaxNotes> notes;    // 或 SRAM 镜像（见下）
    // 分配/释放/检索
    int  alloc();                             // 0xffff=失败
    void free(int slot);
    void note_on(uint8_t note, uint8_t part, uint8_t vel);
    void note_off(uint8_t note, uint8_t part);
    void update_frame();                      // 每 FRT3 tick: mask_acc + ts_scan
    void irq_end(uint16_t full_slot);         // IRQ0: irq_pend[slot]=0xff
};
```

**驻留决策（推荐，11 §1.2）**：必须留页 0 SRAM 的是 **part 记录区 `[0x8048,0x8718)`
（16×0x70，C）**；9 个描述符数组 `a2xx` 按 256 项 = 2304 B 可原生化并可迁 page6
（R18 方案），或扩到页 0 空闲区。两种都需要实现时用静态占用检查选定。

### 4.3 hand API 列表（入参/返回/周期/镜像）

L0 下每个 PC 一个函数（一次调用 = 一条 H8 指令）；下表「API」是例程语义单元与命名
约定，整例程周期列（下界）供 L1 评估用。`in/out` 以 H8 寄存器契约给出，wrapper 负责
`mcu.r[]` 编解码与 `rts/ret`。

| API（建议名） | 替换 PC | in | out / 标志 | 周期 | 镜像 |
|---|---|---|---|---|---|
| `pool_init()` | 0x040462 | — | — | ≥936 | a42d/a42f/a430/a3d8 |
| `init_desc()` | 0x041220 | — | — | ≥240 | d15c[]=0；n=28 时 AoS+0/2/4=0x16 |
| `init_params()` | 0x041266 | — | — | ≥648 | `P-26` |
| `pcm_chan_init()` | 0x041333 | — | — | ≥1092 | PCM 0x3c/3d、reg 0..3 |
| `maint_reset32()` | 0x04329F | — | — | ≥900 | d435/d1ac（留 SRAM 时） |
| `note_fill(slot)` | 0x000F86 | r1=slot | r0/r1（原样）；clobber r2-r6 | ≥1728 | 13+ SoA 字段 |
| `note_scan_a(desc)` | 0x0013A9 | r2=desc, r1=note, r3? | r0（rts 时） | ≥144 | a288/a314/a330 |
| `note_kill_a(desc)` | 0x001459 | r2=desc, r3=part? | r0 | ≥288 | a288/a2a4/a3bc/a4b4 |
| `note_scan_b(part)` | 0x00151E | r3=part? | r0 | ≥336 | a240/a2a4 |
| `alloc_scan(part)` | 0x00157D | — | r0 | ≥228 | a42c/a210 |
| `note_disp(part)` | 0x0015D1 | r3=part | disp | ≥204 | — |
| `note_cmd(part)` | 0x0015FB | r3=part, r2=desc? | r0 | ≥672 | a1e2 等 part 标量 |
| `steal(part)` | 0x0016F4 | r3=part | r0（0/1） | ≥228 | a220/a288 |
| `key_on(part)` | 0x00173E | r3=part, r4=a200[part] | r0（0/1） | ≥372 | a200/a220 |
| `slot_select(mode)` | 0x00179C | r3=part, r6=0/1/2 | r0/r1 | ≥324 | (dp,0xa438) |
| `note_helper(desc)` | 0x0017ED | r2=desc | r0/r1 | ≥108 | a250/a2dc |
| `release_a(slot)` | 0x001823 | r1=slot | r1=slot；clobber r0,r2-r6 | ≥324 | ad0e/d0e0/a3bc/a4b4/a3a0/a3d8 |
| `release_b(slot)` | 0x00187E | r1=slot | 同上 | ≥288 | 同上 |
| `desc_setup(part)` | 0x0018CE | r2=desc, r3=part | r0/r1 | ≥456 | a250/a288/a2c0/a2dc/a2f8/a314/a330/a4aa |
| `link(slot, desc)` | 0x00194C | r1=slot, r2=desc, r3=part | r0 | ≥276 | a368/a384/a3f4/a410/a2c0/d0a8/d0c4 |
| `pool_pop()` | 0x0019AD | — | r0 | ≥84 | a42d/a42f/a430 |
| `free_voice(slot)` | 0x0019C4 | r1=slot | — | ≥336 | ad0e/a3bc/a4b4/a3a0/a3d8/a2c0/a2dc |
| `cleanup_seq()` | 0x001A4D | — | — | ≥120 | ce3f |
| `mask_acc()` | 0x001AD3 | — | — | ≥108 | a4b4/acf2 |
| `unlink(desc)` | 0x001B23 | r2=desc | — | ≥132 | a220/a250/a230/a26c |
| `H1(slot)` | 0x001B44 | r1=slot, r0=0 | r3 | ≥252 | a2c0/a3f4/a410/d0a8/d0c4 |
| `H3(desc)` | 0x001B90 | r2=desc, r3=part | — | ≥108 | a288/a2a4/a200 |
| `H2(slot)` | 0x001BAD | r1=slot, r2=desc, r3=part | — | ≥312 | a2c0/a288/a250/a210 |
| `pcm_start(slot)` | 0x00516C | r1=slot | — | ≥252 | d15c/P+0/2/4/+26/+30 |
| `pcm_dispatch()` | 0x0051CC/0x0051D0 | —（循环直至空） | 不返回（回事件框架） | ≥348/轮 | d15c/d0e0 |
| `pcm_stop(voice)` | 0x005238 | r1=slot（r0=desc?） | — | ≥480 | ad0e/state |
| `pcm_play(voice)` | 0x0052CB | — | — | ≥708 | d158/d15a/d150..d157 |
| `irq_service(slot)` | 0x0025F0 | r1=slot | — | ≥348 | P+0/2/4/+26/+30 |
| `svc_math()` | 0x002669 | — | — | ≥636 | PCM reg10 |
| `note_on_setup(slot)` | 0x00272E | r0=P | — | ≥684 | cdfe/a2 等 |
| `tone_fields(P, tone)` | 0x003580 | r0=P, r5=tone | — | ≥588 | REC0/1、P+0xa8 |
| `gate_setup(P, tone)` | 0x003615 | r0=P, r5=tone | — | ≥708 | cdc6、P-115 |
| `interp(slot, mode)` | 0x003709/0x0038B0 | r0=P, mode | — | ≥396+≥1260 | REC0/1 输出 |
| `cross_copy(P0,P1)` | 0x003A9E | r0=P, r2=P | — | ≥852 | cdc6、AoS 字段 |
| `ts_scan()` | 0x005869 | — | — | ≥540 | ad2a/d178/d17a |
| `coeff_calc(slot, P)` | 0x005998 | r1=slot, r0=P | — | ≥2604 | P+0x86..0x96 |
| `coeff_copy(dst,src)` | 0x005D6E | r0=P, r2=P | — | ≥276 | P 字段 |
| `voice_search(part)` | 0x045CBE | r4=part | (part 混音结果) | ≥1092 | a368/ad0e 只读 |
| `irq0_handler()` | 0x000522 | —（硬件 IRQ） | rte 语义 | ≥108（0x522-0x53D 执行集 9 条） | d15c、0xE03E ack |

### 4.4 PCM / GT 边界 API（建议新增）

为绕开「效果簇 0x3e=28..31」与「voice slot 28..31」在扩展模式的冲突（§0.6、
`src/pcm.cpp:131-152`），建议在 GT `pcm.cpp` 增加：

```c
// 原生 voice 参数写：slot 0..254，reg 语义与 0xE000 窗口一致
void PCM_WriteVoiceReg(uint16_t slot, uint32_t reg, uint32_t value);
// 原生 mask 写（同时更新 current/pending，替代 readback latch 语义）
void PCM_SetVoiceMask(const uint8_t mask[32]);   // bit N=voice N
uint16_t PCM_GetIrqSlot(void);                   // = PCM_ReadExt(0x20)
void PCM_AckVoiceIrq(void);                      // = PCM_Read(0x3E)
```

- voice 侧 hand 代码只调这些 API（或 `PCM_Write` 的 0..27 路径）；
  effect 簇 ROM 的 `0x3e=0x1c..0x1f` 继续走 stock remap 到 `PCM_EFF_BASE`，
  互不冲撞。
- 若不加 API，则必须保证 hand 永不通过 `0x3e` 选 slot 28..31（即 28..31 不可用，
  等效 252 声）——不推荐。
- `reg_slots` 扩展公式/`cycles` 钳/`EFF_BASE`/ext 窗口 GT 已具备（R10、todo 阶段 1），
  本 API 只是给 hand 一个无歧义的 slot 通路。

---

## 5. 风险

| # | 风险/问题 | 假设 | 验证方法 |
|---|---|---|---|
| R1 | **L1 段内不轮询中断**导致 FRT2/FRT3/MIDI 延迟 = hand 段时长；中断延迟可能改变时序（L0 无此风险） | ROM 在这些例程内已 IML=7 或行为对延迟不敏感 | L1 仅限 IML=7 短例程（09 §4.3）；n=28 先做 A/B trace（默认 vs hand）；O2/O3（isr≈1382、pc=0492） |
| R2 | 周期记账用「指令数×12」下界，未含未执行臂/分支惩罚（GT 本身也为平坦 12，`src/mcu.cpp:1457`） | ROM 全臂指令数可静态枚举（h8lift 输入 map 已有 15999 PC） | 用 `mk2cpp/out/h8part/map.csv` 取全臂数，替换 §1 下界；对上 O5 的 cycle 窗口 |
| R3 | **目标 256 vs 妥协上限 255**：`0xff` 哨兵占 slot 255；GT `PCM_MAX_VOICE=255`（`src/pcm.h:27`） | 阶段性有效 N≤255（非最终目标）；`std::array<Voice,256>` 仅容量；真 256 需改哨兵/计数（10 D1） | `-voices:255` 长跑（验到妥协上限）；slot 255 永不分配（alloc 断言） |
| R4 | 效果 `0x3e` 冲突（§4.4） | 新增原生 PCM API 可零补丁解决 | 加 API 后默认路径 0-diff（200-202M trace）；`-voices:64` 下效果听感对照 |
| R5 | `0x5998 coeff_calc` 等定点数学重写错漏 → 音色/音量静默偏差；其 ROM 表（`0x7218/0x9740/0x9060`）在 ROM 页，C++ 可读 | 逐位移植 + int16/int32 语义（MULXU/DIVXU 饱和规则） | 单 slot 对照：同输入下 ROM 执行 vs hand 的 `P+0x86..0x96` 逐字节相等（co-sim 微测试） |
| R6 | **C7 `0x272e`/C16 区间上界**未精确到指令（rts 位置仅按 trace 推断） | 用全臂反汇编（h8dasm + map）钉死区间的 rts/入口 | 重跑 h8dasm 全 ROM 静态反汇编该区间；flow_main 前驱验证 |
| R7 | part 记录区 `[0x8048,0x8718)` 与页 0 24KB 预算；`(dp,0xa1e2)` 是 part 记录指针（11 §1.2） | 记录区必须留 SRAM；`a2xx` 描述符池 2304B 可原生化/迁 page6 | 静态占用图（同 polyphony_256 §2.1 方法） |
| R8 | **`0x0f86` 外部调用者缺失**（flow_main 无静态边） | 事件/指针分派；hand 接管入口仍正确（入口 PC 是分派目标） | `-tracepc` 在 note 事件处抓实际进入 PC；静态扫描 `jsr/jmp rN` 注册表 |
| R9 | `PCM_Update` 256 voice ~9× DSP（`src/pcm.cpp:587,1164,1672`） | 现代 CPU 可承受，或需要 native 混音降载 | 实测占空比；O2 心跳维持 |
| R10 | `d15c` 溢出：IRQ0 若仍留 ROM，`&0x1f` 对 slot≥32 会写错 entry（`0x052D`） | hand 接管 IRQ0（本规格默认） | O7；n=64 时注入 slot 40 结束事件，检查 `irq_pend[40]` |

**实现顺序**：以 09 §5.5 的切片顺序为准（PCM enable/disable flush → mask 累积 →
pool-init → alloc/free → materialize/PCM 写路径 → …）；本节 A/B/C/D 分组是语义闭包
视图，不另排期。

---

## 6. 待定项

| # | 待定项 | 假设/方向 | 验证方法 |
|---|---|---|---|
| Q1 | `0x15d1` 的 `@r6+0x1bfe`（:2148）与 R18「闭包内无 @r6/@r7 访存」矛盾；`0x1bfe` 表落在哪个地址空间待定 | **已闭合（11 §1.5，Strongly supported）**：`0x1bfe` 是 ROM1 16 项 per-part 指针表（tp=0，`<0x8000`→rom1）；`0x15d1` 不在 R18 闭包（闭包从 `0x15e0` 起） | `dasm:2142-2148`；`src/mcu.cpp:707-709`；R18 碎片表 |
| Q2 | `r2d95/r2e83` 的语义与真实 rts 边界 | 按 R11「外部例程」整体搬迁 | 全 ROM 反汇编 + 静态可达；逐条与 gen 实现对照 |
| Q3 | `0x42b8a` 是 H 还是 R（输入 r0=P，清 `P-108/P-74`） | **已闭合（11 §1.4，Strongly supported）：判 H**——2/4 写点是 AoS P-相对解引用，n>28 无 SRAM P；调用链 `0x41315` 带 r0=P 确认（Confirmed） | `dasm:13478-13482/13294`；`ce36/ce38` 非 per-voice |
| Q4 | `d1a6/d1ac` 是否 per-voice（32 宽） | **已闭合（11 §1.1）**：`d1a6` **Confirmed** 非 per-voice（索引 `(dp,0xd1a4)&1`，仅 2 项）；`d1ac` **Strongly supported** 固定 32 项、非 slot 索引；两者不随 N 扩 | `dasm:3240/3279/13866-13876`；残余调用点下标出处 = 11 B6 |
| Q5 | `0x43695/a3/c3/d5` d1ac helper 族是否被 slot>31 调用 | **已闭合（11 §1.1，Strongly supported）**：已执行调用点索引仅 0..6 常量或固定 ROM 值；理论下标越界风险仅存于未执行调用点 | 残余 = 11 B6（全量审计 `jsr #0x36xx` 调用点 r0 出处） |
| Q6 | AoS `P+0x06..0x96` 未命名字段的**真实读写者**（部分只有聚合证据） | 逐字段动态覆盖 trace（长 demo + MIDI 压测） | 全写/全读覆盖表；缺读字段可安全丢弃？——未证前全部保留 |
| Q7 | `Voice` 原生后，`0x2f17 TST @r1+0xacf2`（PCM program path）由谁读 | **已闭合（11 §1.3，Strongly supported）**：`0x2f17` 在例程 `0x2e85`（C9；蹦床 `0x2e83`），由 ts_scan `0x5869` 经 `0x591c/0x5980` 调用 | 约束 = 11 B7：C9（读者）与 mask_acc `0x1ad3`（写者）须同批原生化 |
| Q8 | 快照/哈希对照：`pcm_t` 已因阶段 1 不兼容；hand 化后 `sram` 视图改变 | 仅 O1–O10 + 音频 null 作为 M4 oracle；不做逐字节 SRAM 回归 | 10 的 oracle；n=28 音频 null；O9 默认模式回归（hand 激活契约已冻结：`-mk2cpp` 下含 n=28，09 §3.2） |
| Q9 | `0x0f86` 的外部调用者（R8）与 `0x0f62/0x0f64` 前导块的关系 | 事件/指针分派 | `-tracepc` 抓进入 PC；全臂反汇编 |
| Q10 | C7 `0x272e`、C11 `0x3615`、C9 `0x2e85` 的精确 rts/区间（R6） | 用全臂反汇编钉死 | h8dasm 全 ROM 静态反汇编 + flow_main 前驱 |
| Q11 | Note 池驻留方案与页 0 静态占用（R7） | **已闭合（11 §1.2，Confirmed）**：`(dp,0xa1e2)` 是 per-part 0x70 记录指针（ROM1 `0x1bfe` 表），须留页 0 的是 part 记录区 `[0x8048,0x8718)`；`a2xx` 描述符池可原生化/迁 page6 | `a1e2` 全读/写点枚举（8 读 2 写）；表字节 `build/rom1.bin[0x1bfe..0x1c1d]` |
| Q12 | `0x43695` 族 + `0x7cf/0x2052/0x406e4/0x46621` 等 R11 闭包外残留点的最终归类 | 逐个定 H/M/R | 全臂反汇编 + 动态确认 |

**仍未闭合（截至 2026-09-11）**：Q2、Q6、Q8、Q9、Q10、Q12 及 slice-2 新发现项
B6（`d1ac` 未执行调用点下标出处）、B7（`acf2` 与 C9 的成对性）；逐条见
[11 slice-2 pool 规格](11_slice2_pool_spec.md) §1.6/§6。已闭合项不再跟踪：Q1/Q3/Q4/Q5/Q7/Q11。

---

## 附录 A — 证据索引（行号为 2026-09-11 工作树核对）

- `tools/baselines/dasm_full.txt`：`0x0f86`(:1238)、`0x1188`(:1583)、`0x13a9`(:1934)、
  `0x1459`(:1970)、`0x151e`(:2053)、`0x157d`(:2091)、`0x15d1`(:2142)、`0x15fb`(:2181)、
  `0x16f4`(:2329)、`0x173e`(:2364)、`0x179c`(:2421)、`0x17ed`(:2488)、`0x1823`(:2524)、
  `0x187e`(:2577)、`0x18ce`(:2617)、`0x194c`(:2725)、`0x19ad`(:2784)、`0x19c4`(:2801)、
  `0x1a4d`(:2867)、`0x1ad3`(:2893)、`0x1b23`(:2941)、`0x1b44`(:2970)、`0x1b90`(:3015)、
  `0x1bad`(:3040)、`0x25f0`(:4259)、`0x2669`(:4296)、`0x272e`(:4359)、`0x2d95`(:5452)、
  `0x2e83`(:5664)、`0x3580`(:6615)、`0x3615`(:6752)、`0x3709`(:6913)、`0x37f2`(:6964)、
  `0x38b0`(:6978)、`0x39b1`(:7107)、`0x3a9e`(:7129)、`0x40462`(:12630)、`0x40508`(:12680)、
  `0x41220`(:13219)、`0x41266`(:13245)、`0x41333`(:13307)、`0x4329f`(:13840)、
  `0x45cbe`(:15902)、`0x45cc0`(:15905)、`0x45cde`(:15944)、`0x516c`(:10604)、
  `0x51d0`(:10634)、`0x5238`(:10693)、`0x52cb`(:10747)、`0x53eb`(:10918)、`0x546e`(:10968)、
  `0x54fb`(:11040)、`0x5533`(:11058)、`0x5626`(:11133)、`0x564a`(:11154)、`0x5671`(:11168)、
  `0x5869`(:11212)、`0x58d7`(:11286)、`0x5998`(:11433)、`0x5d6e`(:12062)、`0x4aca9`(:17564)。
- GT：`src/mcu.cpp:161-163`（RAM_SIZE/SRAM_SIZE）、`:658`（b_ram）、`:703,737`（页0 SRAM）、
  `:843-846,1064`（page6）、`:1451-1457`（分派 + cycles+=12）、`:1993-2004`（-voices 解析）、
  `:1658-1668`（MCU_PatchROM 空挂点）；`src/pcm.h:27-29,35-37`；
  `src/pcm.cpp:82-103`（mask 写）、`:131-152`（select/0x3f）、`:215-220`（latch）、
  `:288-308`（ext）、`:587-590,1164,1672`（reg_slots/voice 循环/cycles 钳）；
  `src/mcu_opcodes.cpp:571-580`（disp16 寻址）、`:311-314,345-349`（rts/ret）、
  `:1011-1039`（BSET/BCLR）、`:1041-1079`（MOVG 状态）；调度见 `src/mcu.cpp:1444-1457`。
- docs：voice_memory_map §1-§9；voice_bounds_inventory §1.1-§4；task_irq_map §0-§4；
  mk2_polyphony_256 §6.3-§6.6；polyphony_256_feasibility §1-§6；
  polyphony_256_todo R11/R12/R15/R16/R17/R18。
- `flow_main.txt` 直接调用边见 §1 各行（如 `00001106→00000f86`、`00005346→0000546e`）。

