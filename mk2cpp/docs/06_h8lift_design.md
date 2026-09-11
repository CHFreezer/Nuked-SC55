# 06 h8lift 设计（ROM → C++ 反译器）

> **实施记录（2026-09-11）**：`tools/h8lift/h8dec.c` 已实现，`--check-decode` 对
> `mach_main.txt`+`mach_vec.txt` 全量 **9235 条 0 mismatch**（含 GT FIXME 的 ore=4
> word ×5、ore=5 byte ×2）。发现两处 GT↔h8dasm 语义差（以 GT 为准）：
> (1) 银行映射：h8dasm 把所有非 rom1 当地 rom2，GT 的 5/10/11 页是 SRAM、6/7 扩展
> RAM、12/13 NVRAM（基线只到 cp0/cp4，二者一致）；
> (2) 非法字节：h8dasm 报消耗长度，GT 走 `MCU_ErrorTrap`；h8dec 返回 `valid=0,len=0`。
> 全 ROM 扫描 294,912 PC 中 0 字段不匹配，差异仅为上述语义。发射器（切块/emit/接线）
> 待实现，接口与步骤见 §3/§5。

> **架构修订（2026-09-11，用户指令）**：产物集成进 GT（`mk2cpp.h`），**不建独立 exe/运行时**。
> 本文 §3/§6 的 `Ctx&`/`BlkFn` 草案保留为背景；实施按 `01_architecture.md` 修订 ABI：
> 翻译代码直接操作全局 `mcu` + GT `MCU_*` 辅助函数，**每次调用恰好一条指令**，
> 未翻译 PC 由 GT 回退 `MCU_ReadInstruction()`。块级批处理/`irq_abort` 仅作后续优化。

状态：设计定稿（M1 可实现）。
范围：SC-55mk2 **主 H8/532**（rom1/rom2）执行面与其静态可达面的直译；子 MCU（M3）另见
`03_sm_design`（未写前不实现）。语义 oracle 一律是 GT（`../src/`），不是 h8dasm（T2）。
本文所有数量结论均由本仓库本地 fixture 实测得出，命令与来源见 §5/§6。

关键输入（本地 fixture，不入 git，来源见 `../../tools/baselines/README.md`）：

| 输入 | 内容 | 实测规模 | sha256（本文写作时） |
|---|---|---|---|
| `build/rom1.bin` | 32KB 代码/数据 | 0x8000 | `8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042` |
| `build/rom2.bin` | 512KB 代码/数据 | 0x80000 | `a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0` |
| `tools/baselines/pc_main.txt` | 执行 PC 集（union of 3 runs） | 9217 | `ca20f6a01a54c9140c3db8cf3a01758752c486bf8cbb8bb6083ba1d9a19fba5c` |
| `tools/baselines/flow_main.txt` | 指令级转移边 | 22969 | `d2bb25192cbf3a355625c43106195a64a4c453f8c4baac576b84e12c4007709b` |
| `tools/baselines/pc_vec.txt` | 64 向量入口（不在 pc_main） | 64 | `d0d0abbbd6f415ff18a09a290e07ea2c5557a9c45cec7d1339870f8bad41c6d5` |
| `tools/baselines/mach_main.txt` | h8dasm 机器可读解码（PC 集） | 9217 行 | `4ac7367a0a40932f53ff3425c83ff065879f8dc41a25a7405ddadc965e72788a` |
| `tools/baselines/mach_vec.txt` | 向量入口补充解码 | 18 行（17 真实 + 默认 0xe4） | `e86f7380d9f61b9a2be51b464b2e62c30adbd56197ea2d61cf27865c55008bb1` |
| `tools/baselines/dasm_full.txt` | 反汇编文本（交叉核对用） | 9217 指令行 | `5cce21184d00edd7fc2d2d66335b622dd79b04a4d7dc48f4dcbd26fba8729a02` |
| `%TEMP%\opencode\r16_scan\` | 静态扫描会话产物（临时；再生成见 §4.4） | `r16_hits.txt` / `r16_reach.txt`（69 PC）/ `r16_align.txt` | 不入 git |

术语约定（与 `../../tools/docs/evidence_protocol.md` §6 一致）：

- `flat = (cp << 16) | pc`，仅用 20 bit；rom1 代码 = flat `0x000000-0x007fff`，rom2 代码 = flat `0x04xxxx`。
- **E** = 执行集；**V** = 向量入口集；**R** = 静态可达集；**S** = 静态扫描候选集；**D** = 数据/不可解码。
- "block" 指按**地址**切分的基本块，不指"函数"——同一块可被多个逻辑例程共享（§1.6）。

---

## 1. 基本块划分算法

### 1.1 总流程

```
h8lift rom1 rom2 pcset... flow data_ranges [--out outdir]
  (1) 读 ROM → 校验 sha256（§5）
  (2) h8dec：对 E∪V∪S 的每个起始地址解码 → sorted insn[]（§1.2）
  (3) parity check：h8dec 结果 vs mach_main/mach_vec 逐字段比对，任一 mismatch 即 abort（§1.2）
  (4) 边语义过滤：从 flow 只提取“静态后继”与“动态目标观测”（§1.3）
  (5) block start 集 → 扫过 sorted insn[] 切块（§1.4/1.5）
  (6) 重叠检测（§1.6）；间接目标注册表（§1.7）
  (7) 发射 C++ + map.csv + h8lift.lock（§2/§5）
```

所有中间容器都是 `qsort` 后的数组（key = flat），遍历顺序 = 升序；不允许哈希表迭代进入输出
（§5.3）。失败即写 `out/errors.txt` 并以非零码退出，**绝不"尽力猜一个块发出去"**。

### 1.2 解码器 h8dec 与 parity

h8lift 内嵌一个权威 C 解码器 `h8dec_decode(uint8_t cp, uint16_t pc, insn_t*)`，语义必须逐条对齐
GT `src/mcu_opcodes.cpp` 的操作数/长度规则，字段布局沿用 h8dasm（`tools/disasm/h8dasm.c:33-40`）：

```c
typedef struct {
    uint32_t flat;                 /* (cp<<16)|pc */
    uint8_t  len;                  /* 1..6；实测长度直方图 1:269 2:4141 3:2372 4:2034 5:333 6:68 */
    uint8_t  kind;                 /* 0 none, 1 call, 2 uncond, 3 cond, 4 ret, 5 reg-indirect,
                                      +6 trapa(exception), +7 sleep（h8dasm 未区分，h8lift 必须区分） */
    uint8_t  tpage; uint16_t toff; /* 静态目标（kind 1/2/3） */
    uint8_t  top, reg, siz, ocode, ore, ext;
    uint16_t fall;                 /* pc+len 的 pc 部分（cp 不变） */
} insn_t;
```

细节必须照抄而非"更正确"（T0 优先）：

- `cntjmp`（0x01/06/07，3 字节，`r[reg]--` 未下溢才 `pc+=disp`；06 需 Z、07 需 !Z）：
  `src/mcu_opcodes.cpp:372-443`；
- `0x11` 寄存器间接族（`ret` / `jsr via rN:rN+1`（**call 语义**，push pc/cp） / `jmp rN` / `jsr rN`）：
  `src/mcu_opcodes.cpp:338-371`；
- `bsr/bsr16/jsr/pjsr`：`src/mcu_opcodes.cpp:450/198/214`，压栈后跳转，`fall` 是返回续点；
- `trapa`（0x08）：`src/mcu_opcodes.cpp:185-196`，置 `trapa_pending`，由下一次
  `MCU_Interrupt_Handle`（`src/mcu_interrupt.cpp:64-92`）取向量——**是异常终止指令**，不是普通指令；
- `sleep`（0x1A）：`src/mcu_opcodes.cpp:126-129`，仅置 `mcu.sleep=1`，指令本身不跳转；
- `bset/bclr/orc/andc` 的 **immediate-vs-操作数** 分支、ocodes 24-31 的**位号非寄存器**、
  MOVF 0x90-0x9f 写方向、MULXU/DIVXU 字模式寄存器对、STC 方向：这些正是
  `tools/docs/mk2_polyphony_256.md:527-543`（H1-H5）已修的 h8dasm bug，h8dec 直接按 GT 实现，
  禁止沿用旧文本理解（`tools/docs/evidence_protocol.md:135-143`）。

**parity 是硬门**：发射前把 h8dec 对 `pc_main ∪ pc_vec` 的结果与 `mach_main.txt` 的
`len/kind/tpage/toff/top/reg/siz/ocode/ore/ext` 逐字段比较（`tools/disasm/h8dasm.c:305-318` 的格式），
任何一条不同 → 报 `decode_mismatch flat=... field=...` 并 abort。mach 文件本身已经过
`tools/verify/verify_dasm.exe` Phase A/B 交叉验证（`tools/baselines/README.md:90-105`：
boot 200K → A 0-fail 385 派发 / B 0-fail 80447；快照 400K → A 0-fail 1079 / B 0-fail 115568）。

### 1.3 边语义过滤（本固件的关键坑）

flow_main 的 22969 条边**不是**纯 CFG：它来自 VM 每条指令后的 PC 采样
（`tools/vm/h8vm_main.c:1038-1058`，`mcu.cycles += 12` 后写 `mcu.pc`；`logproc.c:74` 连接相邻行）。
中断在"指令间"发生，于是**任意指令**都可能在下一行出现"跳到 handler"的边。实测：

| src kind | seq 边 | non-seq 边 | 解读 |
|---|---:|---:|---|
| 0 none（7067 条指令） | 7067 | **4902** | non-seq 全部落到 **11 个**目的：`0x2b0 0x2d0 0x344 0x3d4 0x416 0x524 0x7b4e 0x7b5c 0x7b6a 0x7d8a 0x461ac` —— 这是中断注入（handler 首指令执行后的 PC），不是控制流 |
| 1 call（395） | 0 | 588 | ≈395 个静态调用目标 + ≈193 次"调用后立刻进中断" |
| 2 uncond（33） | 1 | 36 | 33 个目标 + 少量中断 |
| 3 cond（1463） | 826 | 2386 | 目标/直落 + 中断注入 |
| 4 ret（243） | 0 | 7093 | 全部为动态返回到 POP 值（+少量 ret 前中断） |
| 5 reg-indirect（16） | 7 | 63 | 动态目标 + `jsr` 续点 + 中断 |

**规则**：block 的静态后继只允许来自 **h8dec 的解码语义**（kind 1/2/3 的目标与直落、kind 1/4/5
的返回续点结构）；flow 的 non-seq 边只用于两件事：① 动态目标（ret/reg-indirect）的**运行时注册
表**；② 覆盖率的"实际走到过"证据。绝不把 non-seq 边一律当成 block start——否则会多出
4902 条伪边、把 handler 的第二个 PC 误当入口。实测该差异：naive（全部 non-seq 边做 start）
得 4838 块；按语义规则得 **2576 块**，覆盖全部 9217 条指令（最大块 80 条指令），后者才是准数。

### 1.4 block start 集合

按以下顺序构造 `is_start` 位图（flat < 0x100000，1MB）：

1. **trace entry**：`in_deg[flat] == 0` 且在 E 内（`tools/disasm/h8dasm.c:323-328` 同规则）；
2. **静态目标**：h8dec 对 kind∈{1,2,3} 的 `(tpage:toff)`；
3. **终止后续点**：任何终止指令（§1.5 表）的 `fall`（调用返回续点、条件直落、trapa/sleep 后续）；
4. **向量入口**：`pc_vec.txt` / `mach_vec.txt` 的 17 个真实入口（`0x16c 0x2ac 0x2cc 0x342 0x3d0 0x412
   0x522 0x1bff 0x7b4c 0x7b5a 0x7d88 0x414ab 0x414b0 0x414b4 0x414b7 0x414ba 0x414bd`）
   ——这些地址永远不会作为 trace 行出现（中断取向量后执行的是 handler 首指令，trace 记的是其**后**PC），
   所以必须由向量表补入（`tools/baselines/README.md:94-96`）；
5. **动态目标**：kind 4/5 的 flow 目标中，若 h8dec 能有效解码，则登记为 start（仅用于生成注册表与
   覆盖，不改变语义）；
6. **静态扫描入口**：`category == STATIC` 且通过边界检查（§1.6）的地址。

### 1.5 block 终止规则与语义

| 指令类（kind） | 例子 | 本块结束？ | 静态后继 | 发射形式 |
|---|---|---|---|---|
| 0 none | MOVG/ADD/... | 否 | fall-through | 继续同一函数 |
| 1 call | `bsr/bsr16/jsr/pjsr` | **是** | 目标 + `fall`（返回续点） | 压栈、设 pc/cp、`return RT_PC_SET` |
| 2 uncond | `bra/jmp/pjmp` | **是** | 目标 | 设 pc/cp、`return RT_PC_SET` |
| 3 cond | Bcc / cntjmp | **是** | 目标 + `fall` | 条件设 pc 后 `return RT_PC_SET`，否则 `return fall_flat` |
| 4 ret | `rts/rte/rtd` | **是** | 动态（栈） | POP 设 pc/cp、`return RT_PC_SET` |
| 5 reg-indirect | `jmp rN` / `jsr rN` / `jsr via` | **是** | 动态（注册表） | `RT_Indirect(...)`、`return RT_PC_SET` |
| +6 trapa | `trapa #n` | **是** | 向量（异常排队） | `RT_Interrupt_TRAPA(n)`、`return RT_PC_SET`，宿主下一次迭代取向量 |
| +7 sleep | `sleep` | **是** | `fall`（唤醒续点） | `c.sleep=1; return fall_flat`；宿主在 sleep 期间只 tick（§2.3） |

规则：

- **tail jump 不内联**：`jmp` 结束块，目标另有块函数；跨页 `pjmp/pjsr` 直接写 `c.cp/c.pc`。
- **call 不内联**：M1-M3 一律"压栈 + 跳块"；不做函数识别/参数恢复（那是 M4 `src/hand/` 的事）。
- **ret 目标动态解析**：值来自 `RT_PopStack`，无需静态表；flow 中观测到的返回目标只用于覆盖分析。
  `0x7c3f` 的 `ret (pop cp,pc)` 在 flow 中有 2011+ 条目标边，正是"共享返回点"的实例。
- **块共享**：按地址发射一次，多前驱只记录 `npred`；不存在"每例程一份拷贝"。

### 1.6 重叠 span 处理

本固件的真实情况是**共享入口**（同一段字节被多个逻辑例程/中断路径进入），而不是同一地址有两种
解码。实测：

- E 内无任何指令 span 相互覆盖（对 mach_main 的全量区间扫描：0 命中）；
- `pc_main` 与 `pc_vec` 的 64 个向量入口无一落入 E 内某指令的字节区间（0 命中）；
- r16 静态候选的 69 个 PC 经 `r16align` 边界检查（`hits_is_instruction_boundary`）全部为 1
  （`%TEMP%\opencode\r16_scan\r16_align.txt`，共 69 个 `boundary=1`、0 个 `boundary=0`）。

设计仍必须处理真冲突，规则如下：

1. `is_start` 建立后，对每个 start 检查是否落在另一条有效指令的 `[flat, flat+len)` 内部；
   同时用一份 `data_ranges.csv`（人工确认的数据区间，例如 rom1 `0x7086-0x714a` 曲线表、
   `0x7238-0x7248` 跳转表本身）禁止线性扫入。
2. 冲突按优先级裁决：**EXEC > VEC > STATIC > D**（依据 `tools/docs/evidence_protocol.md:20-27`
   的证据分级）。败者不发射：`cat=OVERLAP`，运行期走解释器兜底，并写
   `out/overlap_conflicts.csv`（flat、两个解码、优先依据）。
3. **永远不对同一字节区间发射两种解码**；也绝不在解码无效处强行发射（§4.5）。
4. 当前基线冲突应为 0；这是 M1 的验收断言之一（§6.5 M1b）。

### 1.7 间接跳转与跳转表

E 内 kind 5 共 **16 个站点**，flow 观测到 **69 个不同目标**，全部落在 E 内（0 个未翻译）。
其中 10 个是标准 switch 形态 `MOVG2 @rN+disp16 rN; jmp rN`（dasm_full 相邻行配对）：

```
0x311a->0x311e disp=0x6854 | 0x316e->0x3172 disp=0x686c | 0x396c->0x3970 disp=0x7238
0x41db->0x41df disp=0x760a | 0x4206->0x420a disp=0x7622 | 0x4d79->0x4d7d disp=0x78a6
0x4dca->0x4dce disp=0x78d6 | 0x5226->0x522a disp=0x522c | 0x5dc7->0x5dcb disp=0x5dce
0x6190->0x6194 disp=0x6196
```

`0x7238` 的表内容（rom1 字节，16-bit BE word）：`3972 39b5 39cc 39e0 3a30 3a54 3a57 0000`
（前 7 项是同页代码偏移，`0x0000` 终止；index 0 = `0x3972` = `jmp r2` 的直落，标准 switch）。
注意 `0x7086` 是**数据表**（`0000 000a 0014 ...` 步长 10），被 `0x35fa/0x3604` 读进 r2 做插值，
**其后没有 jmp**：把它定义为 `data_ranges`，防止静态扫掠把表字节当代码（这正是 4902 条中断边
之外的另一个"假代码"来源）。

翻译策略（三级）：

1. **运行时注册表（M1 必需）**：由 flow 生成 `(site, target) → BlkFn`；`jmp rN` 发射
   `RT_Indirect(c, site, c.r[n], /*is_call=*/0)`；`jsr rN` / `jsr via` 带 `is_call=1`
   （压栈由 `RT_Indirect` 完成，`jsr via` 还要用 `r[N]&0xff` 作 cp，`r[N+1]` 作 pc，
   见 `src/mcu_opcodes.cpp:350-357`）。查不到 → strict 模式断言 abort；mixed 模式落解释器并
   写 `out/unregistered_indirect.log`（§2.5）。
2. **静态表展开（M2，静态可达补齐）**：对每个 `MOVG2 @rN+disp16 rN; jmp rN` 站点，按
   `data_ranges` 终止符读出全部表项，作为"候选目标"加入 R 并预先注册——但运行时仍以寄存器值
   查表，不假设 index 一定命中候选集。
3. **页面语义**：表地址用的是 `r[reg]` 的页面（`MCU_GetPageForRegister`，reg<4 → dp，
   4-5 → ep，≥6 → tp；`src/mcu.h:208-215`），**不得把观测到的 dp=0 写死**（§7）。

### 1.8 确定性

- 输入解析（hex PC/边）→ 定长数组；`qsort` 比较器带完整 tie-break（flat，其次来源 id），
  不用 `qsort_r`、不依赖元素地址。
- 输出顺序 = flat 升序；文件名/函数名由 flat 派生（`blk_r1_000035F5`），不用自增序号，
  保证"新增一个块"只产生局部 diff。
- 不使用 `rand/time/环境变量/目录枚举顺序`；`--jobs` 若存在只影响执行速度，不影响输出字节。
- 所有文本用 `\n`（无 CRLF）、无 BOM、UTF-8；数值统一小写 hex、零填充宽度固定。

---

## 2. 生成 C++ 接口

### 2.1 Ctx 与 GT 状态

M1 采用 **状态零拷贝**：`Ctx` 就是 GT 的 `mcu_t`（字段见 `src/mcu.h:161-174`），好处是混合模式
（翻译块 ↔ 解释器）不需要任何同步代码，直接消灭一整类"状态漂移"bug。`src/rt/rt.h`：

```cpp
// M1: Ctx 直接别名/包裹 GT mcu_t；独立化推迟到 M2 之后
using Ctx = mcu_t;                    // extern mcu_t mcu;  src/mcu.h:174
// 独立化时（M2+）保持字段顺序一致，并加 static_assert(offsetof(Ctx, pc) == ...)
```

`cycles`、`interrupt_pending[]`、`trapa_pending[]`、`exception_pending`、`sleep`、`ex_ignore`
全部取自 `mcu_t`，不许另起一份。

### 2.2 block 函数签名与返回约定

```cpp
// src/gen/blocks_r1.cpp（自动生成，禁止手改）
enum { RT_PC_SET = -1 };              // 块已自行设置 c.cp/c.pc
typedef int (*BlkFn)(Ctx&);
int blk_r1_00000342(Ctx& c);          // r1 = rom1 页（cp0）；r2 = rom2 页（本例 cp4）
int blk_r2_00041220(Ctx& c);
```

- 返回 `-1`：块内已写 `c.cp/c.pc`（分支取走、jmp、call、ret、indirect、trapa、sleep 之后）。
- 返回 `>= 0`：直落，值是 **flat**（`(cp<<16)|pc`；同页直落时低 16 位即 pc）。宿主用
  `c.pc = (uint16_t)n;`（cp 不变）。
- 块**不负责**本次迭代最后的 `RT_PostInstr`（由宿主调用，见 §2.3 契约）；块内从第 2 条指令起
  的每条指令前执行 `RT_PreInstr`，若取中断则 `return RT_PC_SET` 并置 `c.irq_abort=1`。

### 2.3 每指令边界（复刻 GT 调度）

GT 的每次迭代固定为（`src/mcu.cpp:1327-1446`）：

```
[Pre: if (!ex_ignore) Interrupt_Handle(); else ex_ignore=0]      -> :1343-1346
[若 !sleep 执行 1 条指令]                                         -> :1348-1349
cycles += 12                                                      -> :1351
trace_write(0, cycles, cp:pc)                                     -> :1354-1372
demo/mocknote/snapshot 触发                                       -> :1374-1429
PCM_Update / TIMER_Clock / SM_Update / MCU_UpdateAnalog            -> :1434-1446
```

由于设备更新和中断只在**指令边界**发生，块的粒度可以大于一条指令，但**每条指令**都必须经过
Pre/Post。宿主循环（`src/rt/sched.cpp`）：

```cpp
void RT_RunOnce(Ctx& c) {
    RT_PreInstr(c);                       // GT :1343-1346（含 ex_ignore）
    if (!c.sleep) {
        int n = RT_Dispatch(c);           // 翻译块；或解释器单步兜底
        if (n >= 0) c.pc = (uint16_t)n;
    }
    if (!c.irq_abort) RT_PostInstr(c);    // GT :1351-1446（cycles/trace/设备）
    c.irq_abort = 0;
}
```

块内骨架（每条指令 = 语义 + `RT_PostInstr`；下一条前 `RT_PreInstr`）：

```cpp
int blk_r1_00000342(Ctx& c) {                        // 向量 41 入口（FRT2_OCIB）
    // I1  0342: STC r5 -> --r7   [bf 9d]  （宿主已完成本条 Pre）
    RT_Operand_PreDec(c, /*reg=*/7, /*size=*/1);     // siz||reg==7 -> r7 -= 2（mcu_opcodes.cpp:620-630）
    RT_Op_STC(c, /*reg=*/5, /*size=*/1);
    RT_PostInstr(c);
    // I2  0344: LDC #0x00 r5     [04 00 8d]
    if (RT_PreInstr(c)) { c.irq_abort = 1; return RT_PC_SET; }
    RT_Op_LDC(c, /*reg=*/5, /*size=*/0, /*imm=*/0x00);
    RT_PostInstr(c);
    // I3  0347: stm #0x7f        [12 7f]
    if (RT_PreInstr(c)) { c.irq_abort = 1; return RT_PC_SET; }
    RT_Op_STM(c, 0x7f);
    RT_PostInstr(c);
    // ... 直到终止指令；终止指令的 Post 由宿主做
    RT_Op_BSET_ORC(c, /*reg=*/0, /*size=*/1, /*opcode=*/0x48, /*operand=*/imm); // ...
    return RT_PC_SET;
}
```

要点：

- `sleep`：块在 `sleep` 后返回 `fall_flat` 并已置 `c.sleep=1`；宿主下一轮 `!c.sleep` 为假 →
  只 tick，直到 `RT_PreInstr → MCU_Interrupt_Handle` 清 `sleep` 并设向量（`src/mcu_interrupt.cpp:32`）。
- `irq_abort` 语义：发生在块内 `RT_PostInstr` 之后的下一条 `RT_PreInstr` 取走中断时；此时该条
  指令的 Post 已做，不能再让宿主 Post（否则 cycles/设备多跑一轮）。宿主循环按上表处理。
- 周期固定 +12/指令（`src/mcu.cpp:1351`，GT 的 `// FIXME` 保留），M4 若周期精确化另立里程碑。

### 2.4 dispatch 与混合模式兜底

```cpp
// src/gen/dispatch.cpp
extern const BlkFn g_blocks_cp00[0x10000];   // rom1 页（cp=0）
extern const BlkFn g_blocks_cp04[0x10000];   // rom2 页（cp=4；实测仅 cp0/cp4）
BlkFn RT_Lookup(uint8_t cp, uint16_t pc);    // 未翻译页一律返回 nullptr

int RT_Dispatch(Ctx& c) {
    uint32_t flat = ((uint32_t)c.cp << 16) | c.pc;
    BlkFn f = RT_Lookup(c.cp, c.pc);
    if (f) return f(c);
    RT_FallbackOne(c);                       // = MCU_ReadInstruction()（src/mcu.cpp:1078）
    return -1;                               // pc 已被解释器推进/改写
}
```

- **内存模型不重写**：M1 的 `RT_Read/RT_Write` 直接转发 GT `MCU_Read/MCU_Write`（§3），
  页映射/IO/设备语义 100% 一致（`src/mcu.cpp:663-1069`）。
- per-page 表：每个 0x10000 项 × 8B = 512KB；当前只材料化 cp0/cp4（1MB，可接受）。其余页
  `RT_Lookup` 一律 nullptr → 解释器；若运行期落在未材料化页且该页有代码，计为覆盖缺口而非崩溃。
- strict 模式（`--strict`）：`RT_Lookup` 未命中且 `flat ∈ E∪V` → abort（说明发射器漏块）；
  其他未命中（ROM 数据区被误执行）在 GT 里本来就会 trap/Unknown read，混合模式按 GT 单步，
  由此保证"永不比 GT 更聪明"。
- 解释器兜底逐条计数（`g_rt_fallback_count`、按 flat 直方图），供 `cover` 汇总。

### 2.5 间接目标注册表

```cpp
// src/gen/indirect.cpp
struct IndirectEdge { uint32_t site; uint16_t target; uint16_t hits; };
extern const IndirectEdge g_indirect_edges[];   // (site,target) 升序，来自 flow_main 过滤后
extern const uint32_t g_indirect_edge_count;

int RT_Indirect(Ctx& c, uint32_t site, uint16_t target, int is_call) {
    if (is_call) RT_PushStack(c, c.pc);         // jsr rN / jsr via：GT push pc(/cp)
    BlkFn f = RT_Lookup(c.cp, target);
    if (f) return f(c);
    if (rt_strict) RT_Fatal("unregistered indirect: site=%08x target=%08x", site, target);
    RT_LogIndirectMiss(site, target);           // out/unregistered_indirect.log
    c.untranslated = 1; c.pc = target; return RT_PC_SET;
}
```

- 表由 h8lift 从过滤后的 flow 生成，**升序去重**；`hits` 仅作诊断，不参与选择。
- strict 验收要求：E∪V 内的执行路径上 0 次 miss；当前 69/69 目标都在 E 内，预期 0。
- 未注册目标走 mixed 兜底并按 log 记；任何 miss 都是下一轮补块的输入（覆盖闭环）。
- `jsr via rN:rN+1`（未执行，H4 潜伏）按 `src/mcu_opcodes.cpp:350-357` 发射跨页调用
  （push pc、push cp、`cp=r[N]&0xff`、`pc=r[N+1]`）。

### 2.6 生成文件布局

| 文件 | 内容 |
|---|---|
| `src/gen/blocks_r1.cpp` | cp0 全部块，按 flat 升序，每块带 `// 0xAAAA: <text>` 注释 |
| `src/gen/blocks_r2.cpp` | cp4 全部块（16 进制 page 前缀命名 `blk_r2_0004xxxx`） |
| `src/gen/dispatch.cpp` | `g_blocks_cp00/g_blocks_cp04` 与 `RT_Lookup` |
| `src/gen/indirect.cpp` | 注册表 + `g_indirect_edges` |
| `src/gen/map.csv` / `src/gen/h8lift.lock` | 覆盖/复现元数据（§4/§5） |

命名沿用 `02_conventions.md` §1（`blk_<rom>_<flat>`）；已知语义例程另加
`// alias: voice_alloc (see hand/)` 注释，M4 由 dispatch 重定向。

---

## 3. 运行时 intrinsic/helper API（src/rt）

原则：**先 1:1 转发 GT**（保证 0 分歧），名字统一 `RT_` 前缀 + GT 原名一一对应，便于审阅；
M2 起再按 family 内联/消除全局 `operand_*`，每次内联都用 §6.2 的单家族测试作证。
头文件 `src/rt/rt.h`；实现 `src/rt/rt.cpp`（内存转发 `src/rt/mem.cpp` 也允许，见 `01_architecture.md`）。

### 3.1 内存与取指

```c
uint8_t  RT_Read   (Ctx& c, uint32_t addr);      // = MCU_Read        src/mcu.cpp:858（impl:663）
uint16_t RT_Read16 (Ctx& c, uint32_t addr);      // = MCU_Read16     src/mcu.cpp:876（addr &= ~1）
uint32_t RT_Read32 (Ctx& c, uint32_t addr);      // = MCU_Read32     src/mcu.cpp:885（addr &= ~3）
void     RT_Write  (Ctx& c, uint32_t addr, uint8_t  v);   // = MCU_Write   src/mcu.cpp:896
void     RT_Write16(Ctx& c, uint32_t addr, uint16_t v);   // = MCU_Write16 src/mcu.cpp:1071
uint8_t  RT_Fetch  (Ctx& c);                     // = MCU_ReadCodeAdvance src/mcu.h:192
uint32_t RT_GetAddress(uint8_t page, uint16_t off);       // = MCU_GetAddress src/mcu.h:184
int      RT_AddrErrorIfOdd(Ctx& c, uint32_t ea);          // = operand word 的奇地址异常
```

### 3.2 操作数读取/写入（静态化 operand descriptor）

GT 的 `MCU_Operand_General`（`src/mcu_opcodes.cpp:543-673`）把"寻址模式解析"和"opcode 执行"
放在一起。h8lift 在**翻译期**完成模式解析（reg/disp/type 是常量），运行期只保留与寄存器相关
的副作用，顺序必须一致：

```c
typedef struct {                 // 翻译期填好的常量部分
    uint8_t  type;               // 0 direct, 1 indirect, 2 absolute, 3 immediate
    uint8_t  reg;                // rN（direct/indirect）
    uint8_t  size;               // 0 byte, 1 word
    uint8_t  page_kind;          // PAGE_REG / PAGE_DP / PAGE_BR / PAGE_TP / NONE
    uint8_t  inc;                // NONE / DEC / INC （@-rN / @rN+）
    uint32_t ea_const;           // br 或 dp 的绝对偏移
    uint16_t imm;                // 立即数
} Opnd;

void     RT_Operand_Begin(Ctx& c, Opnd* o);           // 执行 pre-dec / post-inc / ep 计算
                                                       //  = mcu_opcodes.cpp:618-653
uint32_t RT_Operand_Read (Ctx& c, const Opnd* o);      // = MCU_Operand_Read  :486-509
void     RT_Operand_Write(Ctx& c, const Opnd* o, uint32_t v); // = MCU_Operand_Write :511-541
uint8_t  RT_PageForReg (Ctx& c, uint8_t reg);          // = MCU_GetPageForRegister mcu.h:208
```

页面语义硬约束：

- `(dp,xxxx)` 绝对 → dp 页（`src/mcu_opcodes.cpp:608-616`）；
- `(br,$xx)` 绝对 → br 页（`:589-596`）；
- `@rN` / `@rN+disp` / `@-rN` / `@rN+` → `MCU_GetPageForRegister`（dp/ep/tp 由 reg 决定，`:646`）；
- MOVF 0x80-0x9f → `tp<<16 + r6+disp8`（`:713-753`）；MOVL/MOVS → `br<<8 + imm8`（`:755-800`）；
- **运行期取值，禁止把 trace 中观测到的页常量折叠进生成码**（§7 风险 7）。

### 3.3 标志与 ALU

```c
void    RT_SetStatus      (Ctx& c, uint32_t cond, uint32_t mask);   // = MCU_SetStatus  mcu.h:332
void    RT_SetStatusCommon(Ctx& c, uint32_t val,  uint32_t siz);    // = MCU_SetStatusCommon mcu_opcodes.cpp:675
int32_t RT_ADD_Common(int32_t t1, int32_t t2, int32_t cb, uint32_t siz); // :72
int32_t RT_SUB_Common(int32_t t1, int32_t t2, int32_t cb, uint32_t siz); // :22
```

opcode family helper（名字与 `MCU_Opcode_*` 对齐；M1 先整段转发，M2 再逐族内联）：

```c
void RT_Op_MOVG_Immediate(Ctx&, const Opnd*, uint8_t ocode, uint8_t ore); // :825
void RT_Op_MOVG          (Ctx&, const Opnd*, uint8_t ocode, uint8_t ore); // :1041（d=(ocode&2)，读/写/XCH）
void RT_Op_CLR           (Ctx&, const Opnd*, uint8_t ocode, uint8_t ore); // :937（ore=子操作码）
void RT_Op_SHLR          (Ctx&, const Opnd*, uint8_t ocode, uint8_t ore); // :1219（ore=移位码）
void RT_Op_ADDQ          (Ctx&, const Opnd*, uint8_t ocode, uint8_t ore); // :1141（ore=+1/+2/-1/-2）
void RT_Op_MULXU/DIVXU   (...);                                           // :1325/:1354（寄存器对）
void RT_Op_BSET_ORC/BCLR_ANDC/BSET/BCLR/BTST/BNOTI/BTSTI (...);           // :875/:899/:1009/:1025/:923/:1111/:1097
void RT_Op_ADD/SUB/ADDS/SUBS/AND/OR/XOR/CMP/ADDX/SUBX (...);              // :1167/:1181/:1419/:1195/:1205/:1127/:1427/:1134/:1434/:1453
void RT_Op_LDC/STC (...);                                                 // :996/:1003
void RT_Op_MOVE/MOVI/MOVF/MOVL/MOVS/CMP_short (...);                      // :694/:703/:713/:755/:779/:802
void RT_Op_LDM/STM (...);                                                 // :154/:169
```

### 3.4 栈与控制寄存器

```c
void     RT_PushStack(Ctx&, uint16_t v);   // = MCU_PushStack  mcu.h:340（r7 奇→address error）
uint16_t RT_PopStack (Ctx&);               // = MCU_PopStack   mcu.h:348
uint32_t RT_ControlRead (Ctx&, uint32_t reg, uint32_t siz);       // = MCU_ControlRegisterRead  mcu.h:274
void     RT_ControlWrite(Ctx&, uint32_t reg, uint32_t siz, uint32_t v); // = mcu.h:217
uint32_t RT_GetVectorAddress(Ctx&, uint32_t vector);              // = MCU_GetVectorAddress mcu.h:203
```

### 3.5 中断/异常

```c
void RT_Interrupt_Handle(Ctx&);                       // = MCU_Interrupt_Handle   mcu_interrupt.cpp:64
void RT_Interrupt_Exception(Ctx&, uint32_t source);   // = MCU_Interrupt_Exception :40
void RT_Interrupt_TRAPA(Ctx&, uint32_t vector);       // = MCU_Interrupt_TRAPA     :51
void RT_ErrorTrap(Ctx&);                              // = MCU_ErrorTrap           mcu.cpp:176
```

### 3.6 调度与 trace 钩子

```c
void RT_PreInstr (Ctx&);   // if(!ex_ignore) Interrupt_Handle(); else ex_ignore=0;（返 1=取了中断）
void RT_PostInstr(Ctx&);   // cycles+=12; TracePc; demo/mocknote/snapshot; PCM/TIMER/SM/Analog（§2.3）
void RT_TracePc  (Ctx&);   // 格式必须与 GT trace_write(0,...) 一致（mcu.cpp:1363）
uint64_t RT_HashState(Ctx&); // cosim 周期哈希（SRAM/设备寄存器等由调用方范围决定）
```

### 3.7 命名对照（GT → RT）

| GT | RT | GT 位置 |
|---|---|---|
| `MCU_Read/Read16/Read32` | `RT_Read/Read16/Read32` | mcu.cpp:858/876/885 |
| `MCU_Write/Write16` | `RT_Write/Write16` | mcu.cpp:896/1071 |
| `MCU_ReadCodeAdvance` | `RT_Fetch` | mcu.h:192 |
| `MCU_Operand_Read/Write` | `RT_Operand_Read/Write` | mcu_opcodes.cpp:486/511 |
| `MCU_SetStatus` | `RT_SetStatus` | mcu.h:332 |
| `MCU_SetStatusCommon` | `RT_SetStatusCommon` | mcu_opcodes.cpp:675 |
| `MCU_ADD/SUB_Common` | `RT_ADD/SUB_Common` | mcu_opcodes.cpp:72/22 |
| `MCU_PushStack/PopStack` | `RT_PushStack/PopStack` | mcu.h:340/348 |
| `MCU_ControlRegisterRead/Write` | `RT_ControlRead/Write` | mcu.h:274/217 |
| `MCU_GetPageForRegister` | `RT_PageForReg` | mcu.h:208 |
| `MCU_Interrupt_*` | `RT_Interrupt_*` | mcu_interrupt.cpp:40/51/64 |

---

## 4. 覆盖率模型

### 4.1 map.csv schema

```csv
# flat,rom,fileoff,len,kind,cat,block_fn,block_start,rel
0000016f,rom1,00016f,3,0,EXEC,blk_r1_0000016f,0000016f,0
00000172,rom1,000172,3,0,EXEC,blk_r1_0000016f,0000016f,1
00000342,rom1,000342,2,0,VEC,blk_r1_00000342,00000342,0
...
00007086,-,-,-,-,DATA,,,            # 数据表，禁止解码
```

- `rom`：`rom1|rom2|-`；`fileoff`：rom1 = flat，rom2 = flat-0x40000（证据协议 §6）。
- `kind`：h8dec kind（§1.2，含 6=trapa、7=sleep）。
- `cat`：`EXEC`（执行集）`VEC`（向量入口）`STATIC`（静态可达且已发射）`DATA`（数据/未解码）
  `TRAP`（解码有效但语义为 GT trap）`OVERLAP`（冲突落选）`UNKNOWN`。
- `block_fn`：所在块函数；`block_start`/`rel`：块首 flat 与指令序号（`rel=0` 即块首）。

### 4.2 集合与指标

- `E` = pc_main ∪ pc_vec（∪ cosim 中新见 PC，若开启 `--extend-exec`）。
- `V` = 向量表 64 槽可解码入口（17 真实 + 默认 `0xe4`）。
- `R` = 从根（reset 向量 + 全部 V + E 的 trace entry + 注册的间接目标）出发，用 h8dec 有效解码
  做 worklist，走静态后继，遇终止/无效解码/`data_ranges` 停。R 里新增的可发射地址标 `STATIC`。
- `S` = r16 扫描候选（§4.4）。
- `D` = 无效解码或 data_ranges。

`cover` 输出 `out/coverage.md`：

```
exec_coverage   = |{E 且已发射}| / |E|                    （M1 目标 ≥ 99%，M2 = 100%）
vec_coverage    = |{V 且已发射}| / |V|
reach_coverage  = |{R 且已发射}| / |R|                    （M2 目标 100%）
fallback_rate   = 运行期解释器步数 / 总步数               （M2 目标 0）
unknown_reached = 运行期到达 DATA/TRAP/OVERLAP 的次数     （必须 0）
```

运行期 counter 由 `src/rt` 提供（`g_rt_fallback_count`、`g_rt_fallback_by_flat[]`、
`g_rt_indirect_miss[]`），cosim 结束时 dump 成 CSV 供 `cover --runtime` 读取。

### 4.3 静态扫描候选（S）的折叠方式

`S` 是**证据输入**不是发射依据：候选先过三关才可能进 `STATIC`——
① `r16align` 的指令边界检查（当前 69/69 通过）；② h8dec 有效；③ 与 E/V 解码无 span 冲突。
过不了关的候选标 `DATA/OVERLAP`，运行到则解释器，绝不被"扫到就发射"。
扫描候选的存在意义：M2 的"未执行但静态可达"路径补块、以及给 §7 的 RAM/自修改风险提供负面证据。

### 4.4 `%TEMP%\opencode\r16_scan` 再生方法

该目录是会话临时夹具（不入 git，`02_conventions.md` §4）。若已被清掉，用同目录 C 源码重建
（工具链与 `tools/disasm` 相同，clang；在 `%TEMP%\opencode\r16_scan` 内执行）：

```powershell
clang -O2 -D_CRT_SECURE_NO_WARNINGS -o r16scan3.exe r16scan3.c
clang -O2 -o r16align.exe  r16align.c
clang -O2 -o r16class.exe  r16class.c
clang -O2 -o r16reach2.exe r16reach2.c
clang -O2 -o r16final.exe  r16final.c
# 主扫描：原始 ROM 中 [f0-fb rN][hi lo] / [15][hi lo] 形态的 array-base 引用
.\r16scan3.exe ..\..\..\tools\baselines\dasm_full.txt ..\..\..\tools\baselines\flow_main.txt `
               ..\..\..\tools\baselines\pc_vec.txt . ..\..\..\build
# 边界/可达/归属复核
.\r16align.exe  ..\..\..\build ..\..\..\tools\baselines\dasm_full.txt  b_pcs.txt  r16_align.txt
.\r16reach2.exe ..\..\..\build entries.txt b_pcs.txt r16_reach.txt
.\r16class.exe  ..\..\..\build ..\..\..\tools\baselines\dasm_full.txt `
                ..\..\..\tools\baselines\pc_main.txt r16_hits.txt r16_class.txt
```

输出分类：`A`=已执行且在 dasm，`K`=已执行但不在 dasm 基线，`B`=未执行但解码有效，
`C`=未执行且解码无效（`r16scan3.c:226-234`）。h8lift 只吃 `B` 中同时进 `r16_reach.txt` 且
边界检查通过者。没有该目录不影响 M1（E/V 已足够）；M2 静态补齐前必须重新生成并在
`h8lift.lock` 记哈希。

### 4.5 不可解码/数据字节策略

1. 解码无效（trap 操作码，如 `MCU_Operand_NotImplemented` 覆盖的 0x09/0x0B/0x0F/0x16/0x17/0x1B/0x1F，
   或操作数模式非法）→ **不发射**，`cat=TRAP/DATA`；运行到即解释器，行为与 GT 完全一致
   （`MCU_Operand_NotImplemented → MCU_ErrorTrap`，`src/mcu_opcodes.cpp:131-134/689`）。
2. `data_ranges.csv`（人工确认）内的字节一律 `DATA`，静态扫掠禁止进入。
3. "解码有效"≠"该发射"：还要满足 §4.3 三关；宁缺毋滥。
4. 任何 `cat=DATA/TRAP` 的 flat 出现在运行 trace 中 → `unknown_reached++`，coverage 标红，
   着手补证据/补块，不允许静默继续。

---

## 5. 版本与可复现性

### 5.1 生成文件头（`02_conventions.md` §2 格式的细化）

```cpp
// GENERATED by h8lift 0.1.0+<git-describe> (<tool_sha256 前 12 位>) -- DO NOT EDIT
// rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
// rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
// inputs pc_main ca20f6a0... flow_main d2bb2519... pc_vec d0d0abbb... mach_main 4ac7367a...
// counts blocks=2576 instrs=9217 conflicts=0 emitted_at=BUILD (no timestamp)
```

`emitted_at` 写占位符 `BUILD` 而非当前时间：时间戳会破坏可复现字节比对（`fc /b`）。

### 5.2 h8lift.lock（JSON-ish，ASCII，升序）

```
tool=h8lift 0.1.0+<describe>
tool_sha256=<...>
rom1_sha256=... rom2_sha256=...
pc_main_sha256=... flow_main_sha256=... pc_vec_sha256=... mach_main_sha256=... mach_vec_sha256=...
r16_reach_sha256=<或 none>
config=strict=1,emit_comments=1,data_ranges_sha256=<...>
blocks=2576 instrs=9217 conflicts=0 overlap_conflicts=0 indirect_sites=16 indirect_targets=69 missing_targets=0
```

### 5.3 确定性检查

- 同输入两次运行：输出逐字节相同（`fc /b` 或哈希）。
- 打乱 `pc_main/flow` 行序后再运行：输出仍逐字节相同（证明不依赖输入顺序）。
- CI/本地脚本把两次 `h8lift.lock` diff 为空作为门禁。

---

## 6. 测试计划

### 6.1 解码器 parity（翻译器自身的第一道测试）

```powershell
# 用 h8dasm 重新生成机器可读基线（向量补充同 tools/baselines/README.md:92-97）
.\tools\disasm\h8dasm.exe build\rom1.bin build\rom2.bin tools\baselines\pc_main.txt `
    tools\baselines\flow_main.txt NUL %TEMP%\mach_main_regen.txt
.\tools\disasm\h8dasm.exe build\rom1.bin build\rom2.bin tools\baselines\pc_vec.txt `
    tools\baselines\flow_main.txt NUL %TEMP%\mach_vec_regen.txt
# h8lift 自带 parity：字段级 diff，0 mismatch 才允许继续
.\mk2cpp\build\h8lift.exe --check-decode tools\baselines\mach_main.txt tools\baselines\mach_vec.txt
# 可选：VM 回溯验证（现成 T1 交叉证据）
.\tools\verify\verify_dasm.exe --mach %TEMP%\mach_main_regen.txt --mach %TEMP%\mach_vec_regen.txt `
    --trace vm.log --regtrace vm.log --rom1 build\rom1.bin --rom2 build\rom2.bin
```

### 6.2 单指令家族测试（GT vs 生成码，合成状态）

`tests/lift/op/` 每族一个可执行：构造合成 ROM（`movi r7,#...; <被测指令>; bsr ...; ...`）与
随机但**可复现**的 Ctx（固定 seed 的 LCG；r0-r7/sr/dp/ep/tp/br/相关内存页随机），分别用
GT `MCU_ReadInstruction()` 与生成的 `blk_*()` 跑一条/一段，比较：

- `pc, cp, sr（含 N/Z/V/C/T）, r[0..7]`；
- `cycles` 增量（必须 = 12×指令数）；
- 触碰过的内存/设备寄存器哈希（写集合 + 值）。

家族清单（按 GT 表 `src/mcu_opcodes.cpp:1468-1760`）：
短指令（MOVE/MOVI/MOVL/MOVS/MOVF/CMP）、MOVG 读/写/XCH、MOVG_Immediate、ADDQ、CLR 族
（CLR/TST/EXTU/EXTS/SWAP/NEG/NOT）、SHLR 族（7 种移位）、位操作（BSET/BCLR/BNOTI/BTSTI/
ORC/ANDC/BTST）、算术（ADD/SUB/ADDS/SUBS/ADDX/SUBX/MULXU/DIVXU）、逻辑（AND/OR/XOR/CMP）、
控制寄存器（LDC/STC）、跳转族（BRA/Bcc/bsr/bsr16/jsr/pjsr/pjmp/jmp #/rts/rte/rtd/cntjmp/ldm/stm）、
寄存器间接（jmp rN/jsr rN/jsr via）、`trapa`、`sleep`。寻址模式×尺寸矩阵（byte/word ×
direct/indirect/disp8/disp16/pre-dec/post-inc/abs-br/abs-dp/immediate）必须全测；
页面等价测试要故意把 dp/ep/tp/br 设成与 trace 不同的值（§7 风险 7）。

### 6.3 轨迹 co-sim

1. **离线（M1）**：GT `-tracepc` 输出与 mk2cpp 同格式 trace 逐行 diff；每 N 步比较
   SRAM/设备哈希（`01_architecture.md` §3）。
   ```powershell
   build\nuked-sc55.exe -mk2 -tracepc out\cosim\gt.trace 0 3000000
   mk2cpp\build\mk2cpp.exe --tracepc out\cosim\dut.trace --hash-every 100000 0 3000000
   mk2cpp\build\cosim.exe --ref out\cosim\gt.trace --dut out\cosim\dut.trace --hash-every 100000
   ```
   基线 oracle：`tools/baselines/trace_boot3m_base.txt`（boot 3M）与
   `trace_200m_base.txt`（快照 200M-202M，`tools/baselines/README.md:110-115`）。
2. **在线 lockstep（M2+）**：同进程双核，逐指令比较 `pc/cp/sr/regs`，首分歧写
   `out/cosim/divergence_<cycle>.txt` + 触发命令（`02_conventions.md` §3）。

### 6.4 翻译器确定性/接口测试

- 两次运行 + 输入乱序 → 输出字节相同（§5.3）。
- `RT_Indirect`：合成一个未注册目标 → strict 必须 abort（带 site/target），mixed 必须落解释器
  且写 miss log。
- `RT_Lookup` 覆盖：对 E 中每个 flat 断言 `RT_Lookup != nullptr`（发射完整性）。
- 缺块测试：人为从生成文件删一个块 → strict cosim 必须在首次触及时终止并指出 flat。

### 6.5 首批里程碑（验收 + 命令）

| # | 里程碑 | 交付 | 验收命令/判据 |
|---|---|---|---|
| M1a | h8dec parity | `tools/h8lift/h8dec.c` + parity CLI | `h8lift --check-decode`：9217+18 条 0 mismatch；长度直方图与 §1.2 实测一致 |
| M1b | 分块 + 覆盖 | partition + `map.csv` + `cover` | blocks=2576、instrs=9217、`overlap_conflicts=0`、两次运行字节相同（§5.3） |
| M1c | 首个纵切 | `src/rt` + `src/gen`（仅 boot 0x16f-0x2ab + sleep 0x491/0x492 + handler 0x342-0x3fc 等）+ mixed dispatch | boot 0→3M 与 `trace_boot3m_base.txt` 0 分歧（未翻译 PC 允许解释器兜底，fallback 计数只降不升） |
| M1d | 全执行面 | E∪V 全发射（strict 无兜底） | boot 3M 与快照 200M-202M 两窗口 0 分歧；`exec_coverage=100%` |
| M2 | 静态可达补齐 | R + `data_ranges` + 跳转表展开 | demo/mocknote 长跑 ≥300M 0 分歧；`unknown_reached=0`；`reach_coverage=100%` |

实现顺序固定：**先 parity，再多指令块，最后才整段发射**；每个里程碑不通过不进入下一个
（`00_plan.md` §2）。

---

## 7. 风险与失败模式

| # | 风险 | 证据/现状 | 缓解 |
|---|---|---|---|
| 1 | 自修改代码 | GT 的 `MCU_Write` 对 page0 地址 `<0x8000` 及未映射 ROM 页只打印 `Unknown write`，**不写 ROM**（`src/mcu.cpp:1036-1039/1065-1068`）；`MCU_PatchROM` 目前是空实现（`:1544-1554`） | 生成码绑定 ROM sha256；运行期 `RT_Write` 钩子对"写 ROM 范围"直接 abort（debug 构建）；`-voices` 关闭时翻译，开启即换 ROM 哈希重译 |
| 2 | RAM 执行代码 | E∪V 实测只落在 cp0/cp4 ROM；`MCU_Read_impl` 对页 10/11 读 sram（`src/mcu.cpp:822-828`）但无 PC 证据 | dispatch 对未翻译页/非 ROM 页一律解释器兜底；运行期断言"PC 必须映射 ROM（或已知 RAM 执行白名单=空）"；coverage 报 `unknown_reached` |
| 3 | 重叠 span / 同字节多例程 | E 内 0 span 冲突；V 与 E 0 冲突；r16 候选 69/69 边界对齐（§1.6）。真实形态是**共享块/共享返回点**（0x7c3f 2011+ 目标） | 地址键块、只发射一种解码、优先级裁决 + `overlap_conflicts.csv`；共享块记录 `npred`，不做函数识别 |
| 4 | 指令长度不一致 | h8dasm Phase A/B 已 0 失败（README:104-105）；H1-H5 是**文本** bug 不是长度 bug（`mk2_polyphony_256.md:527-543`） | h8dec parity 字段级门禁；mismatch 即 abort 并输出 diff，禁止带病发射 |
| 5 | flow 边被误当 CFG | kind0 有 4902 条 non-seq 边但只有 11 个 handler 目的（§1.3） | 静态后继只取 h8dec 语义；non-seq 边仅用于动态注册/覆盖；naive 分块 4838 vs 语义 2576 作为回归指标 |
| 6 | `sleep`/`ex_ignore`/中断时机 | GT 每指令前 Pre、后 Post，`sleep` 期间仍 tick（`mcu.cpp:1343-1351`） | §2.3 宿主契约；`sleep` 作为块终止；`irq_abort` 跳过重复 Post；co-sim 逐指令 PC 必对 |
| 7 | 操作数页语义（dp/ep/tp/br） | `(dp,0x7238)` 表在 trace 时 dp=0，但页寄存器可变（`mcu.h:208-215`；`mcu_opcodes.cpp:589-652`） | 运行期 `RT_PageForReg`/`RT_AbsBR`/`RT_AbsDP`/MOVF(tp) 取值；单元测试故意改页；禁止把观测页折叠进生成码 |
| 8 | rom2 页别名 | cp1/cp5、cp8/cp12 等映射到同一 rom2 字节（`src/mcu.cpp:791-821` 的 `address_rom` 重排） | 实测仅 cp0/cp4；`RT_Lookup` 按 cp 分表；若新 cp 出现则报"alias page"并人工确认后再发射 |
| 9 | 运行期未注册间接目标 | 16 站 69 目标当前全在 E；`jsr via` 未执行（潜伏） | strict abort + mixed 兜底 + miss log；表展开与注册表对齐；回归测试人为注入未知目标 |
| 10 | 生成物巨大/编译慢 | 2576 块、9217 指令，估 <1.5MB C++ | 按 cp/区域拆文件；块内无模板/无内联函数爆炸；`-O0` 测通、`-O2` 验证语义不变 |

---

## 附 A. 首批 3 个实现任务（顺序固定）

1. **`tools/h8lift/h8dec.c` + parity CLI**（M1a）：从 `tools/disasm/h8dasm.c:52-225` 迁移解码器，
   按 `src/mcu_opcodes.cpp`（T0）实现 H1-H5 语义；`h8lift --check-decode mach_main mach_vec`
   要求 9217+18 条字段级 0 mismatch。产物：`h8dec.c/h` + 单元测试 + `out/decode_diff.txt`。
2. **分块器 + map.csv + cover**（M1b）：实现 §1.3-1.7（语义后继、终止矩阵、重叠检测、
   `data_ranges`），输出 2576 块/9217 指令/0 冲突，并实现 `cover` 的 `exec/vec/reach/fallback`
   统计与确定性测试（跑两遍 + 乱序输入）。
3. **发射器纵切 + `src/rt` + mixed dispatch**（M1c）：先只发射 boot（0x16f-0x2ab）、sleep
   （0x491/0x492）与 FRT2 handler（0x342-0x3fc）所需块，实现 §2 的 `Ctx`/块签名/`RT_PreInstr`/
   `RT_PostInstr`/解释器兜底；以 `trace_boot3m_base.txt` 的 0→3M 窗口 0 分歧收尾，再放量到
   E∪V 全集（M1d）。
