# 证据协议（Ground-Truth 优先分析规则）

适用：本项目 H8/SC55 反汇编、模拟器、ROM 行为分析（dasm / VM / GT / trace / 文档）。
目标绑定：复音 28→256 的所有 patch 落点必须字节级正确，本协议约束解码验证、patch 设计与文档结论。

## 0.0 仓库提交政策（2026-09-11，用户指示，硬性）

**只提交文档与代码。ROM 及其派生的一切产物一律不入 git**（版权风险）。
- 忽略规则：`tools/`、`mk2cpp/` 等**已入库公开目录**内部的派生/调试产物规则
  写在公开 `.gitignore`（`tools/baselines/*`、`*.exe`、
  `mk2cpp/out/`、`mk2cpp/src/gen/` 等），随仓库共享；这些目录名已公开，不构成
  额外信息。
- **不得在任何公开文件（md、`.gitignore` 注释、提交信息、代码注释）中写出
  本地私有路径名或本地目录布局**；本地资料的排除方式只在本地生效，不入库
  文档（本项目为他人 fork）。
- 因此：所有 ROM、反汇编、PC/流集合、trace、快照、日志、中间产物均只在本地存在，按需再生成（再生成命令见 `tools/baselines/README.md`）。
- 本地 fixture 目录（如 ROM、反汇编、trace 存放处）与运行资产目录的内容不入库；仅其说明性 README（如有）入库。
- 调试产物写 `%TEMP%\opencode\`，任务收尾清空。
- 运行所需的上游项目资源文件保持原样（非罗兰 ROM 派生），不做增减。

## 0. 核心（一句话）

`ROM 字节 + src/` = Ground Truth（GT）。`h8dasm`/`VM`/`verify 脚本` 是**我们**对硬件的实现，不是硬件的权威。trace 是强证据，但须按正确的 PC/时序语义解读。文档/变量名/语义标签只是假设。看似不可能时，**先查自己的**算术、地址换算、指令边界、endian、decoder path——查完才允许断言 dasm/VM 有 bug。**不要为了让 VM/dasm/trace/旧文档成立而反过来怀疑 GT。**

## 1. 证据分级（严格优先级）

| Tier | 来源 | 地位 |
|---|---|---|
| T0 | `src/`（mcu.cpp / mcu_opcodes.cpp / pcm.cpp·h）+ `build/rom1.bin`·`rom2.bin` 原始字节 | GT，不可怀疑 |
| T1 | runtime trace（`vm_pc.txt` / `vm.log` / `pcm_trace.log`）、对 GT 的直接插桩 | 强证据，可能有 bug |
| T2 | `tools/disasm/h8dasm.c`、`tools/vm`、`tools/verify/*.py`、一切自制 decoder/脚本 | 可疑实现，必须允许出错 |
| T3 | `mk2_polyphony_256.md`、`dasm_annotated.txt`、注释、旧模型推理、"看起来应该如此" | 仅假设，不可作证明 |

- T2 与 T0 冲突 → 以 T0 为准，并定位 T2 的具体 decoder bug。
- "VM 和 dasm 都这么说"**不是证明**（两者可能共享同一理解错误）。
- T0 看似自相矛盾时，优先假设：自己读错代码 / opcode 位算错 / 地址 bank 换算错 / instruction boundary 错 / endian·width·sign-ext 理解错 / 用错 decoder path。机械验证完这些之前不得声称 GT 错。

**T1 常见坑**（矛盾时先查）：PC 记的是 current 还是 next（本项目 pcmtrace 记写时刻 PC = **后一条指令 PC**）；word 写拆成多 byte 写/归因偏移；logging 本身有 bug。

**T2 常见 bug 类**（怀疑时必须逐类列）：direction 解反 / operand width / register field / addressing mode / instruction length / 特殊寄存器与通用寄存器混淆 / sign extension / bank·page 换算 / 同 opcode 族共用 decoder 漏特殊 case。

## 2. 遇矛盾先列假设，不立即选边

```
H1 dasm decoder 错   H2 VM decoder 错    H3 trace 归因错
H4 自己位运算错      H5 指令边界错        H6 地址/bank/endian/width 理解错
```
按验证成本从低到高排除（先自查 H4/H5/H6，通常最便宜）。

## 3. 位运算与地址/偏移运算必须机械展开（高优先级）

**范围（2026-09-10 加强）**：不仅是 direction/width/寄存器字段等**窄位运算**，还包括**一切地址/偏移/大小运算**——32-bit 地址掩码（`& 0x3ffff`、`>> 16`、`& 0xf`）、page/bank 换算、rom2 位重排、file offset 换算、数组下标/区间边界。教训（2026-09-10 page6 探针轮）：`0x000f0000 & 0x3ffff` 连续三次心算错（0x00000 → 0xC000 → …），而 `memprobe_map.c` 的输出从头就是对的——**心算 32-bit 掩码/重排是本项目已发生的错误源，与窄位运算同级管控**。

**规则**：
1. 凡结论依赖 bit/掩码/重排，**禁止纯心算**，写出逐步展开（全宽二进制或逐位/逐 nibble 分解），如：
```
raw(次字节)=0x90 = 1001 0000b
ocode = 0x90>>3 = 0x12 = 18      ore = 0x90&7 = 0
dir   = ocode&2 = 0x12&2 = 0x02 ≠ 0  → WRITE   （= raw 次字节 bit4）

addr=0x000f0000:  page = (addr>>16) & 0xf = 0x000f & 0xf = 0xf (=15)
                  off  = addr & 0xffff = 0x0000
                  address_rom = addr & 0x3ffff（nibble 级）:
                    0x000F0000: F 在 nibble4（bit16-19 全置位）
                    0x0003FFFF: = 2^18−1，bit0..17（不含 bit18/19）
                    AND → bit16+bit17 = 0x30000
                  addr bit19（0x80000）置位 → |= 0x40000 → 0x70000
                  => rom2[0x70000]   （工具交叉核对：memprobe_map.c page15 off0 → rom2+0x70000 ✓）
```
2. **地址重排/位掩码优先用 C 工具算，不用心算**（与 §14 同源）：`memprobe_map.c` 类工具逐条复刻 `MCU_Read/Write` 位重排，输出即权威。心算与工具不一致 → **以工具为准**，回查输入（地址写错？mask 记错？），不反过来怀疑工具。
3. **重复重算同一表达式 = 错误信号**：同一掩码/重排/换算在会话内重算 ≥2 次且结果摇摆，立即停下心算，改走工具或纸面全宽展开，并回查该值被引用过的所有下游结论（§10）。
4. 结论将影响后续 ≥10 步推理时，机械检查是前置条件。禁止未展开就写 `0x12&2=0`、`0x000f0000&0x3ffff=0x30000` 之类。

## 4. 最小验证案例（单条指令）

```
PC(cp:pc) / ROM 字节 / operand 字节解码 / opcode2 字节 / ocode=opcode2>>3 / ore=opcode2&7
/ size / direction / effective address / 预期读·写 / next PC → 逐项对应 src/ decoder path
```
例：`0d 3c 90`：operand=0x0d（br 页绝对）、opcode2=0x90 → ocode=18(MOVG3)、ore=0、dir=WRITE、EA=br+0x3c、next PC=+3。先单条对 GT，不一致才进入 dasm bug 调查。

## 5. 事实与语义标签分两层，不合并

- **Confirmed facts**：`a3a0[slot] 写入 0x94`；`cleanup 路径测 bit7`；`另一路径 CLR a3a0`。
- **Semantic hypotheses**：0x94 可能=active/registered；bit7 可能=cleanup 禁止位。
后续行为与标签冲突 → **先撤销标签**，不怀疑已确认的 load/store 行为。

## 6. 地址标准化（本项目约定）

```
display: 00045cc2   cp: 0x04   pc: 0x5cc2   flat: 0x045cc2
0000xxxx=rom1，0004xxxx=rom2；rom2 fileoff = flat - 0x40000
SRAM: sram[addr & 0x7fff]；快照 demo_postW.bin 的 sram 段偏移 = 0x450
```
grep 前先确认：格式（display/flat/fileoff）、bank、宽度。搜不到 → 先查地址表示法，不假设代码不存在。

## 7. 禁止作为主要证据

"若是 read 则前一条 load 成 dead load，所以必为 write" 只是 **suspicious signal** → 仍回 ROM+src 验证。真实代码可含 dead/redundant load、side-effect read、未知语义。

## 8. 宣布 `DASM/VM BUG CONFIRMED` 必备字段

```
PC / ROM 字节 / GT 解码（src/ decoder path）/ 正确指令
/ 当前 dasm(VM) 输出 / 具体差异 / bug 类(direction|width|addressing|reg|length|family-special-case)
/ 疑似有错的代码位置
```

## 9. 矛盾矩阵（GT/VM/dasm/trace 不一致时用，只解释真正冲突项）

| Source | Result | Confidence |
|---|---|---|
| ROM 字节 | … | GT |
| src/ decoder | … | GT |
| runtime trace | … | strong |
| VM | … | fallible |
| dasm | … | fallible |
| 旧文档 | … | weak |

不让一个错误解读污染整条分析链。

## 10. Rollback 协议

发现早期结论错（如 direction 算反）→ 明确标记：

```
Previous assumption invalid: <原结论>。Correct GT result: <正确结论>。
```
并**审计所有依赖它的下游结论**（循环语义/寄存器 lifetime/内存读写分类/表解释/分配行为/旧 bug 指控），只修当前一句而保留被污染推论 = 违规。

## 11. 范围控制

旁支问题（无关通道/页面/辅助数学函数）：先判断是否阻塞主任务；不阻塞 → 记 `Unresolved but non-blocking` 后继续，不无限展开。

## 12. 结论置信度三档（不合并表述）

- **Confirmed**：ROM + src/ 直接证明。
- **Strongly supported**：GT + trace/多个独立行为，带少量语义推断。
- **Inferred**：依 usage pattern 推断具体语义名。

## 13. 项目已知 H8/SC55 decoder 陷阱（T2 历史 bug 记录，怀疑时对照）

- MOVG 族（op≥0xa0）：方向 = `ocode&2`（次字节 bit4）：ocode 16=读、18=写/XCH（direct+word）；旧 dasm 恒按读式打印（已修）。
- `ore` 语义随 ocode 变：**非寄存器**的有：1=ADDQ 立即数（0:+1 1:+2 4:−1 5:−2）、2=CLR 子操作码（3=CLR 6=TST 2=EXTU 0=SWAP 5=NOT 4=NEG 1=EXTS）、3=SHLR 移位码、0=MOVG_Immediate 选操作（6/7 写 imm、4/5 减 imm）；**9/11=BSET_ORC/BCLR_ANDC 仅在操作数为立即数时成立，否则是 BSET/BCLR（bit=r[ore]&0xf）**；**24-31=`ore|((ocode&1)<<3)` 为位号（非寄存器）**；其余 ocode 的 `ore` 才是寄存器。
- MOVG_Immediate：源 indirect/absolute 且 ore∈{4,5,6,7} → 码流再读 1/2 字节尾 imm（4/6→1 字节、5/7→2 字节）。
- 计数分支 `01/06/07` = **3 字节**（opcode2 + int8 disp：`r[reg]--`，未下溢则 `pc+=disp`，06 需 Z、07 需 !Z）。
- `0x11` 寄存器间接族：`ret`（pop cp,pc）/ `jsr via rN:rN+1`（push pc/cp 后经寄存器对跳转，**call 语义**）/ `jmp rN` / `jsr rN`。
- MOVF `0x80-0x8f`=读、`0x90-0x9f`=写；MULXU/DIVXU 字模式目的为寄存器对 `r{ore&~1}:r{ore&~1|1}`；STC 方向=`控制寄存器 -> 操作数`。
- `tools/verify`（Phase A 833,332 + Phase B 44,213 条 0 失败）是当前最强交叉验证，但它本身是 **T2**——不替代上面的 ROM+src 验证，也不能反过来为某条解码背书。

## 14. 工具实现语言：优先 C，不用 Python

GT（`src/`）与 VM（`tools/vm/`）均为 C/C++ 工程。**本项目自制工具（验证 / 分析 / 探针 / 解码器）一律优先 C**：

1. **静态编译 + 静态类型**：类型错、数组越界在编译期或静态检查暴露；不像 Python 动态解释那样运行时悄悄错（一个 `int` 与 `uint16_t` 之差、一次漏掉的符号扩展，Python 不报，C 会）。
2. **与 GT/VM 类型零 gap**：可直接复用其头/结构体/常量/函数（`mcu_t`、`sram[]`、`rom1[]/rom2[]`、`MCU_Read/Write`、`MCU_Opcode_Table`、opcode 编码、`address_rom` 位重排），**照搬代码块、逐字节对齐**，避免"换一门语言重新建模 H8 内存/寄存器/指令"引入的隐形 gap。

- 结论性 / 可复用验证工具**必须 C**；一次性快照字节统计可用脚本起步，但**结论须由 C 工具或 GT/VM 本身复核**后才定。
- 教训：提内存扩展（如 page6）前，必须先用 C 的真实内存模型（`MCU_Read/Write` 的页映射、`(page<<16)+offset` 寻址、`address_rom` 位重排）把地址怎么编、会不会重叠、基址寄存器怎么改想清楚——**既不武断说"不可"（page6 其实可 backing），也不武断说"能用"（没想清解码就拍板）**。C 工具用真实模型，天然防止这两种拍脑袋。
- **verify 已迁移 C（2026-09-11）**：`tools/verify/verify_dasm.c`（Phase A+B 一体，替代原 `verify_dasm.py`+`semantics.py`，.py 已删）。相对 Python 版增强：① 向量入口补充解码（`pc_vec.txt`→`mach_vec.txt`，解决"handler 首指令地址永不作为 trace 行 PC"导致的派发漏识别）；② 补全 ocodes 9/11（ORC/ANDC vs BSET/BCLR）与 24-31（BSET/BCLR/BNOTI/BTSTI）语义（Python 版跳过）。
