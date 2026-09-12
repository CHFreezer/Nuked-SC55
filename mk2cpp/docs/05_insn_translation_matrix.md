# 05 指令翻译矩阵（GT oracle 版 H8/532）

状态：M1 设计输入（供 `tools/h8lift` 与 `src/rt` 使用）。
原则：**逐条对齐 `../src/mcu_opcodes.cpp`（GT）**，包括其怪癖、FIXME 与"未实现"路径。本文中
一切 "GT 行为 X" 均给出行号；反汇编器 `../tools/disasm/h8dasm.c` 是 T2 工具，只作对照，不作证明。

- T0（不可怀疑）：`../src/mcu_opcodes.cpp`、`../src/mcu.cpp`、`../src/mcu.h`、
  `../src/mcu_interrupt.cpp`、`../src/mcu_interrupt.h`
- T1：`../tools/vm/h8vm_body.c`（GT 的逐行移植，已对 trace 验证）
- T2：`../tools/verify/verify_dasm.c`（Phase A/B，833,332+44,213 条 0 失败）、
  `../tools/disasm/h8dasm.c`（结构+文本已修，见 §7）
- 文档证据：`../tools/docs/evidence_protocol.md`、`../tools/docs/mk2_polyphony_256.md`

约定：`ctx` = 翻译后运行时的 CPU 状态（字段与 `mcu_t` 一一对应，`src/mcu.h:161-172`）；
`O` = 通用操作数解码结果（对应 GT 全局 `operand_type/operand_ea/operand_ep/operand_size/
operand_reg/operand_data/opcode_extended`，`src/mcu_opcodes.cpp:477-484`）。
本文所有模板中的标识符均为英文；行号一律 `文件:行`。

---

## 1. GT 执行循环与解码算法

### 1.1 顶层循环（`src/mcu.cpp:1327-1514`）

GT 每条"指令周期"固定为 12，不建成本表（`src/mcu.cpp:1351` `mcu.cycles += 12; // FIXME`）。
每轮循环顺序：

```
if (!mcu.ex_ignore) MCU_Interrupt_Handle();   // mcu.cpp:1343-1344
else                mcu.ex_ignore = 0;        //             1345-1346
if (!mcu.sleep)     MCU_ReadInstruction();    //             1348-1349
mcu.cycles += 12;                             //             1351
... tracepc / demo / snapshot ...             //             1353-1429
PCM_Update(cycles); TIMER_Clock(cycles);      //             1434-1436
SM_Update(cycles) 或 UART；MCU_UpdateAnalog  //             1438-1446
```

翻译要求：
- **周期常量 12**（不是真实 H8 周期表）；`ctx.cycles += 12` 由宿主循环负责。
- `ex_ignore` 为一次性"跳过中断处理"标志（RTE/LDC/ORC/ANDC 设置，见 §2/§3）。
- `sleep=1` 时**不取指**，但 cycles 照加；中断入口清 sleep（`src/mcu_interrupt.cpp:32`）。
- 设备更新顺序也必须一致（决定锁相/中断时点）。

### 1.2 `MCU_ReadInstruction`（`src/mcu.cpp:1078-1088`）

```c++
void MCU_ReadInstruction(void) {
    uint8_t operand = MCU_ReadCodeAdvance();     // 1078-1080: 读 (cp<<16)+pc 后 pc++
    MCU_Operand_Table[operand](operand);         // 1082
    if (mcu.sr & STATUS_T)                       // 1084: T=0x8000
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_TRACE); // 1086
}
```

- 取指地址 = `MCU_GetAddress(ctx.cp, ctx.pc) = (ctx.cp << 16) + ctx.pc`（`src/mcu.h:184-186, 188-196`）。
- **T 标志检查在每条指令之后**；一旦指令把 T 置 1（LDC/ORC/ANDC 写 SR 或 RTE 恢复 SR），
  立即挂 TRACE 异常，但真正入口在下一轮 `MCU_Interrupt_Handle()`。
- `MCU_ReadInstruction` 没有自己的异常返回；"trap" 只是 `MCU_ErrorTrap()` 打印
  `cp pc`（`src/mcu.cpp:176-179`），**不改变任何状态、不跳转**。翻译时必须原样保留
  （cosim 中遇到即记为分歧，但机器行为是"继续"）。

### 1.3 `MCU_Operand_Table[256]`（`src/mcu_opcodes.cpp:1468-1725`）

| 首字节 | GT handler | 说明 | 证据 |
|---|---|---|---|
| `00` | `MCU_Operand_Nop` | 空操作，1 字节 | :122-124 |
| `01`/`06`/`07` | `MCU_Jump_JMP` | cntjmp 变体：opcode2 必须 `b8-bf` | :372-443 |
| `02` | `MCU_LDM` | 1 字节 rlist | :154-167 |
| `03` | `MCU_Jump_PJSR` | page+addr16，push pc,cp | :198-212 |
| `04`/`05` | `MCU_Operand_General` | 通用操作数 `#imm8` / `@(br,disp8)` | :1473-1474 |
| `08` | `MCU_TRAPA` | 1 字节，仅高半字节 `1x` 合法 | :185-196 |
| `09` | `MCU_Operand_NotImplemented` | trap | :1478 |
| `0a` | `MCU_Jump_RTE` | 弹 sr,cp,pc；置 `ex_ignore` | :223-229 |
| `0b` | NotImplemented | trap | :1480 |
| `0c`/`0d` | `MCU_Operand_General` | `#imm16` / `@(br,disp8)` word | :1481-1482 |
| `0e` | `MCU_Jump_BSR` | disp8 | :450-464 |
| `0f` | NotImplemented | trap | :1484 |
| `10` | `MCU_Jump_JMP` | `jmp #abs16`（同 cp） | :391-397 |
| `11` | `MCU_Jump_JMP` | 寄存器间接族（ret/jsrviapair/jmp rN/jsr rN） | :340-371 |
| `12` | `MCU_STM` | 1 字节 rlist | :169-183 |
| `13` | `MCU_Jump_PJMP` | page+addr16，改 cp | :466-475 |
| `14` | `MCU_Jump_RTD` | imm8，pop pc，sp+=imm | :316-336 |
| `15`/`1d` | `MCU_Operand_General` | `@(dp,addr16)` byte/word | :1490,1498 |
| `16`/`17` | NotImplemented | trap | :1491-1492 |
| `18` | `MCU_Jump_JSR` | `jsr #abs16` | :214-221 |
| `19` | `MCU_Jump_RTS` | pop pc（cp 不变） | :311-314 |
| `1a` | `MCU_Operand_Sleep` | `sleep=1` | :126-129 |
| `1b` | NotImplemented | trap | :1496 |
| `1c` | `MCU_Jump_RTD` | 读 imm8 后 **trap（TODO）** | :327-331 |
| `1e` | `MCU_Jump_BSR` | disp16 | :450-464 |
| `1f` | NotImplemented | trap | :1500 |
| `20-2f` | `MCU_Jump_Bcc` | disp8（bit4=0） | :231-309 |
| `30-3f` | `MCU_Jump_Bcc` | disp16（bit4=1） | :237-241 |
| `40-4f` | `MCU_Opcode_Short_CMP` | `cmp rN,#imm`（bit3=word） | :802-818 |
| `50-57` | `MCU_Opcode_Short_MOVE` | `rN = imm8` | :694-701 |
| `58-5f` | `MCU_Opcode_Short_MOVI` | `rN = imm16` | :703-711 |
| `60-6f` | `MCU_Opcode_Short_MOVL` | `rN = @(br,disp8)` | :755-777 |
| `70-7f` | `MCU_Opcode_Short_MOVS` | `@(br,disp8) = rN` | :779-800 |
| `80-9f` | `MCU_Opcode_Short_MOVF` | `rN = @(tp,r6+disp8)` / 写 | :713-753 |
| `a0-ff` | `MCU_Operand_General` | 2 字符通用格式 | :1629-1724 |

注意：**不存在 `MULXS`**；`40-4f` 在 GT 是 `CMP r,#imm`（见 §2.13）。
`04/05` 不是前缀，而是"通用指令的第二字符操作数"：`04`=`#imm8`、`0c`=`#imm16`、
`05`=`@(br,disp8)` byte、`0d`=`@(br,disp8)` word、`15`=`@(dp,addr16)` byte、
`1d`=`@(dp,addr16)` word。

### 1.4 `MCU_Operand_General`（`src/mcu_opcodes.cpp:543-673`）

```c++
struct Operand {
    uint32_t type;  // OP_DIRECT / OP_INDIRECT / OP_ABSOLUTE / OP_IMMEDIATE
    uint32_t ea;    // 16-bit 有效地址
    uint32_t ep;    // INDIRECT/ABSOLUTE 的页
    uint32_t size;  // SZ_BYTE=0 / SZ_WORD=1
    uint32_t reg;
    uint32_t data;  // IMMEDIATE 值
    uint8_t  ext;   // opcode2 首字节 == 0x00（扩展）
    uint8_t  opcode;// 解码后的 ocode（0..31，未移位前是 op2）
    uint8_t  ore;   // 解码后的 op2&7
};

Operand decode_operand(Ctx& c, uint8_t op1) {
    Operand o = {};
    uint32_t disp = 0, increase = 0, addr = 0, addrpage = 0;
    o.size = (op1 & 0x08) ? SZ_WORD : SZ_BYTE;        // :558-561
    o.reg  = op1 & 0x07;                              // :562
    switch (op1 & 0xf0) {                             // :563
    case 0xa0: o.type = OP_DIRECT; break;             // :565-567
    case 0xd0: o.type = OP_INDIRECT; break;           // :568-570
    case 0xe0: o.type = OP_INDIRECT; disp = (int8_t)fetch8(c); break;          // :571-574
    case 0xf0: o.type = OP_INDIRECT; disp = (fetch8(c) << 8) | fetch8(c); break; // :575-580
    case 0xb0: o.type = OP_INDIRECT; increase = 2; break; // 预减 :581-584
    case 0xc0: o.type = OP_INDIRECT; increase = 3; break; // 后增 :585-588
    case 0x00:
        if (o.reg == 5) { o.type = OP_ABSOLUTE; addr = ((uint32_t)c.br << 8) | fetch8(c); addrpage = 0; } // :589-596
        else if (o.reg == 4) { o.type = OP_IMMEDIATE; o.data = fetch8(c);
                               if (o.size) o.data = (o.data << 8) | fetch8(c); } // :597-606
        break;
    case 0x10:
        if (o.reg == 5) { o.type = OP_ABSOLUTE; addr = fetch16(c); addrpage = c.dp; } // :608-616
        break;
    }
    if (o.type == OP_INDIRECT) {                      // :618-647
        if (increase == 2) c.r[o.reg] -= (o.size || o.reg == 7) ? 2 : 1; // :620-630
        o.ea = c.r[o.reg] + disp;                     // :631（预减后、后增前）
        if (increase == 3) c.r[o.reg] += (o.size || o.reg == 7) ? 2 : 1; // :632-642
        o.ea &= 0xffff;                               // :644
        o.ep = page_for_register(c, o.reg);           // :646 / mcu.h:208-215
    } else if (o.type == OP_ABSOLUTE) {               // :648-653
        o.ea = addr & 0xffff; o.ep = addrpage & 0xff;
    }
    uint8_t op2 = fetch8(c);                          // :655
    o.ext = (op2 == 0x00);                            // :656
    if (o.ext) op2 = fetch8(c);                       // :657-660
    o.opcode = op2 >> 3; o.ore = op2 & 7;             // :661-662
    return o;
}
```

关键点（必须逐条照抄）：
- `siz` 来自**第一操作数字节 bit3**，与 op2 无关；`reg = op1&7`。
- `@-Rn`/`@Rn+` 的步长：`siz || reg==7` 时 2，否则 1（R7 永远按字）。**先改寄存器再算
  EA**；后增也是**先增再访存**（`ea` 在自增之前算好，但自增先执行）。
- `@(br,disp8)` 的页是 **0**，不是 `br`（`addrpage=0`）；即 flat = `(br<<8)|disp8`。
- `@(dp,addr16)` 用 `c.dp` 页；间接寻址页由寄存器号决定：
  `reg>=6 → tp`，`reg>=4 → ep`，否则 `dp`（`src/mcu.h:208-215`）。
- 扩展前缀：op2 首字节 `0x00` 时再读一字节，`ext=1`。**只有 `MCU_Opcode_MOVG`
  （ocode 16/18）关心 `ext`并直接 trap**（`src/mcu_opcodes.cpp:1043-1054`）；
  其余 ocode 的扩展形式按同一 handler 执行（反汇编器用 `x` 标注）。

### 1.5 长度公式与家族速查

对通用族（首字节 `a0-ff` 或 `04/05/0c/0d/15/1d`）：

```
len = 1 + n + k + t
  n = 操作数附加字节（在 op2 之前）：
        a0-af,d0-df: 0   e0-ef: 1   f0-ff: 2
        b0-bf,c0-cf: 0
        04:1  05:1  0c:2  0d:1  15:2  1d:2
  k = op2 字节数：op2 != 0x00 → 1；op2 == 0x00（扩展）→ 2
  t = 尾立即数字节（仅当最终 ocode==0 且类型为 INDIRECT|ABSOLUTE）：
        ore==4 或 6 → 1；ore==5 或 7 → 2；ore 0-3 → 0（GT 随后 trap）
      若类型为 DIRECT|IMMEDIATE：t = 0（GT 直接 trap，不读尾数）
```

其它族：

| 首字节 | n | 总长 | 备注 |
|---|---|---|---|
| `00` | 0 | 1 | |
| `01/06/07` | op2+disp | 3（合法 `b8-bf`）；否则 2 后 trap | 目标 = `pc_after_disp + (int8)disp` |
| `02` | rlist | 2 | |
| `03` | page+addr16 | 4 | |
| `04` | imm8(n=1) | 3 起 | `1+1+1`（IMMEDIATE 无尾数） |
| `05` | disp8(n=1) | 3 起 | `1+1+1`(+t) |
| `08` | vec | 2 | |
| `09/0b/0f/16/17/1b/1f` | 0 | 1 | trap，不再读字节 |
| `0a` | 0 | 1 | |
| `0c` | imm16(n=2) | 4 起 | `1+2+1`（IMMEDIATE 无尾数） |
| `0d` | disp8(n=1) | 3 起 | `1+1+1`(+t) |
| `0e` | disp8 | 2 | |
| `10` | addr16 | 3 | |
| `11` | subop | 2 | |
| `12` | rlist | 2 | |
| `13` | page+addr16 | 4 | |
| `14` | imm8 | 2 | |
| `15/1d` | addr16 | 4 起 | |
| `18` | addr16 | 3 | |
| `19` | 0 | 1 | |
| `1a` | 0 | 1 | |
| `1c` | imm8 | 2 | 读 imm 后 trap |
| `1e` | disp16 | 3 | |
| `20-2f` | disp8 | 2 | |
| `30-3f` | disp16 | 3 | |
| `40-47` | imm8 | 2 | `cmp` |
| `48-4f` | imm16 | 3 | `cmp` |
| `50-57` | imm8 | 2 | `move` |
| `58-5f` | imm16 | 3 | `movi` |
| `60-6f` | disp8 | 2 | `movl` |
| `70-7f` | disp8 | 2 | `movs` |
| `80-9f` | disp8 | 2 | `movf` |
| `a0-af` | 0 | 2（+t） | direct |
| `b0-bf` | 0 | 2（+t） | `@-Rn` |
| `c0-cf` | 0 | 2（+t） | `@Rn+` |
| `d0-df` | 0 | 2（+t） | `@Rn` |
| `e0-ef` | disp8 | 3（+t） | `@(d8,Rn)` |
| `f0-ff` | disp16 | 4（+t） | `@(d16,Rn)` |

`verify_dasm` Phase A 以完全相同的规则逐条核对 `mach_main.txt` 长度（0 失败），
`h8dasm` 在 `tools/disasm/h8dasm.c:57-223` 复刻同一套规则。

### 1.6 操作数读写（`src/mcu_opcodes.cpp:486-541`）

```c++
uint32_t op_read(Ctx& c, const Operand& o) {                 // :486-509
    switch (o.type) {
    case OP_DIRECT: return o.size ? c.r[o.reg] : (c.r[o.reg] & 0xff);
    case OP_INDIRECT: case OP_ABSOLUTE:
        if (o.size) {
            if (o.ea & 1) exception(EXC_ADDRESS_ERROR);      // :498-501（不返回！）
            return rd16(c, get_address(o.ep, o.ea));         // :502
        }
        return rd(c, get_address(o.ep, o.ea));               // :504
    case OP_IMMEDIATE: return o.data;                        // :505-506
    }
}
void op_write(Ctx& c, const Operand& o, uint32_t v) {        // :511-541
    switch (o.type) {
    case OP_DIRECT:
        if (o.size) c.r[o.reg] = v;
        else { c.r[o.reg] &= ~0xff; c.r[o.reg] |= v & 0xff; }
        break;
    case OP_INDIRECT: case OP_ABSOLUTE:
        if (o.size) {
            if (o.ea & 1) exception(EXC_ADDRESS_ERROR);      // :528-531（不返回！）
            wr16(c, get_address(o.ep, o.ea), v);             // :532
        } else wr(c, get_address(o.ep, o.ea), v);            // :535
        break;
    case OP_IMMEDIATE: exception(EXC_INVALID_INSTRUCTION);   // :537-539
        break;
    }
}
```

- **奇地址不中止访问**：先挂 ADDRESS_ERROR 异常，再照常以"地址 &~1"完成读/写
  （`rd16`= `a&=~1`，`src/mcu.cpp:876-883`；`wr16` 同 `:1071-1076`）。
- `OP_IMMEDIATE` 只可能来自解码，写回即 INVALID_INSTRUCTION；异常**覆盖式**写入
  `exception_pending`（`src/mcu_interrupt.cpp:40-49`），同一条指令内后发生的覆盖先发生的。

### 1.7 内存/页/状态/栈/控制寄存器

- 地址 = `(page << 16) + addr`（`src/mcu.h:184-186`）；读走 `MCU_Read_impl`
  （`src/mcu.cpp:663-856`）：page 0 off<0x8000=rom1；rom2 重排
  `address_rom = addr & 0x3ffff; if (addr & 0x80000) address_rom |= 0x40000;`
  （`:665-667`）；**读路径 page 取 `(address>>16)&0xf`**（`:668`），即 cp 高 4 位被忽略。
- 写路径 page 映射与页 6/7 扩展见 `src/mcu.cpp:896-1069`；page6/7 backing 属 M5
  扩展（`pcm_ext_enabled`），已随 2026-09-11 回滚从 `src/` 移除。
- SR：`sr_mask = 0x870f`（T=0x8000，int mask=0x700，N=0x08，Z=0x04，V=0x02，C=0x01；
  `src/mcu.h:90-98`）。`MCU_SetStatus(cond, mask)` 只改指定位（`src/mcu.h:332-338`）。
- `setcommon(v, siz)`（`src/mcu_opcodes.cpp:675-687`）：
  `v &= siz?0xffff:0xff; N=v&(siz?0x8000:0x80); Z=(v==0); V=0;` **C 不动**。
- 控制寄存器（`src/mcu.h:217-330`）：
  - word：reg0=SR（写后 `&sr_mask`）；reg5=DP（写 `data&0xff`，读 `dp|dp<<8`）；
    reg4=EP；reg3=BR；其它 → trap。
  - byte：reg1=SR 低 8 位（写后 `&sr_mask`）；reg3=br；reg4=ep；reg5=dp；reg7=tp；
    其它 → trap。
  - 读 byte reg1 返回 `(sr & sr_mask) & 0xff`（=CCR/T 之外的标志位）。
- 栈：`push16`：`if (r7&1) ADDRESS_ERROR; r7-=2; wr16(r7, d)`；
  `pop16`：`if (r7&1) ADDRESS_ERROR; d=rd16(r7); r7+=2`（`src/mcu.h:340-356`）。
- 中断/异常入口 `MCU_Interrupt_StartVector`（`src/mcu_interrupt.cpp:56-62`）：
  `address = MCU_GetVectorAddress(v) = Read32(v*4)`（big-endian，地址 &~3）；
  push pc、push cp、push sr；`sr &= ~T`；若 `mask>=0` 则 `sr=(sr&~0x700)|(mask<<8)`；
  `sleep=0`；`cp=address>>16; pc=address&0xffff`。
- `MCU_Interrupt_Handle` 优先级（`src/mcu_interrupt.cpp:64-209`）：
  `trapa_pending[0..15]` → `exception_pending` → NMI → 其它源（level > 当前 mask）。
  `MCU_Interrupt_Exception` 只写 `exception_pending`（`:40-49`）；
  `MCU_Interrupt_TRAPA` 只置 `trapa_pending[vector]`（`:51-54`）。
  `ex_ignore` 只在顶层循环消费（§1.1）。翻译器必须原样复制入口顺序与向量读取。

---

## 2. 短指令（首字节直接分派）语义与模板

通用运行时 API（模板中引用；实现照抄 GT）：

```c++
uint8_t  fetch8 (Ctx&);                 // MCU_ReadCodeAdvance  mcu.h:192-196
uint16_t fetch16(Ctx&);                 // fetch8()<<8 | fetch8()
uint8_t  rd  (Ctx&, uint32_t a);        // MCU_Read            mcu.cpp:858-874
uint16_t rd16(Ctx&, uint32_t a);        // a&=~1               mcu.cpp:876-883
void     wr  (Ctx&, uint32_t a, uint8_t v);
void     wr16(Ctx&, uint32_t a, uint16_t v);
uint32_t get_address(uint32_t page, uint16_t addr) { return (page << 16) + addr; } // mcu.h:184-186
void     srset(Ctx&, bool cond, uint16_t mask);       // MCU_SetStatus        mcu.h:332-338
void     setcommon(Ctx&, uint32_t v, uint32_t siz);   //                      opcodes:675-687
uint32_t addcommon(Ctx&, int32_t, int32_t, int32_t, uint32_t); // ADD Common  opcodes:72-120
uint32_t subcommon(Ctx&, int32_t, int32_t, int32_t, uint32_t); // SUB Common  opcodes:22-70
void     exception(Ctx&, uint32_t src); // MCU_Interrupt_Exception  interrupt.cpp:40-49
void     trap(Ctx&);                    // MCU_ErrorTrap: 只打印 cp:pc，不改变状态  mcu.cpp:176-179
void     push16(Ctx&, uint16_t);        // mcu.h:340-346
uint16_t pop16(Ctx&);                   // mcu.h:348-356
uint32_t cr_read (Ctx&, uint32_t reg, uint32_t siz);          // MCU_ControlRegisterRead  mcu.h:274-330
void     cr_write(Ctx&, uint32_t reg, uint32_t siz, uint32_t data); // MCU_ControlRegisterWrite mcu.h:217-272
```

`addcommon`/`subcommon` 必须使用 GT 的 **int8/int16 收窄 + C 位直接 `(t1>>16)&1` /
`(t1>>8)&1`** 写法（含 `INT8_MIN/MAX`、`INT16_MIN/MAX` 判 V；`subcommon` 中 V 用
`signed` 减法溢出判定）。`setcommon` 不清 C、恒清 V。

### 2.1 `0x00` NOP（`src/mcu_opcodes.cpp:122-124`，表 :1469）

```c++
/* nothing */
```

### 2.2 `0x01` / `0x06` / `0x07` cntjmp（`:372-443`）

形式：`op1 | op2 | disp8`，op2 必须 `0xb8-0xbf`（`op2>>3 == 0x17`），否则 trap。

```c++
uint8_t op2 = fetch8(c); uint8_t reg = op2 & 7;
if ((op2 >> 3) == 0x17) {
    uint16_t disp = (int8_t)fetch8(c);
    bool z = (c.sr & STATUS_Z) != 0;
    bool take = (op1 == 0x01) ? true : (op1 == 0x06 ? z : !z);
    if (take) {
        c.r[reg]--;                       // uint16 回绕
        if (c.r[reg] != 0xffff) c.pc += disp;
    }
} else trap(c);
```

语义：先减后判；减到 0xffff 的那次**不跳**（所以循环体执行 `r[reg]` 次）。总长 3
（trap 路径消费 op2 后长度 2）。`h8dasm.c:69-80`；审计修复见 §7。

### 2.3 `0x02` LDM（`:154-167`）

```c++
uint8_t rlist = fetch8(c);
for (int i = 0; i < 8; i++)
    if (rlist & (1 << i)) {
        uint16_t d = pop16(c);
        if (i != 7) c.r[i] = d;     // bit7 也 pop（r7+=2），但值丢弃
    }
```

### 2.4 `0x12` STM（`:169-183`）

```c++
uint8_t rlist = fetch8(c);
for (int i = 7; i >= 0; i--)
    if (rlist & (1 << i)) {
        uint16_t d = c.r[i];
        if (i == 7) d -= 2;         // GT 原样：压入 r7-2
        push16(c, d);
    }
```

高寄存器先压（高地址）；若含 bit7，压入值为 `r7-2`，之后 push 再 `r7-=2`。

### 2.5 `0x08` TRAPA（`:185-196`）

```c++
uint8_t vec = fetch8(c);
if ((vec & 0xf0) == 0x10) c.trapa_pending[vec & 0x0f] = 1;
else trap(c);
```

入口在下一轮 `MCU_Interrupt_Handle`，优先级最高。

### 2.6 `0x03` PJSR / `0x13` PJMP（`:198-212, 466-475`）

```c++
// PJSR: push pc; push cp; cp=page; pc=addr
uint8_t page = fetch8(c); uint16_t addr = fetch16(c);
push16(c, c.pc); push16(c, c.cp);
c.cp = page; /* GT: if (c.cp == 0x27) c.cp += 0; 死代码 :209-210 */
c.pc = addr;
// PJMP: cp=page; pc=addr（无 push）
```

### 2.7 `0x10` JMP #abs / `0x18` JSR #abs（`:214-221, 391-397`）

```c++
uint16_t addr = fetch16(c);
c.pc = addr;                       // JMP：cp 不变
// JSR: push16(c, c.pc); c.pc = addr;
```

### 2.8 `0x0e` BSR / `0x1e` BSR16（`:450-464`）

```c++
uint16_t disp = (op1 == 0x0e) ? (uint16_t)(int8_t)fetch8(c) : fetch16(c);
push16(c, c.pc);
c.pc += disp;                      // uint16 回绕；BSR16 的 disp 不做符号扩展
```

### 2.9 `0x19` RTS / `0x0a` RTE / `0x14`+`0x1c` RTD
（`:311-314, 223-229, 316-336`）

```c++
// RTS
c.pc = pop16(c);
// RTE
c.sr = pop16(c);                   // 不 &sr_mask（与 LDC 不同）
c.cp = (uint8_t)pop16(c);
c.pc = pop16(c);
c.ex_ignore = 1;
// RTD 0x14
int16_t imm = (int8_t)fetch8(c);   // 先读 imm（长度恒 2）
c.pc = pop16(c);
c.r[7] += imm;
if (c.r[7] & 1) trap(c);           // 注意：MCU_ErrorTrap 打印，不是异常向量
// RTD 0x1c
int16_t imm = (int8_t)fetch8(c); c.pc = pop16(c); trap(c); // TODO: 未实现
```

### 2.10 `0x1a` SLEEP（`:126-129`）

```c++
c.sleep = 1;
```

### 2.11 `0x11` 寄存器间接族（`:338-371`）

```c++
uint8_t b = fetch8(c); uint8_t h = b >> 3, l = b & 7;
if (b == 0x19) {                                  // ret：cp=pop; pc=pop
    c.cp = (uint8_t)pop16(c); c.pc = pop16(c);
} else if (h == 0x19) {                           // jsr via r{even}:r{even+1}（call）
    push16(c, c.pc); push16(c, c.cp);
    l &= ~1;                                      // H4：pair 对齐
    c.cp = c.r[l] & 0xff; c.pc = c.r[l + 1];
} else if (h == 0x1a) {                           // jmp rN（cp 不变）
    c.pc = c.r[l];
} else if (h == 0x1b) {                           // jsr rN（cp 不变）
    push16(c, c.pc); c.pc = c.r[l];
} else trap(c);
```

字节域：`0xc8-0xcf` = pair call（`l` 强制偶数）；`0xd0-0xd7` = `jmp rN`；
`0xd8-0xdf` = `jsr rN`；`0x19` = ret；其余 trap。

### 2.12 `0x20-0x3f` Bcc（`:231-309`）

```c++
uint16_t disp = (op1 & 0x10) ? fetch16(c) : (uint16_t)(int8_t)fetch8(c);
bool N = c.sr & STATUS_N, C = c.sr & STATUS_C, Z = c.sr & STATUS_Z, V = c.sr & STATUS_V;
bool branch = false;
switch (op1 & 0x0f) {
case 0x0: branch = true; break;            // BRA / BT
case 0x1: branch = false; break;           // BRN / BF
case 0x2: branch = (C | Z) == 0; break;    // BHI
case 0x3: branch = (C | Z) == 1; break;    // BLS
case 0x4: branch = C == 0; break;          // BCC / BHS
case 0x5: branch = C == 1; break;          // BCS / BLO
case 0x6: branch = Z == 0; break;          // BNE
case 0x7: branch = Z == 1; break;          // BEQ
case 0x8: branch = V == 0; break;          // BVC
case 0x9: branch = V == 1; break;          // BVS
case 0xa: branch = N == 0; break;          // BPL
case 0xb: branch = N == 1; break;          // BMI
case 0xc: branch = (N ^ V) == 0; break;    // BGE
case 0xd: branch = (N ^ V) == 1; break;    // BLT
case 0xe: branch = (Z | (N ^ V)) == 0; break; // BGT
case 0xf: branch = (Z | (N ^ V)) == 1; break; // BLE
}
if (branch) c.pc += disp;
```

### 2.13 `0x40-0x4f` CMP r,#imm（`:802-818`）

```c++
uint16_t t2 = (op1 & 0x08) ? fetch16(c) : fetch8(c);
subcommon(c, (int32_t)c.r[op1 & 7], t2, 0, (op1 & 0x08) ? SZ_WORD : SZ_BYTE);
```

### 2.14 `0x50-0x57` MOVE / `0x58-0x5f` MOVI（`:694-711`）

```c++
// MOVE (0x50-0x57)
uint8_t d = fetch8(c);
c.r[op1 & 7] &= ~0xff; c.r[op1 & 7] |= d;
setcommon(c, d, SZ_BYTE);
// MOVI (0x58-0x5f)
uint16_t d = fetch16(c);
c.r[op1 & 7] = d;
setcommon(c, d, SZ_WORD);
```

### 2.15 `0x60-0x6f` MOVL / `0x70-0x7f` MOVS（`:755-800`）

```c++
uint32_t reg = op1 & 7;
uint32_t addr = ((uint32_t)c.br << 8) | fetch8(c);   // 页 0！不是 br 页
if (op1 & 0x08) {                                    // word（0x68- / 0x78-）
    if (addr & 1) exception(c, EXC_ADDRESS_ERROR);   // 不中止
}
// MOVL
if (op1 & 0x08) { uint16_t d = rd16(c, addr); c.r[reg] = d; setcommon(c, d, SZ_WORD); }
else { uint8_t d = rd(c, addr); c.r[reg] &= ~0xff; c.r[reg] |= d; setcommon(c, d, SZ_BYTE); }
// MOVS
if (op1 & 0x08) { uint16_t d = c.r[reg]; wr16(c, addr, d); setcommon(c, d, SZ_WORD); }
else { uint8_t d = c.r[reg] & 0xff; wr(c, addr, d); setcommon(c, d, SZ_BYTE); }
```

### 2.16 `0x80-0x9f` MOVF（`:713-753`）——**GT 语义不对称，必须照抄**

```c++
uint8_t  reg  = op1 & 7;
uint32_t siz  = (op1 & 0x08) != 0;                   // bit3
int8_t   disp = (int8_t)fetch8(c);
uint32_t addr = ((c.r[6] + disp) & 0xffff) | ((uint32_t)c.tp << 16);
if ((op1 & 0x10) == 0) {                             // 读方向（0x80-0x8f）
    if (siz) {                                       // 0x88-0x8f：读 16 位
        uint16_t d = rd16(c, addr);                  // 无奇偶检查
        c.r[reg] &= ~0xff; c.r[reg] |= d;            // GT :725-726：d 为 16 位，OR 进整寄存器！
        setcommon(c, d, SZ_BYTE);                    // 状态按 byte（:728）
    } else {                                         // 0x80-0x87：读 8 位
        uint8_t d = rd(c, addr);
        c.r[reg] = d;                                // 整体覆盖，高字节清零
        setcommon(c, d, SZ_WORD);                    // 状态却按 word（:734）
    }
} else {                                             // 写方向（0x90-0x9f）
    if (siz) {                                       // 0x98-0x9f：写 8 位
        uint16_t d = c.r[reg] & 0xff;
        wr(c, addr, d); setcommon(c, d, SZ_BYTE);
    } else {                                         // 0x90-0x97：写 16 位
        uint16_t d = c.r[reg];
        wr16(c, addr, d); setcommon(c, d, SZ_WORD);
    }
}
```

读/写两路的 `siz` 含义相反（这是 GT 原样，不是笔误：`src/mcu_opcodes.cpp:723-751`；
`tools/vm/h8vm_body.c:711-751` 逐行相同）。`tools/docs/mk2_polyphony_256.md:580-583`
记录了由此产生的"高字节泄漏"，翻译必须复现。

---

## 3. `MCU_Opcode_Table[32]` 逐条矩阵

索引语义表：`src/mcu_opcodes.cpp:1727-1760`。所有 handler 形如
`void f(uint8_t opcode /*已 >>3，即 index*/, uint8_t opcode_reg /*op2&7*/)`；
模板中 `O` 为 §1.4 解码结果，`ore = O.ore`。**除特别说明外，指令长度都已在
§1.5 中消费**；模板只描述状态变化。

### 3.0 Index 0 `MOVG_Immediate`（`:825-873`）

| ore | 出现条件 | 语义 | 长度增量 |
|---|---|---|---|
| 6 | `INDIRECT|ABSOLUTE` | `d = (int8_t)imm8`；`op_write(d)`；`setcommon(d, O.size)` | +1 |
| 7 | 同上 | `d = imm16`；`op_write(d)`；`setcommon(d, O.size)` | +2 |
| 4 | 同上且 byte | `subcommon(op_read, imm8, 0, BYTE)`，**不写回** | +1 |
| 4 | 同上且 word | `subcommon(op_read, (uint16_t)(int8_t)imm8, 0, WORD)`，**FIXME**（只读 1 字节符号扩展） | +1 |
| 5 | 同上且 word | `subcommon(op_read, imm16, 0, WORD)` | +2 |
| 5 | 同上且 byte | `subcommon(op_read, imm16, 0, BYTE)`，**FIXME**（byte 却读 2 字节） | +2 |
| 0-3 / DIRECT / IMMEDIATE | | `trap`，**不读尾数** | 0 |

```c++
bool mem = (O.type == OP_INDIRECT || O.type == OP_ABSOLUTE);
switch (ore) {
case 6: if (mem) { uint32_t d = (int8_t)fetch8(c); op_write(c, O, d); setcommon(c, d, O.size); } else trap(c); break;
case 7: if (mem) { uint32_t d = fetch16(c);       op_write(c, O, d); setcommon(c, d, O.size); } else trap(c); break;
case 4: if (mem && O.size == SZ_BYTE) {            // GT 顺序：先 op_read 再取 imm
        uint32_t t1 = op_read(c, O); uint32_t t2 = fetch8(c);
        subcommon(c, t1, t2, 0, SZ_BYTE);
    } else if (mem && O.size == SZ_WORD) {          // FIXME：只读 1 字节并符号扩展
        uint32_t t1 = op_read(c, O); uint32_t t2 = (uint16_t)(int8_t)fetch8(c);
        subcommon(c, t1, t2, 0, SZ_WORD);
    } else trap(c);
    break;
case 5: if (mem && O.size == SZ_WORD) {
        uint32_t t1 = op_read(c, O); uint32_t t2 = fetch16(c);
        subcommon(c, t1, t2, 0, SZ_WORD);
    } else if (mem && O.size == SZ_BYTE) {          // FIXME：byte 却读 2 字节
        uint32_t t1 = op_read(c, O); uint32_t t2 = fetch16(c);
        subcommon(c, t1, t2, 0, SZ_BYTE);
    } else trap(c);
    break;
default: trap(c); break;
}
```

### 3.1 Index 1 `ADDQ`（`:1141-1165`）

```c++
int32_t t1 = op_read(c, O), t2;
switch (ore) { case 0: t2 = 1; break; case 1: t2 = 2; break;
               case 4: t2 = -1; break; case 5: t2 = -2; break;
               default: trap(c); return; }           // 注意：GT 已先 op_read
t1 = addcommon(c, t1, t2, 0, O.size);
op_write(c, O, t1);
```

### 3.2 Index 2 `CLR` 族（`:937-994`）——`ore`=子操作码

```c++
switch (ore) {
case 3: if (O.type != OP_IMMEDIATE) {                          // CLR
        op_write(c, O, 0);
        srset(c, 0, STATUS_N); srset(c, 1, STATUS_Z);
        srset(c, 0, STATUS_V); srset(c, 0, STATUS_C); }
        else trap(c); break;
case 6: if (O.type != OP_IMMEDIATE) {                          // TST
        uint32_t d = op_read(c, O);
        setcommon(c, d, O.size); srset(c, 0, STATUS_C); }
        else trap(c); break;
case 2: if (O.type == OP_DIRECT && O.size == SZ_BYTE) {        // EXTU
        uint32_t d = (uint8_t)c.r[O.reg]; c.r[O.reg] = d;
        srset(c, 0, STATUS_N); srset(c, d == 0, STATUS_Z);
        srset(c, 0, STATUS_V); srset(c, 0, STATUS_C); }
        else trap(c); break;
case 0: if (O.type == OP_DIRECT && O.size == SZ_BYTE) {        // SWAP
        uint32_t d = c.r[O.reg];
        d = ((d & 0xff) << 8) | (d >> 8);
        c.r[O.reg] = d; setcommon(c, d, SZ_WORD); }
        else trap(c); break;
case 5: if (O.type != OP_IMMEDIATE) {                          // NOT
        uint32_t d = ~op_read(c, O); op_write(c, O, d);
        setcommon(c, d, O.size); }
        else trap(c); break;
case 4: if (O.type != OP_IMMEDIATE) {                          // NEG
        uint32_t d = subcommon(c, 0, op_read(c, O), 0, O.size);
        op_write(c, O, d); }
        else trap(c); break;
case 1: if (O.type == OP_DIRECT && O.size == SZ_BYTE) {        // EXTS
        uint32_t d = c.r[O.reg];                               // 注意：状态用扩展前的 d
        c.r[O.reg] = (int8_t)d;
        setcommon(c, d, SZ_WORD); }                            // GT :984-989
        else trap(c); break;
default: trap(c); break;
}
```

### 3.3 Index 3 `SHLR` 族（`:1219-1323`）——`ore`=移位码

统一结构：`d = op_read(O)`（IMMEDIATE 直接 trap）；按 `O.size` 取 msb/置位宽；
写回 `op_write`；`srset(C)`；`setcommon(d, O.size)`（N/Z、**V=0**、C 已被显式更新）。

| ore | 助记符 | 语义 | C_out |
|---|---|---|---|
| 3 | SHLR | `d >>= 1` | `d&1`（移位前） |
| 2 | SHLL | `d <<= 1` | 旧 msb |
| 6 | ROTXL | `d = (d<<1) \| old_C` | 旧 msb |
| 4 | ROTL | `C=旧 msb; d=(d<<1)\|C` | 旧 msb |
| 0 | SHAL | 与 SHLL 完全相同（**V 不置**） | 旧 msb |
| 1 | SHAR | `d = (d>>1) \| 旧 msb` | `d&1` |
| 5 | ROTR | `C=d&1; d>>=1; d\|=C<<(size?15:7)` | `d&1` |
| 7 | — | trap | |

```c++
if (O.type == OP_IMMEDIATE) { trap(c); return; }
uint32_t d = op_read(c, O), C = 0;
switch (ore) {
case 3: C = d & 1; d >>= 1; break;
case 2: C = (d & (O.size ? 0x8000 : 0x80)) != 0; d <<= 1; break;
case 6: { uint32_t c_in = (c.sr & STATUS_C) != 0;
          C = (d & (O.size ? 0x8000 : 0x80)) != 0; d <<= 1; d |= c_in; } break;
case 4: C = (d & (O.size ? 0x8000 : 0x80)) != 0; d <<= 1; d |= C; break;
case 0: C = (d & (O.size ? 0x8000 : 0x80)) != 0; d <<= 1; break;
case 1: { C = d & 1; uint32_t msb = d & (O.size ? 0x8000 : 0x80);
          if (O.size) d &= 0xffff; else d &= 0xff;
          d >>= 1; d |= msb; } break;
case 5: { C = (d & 1) != 0; d >>= 1; d |= C << (O.size ? 15 : 7); } break;
default: trap(c); return;
}
op_write(c, O, d);
srset(c, C, STATUS_C);
setcommon(c, d, O.size);
```

### 3.4 Index 4 `ADD`（`:1167-1179`）

```c++
int32_t t1 = c.r[ore], t2 = op_read(c, O);
t1 = addcommon(c, t1, t2, 0, O.size);
if (O.size) c.r[ore] = t1;
else { c.r[ore] &= ~0xff; c.r[ore] |= t1 & 0xff; }
```

### 3.5 Index 5 `ADDS`（`:1419-1425`）——**不更新标志**

```c++
uint32_t d = op_read(c, O);
if (!O.size) d = (int8_t)d;          // 符号扩展到 32 位
c.r[ore] += d;                        // 直接 16 位回绕写回，不 mask
```

### 3.6 Index 6 `SUB`（`:1181-1193`）

```c++
int32_t t1 = c.r[ore], t2 = op_read(c, O);
t1 = subcommon(c, t1, t2, 0, O.size);
if (O.size) c.r[ore] = t1;
else { c.r[ore] &= ~0xff; c.r[ore] |= t1 & 0xff; }
```

### 3.7 Index 7 `SUBS`（`:1195-1203`）——**不更新标志**

```c++
int32_t t1 = c.r[ore], t2 = op_read(c, O);
if (O.size) c.r[ore] = t1 - t2;
else        c.r[ore] = t1 - (int8_t)t2;
```

### 3.8 Index 8 `OR`（`:1127-1132`）

```c++
uint32_t d = op_read(c, O);
c.r[ore] |= d;
setcommon(c, c.r[ore], O.size);
```

### 3.9 Index 9 `BSET_ORC`（`:875-897`）——**H1：仅立即数走 ORC**

```c++
if (O.type == OP_IMMEDIATE) {                 // ORC
    uint32_t d = op_read(c, O);
    uint32_t val = cr_read(c, ore, O.size);
    val |= d;
    cr_write(c, ore, O.size, val);
    if (ore >= 2) setcommon(c, val, O.size);
    c.ex_ignore = 1;
} else {                                       // BSET: bit = r[ore]&0xf
    uint32_t d = op_read(c, O);
    uint32_t bit = c.r[ore] & 0x0f;
    srset(c, (d & (1u << bit)) == 0, STATUS_Z);
    d |= 1u << bit;
    op_write(c, O, d);
}
```

### 3.10 Index 10 `AND`（`:1205-1217`）

```c++
uint32_t d = c.r[ore] & op_read(c, O);
if (O.size) c.r[ore] = d;
else { c.r[ore] &= ~0xff; c.r[ore] |= d & 0xff; }
setcommon(c, c.r[ore], O.size);
```

### 3.11 Index 11 `BCLR_ANDC`（`:899-921`）——H1 同 9

```c++
if (O.type == OP_IMMEDIATE) {                 // ANDC
    uint32_t d = op_read(c, O);
    uint32_t val = cr_read(c, ore, O.size) & d;
    cr_write(c, ore, O.size, val);
    if (ore >= 2) setcommon(c, val, O.size);
    c.ex_ignore = 1;
} else {                                       // BCLR
    uint32_t d = op_read(c, O);
    uint32_t bit = c.r[ore] & 0x0f;
    srset(c, (d & (1u << bit)) == 0, STATUS_Z);
    d &= ~(1u << bit);
    op_write(c, O, d);
}
```

### 3.12 Index 12 `XOR`（`:1427-1432`）

```c++
uint32_t d = op_read(c, O);
c.r[ore] ^= d;
setcommon(c, c.r[ore], O.size);
```

### 3.13 Index 13 `NotImplemented`（`:820-823`）

```c++
trap(c);
```

### 3.14 Index 14 `CMP`（`:1134-1139`）

```c++
subcommon(c, (int32_t)c.r[ore], (int32_t)op_read(c, O), 0, O.size);
```

### 3.15 Index 15 `BTST`（`:923-935`）

```c++
if (O.type != OP_IMMEDIATE) {
    uint32_t d = op_read(c, O);
    uint32_t bit = c.r[ore] & 0x0f;
    srset(c, (d & (1u << bit)) == 0, STATUS_Z);    // 只动 Z
} else trap(c);
```

### 3.16 Index 16 `MOVG` 读（`:1041-1095`，d=0）

```c++
if (O.ext) { trap(c); return; }                    // 扩展形式全部 trap
uint32_t d = op_read(c, O);
if (O.size) c.r[ore] = d;
else { c.r[ore] &= ~0xff; c.r[ore] |= d & 0xff; }
setcommon(c, d, O.size);
```

### 3.17 Index 17 `LDC`（`:996-1001`）

```c++
uint32_t d = op_read(c, O);
cr_write(c, ore, O.size, d);
c.ex_ignore = 1;
```

### 3.18 Index 18 `MOVG` 写 / XCH（`:1041-1095`，d=1）

```c++
if (O.ext) { trap(c); return; }
if (O.type == OP_DIRECT) {                         // XCH：仅 word
    if (O.size) { uint16_t t = c.r[ore]; c.r[ore] = c.r[O.reg]; c.r[O.reg] = t; }
    else trap(c);
} else {
    uint32_t d = c.r[ore];
    op_write(c, O, d);
    setcommon(c, d, O.size);
}
```

### 3.19 Index 19 `STC`（`:1003-1007`）——方向：控制寄存器 → 操作数

```c++
uint32_t d = cr_read(c, ore, O.size);
op_write(c, O, d);                                 // IMMEDIATE 会在 op_write 里 trap
```

### 3.20 Index 20 `ADDX`（`:1434-1451`）

```c++
int32_t t1 = c.r[ore], t2 = op_read(c, O);
int32_t C = (c.sr & STATUS_C) != 0;
int32_t Z = (c.sr & STATUS_Z) != 0;
t1 = addcommon(c, t1, t2, C, O.size);
if (!Z) srset(c, 0, STATUS_Z);                     // Z = 旧 Z && 新 Z
if (O.size) c.r[ore] = t1;
else { c.r[ore] &= ~0xff; c.r[ore] |= t1 & 0xff; }
```

### 3.21 Index 21 `MULXU`（`:1325-1352`）

```c++
uint32_t t1 = op_read(c, O), t2 = c.r[ore], N, Z;
if (!O.size) t2 &= 0xff;
t1 *= t2;
if (O.size) {
    ore &= ~1;                                     // 目的寄存器对
    c.r[ore | 0] = t1 >> 16;
    c.r[ore | 1] = t1;
    N = (t1 & 0x80000000UL) != 0;                   // FIXME
} else {
    t1 &= 0xffff;
    c.r[ore] = t1;
    N = (t1 & 0x8000UL) != 0;                       // FIXME
}
Z = (t1 == 0);
srset(c, N, STATUS_N); srset(c, Z, STATUS_Z);
srset(c, 0, STATUS_V); srset(c, 0, STATUS_C);
```

### 3.22 Index 22 `SUBX`（`:1453-1466`）

```c++
int32_t t1 = c.r[ore], t2 = op_read(c, O);
int32_t C = (c.sr & STATUS_C) != 0;
t1 = subcommon(c, t1, t2, C, O.size);              // 不保留 Z（与真实 H8 不同）
if (O.size) c.r[ore] = t1;
else { c.r[ore] &= ~0xff; c.r[ore] |= t1 & 0xff; }
```

### 3.23 Index 23 `DIVXU`（`:1354-1417`）

```c++
uint32_t t1 = op_read(c, O), R, Q;
if (t1 == 0) {                                     // FIXME: 非真实异常向量
    trap(c);
    srset(c, 0, STATUS_N); srset(c, 1, STATUS_Z);
    srset(c, 0, STATUS_V); srset(c, 0, STATUS_C);
    return;
}
if (O.size) {
    ore &= ~1;
    uint32_t t2 = ((uint32_t)c.r[ore | 0] << 16) | c.r[ore | 1];
    R = t2 % t1; Q = t2 / t1;
    if (Q > 0xffff) { srset(c,0,N); srset(c,0,Z); srset(c,1,V); srset(c,0,C); }
    else { c.r[ore | 0] = R; c.r[ore | 1] = Q; setcommon(c, Q, SZ_WORD); srset(c,0,C); }
} else {
    uint32_t t2 = c.r[ore];                        // 16 位被除数（GT 原样）
    R = t2 % t1; Q = t2 / t1;
    if (Q > 0xff) { srset(c,0,N); srset(c,0,Z); srset(c,1,V); srset(c,0,C); }
    else { R &= 0xff; Q &= 0xff; c.r[ore] = (R << 8) | Q;
           setcommon(c, Q, SZ_BYTE); srset(c,0,C); }
}
```

### 3.24 Index 24/25 `BSET`（`:1009-1023`）——位号非寄存器（H2）

```c++
if (O.type != OP_IMMEDIATE) {
    uint32_t d = op_read(c, O);
    uint32_t bit = ore | ((opcode & 1) << 3);      // opcode=index(24/25)；等价 raw_op2 & 0x0f
    srset(c, (d & (1u << bit)) == 0, STATUS_Z);
    d |= 1u << bit;
    op_write(c, O, d);
} else trap(c);
```

### 3.25 Index 26/27 `BCLR`（`:1025-1039`）

同 24/25，`d &= ~(1u << bit);`。

### 3.26 Index 28/29 `BNOTI`（`:1111-1125`）

```c++
uint32_t d = op_read(c, O);
uint32_t bit = ore | ((opcode & 1) << 3);
srset(c, (d & (1u << bit)) == 0, STATUS_Z);
d ^= (1u << bit);
op_write(c, O, d);
```

### 3.27 Index 30/31 `BTSTI`（`:1097-1109`）

```c++
uint32_t d = op_read(c, O);
uint32_t bit = ore | ((opcode & 1) << 3);
srset(c, (d & (1u << bit)) == 0, STATUS_Z);        // 只动 Z，不写回
```

### 3.28 解码实例（对照 h8dasm/verify 基线）

| 字节 | GT 解码 | 结果 |
|---|---|---|
| `0d 3c 90` | `0d`=ABS(br) word；`3c`=disp；`90`=ocode18/ore0 | `MOVG3 r0 -> @(br,0x3c)`（写） |
| `a5 13` | `a5`=DIRECT r5 word；`13`=ocode2/ore3 | `CLR r5` |
| `a8 08` | `a8`=DIRECT r0 word；`08`=ocode1/ore0 | `ADDQ #1 r0` |
| `d2 49` | `d2`=@r2；`49`=ocode9/ore1 非立即 | `BSET @r2, bit=r1&0xf`（H1） |
| `ea 02 f9` | `ea`=@(d8,r2)；`02`；`f9`=ocode31/ore1 | `BTSTI @(2,r2), #9`（H2） |
| `aa ff` | `aa`=DIRECT r2 word；`ff`=ocode31/ore7 | `BTSTI r2, #15` |
| `a0 06 12` | `a0`=DIRECT r0；`06`=ocode0/ore6；`12`=imm8 | `MOVG #0x12 -> r0`，长度 3 |

---

## 4. 间接控制流清单与翻译动作

"静态目标"= 解码期即可算出 flat 地址；"动态"= 运行时从寄存器/栈/向量表取得。

| 形式 | 编码 | 目标来源 | 翻译动作 |
|---|---|---|---|
| `jmp #abs16` | `10 lo hi` | `(cp<<16)\|abs`，静态 | 直接边；登记目标 PC |
| `jsr #abs16` | `18 lo hi` | 同上 + push | 直接调用边；登记返回点 |
| `pjmp #page:addr` | `13 pg lo hi` | `(pg<<16)\|addr`，静态 | 跨页直接边；登记（可跨 rom1/rom2） |
| `pjsr #page:addr` | `03 pg lo hi` | 同上 + push pc,cp | 跨页调用边；登记返回点 |
| `bsr disp8` | `0e d` | `pc_after+disp`，静态 | 直接调用边 |
| `bsr16 disp16` | `1e lo hi` | `pc_after+disp`（无符号回绕），静态 | 直接调用边 |
| `Bcc disp8/16` | `2x/3x` | 两目标均静态 | 条件边 + 直落边 |
| `cntjmp` | `01/06/07 b8-bf d` | `pc_after+disp`，静态 | 条件边（含寄存器递减副作用） |
| `jmp @Rn` | `11 d0-d7` | `(cp<<16)\|r[N]` | **动态**：`dispatch_dynamic(flat)`；运行时登记目标 |
| `jsr @Rn` | `11 d8-df` | `(cp<<16)\|r[N]` + push pc | **动态调用**：先压返回地址，再动态派发 |
| `jsr via rN:rN+1` | `11 c8-cf` | `(r[N]&0xff)<<16 \| r[N+1]` + push pc,cp | **动态跨页调用**（H4：call 语义） |
| `ret` | `11 19` | pop cp, pc | **动态返回**：目标 = 栈内容；依赖返回点登记 |
| `rts` | `19` | pop pc（cp 不变） | 动态返回 |
| `rte` | `0a` | pop sr, cp, pc | 动态返回；置 `ex_ignore` |
| `rtd` | `14 d` | pop pc + sp 调整 | 动态返回 |
| 中断/异常 | — | `Read32(vector*4)` 向量表 | 宿主循环按 GT 优先级派发；向量入口注册为块 |
| `trapa` | `08 1x` | 向量表 `VECTOR_TRAPA_x` | 挂 pending；下一轮宿主处理 |

翻译器策略（与 `mk2cpp/docs/00_plan.md:60-63` 一致）：
- 动态目标不允许静态枚举：以运行时"首次遇到即登记 + 断言已翻译"的方式维护目标集；
  未命中回落 GT 解释器单步。
- `ret`/`rts`/`rte`/`rtd` 的目标应视作"返回点集合"，由 `jsr/bsr/pjsr` 调用点的返回
  PC + 中断压栈点共同登记；cosim 覆盖所有实际返回目标。
- **块边界**：`sleep`、`ex_ignore` 设置点（RTE/LDC/ORC/ANDC）、T 检查点、
  中断/异常挂起点必须在指令边界结束块，否则宿主循环的
  "每指令 `Interrupt_Handle`"会被合并，时序偏移（GT 每指令都轮询，`mcu.cpp:1343-1349`）。

---

## 5. `tools/h8lift` 机器可校验表格式建议

建议新增数据文件 `tools/h8lift/insn_matrix.csv`（工具入库、表入库；由本文档作为语义
背书），h8lift 只吃表 + ROM，不写死 switch。列定义：

```csv
operand_lo,operand_hi,op2_lo,op2_hi,n,k,t,len_expr,tmpl,flow,mem,flags,notes
```

- `operand_lo/hi`：首字节闭区间（十六进制，如 `0xa0`）。
- `op2_lo/hi`：适用 op2 原始字节区间；不适用写 `*`。通用族填实际范围（如
  `0x00,0x07` 表示 ocode 0），MOVG 方向用 `0x80,0x97` 无法单区间表达时拆行。
- `n`：操作数附加字节数（0/1/2，见 §1.5）。
- `k`：op2 字节数（`1`；若 op2 首个为 0 则运行时改 2，写 `1/2`）。
- `t`：尾立即数字节数（0/1/2，仅 MOVG_Immediate）。
- `len_expr`：可求值表达式，符号 `n,k,t`；固定长度直接写数字。h8lift 必须用该
  表达式算出每条指令长度并对 `mach_main.txt` 逐条比对（0 mismatch 才允许写块）。
- `tmpl`：§2/§3 的模板 ID（`T_ADD`、`T_MOVG_RD`、`T_MOVF_RW`…，见下）。
- `flow`：`none|cond|call|jump|ret|indirect|trap`。
- `mem`：`r|w|rw|-`（是否可能读写数据内存）。
- `flags`：被写标志集合的子集（`N,Z,C,V`；`-` 表示不写）。
- `notes`：`sleep=1`、`ex_ignore=1`、`H1/H2`、`FIXME:…` 等。

字段与 GT 的硬约束（建议 h8lift 启动自检 + CI 脚本）：

1. `tmpl` 全集必须与本文 §2/§3 一一对应，无孤儿行、无重复覆盖。
2. 对 `pc_main.txt` 每个 PC 解码，`len_expr` 结果必须等于 `mach_main.txt` 的 `len`
   列（`h8dasm` 产出的机器可读 dump 第 2 列，见 `tools/disasm/h8dasm.c:305-318`）。
3. 所有 `flow!=none` 的行，其静态目标必须能从 `flow_main.txt` 观察到（诊断用）；
   动态行（`indirect|ret`）不要求。
4. 任何 `trap` 行落在执行集内 = 立即失败（ROM 不应执行未实现指令）。

模板 ID 全集（与 §2/§3 对应）：
`T_NOP, T_CNTJMP_01, T_CNTJMP_06, T_CNTJMP_07, T_LDM, T_STM, T_TRAPA,
T_PJSR, T_PJMP, T_JSR_ABS, T_JMP_ABS, T_RTE, T_RTS, T_RTD14, T_RTD1C,
T_BSR8, T_BSR16, T_BCC8, T_BCC16, T_SLEEP,
T_JMP_IND_RET, T_JMP_IND_VIAPAIR, T_JMP_IND_RN, T_JSR_IND_RN,
T_CMP_SHORT8, T_CMP_SHORT16, T_MOVE, T_MOVI, T_MOVL_B, T_MOVL_W,
T_MOVS_B, T_MOVS_W, T_MOVF_RB, T_MOVF_RW, T_MOVF_WW, T_MOVF_WB,
T_MOVG_IMM_W1, T_MOVG_IMM_W2, T_SUBIMM_B1, T_SUBIMM_W1SX, T_SUBIMM_W2,
T_SUBIMM_B2, T_ADDQ, T_CLR, T_TST, T_EXTU, T_SWAP, T_NOT, T_NEG, T_EXTS,
T_SHAL, T_SHAR, T_SHLL, T_SHLR, T_ROTL, T_ROTR, T_ROTXL,
T_ADD, T_ADDS, T_SUB, T_SUBS, T_OR, T_BSET_ORC, T_AND, T_BCLR_ANDC,
T_XOR, T_CMP, T_BTST, T_MOVG_RD, T_LDC, T_MOVG_WR, T_XCH, T_STC,
T_ADDX, T_MULXU_B, T_MULXU_W, T_SUBX, T_DIVXU_B, T_DIVXU_W,
T_BSET_B0, T_BSET_B8, T_BCLR_B0, T_BCLR_B8,
T_BNOTI_B0, T_BNOTI_B8, T_BTSTI_B0, T_BTSTI_B8, T_TRAP`

---

## 6. 必须保留的 GT 偏离（对照真实 H8；附证据）

| # | 偏离 | GT 证据 | 影响 |
|---|---|---|---|
| G1 | 每条指令固定 12 cycles，无周期表 | `src/mcu.cpp:1351` | 时序/中断窗口 |
| G2 | `setcommon` 恒清 V、不碰 C；ADD/SUB Common 才写全 N/Z/C/V | `mcu_opcodes.cpp:675-687, 22-120` | 标志态 |
| G3 | ADDX 保留 Z（旧 Z && 新 Z）；**SUBX 不保留 Z** | `:1434-1451, 1453-1466` | 多字节减法标志 |
| G4 | SHAL 与 SHLL 同实现，V 永不置位 | `:1272-1284` | 有符号左移 |
| G5 | EXTS 的 N/Z 用**扩展前**的 16 位值计算 | `:984-989` | `0x0080`→`0xff80` 时 N=0 |
| G6 | MOVF `bit3` 在读写两路含义相反；读 word 用 `r[reg] \|= d`（16 位全 OR） | `:713-753` | 高字节泄漏（`mk2_polyphony_256.md:580-583`） |
| G7 | MOVG 扩展形式（`00 xx`）全 trap | `:1043-1054` | `x` 后缀指令 |
| G8 | MOVG_Immediate ore=4 word 只读 1 字节符号扩展；ore=5 byte 读 2 字节 | `:847-868`（两处 FIXME） | 长度/算术 |
| G9 | DIVXU 除零 = `MCU_ErrorTrap`（打印）+ 写 N=0/Z=1/V=0/C=0，无向量 | `:1360-1368`（FIXME） | 异常路径 |
| G10 | RTD `0x1c` 未实现：读 imm8、弹 pc、再 trap | `:327-331`（TODO） | 未执行路径 |
| G11 | `MCU_ErrorTrap` 只 printf `cp pc`，不 halt、不改状态 | `mcu.cpp:176-179` | 所有 trap 路径 |
| G12 | STM 含 r7 时压入 `r7-2`；LDM 含 r7 时 pop 但丢弃 | `:169-183, 154-167` | 栈 |
| G13 | 奇地址访问挂 ADDRESS_ERROR 后**照常访问**（地址 &~1）；push/pop 同 | `:498-504, 528-535`；`mcu.h:340-356` | 异常+数据 |
| G14 | BSR16 disp 无符号；Bcc disp16 无符号加；cntjmp disp8 符号扩展 | `:450-464, 231-309, 372-443` | 目标计算 |
| G15 | cntjmp 在每次递减后判 `!=0xffff`（0→0xffff 那次不跳） | `:380-384` | 循环次数 |
| G16 | `ret`(`11 19`)弹 cp+pc；`jmp/jsr rN` 不改 cp；pair call 用 `r{l&~1}` | `:345-366` | 控制流 |
| G17 | PJSR 后 `if (cp==0x27) cp+=0;` 死代码 | `:209-210` | 无 |
| G18 | RTE 弹出 sr **不 &sr_mask**；LDC 写 sr 会 &sr_mask | `:225-228`；`mcu.h:220-249` | SR 状态 |
| G19 | `ex_ignore` 由 RTE/LDC/ORC/ANDC 设置，跳过一次中断处理 | `:228, 887, 911, 1000`；`mcu.cpp:1343-1346` | 中断时序 |
| G20 | 读取 page 0 IO/扩展页有副作用（PCM/SM/LCD/中断清标志） | `mcu.cpp:663-856` | 内存语义 |
| G21 | MOVL/MOVS 地址 = `br<<8\|disp8`，**页 0**；`@(br,d8)` 的页也是 0 | `:759-761, 783-785, 592-595` | 寻址 |
| G22 | MULXU 字模式 N 取 32 位积 bit31；byte 模式 N 取 bit15（FIXME） | `:1339, 1345` | 标志 |
| G23 | `MCU_Read16/32` 内部 `&~1`/`&~3`，不报错（报错只在调用方） | `mcu.cpp:876-894` | 对齐 |
| G24 | 只有 `MCU_Opcode_MOVG` 检查 `opcode_extended`；`MOVG_Immediate` 扩展照样执行 | `:656-660, 1043-1054` | `x` 形式 |

---

## 7. `h8dasm` 对照与 H1–H5 审计

`h8dasm.c` 已按 GT 重写结构（长度/目标/字段），并由 `verify_dasm` Phase A/B 复验
（`tools/docs/mk2_polyphony_256.md:539-543`）：

- 结构修复（§9.3，`mk2_polyphony_256.md:500-508`）：
  1. `MOVG_Immediate` 尾立即数（`h8dasm.c:146-157`）；
  2. `01/06/07` cntjmp 改 **3 字节**、显式条件目标（`h8dasm.c:67-80`）；
  3. `0x11` 族细分 ret / `jsr via rN:rN+1` / `jmp rN` / `jsr rN`（`h8dasm.c:81-92`）。
- 文本修复（§9.4，`mk2_polyphony_256.md:510-525`）：`ore` 依 ocode 语义打印
  （MOVG3 方向、ADDQ 立即数、CLR/SHLR 子码、MOVG_Immediate 操作）。
- **H1–H5**（`mk2_polyphony_256.md:527-543`，`tools/docs/evidence_protocol.md:135-143`）：
  - **H1**：ocode 9/11 仅立即数操作数是 ORC/ANDC，否则是 BSET/BCLR（bit=`r[ore]&0xf`）；
  - **H2**：ocode 24-31 的位号 = `ore|((ocode&1)<<3)`，不是寄存器；
  - **H3**：MOVF `0x90-0x9f` 为写方向；
  - **H4**：`11 c8-cf` 是 push pc/cp 的 **call**（`jsr via`），pair 为 `(reg&~1):+1`；
  - **H5**：MULXU/DIVXU 字模式目的/源为 `r{ore&~1}:r{ore&~1|1}`；STC 方向为
    控制寄存器→操作数。
- 这些均为 **h8dasm 打印文本 bug**，GT 行为不变；本文档的语义一律以 `src/` 为准。

---

## 8. 未清/可疑清单（a）与零分歧风险（b）

### (a) GT 中长度/语义不清或有 bug 的 opcode（须原样保留，但 h8lift/审查要特别标注）

1. **MOVG_Immediate ore=4 word**（`:847-852`，FIXME）：16 位 SUB 只读 1 字节并符号扩展，
   与真实 H8 不符；长度按 1 字节算。
2. **MOVG_Immediate ore=5 byte**（`:861-868`，FIXME）：byte 操作却读 2 字节立即数；
   长度按 2 字节算。
3. **DIVXU 除零**（`:1360-1368`，FIXME）：`MCU_ErrorTrap` 打印 + 手工置标志；不是异常向量。
4. **RTD 0x1c**（`:327-331`，TODO）：读 imm8、pop pc 后 trap；语义未实现。
5. **EXTS 状态**（`:984-989`）：N/Z 基于扩展前值，疑似 bug。
6. **MOVF 读 word 的 `r[reg] |= d`**（`:725-726`）：16 位数据 OR 进寄存器，高字节不被清；
   `setcommon(..., SZ_BYTE)` 与"读 16 位"自相矛盾。VM/探针已实证（`mk2_polyphony_256.md:580-583`）。
7. **SHAL ≡ SHLL、V 恒 0**（`:1272-1284`）。
8. **SUBX 不保留 Z**（`:1453-1466`）。
9. **MULXU N 标志**（`:1339, 1345`，FIXME）：word 用 bit31，byte 用 bit15（mask 后）。
10. **RTE sr 不 mask**（`:223-229`）与 LDC/ORC 的 mask 行为不一致。
11. **`MCU_ErrorTrap` 无状态改变**（`mcu.cpp:176-179`）：trap 后继续执行下一条，PC 停在
    当前（操作数已消费）。若 ROM 真跑到未实现路径，cosim 也要复现"继续跑"。
12. **`PJSR` 0x27 死代码**（`:209-210`）与 **MOVG 扩展分支两路同 trap**（`:1043-1054`，
    注释 FIXME）仅为可读性噪声，无行为影响。
13. **`MCU_Opcode_Short_MOVF` 命名/位义**：`0x88`/`0x90` 才是 16 位读/写，
    `0x80`/`0x98` 是 8 位读/写（与 `bit3` 直觉相反）；h8dasm 的 `movfw` 标记同时覆盖
    `0x88` 读与 `0x90` 写（H3 仅修方向）。

### (b) 零分歧翻译器的 top risks

1. **长度规则**：MOVG_Immediate 尾立即数 4 种 FIXME 组合 + 扩展前缀 `00`，是最容易
   错切基本块的地方（错误会连锁导致后续全部错位）。必须用 `len_expr` 对
   `mach_main.txt` 全量比对。
2. **副作用顺序**：`@-Rn` 先减后算 EA；`@Rn+` 先增后访存；奇地址先挂异常再照常访问；
   写立即数操作数先挂 INVALID_INSTRUCTION。翻译必须保持"挂异常不返回"。
3. **标志位精确性**：G2–G5、G22；尤其 ADDX/SUBX 的 Z、SHAL 的 V、EXTS 的 N/Z、
   MULXU 的 N。任何"更正确的 H8"都会分歧。
4. **中断/异常时序**：GT **每指令**轮询 `MCU_Interrupt_Handle`，trapa > exception >
   NMI > 外设；`ex_ignore` 跳过一次；T 检查在指令尾部。块翻译若按块轮询即分歧。
5. **间接控制流**：`jmp @Rn`/`ret`/`rts`/`rte`/`rtd` 无静态目标；返回点登记、
   动态目标注册、跨页 `jsr via pair` 与 SM/中断压栈的配合。
6. **页/银行**：cp 高 4 位在 `MCU_Read_impl` 被截断；`@(br,d8)` 与 MOVL/MOVS 走页 0；
   间接页取 ep/dp/tp；ROM 页只读语义。地址换算必须机械展开（`evidence_protocol.md:44-66`）。
7. **IO 副作用**：PCM/SM/LCD/按键/中断清标志的读写字面量必须走 GT 同款 `MCU_Read/Write`，
   不能用纯 `mem[]` 数组偷懒。
8. **标志与设备更新的循环顺序**：`cycles += 12` 后依次 `PCM_Update/TIMER_Clock/SM_Update
   /UpdateAnalog`（`mcu.cpp:1434-1446`），与中断入口共同决定 PCM/SM 时序。
9. **STM/LDM r7 特例**与 push/pop 的奇偶异常语义。
10. **`cycle=12` 常量的时序观**：M1–M3 不允许"顺手精确化"周期（`mcu.cpp:1351`）。

---

## 附：证据索引

| 主题 | 位置 |
|---|---|
| 指令循环/周期/设备顺序 | `src/mcu.cpp:1305-1514` |
| 取指与 T 检查 | `src/mcu.cpp:1078-1088` |
| 内存映射与位重排 | `src/mcu.cpp:663-856, 896-1069` |
| 操作数表 | `src/mcu_opcodes.cpp:1468-1725` |
| 通用操作数解码 | `src/mcu_opcodes.cpp:543-673` |
| 操作数读写 | `src/mcu_opcodes.cpp:486-541` |
| 32 条 opcode handler | `src/mcu_opcodes.cpp:825-1466` |
| ADD/SUB Common | `src/mcu_opcodes.cpp:22-120` |
| 状态/栈/控制寄存器 | `src/mcu.h:90-98, 184-356` |
| 中断/异常/TRAPA | `src/mcu_interrupt.cpp:21-209` |
| h8dasm 结构/文本 | `tools/disasm/h8dasm.c:52-225` |
| verify 语义模型 | `tools/verify/verify_dasm.c:380-999` |
| VM 镜像实现 | `tools/vm/h8vm_body.c:675-798` |
| H1–H5 审计 | `tools/docs/mk2_polyphony_256.md:500-543` |
| 解码陷阱速查 | `tools/docs/evidence_protocol.md:135-143` |
