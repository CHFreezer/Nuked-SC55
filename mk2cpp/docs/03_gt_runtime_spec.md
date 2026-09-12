# 03 GT 运行时规格：翻译核的等价执行环境

状态：研究文档（M1 基础设施）。目标：把 standalone translated core（`src/rt` + `src/gen`）
必须逐位复刻的 GT 执行环境写清楚，使其与 `../src/`（GT，oracle）在相同 ROM、相同输入下
**0 分歧**。本文只做规格描述，不改任何源码。

> **回滚注记（2026-09-11）**：`pcm_ext_*`、page6/7 backing、0xE800 窗口、`-voices:`、
> `PCM_WriteExt/ReadExt` 相关条目在 M5 回滚后已不在 `src/` 中；本文对应行保留为
> M5 设计记录，不再代表当前实现。

引用约定：

- 所有行号均指仓库根下的 `src/`，写作 `src/mcu.cpp:668` 形式。
- 未特别说明处均为 **MK2（`romset = ROM_SET_MK2`，`mcu_mk1=0`，`mcu_jv880=0`）** 路径。
- GT 的执行模型是"解释器 + 固定宿主循环"，所以翻译核不能只翻译指令序列，还必须复刻
  内存访问副作用、寄存器掩码、异常排队与宿主调度顺序。

三条总原则（后面反复出现）：

1. **读有副作用**：`MCU_Read` 可能修改 `adf_rd`/`ssr_rd`/`io_sd`/`ga_int_trigger` 并清中断
   （`src/mcu.cpp:549-553`、`704-709`、`740-758`）。翻译核不得缓存/预取，读一次必须恰好执行一次。
2. **`MCU_ErrorTrap` 不停机**：它只 `printf("%.2x %.4x\n", mcu.cp, mcu.pc)`（`src/mcu.cpp:176-179`），
   随后继续执行。非法指令、非法控制寄存器、未实现操作都"打印后照走"，且非法操作通常会先
   消耗掉相应取指字节（例如 `MCU_Operand_General` 的取指顺序，`src/mcu_opcodes.cpp:543-673`）。
3. **时序与设备时钟与 CPU 解耦**：CPU 每步恒定 `cycles += 12`（`src/mcu.cpp:1351`）；PCM/定时器/
   SM 各自按自己的比例从 `mcu.cycles` 推进（`src/mcu.cpp:1434-1446`）。

---

## 1. 内存映射与读写顺序（`MCU_Read_impl` / `MCU_Write`）

### 1.1 地址分解与前导逻辑

`MCU_Read_impl`（`src/mcu.cpp:663-856`）先做与页无关的预处理：

- `address_rom = address & 0x3ffff`；若 `address & 0x80000 && !mcu_jv880` 则 `address_rom |= 0x40000`
  （`src/mcu.cpp:665-667`）。这条只影响 page 8/9/14/15 读 ROM2 的高半区。
- `page = (address >> 16) & 0xf`；`address &= 0xffff`；`ret` 初值 `0xff`（`src/mcu.cpp:668-670`）。
- 读取入口不做任何 `pc`/页寄存器修改；页寄存器由操作数编码显式给出（见第 4 节）。
- `MCU_Write`（`src/mcu.cpp:896-1069`）同样先 `page=(address>>16)&0xf; address&=0xffff`
  （`src/mcu.cpp:898-899`），**没有** `address_rom` 逻辑。

`rom2_mask` 初值 `ROM2_SIZE-1 = 0x7ffff`（`src/mcu.cpp:661`），JV880 时先 `/2`
（`src/mcu.cpp:2151`），但随后又由实际文件长度覆盖为 `rom2_read-1`（`src/mcu.cpp:2211-2214`）。

### 1.2 page 0 读：MK2 路径（`!mcu_mk1`），精确条件链

顺序严格如下（`src/mcu.cpp:673-718`）：

1. `!(address & 0x8000)` → `ret = rom1[address & 0x7fff]`（`src/mcu.cpp:673-675`）。
2. `else`（`address >= 0x8000`），`base = mcu_jv880 ? 0xf000 : 0xe000`（`src/mcu.cpp:680`）：
   - a. `base <= address < (base | 0x400)` → `PCM_Read(address & 0x3f)`（`src/mcu.cpp:681-683`）。
   - b. `else if pcm_ext_active && !mcu_jv880 && 0xe800 <= address < 0xe840`
     → `PCM_ReadExt(address & 0x3f)`（`src/mcu.cpp:685-688`）。
   - c. `else if !mcu_scb55 && 0xec00 <= address < 0xf000` → `SM_SysRead(address & 0xff)`（`src/mcu.cpp:689-692`）。
   - d. `else if address >= 0xff80` → `MCU_DeviceRead(address & 0x7f)`（`src/mcu.cpp:693-696`）。
   - e. `else if 0xfb80 <= address < 0xff80 && (dev_register[DEV_RAME] & 0x80)` 
     → `ram[(address - 0xfb80) & 0x3ff]`；`DEV_RAME = 0x79`（`src/mcu.cpp:697-699`，`src/mcu.h:82`）。
   - f. `else if 0x8000 <= address < 0xe000` → `sram[address & 0x7fff]`（`src/mcu.cpp:700-703`）。
   - g. `else if address == (base | 0x402)` → 返回 `ga_int_trigger`，随后 `ga_int_trigger=0` 并清中断：
     JV880 清 `INTERRUPT_SOURCE_IRQ0`，否则 `INTERRUPT_SOURCE_IRQ1`（`src/mcu.cpp:704-709`）。
   - h. `else` 打印 `Unknown read %x` 并返回 `0xff`（`src/mcu.cpp:710-714`）。

关键顺序后果：

- **0xe400-0xe7ff 的读没有专门分支**（除了精确的 `base|0x402`）：LCD/GA 区只支持那一个读地
  址（MK2 为 0xe402，JV880 为 0xf402），其余读落到 "Unknown read" 返回 `0xff`。
- **RAM 窗口先于 SRAM 分支**：`0xfb80-0xff7f` 在 RAME bit7=0 时既不是 RAM 也不是 SRAM，最终
  "Unknown read" 返回 `0xff`（`src/mcu.cpp:697-703`）。
- **设备窗口覆盖 `0xff80-0xffff`**，`& 0x7f` 得到 0x00-0x7f 的设备寄存器索引。

### 1.3 page 0 读：MK1 路径（`mcu_mk1`）

顺序（`src/mcu.cpp:719-774`）：

1. `0xe000 <= address < 0xe040` → `PCM_Read(address & 0x3f)`（`src/mcu.cpp:721-724`）。
2. `address >= 0xff80` → `MCU_DeviceRead(address & 0x7f)`（`src/mcu.cpp:725-728`）。
3. RAM 窗口 + RAME bit7 → `ram`（`src/mcu.cpp:729-733`）。
4. `0x8000 <= address < 0xe000` → `sram[address & 0x7fff]`（`src/mcu.cpp:734-737`）。
5. `0xf000 <= address < 0xf100` → **带副作用的读**（`src/mcu.cpp:738-759`）：
   - `io_sd = address & 0xff`；
   - `mcu_cm300` 时直接返回 `0xff`；
   - `LCD_Enable((io_sd & 8) != 0)`；
   - 按 `io_sd` bit0-3 为低有效选通行，返回按键矩阵 `mcu_button_pressed` 的反相 AND，返回 `0xff` 或计算值。
   注意该分支**立即 return**，不做后续设备读。
6. `address == 0xf106` → `ga_int_trigger`、清 0、清 `INTERRUPT_SOURCE_IRQ1`（`src/mcu.cpp:760-765`）。
7. 否则 `Unknown read` 返回 `0xff`（`src/mcu.cpp:766-770`）。

MK1 读 **0xf000-0xf0ff 的地址值本身决定 `io_sd`**（不是数据总线写），这是扫描键盘/LCD 的机制。

### 1.4 page 0 写：MK2 路径（`!mcu_mk1`）

顺序（`src/mcu.cpp:900-1039`）：

1. `address & 0x8000` 为真：
   - a. `base|0x400 <= address < base|0x800`（MK2：0xe400-0xe7ff）（`src/mcu.cpp:907-930`）：
     - `address == base|0x404 || base|0x405` → `LCD_Write(address & 1, value)`；
     - `base|0x401` → `io_sd = value; LCD_Enable((value & 1) == 0)`；
     - `base|0x402` → `ga_int_enable = value << 1`；
     - 其余仅 `printf("Unknown write ...")`，**不落任何存储**。
   - b. `else if base <= address < base|0x400` → `PCM_Write(address & 0x3f, value)`，可选 `g_pcm_trace`
     记录（`src/mcu.cpp:931-947`）。
   - c. `else if pcm_ext_active && !mcu_jv880 && 0xe800 <= address < 0xe840` → `PCM_WriteExt`（`src/mcu.cpp:948-951`）。
   - d. `else if !mcu_scb55 && 0xec00 <= address < 0xf000` → `SM_SysWrite`（`src/mcu.cpp:952-955`）。
   - e. `else if address >= 0xff80` → `MCU_DeviceWrite(address & 0x7f, value)`（`src/mcu.cpp:956-959`）。
   - f. `else if 0xfb80 <= address < 0xff80 && RAME bit7` → `ram[...] = value`（`src/mcu.cpp:960-964`）。
   - g. `else if 0x8000 <= address < 0xe000` → `sram[address & 0x7fff] = value`（`src/mcu.cpp:965-982`）。
     - `SC55_TRACE=1` 时对 `0x5c00-0x5d30` 有额外日志，不影响状态（`src/mcu.cpp:968-980`）。
   - h. `else` 打印 `Unknown write`（`src/mcu.cpp:983-986`）。
2. `else`（address < 0x8000）：
   - JV880 且 `0x6196 <= address <= 0x6199` → nop（`src/mcu.cpp:1032-1035`）；
   - 否则 `Unknown write`（`src/mcu.cpp:1036-1039`）。**向 ROM 区写入被丢弃**。

注意读/写顺序**不对称**：写侧先判 LCD/GA 范围再判 PCM，读侧先判 PCM 再判零散的 `base|0x402`。

### 1.5 page 0 写：MK1 路径

顺序（`src/mcu.cpp:988-1030`）：

1. `0xe000-0xe03f` → `PCM_Write`（`src/mcu.cpp:990-993`）。
2. `>=0xff80` → `MCU_DeviceWrite`（`src/mcu.cpp:994-997`）。
3. RAM 窗口 + RAME → `ram`（`src/mcu.cpp:998-1002`）。
4. `0x8000-0xdfff` → `sram`（`src/mcu.cpp:1003-1006`）。
5. `0xf000-0xf0ff` → `io_sd = address & 0xff; LCD_Enable((io_sd & 8) != 0)`（`src/mcu.cpp:1007-1011`）。
6. `0xf105` → `LCD_Write(0, value); ga_lcd_counter = 500`（`src/mcu.cpp:1012-1016`）。
7. `0xf104` → `LCD_Write(1, value); ga_lcd_counter = 500`（`src/mcu.cpp:1017-1021`）。
8. `0xf107` → `io_sd = value`（`src/mcu.cpp:1022-1025`）。
9. 否则 `Unknown write`（`src/mcu.cpp:1026-1029`）。

MK1 的 LCD 中断模拟在宿主循环里：`ga_lcd_counter` 每步递减，到 0 时对 `MCU_GA_SetGAInt(1,0/1)`
（`src/mcu.cpp:1502-1513`）。

### 1.6 page 1-15 读表（`src/mcu.cpp:791-853`）

| page | 读来源 | 证据 |
|---|---|---|
| 1-4 | `rom2[address_rom & rom2_mask]` | `src/mcu.cpp:791-802` |
| 8,9 | `!jv880` → rom2；jv880 → `0xff` | `src/mcu.cpp:803-814` |
| 14,15 | `!jv880` → rom2；jv880 → `cardram[address & 0x7fff]` | `src/mcu.cpp:815-821` |
| 10,11 | `!mcu_mk1` → `sram[address & 0x7fff]`；否则 `0xff` | `src/mcu.cpp:822-828` |
| 12,13 | jv880 → `nvram[address & 0x7fff]`；否则 `0xff` | `src/mcu.cpp:829-835` |
| 5 | `mcu_mk1` → `sram[address & 0x7fff]`；否则 `0xff` | `src/mcu.cpp:836-841` |
| 6 | `pcm_ext_enabled ? b_ram[address] : 0x00` | `src/mcu.cpp:842-846` |
| 7 | `pcm_ext_enabled ? b_ram2[address] : 0x00` | `src/mcu.cpp:847-850` |
| default | `0x00`（当前枚举已覆盖全部 0-15，实际不可达） | `src/mcu.cpp:851-853` |

### 1.7 page 1-15 写表（`src/mcu.cpp:1041-1068`）

| page | 写目标 | 证据 |
|---|---|---|
| 5 | `mcu_mk1` → `sram[address & 0x7fff]` | `src/mcu.cpp:1041-1044` |
| 10 | `!mcu_mk1` → `sram[address & 0x7fff]` | `src/mcu.cpp:1045-1048` |
| 12 | jv880 → `nvram[address & 0x7fff]` | `src/mcu.cpp:1049-1052` |
| 14 | jv880 → `cardram[address & 0x7fff]` | `src/mcu.cpp:1053-1056` |
| 7 | `pcm_ext_enabled` → `b_ram2[address]` | `src/mcu.cpp:1057-1060` |
| 6 | `pcm_ext_enabled` → `b_ram[address]` | `src/mcu.cpp:1061-1064` |
| 其余 | `Unknown write`（无副作用） | `src/mcu.cpp:1065-1068` |

注意：page 1-4/8/9/11/13/15 的写**全部无效**（与读映射 ROM/cardram 不对称）；6/7 关扩展时
读 `0x00`、写无效；page 11/13 无任何分支。

### 1.8 16/32 位访问

- `MCU_Read16(address)`：`address &= ~1`，按大端先高字节读两个 8 位（`src/mcu.cpp:876-883`）。
- `MCU_Read32(address)`：`address &= ~3`，顺序读 4 字节 `b0<<24|b1<<16|b2<<8|b3`（`src/mcu.cpp:885-894`）。
- `MCU_Write16(address,value)`：`address &= ~1`，先写高字节再写低字节（`src/mcu.cpp:1071-1076`）。
- 没有 `MCU_Write32`。
- 字访问的奇地址检查发生在**操作数层**：`MCU_Operand_Read/Write` 里 `ea & 1` 触发
  `EXCEPTION_SOURCE_ADDRESS_ERROR`，但**仍然继续执行读写**（`src/mcu_opcodes.cpp:498-504`、`528-535`）。
- 立即数操作数被写时触发 `EXCEPTION_SOURCE_INVALID_INSTRUCTION`（`src/mcu_opcodes.cpp:537-539`）。

### 1.9 设备窗口：`MCU_DeviceRead/Write`（`src/mcu.cpp:405-606`）

- 写：`address &= 0x7f`；`0x10-0x3f` 走 `TIMER_Write`（FRT1-3）（`src/mcu.cpp:408-412`）；
  `0x50-0x54` 走 `TIMER2_Write`（`src/mcu.cpp:413-417`）；`0x55` 不在范围。
- 读：`0x10-0x3f` → `TIMER_Read`（`src/mcu.cpp:529-532`）；`0x50-0x54` → `TIMER_Read2`（`src/mcu.cpp:533-536`）。
- 写特例：
  - `DEV_ADCSR`（0x68）：只改 `&0x7f` 位域；`data.bit7==0 && adf_rd` 时清 ADF 并清 ANALOG 中断；
    `data.bit6==0` 时清 ANALOG 中断；**提前 return**，不做统一的 `dev_register[address]=data`
    （`src/mcu.cpp:482-494`）。
  - `DEV_SSR`（0x5c）：清 TX/RX 标志并设定 `uart_tx_delay/uart_rx_delay = mcu.cycles + 3000`，
    清对应中断请求（`src/mcu.cpp:495-518`）。
  - 其余地址统一在函数末尾 `dev_register[address] = data`（`src/mcu.cpp:523`），所以大部分设备寄存器
    是"写存储"模型。
- 读特例（`src/mcu.cpp:537-604`）：
  - `DEV_ADDRAH..DEV_ADDRDL`、`DEV_SCR/SMR/TDR`、FRT2/3 若干返回 `dev_register`；
  - `DEV_ADCSR` 读时 `adf_rd = (dev_register[address] & 0x80) != 0`（`src/mcu.cpp:548-550`）；
  - `DEV_SSR` 读时 `ssr_rd = dev_register[address]`（`src/mcu.cpp:551-553`）；
  - `DEV_RDR` 返回 `uart_rx_byte`（`src/mcu.cpp:554-555`）；
  - `0x00` 读返回 `0xff`（即使写入会存到 `dev_register[0]`）（`src/mcu.cpp:556-557`）；
  - `DEV_P7DR`（0x0e）非 JV880 返回 `0xff`；JV880 按 `io_sd` 组合按键（`src/mcu.cpp:558-574`）；
  - `DEV_P9DR`（0x7f）按 `DEV_P9DDR` 方向位合成 cfg（`src/mcu.cpp:575-586`）；
  - 其余落 `return dev_register[address]`（`src/mcu.cpp:605`）。
- 设备复位 `MCU_DeviceReset`：只设 `DEV_RAME=0x80`、`DEV_SSR=0x80`（`src/mcu.cpp:608-614`）；
  `dev_register` 其余字节依赖全局零初始化**且复位不清理**。
- 模拟量采样 `MCU_AnalogSample` 把 10bit 值拆进 `ADDRAH/ADDRAL`：高 8 位 `value>>2`，低 2 位
  `(value<<6)&0xc0`（`src/mcu.cpp:387-388`）。

---

## 2. CPU 状态与寄存器模型

### 2.1 `mcu_t`（`src/mcu.h:161-172`，定义 `src/mcu.cpp:649`）

```c
struct mcu_t {
    uint16_t r[8];                  // 通用寄存器（始终 16 位）
    uint16_t pc;                    // 页内偏移，仅 16 位，自增不回卷 cp
    uint16_t sr;                    // 状态寄存器（实际有效位见 sr_mask）
    uint8_t cp, dp, ep, tp, br;     // 代码页 / 数据页 / 扩展页 / 临时页 / 基址
    uint8_t sleep;                  // SLEEP 指令置 1
    uint8_t ex_ignore;              // 跳过下一次中断轮询
    int32_t exception_pending;      // -1 = 无；0..2 = 异常源
    uint8_t interrupt_pending[INTERRUPT_SOURCE_MAX]; // 21 项
    uint8_t trapa_pending[16];
    uint64_t cycles;
};
```

- `INTERRUPT_SOURCE_MAX = 21`（`src/mcu_interrupt.h:26-49`）。
- 地址生成 `MCU_GetAddress(page, address) = (page << 16) + address`（`src/mcu.h:184-186`）。
- `MCU_ReadCode()` 用 `mcu.cp`：`MCU_GetAddress(mcu.cp, mcu.pc)`（`src/mcu.h:188-190`）。

### 2.2 页寄存器与操作数页选择

- 数据操作数页选择 `MCU_GetPageForRegister(reg)`（`src/mcu.h:208-215`）：
  - `reg >= 6`（r6/r7）→ `mcu.tp`；
  - `reg >= 4`（r4/r5）→ `mcu.ep`；
  - 其余（r0-r3）→ `mcu.dp`。
- 绝对短地址（`operand 0x00` + `reg==5`）用 `mcu.br` 作高字节、页为 0（`src/mcu_opcodes.cpp:589-596`）；
  绝对长地址（`operand 0x10` + `reg==5`）页取 `mcu.dp`（`src/mcu_opcodes.cpp:608-615`）。
- 间接操作数的 `ep` 在**读 opcode 之前**算出（`src/mcu_opcodes.cpp:646`）。

### 2.3 控制寄存器读写（`src/mcu.h:217-330`）

`MCU_ControlRegisterWrite(reg, siz, data)`：

- `siz=1`（字）：`reg 0` → `sr = data & sr_mask`；`reg 5` → `dp=data&0xff`；`reg 4` → `ep`；
  `reg 3` → `br`；其他 → `MCU_ErrorTrap()`（`src/mcu.h:219-242`）。
- `siz=0`（字节）：`reg 1` → `sr` 低 8 位再 `& sr_mask`；`reg 3` → `br`；`reg 4` → `ep`；
  `reg 5` → `dp`；`reg 7` → `tp`；其他 → trap（`src/mcu.h:244-271`）。

`MCU_ControlRegisterRead(reg, siz)`：

- `siz=1`：`reg 0` → `sr & sr_mask`；`reg 5/4/3` → 对应值复制到高低两字节；其他 trap（`src/mcu.h:275-300`）。
- `siz=0`：`reg 1` → `sr & sr_mask`；`reg 3/4/5/7` 读 `br/ep/dp/tp`；其他 trap（`src/mcu.h:302-328`）。

注意寄存器编号在字节/字下的映射不同（字用 0/3/4/5，字节用 1/3/4/5/7）；翻错编号会得到
不同的 trap 行为。

### 2.4 栈

- `MCU_PushStack(data)`：`r7 & 1` 先触发 ADDRESS_ERROR（不清除，不中止），然后 `r7 -= 2`，
  `MCU_Write16(r7, data)`（大端）（`src/mcu.h:340-346`）。
- `MCU_PopStack()`：同样先查奇地址，读 `MCU_Read16(r7)`，再 `r7 += 2`（`src/mcu.h:348-356`）。
- 中断入栈顺序 `pc, cp, sr`（`src/mcu_interrupt.cpp:23-25`）；`RTE` 出栈顺序 `sr, cp, pc`
  （`src/mcu_opcodes.cpp:223-228`）。`PJSR` 也是先 `pc` 后 `cp`（`src/mcu_opcodes.cpp:206-207`），
  与 `jmp @Rn` 的出栈顺序一致（`src/mcu_opcodes.cpp:345-349`）。
- `LDM/STM`（`src/mcu_opcodes.cpp:154-183`）：`LDM` 按 bit0..bit7 顺序 pop，**跳过 r7 的写入**
  （`if (i != 7)`，`src/mcu_opcodes.cpp:163-164`）；`STM` 按 bit7..bit0 顺序 push，r7 压入 `r7-2`
  （`src/mcu_opcodes.cpp:173-181`）。

### 2.5 待处理队列与标志

- `MCU_Interrupt_SetRequest(interrupt,value)`：直接 `interrupt_pending[interrupt] = value`，
  无掩码、无优先级逻辑（`src/mcu_interrupt.cpp:35-38`）。
- `MCU_Interrupt_Exception(exc)`：`exception_pending = exc`，**覆盖**旧值（`src/mcu_interrupt.cpp:40-49`）。
- `MCU_Interrupt_TRAPA(v)`：`trapa_pending[v] = 1`（`src/mcu_interrupt.cpp:51-54`）。
- `mcu.ex_ignore`：置 1 的指令有 `RTE`（`src/mcu_opcodes.cpp:228`）、`LDC`（`:1000`）、
  `ORC/ANDC`（`:887`、`:911`）。宿主循环里"若 `ex_ignore` 则本步跳过中断轮询并清零"
  （`src/mcu.cpp:1343-1346`），即这些指令后**至少执行一条指令**才允许中断。
- `mcu.sleep`：SLEEP 置 1（`src/mcu_opcodes.cpp:126-129`）；宿主循环睡眠时仍每步轮询中断，
  `MCU_Interrupt_Start` 置 `sleep=0`（`src/mcu_interrupt.cpp:32`）。睡眠时 `pc` 不前进
  （`src/mcu.cpp:1348-1349`），但 `cycles += 12` 与所有设备时钟照常（`src/mcu.cpp:1351-1446`）。

## 3. 状态寄存器与标志语义

### 3.1 位定义

- `sr_mask = 0x870f`（`src/mcu.h:90`）；`STATUS_T=0x8000`、`N=0x08`、`Z=0x04`、`V=0x02`、`C=0x01`、
  `STATUS_INT_MASK=0x700`（`src/mcu.h:91-98`）。
- `MCU_SetStatus(cond,mask)`：置位/清位，**不做掩码**（`src/mcu.h:332-338`）。
- `MCU_SetStatusCommon(val,siz)`：按 `siz` 截 16/8 位；置 N（bit15/bit7）、Z（==0）、**V=0**；
  **C 保持原值**（`src/mcu_opcodes.cpp:675-687`）。
- `MCU_ADD_Common` / `MCU_SUB_Common`：按操作数宽度分别用无符号算 C（bit16/bit8）、有符号算 V，
  然后统一 `SetStatus(N,Z,C,V)`（`src/mcu_opcodes.cpp:22-70`、`72-120`）。
- `MCU_Interrupt_Start` 会清 `STATUS_T`（`src/mcu_interrupt.cpp:26`）。
- `RTE` 直接 `mcu.sr = MCU_PopStack()`，**没有 `& sr_mask`**（`src/mcu_opcodes.cpp:225`），
  可能引入 mask 外位；`LDC`/控制寄存器写才有掩码（字写 `src/mcu.h:223-225`，
  字节写 `src/mcu.h:247-250`）。

### 3.2 哪些指令改哪些标志（GT 实际实现，不是 H8 手册）

| 指令族 | N | Z | V | C | 证据 |
|---|---|---|---|---|---|
| `Short MOVE`(0x50-57) | 按字节 | 按字节 | 置 0 | 保持 | `src/mcu_opcodes.cpp:694-701` |
| `MOVI`(0x58-5F) | 按字 | 按字 | 置 0 | 保持 | `src/mcu_opcodes.cpp:703-711` |
| `MOVL/MOVS`(0x60-7F) | 按 bit3 宽度 | 同 | 置 0 | 保持 | `src/mcu_opcodes.cpp:755-800` |
| `MOVF`(0x80-9F) | 宽度取反（见下） | 同 | 置 0 | 保持 | `src/mcu_opcodes.cpp:713-753` |
| `MOVG`/`XCH`(表 0x10/0x12) | 读与写都置 | 读与写都置 | 置 0 | 保持 | `src/mcu_opcodes.cpp:1077-1092`；XCH 除外 `:1061-1074` |
| `ADD/ADDQ/SUB/NEG/CMP` | 置 | 置 | 置 | 置 | `src/mcu_opcodes.cpp:1141-1193`、`:978-983`、`:1134-1139` |
| `ADDX/SUBX` | 置 | 置（ADDX 有粘滞） | 置 | 置 | `src/mcu_opcodes.cpp:1434-1466` |
| `AND/OR/XOR/NOT` | 置 | 置 | 置 0 | 保持 | `src/mcu_opcodes.cpp:1205-1217`、`:1127-1132`、`:1427-1432`、`:971-977` |
| 移位/循环 7 种 | 置 | 置 | 置 0 | 移出位 | `src/mcu_opcodes.cpp:1219-1323` |
| `MULXU` | 见下 | 置 | 清 0 | 清 0 | `src/mcu_opcodes.cpp:1325-1352` |
| `DIVXU` | 见下 | 见下 | 见下 | 清 0 | `src/mcu_opcodes.cpp:1354-1417` |
| `CLR` | 清 0 | 置 1 | 清 0 | 清 0 | `src/mcu_opcodes.cpp:939-946` |
| `TST` | 置 | 置 | 置 0 | **清 0** | `src/mcu_opcodes.cpp:947-952` |
| `EXTU` | 清 0 | 置 | 清 0 | 清 0 | `src/mcu_opcodes.cpp:953-961` |
| `EXTS` | 用**扩展前**值 | 同 | 置 0 | **保持** | `src/mcu_opcodes.cpp:984-989` |
| `SWAP` | 按字 | 按字 | 置 0 | 保持 | `src/mcu_opcodes.cpp:962-970` |
| `BSET/BCLR/BTST/BNOTI/BTSTI` | 只动 | Z | 保持 | 保持 | `src/mcu_opcodes.cpp:875-935`、`:1009-1125` |
| `ORC/ANDC`（reg>=2） | 置 | 置 | 置 0 | 保持 | `src/mcu_opcodes.cpp:875-897`、`:899-921` |
| `LDC/STC/XCH/ADDS/SUBS/NOP/SLEEP/跳转` | 不动 | 不动 | 不动 | 不动 | `src/mcu_opcodes.cpp:996-1007`、`:1195-1203`、`:1419-1425` |

必须照抄的 GT 细节：

- **MOVG 读写都置 N/Z/V**（`src/mcu_opcodes.cpp:1077-1080`、`1084-1092`）。H8 手册只要求读置，
  但 GT 写也置；`XCH` 不置（`:1061-1074`）。
- **MOVF 的尺寸语义"内外不一致"**（`src/mcu_opcodes.cpp:713-753`）：`opcode.bit3` 决定**内存访问宽度**，
  而**寄存器传输与标志宽度取反**——`bit3=1` 时读 16 位但只写 r[reg] 低字节、按字节置标志；
  `bit3=0` 时读 8 位但写整个 r[reg]、按字置标志；写方向同理。必须逐字节复刻。
- **EXTS 用扩展前的 16 位值算 N/Z** 且不动 C（`src/mcu_opcodes.cpp:984-989`）；
  与之对比 `EXTU` 显式清 C（`:953-961`）。
- **ADDX 的粘滞 Z**：先按结果算 Z，若进位前 Z==0 则强制 Z=0（`src/mcu_opcodes.cpp:1441-1442`）。
- **SUBS 字节形式会覆盖整个 16 位寄存器**（`mcu.r[reg] = t1 - (int8_t)t2`，`src/mcu_opcodes.cpp:1195-1203`），
  不像其它字节指令那样只改低字节。
- **MULXU 的 N 取完整积的 bit31（字）/bit15（字节）并标 FIXME**（`src/mcu_opcodes.cpp:1339`、`:1345`）；
  字模式写到 `r[reg|0]=t1>>16; r[reg|1]=t1`（`opcode_reg &= ~1`，`:1336-1339`）。
- **DIVXU 除零不抛异常而是 `MCU_ErrorTrap()`**（`src/mcu_opcodes.cpp:1360-1368`），随后置
  N=0,Z=1,V=0,C=0 并返回；商溢出时 N=0,Z=0,V=1,C=0，不写回（`:1379-1392`、`:1401-1415`）。
- **`MCU_Opcode_Short_CMP` 是 `r[reg] - imm`**（字/字节由 opcode.bit3）且立即数按无符号拼接
  （`src/mcu_opcodes.cpp:802-818`）。

## 4. 指令获取与分派

### 4.1 取指与 `pc` 语义

- `MCU_ReadCodeAdvance()`：读 `MCU_GetAddress(mcu.cp, mcu.pc)` 后 `mcu.pc++`（`src/mcu.h:192-196`）。
  `pc` 是 `uint16_t`，0xffff→0x0000 回卷**不会进位到 `cp`**；跨页必须显式 `pjmp/pjsr`。
- `MCU_ReadInstruction()`：取 1 个操作数编码字节，调用 `MCU_Operand_Table[operand](operand)`，
  然后检查 `if (mcu.sr & STATUS_T) MCU_Interrupt_Exception(EXCEPTION_SOURCE_TRACE)`
  （`src/mcu.cpp:1078-1088`）。即 trace 异常在**指令执行后**排队，下一步才开始处理。
- 睡眠时宿主不调用 `MCU_ReadInstruction`，`pc` 停在 SLEEP 后位置（`src/mcu.cpp:1348-1349`）。

### 4.2 操作数编码与取指顺序（`MCU_Operand_General`，`src/mcu_opcodes.cpp:543-673`）

顺序很重要，翻译核必须按同样次序消耗字节（错误会直接导致 `pc` 分歧）：

1. `siz = (operand & 0x08) ? WORD : BYTE`；`reg = operand & 7`（`:558-562`）。
2. 按 `operand & 0xf0` 决定寻址并**立即取相应字节**：
   - `0xa0` 直接寄存器；`0xd0` 间接；
   - `0xe0` 间接 + disp8（有符号，取 1 字节）；`0xf0` 间接 + disp16（大端，取 2 字节）；
   - `0xb0` 间接、预递减；`0xc0` 间接、后递增（`:565-588`）；
   - `0x00`：`reg==5` → 绝对短 (`br<<8 | imm8`，页 0)；`reg==4` → 立即数（按 `siz` 取 1/2 字节）；
     其它 `reg` 组合不产生 `type` 变化，保持 `GENERAL_DIRECT`（`:589-607`）；
   - `0x10`：`reg==5` → 绝对长（2 字节，页 `dp`）（`:608-616`）。
3. 间接寻址：`INCREASE_DECREASE` 先 `r[reg] -= (siz||reg==7)?2:1` 再算 `ea = r[reg]+disp`；
   `INCREASE_INCREASE` 先算 `ea` 再 `r[reg] += ...`；`ea &= 0xffff`；`ep = MCU_GetPageForRegister(reg)`
   （`:618-647`）。绝对寻址 `ea = addr & 0xffff; ep = addrpage`（`:648-653`）。
4. 取操作码字节 `opcode`；若 `opcode == 0x00` 则**再取 1 字节**作为扩展操作码
   （`opcode_extended=1`，`:655-660`）。
5. `opcode_reg = opcode & 7; opcode >>= 3;` 然后 `MCU_Opcode_Table[opcode](opcode, opcode_reg)`
   （`:661-672`）。

### 4.3 表结构

- `MCU_Operand_Table[256]`（`src/mcu_opcodes.cpp:1468-1725`）：
  - `0x00` NOP；`0x01/0x06/0x07/0x10/0x11` = `MCU_Jump_JMP`（不同子形态）；
    `0x02` LDM；`0x03` PJSR；`0x04/0x05/0x0c/0x0d/0x15/0x1d/0xa0-0xff` = General；
    `0x08` TRAPA；`0x0a` RTE；`0x0e/0x1e` BSR；`0x12` STM；`0x13` PJMP；`0x14/0x1c` RTD；
    `0x18` JSR；`0x19` RTS；`0x1a` SLEEP；`0x20-0x3f` Bcc；
    `0x40-0x4f` Short CMP；`0x50-0x57` MOVE；`0x58-0x5f` MOVI；`0x60-0x6f` MOVL；
    `0x70-0x7f` MOVS；`0x80-0x9f` MOVF；
    `0x09/0x0b/0x0f/0x16/0x17/0x1b/0x1f` 未实现（trap）。
- `MCU_Opcode_Table[32]`（`src/mcu_opcodes.cpp:1727-1760`）：
  `0x00` MOVG_Immediate/CMP-imm；`0x01` ADDQ；`0x02` CLR 族；`0x03` 移位/循环族；
  `0x04/0x06` ADD/SUB；`0x05/0x07` ADDS/SUBS；`0x08` OR；`0x09` BSET/ORC；
  `0x0a` AND；`0x0b` BCLR/ANDC；`0x0c` XOR；`0x0d` 未实现；`0x0e` CMP；`0x0f` BTST；
  `0x10/0x12` MOVG；`0x11` LDC；`0x13` STC；`0x14` ADDX；`0x15` MULXU；`0x16` SUBX；
  `0x17` DIVXU；`0x18/0x19` BSET；`0x1a/0x1b` BCLR；`0x1c/0x1d` BNOTI；`0x1e/0x1f` BTSTI。

### 4.4 跳转/调用细节

- `PJSR`：取 page、addr 后压 `pc`、`cp`，设 `cp=page; pc=addr`；`cp==0x27` 的 `+=0` 为空操作
  （`src/mcu_opcodes.cpp:198-212`）。
- `JSR`：只压 `pc`，页不变（`:214-221`）。
- `RTS`：只弹 `pc`（`:311-314`）。
- `RTE`：`sr=Pop(); cp=Pop(); pc=Pop(); ex_ignore=1`（`:223-229`）。
- `RTD`：取 disp8 后弹 `pc`，`operand==0x14` 时 `r7 += disp` 且奇数 → `MCU_ErrorTrap`
  （不是异常）；`operand==0x1c` 直接 trap（`:316-336`）。
- `jmp @Rn`（opcode 0x19）：`cp = Pop(); pc = Pop()`（`:345-349`）；
  `jmp @(Rn)` 形式（`opcode_h==0x19`）压 `pc/cp` 后 `opcode_l &= ~1; cp = r[l] & 0xff; pc = r[l+1]`
  （`:350-357`）；`jmp @Rn`（0x1a）只设 `pc`，`0x1b` 压 `pc` 后设 `pc`（`:358-366`）。
- `operand==0x01`（次字节 `opcode>>3==0x17`）：`r[reg]--`，只要结果 != 0xffff 就 `pc += disp`
  （`:372-390`）。
- `operand==0x06`：仅当 `STATUS_Z` 为 1 时才 `r[reg]--` 并按同为 0x17 的次字节决定是否分支；
  `operand==0x07` 则在 `STATUS_Z` 为 0 时做同样动作（`:391-443`）。
- `Bcc`：disp 8/16 由 `operand & 0x10` 决定；条件译码用 sr 的快照（`src/mcu_opcodes.cpp:231-309`）。
- `TRAPA`：只接受后续字节 `0x10-0x1f`，置 `trapa_pending[opcode&0xf]`；否则 trap（`:185-196`）。
  真正入向量发生在下一次 `MCU_Interrupt_Handle`（见第 6 节）。

## 5. 时序与宿主调度（`work_thread`）

### 5.1 循环顺序（`src/mcu.cpp:1327-1514`）

每轮严格按以下顺序：

1. **音频背压**：按 `pcm.config_reg_3c & 0x40` 将 `sample_write_ptr &= ~3` 或 `&= ~1`；若
   `sample_read_ptr == sample_write_ptr` 则解锁互斥并 `SDL_Delay(1)` 自旋，直到音频回调消费
   （`src/mcu.cpp:1329-1341`）。纯离线翻译核可跳过，但**不得改变循环内其它状态**。
2. **中断**：`if (!mcu.ex_ignore) MCU_Interrupt_Handle(); else mcu.ex_ignore = 0;`（`src/mcu.cpp:1343-1346`）。
3. **执行**：`if (!mcu.sleep) MCU_ReadInstruction();`（`src/mcu.cpp:1348-1349`）。
4. **周期**：`mcu.cycles += 12;`（固定 12，不区分指令）（`src/mcu.cpp:1351`）。
5. `-tracepc` 窗口日志（写文件，不影响状态）（`src/mcu.cpp:1353-1372`）。
6. `-demo` 按键序列：按 `demo_start + demo_off[]` 更新 `mcu_button_pressed`
   （`src/mcu.cpp:1374-1400`）。
7. `-mocknote`：向 SM UART 注入 `0x90 60 100`（调用 `SM_PostUART`）（`src/mcu.cpp:1402-1415`）。
8. `-savesnap`：把整机状态写入 `demo_snap.bin`（`state_save`，字段顺序见下）（`src/mcu.cpp:1417-1429`）。
9. `PCM_Update(mcu.cycles)`（`src/mcu.cpp:1434`）。
10. `TIMER_Clock(mcu.cycles)`（`src/mcu.cpp:1436`）。
11. **要么** `SM_Update(mcu.cycles)`（当 `!mcu_mk1 && !mcu_jv880 && !mcu_scb55`），
    **要么** `MCU_UpdateUART_RX(); MCU_UpdateUART_TX();`（`src/mcu.cpp:1438-1444`）。
12. `MCU_UpdateAnalog(mcu.cycles)`（`src/mcu.cpp:1446`）。
13. MK1：`ga_lcd_counter` 递减到 0 时 `MCU_GA_SetGAInt(1,0)` 再 `(1,1)`（`src/mcu.cpp:1502-1513`）。

要点：PCM 在 TIMER 之前；SM/analog 在 TIMER 之后；所有设备推进都在 `cycles += 12` 之后。

### 5.2 周期推进只发生在宿主循环

`mcu.cycles` 仅在 `src/mcu.cpp:1351` 自增；CPU 指令本身不改周期。设备各自的时钟：

- **TIMER**：`while (timer_cycles*2 < cycles)`，每 tick 依次处理 FRT0/1/2 与 TMR，最后
  `timer_cycles++`（`src/mcu_timer.cpp:205-335`，尤其 `:208`、`:333`）。FRT 预分频按 `tcr & 3`
  用 `timer_cycles & {3,7,31}`；扩展时钟 `o/2`（mk2）或 `o/4`（mk1）（`:215-241`）。
  TMR 预分频按 `timer.tcr & 7`（`:271-302`）。标志与中断见第 7 节。
- **PCM**：`while (pcm.cycles < cycles)`，每内步 `pcm.cycles += (slots+1)*25`
  （JV880 再 `*25/29`），`slots` 由 `config_reg_3d` 决定（`src/pcm.cpp:584-591`、`1670-1675`）。
- **SM**：`while (sm.cycles < cycles * 5)`；每步先 `SM_HandleInterrupt()`，非 sleep 则取指执行，
  然后 `sm.cycles += 12*4`，再 `SM_UpdateTimer()`、`SM_UpdateUART()`（`src/submcu.cpp:1417-1435`）。
  即 SM 约 5× 主频、每 SM 指令 48 "SM cycles"。
- **Analog**：`MCU_UpdateAnalog` 每步调用；`ADCSR.bit5` 启动后延迟 200 cycles 出结果
  （`analog_end_time`），单次模式清 bit5，扫描模式按 `ctrl&3` 采多通道，置 ADF(bit7)，
  若 ADIE(bit6) 则请求 ANALOG 中断（`src/mcu.cpp:616-647`）。
- **UART**：RX/TX 延迟均用 `mcu.cycles + 3000`（`src/mcu.cpp:500`、`:505`、`:1146`、`:1164`）。

### 5.3 快照字段顺序（如要与 GT 的 `demo_snap.bin` 互通）

`state_save` 顺序（`src/mcu.cpp:1225-1263`）：
`mcu` 整结构 → `ram` → `sram` → `dev_register` → `frt` → `timer` → `timer_cycles` →
`timer_tempreg` → `mcu_p0_data/mcu_p1_data` → `io_sd` → `sw_pos` → `ad_val` → `ad_nibble` →
`ga_int` → `ga_int_enable` → `ga_int_trigger` → `ga_lcd_counter` → `analog_end_time` →
`uart_buffer/uart_write_ptr/uart_read_ptr/uart_rx_byte` → `sm` → `sm_ram` → `sm_shared_ram` →
`sm_access` → `sm_p0_dir/sm_p1_dir` → `sm_device_mode` → `sm_cts` → `sm_timer_cycles` →
`sm_timer_prescaler/sm_timer_counter` → `pcm` → `LCD_StateSave`。`state_load` 对应
（`src/mcu.cpp:1265-1303`）。

注意：`uart_rx_delay/uart_tx_delay`（`src/mcu.cpp:402-403`）**不在快照里**，读档后从旧值继续；
且 `mcu`/`frt`/`pcm` 等按 C 结构体原始内存 dump，布局依赖编译器的 padding，翻译核若要读 GT 的
`demo_snap.bin` 必须保持同样的结构体布局与字段顺序。

## 6. 中断与异常（`src/mcu_interrupt.cpp`）

### 6.1 请求/异常入口

- 请求：裸存数组（`src/mcu_interrupt.cpp:35-38`）。
- 异常：`exception_pending` 单值覆盖（`src/mcu_interrupt.cpp:40-49`）。
- TRAPA：`trapa_pending[16]`（`src/mcu_interrupt.cpp:51-54`）。
- 当前源码中**没有任何路径设置 `INTERRUPT_SOURCE_NMI`**（`SetRequest` 全部调用点见 §7），
  因此 NMI 分支是死代码；但语义仍需保留。

### 6.2 `MCU_Interrupt_Handle` 精确顺序（`src/mcu_interrupt.cpp:64-209`）

1. **TRAPA 扫描**：`i = 0..15`，命中即清 `trapa_pending[i]`，以向量 `VECTOR_TRAPA_0 + i`、
   `mask = -1` 进入（`src/mcu_interrupt.cpp:84-92`）。
2. **硬件异常**：`exception_pending >= 0` 时映射
   `ADDRESS_ERROR→VECTOR_ADDRESS_ERROR`、`INVALID_INSTRUCTION→VECTOR_INVALID_INSTRUCTION`、
   `TRACE→VECTOR_TRACE`，均 `mask=-1`；随后 `exception_pending=-1`（`src/mcu_interrupt.cpp:93-110`）。
3. **NMI**：`interrupt_pending[NMI]` 非 0 时以 `mask=7` 进入；**清除被注释掉**，即 NMI 不会被
   消费（`src/mcu_interrupt.cpp:111-116`）。
4. **按源序号轮询**：`i = NMI+1 .. INTERRUPT_SOURCE_MAX-1`（`src/mcu_interrupt.cpp:117-208`）：
   - `mask = (mcu.sr >> 8) & 7`（`:117`）；
   - `IRQ0` 需 `DEV_P1CR(0x7c) & 0x20`，level = `IPRA>>4 & 7`（`:126-131`）；
   - `IRQ1` 需 `DEV_P1CR & 0x40`，level = `IPRA & 7`（`:132-137`）；
   - FRT0/1/2 OCIA/OCIB/FOVI 与 TMR CMIA/CMIB/OVI、ANALOG、UART_RX/TX 的 level 分别取
     `IPRB/IPRC/IPRD` 的对应半字节（`:138-197`）；
   - **FRT ICI 源（`FRT0/1/2_ICI`）没有 case**，`vector=-1, level=0`，永远不会被选中（`:198-200`）；
   - 命中条件 `(int32_t)mask < level`；第一个满足的源即进入 `StartVector(vector, level)` 并 return
     （`:202-207`）。注意这是"遍历顺序优先"，不是全局最高优先级；level 0（IPR 半字节为 0）
     永远不触发。

### 6.3 入向量与入栈

- `MCU_Interrupt_StartVector(vector, mask)`：先 `address = MCU_GetVectorAddress(vector)`，再
  `MCU_Interrupt_Start(mask)`，再设 `cp/pc`（`src/mcu_interrupt.cpp:56-62`）。
- `MCU_GetVectorAddress(v) = MCU_Read32(v*4)`（`src/mcu.h:203-206`）；`MCU_Read32` 大端读 4 字节
  （`src/mcu.cpp:885-894`）。因此向量表项字节布局为 `[b0(忽略)][b1=cp][b2=pc_hi][b3=pc_lo]`：
  `cp = address>>16`（uint8_t 截断后是 b1），`pc = address & 0xffff`（`src/mcu_interrupt.cpp:60-61`）。
  也就是说 b0 不参与，`address>>16` 只取到 b1。
- `MCU_Interrupt_Start(mask)`（`src/mcu_interrupt.cpp:21-33`）：
  压 `pc`、`cp`、`sr`（top=sr），清 `STATUS_T`；`mask>=0` 时清 `STATUS_INT_MASK` 再置
  `mask<<8`；`sleep=0`。
- 复位向量在 `MCU_Reset` 中同样用 `MCU_GetVectorAddress(VECTOR_RESET)` 并截断
  （`src/mcu.cpp:1116-1118`）。

必须逐位复刻的点：TRAPA 优先于异常优先于 NMI 优先于外设；trapa/异常保留 I 掩码（`mask=-1`），
外设更新 I 掩码；TRAPA 是延迟处理（不是指令内立即入栈）；向量读取在压栈**之前**发生
（`src/mcu_interrupt.cpp:58-59`），若压栈触发 ADDRESS_ERROR 会覆盖 `exception_pending`。

## 7. 设备模型接口与共享全局

翻译核应**直接链接 GT 的设备实现**，只替换 CPU。CPU 侧会调用的外部 API：

| API | 声明/定义 | 调用点 | 说明 |
|---|---|---|---|
| `PCM_Read/PCM_Write` | `src/pcm.h:70-71`，`src/pcm.cpp:79/210` | `src/mcu.cpp:683`、`:946` | 0x3f 寄存器镜像；写 voice mask 有 pending/updating 语义 |
| `PCM_ReadExt/PCM_WriteExt` | `src/pcm.h:75-76`，`src/pcm.cpp:288/298` | `src/mcu.cpp:687`、`:950` | 仅 `pcm_ext_active` 时路由（0xe800-0xe83f） |
| `PCM_Update` | `src/pcm.h:78`，`src/pcm.cpp:584` | `src/mcu.cpp:1434` | 每个宿主步调用；内部按自己的节拍推进，可能置 IRQ0/GA int（`src/pcm.cpp:1527-1538`） |
| `PCM_Reset` | `src/pcm.h:77`，`src/pcm.cpp:310-313` | `src/mcu.cpp:2349` | `memset(&pcm)`；`pcm.cycles` 也清零 |
| `TIMER_Write/TIMER_Read` | `src/mcu_timer.h:39-40` | `src/mcu.cpp:410/531` | FRT0-2；写 FCSR 清标志并清中断 |
| `TIMER2_Write/TIMER_Read2` | `src/mcu_timer.h:43-44` | `src/mcu.cpp:415/535` | TMR |
| `TIMER_Clock` | `src/mcu_timer.h:41` | `src/mcu.cpp:1436` | `timer_cycles*2 < cycles` |
| `SM_SysRead/SM_SysWrite` | `src/submcu.h:53-54`，`src/submcu.cpp:218/259` | `src/mcu.cpp:691/954` | 0xec00-0xefff 窗口；`SM_SysWrite` 可写 P0/P1（`src/submcu.cpp:241-248`） |
| `SM_Update` | `src/submcu.h:47`，`src/submcu.cpp:1417` | `src/mcu.cpp:1439` | 5× 时钟驱动整个子 MCU |
| `SM_PostUART` | `src/submcu.h:55`，`src/submcu.cpp:1387-1391` | `src/mcu.cpp:1409-1411` | 与主 MCU 共用 `uart_buffer` |
| `SM_Reset` | `src/submcu.h:46`，`src/submcu.cpp:316-320` | `src/mcu.cpp:2348` | |
| `LCD_Write/LCD_Enable` | `src/lcd.h:32-33`，`src/lcd.cpp:87/39` | `src/mcu.cpp:910/914`、`:1014/1019/1010` | MK2 走 0xe404/0xe405/0xe401；MK1 走 0xf104/0xf105 |
| `LCD_Init/Update/QuitRequested` | `src/lcd.h:30/38/36` | `src/mcu.cpp:2344/1536/1533` | 宿主/窗口，翻译核可替换 |
| `MCU_GA_SetGAInt` | `src/mcu.h:454`，`src/mcu.cpp:1749-1760` | `src/submcu.cpp:203`、`src/pcm.cpp:228/1535`、`src/mcu.cpp:1509-1510` | 触发 `ga_int_trigger`；按 `ga_int_enable` 门控 |
| `MCU_ReadP0/P1`、`MCU_WriteP0/P1` | `src/mcu.h:450-453`，`src/mcu.cpp:1559-1589` | `src/submcu.cpp:284/288/243/247` | SM 访问主 MCU 端口；P1 由 `mcu_p0_data` 与按键决定 |
| `MCU_PostUART` | `src/mcu.h:459`，`src/mcu.cpp:1130-1134` | MIDI/前端 | 环形缓冲写入 |
| `MCU_UpdateUART_RX/TX` | `src/mcu.cpp:1136/1156` | `src/mcu.cpp:1442-1443` | 仅非 SM romset 使用 |
| `MCU_UpdateAnalog` | `src/mcu.cpp:616` | `src/mcu.cpp:1446` | ADC 采样与中断 |
| `MCU_PostSample` | `src/mcu.h:458`，`src/mcu.cpp:1727-1747` | `src/pcm.cpp:676/701` | 主音量/量化/写音频环形缓冲 |
| `MCU_EncoderTrigger` | `src/mcu.h:456`，`src/mcu.cpp:1762-1767` | `src/lcd.cpp:549/551` | JV880 旋钮 |

共享全局状态（翻译核必须与 GT 设备模型**共用同一份**或逐位复制）：

- `uint8_t dev_register[0x80]`（`src/mcu.h:88`，定义 `src/mcu.cpp:199`）。
- `uint8_t rom1[0x8000]`、`rom2[0x80000]`、`ram[0x400]`、`sram[0x8000]`、`nvram/cardram`、
  `b_ram/b_ram2`（`src/mcu.cpp:651-658`）；`rom2_mask`（`:661`）。
- `frt[3]`、`timer`、`timer_cycles`、`timer_tempreg`（`src/mcu_timer.cpp:22-26`）。
- `pcm`（`src/pcm.h:63`）、`waverom1/2/3/card/exp`（`src/pcm.h:64-68`）。
- `sm`、`sm_rom`（`src/submcu.h:42-44`）以及 `sm_ram/sm_shared_ram/sm_access/sm_device_mode/...`
  （序列化于 `src/mcu.cpp:1213-1219`、`1250-1260`）。
- 主 MCU 侧宿主状态：`io_sd/sw_pos/ad_val/ad_nibble/adf_rd/ssr_rd/analog_end_time`（`src/mcu.cpp:201-204`、
  `391-395`）、`ga_int[8]/ga_int_enable/ga_int_trigger/ga_lcd_counter`（`src/mcu.cpp:193-196`）、
  `uart_buffer/uart_write_ptr/uart_read_ptr/uart_rx_byte/uart_rx_delay/uart_tx_delay`（`src/mcu.cpp:397-403`）、
  `mcu_button_pressed`（`src/mcu.cpp:206`）、`mcu_p0_data/mcu_p1_data`（`src/mcu.cpp:1556-1557`）、
  `master_gain`（`src/mcu.cpp:191`）。
- 型号/功能全局：`romset`、`mcu_mk1/mcu_cm300/mcu_st/mcu_jv880/mcu_scb55/mcu_sc155`、
  `pcm_float`、`master_gain`（`pcm_ext_*` 已随 M5 回滚删除；`src/mcu.cpp` 顶部）。

## 8. 初始化顺序

`main` 中与执行环境有关的顺序（`src/mcu.cpp:2196-2353`）：

1. `LCD_SetBackPath(basePath + "/back.data")`（`src/mcu.cpp:2196`）。
2. `memset(&mcu, 0, sizeof(mcu_t))`（`src/mcu.cpp:2198`）——注意随后 `MCU_Init` 会再做一次。
3. 读 `rom1`（必须 0x8000 字节）（`src/mcu.cpp:2201-2207`）；读 `rom2`（0x80000 或半长），
   设定 `rom2_mask = rom2_read-1`（`src/mcu.cpp:2209-2221`）。
4. 读并 `unscramble` waverom；非 JV880 读 `sm_rom`（0x1000）（`src/mcu.cpp:2223-2319`）。
   型号分支：`mcu_mk1`、`mcu_jv880`、其余（含 SCB55）三套尺寸/文件表（`src/mcu.cpp:90-154`）。
5. `SDL_Init(AUDIO|VIDEO|TIMER)`（`src/mcu.cpp:2324`）；`MCU_OpenAudio`（`:2331`）；
   `MIDI_Init`（`:2338`，失败仅告警）。
6. `LCD_Init()`（`:2344`）→ `MCU_Init()`（`:2345`，`memset(&mcu)`）→ `MCU_PatchROM()`（`:2346`）
   → `MCU_Reset()`（`:2347`）→ `SM_Reset()`（`:2348`）→ `PCM_Reset()`（`:2349`）
   → 可选 `MIDI_Reset(GS/GM)`（`:2351`）→ `MCU_Run()`（`:2353`）。
7. `MCU_Run` 建 `work_thread`；主线程只跑 `LCD_Update` + `SDL_Delay(15)`（`src/mcu.cpp:1524-1542`）。

初始化陷阱：

- `MCU_Reset`（`src/mcu.cpp:1095-1128`）只清 8 个通用寄存器、`pc`、`sr=0x700`、`cp/dp/ep/tp/br=0`、
  取复位向量、`exception_pending=-1`、`MCU_DeviceReset`；**不清** `sleep/ex_ignore/cycles/interrupt_pending/
  trapa_pending`。这些依赖之前的 `MCU_Init`/`memset`。复位时设备寄存器也不清零（只 RAME/SSR）。
- `TIMER_Reset`（`src/mcu_timer.cpp:41-47`）在 `src/` 中**没有任何调用点**；FRT/TMR 初值来自全局零初始化。
- `MCU_PatchROM` 目前是空函数（仅注释掉的扩展补丁），`pcm_ext_active` 保持 0
  （`src/mcu.cpp:1544-1554`）。
- 机型标志在参数解析后设置，JV880 还会改 LCD 尺寸/颜色与 `rom2_mask`（`src/mcu.cpp:2122-2161`）。
- 主机音频参数：`spec.freq = (mcu_mk1 || mcu_jv880) ? 64000 : 66207`（`src/mcu.cpp:1666`）。

## 9. `src/rt` drop-in 检查清单

翻译核要达到与 GT 0 分歧，`src/rt` 必须实现/保证：

1. **内存模型逐条件链复刻**：`MCU_Read_impl`/`MCU_Write` 的所有分支顺序、页表、`address_rom`
   位19 处理、RAME 门控、page 6/7 `pcm_ext_enabled` 门控、MK1/MK2 双路径（§1）。
2. **地址分解完全一致**：page=`(addr>>16)&0xf`、低 16 位截断、`&0x7f`/`&0x3f`/`&0x7fff`/`&0x3fff`
   等各窗口掩码；`Read16/32` 与 `Write16` 的大端序（`src/mcu.cpp:876-894`、`1071-1076`）。
3. **读副作用一次且仅一次**：`ADCSR` 读的 `adf_rd`、`SSR` 读的 `ssr_rd`、`0xe402/0xf106` 的
   `ga_int_trigger` 清除与 IRQ 清、MK1 `0xf000` 读的 `io_sd`/`LCD_Enable`/按键合成（§1.2-1.3、§1.9）。
4. **设备写语义**：`MCU_DeviceWrite` 的特例提前返回、统一末尾存储、TIMER/FRT 路由阈值
   `0x10-0x3f`/`0x50-0x54`（`src/mcu.cpp:405-524`）。
5. **完整 `mcu_t`** 及所有队列/标志（§2）；`sr_mask=0x870f` 的写掩码、`RTE` 不掩码的例外
   （`src/mcu.h:223`、`src/mcu_opcodes.cpp:225`）。
6. **栈语义**：奇地址 ADDRESS_ERROR 但继续、push/pop 大端、中断/RTE/LDM/STM 顺序（§2.4）。
7. **指令语义按 GT 的整数宽度**：`MCU_ADD/SUB_Common` 的 C/V 算法、`SetStatusCommon` 的
   "V=0、C不变"、MOVG/MOVF/EXTS/ADDX/SUBS/MULXU/DIVXU 的特殊处理（§3.2）。
8. **取指/分派一致**：操作数取指顺序（disp/abs/imm 先于 opcode）、`0x00` 扩展 opcode、
   `opcode>>3` 表索引、`pc` 16 位回卷、trap 后继续且已消耗字节一致（§4）。
9. **宿主调度一致**：中断轮询在指令前、`cycles += 12`、`ex_ignore` 跳过一次、sleep 不取指、
   PCM→TIMER→SM/UART→Analog 的顺序（§5）。
10. **中断判定一致**：TRAPA→异常→NMI→源轮询、`mask < level`、IRQ0/1 的 P1CR 门控、
    向量 4 字节布局、FRT ICI 永不触发、NMI 不消费（§6）。
11. **设备 API 复用**：按 §7 表格调用 GT 设备函数，并共享同一份全局状态（尤其 `dev_register`、
    `ram/sram`、`pcm`、`frt/timer`、`sm`）。
12. **初始化一致**：§8 顺序；`TIMER_Reset` 不调用；`MCU_PatchROM` 在 `MCU_Reset` 之前；
    `PCM_Reset` 在 `MCU_Reset` 之后。
13. **可选但等价性所需**：`-tracepc` 输出格式（`m <cycles> <cp:pc>` / `s <sm_cycles> <pc>`，
    `src/mcu.cpp:244-251`）、`-savesnap/-loadsnap` 的 `state_save/state_load` 字段，
    以及 `-demo/-mocknote` 的注入时刻。

## 10. 易错点（带证据）

1. **读/写 page 0 分支顺序不同**：写先判 `base|0x400` 范围（`src/mcu.cpp:907`）再判 PCM
   （`:931`）；读先判 PCM（`:681`）而 `base|0x400` 只匹配 `==base|0x402`（`:704`）。把两者
   写成同构会出错。
2. **RAME 语义**：`0xfb80-0xff7f` 只有在 `dev_register[0x79]&0x80` 时才落到 `ram`；否则
   读 `0xff`（`:697-703`、`:710-714`），写被忽略（`:960-986`）。`DEV_RAME=0x79`。
3. **比 SRAM 窗口窄**：RAM 窗口在 SRAM 之前判断，但 RAM 只覆盖 0x400 字节，`(address-0xfb80)&0x3ff`。
4. **页 6/7 非扩展时读 0x00 而不是 0xff**（`src/mcu.cpp:845`、`:849`），写则进入 unknown
   （`:1057-1068`）。
5. **写 ROM 无副作用**（`src/mcu.cpp:1036-1039`），JV880 只有 `0x6196-0x6199` nop 例外（`:1032-1035`）。
6. **`MCU_ErrorTrap` 不停止**（`src/mcu.cpp:176-179`）；非法指令同样消耗字节并继续，且
   `MCU_Opcode_Table` 的 `DIVXU` 除零也用 ErrorTrap 而非异常（`src/mcu_opcodes.cpp:1360-1368`）。
7. **MOVG 写也改标志**（`src/mcu_opcodes.cpp:1077-1080`），XCH 不改（`:1061-1074`）。
8. **MOVF 宽度反转**：内存访问宽度按 opcode.bit3，寄存器/标志宽度相反（`src/mcu_opcodes.cpp:713-753`）。
9. **EXTS 标志用扩展前值且不清 C**（`src/mcu_opcodes.cpp:984-989`）；`EXTU` 显式清 C（`:953-961`）。
10. **ADDX 的粘滞 Z**（`src/mcu_opcodes.cpp:1441-1442`）；`SUBS.b` 覆盖整 16 位（`:1195-1203`）。
11. **地址错误不中止访问**：`MCU_Operand_Read/Write` 奇 `ea` 与 `PushStack/PopStack` 奇 `r7`
    都是"触发异常后继续"（`src/mcu_opcodes.cpp:498-504`、`528-535`；`src/mcu.h:340-356`）。
12. **立即数写 = INVALID_INSTRUCTION 异常**（`src/mcu_opcodes.cpp:537-539`）。
13. **`ex_ignore` 只跳过一次中断轮询**，由 RTE/LDC/ORC/ANDC 设置
    （`src/mcu_opcodes.cpp:228`、`:1000`、`:887`、`:911`；`src/mcu.cpp:1343-1346`）。
14. **`RTE` 的 `sr` 不做 `sr_mask`**（`src/mcu_opcodes.cpp:225`）；其它路径大多有掩码。
15. **向量 4 字节布局是 `[x][cp][pc_hi][pc_lo]`**：`cp=(addr>>16)&0xff` 实际取 b1，
    `pc=addr&0xffff`（`src/mcu_interrupt.cpp:58-61` + `src/mcu.cpp:885-894`）。
16. **FRT 的 ICI 源没有分派 case**，永不触发（`src/mcu_interrupt.cpp:198-200`）；IPR 半字节为 0
    的源永不触发（`:202-207`）。
17. **NMI 请求不会被清除**（`src/mcu_interrupt.cpp:111-116`），且现源码没有任何 NMI 请求源。
18. **TRAPA 延迟处理**：指令只置 `trapa_pending`（`src/mcu_opcodes.cpp:185-196`），下一轮
    `MCU_Interrupt_Handle` 最先处理且保留 I 掩码（`src/mcu_interrupt.cpp:84-92`）。
19. **`MCU_Reset` 不复位 cycles/中断队列/sleep/dev_register**；`TIMER_Reset` 未被调用
    （`src/mcu.cpp:1095-1128`、`src/mcu_timer.cpp:41-47` 无调用点）。
20. **PCM 与 TIMER 的调用顺序**：`PCM_Update` 先于 `TIMER_Clock`（`src/mcu.cpp:1434-1436`），
    二者都使用 `mcu.cycles`；SM/analog 在其后（`:1438-1446`）。
21. **SM 时钟是 `while (sm.cycles < cycles*5)` 且每指令 +48**（`src/submcu.cpp:1419`、`:1431`）；
    SM 内部又调用 `SM_UpdateTimer/SM_UpdateUART`（`:1433-1434`）。
22. **`TIMER_Clock` 是 `timer_cycles*2 < cycles`**（`src/mcu_timer.cpp:208`），FRT/TMR 预分频
    基于 `timer_cycles` 的位，不是 `mcu.cycles`（`:218-241`、`:277-301`）。
23. **UART delay 与 SSR 清标志联动**：清 TX/RX 标志时把 delay 推到 `mcu.cycles+3000`
    （`src/mcu.cpp:497-508`），RX/TX 更新各自有使能位要求（`:1138`、`:1158`）。
24. **ADC 延迟与单次/扫描模式**：`analog_end_time` 是静态的、复位不清；单次模式自动清
    `ADCSR.bit5`，扫描模式按 `ctrl&3` 循环采样（`src/mcu.cpp:616-647`）。
25. **音频背压会改 `sample_write_ptr` 的对齐位**（`src/mcu.cpp:1329-1332`），虽不影响 CPU 状态，
    但若要逐位复刻 `state_save` 快照需保留该逻辑。
