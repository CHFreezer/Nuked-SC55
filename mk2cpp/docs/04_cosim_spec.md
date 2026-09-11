# 04 — mk2cpp Co-sim 规范（M1–M4 验证协议）

> **架构修订（2026-09-11，用户指令）**：不再有独立 `mk2cpp.exe`。对照 = **同一
> `nuked-sc55.exe` 的两种模式**（默认解释器 vs `-mk2cpp` 翻译核），同输入/同窗口分别
> 产出 trace/快照/哈希再比对；在线双核方案作废（§2 仅存档背景）。命令示例中的
> `mk2cpp.exe` 一律替换为 `nuked-sc55.exe -mk2cpp`。

状态：M1 设计定稿（可直接执行）；M2+ 设计定稿（待实现）。
上游依据：`00_plan.md`、`01_architecture.md`、`02_conventions.md`、
`../tools/docs/evidence_protocol.md`、`../tools/docs/task_irq_map.md`、
`../tools/baselines/README.md`。
适用范围：`mk2cpp`（DUT）与 GT 仿真器（`../src/` + `../build/nuked-sc55.exe`，oracle）
之间的差分验证。**本规范不改动 `../src/` 任何代码**；一切 GT 侧行为断言必须给
`src/文件:行`，一切 fixture 断言必须给 `tools/...` 路径与量化指标。

术语与证据等级（沿用 `evidence_protocol.md` §1）：

| 记号 | 含义 | 等级 |
|---|---|---|
| GT | `../src/` + `build/rom1.bin`/`rom2.bin`/`rom_sm.bin` 原始字节 | T0（不可怀疑） |
| T1 | GT runtime trace / snapshot / `-pcmtrace` 日志 | 强证据（可能有 logging bug） |
| T2 | VM、dasm、`cosim`/`cover` 等自制工具 | 可疑，必须允许出错 |
| T3 | 旧文档、变量名、语义标签 | 仅假设 |

两条不可协商的硬规则：

1. **0 分歧**，不是"接近"。M1–M3 的合格判据是逐指令、逐状态字节一致
   （`00_plan.md:19-23,37-47`）。
2. **任何分歧必须可一键复现**：分歧包 + 触发命令同时落盘
   （`02_conventions.md:22`）。复现不出来的分歧视为 harness 缺陷，先修 harness。

---

## 1. 总协议

### 1.1 角色

```
GT 参考侧（oracle）                         DUT 侧（mk2cpp）
  build/nuked-sc55.exe                        mk2cpp.exe
    -tracepc  -> gt_*.trace                     -tracepc  -> dut_*.trace
    -savesnap -> demo_snap.bin -> gt_*.qst      -savesnap -snapfile -> dut_*.qst
    -pcmtrace -> pcm_trace.log                  -pcmtrace -> pcm_trace.log
        \                                          /
         +-- cosim compare / cosim state-diff ----+
                        |
              out/cosim/reports/*.md
              out/cosim/divergence/*.txt
```

### 1.2 比较层级（M1 离线 / M2+ 在线共用编号）

| 层 | 内容 | M1 粒度 | M2+ 粒度 |
|---|---|---|---|
| L0 | 统一 trace 行序列：`m <cycles> <cp:pc>` / `s <sm_cycles> <pc>`，含交错顺序，逐字节 | 每条指令 | 每条指令 |
| L1 | `cp:pc`（含在 L0 内） | 每条指令 | 每条指令 |
| L2 | `sr` + `r0..r7` + `br/dp/ep/tp` | 检查点（state dump） | 每条指令 |
| L3 | SRAM/RAM/dev/FRT/timer/UART/SM/LCD/PCM 状态哈希 | 每 N cycles（默认 1M，长跑 10M） | 每 N instructions（默认 1024） |
| L4 | PCM 输出流（`MCU_PostSample` int16 L/R）与 `-pcmtrace` 控制写序列 | `-pcmtrace`（M1–M3）/ 音频流（M4） | 同左 |

对 L0 的语义必须逐项照抄 GT，**禁止"归一化"**：

- `m` 行在 `mcu.cycles += 12` **之后**、下一指令 PC 上打印
  （`src/mcu.cpp:1351,1363`）；`m 12 00:016f` 即 reset 后第一条指令执行完。
- `s` 行在 SM 指令**执行前**、以指令起点 `sm.cycles` 打印；SM 每指令
  `sm.cycles += 12*4`（`src/submcu.cpp:1425-1431`）。
- 交错顺序：主循环先写 `m`（`src/mcu.cpp:1353-1372`），随后
  `PCM_Update`→`TIMER_Clock`→`SM_Update`→`MCU_UpdateAnalog`
  （`src/mcu.cpp:1434-1446`），`s` 行在该轮 `SM_Update` 内产生。
- 窗口是半开区间且判据在 `mcu.cycles`（后增量）上：`>= start && < end`
  （`src/mcu.cpp:1356,1365`）；trace 文件在窗口内首行惰性打开、`>= end`
  时 `fflush`+`fclose` 后**进程继续运行**（`src/mcu.cpp:1358-1371`）。

### 1.3 确定性前置条件（M1/M2 通用）

GT 的可观测状态不存在随机源；唯一外部扰动是 MIDI 输入（默认自动打开端口，
`src/mcu.cpp:2338-2342`）与 wall-clock（只影响墙钟，不影响模拟状态）。因此：

1. cosim 运行**禁止外部 MIDI 流量**；`-p` 不指定可用端口即不接设备。
   `-demo`/`-mocknote` 是确定性输入，允许。
2. GT 侧固定 `SDL_VIDEODRIVER=dummy`、`SDL_AUDIODRIVER=dummy`
   （headless；本机实测可控）。LCD 线程只读状态、不写模拟状态
   （`src/lcd.cpp:82-85`, `mcu.cpp:1531-1538`），不影响收敛。
3. DUT 侧 cosim 模式必须：单线程、无 wall-clock 调用、无 `rand`、音频不外放
   （仅落盘/哈希）；`PCM_Update` 必须由模拟 cycles 驱动，调度顺序与
   `work_thread` 完全一致（`src/mcu.cpp:1343-1446`）。
4. 每次运行前校验 `meta`：ROM sha256、GT exe hash、`cosim_state_v1` ABI、
   启动快照 hash。任一不符 → 拒绝比较，而不是"凑合比"。
5. **快照即起点**：所有分段运行只允许从 `gt_*.qst` 恢复，恢复语义必须逐字段
   复刻 GT `state_load`（含其"丢字段"行为，见 §8 B1），不得私自补全。

---

## 2. M1 离线协议

### 2.1 GT 参考生成（命令与运维细节）

GT 是 GUI/音频程序，**不会自己退出**：`MCU_Run` 只在
`LCD_QuitRequested()` 时结束（`src/mcu.cpp:1524-1542`）；headless 下没有窗口可关。
因此参考运行必须由 runner 轮询产物完成后**强制终止**。

固定前置（PowerShell，路径按仓库根展开）：

```powershell
$env:SDL_VIDEODRIVER='dummy'
$env:SDL_AUDIODRIVER='dummy'
$env:SDL_RENDER_DRIVER='software'
$REPO = '<repo root>'; $OUT = "$REPO\mk2cpp\out\cosim"
```

参考 trace（R0：boot 0–3M，cwd=build，输出绝对路径）：

```powershell
$p = Start-Process -FilePath "$REPO\build\nuked-sc55.exe" `
     -WorkingDirectory "$REPO\build" -PassThru `
     -ArgumentList '-mk2', '-tracepc', "$OUT\ref\gt_boot_0_3M.trace", '0', '3000000' `
     -RedirectStandardOutput "$OUT\logs\gt_boot_0_3M.stdout.txt" `
     -RedirectStandardError  "$OUT\logs\gt_boot_0_3M.stderr.txt"
# 轮询：文件大小连续 2 s 不变，且最后一行满足 0 <= cyc < 3000000
Stop-Process -Id $p.Id -Force
```

参考 200M 窗口（R1：与 `trace_200m_base.txt` 同源：reset + `-demo` 全跑）：

```powershell
Start-Process -FilePath "$REPO\build\nuked-sc55.exe" -WorkingDirectory "$REPO\build" `
  -ArgumentList '-mk2','-demo','-tracepc',"$OUT\ref\gt_demo_200M_2M.trace",'200000000','202000000'
```

参考快照（R2：200M，供窗口回放；GT 的 `-savesnap` **固定写 cwd 的
`demo_snap.bin`**，`src/mcu.cpp:1420`）：

```powershell
# cwd 用可写 scratch；ROM BasePath = argv[0] 目录（src/mcu.cpp:2080-2087），
# 因此从 scratch 启动仍能读到 build/ 下的 ROM。
$wd = "$OUT\scratch"; New-Item -ItemType Directory -Force -Path $wd | Out-Null
$p = Start-Process -FilePath "$REPO\build\nuked-sc55.exe" -WorkingDirectory $wd -PassThru `
     -ArgumentList '-mk2','-demo','-savesnap','200000000' `
     -RedirectStandardOutput "$OUT\logs\gt_c200M.stdout.txt"
# 轮询 demo_snap.bin 出现，并核对 stdout 行：snap: dumped full state at c<cycle>
# （该打印来自 src/mcu.cpp:1424）
Move-Item "$wd\demo_snap.bin" "$OUT\ref\gt_c200M.qst" -Force
Stop-Process -Id $p.Id -Force
```

恢复运行（R3：窗口 trace，必须使用与 DUT 完全相同的参数）：

```powershell
Start-Process -FilePath "$REPO\build\nuked-sc55.exe" -WorkingDirectory "$REPO\build" `
  -ArgumentList '-mk2','-loadsnap',"$OUT\ref\gt_c200M.qst",`
                '-tracepc',"$OUT\ref\gt_c200M_2M.trace",'200000004','202000004'
```

关键运维事实（全部有 GT 源码依据）：

| 事实 | 依据 |
|---|---|
| `-savesnap` 只写 `demo_snap.bin` 到 cwd；无文件名参数 | `src/mcu.cpp:1910-1913,1420` |
| 快照点在指令与 trace 之后、`PCM_Update`/`TIMER_Clock`/`SM_Update`/`MCU_UpdateAnalog` 之前 | `src/mcu.cpp:1349-1351,1417-1429,1434-1446` |
| 快照 cycles = 首个 `>= snap_dump_at` 的后增量值；恢复后第一条指令从 `snap_cycle+12` 起 | 同上 |
| `-loadsnap` 覆盖开始状态，但 `-demo`/`-mocknote` 的 static 状态不会被快照恢复；`-mocknote` 判据是 `cycles >= 144M`，快照运行若带 `-mocknote` 会立刻触发 | `src/mcu.cpp:1314-1325,1374-1400,1407-1415` |
| `-tracepc` 只解析数字窗口参数，默认 `200M..210M` | `src/mcu.cpp:1897-1905,229-231` |
| `-pcmtrace` 写 cwd 的 `pcm_trace.log`，格式 `pcm reg=%02x val=%02x pc=%02x:%04x cyc=%llu`，只记 regs `0..3/0x3c/0x3d/0x3e` | `src/mcu.cpp:933-945` |

**快照恢复与 from-reset 的 trace 关系（实测，2026-09-11）**：快照落点在设备更新
之前，恢复后首个 `SM_Update` 会一次性补跑"快照那一轮的 SM 批次"，因此
`from-snapshot` trace 与 `from-reset` trace **不是简单后缀关系**：后者的某个
`s` 批次会在前者中晚一拍出现。实测 C=48,000,000 时，两者从第一个满足
`sm_cycles >= C*5` 的 `s` 行起才逐字节相同（C*5=240,000,000；C=48M 快照的
首个相同行是 `s 240000000`）。协议结论：

1. M1 常规门禁（S1）比较的是 **GT-from-snapshot vs DUT-from-snapshot**
   （同一快照、同一窗口），必须 L0 精确一致；
2. 与 from-reset 冻结基线（S1R）对照时，允许并仅允许上述前缀差异，比较从
   `sm_cycles >= snap_cycle*5` 的首行开始；
3. DUT 的快照加载/恢复必须复刻"设备更新前落盘"的相位，否则会在这里产生
   系统性假分歧。

### 2.2 DUT 镜像 CLI（契约）

DUT 必须接受与 GT 同名的旗标，使两侧命令向量尽量逐字相同；仅快照文件名与
哈希工具为扩展：

```text
mk2cpp.exe -mk2 [-headless] [-deterministic]
           [-loadsnap <file>] [-savesnap <cycle> -snapfile <file>]
           [-tracepc <file> [start end]]
           [-hash-every <cycles> -hashfile <file>]
           [-demo [cycles]] [-mocknote [cycles]] [-pcmtrace]
           [-voices:<n>] [-float]
```

- `-tracepc` / `-loadsnap` / `-demo` / `-mocknote` / `-voices:` / `-float` / `-pcmtrace`
  与 GT 同义同格式。
- `-snapfile` 仅为替代 GT 的固定名；未给时也必须落 `demo_snap.bin`。
- `-hash-every N -hashfile f`：每 N cycles 写一行
  `H <cycles> <cp:pc> <sr> <fnv64(mcu)> <fnv64(sram)> <fnv64(dev)> <fnv64(sm)> <fnv64(pcm)> <fnv64(lcd)>`
  （分块定义见 §2.5/2.6）。
- `-headless -deterministic` 必须默认开启（交互/音频模式不得进入 cosim 路径）。

### 2.3 DUT trace（与 R0/R1/R3 对称）

```powershell
# boot
& "$REPO\mk2cpp\build\mk2cpp.exe" -mk2 -headless -deterministic `
  -tracepc "$OUT\dut\dut_boot_0_3M.trace" 0 3000000
# 200M 快照窗口（与 R3 参数逐字相同）
& "$REPO\mk2cpp\build\mk2cpp.exe" -mk2 -headless -deterministic `
  -loadsnap "$OUT\ref\gt_c200M.qst" -tracepc "$OUT\dut\dut_c200M_2M.trace" 200000004 202000004 `
  -savesnap 202000000 -snapfile "$OUT\dut\dut_c202M.qst" -hash-every 1000000 -hashfile "$OUT\dut\dut_c200M_2M.hash.tsv"
```

### 2.4 离线比较工具

```text
cosim compare --ref <gt.trace> --dut <dut.trace> [--history 64]
              [--report <md>] [--divergence <txt>] [--ignore-sm]
cosim qst --gt-snap <demo_snap.bin> --out <state.qst> --label <name>
cosim state-diff --ref <state.qst> --dut <state.qst> [--chunk <ID,...>] [--report <md>]
cosim hash --state <state.qst> --out <hash.tsv>
```

`cosim compare` 必须做到：流式读两个文件、逐行比较、内存 O(history)、首分歧
立即落盘并给出双方最近 `history`（默认 64）步；`--ignore-sm` 仅允许在
M1 早期 SM 尚未接入时使用，必须在报告中置 exemption 标记。

### 2.5 `cosim_state_v1` 状态转储格式

用途：把 GT 原始快照（无头、无版本、含 padding）转换成**平台无关、字段级、
可 diff** 的规范格式；同时作为 DUT state dump 的落盘格式。

```text
offset  type        field
0       char[4]     magic = "MK2Q"
4       u32         version = 1
8       u32         header_len
12      char[16]    romset（"SC-55mk2"）
28      u64         cycles
36      u32         chunk_count
40      u8[24]      fnv1a64(rom1)||fnv1a64(rom2)||fnv1a64(rom_sm)
64      u32         sizeof_pcm_t；u32 PCM_SLOTS；u32 state_raw_size（ABI guard）
...
chunk 序列（TLV，全部小端、字段级打包、padding 不落盘）：
  u32 id ('MCU0','RAM0','SRM0','DEV0','FRT0','TMR0','UAR0',
          'SMI0','SMR0','SMS0','SMA0','SMD0','PCM0','LCD0')
  u32 len；u8[len] payload；u64 fnv1a64(payload)
尾部：u64 fnv1a64(全部 payload)；u32 终止标记 'END0'
```

chunk 与 GT 字段的对应（GT `state_save`，`src/mcu.cpp:1225-1263`；LCD 在
`src/lcd.cpp:44-80`）：

| chunk | 内容（GT 字段） | GT 行 |
|---|---|---|
| `MCU0` | `mcu_t`（cycles/regs/sr/cp..br/sleep/ex_ignore/exception_pending/interrupt_pending/trapa_pending） | 1227 |
| `RAM0` | `ram[0x400]` | 1228 |
| `SRM0` | `sram[0x8000]` | 1229 |
| `DEV0` | `dev_register[0x80]` | 1230 |
| `FRT0` | `frt[3]`（含 `status_rd`） | 1231 |
| `TMR0` | `timer` + `timer_cycles` + `timer_tempreg` | 1232-1234 |
| `UAR0` | `uart_buffer[8192]` + rw ptr + main `uart_rx_byte` | 1246-1249 |
| `SMI0` | `sm`（`submcu_t`） | 1250 |
| `SMR0`/`SMS0`/`SMA0`/`SMD0` | `sm_ram`、`sm_shared_ram`、`sm_access`、`sm_device_mode` | 1251-1256 |
| `PCM0` | `pcm_t`（ram1/ram2/mask/latch/config/eram/accum/rcsum/fstate） | 1261 |
| `LCD0` | LCD 控制器寄存器 + `LCD_Data[80]` + `LCD_CG[64]` + `lcd_enable` | `lcd.cpp:44-80` |

ABI guard：header 记录 `sizeof(pcm_t)`、`PCM_SLOTS`、原始快照字节数。`pcm_t`
一旦变化旧快照即失效（历史记录：`tools/docs/polyphony_256_todo.md:182`
"快照不兼容记录（`pcm_t` 尺寸变，旧 `demo_postW.bin` 弃用）"）。实测当前
MK2 快照 94,547 B（`pcm_t`/eram 占大头）。

### 2.6 哈希定义

统一 `fnv1a64`（本地 C 实现，无外部依赖；`evidence_protocol.md` §14 要求 C 工具）：

```
hash = 14695981039346656037      # FNV-1a 64 offset basis (0xcbf29ce484222325)
for b in payload: hash ^= b; hash *= 1099511628211   # 0x100000001b3
```

| 哈希 | 覆盖 | 备注 |
|---|---|---|
| `hash_mcu` | `MCU0` 字段级序列 | 含 `cycles`/`sr`/regs |
| `hash_sram` | `SRM0`（0x8000 字节） | M1 主判据之一 |
| `hash_dev` | `DEV0` + `FRT0` + `TMR0` | 设备寄存器 + 定时器状态 |
| `hash_sm` | `SMI0..SMD0` | SM CPU + 三块 SM 内存 + 设备 |
| `hash_pcm` | `PCM0` | 控制/参数/eram/filter 状态（不含音频流） |
| `hash_lcd` | `LCD0` | 帧序列等价性（M3） |
| `hash_audio` | `MCU_PostSample` 输出的 int16 L/R 流（`src/mcu.cpp:1727-1747`） | 仅 M4；n=28 要求 0 误差 |

周期哈希的参考值来源：DUT 写 `.hash.tsv`；GT 侧由 runner 在相同 cycles 上
多次运行 `-savesnap <cycle>` 产生（每次一个检查点），再经
`cosim qst` + `cosim hash` 生成同名 TSV。M1 默认 N=1,000,000 cycles；
≥300M 长跑默认 N=10,000,000（从 200M 快照往后做，控制运行次数）。

### 2.7 首分歧报告（divergence bundle）

`out/cosim/divergence/<scenario>_c<cycle>.txt`，格式固定：

```text
MK2COSS-DIV v1
scenario: boot_0_3M
ref: out/cosim/ref/gt_boot_0_3M.trace  sha256=<...>  lines=<n>
dut: out/cosim/dut/dut_boot_0_3M.trace  sha256=<...>  lines=<n>
first_diff: line=<index>  kind={missing_ref_line|missing_dut_line|pc|cprefix|cycles|sm_pc|interleave}
cycle_ref=<...>  cycle_dut=<...>
ref_line: <原文>
dut_line: <原文>
rom_at_ref_pc: <rom1|rom2>[<fileoff>] = <hex bytes 8>
rom_at_dut_pc: <rom1|rom2>[<fileoff>] = <hex bytes 8>
history[64] (oldest first):
  <i>  ref: m <cyc> <cp:pc> [s <cyc> <pc> ...]
       dut: m <cyc> <cp:pc> [...]
chunk_diff:          # 仅当分歧由 L3/state-diff 触发
  SRM0 @0x1234 ref=.. dut=..
repro:
  <完整命令行 1>
  <完整命令行 2>
```

`kind` 分类必须自动判定；`cycle_ref/dut` 不同即为调度/时序分歧（优先怀疑
IRQ/timer/SM 相位差），相同而 PC 不同即为译码/语义分歧。

### 2.8 文件与命名（`mk2cpp/out/cosim/`）

```text
out/cosim/
  meta/            <label>.meta.json    # ROM sha256、GT/DUT hash、ABI、命令、起始快照
  ref/             gt_*.trace gt_*.qst gt_*.state gt_*.hash.tsv
  dut/             dut_*.trace dut_*.state dut_*.hash.tsv
  logs/            gt_*.stdout.txt gt_*.stderr.txt dut_*.log pcm_trace_*.log
  reports/         <scenario>.md
  divergence/      <scenario>_c<cycle>.txt（+ 可选 .state.ref/.state.dut）
  scratch/         GT 的 cwd（`-savesnap`/`-pcmtrace` 固定文件名落在此处），每轮清理
```

所有产物本地、不入 git（`02_conventions.md:24-31`、`evidence_protocol.md:8-13`）。

---

## 3. M2+ 在线协议

### 3.1 目标

同一进程内"单步 GT 解释器 vs 单步翻译块"，逐指令比较
`cycles/cp/pc/sr/r0..r7/br/dp/ep/tp`，每 N 条比较全状态哈希，首个分歧立即
abort 并输出 §2.7 等价的历史包。支持 `--stop-at`、`--ignore-cycles`。

### 3.2 GT 全局状态单例分析（能否开两个实例？）

GT 的全部机器状态与调度都挂在文件作用域全局量上：

| 类别 | 例子（证据） |
|---|---|
| 已结构体化的机器态 | `mcu`（`src/mcu.h:174`）、`pcm`（`src/pcm.h:63`）、`sm`（`src/submcu.h:42`）、`frt[3]`/`timer`（`src/mcu_timer.cpp:22-26`） |
| 裸数组/散量 | `dev_register`（`src/mcu.h:88`）、`ram`/`sram`（`src/mcu.cpp:653-654`）、`b_ram`/`b_ram2`（657-658）、`uart_buffer`（`src/mcu.h:445-448`）、`ga_int/ga_int_enable/ga_int_trigger/ga_lcd_counter`（193-196）、`ad_val/ad_nibble/sw_pos/io_sd`（201-204）、LCD 寄存器与显存（`src/lcd.cpp:30-37`） |
| 函数级 static / 私有态 | `demo_step`（`src/mcu.cpp:1392`）、`snap_done/mocknote_done`（222,1223）、main UART `uart_rx_delay/uart_tx_delay`（402-403）、SM UART `uart_rx_gotbyte/uart_rx_byte/uart_rx_delay`（`src/submcu.cpp:82-84`）、`adf_rd`（`src/mcu.cpp:391`，全局但不在 `state_save`） |
| 调度 | 唯一 `work_thread`（`src/mcu.cpp:1305-1522`）：中断→指令→`cycles+=12`→设备更新；音频空缓冲时才让出（`1333-1341`），退出由 `MCU_Run` 的 LCD 循环控制（`1524-1542`） |

结论：

- **同一进程同时存在两个 GT 实例不可行**（当前源码形态）。要做需要把上述
  全局量全部收进 `machine_t` 并让所有函数经 ctx 访问，或把 TU 复制两份；
  后者会撞 `main`、SDL 的 `extern "C"` 符号和内联函数对全局 `mcu` 的无参
  引用（`src/mcu.h:184-196`），宏观改名（mcu/pcm/sm/frt/...）虽可行但会把
  验证工具本身变成高风险 T2。
- 因此 M2 采用**单实例 + 串行双跑**（设计 A）；真正的同时双核留到 M4 之后
  若确有必要再评估。win32 无 `fork()`，且 GT 带 SDL 线程，即使有也不安全。

### 3.3 设计 A（推荐）：checkpoint 串行双跑 lockstep

设备模型**单实例**，两个 CPU pass 交替执行、互不重叠；状态在 pass 间深拷贝。

```text
for each quantum q of Q instructions:
    S = snapshot_all()                 # 完整内部快照（非 GT 快照，见下）
    run_GT(q, ring[64])                # oracle pass：GT 解释器 + GT 设备调度
    restore_all(S)
    run_DUT(q, compare_each, ring[64]) # 翻译块 + 同一设备调度
    if mismatch or hash_diff:
        restore_all(S); dump divergence; abort
    # 成功则保留 DUT 结束状态进入下一 quantum
```

- 每步比较元组：`(cycles, cp, pc, sr, r0..r7, br, dp, ep, tp)`；比较放在
  指令执行后、设备更新前（与 GT `m` 行同相位）。
- 调度包装必须逐行复刻 `work_thread` 顺序：
  `MCU_Interrupt_Handle` → `MCU_ReadInstruction` → `cycles+=12` →（可选 trace）
  → `PCM_Update` → `TIMER_Clock` → `SM_Update` → `MCU_UpdateAnalog`
  （`src/mcu.cpp:1343-1446`）。
- Q 默认 4096 条指令（≈49k cycles），N（全量哈希）默认 1024 条指令。
- `snapshot_all/restore_all` 用**内部完整版**，比 GT 的 `state_save` 多收：
  `adf_rd`（`src/mcu.cpp:391`）、`uart_rx_delay/uart_tx_delay`（402-403）、
  SM `uart_rx_gotbyte/uart_rx_delay`（`src/submcu.cpp:82-84`）、
  `b_ram/b_ram2`（657-658）、`mcu_button_pressed`（`src/mcu.h:443`）、
  `demo_step`/`mocknote_done`/`snap_done`、trace 文件开关状态。
  否则 pass 会出现"看似玄学"的第 N 步分歧（典型：UART 半字节在途）。
- 成本：状态 ~95KB memcpy/quantum（微秒级）+ 双份 CPU+设备执行。
  成功路径用 DUT pass 的末态，无需再拷贝。
- 可行性依据：`MCU_ReadInstruction`（`src/mcu.cpp:1078-1088`）是纯单步函数，
  可在不外放音频、不进入 `work_thread` 的情况下调用；设备函数
  `PCM_Update/TIMER_Clock/SM_Update/MCU_UpdateAnalog` 均可直接调用
  （`src/pcm.cpp:584`、`src/mcu_timer.cpp:205`、`src/submcu.cpp:1417`、
  `src/mcu.cpp:616`）。

### 3.4 设计 B（备选）：双实例（当前不可行）

若将来允许重构 GT：引入 `struct machine_t { mcu_t mcu; pcm_t pcm; submcu_t sm;
uint8_t sram[0x8000]; ... }`，所有函数加 `machine_t&` 参数；SDL 与文件 IO 全部
移出核心。届时双实例自然成立，可做真双核。**在 `src/` 冻结期间不采用。**

### 3.5 设计 C（兜底）：进程级 replay + checkpoint 二分

如果同进程串行双跑因链接/线程原因也不可用（例如 GT TU 冲突无法消除）：

1. 仍用 M1 管线，但从**更近的快照**开始、只跑小窗口（`-loadsnap` +
   `-tracepc`），两侧各跑一遍比较。
2. 窗口内出现分歧 → 二分：把窗口切两半，用 GT `-savesnap` 生成中点快照，
   直到分歧定位到单条指令。
3. 该流程无新依赖、完全由 GT 现有 CLI（`-savesnap/-loadsnap/-tracepc`）支撑，
   缺点是每步都要起进程 + 重放，适合定位不适合长跑。

### 3.6 性能预算（基准机：本仓库开发机；dummy SDL 驱动）

实测（本机，2026-09-11）：

| 项目 | 实测 | 预算（验收上限） |
|---|---|---|
| GT 无 trace（`-savesnap 48M`） | 48M cycles / 2.5 s（含启动） | — |
| GT 全量 trace（0–24M） | 24M cycles / 4.5 s，4.31M 行 / 78.8 MB | trace 吞吐 ≥ 15 MB/s |
| GT 单条 `m/s` 行密度 | ≈0.18 行/cycle | 以冻结 fixture 行数为准 |
| GT 快照文件 | 94,547 B | 变动即触发 ABI 复核 |
| 2M-cycle 窗口 trace | ≈7.5 MB / 窗口 | 单窗口产物 ≤ 20 MB |
| M2 在线双跑 | 未实现 | ≤ 3× 单跑 GT；2M cycles ≤ 30 s；300M 长跑 ≤ 20 min |
| 首分歧 abort 延迟 | 未实现 | ≤ 1 条指令（L2）/ ≤ 1 个 quantum（L3） |

长跑（300M cycles）策略：不落全量 trace（估算 ~56M 行 / ~1 GB），只在尾部
2M cycles 落 trace + 每 10M cycles 检查点哈希（GT 侧按检查点多次
`-savesnap`，从 200M 快照往后跑控制单次时长）。

---

## 4. 测试语料（Corpus）

所有场景默认 `ROM_SET_MK2`、`-float` 关闭、`-voices:28`（stock）。路径
`$OUT = mk2cpp\out\cosim`；fixture 引用 `tools/baselines/`。

### 4.1 场景表

| ID | 场景 | 起始 | GT 命令要点 | DUT 命令要点 | 量化预期 / oracle |
|---|---|---|---|---|---|
| S0 | boot 0–3M | reset | `-mk2 -tracepc <f> 0 3000000` | 同左 | `tools/baselines/trace_boot3m_base.txt`：375,116 行（m 249,999 / s 125,117）、6,759,570 B；`00:03db` 出现 115 次；`00:0492` 159,999 次；首条 `m 12 00:016f`；`04:662b` 首现 c600 |
| S0b | LCD enable | reset | `-mk2 -savesnap 4222836` | `-savesnap 4222836 -snapfile ...` | `LCD0.lcd_enable` 0→1；历史 T1：`LCDEN 0 pc=04:662b cyc≈588`、`LCDEN 1 pc=04:668d cyc≈4.22M`（`task_irq_map.md:495`，当前 GT 无该打印，改用快照字段判定） |
| S1 | 200M 窗口（快照） | `gt_c200M.qst` | `-loadsnap ... -tracepc <f> 200000004 202000004` | 同参数 + `-hash-every 1000000` | L0 0 分歧（≈375k 行）；`00:03db` 50 次（首现 200,844,960，末现 200,957,580）；`00:0492` 120,197 次；终点 `SRM0/DEV0/PCM0/SM*` 哈希相等 |
| S1R | 200M 窗口（from reset，release 验收） | reset | `-mk2 -demo -tracepc <f> 200000000 202000000` | 同左（跑到 202M） | 对照冻结的 `tools/baselines/trace_200m_base.txt`（375,001 行、7,458,352 B）；S1 与该基线从首个 `sm_cycles >= snap_cycle*5` 的行起逐字节一致（见 §2.1 快照恢复规则） |
| S2 | demo 全曲长跑 ≥300M | reset | `-mk2 -demo -tracepc <f> 298000000 300000000 -savesnap 300000000` | 同左 | 尾部 2M 0 分歧；c300M 快照状态相等；历史 O8：215.37M 电源循环后不复位（`task_irq_map.md:502`） |
| S3 | mocknote/SM UART | reset | `-mk2 -mocknote 144000000 -pcmtrace -savesnap 150000000` | 同左 | `pcm_trace.log` 在 c≈144.05M 出现 `pc=00:5527`（disable flush）与 `pc=00:5664`（enable flush，非零）写（`task_irq_map.md:499`）；150M 快照 `SM*` 哈希相等 |
| S4 | IRQ/PCM 密集 | `gt_c200M.qst` | S1 即可覆盖（demo 音符 + PCM IRQ） | 同 S1 | `00:03db` 计数稳定在 50/2M |
| S5 | 异常/边界 | 视可达性 | 例外向量（除零/非法指令）在固件执行集中不可达则标记 `unreachable` | — | 不作为 M1 gate；登记于 `99_blockers.md` |

### 4.2 边界压力（M2+ / M4）

| ID | 目的 | 命令 | 预期 |
|---|---|---|---|
| E1 | IRQ 相位抖动 | S1 窗口 + online lockstep | 每步 cycles/sr 相等；无 L3 相位漂移 |
| E2 | UART 半字节在途 | S3 + 快照任意时刻 | 内部完整快照恢复后逐字节一致（验证 §3.3 的隐藏态补齐） |
| E3 | page6/7 | `-voices:32`（M4） | 快照补 `PAGE67` chunk；**当前 GT 快照丢 page6/7**（`src/mcu.cpp:657-658` 不在 `state_save`），M4 前必须解决或禁用该场景 |
| E4 | 长睡眠/定时器翻转 | S2 | 无复位，心跳计数单调增长 |
| E5 | `-voices:48` 历史 stall | M4 回归 | 失败签名 `pc=00:037A`、`iml=7`（`task_irq_map.md:504`） |

### 4.3 Fixture 再生

| fixture | 状态 | 再生命令 |
|---|---|---|
| `trace_boot3m_base.txt` | 在库 | `build\nuked-sc55.exe -mk2 -tracepc tools\baselines\trace_boot3m_base.txt 0 3000000`（reset） |
| `trace_200m_base.txt` | 在库 | `build\nuked-sc55.exe -mk2 -demo -tracepc tools\baselines\trace_200m_base.txt 200000000 202000000` |
| `demo_postW.bin` | **已清理，需再生** | `cd <scratch>; build\nuked-sc55.exe -mk2 -demo -savesnap 198000000`（W 键在 192M 按下、198M 释放：`src/mcu.cpp:1382-1391`），产物改名 `gt_demo_postW.qst`；或直接改用 §4.1 的 `gt_c200M.qst` |
| `probe_snap_page0/6.bin` | 在库，但尺寸与当前 ABI 不符（78,075 B vs 94,547 B），不可直接用于当前 GT | 如需使用，用 VM `h8vm.exe ... savesnap <file> <cycle>` 重生，并先过 `cosim qst` ABI guard |
| `gt_c200M.qst` | 不存在，M1 必须生成 | §2.1 R2 |

---

## 5. 验收标准

### 5.1 M1（离线）

- A1.1 fixture 资格：同一 GT 命令连跑两次，trace 字节级相同（sha256 相等）；
  快照 raw 字节相同。不满足则先修环境（§1.3）。
- A1.2 S0：`cosim compare` 0 分歧（375,116 行）；检查点 c=1M/2M/3M 的
  `MCU0/SRM0/DEV0/PCM0/SM*` 全等；`00:03db` 计数 = 115。
- A1.3 S1：快照恢复窗口 0 分歧（≥375,000 行）；终点全 chunk 全等；
  `00:03db` 计数 = 50。
- A1.4 工具自检：对 DUT trace 注入单字节/单行扰动，`cosim compare` 100% 报出
  正确 `line/kind`，并输出 64 步历史。
- A1.5 确定性：同一 DUT 输入连跑两次 trace 相同；不触网、不读时钟。
- A1.6 覆盖：执行集不超出 `tools/baselines/pc_main.txt`；新增 PC 必须显式登记
  （覆盖仪表盘见 §7）。
- 性能：S0 全流程（GT+DUT+diff）≤ 60 s；S1 ≤ 120 s；单窗口磁盘 ≤ 20 MB。

### 5.2 M2（在线 + 主固件全执行面）

- A2.1 执行集 100% 翻译：语料运行中 `dispatch->fallback` 调用 = 0
  （`map.csv` 覆盖 `pc_main.txt` 全部 9,217 PC）。
- A2.2 S0–S4 全 0 分歧；S2 ≥300M 无复位；每 10M cycles 的 L3 哈希全等。
- A2.3 在线 lockstep：在合成注入的第 k 步故障上，abort 定位误差 ≤ 1 条指令；
  实现通过的 overhead ≤ 3×；2M cycles 窗口 ≤ 30 s。
- A2.4 混合模式回归：未翻译回落路径存在时，`fallback_count > 0` 的运行仍
  0 分歧（保证兜底路径本身不引入差异）。

### 5.3 M3（子 MCU / LCD）

- A3.1 demo 语料 `s` 行逐条一致（≥2M 条 SM 指令），SM PC 集覆盖
  `tools/baselines/pc_sm_251.txt`。
- A3.2 每个检查点 `LCD0` 全等；`lcd_enable` 0→1 恰在 c≈4.22M 发生一次。
- A3.3 SM 5× 相位：所有 `s <sm_cycles>` 的 cycles 序列一致（用 L0 保证）。

### 5.4 M4（语义化 / 256）

- A4.1 n=28 null 测试：≥2M cycles 的 `hash_audio` 完全相等（容差 0；若引入
  量化误差，必须写明量化步长与理论界）。
- A4.2 n=32/64/128/256：无复位、心跳持续、PCM 活跃、mask 写覆盖全部 slot；
  与 `tools/docs/task_irq_map.md` §4 O1–O10 矩阵逐一对应（256 目标）。
- A4.3 E3 通过：`PAGE67` 进入快照或场景明确豁免。

---

## 6. 分歧 Triage 程序

按 `evidence_protocol.md` 的分级执行，**从最便宜、最可能是自己错的检查开始**。

### Tier 0 — 复现与排除 harness 噪声（先做）

1. 用分歧包里的两条命令原样重跑；先确认 `meta` 的 ROM sha256 / GT exe hash /
   ABI 一致。
2. 检查环境与旗标：trace 窗口语义（后增量、半开）、`-loadsnap` 起点
   `snap_cycle+12`、误带 `-demo/-mocknote`、cwd 导致 `demo_snap.bin` /
   `pcm_trace.log` 落错、外部 MIDI。
3. GT 自洽：同一命令连跑两次 trace 是否字节相同。不同 → 环境问题，禁止
   继续把差异算到 DUT 头上。
4. 确认 DUT 的调度顺序逐行等于 `src/mcu.cpp:1343-1446`。

### Tier 1 — 定位最早分歧点

5. 取 `first_diff` 的 `line/kind/cycle`。先看**紧邻上一条**两侧状态：
   若上一条两者最后写入的 PC 相同、cycle 相同，则分歧来自该条指令本身；
   若 cycle 已不同，先查 IRQ/timer/SM 相位。
6. 用 `rom_at_*` 取双方该 PC 的 ROM 字节，核对指令边界：H8 指令长度/
   operand 宽度/direction/寻址模式（`evidence_protocol.md` §4、§13）。
7. 对照 GT decoder：`src/mcu_opcodes.cpp` 对应 opcode 的 path，逐位展开
   `ocode/ore/bit/EA`（证据协议 §3 禁止心算掩码/重排）。

### Tier 2 — 状态分块定位（分歧不表现为 PC 差异，或 PC 相同）

8. 用最近的两个检查点做**二分**：比较 `MCU0/SRM0/DEV0/FRT0/TMR0/UAR0/SM*/
   PCM0/LCD0` 哪个 chunk 先开始不同；不同 chunk 决定调查方向：
   - `MCU0`：flags/寄存器/异常 pending；
   - `SRM0`：地址重排、页映射、写宽/方向（`src/mcu.cpp:896-1069`）；
   - `DEV0/FRT0/TMR0`：定时器写/读 clear 语义（`src/mcu_timer.cpp:49-203`）；
   - `SM*`：UART 半字节在途、`sm_device_mode` 握手、5× 相位；
   - `PCM0`：mask/latch/eram/`PCM_Update` 精度；
   - `LCD0`：控制器命令/显存。
9. 若 L3 哈希不同但 L0 全同：优先怀疑**未进快照的隐藏态**（`adf_rd`、
   `uart_rx_delay/uart_tx_delay`、`uart_rx_gotbyte/uart_rx_delay`、
   `b_ram/b_ram2`、`mcu_button_pressed`、`demo_step`）在双跑间未复刻
   （M1 快照场景则是 GT 自身丢字段，见 §8 B1）。

### Tier 3 — 根因归因

10. 机械验证低层假设：direction/operand width/register field/addressing/
    length/sign-ext/bank-page 换算（`evidence_protocol.md` §2 的 H1–H6、
    §13 的历史 bug 清单）。**先查自己，再查工具，最后才允许怀疑 trace 归因**。
11. 需要断言 GT/dasm/trace 有错时，先满足 `evidence_protocol.md` §8 必备字段
    （PC/ROM 字节/GT path/正确指令/差异/bug 类/疑似代码位置），否则结论无效。
12. 任一早期结论（如 trace cycle 相位、快照字段集）被推翻，执行 §10 rollback
    审计：列出所有下游结论并逐个重验。

### Tier 4 — 处置与回归

13. 修复 DUT → 重跑该场景 + 相邻场景 → 通过后把最小复现加入 Corpus 表。
14. 确认 harness 缺陷（工具/脚本）→ 修工具并用已知注入用例自检（A1.4）。
15. 确认 GT/fixture 缺陷（如 page6/7 快照缺失）→ 记入
    `mk2cpp/docs/99_blockers.md`，给出影响范围与绕行；**不得改 `src/`**。
16. 每次 triage 结束更新 `out/cosim/reports/<scenario>.md` 的结论段。

---

## 7. 报告模板与覆盖仪表盘

### 7.1 `out/cosim/reports/<scenario>.md`

```markdown
# cosim report — <scenario>

- date/operator: <...>
- GT: build/nuked-sc55.exe sha256=<...>; ROM sha256: rom1=<...> rom2=<...> rom_sm=<...>
- DUT: mk2cpp.exe sha256=<...>; commit=<...>; build=<Debug|Release>
- ABI: state_raw_size=<...> sizeof_pcm_t=<...> PCM_SLOTS=<...>
- start: <reset | gt_*.qst sha256=<...> at c<...>>
- window: [A,B); hash-every=<N>

## Commands
<逐字命令 1>
<逐字命令 2>

## Result
| 层 | 项 | 结果 |
|---|---|---|
| L0 | lines compared | <n> |
| L0 | first divergence | <none | line/kind/cycle> |
| L2 | checkpoints compared | <k>（c=...） |
| L3 | hash rows compared | <n> |
| L4 | pcm_trace lines / audio hash | <...> |
| coverage | executed PC vs pc_main.txt | <...> |

## Verdict
PASS / FAIL + 一句话结论 + 分歧包路径

## Notes
exemptions（如 --ignore-sm）、已知 fixture 限制、后续动作
```

### 7.2 覆盖仪表盘（`tools/cover` 占位）

接口（`mk2cpp/tools/README.md:30-37` 已立项）：

```text
cover <map.csv> <pcset.txt> [--trace <dut.trace>] [--static <r16_hits.txt>]
      --out mk2cpp/out/coverage.md
```

- `map.csv`：`flat,block,len,kind`，`kind ∈ {translated,fallback,missing}`；
- 输入执行集 + DUT trace 解析出的 PC 集，计算：已翻译执行、兜底执行、执行集
  缺口（应为 0）、静态可达未覆盖（`flow_main.txt` + r16 命中）；
- `out/cosim/reports/*.md` 的 coverage 行直接引用 `mk2cpp/out/coverage.md` 的
  汇总数字，避免两套口径；
- M2 验收 A2.1 的 `fallback_count` 由 `cover` 输出。

---

## 8. 已知限制与 Blocker（当前阶段）

| # | 限制 | 证据 | 影响与处置 |
|---|---|---|---|
| B1 | GT `state_save` 不含 `b_ram/b_ram2`（page6/7），也不含 `adf_rd`、UART delay、SM UART 在途态、按钮/demo 进度 | `src/mcu.cpp:391,401-403,657-658,1225-1263`；`src/submcu.cpp:82-84` | M1–M3 stock 影响小（快照恢复语义两侧一致即可）；M4/`-voices:` 用快照前必须补 chunk 或豁免 E3 |
| B2 | 旧快照（`demo_postW.bin`、`probe_snap_page*.bin`）与当前 `pcm_t` ABI 不兼容 | `tools/docs/polyphony_256_todo.md:182`；实测旧 78,075 B vs 新 94,547 B | 用 §4.3 重新生成并过 ABI guard |
| B3 | 历史 T1 oracle（`LCDEN ...`、`isr=...` 打印）在当前 `src/` 中不存在 | 当前 `src/lcd.cpp:39-42`、`src/mcu.cpp` 无对应 printf；历史用法见 `task_irq_map.md:495-497` | 改用 §4.1 S0b（快照字段）与 S0/S1 计数（trace 派生：`00:03db`、`00:0492`）作为替代 oracle |
| B4 | GT 进程不自行退出 | `src/mcu.cpp:1524-1542` | runner 必须 poll+kill（§2.1） |
| B5 | 真双核不可行 | §3.2 | M2 用设计 A；设计 C 兜底 |

---

## 附录 A — 最小 M1 命令序列（可直接执行）

```powershell
# 0) 环境
$REPO='<repo root>'; $OUT="$REPO\mk2cpp\out\cosim"
$env:SDL_VIDEODRIVER='dummy'; $env:SDL_AUDIODRIVER='dummy'; $env:SDL_RENDER_DRIVER='software'
New-Item -ItemType Directory -Force -Path "$OUT\ref","$OUT\dut","$OUT\logs","$OUT\reports","$OUT\divergence","$OUT\scratch" | Out-Null

# 1) GT 参考 trace：boot [0,3M)（poll 完成后 kill；GT 不会自己退出）
$p = Start-Process -FilePath "$REPO\build\nuked-sc55.exe" -WorkingDirectory "$REPO\build" -PassThru `
     -ArgumentList '-mk2','-tracepc',"$OUT\ref\gt_boot_0_3M.trace",'0','3000000' `
     -RedirectStandardOutput "$OUT\logs\gt_boot_0_3M.stdout.txt"
#   轮询 "$OUT\ref\gt_boot_0_3M.trace" 大小连续 2s 不变 -> Stop-Process -Id $p.Id -Force
#   校验：与 tools\baselines\trace_boot3m_base.txt sha256 相同

# 2) DUT trace：同一窗口
& "$REPO\mk2cpp\build\mk2cpp.exe" -mk2 -headless -deterministic `
  -tracepc "$OUT\dut\dut_boot_0_3M.trace" 0 3000000

# 3) 逐行比较 + 首分歧历史
& "$REPO\mk2cpp\build\cosim.exe" compare `
  --ref "$OUT\ref\gt_boot_0_3M.trace" --dut "$OUT\dut\dut_boot_0_3M.trace" `
  --history 64 --report "$OUT\reports\boot_0_3M.md" --divergence "$OUT\divergence\boot_0_3M.txt"

# 4) 200M 快照窗口（GT 侧先生成 gt_c200M.qst；见 §2.1 R2/R3；DUT 侧同参数）
& "$REPO\mk2cpp\build\mk2cpp.exe" -mk2 -headless -deterministic `
  -loadsnap "$OUT\ref\gt_c200M.qst" `
  -tracepc "$OUT\dut\dut_c200M_2M.trace" 200000004 202000004 `
  -savesnap 202000000 -snapfile "$OUT\dut\dut_c202M.qst" `
  -hash-every 1000000 -hashfile "$OUT\dut\dut_c200M_2M.hash.tsv"
& "$REPO\mk2cpp\build\cosim.exe" compare `
  --ref "$OUT\ref\gt_c200M_2M.trace" --dut "$OUT\dut\dut_c200M_2M.trace" `
  --history 64 --report "$OUT\reports\c200M_2M.md" --divergence "$OUT\divergence\c200M_2M.txt"
& "$REPO\mk2cpp\build\cosim.exe" state-diff `
  --ref "$OUT\ref\gt_c202M.state" --dut "$OUT\dut\dut_c202M.state" `
  --report "$OUT\reports\c202M_state.md"
```

## 附录 B — 引用索引（GT 行为）

- trace/调度：`src/mcu.cpp:244-251,1078-1088,1305-1522`（work_thread、
  tracepc 窗口、demo/mocknote/savesnap 顺序、设备更新顺序）
- 快照：`src/mcu.cpp:1208-1303,1314-1325,1417-1429`；`src/lcd.cpp:44-80`
- CLI：`src/mcu.cpp:1863-1917,2010-2024`
- pcmtrace：`src/mcu.cpp:933-945`
- 全局状态：`src/mcu.h:88,161-174,443-448`；`src/pcm.h:27-61,63`；
  `src/submcu.h:31-42`；`src/submcu.cpp:65-84`；`src/mcu_timer.cpp:22-26`；
  `src/lcd.cpp:30-37`
- 隐藏态（不在 GT 快照）：`src/mcu.cpp:391,401-403,657-658,1392`；
  `src/submcu.cpp:82-84`
- 中断/定时器：`src/mcu_interrupt.cpp:64-209`；`src/mcu_timer.cpp:205-335`
- SM：`src/submcu.cpp:1270-1360,1387-1435`
- PCM/音频：`src/pcm.cpp:28-33,79-103,310-313,584`；`src/mcu.cpp:1620-1747`
- 证据协议与历史 oracle：`../tools/docs/evidence_protocol.md`（§1/§3/§8/§10）；
  `../tools/docs/task_irq_map.md:493-504`；`../tools/baselines/README.md:110-116`
