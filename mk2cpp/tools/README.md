# mk2cpp/tools

工具分目录：`h8lift/`（ROM→C++）、`tracediff/`（两模式差分）、`cover/`（覆盖率）、
`pcmdiff/`（音频 WAV 逐样本对照）、`midisched/`（SMF→`-midiseq` schedule）。
均 C/C++，clang 构建；产物与调试输出写 `../out/`，不入 git。

## 实施状态（2026-09-11）

| 工具 | 状态 | 自测结果 |
|---|---|---|
| `h8lift/h8dec.c` | ✅ 已实现（解码器） | `--check-decode mach_main+mach_vec`：**checked=9235 mismatches=0** |
| `tracediff/tracediff.c` | ✅ 已实现 | 同文件 identical/exit0；删行变异在预期行命中/exit1；pc-only、cycle-only 均命中；CRLF/大小写/前缀用例通过 |
| `cover/cover.c` | ✅ 已实现 | 100% / 60% / DATA·TRAP 不计入 / static 自动识别 四组自测通过；确定性输出（两次 SHA-256 相同） |
| `h8lift/h8part.c` | ✅ 已实现（切块器） | M1 9217 指令 → 2576 块（与 doc06 §1.3 一致）；M2 `--vec --static` 15999 指令 → 9463 块（223 overlap）；确定性（输入洗牌同哈希）；cover 100% |
| `h8lift/h8emit.c` | ✅ 已实现（发射器） | M1 9217 函数；M2 15999 函数（13 处 `TODO(gt)`）、0 stub；切片/全量 clang-cl 编译通过；对 GT handler 差分 73,736 次执行 0 mismatch |
| `h8lift/h8reach.c` | ✅ 已实现（静态可达，M2） | 15999 总 PC（9217 exec + 18 vec + 6764 static + 206 跳转表项）；10 个跳转表站点、1015 条表项 |
| `h8lift/smemit.c` | ✅ 已实现（SM 发射器，M3） | rom_sm 4KB 全译 4096 函数、0 `(null)`；SHA256 自检通过；opcode 模型由 `h8lift/sm_parse.py` 生成（impl 取 GT `SM_Opcode_Table`，len 取 GT handler 路径并以 `tools/disasm/smdasm.c` 的完整 AM 表交叉校验 165/165）；boot+demo200 两模式 SM trace 0 分歧 |
| `tests/two_mode_check.ps1` | ✅ 已实现（编排） | boot / demo200 两模式全 PASS；失败路径验证 exit 1 |
| `pcmdiff/pcmdiff.c` | ✅ 已实现（音频对照，M4） | 构造 10 样本 s16 WAV 自测：相等 exit0、差 1 LSB tol0 exit1、tol1 exit0、差 5 LSB tol1 exit1、`--stats` exit0；缺文件 exit2；data 未回填报错 |
| `midisched/midisched.c` | ✅ 已实现（SMF→schedule，M4） | 构造 SMF 自测：tempo 500000@ppq480 → 12000000 cycles、running status、`--start`/`-o`、缺文件 exit2 |
| `hashdump`（GT `-hashdump`） | ✅ 已实现 | h1=h2=h3（解释器两跑 + `-mk2cpp` 回退）同哈希；默认无副作用 |

已知 GT 与 `h8dasm` 的差异（h8dec 以 GT 为准，已在 stderr 提示）：
1. **银行映射**：`h8dasm.c` 把所有非 rom1 地址当 rom2；GT 的 5/10/11 页是 SRAM、6/7 是扩展 RAM、12/13 是 NVRAM。基线只涉及 cp0/cp4，两者一致。
2. **非法/陷入字节**：h8dasm 报"已消耗长度"，GT 走 `MCU_ErrorTrap` 后继续/触发异常；`h8dec` 返回 `valid=0,len=0`。全 ROM 扫描 294,912 PC 中 0 字段不匹配（差异仅此语义）。

## h8lift / h8dec / tracediff / cover 命令

```
h8dec.exe --check-decode <mach_main.txt> <mach_vec.txt> [--rom1 rom1.bin --rom2 rom2.bin]
tracediff.exe --ref <trace> --dut <trace> [--history N] [--report md] [--divergence txt]
cover.exe --map <map.csv> --exec <pc_main.txt> [--static <hits.txt>] [--out out.md] [--missing-limit N]
pcmdiff.exe --ref <ref.wav> --dut <dut.wav> [--tolerance LSB] [--stats] [--report md] [--max-print N]
midisched.exe <in.mid> [-o out.sched] [--start cycles] [--ppq N] [--byte-gap cycles] [--verbose]
```

## h8lift — ROM → C++ 反译器（M1）

```
h8lift <rom1.bin> <rom2.bin> <pcset.txt> <flow.txt> <outdir>
```

- 输入：ROM 字节、执行 PC 集（可多份并集）、flow 边（`src dst`）。
- 输出（供 GT 直接编译，`include/mk2cpp.h` ABI）：
  - `<outdir>/mk2c_r1.cpp`、`mk2c_r2.cpp`：按 PC 的翻译实现（直接操作 GT `mcu`，
    恰好一条指令语义；函数指针表 `mk2cpp_tab_cp0/cp4`）。
  - `<outdir>/map.csv`：`flat,rom,fileoff,len,kind,fn`，供覆盖率统计。
- 语义来源：`../../tools/disasm/h8dasm.c`（解码）+ `../../src/mcu_opcodes.cpp`
  （标志/寄存器行为）。**必须逐条对齐 GT**，不得"更正确"。
- 版本标记：输出文件头写工具版本 + ROM sha256；GT 侧 `MK2CPP_Version()` 报告。

## smemit — SM（子 MCU）ROM → C++ 反译器（M3）

```
smemit.exe <rom_sm.bin> <outdir> [--extra <pc-list.txt>]
```

- 输入：`rom_sm.bin`（4096 字节子 MCU 固件，向量表在 `sm_rom[0x0fec..0x0fff]`）。
- 输出（`<outdir>`，本地不入 git，供 GT `-DMK2CPP_GEN_DIR` 编译）：
  - `mk2c_sm.cpp`：逐 SM PC 的翻译实现（`smk2_pcXXXX`），恰好一条 GT SM 指令语义；
    全译 `sm.pc ∈ [0xf000,0xffff]` 共 4096 个位置（未执行的位置为死翻译或
    `SM_ErrorTrap`，永不分派到）。
  - `mk2cpp_gen_sm_init.cpp`：`MK2CPP_SM_FillTables()` 注册每个 SM PC。
  - `map_sm.csv`：`pc,op,len,fn`；`sm_pcs.txt`：翻译 PC 集（每行 `XXXX`）。
- 语义来源：GT `src/submcu.cpp`（`SM_Opcode_Table` + `SM_Opcode_*` handler），
  **逐条对齐，不得更正确**（含 T 标志 LDA/ORA/AND 与 IPCE 读的 FIXME 路径）。
- opcode 模型（`sm_op_impl[256]`/`sm_op_len[256]` 表 + 各寻址式表达式）由
  `h8lift/sm_parse.py` 从已入库来源生成，**非手算**：`impl` 取 GT
  `src/submcu.cpp` 的 `SM_Opcode_Table`（GT 只实现 165/256），`len` 取各 GT
  handler 路径实际取指字节数，并逐条与 `tools/disasm/smdasm.c` 的完整 M37450
  寻址模式表交叉校验；同时校验 `smemit.c` 的 `emit_body`/load/store 表达式
  覆盖与实现集完全一致。生成后自带校验（在仓库根执行）：
  `tools/python/python.exe mk2cpp/tools/h8lift/sm_parse.py --check mk2cpp/tools/h8lift/smemit.c`
  （仓库内嵌解释器说明见 `tools/python/README.md`；无内嵌运行时则用系统 Python 3）。
  `--extra` 可并入额外 PC（如 `tools/baselines/pc_sm_251.txt`），但仅限
  `[0xf000,0xffff]` 区间。

## tracediff — 两模式差分验证（M1）

```
tracediff --ref <trace_or_state> --dut <trace_or_state> [--history N] [--hash-every N]
```

- 对照对象：**同一 `nuked-sc55.exe`** 的默认解释器模式与 `-mk2cpp` 模式，
  同输入/同窗口分别产出 trace/快照/哈希（不建独立 DUT 进程）。
- 对比内容：PC/cp 序列、周期性 SRAM/设备寄存器哈希、快照字段；
  首分歧输出最近 N 步历史与分歧包。
- 所有输出写 `../out/cosim/`；分歧包必须可一键复现。

## cover — 覆盖率仪表盘（M1）

```
cover <map.csv> <pcset.txt> [--static <r16_hits.txt>]
```

- 统计：已翻译 / 解释器兜底 / 静态可达未覆盖 / 执行集缺口。
- 输出 Markdown 到 `../out/coverage.md`，并给出未覆盖 PC 的聚类（按 rom/区域）。

## pcmdiff — WAV PCM 逐样本对照（M4）

```
clang -O2 -o mk2cpp\tools\pcmdiff\pcmdiff.exe mk2cpp\tools\pcmdiff\pcmdiff.c
pcmdiff --ref <ref.wav> --dut <dut.wav> [--tolerance LSB] [--stats]
        [--report out.md] [--max-print N] [--meta-ref file] [--meta-dut file]
```

- 输入：两个 RIFF/WAVE 文件；遍历 chunk 定位 `fmt `/`data`（不假定 44B 头），
  支持 PCM=1 的 8/16/24/32 bit，逐样本（非逐字节）比较，容差以 LSB 计。
- 判定：默认 tolerance 0；exit `0` 在容差内且长度一致，`1` 有超差/长度不符，
  `2` 读取或格式错误。`--stats` 只输出统计并 exit `0`（长度不符也会报告）。
- 统计：首差样本 index、超差样本数/占比、`max|Δ|`、`sum|Δ|`、RMS、峰值；
  若存在 `<wav>.meta`（或 `--meta-ref/--meta-dut`），按 `first_cycle/rate`
  估算首差 cycle；`--report` 写 Markdown（含首差样本表）。
- 防护：`data` chunk size=0 但文件仍有 payload 时按「WAV 未收尾」报错
  （防止 `-audiowin` 未回填头而误判为相等）。
- 自动/人工：本工具只服务自动对照；听感判断仍必须人工（`tests/README.md`）。

## midisched — SMF → `-midiseq` schedule（M4）

```
clang -O2 -o mk2cpp\tools\midisched\midisched.exe mk2cpp\tools\midisched\midisched.c
midisched <in.mid> [-o out.sched] [--start cycles] [--ppq N]
          [--byte-gap cycles] [--verbose]
```

- 输入：SMF format 0/1（format 2 报错）；解析 running status、VLQ、
  tempo map（`FF 51 03`），合并各轨为全局绝对时间；SMPTE division 支持
  帧率换算，`--ppq` 可在 division 异常时兜底。
- 输出：`<cycle> <hexbyte> [<hexbyte>...]` 文本 schedule，`#` 开头为注释；
  **cycle 相对 `-midiseq <file> [start]` 的 start**（`--start` 可再加常量）。
- 字节节流：默认每个 MIDI 字节间隔 `--byte-gap 7680` cycles（320µs@24MHz），
  防止 GT 8192B UART 环溢出；`--byte-gap 0` 时同一事件各字节合并为一行。
- 产物不入 git：schedule 放 `mk2cpp/out/m4/corpus/`（缺失时 M4 脚本 SKIP）。

## 约定

- 工具本身入库；不写死任何本地绝对路径，路径全部命令行传入。
- 错误信息用英文（便于 grep），文档与提交信息用中文。
- 新增工具必须更新本文件与 `../README.md` 的目录表。
  （Wave 0c 写权限限 `mk2cpp/tests/**`、`mk2cpp/tools/**`，`../README.md`
  的目录表条目待补。）
