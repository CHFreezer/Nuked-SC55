# mk2cpp/tools

工具分目录：`h8lift/`（ROM→C++）、`tracediff/`（两模式差分）、`cover/`（覆盖率）。
均 C/C++，clang 构建；产物与调试输出写 `../out/`，不入 git。

## 实施状态（2026-09-11）

| 工具 | 状态 | 自测结果 |
|---|---|---|
| `h8lift/h8dec.c` | ✅ 已实现（解码器） | `--check-decode mach_main+mach_vec`：**checked=9235 mismatches=0** |
| `tracediff/tracediff.c` | ✅ 已实现 | 同文件 identical/exit0；删行变异在预期行命中/exit1；pc-only、cycle-only 均命中；CRLF/大小写/前缀用例通过 |
| `cover/cover.c` | ✅ 已实现 | 100% / 60% / DATA·TRAP 不计入 / static 自动识别 四组自测通过；确定性输出（两次 SHA-256 相同） |
| `h8lift/h8part.c` | ✅ 已实现（切块器） | 9217 指令 → 2576 块（与 doc06 §1.3 一致）；确定性（输入洗牌同哈希）；cover 100% |
| `h8lift/h8emit.c` | ✅ 已实现（发射器） | 9217 函数、0 stub；切片/全量 clang-cl 编译通过；对 GT handler 差分 73,736 次执行 0 mismatch |
| `tests/two_mode_check.ps1` | ✅ 已实现（编排） | boot / demo200 两模式全 PASS；失败路径验证 exit 1 |
| `hashdump`（GT `-hashdump`） | ✅ 已实现 | h1=h2=h3（解释器两跑 + `-mk2cpp` 回退）同哈希；默认无副作用 |

已知 GT 与 `h8dasm` 的差异（h8dec 以 GT 为准，已在 stderr 提示）：
1. **银行映射**：`h8dasm.c` 把所有非 rom1 地址当 rom2；GT 的 5/10/11 页是 SRAM、6/7 是扩展 RAM、12/13 是 NVRAM。基线只涉及 cp0/cp4，两者一致。
2. **非法/陷入字节**：h8dasm 报"已消耗长度"，GT 走 `MCU_ErrorTrap` 后继续/触发异常；`h8dec` 返回 `valid=0,len=0`。全 ROM 扫描 294,912 PC 中 0 字段不匹配（差异仅此语义）。

## h8lift / h8dec / tracediff / cover 命令

```
h8dec.exe --check-decode <mach_main.txt> <mach_vec.txt> [--rom1 rom1.bin --rom2 rom2.bin]
tracediff.exe --ref <trace> --dut <trace> [--history N] [--report md] [--divergence txt]
cover.exe --map <map.csv> --exec <pc_main.txt> [--static <hits.txt>] [--out out.md] [--missing-limit N]
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

## 约定

- 工具本身入库；不写死任何本地绝对路径，路径全部命令行传入。
- 错误信息用英文（便于 grep），文档与提交信息用中文。
- 新增工具必须更新本文件与 `../README.md` 的目录表。
