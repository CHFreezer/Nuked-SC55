# 10 M4 验收 oracle 工具链

> **暂缓（2026-09-11）**：本文的 `-voices:255` 压力矩阵（n=32/64/128/255）、
> `pcm.ext_voices` 判据、O4/S4 的 `cfg3d=0x7b + ext_voices` 期望随 256 扩展回滚
> **暂缓**（里程碑拆分后归 M5）。n=28 音频 null（`m4_audio_null.ps1`）与
> 音频/状态窗口口径仍有效（属 M4）。

状态：已评审 v1，2026-09-11。
配套文档：[07 voice 语义](07_m4_voice_spec.md) · [08 PCM 引擎与音频路径](08_m4_pcm_api.md) · [09 hand 覆盖表与集成](09_m4_integration.md) · [00 计划](00_plan.md)。
阅读顺序：00_plan §M4 → 07 → 08 → 09 → **10（验收）**。
口径：目标 = **256 声同时发音**；验收上限 `-voices:255` 是 `0xff` 哨兵 + 8-bit 池计数妥协下的**阶段性上限，非最终目标**（07 §0.5、§6 D1）；音频/听感窗口 ≥300M，状态 ≥200M；
超时 = ceil(终点/24e6×2)+15s；前期限定 L0（一次 `MK2CPP_Step` = 恰好一条 H8 指令），L1 在 n=28 null 通过前不得启用。

范围：M4「voice/PCM 原生 C++ 重写 + 容量 256 复音」的验收 oracle：
n=28 音频 null、n=32/64/128/255 压力长跑、O1–O10 矩阵对应、脚本架构、缺口。
行号口径：评审基线 HEAD `84d3e51`；引用 GT/mk2cpp 行号在实现推进后可能漂移。
依据：`mk2cpp/README.md`、`mk2cpp/docs/00_plan.md`、`tools/docs/plan_256.md` §6、
`tools/docs/polyphony_256_todo.md`、`tools/docs/task_irq_map.md` §4、
`mk2cpp/tests/`、`src/mcu.cpp`、`src/pcm.cpp/h`，
以及 07/08/09 三篇设计（voice 例程闭包/双轨状态/活性上限 255；PCM 引擎/音频路径/
`-wav:`；hand 分派/激活/`-mk2cpp-hand:0|1` A/B）。验收口径与它们对齐，冲突处见 §6。
脚本草稿在本地 `out/`（不入 git）；落地时脚本迁到 `mk2cpp/tests/`、C 工具迁到
`mk2cpp/tools/`（需同步 README）。

---

## 0. 摘要（先看这里）

1. **M1–M3 的 trace/hash 两模式 0 分歧 oracle 在 M4 只部分适用**：09 把 M4 前期
   锁定在 **L0（一次 `MK2CPP_Step` = 恰好一条 H8 指令）**，所以 hand 切片期间 `-tracepc`
   行数与顺序仍可逐行对照（09 §4.3/§5.3 gate 2）；一旦启用 L1 整例程 hook
   （n=28 null 通过前不得启用，09 §4.3），trace 行数会变，只能退回音频/状态 oracle。
   因此 M4 的权威等价性 oracle = **n=28 音频逐样本 null** + **行为/状态标量（O1–O10）**；
   **O9 的 trace 基线回归只覆盖「无 `-voices` 的默认路径」**。
   音频 null 工具链（§2）必须先于 L1 可用（见 §6 D7）。
2. 现有 `two_mode_check.ps1` 可继续作为「默认路径零回归 + 两模式 trace/hash」自动
   对照（用户约定：自动 trace/hash 比对照旧可用），但它设置了
   `SDL_VIDEODRIVER/AUDIODRIVER=dummy`（`two_mode_check.ps1:70-72`），**不得**被
   M4 的新验收（音频/听感/压力）复用为运行方式。
3. 音频取数缺一个 GT 出口：样本在 `MCU_PostSample`（`src/mcu.cpp:1841-1861`）生成
   后只进环形缓冲，没有任何 dump/hash。08 §3.5 已给出 **`-wav:<file>`**（生产者侧
   tap、默认关闭、零行为变化）；本文在其上加 **`-audiowin <start> <end>`**（固定取样窗，
   保证两次运行可比）与 **`-audiohash <cycles> <file>`** 检查点（08 已提议），
   并配 `pcmdiff` C 工具做逐样本对照/容差统计。**缺口 #1 = 落地 `-wav:`/`-audiowin`/
   `-audiohash` + `.meta` sidecar**。
4. `-pcmtrace`（`src/mcu.cpp:934-946`）只记 `reg<=3/0x3c/0x3d/0x3e`，**不记
   `0xE800` 扩展写**（`mcu.cpp:949-952`）、不记 `0x3f` 效果别名、不记读——O6/O7 及
   n>32 的 mask 证据不完整（`task_irq_map.md:483-484` 已列为 open item）。
5. `task_irq_map.md` 中 O2/O3/O10 的采集依赖 `-savesnap` 打印
   `SNAP ... isr=`/`pc=/sleep=/iml=`，但**当前源码没有 `isr` 计数符号**
   （grep 无 `g_isr4fe`；`task_irq_map.md:20,125` 引用的 `mcu.cpp:1084-1087` 在现
   版本已是 `MCU_ReadInstruction` 尾部）——**缺口 #2 = `-snapinfo` 文本标量 +
   `isr4fe` 计数**，或扩 `-hashdump` 字段。
6. 256 压力需要可控的「≥255 同时发音」MIDI 素材与注入通道：
   `-mocknote` 只发 1 个 note-on（`mcu.cpp:1512-1521`），不够。
   **缺口 #3 = `-midiseq <file> [start]`（按周期注入 schedule）+ `midisched` C 工具
   （SMF→schedule）**；素材放 `mk2cpp/out/m4/corpus/`（out 已 gitignore，不入 git）。
7. 口径已统一：**目标 = 256 声同时发音**；CLI/`PCM_MAX_VOICE` 上限 **255**（`pcm.h:27`、
   `mcu.cpp:1996`、help `mcu.cpp:2162`）与 R12 的 N=255 是 `0xff` 哨兵 + 8-bit 池计数
   的妥协上限（out/m4/12 §5.2），**非目标数字**；验收 `-voices:255` 只验到该妥协上限，
   真 256 见 §6 D1。
8. 超时统一按 **24MHz 换算 ×2**：`timeout ≈ ceil(取样终点/24e6 × 2) + 15s` 余量；
   音频/听感窗口 ≥300M（demo 200M 尚无 PCM 输出），状态窗口 ≥200M
   （`plan_256.md:95-102`）。
9. 铁律的自动/人工分界：**自动** = trace/hash/pcmtrace/wav/audiohash 对比 + 判据解析；
   **必须人工在场** = LCD 画面/声音观察、n=28 听 A/B、255 压力（妥协上限）听丢音/爆音。
   新脚本默认 dry-run，`-Execute` 才启动 GT，且 `-Execute` 必须带 `-UserPresent`。
10. **与 07/08/09 的对齐**：① 抓取命名采用 08 的 `-wav:<file>`，本文加窗口/哈希/元数据
    扩展（§2.2）；② 激活契约采用 09 §3.2「hand 表在 `-mk2cpp` 下始终参与分派（含
    n=28）、`pcm_ext_active` 仅当 `-mk2cpp && -voices:n` 且 n≠28」，故
    `-mk2cpp -voices:28` 就是 n=28 null 路径；③ n=28 null 额外做 09 gate 5 的
    **`-mk2cpp-hand:0` vs hand-on 同二进制 A/B**（§2.1 可选第三跑）；
    ④ 验收上限 `-voices:255`（**目标 256 声；255 = 0xff 哨兵/8-bit 计数妥协上限，非目标**；
    07 §0.5），真 256 见 §6 D1；⑤ L1 启用时机见 §6 D7，影响 trace gate 是否可用。

---

## 1. 现状资产盘点

### 1.1 oracle 脚本（已入库）

| 资产 | 说明 | 关键行 |
|---|---|---|
| `mk2cpp/tests/two_mode_check.ps1` | 同一 exe 跑两模式（默认解释器 vs `-mk2cpp`），场景 `boot`/`demo200`/`custom`；轮询 `-hashdump` 文件与 trace 尾行达窗后 `Stop-Process -Force`；比较 tracediff + SHA256 + 冻结 baseline | 轮询/杀进程 `:198-300`；trace/hash 比较 `:362-422`；`SDL_*_DRIVER=dummy` `:70-72` |
| `mk2cpp/tests/README.md` | 参数表、输出物、退出码、注意事项 | `:26-90` |
| `mk2cpp/tools/tracediff/tracediff.exe` | 两 trace 差分；exit 0=一致，1=首个分歧，2/3=前缀问题 | `mk2cpp/tools/README.md:73-84` |
| `mk2cpp/tools/cover/cover.exe` | 覆盖率仪表盘 | `mk2cpp/tools/README.md:85-92` |
| `tools/baselines/` | 冻结 trace：`trace_boot3m_base.txt`（[0,3M)）、`trace_200m_base.txt`（[200M,202M)）；另有 PC/flow/disasm fixture（该目录除 README 外 gitignored） | `tools/baselines/README.md:110-116` |

现脚本的定位：**M1–M3 回归**（默认路径 0-diff、两模式 trace/hash 一致）。M4 需要在其
之外新增音频/压力 oracle；不要改其行为（默认路径回归要保原样）。

### 1.2 GT CLI argv 现状（`src/mcu.cpp:1946-2177`）

| 选项 | 解析位置 | 语义/默认 | 现状 |
|---|---|---|---|
| `-mk2` | `:1984-1988` | ROM_SET_MK2，`autodetect=false` | 已有 |
| `-mk2cpp` | `:1977-1983` | 置 `mk2cpp_enabled=1` + `MK2CPP_Init()`；打印 mixed/版本 | 已有 |
| `-voices:<n>` | `:1993-2005` | `28..PCM_MAX_VOICE(255)`；`n!=28` 置 `pcm_ext_enabled=1`；越界警告忽略 | 已有（GT 扩展态；B 补丁未落地） |
| `-demo [cycles]` | `:2006-2011` | demo 按键序列；默认 144M，<144M 钳到 144M（`:2181-2185`） | 已有 |
| `-mocknote [cycles]` | `:2012-2017` | 经 `SM_PostUART` 发 `90 3C 64` 三字节；默认 144M | 已有（单音） |
| `-tracepc <file> [start end]` | `:2018-2026` | 统一 main+SM PC trace；默认 `[200M,210M)` | 已有 |
| `-pcmtrace` | `:2027-2030` | 写 `pcm_trace.log`（CWD，固定名）：`reg<=3/3c/3d/3e` | 已有（覆盖不足，见 §5） |
| `-savesnap <cycles>` | `:2031-2034` | 写 `demo_snap.bin`（CWD）全状态二进制，一次性 | 已有 |
| `-hashdump <cycles> <file>` | `:2035-2046` | 文本 FNV-1a 状态哈希，一次性 | 已有 |
| `-loadsnap <file>` | `:2047-2050` | 启动载入快照 | 已有（M4 建议不用，统一 from reset） |
| `-gain:<x| xdb>` | `:2051-2072` | 线性倍数或 dB；作用在 `MCU_PostSample` 钳位前 | 已有（两次运行必须一致或都省略） |
| 其他 | `-p:`/`-a:`/`-ab:`/`-float`/ROM 集/`-gs`/`-gm` | — | 与 M4 oracle 无关 |

帮助文本 `:2143-2164` 已列 test/verification 选项，**新增选项必须同步改 help 与
`mk2cpp/tools/README.md`/`mk2cpp/README.md`**（`mk2cpp/tools/README.md:98`）。

时间基准：`mcu.cycles += 12`/指令（`:1457`），主 H8 = 24MHz；`demo` 起点 144M，
首个有 PCM 输出的听感窗口 ≥300M。

### 1.3 trace 格式（`mcu.cpp:226-252`、`:1459-1478`）

- `touchpc` 统一文件，主行：`m <mcu_cycles> <cp:pc>`（`%02x:%04x`）；SM 行：
  `s <sm_cycles> <pc>`（`%04x`）。
- 窗口 `[tracepc_start, tracepc_end)`：进入窗口惰性 `fopen(file,"w")`（缓冲 1MiB），
  到 `end` 后 `fflush/fclose`；SM 侧由 `trace_write(1,...)` 写同一文件
  （`src/submcu.cpp` `SM_Update` 内调用，M3 已对接）。
- 完成判定：尾行 cycle ≥ 窗口末端（现脚本用尾行 `m`/`s` 的第二个字段，
  `two_mode_check.ps1:181-196`）。

### 1.4 hashdump 格式（`mcu.cpp:1306-1404`）

文本 v1：`version = hashdump v1`、`requested_cycles`、`mcu.cp/pc/sr/cycles`、
`pcm.config_reg_3c/3d`、`pcm.irq_assert`、`pcm.cycles`、`sm.pc/cycles`，随后
`hash.<range> = <16hex FNV-1a64>`：`mcu/ram/sram/dev_register/frt/timer/.../pcm/
sm/sm_ram/sm_shared_ram/.../uart_buffer/uart_rx_byte` + `hash.lcd_state`。
**缺**：`mcu.sleep`、IML 解释值、`pcm.select_channel`、`pcm.voice_mask[32]`、
`irq_channel`、心跳 `isr4fe` 计数——O2/O3/O7 直接判据缺工具（§5.1）。
**状态 dump 文本冻结**：为保 M1–M3 历史基线兼容（boot hashdump SHA256
`91CE3BF8…` / demo `4B445776…`），`-hashdump` 保持 v1 不新增字段；
`pcm.ext_voices`（方案 A 的 S4/O4 判据，out/m4/12 §4.3）只由 `-snapinfo`
提供，不加入 `-hashdump`。

### 1.5 pcmtrace 格式（`mcu.cpp:934-946`）

每行：`pcm reg=%02x val=%02x pc=%02x:%04x cyc=%llu`（同一 `reg` 在一次运行可多次
出现；文件在 CWD，固定名 `pcm_trace.log`，无缓冲）。
覆盖：PCM 主窗口 `0xE000-0xE3FF` 且 `reg ∈ {0,1,2,3,0x3c,0x3d,0x3e}`。
**不覆盖**：`0xE800-0xE83F` 扩展写（`mcu.cpp:949-952` 直接进 `PCM_WriteExt`，无日志）、
`0x3f` 效果别名（R17）、任何读（含 `0xE820` 完整 IRQ slot）。

### 1.6 音频输出路径（对照为什么需要 dump）

- 混合/限幅在 `PCM_Update`（`src/pcm.cpp:584-703`），每个输出样本调
  `MCU_PostSample`；`MCU_PostSample` 应用 `master_gain`、右移 15、int16 钳位后写
  `sample_buffer` 环形缓冲（`mcu.cpp:1841-1861`）。
- SDL 回调 `audio_callback` 只搬环形缓冲（`mcu.cpp:1734-1741`）；设备参数：
  mk2 采样率 **66207Hz**、2ch、S16（`mcu.cpp:1779-1783`）。
- 结论：**采样序列是 mcu/pcm 周期的确定性函数**；dump 必须挂在
  `MCU_PostSample`（生成侧），不能挂在音频回调（消费侧，受设备节拍影响）。
- 当前**没有**任何 wav/raw/hash 音频出口；08 §3.5 已给出 `-wav:<file>` 设计（未实现），
  本文在其上加固定窗口/哈希/元数据（§2.2）。

### 1.7 心跳/状态诊断（与文档漂移）

- `task_irq_map.md:20,125,496-504` 的 O2/O3/O10 依据 `-savesnap` 打印
  `SNAP ... isr= / pc= / sleep= / iml= / pend=`；`g_isr4fe` 引用
  `src/mcu.cpp:1084-1087`。
- 实际当前源码：`:1084-1089` 是 `MCU_ReadInstruction` 尾+T 标志异常；全仓库 grep
  `isr4fe/g_isr` 只命中文档，**无实现**。`-savesnap` 只打印
  `snap: dumped full state at c...`（`:1530`）。
- ⇒ O2/O3/O10 的机读采集当前不可用，属必须先补的验收工具缺口（§5.1 G3）。

### 1.8 baseline fixture（状态与音频均已冻结）

- `tools/baselines/trace_boot3m_base.txt`、`trace_200m_base.txt`：O9/默认回归的冻结
  trace（本地 fixture，缺失时 `two_mode_check.ps1` 记 SKIP）。
- `tools/baselines/README.md:110-116` 明确这两个是 stock GT 的 0-diff oracle。
- **音频 baseline 已冻结（G6，2026-09-11）**：`tools/baselines/m4_audio/`
  `stock_300M_320M.wav / .wav.meta / .audiohash / .state.hash` +
  `SHA256SUMS.txt`（本地 fixture，`tools/baselines/*` 已 gitignore，不入 git；
  以 SHA256SUMS 冻结完整性）。`m4_audio_null.ps1 -CheckBaseline` 会把本次 stock
  跑出的 WAV payload/`-audiohash` 与它对照，防止「两个模式一起错」；
  stock PCM/ROM 路径或音频 tap 变更后需重冻并更新 SHA256SUMS。

---

## 2. n=28 音频 null 测试设计

### 2.1 目标与命令

验证：M4 原生引擎在 n=28（stock 语义）下与默认解释器**音频输出等价**。

```
stock: build\nuked-sc55.exe -mk2 -demo \
         -wav:<out>\stock.wav -audiowin 300000000 320000000 \
         -audiohash 320000000 <out>\stock.audiohash
M4   : build\nuked-sc55.exe -mk2 -mk2cpp -voices:28 -demo \
         -wav:<out>\m4.wav -audiowin 300000000 320000000 \
         -audiohash 320000000 <out>\m4.audiohash
可选A/B: build\nuked-sc55.exe -mk2 -mk2cpp -voices:28 -mk2cpp-hand:0 -demo \
         -wav:<out>\handoff.wav -audiowin 300000000 320000000   (09 §5.3 gate 5)
```

- `-voices:28` 时 GT 保持 stock 语义（`mcu.cpp:2003` `pcm_ext_enabled=(n!=28)`），
  等价比较才有意义；`-mk2cpp` 打开 M4 原生引擎（激活契约 = 09 §3.2：hand 表在
  `-mk2cpp` 下始终参与分派，含 n=28；见 §6 D2）。
- 两次运行除模式参数外**一切相同**（ROM 集、demo、gain 省略、音频窗口、
  采样率由 ROM 集决定）。
- 运行方式遵守铁律：**LCD 窗口 + 声音输出 + 超时 kill + 用户在场**
  （`plan_256.md:89-102`）；不用 `-loadsnap`，统一 from reset。

09 §5.3 的**每切片 gate**（hand 每落地一个 PC 切片都要过，本文脚本提供自动化部分）：
1. `-mk2cpp -mocknote 144000000 -pcmtrace -savesnap 150000000` 与不带 `-mk2cpp` 的
   `pcm_trace.log` 逐行一致（reg/val/pc/cyc）；
2. boot 0–3M + demo200 trace 与 `tools/baselines` 0 分歧（L0 阶段 trace 行数不变）；
3. `-hashdump 202000000` 的 `hash.pcm` 与 stock 一致；
4. hand 命中可证（`MK2CPP_HandHitCount()>0`/per-entry hits，09 §2.6）；
5. `-mk2cpp-hand:0` 重跑与 hand-on 产物**字节一致**（同二进制 A/B，09 §5.4）。
本脚本的 `-HandOff` 可选第三跑即 gate 5；gate 1–3 由 `two_mode_check.ps1` +
`pcmdiff`/`Get-FileHash` 覆盖。

### 2.2 取数：`-wav:` / `-audiowin` / `-audiohash` 接口设计（默认零变化）

命名与 tap 点**采用 08 §3.5 的 `-wav:<file>`**；本文只加"固定窗口 + 检查点 + 元数据"，
保证两次独立进程的产物可比（否则播放长度受 kill 时刻影响）。

```
-wav:<file>                [08 §3.5] 生产者侧 tap：MCU_PostSample 钳位后的真实链路样本，
                           写标准 44B RIFF/WAVE 头（PCM=1, ch=2, S16, rate=spec.freq）；
                           默认关闭。无此 flag 时仅多一次空指针判断，默认路径零变化。
-audiowin <start> <end>    [本文加项] 只写 mcu.cycles ∈ [start,end) 的样本；缺省=整段运行。
                           到 end 时写 `<file>.meta` 并 fclose（确定性完成标记）。
-audiohash <cycles> <file> [08 建议] 在目标 cycle 对 PostSample int16 流写 FNV-1a64
                           单行文本（`audio_fnv1a = ...`）；可多次出现做长跑检查点。
```

实现要点（落地时，默认路径零变化）：
- 新 static 开关/窗口/`FILE*`/计数；解析在 `mcu.cpp` argv 链（`:2046` 后）。
- `MCU_PostSample`（`:1841-1861`）在 `master_gain`+int16 钳位后追加：
  `if (g_wav && mcu.cycles 在窗) { 惰性写; int16_t lr[2]; fwrite; FNV }`。
  不碰环形缓冲、不走音频回调 → 不受真实设备抖动影响。
- 窗口关闭/哈希落盘在 `work_thread` 的 tracepc 开/关旁（`:1459-1478`）；进程退出时
  回填 RIFF/data size（08 §3.5）。
- 若评审不接受 `-audiowin`：备选是只做 `-wav:` + `-audiohash` 检查点，但 WAV
  长度随 kill 时刻抖动，逐字节比较需先按 `.meta`/`audiohash` 对齐，脚本更脆。

`<file>.meta`（`wavdump v1`，`-audiowin` 时写）：

```
version = wavdump v1
format = s16le
channels = 2
rate = 66207              # mk2: 66207；mk1/jv880: 64000（mcu.cpp:1780）
start_cycles = 300000000
end_cycles = 320000000
first_cycle = 300000012   # 第一个落窗样本时的 mcu.cycles
last_cycle  = 319999990
sample_pairs = 1372195    # 样本帧数
data_bytes = 5488780      # = pairs*4
payload_fnv1a = 0123...   # FNV-1a64(payload)，GT 现成 fnv1a64（mcu.cpp:1314）
```

### 2.3 对照窗口与超时

| 项 | 取值 | 依据 |
|---|---|---|
| 音频对照窗（默认） | `[300M,320M)` ≈ 0.83s 音频 | `plan_256.md:101`（demo 200M 无 PCM，声音 ≥300M） |
| 现场听感窗 | 可放宽到 `[300M,400M)` ≈ 4.2s | 同上 |
| 状态窗（辅助） | `[200M,202M)` + hash@202M | `plan_256.md:95,102` |
| 超时 | `ceil(End/24e6×2)+15s`：320M→约 42s | `plan_256.md:96` |
| 进程管理 | 轮询 `.meta` 出现即视为完成，`finally` 里 `Stop-Process -Force` | `two_mode_check.ps1:198-300` 同款思路 |

不允许：`SDL_VIDEODRIVER/SDL_AUDIODRIVER=dummy`；不设超时的无限等待；无人值守跑完
就下结论。

### 2.4 容差定义

- **默认 = 0（逐字节/逐样本相等）**：M4 若保留定点 DSP 步骤（`00_plan.md` §M4
  「保留量化步骤并用音频对照」），WAV payload 应逐字节一致、`payload_fnv1a` 与
  `-audiohash` 相等；比较直接 `Get-FileHash SHA256` + `pcmdiff --tolerance 0`。
- **量化容差（仅当 M4 确引入浮点）**：必须先用 `filter_float_port.md` 的量化误差
  结论**书面固化**，建议初始门限：
  - 单样本 `|Δ| ≤ 1 LSB`（int16）；
  - 连续超差点 ≤ 3 样本，且超差样本占比 ≤ 0.1%；
  - 1s 窗口 RMS 差 ≤ -80dBFS；
  - 同时 `hashdump` 在 300M/320M 的状态标量（O1–O4；若 M4 数据结构不同则只比
    `mcu.pc/sr/cycles`、`pcm.config_reg_3d/3c`、LCD 帧）无异常。
- 容差模式的结论必须标注「量化误差来源 + 适用版本」，不得静默放宽。

### 2.5 输出物（`mk2cpp/out/m4/audio_null/`）

```
stock.wav / .meta / .audiohash   默认解释器 dump（08 -wav: + 本文 -audiowin/-audiohash）
m4.wav / .meta / .audiohash      M4 n=28 dump
handoff.wav / .meta              可选 -mk2cpp-hand:0 A/B（09 gate 5）
stock.stdout.txt / m4.stdout.txt LCDEN/demo/hashdump 行
stock.stderr.txt / m4.stderr.txt
audio_diff.md                    pcmdiff 报告（首差样本/最大Δ/RMS/容差判定）
summary.txt / plan.txt           检查表 + dry-run 命令留档
```

---

## 3. n=255 压力测试设计（目标 256 声；255 = 妥协上限）

### 3.1 ≥255 同时发音 MIDI 素材要求

**素材放哪（不入 git）**：`mk2cpp/out/m4/corpus/`（`mk2cpp/out/` 已 gitignore；
或构建输出目录的临时子目录，收尾清空）。**原始 SMF 与转换后的 schedule 都不入库**。

**待办（需用户）**：≥255 同时发音的验收素材（SMF 或指定曲目）尚未提供；
确认后由 `midisched` 转 schedule。

内容要求（建议验收素材 `poly255.smf` + 生成的 `poly255.sched`）：
- 16 通道都用满，鼓通道（ch10）另加；含 program change / bank select / sustain(CC64)
  保持音，确保同时发音峰 ≥255；
- 音符重叠窗口足够长（≥10M cycles ≈0.42s 峰值保持），并有多次峰谷交替；
- 从 reset 运行，注入起点 ≥192M（启动动画 120M + demo 键序窗口之后
  `plan_256.md:97-101`）；建议 schedule 起点 200M，峰值放 300M–320M（听感窗）；
- 总时长至少到 400M，留出长跑观察。

注入通道：`-mocknote` 只有 3 字节（`mcu.cpp:1512-1521`），不够。
**新增 `-midiseq <file> [start]`**：
- `<file>` 为文本 schedule：`<cycle> <hexbyte> [<hexbyte> ...]`，`#` 注释；
  GT 在 `mcu.cycles` 到点后按序 `MCU_PostUART`（与真机 MIDI 入径一致，
  `src/midi_win32.cpp:51-59`）；buffer 共用 8192B 环形（`mcu.h:445`），超速会覆盖，
  实现须带 backpressure 或在 `midisched` 里保证字节间隔（见 §5.1 G5）。
- 配套 C 工具 `midisched`（SMF→schedule，含 tempo→cyc@24MHz 换算）放
  `mk2cpp/tools/midisched/`，README 同步。

### 3.2 运行矩阵

| 级别 | 命令骨架（from reset，窗口+声音+kill） | 关注点 |
|---|---|---|
| n=32 | `-mk2cpp -voices:32 -midiseq poly32.sched 200000000 -pcmtrace -wav:out.wav -audiowin 300M 320M -audiohash 400M out.ah -hashdump ... -tracepc ...` | mask 刚好跨 32 位边界（O5/O6 交界） |
| n=64 | 同上 `-voices:64` | ext 窗口 mask 字节 4..7 |
| n=128 | `-voices:128` | select_channel/IRQ slot 中段 |
| n=255 | `-voices:255` | 验收上限（R12）；`cfg3d=0x7b`（hashdump 或 snapinfo）+ `pcm.ext_voices=255`（仅 `-snapinfo`；方案 A，out/m4/12） |
| n=256 | **当前 CLI 拒绝**（`mcu.cpp:1996`） | 本期不做（§6 D1）；容量 256 的术语见 07 §0.5 |

每级建议跑 `RunTo=400M`（≈16.7s 模拟时间；超时 ≈49s），中途采样点若需多点见
§5.1 G4（多 `-hashdump`）。n=28 与 `-demo` 的音频 null 由 §2 脚本负责，此处只跑
扩展级别。

### 3.3 判定项

| # | 判定 | 采集 | 期望 | 现状 |
|---|---|---|---|---|
| S1 | 无复位/无 stall | `-tracepc [RunTo-2M,RunTo)` 尾行 cycle 达窗；窗口内 `00:037A` 不持续；stdout `LCDEN 0` 次数 ≤2（boot 1 + demo 电源循环 1） | 进程到点被杀前仍在推进 | 部分可用（trace），isr 需 G3 |
| S2 | PCM 激活 | `-wav:`/`-audiohash` 非零（pcmdiff 统计 RMS/过零）；`-pcmtrace` 有 mask/config 写 | 有实际波形输出 | dump 缺（G1） |
| S3 | CPU 占空比合理 | `proc.TotalProcessorTime / wall`；模拟实时倍率 = RunTo / (wall×24e6) | 建议 duty ≤0.7（单核占比），实时倍率 ≥0.9，**报告为主**，阈值待定（§6 D3） | 脚本可算，无 GT 改动 |
| S4 | `config_reg_3d = 0x7b` 且 `ext_voices = n` | `pcm.config_reg_3d`（hashdump 或 snapinfo）+ `pcm.ext_voices`（**仅** `-snapinfo`；`-hashdump` v1 格式保持不变，方案 A，out/m4/12 §4.3） | 所有级别 `cfg3d=0x7b`（bit5=ROM bank，保持 stock）、`pcm.ext_voices=n`（32/64/128/255）；字段缺失 → SKIP | 已定案（方案 A，2026-09-11） |
| S5 | `select_channel` 覆盖 0..n-1 | `-pcmtrace reg=3e` 全体值 | 唯一值 = n，最大 = n-1 | 现成（n=255 时 255 个值） |
| S6 | `voice_enable` = n bit | `-pcmtrace reg=00..03` 位并集 popcount；n>32 加 ext `0xE800..0xE81B` 写 | popcount(n)=n；ext 非零且与低 32bit 不重叠错误 | 低 32 位现成；ext 缺（G2） |
| S7 | IRQ slot ≥32 完整 | ext read `0xE820` 或 `pcm.irq_channel`/`d15c` 日志 | R15：完整 slot、无 `&0x1f` 截断 | 缺（G2/G3） |

### 3.4 O1–O10 逐项对应检查方法（`task_irq_map.md:488-508`）

| # | 信号 | 期望（摘原文） | 采集/检查方法 | 自动化程度 | 现状 |
|---|---|---|---|---|---|
| O1 | LCD enable | `LCDEN 0 pc=04:662b cyc≈588`、`LCDEN 1 pc=04:668d cyc≈4.22M`；`-demo` 另有 145.72M/163.94M | stdout 正则 `LCDEN`（`src/lcd.cpp:39-49` 打印）；同时**人工看 LCD** | 自动 + 人工 | 现成（stdout） |
| O2 | Heartbeat | `isr≈1382/20M`，持续增长；demo 320M→isr≈21460 | 需 `isr` 计数 + 文本标量 | 自动 | **缺 G3**（当前源码无 `g_isr4fe`，见 §1.7） |
| O3 | Idle 状态 | 20M 时 `pc=00:0492, sleep=1, iml=0, pend=0` | 同上（`-snapinfo`/hashdump 扩展） | 自动 | **缺 G3** |
| O4 | Config 字节 | stock `0x7b`；扩展也 `0x7b` + `ext_voices=n`（方案 A，out/m4/12；原 `cfg3d=n-1` 期望作废） | `pcm.config_reg_3d`（hashdump 或 snapinfo）+ `pcm.ext_voices`（仅 `-snapinfo`；`-hashdump` v1 格式不变），或 `-pcmtrace reg=3d`（应恒 `7b`） | 自动 | **现成（`ext_voices` 字段落地后）** |
| O5 | 音符驱动主 mask（n≤32） | `-mocknote 144M -pcmtrace`：`pc=00:5527`（disable flush）/`00:5664`（enable flush 非零）≈144.05M | `-pcmtrace` reg 00..03 + PC/cyc | 自动 | 现成 |
| O6 | 扩展 ext 写（n>32） | 音符后 `0xE800..0xE803` 非零写在 `0x5529/0x5666` 回读前 | 需 `-pcmtrace` 记录 `PCM_WriteExt`（或解析快照 `pcm.voice_mask[4..]`） | 自动 | **缺 G2** |
| O7 | IRQ slot（n>32） | slot≥32 时 `d15c[slot]` 完整下标，无 `slot-32` 混叠；`0xE820` 读路径 | 需 ext 读日志/`irq_channel` 快照字段 | 自动 | **缺 G2/G3** |
| O8 | 电源循环后存活 | ≈215.37M 不复位/不 stall；`0x4134C` 配置重init；isr 继续涨 | `-tracepc [214M,216M)` grep `04:134c`；hashdump@200M/220M；`-demo` | 自动 + 人工 | trace 现成；isr 缺 G3 |
| O9 | 回归（无 flag） | 无 flag 的 200M–202M PC trace 与冻结 baseline 逐字节一致 | `two_mode_check.ps1 -Scenario demo200` + `tools/baselines/trace_200m_base.txt` | 自动（允许照旧 headless） | **现成** |
| O10 | 失败特征 | stall：`isr≈75`、`pc=00:037A`、`iml=7`（FRT2 handler）或复位循环 | trace 里 `00:037A` 持续 + stdout `LCDEN 0` 重复；`isr` 平 | 自动 + 人工 | trace 现成；isr 缺 G3 |

> O1–O10 的「用户现场观察」项（LCD 动画、demo 音、255 丢音/爆音）不因上表自动化而取消；
> 铁律要求带窗口+声音并现场看/听（`plan_256.md:91-94`）。

---

## 4. 脚本架构

### 4.1 自动/人工分界（硬边界）

**可自动（脚本判定，允许照旧的确定性对照）**
- trace/hash/pcmtrace/wav/audiohash 文件级对比（tracediff/SHA256/pcmdiff）；
- O1/O4/O5/O8/O9/O10 的文本或二进制判据；
- 进程生命周期：带窗口启动、轮询落盘、超时 `Stop-Process -Force`、CPU duty 统计；
- dry-run 参数校验与命令打印。

**必须人工在场（脚本只负责提示与留窗）**
- 任何从 reset 跑真实 GT 的 M4 验收（铁律：不得 headless/静默）；
- n=28 音频 A/B 听感确认；
- 255 压力听：丢音、爆音、音质劣化、卡顿；LCD 是否异常/复位动画；
- 出现分歧时的现场判断（是否继续、是否保存现场产物）。

因此新脚本：
1. 默认 **dry-run**（只校验/打印，不启动 GT）；
2. `-Execute` 才启动 GT，且必须 `-UserPresent`，否则拒绝；
3. **绝不**设置 `SDL_VIDEODRIVER/SDL_AUDIODRIVER=dummy`（与 `two_mode_check.ps1:70-72`
   明确不同）；
4. `finally` 里无条件 kill；超时按 24MHz×2 计算，不用固定长等待。

### 4.2 脚本（已从本地草稿落地到 `mk2cpp/tests/`）

| 脚本 | 用途 | 关键参数 | 现状 |
|---|---|---|---|
| `m4_audio_null.ps1` | §2 n=28 stock vs `-mk2cpp -voices:28` 音频 null；`-HandOff` 可选加第三跑（09 gate 5）；`-CheckBaseline` 加 G6 冻结基准门禁（`-BaselineDir` 默认 `tools\baselines\m4_audio`） | `-Scenario demo\|midi`、`-AudioStart/End`（默认 300M/320M）、`-MidiSchedule`、`-Gain`、`-HandOff`、`-CheckBaseline`/`-BaselineDir`/`-BaselineName`、`-TimeoutSec`（0=自动）、`-Execute`、`-UserPresent` | 已落地：dry-run 打印命令/超时/比较计划（含基准状态）；真实运行骨架含轮询+kill+SHA256/pcmdiff 比较；`-CheckBaseline` 先验 SHA256SUMS 再比 stock payload/audiohash |
| `m4_stress_voices.ps1` | §3 压力矩阵与 S1–S7 / O1–O10 判据 | `-VoiceLevels @(32,64,128,255)`、`-Scenario midi\|demo`、`-RunTo 400M`、`-AudioStart/End`、`-HashAt`、`-TraceFrom/To`、`-Execute`、`-UserPresent` | 草稿：dry-run 打印每级命令与预期检查；真实运行骨架含 pcmtrace/hashdump/trace 解析与汇总表 |

脚本路径解析：`$PSScriptRoot` = `mk2cpp/tests` → repo 根 = `..\..`；GT exe 固定
`build\nuked-sc55.exe`，工作目录 `build`（ROM BasePath）。
`pcm_trace.log`/`demo_snap.bin` 是 CWD 固定名（`mcu.cpp:940,1526`），多次运行需跑完
即改名搬运，脚本已按 per-run 子目录处理。

---

## 5. 缺口清单与最小实现方案

### 5.1 GT 侧（新增选项，默认全部零变化）

| 缺口 | 最小实现 | 风险/注意 |
|---|---|---|
| **G1 `-wav:`/`-audiowin`/`-audiohash`**（P1） | 见 §2.2 + 08 §3.5；argv 解析 + `MCU_PostSample` 落盘（WAV 头回填）+ work_thread 关窗写 `.meta`/hash；约 80 行 | 落盘在主循环内，文件写可能拖慢实时（窗口 20M cycles ≈ 1.1MB，可忽略）；默认路径零变化 |
| **G2 `-pcmtrace` 覆盖扩展**（P1） | ① `PCM_WriteExt` 调用点（`mcu.cpp:949-952`）记 `ext reg/val/pc/cyc`；② 记 `0x3f`（R17）与 `0xE820` 读；③ 可选 `-pcmtrace [file]` 支持 per-run 文件名 | 日志量：mask latch 每音符 2 簇×32B，压力下可控；现文件为无缓冲 `fprintf`，建议保持小窗口并在跑完搬走 |
| **G3 `-snapinfo` + `isr4fe`**（P1） | 在取指后统计 `mcu.pc==0x04FE` 的 fetch 计数（对应文档 `g_isr4fe`）；无参选项，每处 `-hashdump` 点与进程退出时向 stdout 写一行 `SNAP ... cycles/pc/cp/sr/sleep/iml/pend/cfg3c/cfg3d/select_channel/voice_mask_popcount/irq_channel/ext_voices`（`ext_voices` = 方案 A 的 S4/O4 判据，out/m4/12 §4.3；该标量只在此 snapinfo 输出提供，状态 dump 文本保持 v1 不变） | 计数器位置要与文档口径一致（04FE epilogue）；M4 接管后 04FE 可能不再执行 → **同时给出「事件计数」替代口径或明示只对解释器路径有效** |
| **G4 多 `-hashdump` 点**（P2） | `-hashdump` 改数组（≤8 个），各自一次性写；默认单点行为不变 | 用于长跑漂移/多点 isr 观察；不阻塞 P1 |
| **G5 `-midiseq <file> [start]`**（P1） | 文本 schedule 注入 `MCU_PostUART`；到点检查环形缓冲余量，满则延后；默认起点 200M | 与 `-demo`/`-mocknote` 同时用要文档化优先级；buffer 8192B（`mcu.h:445`）不是瓶颈，但仍需 backpressure 防覆盖 |
| **G6 音频 baseline 冻结**（P0，**已冻结 2026-09-11**） | 用 stock 跑 `-wav:`+`-audiowin`（300M–320M）+ `-audiohash 320M` + `-hashdump`@320M，存 `tools/baselines/m4_audio/`（本地 fixture，gitignore，不入 git），以 `SHA256SUMS.txt` 冻结四个文件；`m4_audio_null.ps1 -CheckBaseline` 先验 SHA256SUMS 再要求本次 stock 的 WAV payload/`-audiohash` 与基准一致（不符=FAIL，基准缺失=SKIP） | 基准目录固定为 `tools/baselines/m4_audio/`；以后每次改 stock PCM/ROM 路径或音频 tap 后重冻（并解释原因） |

### 5.2 工具侧（C，遵循「自制工具优先 C」，放 `mk2cpp/tools/`）

| 缺口 | 工具 | 功能 |
|---|---|---|
| **G7 WAV 比较器** | `mk2cpp/tools/pcmdiff/pcmdiff.c` | `--ref/--dut <wav>`（定位 44B 头/data chunk）+ `--tolerance <lsb>`：逐样本首差 index/估算 cycle（用 `.meta` rate/start）、最大/均值 Δ、超差计数、RMS/峰值、`--stats` 仅统计；exit 0/1/2 |
| **G8 SMF→schedule** | `mk2cpp/tools/midisched/midisched.c` | 解析 SMF（format 0/1，running status、tempo map），按 24MHz 输出 `<cycle> <bytes...>`；可 `--ppq` 兜底；输出即 G5 输入 |
| （可选）**G9 状态快照解析** | `mk2cpp/tools/snapinfo` 或复用 `-snapinfo` 文本 | 读 `demo_snap.bin`（`state_save` 顺序，`mcu.cpp:1226-1264`）导出 `config_reg_3d/select_channel/voice_mask`；仅在 G3 未落地时兜底（**注意快照格式含 `pcm sizeof` 在扩展态不兼容**，`polyphony_256_todo.md:182`） |

新增工具必须补 `mk2cpp/tools/README.md` 与 `mk2cpp/README.md` 目录表
（`mk2cpp/tools/README.md:98`）。

### 5.3 语料 fixture（本地）

- `mk2cpp/out/m4/corpus/poly255.smf` + `poly255.sched`（≥255 同时音；来源见 §6 Q1）；
- `mk2cpp/out/m4/corpus/poly28.smf`（16 通道密集但 ≤28 音，用于 n=28 null 的「非 demo」
  目标性素材）；
- `tools/baselines/m4_audio/stock_300M_320M.wav(.wav.meta/.audiohash/.state.hash)`
  + `SHA256SUMS.txt`（G6，已冻结；本地 fixture、gitignore，校验/重冻见
  `tools/baselines/README.md`）。

---

## 6. 待定项与待办（需用户）

### 6.1 待定项（D1–D7；与 07 §6、08 §7、09 §7 互为补充）

| # | 事项 | 状态/说明 |
|---|---|---|
| **D1** | 真 256 声是否/何时放开 | **目标 = 256 声同时发音**；`-voices:255` 是 0xff 哨兵 + 8-bit 计数妥协上限（out/m4/12 §5.2），**非目标**；方案 A 下 `cfg3d` 恒 `0x7b`、与 256 无关。若把 `-voices:256` 当硬指标：需放开 clamp、数组 ≥264/EFF 260..263、替换哨兵表示（R12）与 mask bit255 的表示，属设计变更；本期不做 |
| **D2** | M4 原生引擎激活契约 | 已冻结（09 §3.2）：hand 覆盖表在 `-mk2cpp` 下始终参与（含 n=28）；`pcm_ext_active=1` 仅当 `-mk2cpp && -voices:n` 且 n≠28；`-mk2cpp-hand:0\|1` 做同二进制 A/B；无 `-mk2cpp` 时 stock 逐字节不变（O9） |
| **D3** | CPU duty 定义与阈值 | 建议 dual metric：进程 CPU 占单核比 + 模拟实时倍率；阈值先报告（暂定 duty ≤0.7、倍率 ≥0.9），待用户确认 |
| **D4** | M4 后 hashdump 语义 | 原生数据结构与 GT 的 `pcm`/`sram` 可能不再同布局；09 要求 hand 状态入快照（`MK2CPP_StateSave/Load`）前，n=28 null 以音频为准，hash 只比与布局无关的标量（O1–O4） |
| **D5** | pcmtrace 文件名 | 建议 `-pcmtrace [file]`（缺省仍 `pcm_trace.log`），避免并行/多级运行互相覆盖 |
| **D6** | `config_reg_3d` bit5 与 n>32 | **已解决（方案 A，2026-09-11，out/m4/12 §4）**：`reg_slots = pcm_ext_active ? pcm_ext_voices : ((config&31)+1)`；`config_reg_3d` 保持 stock `0x7b`（bit5=ROM bank 模式）；报告字段取 `pcm.ext_voices`（仅 `-snapinfo`，不是 `engine.voices`；`-hashdump` 格式保持不变）；`rom2[0x1433]` config stamp 取消；S4/O4 新期望 = `cfg3d=0x7b` + `ext_voices=n`（判据字段缺失时 SKIP） |
| **D7** | L1 整例程 hook 的启用时机与范围 | L0 先行已冻结（n=28 null 通过前不得启用 L1；09 §4.3）。L1 是否/何时进入 M4、哪些 IML=7 短例程适用、周期协议见 09 §7 S3。oracle 影响：L0 期间 trace/`pcm_trace` 可作逐切片 gate；启用 L1 后只能靠音频 null + 状态标量（本文 n=28/wav 工具链须先行落地） |

### 6.2 待办（需用户）

1. **Q1 MIDI 素材来源（阻塞压力测试）**：需用户提供 ≥255 同时音的 SMF（或指定曲目、
   或允许自制密集 chords SMF）；确认后由 `midisched` 转 schedule，放
   `mk2cpp/out/m4/corpus/`（不入 git）。**当前待提供**。
2. **Q2 听感验收安排（阻塞结论）**：n=28 A/B 与 255 压力需要在场听/看；需约定时间与
   运行次数（建议每级 2 次：一次自动对比 + 一次人工听感）。
3. **G6 音频 baseline 冻结**：已在 stock 可信期跑完并冻结于
   `tools/baselines/m4_audio/`（`stock_300M_320M.wav/.wav.meta/.audiohash/.state.hash`
   + `SHA256SUMS.txt`；2026-09-11，本地 fixture、gitignore）。
   `m4_audio_null.ps1 -CheckBaseline` 已接入：SHA256SUMS 不符=FAIL，
   本次 stock 的 WAV payload/`-audiohash` 必须与基准一致；基准缺失=SKIP。
   改动 stock PCM/ROM 路径或音频 tap 后需重冻（用户在场）并更新 SHA256SUMS。
4. **wav 窗口/时长**：默认自动对照 `[300M,320M)`、听感可 `[300M,400M)`；是否要更长
   （例如完整 demo 段）；是否接受 `-audiowin`（08 只提了 `-wav:`/`-audiohash`，见 §2.2）。
5. **09 §7 S1–S6**（`-voices:n` 单独行为、引擎状态归属、L1 范围/周期协议、T 检查、
   `-mk2cpp-stats`、hand 版本门禁）：直接影响 M4 oracle 的时机与 trace 可用性。

---

## 附录 A：已读来源（file:line 索引）

- CLI/help：`src/mcu.cpp:1946-2177`（`-mk2cpp`:1977、`-mk2`:1984、`-voices`:1993、
  `-demo`:2006、`-mocknote`:2012、`-tracepc`:2018、`-pcmtrace`:2027、`-savesnap`:2031、
  `-hashdump`:2035、`-loadsnap`:2047、`-gain`:2051、help:2143-2164）。
- trace：`src/mcu.cpp:226-252`（格式）、`:1459-1478`（开/关窗）。
- hashdump：`src/mcu.cpp:1306-1404`（v1 字段 `:1341-1381`）。
- pcmtrace：`src/mcu.cpp:934-946`；ext 分支 `:949-952`。
- 音频：`src/pcm.cpp:584-703`（混音/PCM_Update）、`src/mcu.cpp:1841-1861`
  （MCU_PostSample）、`:1734-1741`（callback）、`:1771-1833`（OpenAudio，率 `:1780`）。
- 扩展语义：`src/pcm.cpp:586-587`（cfg3d→reg_slots；方案 A 后扩展取 `pcm_ext_voices`，config 恒 0x7b，out/m4/12）、`src/pcm.h:27-29`（常量）、
  `src/pcm.h:31-61`（pcm_t）、`src/pcm.h:75-76`（WriteExt/ReadExt）、
  `src/mcu.cpp:1658-1668`（MCU_PatchROM 占位）。
- 常量：`src/mcu.h:440`（`pcm_ext_voices`）、`:445`（`uart_buffer_size=8192`）。
- MIDI 入径：`src/midi_win32.cpp:51-59`（`MCU_PostUART`）。
- 脚本：`mk2cpp/tests/two_mode_check.ps1`（headless env `:70-72`、轮询 `:198-300`、
  比较 `:362-422`）、`mk2cpp/tests/README.md:26-90`。
- 基线：`tools/baselines/README.md:110-116`。
- 约定：`mk2cpp/README.md:56-83`（规则 1-4、M4 行 `:81`）、`mk2cpp/docs/00_plan.md` §M4
  （M4 验收）、`mk2cpp/docs/02_conventions.md:37-46`（产物与 git）。
- 铁律/时间窗：`tools/docs/plan_256.md:89-102`；
  `tools/docs/polyphony_256_todo.md:167-169,209-215`。
- O1–O10：`tools/docs/task_irq_map.md:488-508`（O2/O3/O10 采集依赖的 `isr` 与当前源码
  漂移见 `:20,125` vs 本仓库 grep 结果；`-pcmtrace` 扩展项 `:483-484`）。
- 配套设计：[07 voice 语义](07_m4_voice_spec.md)（例程闭包、双轨状态、
  活性上限 255 §0.5）、[08 PCM 引擎与音频路径](08_m4_pcm_api.md)（PCM 引擎地图、
  `-wav:` §3.5、容量 256 R1、`cfg3d` bit5 R2、音频链路 §3）、
  [09 hand 覆盖表与集成](09_m4_integration.md)（覆盖表、激活矩阵 §3、每切片 gate §5.3、
  待定项 §7）、[00 计划](00_plan.md) §M4。

## 附录 B：实现边界

- 本文为验收设计规格；脚本默认 dry-run，真实执行路径的 GT 行为需在 G1/G3/G5 落地后验证；
- `pcmdiff`/`midisched` 仅给出接口与职责，实现属后续任务；
- 所有真实 GT 运行仍遵守铁律：LCD 窗口 + 声音 + 超时 kill + 用户在场。
