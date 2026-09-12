# mk2cpp/tests

2026-09-13 按现有三个 Python 脚本及 GT 源码核对。命令从仓库根执行，使用自己的
Python 3（仅标准库，不提交嵌入式解释器）。本机解释器与素材设置见本地 `local.md`；
ROM、schedule、基线和运行输出不入库。

## 当前测试与里程碑边界

**目前 `-mk2cpp` 仍运行 M1–M4 的逐指令翻译/混合回退。** M4 已保留为整理与回归
检查点，M4.5 原生控制分支待实现；这些脚本通过不代表原生算法重建已完成。

M4.5 继续用同一 flag 进入原生分支，复用 PCM 仿真、LCD、RtMidi、SDL 等设施；
不加 flag 维持原模拟器。目标和验收见 [13 M4.5 规格](../docs/13_m45_native_reconstruction.md)。
现有逐 PC trace/寄存器/旧 SRAM 对照属于参考与迁移层；原生后端需要逻辑状态、事件
时序、音频、默认分支回归及运行依赖审计，不能被强制要求复现 H8 指令轨迹。
拟新增的算法/设备/事件验证按 [14 §8](../docs/14_m45_native_architecture.md#8-验证架构与完成证据)
与 [15](../docs/15_m45_construction_guide.md) 实施；现有脚本不因此自动升级为原生验收工具。

**新增目标门禁（待实现）**：原生全生命周期零原 ROM 依赖，未修改原厂 bank 的输出逐 bit
一致。原素材离线导入 bank；GT 使用原 ROM，native 在原 ROM 不可访问的独立环境中
使用 bank。先验证资产表值、引用与波形读取等价，再比较同时间原点/窗口、同格式和
帧数的 PCM payload（既有增益/限幅后、SDL 转换前），不得自动移位、重采样或放宽容差。
执行与文件访问诊断还需证明无隐藏固件/ROM 回退；完整条件见
[14 §8.1](../docs/14_m45_native_architecture.md#81-零原-rom-与-bit-级一致的验收)。
原整数/`-float` 模式各自与同配置参考比较。以下旧脚本的命令、整 WAV/hash 与 PC 门禁
保持原行为，不因目标改变就视为已经满足新门禁。

**透明资产功能门禁（待实现）**：按 [16](../docs/16_transparent_bank.md) 验证公开目录、
WAV 独立试听、编辑/替换实际参与发声、兼容数据与缓存失效、音高/循环/控制和保存恢复。
用户修改后的素材不与原厂音频要求 bit 相同；未修改原厂 gate 和默认分支回归持续保留。
只证明 WAV 被打开或音频哈希变化不足以证明替换正确。现有三个脚本尚未覆盖这些能力。

| 现有脚本 | 用途 | 默认是否运行仿真 | SDL 模式 |
|---|---|---|---|
| [m4_quick_gate.py](m4_quick_gate.py) | 指定回归曲，两模式并行，整 WAV 文件 SHA-256 | 是 | dummy 音频/视频，无需用户在场 |
| [two_mode_check.py](two_mode_check.py) | boot/demo200 或自定义窗口；两模式依次比较 trace 和完整 hash 文件 | 是 | dummy 音频/视频，无需用户在场 |
| [m4_audio_null.py](m4_audio_null.py) | stock 28 音频窗口、选定状态标量、可选 hand A/B 与冻结基线 | 否；默认只做规划及 GT `-h` 探测 | `--execute --user-present` 才运行真实 LCD/音频 |

`m4_audio_null.py` **仍存在且可用**。旧 `.ps1` 测试入口已经删除；
`m4_stress_voices.ps1` 也已删除，M5 重启时按原生架构以 Python 补齐容量测试。

## 默认回归范围与预算

- 外部 MIDI 默认只测指定回归曲，不默认或半默认扩展到其他曲子。通常运行
  `python3 mk2cpp/tests/m4_quick_gate.py`，历史墙钟约 65 秒，实际以本次输出为准。
- 主链路/非 voice 路径改动时，再按需运行 `two_mode_check.py --scenario boot`
  或 `--scenario demo200`；文档编辑不需要启动音频回归。
- `m4_quick_gate.py --demo` 默认推进到 1440M cycles，即 reset 后 60 秒模拟时间；
  录音窗口是 `[144M,1440M)`，实际录音 **54 秒**，历史墙钟约 61 秒。
  不默认跑完整 demo 播放列表。`two_mode_check.py` 的 demo200 窗口只有 2M cycles，
  是独立的冻结 trace 场景。
- 全量曲库和容量压力矩阵仅在用户明确要求相应范围时运行，分批并汇报耗时。
- 单次前台测试预算不超过 3 分钟，预计超出时提前告知并后台/分片。
  **脚本硬超时不等于工作预算**：quick gate 默认超时为 300 秒；前台运行可显式用
  `--timeout 180`。two-mode 默认每个进程 120 秒、两模式串行，总时间可能超过 3 分钟。

三个脚本的仿真命令均带 `-nomidi`，手工 oracle 也必须带，避免 RtMidi 外部输入破坏
确定性。脚本通过 `-midiseq` 注入的指定输入仍有效。两个 headless 脚本设置
`SDL_VIDEODRIVER=dummy`、`SDL_AUDIODRIVER=dummy`、`SDL_RENDER_DRIVER=software`；
audio-null 的真实运行会移除环境中的前两个变量，启用真实窗口与音频。

## m4_quick_gate.py

```bash
# 指定回归曲；显式限制本次等待预算
python3 mk2cpp/tests/m4_quick_gate.py --timeout 180

# 内置 demo，reset 后推进到 60 秒，录音 54 秒
python3 mk2cpp/tests/m4_quick_gate.py --demo --timeout 180
```

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--sched` | `mk2cpp/out/m4/corpus/regression.sched` | 指定回归曲 schedule；其他素材只在明确调试范围内使用 |
| `--start` | `200000000` | MIDI 注入及录音起点 |
| `--demo` | 关闭 | 用内置 demo 替代外部 schedule |
| `--demo-start` / `--demo-end` | `144000000` / `1440000000` | demo 录音窗口 |
| `--timeout` | `300` | 两个进程启动后共用的等待截止时间，单位秒；不是每个模式再各等 300 秒 |
| `--exe` | `build/nuked-sc55[.exe]` | GT 可执行文件 |
| `--outdir` | `mk2cpp/out/quick_gate` | WAV、meta 与日志目录 |

MIDI 录音终点为 `start + schedule 最后一个有效行的 cycle + 2000000`；这不是“等所有
release 声部结束”。比较的是当前窗口生成的整 WAV 文件（含文件头），通过条件为
两个模式均取得非空 `.meta` 完成标记且非空 WAV 的 SHA-256 相同。
脚本不比较 PC trace、状态 hash 或冻结 WAV 基线，也不执行人工试听。

输出为 `stock.wav`、`tr.wav`、各自 `.wav.meta` 和 stdout/stderr 日志，结果打印到终端。
完成或超时后用 Python `proc.kill()` 结束仍在运行的进程。

## two_mode_check.py

```bash
# boot [0,3M)
python3 mk2cpp/tests/two_mode_check.py

# demo [200M,202M)
python3 mk2cpp/tests/two_mode_check.py --scenario demo200

# 自定义窗口：脚本参数必须放在 --args 之前
python3 mk2cpp/tests/two_mode_check.py --scenario custom \
    --window-from 0 --window-to 1000000 --hash-at 1000000 --args -demo
```

`--args` 使用 `argparse.REMAINDER`：**其后所有 token 都直接交给 GT**。如果把
`--window-to` 等脚本参数放在它后面，脚本就收不到这些设置。只在 custom 场景使用它。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--scenario` | `boot` | `boot`、`demo200` 或 `custom` |
| `--window-from` / `--window-to` | `0` / `0` | custom 必填合法窗口，终点大于起点 |
| `--hash-at` | `0` | custom 状态采样 cycle，0 表示窗口终点 |
| `--args` | 空 | custom 的附加 GT 参数，必须位于脚本参数末尾 |
| `--timeout` | `120` | 每个模式的等待超时，秒；两模式串行 |
| `--exe` | `build/nuked-sc55[.exe]` | GT 可执行文件 |
| `--outdir` | `mk2cpp/out/twomode` | trace、hash 和日志目录 |

boot 固定 `[0,3000000)` / hash@3000000；demo200 固定
`[200000000,202000000)` / hash@202000000。窗口和 hash 参数仅用于 custom。

脚本比较：

- `def.trace` 与 `tr.mk2cpp.trace` 的整文件 SHA-256；
- `def.hash` 与 `tr.mk2cpp.hash` 的整文件 SHA-256；
- 冻结场景的默认 trace 与 `tools/baselines` 对应 fixture；fixture 不存在则该项 SKIP。

输出还包括 `def.stdout.txt` / `def.stderr.txt` 与对应的 `tr.mk2cpp` 日志。
等待条件是非空 hash，加上 trace 尾部 cycle 达到终点附近，或非空 trace 连续四次
轮询大小不变。尾行解析接受 `m` 和 `s`；不是“必须最后一条主 MCU trace 精确到终点”。
取得输出或超时后结束 GT 进程。

当前统一 trace 含子 MCU 行，完整状态 hash 也覆盖 SM/LCD 状态。通过只证明所测
配置和窗口的一致性，不能单凭它证明所有 SM/面板行为、全执行面覆盖或 M4.5 已完成。

## m4_audio_null.py

默认不运行仿真或试听，但**会启动 GT `-h` 子进程做能力探测**（最多等待 30 秒），
校验参数并写 `plan.txt`。路径合法时，缺少必需选项只打印错误；加 `--require-ready`
才因此退出 2。dry-run 成功不是音频 PASS。

真实仿真必须同时带 `--execute --user-present`；这两个参数只适用于本脚本。
自动检查通过后仍需用户完成 LCD 观察和 A/B 试听，脚本的绿色摘要不能代替人工验收。

```bash
# 默认规划：只运行 GT -h，不播放
python3 mk2cpp/tests/m4_audio_null.py

# 本地 GT 构建能力检查
python3 mk2cpp/tests/m4_audio_null.py --require-ready

# 用户在场：真实 LCD + 音频，stock 与 M4 依次运行
python3 mk2cpp/tests/m4_audio_null.py --execute --user-present

# 加入本地冻结基线比较
python3 mk2cpp/tests/m4_audio_null.py --check-baseline --execute --user-present

# 指定 MIDI 场景与 hand 开关 A/B
python3 mk2cpp/tests/m4_audio_null.py --scenario midi \
    --midi-schedule mk2cpp/out/m4/corpus/regression.sched --hand-off \
    --execute --user-present
```

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--scenario` | `demo` | `demo` 或 `midi`，后者用固定 200M 起点注入 |
| `--audio-start` / `--audio-end` | `300000000` / `320000000` | WAV 录音窗口 |
| `--state-at` | `0` | 0 表示 audio-end；必须至少 200M |
| `--timeout` | `0` | 正数覆盖每个模式超时；否则 `max(10, ceil(audio_end / 24e6 * 2 + 15))` 秒 |
| `--outdir` | `mk2cpp/out/m4/audio_null` | 计划、采集与比较结果 |
| `--midi-schedule` | 空 | midi 场景必需 |
| `--gain` | 空 | 两模式使用相同的 `-gain:<值>` |
| `--check-baseline` | 关闭 | 增加冻结 stock 音频基线比较 |
| `--baseline-dir` | `tools/baselines/m4_audio` | 本地基线目录 |
| `--baseline-name` | `stock_300M_320M` | 冻结文件名前缀 |
| `--hand-off` | 关闭 | 增加 `-mk2cpp-hand:0` 第三个运行模式 |
| `--require-ready` | 关闭 | dry-run 在缺少必需 GT 选项时退出 2 |
| `--execute` / `--user-present` | 关闭 | 两者同时给出才允许真实仿真 |
| `--exe` | `build/nuked-sc55[.exe]` | GT 可执行文件 |

默认 320M 终点对应每模式 42 秒硬超时；WAV 窗口长度为 20M cycles，约 0.833 秒。
`--state-at` 若晚于 audio-end，默认超时仍按 audio-end 计算，应自行安排足够的超时。
当前脚本的 GT 工作目录固定为仓库 `build/`，仅指定 `--exe` 不会改变它。

完成条件为 WAV/meta/audiohash/state 文件就绪，随后结束进程。自动检查包括：

- `audio_fnv1a`（到 audio-end 的累计输出流），以及窗口内 WAV payload 的 SHA-256；
  WAV 通过初步解析但 payload hash 未能匹配时，可调用已构建的 `pcmdiff --tolerance 0`。
- meta 的起止 cycle、采样率、通道、格式与 sample_pairs；两边均有 payload FNV 时比较它。
- `SCALAR_KEYS`：`mcu.cp/pc/sr/cycles`、`pcm.config_reg_3c/3d`、`pcm.irq_assert`。
  **这里只比较选定标量，不比较完整 state.hash 文件**。
- 日志中 `LCDEN 0` 出现不超过两次的复位循环启发式；这不是 LCD 图像等价检查。
- 可选 hand-off 的 audiohash/WAV/状态标量对照；CPU duty 仅作信息，标准库
  `subprocess.Popen` 无 `cpu_times()`，当前会显示 n/a，不能当作性能测量结果。

标量/meta 比较使用字典取值；两边同时缺少某个字段可能不会触发差异，因此 PASS
不能代替输出格式完整性检查。WAV 比较也不是完整的 RIFF 格式校验器。

基线检查遵循实现的两层处理：

- `SHA256SUMS.txt` 中可解析条目所列文件缺失或哈希错误，`baseline:integrity` 为 FAIL。
- 基线目录、manifest 缺失/不可读或没有可解析条目，该组检查 SKIP。
- 冻结 WAV 或 audiohash 缺失/不可解析，对应比较项可 SKIP；若它同时是 manifest
  中的条目，前面的 integrity FAIL 仍保留。冻结输出与当前 stock 不同则 FAIL。

输出包括 `plan.txt`、`summary.txt`、`stock`/`m4`/可选 `handoff` 的 WAV/meta、
`.audiohash`、`.state.hash` 和日志，必要时另有 `audio_diff*.md`。
SKIP 不是 PASS；返回 0 也不代表冻结基线或人工试听已经验收。

## 当前 GT 接口与历史扩展

[GT 源码](../../src/mcu.cpp) 已实现并在帮助中列出 `-wav:`、`-audiowin`、
`-audiohash`、`-midiseq`、`-snapinfo` 和 `-mk2cpp-hand:0|1`，并非仍待 Wave 0 实现。
本地二进制可能落后于源码，audio-null 的能力探测仍有用途；探测到帮助文本不等于
执行行为已经测试通过。

- WAV 在窗口结束时回填长度并关闭，最后写 `.meta` 完成标记；强杀进程不负责完成文件头。
- `-audiohash` 是指定 cycle 之前输出流的累计 FNV-1a64；`-audiowin` 只限制 WAV 窗口。
- `-midiseq` 以相对 start 的 cycle 注入字节，经过 UART 环形缓冲及背压。
- `-snapinfo` 现有心跳、MCU/PCM 标量与 voice mask popcount；**没有 `pcm.ext_voices`**。
- 当前三个脚本都不验证已回滚的 `PCM_WriteExt`/扩展寄存器 trace 或 256 容量。

`-voices:` 容量接口已回滚。audio-null 当前仍给翻译模式附带历史 `-voices:28` 参数，
当前 GT 没有对应容量处理，忽略它后仍跑 stock 28；这不是可用的复音选择接口。
255 是旧实验的哨兵妥协，不是当前能力或新的验收上限。M5 目标是 **256 个可同时
活跃的底层发声单元**，在 M4.5 之后于既有 PCM 仿真基础上扩容，保持默认分支行为。
旧矩阵仅作历史研究，见 [10](../docs/10_m4_oracle.md)；新路线以 [00](../docs/00_plan.md)
与 [13](../docs/13_m45_native_reconstruction.md) 为准。

M4 的既有音频 null、状态产物和回归记录见 [12 §1.2](../docs/12_polyphony_reassessment.md)。
不能再写成“只有 dry-run，从未运行”；也不能把历史记录算作当前修改后自动重跑的结果。

## MIDI 素材与运行记录

[midisched](../tools/README.md) 把 SMF 转成文本 schedule，输出放 `mk2cpp/out/`。
cycle 相对 `-midiseq` 的 start；默认字节间隔为 7680 cycles（320 微秒）。
它处理 MIDI 串行化；GT 的缓冲背压仍需保留，单靠转换器不能证明所有负载无溢出。
指定素材身份与本机路径留在 `local.md`，不要写入公开文档。

记录每次实际使用的代码/二进制版本、ROM/PCM 配置、输入、窗口、命令、自动检查
及人工验收结果。窗口比较通过不证明原生后端已经接管：当前调度优先 hand，其次 gen，
最后解释器回退（见 [mk2cpp.cpp](../src/mk2cpp.cpp)）。缺少 gen 不代表完全没有 hand；
单看版本字符串或相同音频不能判断实际执行了多少翻译代码，必须结合注册/命中证据。

## 退出码

| 脚本 | 0 | 1 | 2 |
|---|---|---|---|
| quick gate | 输出就绪且整 WAV SHA-256 相同 | 超时/输出缺失/音频不同 | 显式检查到 exe 或 schedule 缺失、参数解析错误 |
| two-mode | 所有已执行比较无 FAIL，缺基线可 SKIP | 运行或 trace/hash/基线比较失败 | 显式检查到 exe 缺失、custom 窗口不合法、参数解析错误 |
| audio-null | dry-run 成功，或自动检查无 FAIL（可含 SKIP） | 自动检查有 FAIL | 参数/路径检查失败、必需能力缺失的门禁或用户在场门禁未满足 |

这是各脚本正常处理路径的退出约定；未捕获的 I/O/子进程异常不保证返回 2。
Python 参数统一为 `--check-baseline`、`--require-ready`、`--execute` 等双横线小写写法；
没有旧 PowerShell 的 `-KeepGoing`、`-CheckBaseline` 或 `-VoiceLevels` 参数。
