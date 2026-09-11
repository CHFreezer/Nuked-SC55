# 08 M4 PCM 引擎与音频路径规格

状态：已评审 v1，2026-09-11。
配套文档：[07 voice 语义](07_m4_voice_spec.md) · [09 hand 覆盖表与集成](09_m4_integration.md) · [10 验收 oracle](10_m4_oracle.md) · [00 计划](00_plan.md)。
阅读顺序：00_plan §M4 → 07 → **08（PCM/音频）** → 09 → 10。
口径：验收统一 `-voices:255`（"256 复音"指容量）；原生引擎在 `-mk2cpp` 下默认参与（含 n=28），`pcm_ext_active=1` 仅当 `-mk2cpp && -voices:n` 且 n≠28；前期 L0（一次 `MK2CPP_Step` = 一条 H8 指令）。

范围：GT（`src/`）PCM 引擎地图 + 阶段 1 扩展点 + 音频输出链路 + `mk2cpp/src/hand/` 原生引擎 API。
行号口径：评审基线 HEAD `84d3e51`（已含阶段 1 PCM 扩容）；引用行号在实现推进后可能漂移。旧文档（如 `mk2_polyphony_256.md` 的 `pcm.cpp:536/:1111/:1520`）行号已漂移，本文一律以评审基线重新核对。
证据等级：沿用 `tools/docs/evidence_protocol.md` §12（C=Confirmed / S=Strongly supported / I=Inferred）。本文凡"语义命名"未在源码注释中出现的，按 S/I 标注。
配套：`tools/docs/plan_256.md`、`tools/docs/polyphony_256_todo.md` 阶段 1、`mk2cpp/docs/00_plan.md` §M4、`mk2cpp/docs/04_cosim_spec.md` §5.4。

## 0. 结论摘要（先读）

1. **GT 的 PCM 引擎是"寄存器阵列 + 单采样步固定点 DSP"**：固件经 `br=0xe000` 窗口读写 0..0x3f 寄存器（`PCM_Write/PCM_Read`），每步 `PCM_Update` 依次跑效果固定块 → 逐 voice DSP → 输出。没有第二条 per-voice 迭代路径（`src/pcm.cpp:584-1677`）。
2. **寄存器到内部数组的映射可以精确给出**：写解码见 `src/pcm.cpp:79-204`，读解码见 `:210-283`；`ram1[slot][0..5]`/`ram2[slot][0..11]` 全部 12+6 个可达索引都有明确用途，`ram1[6..7]`/`ram2[12..15]` 固件不可达。
3. **66207 Hz 不是硬编码**：PCM 内步固定推进 725 master cycles（`(28+1)*25`，`src/pcm.cpp:1672-1675`）；24 MHz / 725 = 33103.4 步/s；`config_reg_3c` bit6（oversampling，启动值 0xc3）使每步调两次 `MCU_PostSample`（`:690-702`）→ 66206.9 ≈ 66207 Hz（`src/mcu.cpp:1780`）。阶段 1 的 `cycles` 钳 28（`:1672`）就是为保住这个节奏。
4. **阶段 1 的默认不变性靠"重定位等价"**：效果行 28..31 在所有模式下都搬到 `PCM_EFF_BASE=256..259`（`src/pcm.cpp:141-145`、`:38-41`），数组扩大到 260 槽；默认路径的寄存器语义、DSP 索引、`pcm_t` 布局在阶段 1 后冻结。默认 200M-202M trace 逐字节一致、EFF=256 vs 28 快照强等价（`polyphony_256_todo.md:173-183`）。
5. **音频链路没有重采样、没有 wav/dump**：work_thread 产 int16 立体声帧 → `sample_buffer` 环形缓冲 → SDL 回调按 66207 Hz 消费（`src/mcu.cpp:1734-1741,1841-1861`）；回溯为空缓冲自旋（`:1430-1442`）。现有 CLI 只有 `-a:/ -ab:/ -gain:/ -float`，无任何录音能力（grep `argv`，`src/mcu.cpp:1946-2177`）。
6. **最小音频抓取方案可行且低风险**：在 `MCU_PostSample` 钳位之后加 `-wav:<file>` 生产者侧 tap（增益/钳位后的真实 SDL 数据），默认零行为变化；n=28 null 测试即比对两个 WAV 的 PCM payload（sha256）。可行性见 §6。
7. **M4 原生引擎分三步**（与 00_plan §M4 一致）：M4a 固件控制语义化（寄存器写→typed API）但仍用 GT DSP；M4b 逐行机械移植 DSP（保 20 位量化/`nfs`/浮点分支）并用 wav tap 做逐样本 diff；M4c 255 优化（跳过 inactive voice、可选并行）。可观测契约是 `pcm_t` 全量镜像（hash/snapshot）+ `MCU_PostSample` 字节流（音频）+ `-pcmtrace`（控制写）。
8. **命名口径与设计冲突**：① 验收统一 `-voices:255`（"256 复音"指容量；阶段 1 `PCM_MAX_VOICE=255`，voice 0..254，效果 256..259，数组 260；n=256 需数组 ≥264/EFF 260..263，是否放开见待定项）。② `config_reg_3d` 低 5 位当 voice 数、bit5 当波形 ROM bank 模式（`src/pcm.cpp:46-49`），扩展公式 `config+1` 会撞 bit5——**已解决（方案 A，2026-09-11，out/m4/12）**：扩展模式 `reg_slots = pcm_ext_active ? pcm_ext_voices : ((config&31)+1)`，`config_reg_3d` 保持 stock `0x7b`，诊断由 `-snapinfo` 新增 `pcm.ext_voices`，`-hashdump` 格式保持不变（见 §7 P1）。

---

## 1. GT PCM 引擎地图

### 1.1 `pcm_t` 字段（`src/pcm.h:31-61`）

| 字段 | 类型 | 作用 | 证据 |
|---|---|---|---|
| `ram1[PCM_SLOTS][8]` | u32×8 | per-slot 20 位参数/状态：地址、循环、滤波状态、DPCM 参考 | `pcm.h:32`；消费 `pcm.cpp:1182-1435` |
| `ram2[PCM_SLOTS][16]` | u16×16 | per-slot 控制/包络/相位/pan/send 等 | `pcm.h:33`；消费 `pcm.cpp:1168-1660` |
| `select_channel` | u32 | 当前寄存器目标通道（固件 `br+0x3e` 写）；库存 0..27 + 28..31→EFF 重映射；扩展 full byte | `pcm.cpp:131-152` |
| `voice_mask[32]` | u8×32 | 已提交 voice enable mask，bit N=voice N | `pcm.h:36`；`pcm.cpp:215-220` |
| `voice_mask_pending[32]` | u8×32 | 待发 mask（写 0x00-0x03 / ext 时更新） | `pcm.cpp:82-103,288-296` |
| `voice_mask_updating` | u32 | pending≠committed 标志；status bit5；读地址<4 触发提交 | `pcm.cpp:215-220,234-235` |
| `write_latch` | u32 | `br+0x04..0x2f` 3 字节写锁存值 | `pcm.cpp:153-181` |
| `wave_read_address` / `wave_byte_latch` | u32/u8 | 波形 ROM 读地址（3 字节）与读锁存（0x3f 读回） | `pcm.cpp:104-121,239-241` |
| `read_latch` | u32 | `ram1/ram2` 读回锁存（`br+0x39..0x3b` 分 4/8/8 位读出） | `pcm.cpp:243-280` |
| `config_reg_3c` | u8 | 输出位宽/噪声/DAC 模式/Oversampling/写回使能 | `pcm.cpp:123-126,600-652` |
| `config_reg_3d` | u8 | 低 5 位=voice 数−1（仅库存公式）；bit5=波形 ROM bank 模式（>>21 vs >>19）；扩展模式只作 ROM bank 镜像，voice 数取 `pcm_ext_voices`（方案 A，out/m4/12） | `pcm.cpp:127-130,46-49,587` |
| `irq_channel` / `irq_assert` | u32/u32 | 最近触发 IRQ 的 slot / 待应答标志 | `pcm.cpp:1527-1538,221-238` |
| `nfs` | u32 | "非首样本"门控：首步不写回地址/包络状态；跑完一步置 1 | `pcm.cpp:706,1099-1103,1223-1227,1356-1363,1668` |
| `tv_counter` | u32 | 14 位全局包络时钟，每步 −1；`!nfs` 时从 `ram2[EFF+3][8]` 装载 | `pcm.cpp:705-712` |
| `cycles` | u64 | PCM 自己的采样时钟，按 725/步推进 | `pcm.cpp:591,1675` |
| `eram[0x4000]` | u16×16384 | 效果延迟 RAM（每字 14 位+2 位指数） | `pcm.h:54`；`pcm.cpp:552-582` |
| `accum_l/accum_r` | int | 上一步 voice 链末尾的 20 位总线结果 | `pcm.cpp:1633-1634`；消费 `:660-661` |
| `rcsum[2]` | int×2 | 效果送回（reverb/chorus send 累加） | `pcm.cpp:1601-1624`；消费 `:731-750` |
| `fstate[PCM_SLOTS][2]` | float×2 | `-float` 谐波滤波器状态（state1/state2） | `pcm.h:60`；`pcm.cpp:1440-1475` |

> 说明：`PCM_SLOTS=260`、`PCM_EFF_BASE=256`、`PCM_MAX_VOICE=255`（`pcm.h:27-29`）。效果固定占 256..259；voice 0..254。`sizeof(pcm)` 约 51.5 KB，`hashdump`/`state_save` 按整结构原始字节处理（`mcu.cpp:1262,1302,1368`）。

### 1.2 寄存器接口：地址解码（写 `src/pcm.cpp:79-204`，读 `:210-283`）

#### 1.2.1 写

| `br+off` | 行为 | 证据 |
|---|---|---|
| 0x00 | voice enable voices 24..31（库存只允许低 nibble 24..27；`pcm_ext_enabled` 时允许全字节 24..31） | `pcm.cpp:82-90` |
| 0x01 | voice enable voices 16..23 | `pcm.cpp:92-94` |
| 0x02 | voice enable voices 8..15 | `pcm.cpp:95-97` |
| 0x03 | voice enable voices 0..7 | `pcm.cpp:98-101` |
| 0x04-0x07 | `ram1[sel][4]` 3 字节写（0x04 高 4 位、0x05、0x06）；0x07 提交 | `pcm.cpp:153-181` |
| 0x08-0x0b | `ram1[sel][2]`；0x0b 提交 | 同上（ix 位序见下） |
| 0x0c-0x0f | `ram1[sel][0]`；0x0f 提交 | 同上 |
| 0x10/0x11 | `ram2[sel][0]`（高/低字节，0x11 提交） | `pcm.cpp:183-204` |
| 0x12/0x13 | `ram2[sel][1]` | 同上 |
| 0x14/0x15 | `ram2[sel][2]` | 同上 |
| 0x16/0x17 | `ram2[sel][3]` | 同上 |
| 0x18/0x19 | `ram2[sel][4]` | 同上 |
| 0x1a/0x1b | `ram2[sel][5]` | 同上 |
| 0x1c/0x1d | `ram2[sel][6]` | 同上 |
| 0x1e/0x1f | `ram2[sel][7]` | 同上 |
| 0x20-0x23 | 波形 ROM 读地址 3 字节写；0x23 提交并锁存 ROM 字节 | `pcm.cpp:104-121` |
| 0x24-0x27 | `ram1[sel][5]`；0x27 提交 | `pcm.cpp:153-181` |
| 0x28-0x2b | `ram1[sel][3]`；0x2b 提交 | 同上 |
| 0x2c-0x2f | `ram1[sel][1]`；0x2f 提交 | 同上 |
| 0x30/0x31 | `ram2[sel][8]` | `pcm.cpp:183-204` |
| 0x32/0x33 | `ram2[sel][9]` | 同上 |
| 0x34/0x35 | `ram2[sel][10]` | 同上 |
| 0x36/0x37 | `ram2[sel][11]` | 同上 |
| 0x3c | `config_reg_3c = data` | `pcm.cpp:123-126` |
| 0x3d | `config_reg_3d = data` | `pcm.cpp:127-130` |
| 0x3e | `select_channel`：库存 `&0x1f`，28..31→`EFF_BASE+0..3`；扩展（`pcm_ext_active`）full byte 0..255 | `pcm.cpp:131-146` |
| 0x3f | 库存：进入分支但不改任何状态（忽略）；扩展：效果选择别名 `EFF_BASE+(v&3)` | `pcm.cpp:147-152` |
| 0x38/0x3a/0x3b | 无写分支（读锁存用） | — |

**ram1 的 ix 位序**（`pcm.cpp:171-179`）：`ix = (off&0x20?1:0) | ((off&0x08)==0?4:0) | ((off&0x04)==0?2:0)`，在 `off&3==3` 时提交。由此得 `0x04→4, 0x08→2, 0x0c→0, 0x24→5, 0x28→3, 0x2c→1`。
**ram2 的 ix 位序**（`pcm.cpp:198-201`）：`ix = (off>>1)&7`，`off&0x20` 时 `|8`，在奇地址提交。由此得 `0x10→0, 0x12→1, …, 0x1e→7, 0x30→8, …, 0x36→11`。

三条不可忽略的副作用：
- 写 `0x00-0x03` 只改 **pending**；读任何 `off<4`（固件 `movl @(br,$00)` 回读）才把 pending 拷进 `voice_mask`（`pcm.cpp:215-220`）。即 **提交点=读回**。
- 读 `0x3c/0x3e` 返回 status（`irq_channel&0x1f | updating<<5`，扩展模式 IRQ 完整值走 `PCM_ReadExt(0x20)`）；读 `0x3e` 且 `irq_assert` 时清 IRQ 并向 CPU 撤中断（`pcm.cpp:221-238`）。
- 读 `0x3f` 返回 `wave_byte_latch`（库存；扩展模式读仍如此，写才变成效果别名）（`pcm.cpp:239-241,147-152`）。

#### 1.2.2 读

`off` 0x04-0x0f/0x24-0x2f 在 `(off&3)==1` 时把 `ram1[sel][ix]` 读进 `read_latch`；0x10-0x1f/0x30-0x37 在偶地址把 `ram2[sel][ix]` 读进 `read_latch`；`0x39/0x3a/0x3b` 分别返回 `read_latch` 的高 4/中 8/低 8 位（`pcm.cpp:243-280`）。0x3f 返回波形字节锁存（见上）。

### 1.3 per-slot `ram1/ram2` 语义（结合 DSP 使用点与固件镜像）

> 下表"固件镜像"列来自 `tools/docs/voice_memory_map.md` §4.1（P−N 字段）与本工作树解码核对；`P-0x18=br+1a`、`P-0x16=br+36` 的旧猜测按本表修正（旧文标 "pitch?"，实际是 TV adjust / TV 系数）。

| 寄存器 | 内部 | 语义 | 证据 |
|---|---|---|---|
| br+04..07 | `ram1[slot][4]` | **当前波形地址指针**（20 位，配 `ram2[7]` bit8-11 hiaddr） | 读 `pcm.cpp:1182`、回写 `:1357` |
| br+08..0b | `ram1[slot][2]` | **循环地址** address_loop | `pcm.cpp:1184,1186,1245` |
| br+0c..0f | `ram1[slot][0]` | **结束地址** address_end | `pcm.cpp:1183,1186` |
| br+10/11 | `ram2[slot][0]` | **音高步进**（14 位小数相位增量；由 `ram2[7]&31` 选"调制源 slot"，取该源的本寄存器） | `pcm.cpp:1097,1219-1221` |
| br+12/13 | `ram2[slot][1]` | **pan**：高字节 L、低字节 R（`active` 时生效，否则 0） | `pcm.cpp:1563,1566-1567` |
| br+14/15 | `ram2[slot][2]` | **效果送出**：高字节→`rc0`（注释 "reverb"）、低字节→`rc1`（注释 "chorus"） | `pcm.cpp:1564,1569-1570` |
| br+16/17 | `ram2[slot][3]` | **包络 1 adjust**（低字节 speed、高字节 target），`calc_tv(0)` | `pcm.cpp:1543,388-550` |
| br+18/19 | `ram2[slot][4]` | **包络 2 adjust**，`calc_tv(1)` | `pcm.cpp:1544` |
| br+1a/1b | `ram2[slot][5]` | **TV 滤波 adjust**，`calc_tv(2)` | `pcm.cpp:1545,1437` |
| br+1c/1d | `ram2[slot][6]` | bit0=IRQ 使能；bit1=输出选择（0=低通 `ram1[3]`，1=残差 `v3`）；bit8-14=B 系数（`reg2_6`） | `pcm.cpp:1527,1550,1433` |
| br+1e/1f | `ram2[slot][7]` | bit0-4=**调制源 slot**；bit5=**key on**；bit6=**loop 模式 b6**；bit7=**反向播放 b7**；bit8-11=**波形 bank（hiaddr）**；bit12-15=old nibble | `pcm.cpp:1168-1181,1097,1213,1356-1364,1637-1644` |
| br+30/31 | `ram2[slot][8]` | bit0-13=**分数相位 sub_phase**；bit14=IRQ 已断言；bit15=循环方向 b15 | `pcm.cpp:1176,1219-1227,1527-1531` |
| br+32/33 | `ram2[slot][9]` | **包络 1 当前 level** | `calc_tv` e=0；`pcm.cpp:1543` |
| br+34/35 | `ram2[slot][10]` | **包络 2 当前 level** | `pcm.cpp:1544` |
| br+36/37 | `ram2[slot][11]` | **TV 当前系数/level**（同时是滤波器系数源：A1=`>>8`、A2=`(>>1)&127`） | `pcm.cpp:1437,1447-1451,1545` |
| br+24..27 | `ram1[slot][5]` | DPCM 参考/上一样本累加器（`reference`） | `pcm.cpp:1368-1405,1525` |
| br+28..2b | `ram1[slot][3]` | 滤波器低通状态（`ram1[3]=v1`） | `pcm.cpp:1470,1486,1508` |
| br+2c..2f | `ram1[slot][1]` | 滤波器状态 1（`ram1[1]=v5`） | `pcm.cpp:1495,1521` |

不可达索引：`ram1[6..7]`、`ram2[12..15]` 在 GT 写/读解码中无路径（`pcm.cpp:153-203,243-268`），DSP 也不消费（全文检索索引集合 `{0..5}` / `{0..11}`）。

### 1.4 `PCM_Update` 每采样步数据流（`src/pcm.cpp:584-1677`）

```
while (pcm.cycles < cycles)                       // :591
  1) 输出/主混音块                                  // :595-703
     - config_reg_3c → noise_mask/orval/write_mask   // :600-652
     - 效果槽 EFF+2：噪声 LFSR 推进；accum += EFF+2[0/1]；
       生成 tt[0/1] = ((accum+噪声) & ~write_mask) << 12；MCU_PostSample(tt)  // :655-676
     - 若 config_3c.bit6（oversampling）：再来一遍 → 第二次 MCU_PostSample  // :690-701
  2) tv_counter -= 1 (&0x3fff)；!nfs 时从 EFF+3[8] 装载 // :705-712
  3) chorus/reverb 配置：EFF+3[8] → EFF+3[9]/[10]        // :716-726
  4) 效果返回混入：EFF+3[1]×ram1[EFF+1][1]、EFF+2[1]×ram1[EFF+1][0] // :728-751
  5) 固定效果算法块 1..16 + 17..20 + 21/22/23/31      // :753-1156
  6) 清零 EFF+3[1]/[3] 与 rcsum[0/1]（本步总线起点）    // :1159-1162
  7) for slot in 0..reg_slots-1：                     // :1164-1661
       key = voice_active[slot]；okey = ram2[7].bit5；active=key&&okey  // :1168-1172
       地址生成器（b15/b6/b7/hiaddr、4 次 nibble 比较、sub_phase、IRQ flag）// :1174-1363
       DPCM 4 样本累加 reference                      // :1365-1405
       插值（interp_lut 3×128）→ test                  // :1407-1435
       20 位滤波器：float / mk1 / mk2 三实现            // :1437-1522
       IRQ 断言 + 中断请求                              // :1527-1538
       calc_tv(e=0/1/2) → volmul1/2/滤波器系数          // :1540-1545
       输出选择 ram2[6].bit1；两级音量；pan；效果送出     // :1550-1570
       效果路由（slot2 开关）与 rcsum                   // :1573-1624
       总线累加 suml/sumr → ram1[EFF+3][1]/[3]，最后 slot → accum_l/r // :1626-1634
       key 时写回 nibble/key；inactive 时清状态          // :1637-1660
  8) EFF+3[7] |= 0x20；nfs = 1                        // :1663-1668
  9) pcm.cycles += (cycle_slots+1)*25（扩展钳 28；JV880 再 *25/29）// :1670-1675
```

关键量化细节（M4 机械移植必须逐条照抄）：
- `addclip20`：20 位加 + 进位入 + **饱和到 0x7ffff/0x80000**（`pcm.cpp:315-323`）。
- `multi`：27 位乘后回绕截断（`pcm.cpp:325-338`）。
- `calc_tv`（包络/TV 状态机）：type 由 speed 位组合决定；`nfs` 门控写回；`addlow` 来自 `tv_counter` 位；e=0/1 出音量、e=2 只更新系数（`pcm.cpp:388-550`）。
- 滤波器三实现：`pcm_float`（float 状态 `fstate`，`clamp20` 截断回 20 位，`pcm.cpp:342-349,1440-1475`）；`mcu_mk1`（定点 per-add 饱和，`:1476-1496`）；默认 mk2 用 32 位数学避开溢出（`:1497-1522`）。M4 n=28 null 必须与默认实现逐位一致。
- `nfs`：首步不写回地址相位与若干状态（`:1099-1103,1223-1227,1356-1363`），跑完一步置 1（`:1668`）。
- `interp_lut[3][128]`（`:359-386`）+ 每样本 nibble 选择的 shift 逻辑（`:1371-1435`）。

### 1.5 采样节奏与 66207 Hz 来源

| 量 | 值 | 证据 |
|---|---|---|
| PCM 内步 | `(reg_slots+1)*25` master cycles；stock `reg_slots=28` → **725** | `pcm.cpp:1672-1675` |
| master 时钟 | 24 MHz（H8/532 主机） | `plan_256.md:96`；与 66207 反推一致 |
| 内步率 | 24e6/725 = **33103.45 步/s** | 推算 |
| 每步 `MCU_PostSample` | 1 次；`config_reg_3c.bit6=1` 时 2 次 | `pcm.cpp:676,690-701` |
| 启动 config_3c | `0xc3`（bit6=1） | `task_irq_map.md:198`；`pcm.cpp:649-652` |
| 实际输出 | 33103.45×2 = **66206.9 ≈ 66207 Hz** | `src/mcu.cpp:1780`（MK1/JV880 用 64000） |
| 扩展模式钳制 | `cycle_slots = pcm_ext_active ? 28 : reg_slots`（否则 256 槽 → (255+1)*25=6400 → 3.75k 步/s ≈7.5 kHz，音频饿死） | `pcm.cpp:1670-1675`；`mk2_polyphony_256.md:408` |

### 1.6 效果路由：cases 17/18/21/22/23/31（EFF+3）

效果返回 `rcadd[0..5]`/`rcadd2[0..5]` 在固定效果块算出（`pcm.cpp:967-1076`），注入点是 voice 循环里的 `slot2` 开关（`pcm.cpp:1573-1596`）：

```
slot2 = (slot == reg_slots-1) ? (PCM_EFF_BASE+3) : slot+1    // :1573
switch (slot2) {
  case 17: ram1[EFF+3][1] += rcadd[0]   // reverb?（源注释 "17,18 - reverb"） :1578-1580
  case 18: ram1[EFF+3][3] += rcadd[1]   // :1581-1583
  case 21: ram1[EFF+3][1] += rcadd[2]   // :1584-1586
  case 22: ram1[EFF+3][3] += rcadd[3]   // :1587-1589
  case 23: ram1[EFF+3][1] += rcadd[4]   // :1590-1592
  case EFF+3: ram1[EFF+3][3] += rcadd[5]// 阶段 1 前是 case 31；最后 slot 注入 :1593-1595
}
```

`rcsum[0/1]` 的对应注入在 `:1601-1624`（17/18/22/EFF+3 → `rcsum[1]`；21/23 → `rcsum[0]`；所有 voice 的 `rc0/rc1` 追加在 `:1623-1624`）。`ram1[EFF+3][1]`（L）/`[3]`（R）是**跨 voice 的混音总线**：每个 slot 先累加自己的 `suml/sumr`，不是最后 slot 就写回总线，最后 slot 写入 `accum_l/r`（`:1626-1634`）；下一步的主混音块再读 `accum`（`:660-661`）。

阶段 1 之所以要"效果槽外置"：若 `reg_slots>28` 让 slot 28..31 当 voice，总线 `ram1[31][1]/[3]` 会被 voice 状态覆盖、case 31 双注入、key-off 清效果态、IRQ 污染（`mk2_polyphony_256.md:426`）。外置后 `slot2` 语义对 n≥28 保持：slot 16/17/20/21/22 的注入点不变，仅"最后 slot"从 27 变成 n−1（`:1573`）。

---

## 2. 阶段 1 扩展点总表（当前工作树）

| # | 扩展点 | 位置 | 生效条件 | 默认路径为何不变 |
|---|---|---|---|---|
| E1 | `PCM_MAX_VOICE 255 / PCM_EFF_BASE 256 / PCM_SLOTS 260` | `pcm.h:27-29` | 编译期 | 数组变大，索引 0..31 语义不变；效果行在所有模式都固定在 256..259 |
| E2 | `ram1/ram2/fstate` → `[PCM_SLOTS]` | `pcm.h:32-33,60` | 编译期 | 同上；阶段 1 已验证默认 trace 0-diff |
| E3 | `voice_mask/pending` → `u8[32]`；latch `memcpy` 32B | `pcm.h:36-38`；`pcm.cpp:215-220` | 编译期 | 主窗口 4 次写 + 读回提交流程不变；默认高 224 位恒 0 |
| E4 | 主窗口 reg0 全字节（voice 24..31） | `pcm.cpp:89-90` | `pcm_ext_enabled`（即 `-voices:n≠28`） | 默认 `data & 0x0f`（voice 24..27），位 28..31 写不进去与库存一致 |
| E5 | `select_channel` 扩展 full byte | `pcm.cpp:131-146` | `pcm_ext_active` | 默认 `&0x1f` + 28..31→EFF 重映射（重定位等价已验证） |
| E6 | reg `0x3f` 效果选择别名 | `pcm.cpp:147-152` | `pcm_ext_active` | 默认 0x3f 写无分支、读仍返回 `wave_byte_latch`（`:239-241`） |
| E7 | `PCM_WriteExt/PCM_ReadExt`（0xe800-0xe83f） | `pcm.cpp:288-308` | `pcm_ext_active` | 无调用则不存在；ext 0..0x1b=mask 字节 4..31，0x20=完整 IRQ slot，0x21=active 探针 |
| E8 | 0xe800 窗口路由 | `mcu.cpp:686-689,949-952` | `pcm_ext_active && !mcu_jv880` | 默认落到既有的 SM/SRAM/Unknown 分支，行为与阶段 0 相同 |
| E9 | page6/7 `b_ram/b_ram2` backing | `mcu.cpp:658-659,843-851,1058-1064` | `pcm_ext_enabled` | 默认 page6/7 读 `0x00`、写 Unknown（原有行为）；快照仍不含 page6/7（`04_cosim_spec.md` B1/E3） |
| E10 | `reg_slots` 模式开关 | `pcm.cpp:587` | `pcm_ext_active` | 默认 `(config_reg_3d & 31)+1`（启动 0x7b → 28）；扩展取 `pcm_ext_voices`（方案 A，2026-09-11；不再用 `config+1`，config 恒 0x7b） |
| E11 | `cycles` 钳 28 | `pcm.cpp:1672-1675` | `pcm_ext_active` | 默认 `cycle_slots=reg_slots=28 → 725`，同值 |
| E12 | 效果槽外置（~217 处 `ram1/2[28..31]` → `EFF_BASE+0..3`）+ `pcm_mod_slot` | `pcm.cpp:38-41,655-1076,1097,1221` | 所有模式（重定位） | 库存固件对 ch28..31 的访问被 `select_channel` 一并重映射；调制源索引 28..31 经 `pcm_mod_slot` 映射；阶段 1 已验 EFF=256 vs 28 快照强等价 |
| E13 | CLI `-voices:<n>`（28..255） | `mcu.cpp:1993-2005` | 用户参数 | 无 flag：`pcm_ext_enabled=0, pcm_ext_voices=28` |
| E14 | `MCU_PatchROM` 空实现 | `mcu.cpp:1658-1668` | — | `pcm_ext_active` 恒 0（当前）；该钩子留作 M4 native 挂点或删除（无 ROM 补丁用途） |

不变性现状：阶段 1 完成记录为"默认 200M-202M trace 与基准逐字节一致；EFF=256 vs 28 的 200M 快照强等价（差异仅在效果槽且配对完全相等，eram/accum/rcsum/fstate 零差异）"（`polyphony_256_todo.md:183`）。**快照/`hashdump` 的 `pcm` 原始字节已因结构扩宽而改变**（旧 `demo_postW.bin` 弃用，`04_cosim_spec.md` B2）；M4 任何再动 `pcm_t` 布局都必须同步 ABI guard。

---

## 3. 音频输出路径

### 3.1 `accum_l/accum_r` → SDL 完整链路

```
PCM_Update 内步                       src/pcm.cpp
  voice 循环末 → pcm.accum_l/accum_r（20 位有符号，最后 slot 的 suml/sumr） :1626-1634
  下一步主混音块：
    accum += EFF+2[0]/[1]（效果和声/输出级状态）                          :660-670
    tt[0/1] = ((accum + orval|noise) & ~write_mask) << 12                 :663-674
    MCU_PostSample(tt)                                                    :676
    若 config_3c.bit6：第二次 tt=([3]/[5] 路径) + MCU_PostSample          :690-701

MCU_PostSample                       src/mcu.cpp
  if (master_gain != 1.0) sample *= master_gain（float）                  :1843-1847
  sample >>= 15（算术右移，20 位<<12 → 17 位）                            :1848,1853
  clamp 到 INT16                                                         :1849-1857
  sample_buffer[write_ptr] = L, [write_ptr+1] = R；write_ptr +=2 mod N   :1858-1860

SDL 消费                             src/mcu.cpp
  audio_callback：memcpy ring→SDL，清零已消费区，read_ptr 前移         :1734-1741
  MCU_OpenAudio：S16SYS、2ch、freq=66207（mk1/jv880=64000）、
                 samples=pageSize/4；SDL_OpenAudioDevice + PauseAudio(0) :1771-1833
  背压：work_thread 若 read_ptr==write_ptr，解锁+SDL_Delay(1) 等消费    :1430-1442
```

### 3.2 缓冲与重采样

- 无重采样器：请求频率就是仿真采样率（`mcu.cpp:1780`）；若 SDL 实际设备不支持，`spec_actual` 由 SDL 内部转换（启动打印 Requested/Actual，`:1817-1827`）。
- 环形缓冲：`audio_buffer_size = pageSize*pageNum` shorts（`-ab:<pageSize>:[pageNum]`，默认 512/32）；`spec.samples = pageSize/4` 帧/回调（`:1776-1783`，默认解析 `:1938-1939`）。
- 写指针对齐（`mcu.cpp:1430-1433`）：按 `config_reg_3c.bit6` 将 `sample_write_ptr &= ~3` 或 `&= ~1`，保证 read==write 判定落在整帧/整 PostSample 组边界。
- 欠载：回调照常复制缓冲并清零，产生静音；生产者不会被回调阻塞（除空缓冲自旋）。

### 3.3 线程模型

| 线程 | 职责 | 证据 |
|---|---|---|
| main | SDL 窗口/事件 + `LCD_Update`（持 work_thread 锁 渲染）+ `SDL_Delay(15)` | `mcu.cpp:1638-1656`；`lcd.cpp:431-541` |
| work_thread | 中断轮询→取指/MK2CPP_Step→`cycles+=12`→trace/demo/mocknote/snap/hash→`PCM_Update`→`TIMER_Clock`→`SM_Update`/UART→Analog；空音频缓冲时让出 | `mcu.cpp:1406-1636`，关键顺序 `:1444-1560` |
| SDL 音频回调线程 | 消费 ring → 设备 | `mcu.cpp:1734-1741` |
| MIDI 输入线程 | win32/rtmidi 线程往 UART ring 投字节 | `src/midi_win32.cpp` / `midi_rtmidi.cpp`（非本轮重点） |

### 3.4 现有 CLI 清单（与音频/记录相关，grep `argv`，`src/mcu.cpp:1946-2177`）

| 选项 | 作用 | 行 |
|---|---|---|
| `-p:<port>` | MIDI 端口 | `:1948-1951` |
| `-a:<dev>` | 音频设备索引 | `:1952-1955` |
| `-ab:<pageSize>[:<num>]` | 音频缓冲尺寸 | `:1956-1976` |
| `-gain:<x|db>` | 主机侧线性增益（PostSample 之前） | `:2051-2072` |
| `-float` | 浮点谐振滤波器（影响音频与 `fstate`） | `:1989-1992` |
| `-voices:<n>` | 阶段 1 复音开关 | `:1993-2005` |
| `-demo [cycles]` / `-mocknote [cycles]` | 确定性输入 | `:2006-2017` |
| `-tracepc <f> [a b]` / `-pcmtrace` | PC trace / PCM 控制写日志 | `:2018-2030` |
| `-savesnap <c>` / `-loadsnap <f>` / `-hashdump <c> <f>` | 状态快照/哈希 | `:2031-2050` |

**没有 wav/dump/任何音频落盘能力**（`grep` 全文无 wav/录音相关；`-pcmtrace` 只记控制寄存器 `0..3/0x3c/0x3d/0x3e`，`mcu.cpp:934-946`）。

### 3.5 最小新增设计：`-wav:<file>`（真实数据链路，默认零变化）

```
新增（仅建议，未实现）：
  static FILE *g_wav = nullptr;            // 默认 nullptr
  static long  g_wav_data_bytes = 0;

解析：`-wav:<file>` → fopen(path,"wb")，写 44B 标准 WAV 头
      （RIFF/WAVE/fmt PCM=1, ch=2, rate=spec.freq, bits=16, block=4），
      data size 占位 0。

tap：在 MCU_PostSample 的 clamp 之后（mcu.cpp:1857 后）、写 sample_buffer 之前/之后：
      if (g_wav) { int16_t lr[2]={(int16_t)sample[0],(int16_t)sample[1]};
                   fwrite(lr,2,2,g_wav); g_wav_data_bytes += 4; }

收尾：退出前 fseek 回填 RIFF size / data size 并 fclose。
```

- **默认行为不变**：无 `-wav:` 时仅多一次 `g_wav` 空指针判断（host 侧，不消耗 `mcu.cycles`），音频字节流与 CPU 轨迹不受影响。
- **真实链路**：tap 在 `master_gain` 与 int16 clamp 之后，与 SDL 实际消费的 `sample_buffer` 内容同源；包含 oversampling 的第二次 `MCU_PostSample`。
- **确定性**：采样流由 `pcm.cycles` 驱动，与墙钟/ring 欠载无关；同一输入两模式应逐字节一致（n=28 null 判据）。
- **可选加项**：`-audiohash <cycles> <file>`（对 `MCU_PostSample` 流做 FNV-1a 64 并在目标 cycle 落盘），对齐 `04_cosim_spec.md` L4 `hash_audio`；长跑（≥300M）用哈希、听感/审计用 wav。
- **约束**：GT 运行规范仍适用——GUI+声音+超时 kill（`plan_256.md` §6），wav 不能替代现场听感确认。

---

## 4. M4 原生引擎 API 设计（`mk2cpp/src/hand/`）

### 4.1 集成边界与分期

```
M4a（控制语义化，先落地）
  翻译固件 → 寄存器写 ──(shim)──> 原生 typed Voice 状态 ──同步──> pcm.ram1/ram2 镜像
  DSP 仍用 GT PCM_Update（n=28 音频天然 0 差；n>28 走阶段 1 数组/掩码/EFF）

M4b（DSP 机械移植）
  Engine::update() 取代 GT PCM_Update 内层；逐行照抄 pcm.cpp:584-1677 的量化
  每样本对拍：wav tap（默认路径 vs native 路径）+ pcm 镜像 hash

M4c（255 优化）
  inactive voice 跳过/批处理（须证明与全跑等价）；目标 n=255 实时
```

分期理由：M4a 先把"固件寄存器语义→原生状态"打通，用现成 DSP 保证 n=28 null；M4b 才引入数值重写的风险，并用 M4a 的镜像做 oracle。

### 4.2 固件寄存器写/读 → 原生 voice API 映射

> GT 侧新增的 voice 参数/掩码 API（`PCM_WriteVoiceReg`/`PCM_SetVoiceMask`/`PCM_GetIrqSlot`/
> `PCM_AckVoiceIrq`）见 07 §4.4；本节是 hand 引擎的 typed API。

| 固件动作（寄存器） | 原生 API（hand） | 备注 |
|---|---|---|
| `br+0x3e`（或 0x3f 别名）写 | `Engine::select_channel(ch)` / `fx_select(0..3)` | stock 28..31→EFF；扩展 full byte；0x3f 是效果别名 |
| `br+0x00..0x03` 写 + `br+0x00`(读回) | `Engine::set_mask_byte(idx,val)` + `commit_mask()` | 必须保持两相语义（pending/updating/读回提交） |
| `br+0x04..0x0f` 写 | `Voice_SetAddress(slot, cur/loop/end)` | 当前指针/循环/结束 20 位 |
| `br+0x24..0x2f` 写 | `Voice_SetFilterState / SetDpcmRef` | 固件正常由 DSP 回写；API 留作恢复/调试 |
| `br+0x10/0x11` 写 | `Voice_SetPitch(slot, inc)` | 14 位分数步进 |
| `br+0x12/0x13` 写 | `Voice_SetPan(slot, l, r)` | 高/低字节 |
| `br+0x14/0x15` 写 | `Voice_SetSend(slot, reverb, chorus)` | rc 两半 |
| `br+0x16..0x1b` 写 | `Voice_SetEnvAdjust(slot, ENV0/ENV1/TV, adj)` | speed/target 分别低/高字节 |
| `br+0x1c/0x1d` 写 | `Voice_SetIrqEnable` / `SetSampleSource` / `SetFilterB` | 拆位 |
| `br+0x1e/0x1f` 写 | `Voice_SetPitchSource(slot, src)` / `SetKey` / `SetLoopMode` / `SetReverse` / `SetBank` | 原生存全宽 pitch source（见风险 R3） |
| `br+0x30/0x31` 写 | `Voice_SetPhase`（一般内部状态） | b15/IRQ flag/小数相位 |
| `br+0x20..0x23` 写 + `0x3f` 读 | `Wave_ReadROM(addr)` | 直接复用 GT `PCM_ReadROM`（`pcm.cpp:43-77`） |
| `br+0x3c` 写 | `Engine::set_output_config(v)` | 位语义见 §1.4 |
| `br+0x3d` 写 | `Engine::set_rom_bank_mode(bit5)` + `set_voice_count(n)` | **已解耦（方案 A，out/m4/12，2026-09-11）**：voice 数来自 `-voices`/`pcm_ext_voices`，不再由 config 字节推导 |
| `0xe800+0x00..0x1b` 写 | `Engine::set_mask_byte(4+a, v)` | 高 224 位 voice enable |
| `br+0x3c/0x3e` 读 | `Engine::poll_status() → {slot, updating, ack}` | 含 IRQ ack 副作用；扩展 slot 走 `0xe820` |
| VOICE key on/off（掩码位变化） | `Voice_KeyOn(slot)` / `Voice_KeyOff(slot)` | key = mask & `ram2[7].bit5`（`:1168-1171`） |

固件侧 mirror 命名可对齐 `voice_memory_map.md`：`P-0x14/0x13/0x12` = br+05/09/0d（cur/loop/end 中字节），`P-0x10/0x0e/0x0c` = br+06/0a/0e（低字节），`P-0x18` = br+1a（TV adjust），`P-0x16` = br+36（TV 系数）——与 §1.3 表一致；旧文对 `P-0x18/0x16` 的 "pitch?" 猜测应以此为准。

### 4.3 hand 侧最小 API 清单（建议签名）

```cpp
// mk2cpp/src/hand/pcm_engine.h  （C++11；项目根 CMake 亦要求 gnu++98 兼容则退回 typedef/POD）
namespace mk2c {

enum Env { ENV_VOL1 = 0, ENV_VOL2 = 1, ENV_TV = 2 };

struct Voice {
    // --- 直映 ram1/ram2 的镜像字段（顺序与 pcm_t 对齐，便于同步） ---
    uint32_t addr_cur, addr_loop, addr_end;   // ram1[4], [2], [0]
    uint32_t dpcm_ref;                        // ram1[5]
    int32_t  filt1, filt2;                    // ram1[1], [3]
    uint16_t pitch;                           // ram2[0]
    uint16_t pan;                             // ram2[1]
    uint16_t send;                            // ram2[2]
    uint16_t env_adj[3];                      // ram2[3..5]
    uint16_t ctrl;                            // ram2[6]
    uint16_t actl;                            // ram2[7]
    uint16_t phase;                           // ram2[8]
    uint16_t env_lvl[3];                      // ram2[9..11]
    // --- 原生扩展（不写进 pcm_t；快照需额外处理，见 4.5） ---
    uint32_t pitch_src;                       // 全宽调制源（stock: actl&31）
    bool     key;                             // voice_active 位
    float    fstate1, fstate2;                // -float 状态（镜像 pcm.fstate）
};

class Engine {
public:
    void reset();
    void set_voice_count(uint32_t n);          // 来自 -voices/配置，与 rom bank 解耦
    void select_channel(uint32_t ch);          // 0..255
    void write_reg(uint32_t off, uint8_t v);   // 完整 GT PCM_Write 语义（兼容 shim）
    uint8_t read_reg(uint32_t off);
    void write_ext(uint32_t off, uint8_t v);   // 0xe800

    // 语义层：M4a 固件重写直接调用
    void key_on(uint32_t slot);
    void key_off(uint32_t slot);
    void set_pitch(uint32_t slot, uint16_t inc);
    void set_pitch_source(uint32_t slot, uint32_t src);
    void set_address(uint32_t slot, uint32_t cur, uint32_t loop, uint32_t end);
    void set_bank(uint32_t slot, uint32_t hiaddr, uint16_t old_nibble);
    void set_loop_mode(uint32_t slot, bool loop, bool reverse);
    void set_env_adjust(uint32_t slot, Env e, uint16_t adjust);
    void set_filter_b(uint32_t slot, uint16_t b7);       // ram2[6] bit8-14
    void set_pan(uint32_t slot, uint8_t l, uint8_t r);
    void set_send(uint32_t slot, uint8_t reverb, uint8_t chorus);
    void set_irq_enable(uint32_t slot, bool en);
    void set_sample_source(uint32_t slot, bool residual);

    // 效果（固定 4 槽）
    void fx_select(uint32_t fx);                          // 0..3
    void fx_write(uint32_t fx, uint32_t reg, uint16_t v);

    // 每采样步（替代 GT PCM_Update 内层；在同一调度点调用）
    void update(uint64_t mcu_cycles);

    // 观测/持久化
    void sync_mirror();                                   // typed 状态 → pcm.ram1/ram2/fstate
    unsigned poll_irq();                                  // 返回 slot；含 ack 副作用
};

} // namespace mk2c
```

集成挂点（实现阶段再动 GT，单行、默认关闭）：
- work_thread 的 `PCM_Update(mcu.cycles)`（`mcu.cpp:1548`）在 native 激活时改调 `Engine::update`；
- `MCU_Write/Read` 的 `PCM_Write/Read`（`:684,947,951`）在 native 激活时先过 shim（或直接由 hand 的 `write_reg` 实现）；
- CMake 增加 `file(GLOB MK2CPP_HAND_SRC mk2cpp/src/hand/*.cpp)` 与 `MK2CPP_HAS_HAND` 定义（当前 `src/hand/` 不在编译列表，`CMakeLists.txt:200-204`）。

### 4.4 数据流

```
 [gen/ 翻译固件：voice alloc/note/param]        [hand/ 语义化例程（M4a+）]
             │ 写 0xe0xx (MCU_Write)                       │ 直接 API
             ▼                                             ▼
      hand PCM shim ──► Engine (Voice[256] + FX[4] + eram + tv_counter …)
             │                     │
             │ sync_mirror()       │ update()（同调度点、同样两次 PostSample）
             ▼                     ▼
      GT pcm_t（hash/snapshot/trace oracle）      MCU_PostSample → sample_buffer → SDL/-wav
```

### 4.5 可观测一致性契约（n=28 null 与长跑）

| 观测面 | 必须一致的内容 | 依据/判据 |
|---|---|---|
| 音频 | `MCU_PostSample` int16 L/R 流逐字节（n=28；容差 0） | M4 oracle A4.1（`mk2cpp/docs/04_cosim_spec.md:517-520`，`plan_256.md:101`）；`-wav:`/`-audiohash` 抓取 |
| `pcm_t` 镜像 | 全字段与 GT 同步：`ram1/ram2/select_channel/voice_mask×3/write_latch/wave_*/read_latch/config_*/irq_*/nfs/tv_counter/cycles/eram/accum_l/r/rcsum/fstate` | `hashdump_range("pcm",&pcm,sizeof(pcm))`（`mcu.cpp:1368`）、`state_save`（`:1262`） |
| IRQ | `irq_assert/irq_channel` 值与 `MCU_Interrupt_SetRequest(IRQ0)` 相位 | `pcm.cpp:1527-1538`；相位错会传染 CPU trace/hash |
| 控制写 trace | `-pcmtrace` 的 `reg/val/pc/cyc` 序列（O5/O6） | `mcu.cpp:934-946`；若 shim 绕过 `PCM_Write` 需在 shim 内复刻同一格式与 PC/cycle |
| 状态快照 | `state_save/load` 往返后 native 引擎可继续且与 GT 相同 | `mcu.cpp:1262,1302`；原生扩展字段（`pitch_src/key`）不在 `pcm_t` → 需派生态或额外 hand chunk（见 R7） |
| ROM 读 | `PCM_ReadROM` bank 模式（`config_reg_3d.bit5`）与 `wave_byte_latch` | `pcm.cpp:43-77,104-121,239-241` |

---

## 5. 风险

| # | 风险/问题 | 影响 | 建议 |
|---|---|---|---|
| R1 | **容量 256 vs 验收 255**：`PCM_MAX_VOICE=255` 表示最多 255 voice（0..254），效果在 256..259，数组 260；`-voices:` 拒收 256（`mcu.cpp:1996-1999`） | 命名容易混淆 | 已统一：验收 `-voices:255`，"256 复音"指容量（07 §0.5）；是否扩到 256 见 §7 待定项 |
| R2 | **`config_reg_3d` 双用途**：bit0-4=voice 数−1（库存），bit5=波形 ROM bank 模式（`:46-49`）。扩展公式 `reg_slots=config+1` 在 bit5=0 的 n 区间（[29,32]∪[65,96]∪[129,160]∪[193,224]；"n≥33 撞 bit5" 的旧措辞有误，见 out/m4/12 §2.2）把 bit5 清 0，导致 waverom2 被当成 waverom3 等错读 | 扩展模式波形错乱 | **已解决（方案 A，2026-09-11）**：`reg_slots = pcm_ext_active ? pcm_ext_voices : ((config&31)+1)`，`config_reg_3d` 保持 stock `0x7b`；`rom2[0x1433]` config stamp 取消；诊断由 `-snapinfo` 加 `pcm.ext_voices`，`-hashdump` 格式保持不变（out/m4/12 §4） |
| R3 | **调制源字段只有 5 位**（`ram2[7]&31`，`:1097,1221`）：voice ≥32 无法把自己的 slot 号写进该字段 | 大复音下自调制/互调制语义不可表达 | native Voice 存全宽 `pitch_src`；对固件镜像按需截断；确认固件是否读回该字段（`voice_memory_map.md` 显示只读 mirror，风险可能低，需证据） |
| R4 | **DSP 机械移植的量化陷阱**：`addclip20` 进位入、`multi` 27 位回绕、`calc_tv` type/nfs 门控、`MOVG` 式 shift、`fstate` float 位型 | n=28 null 失败 | 逐行照抄（不要"优化"数学）；M4b 用逐样本 diff + 阶段 1 快照强等价做回归 |
| R5 | **性能**：256 voice×66207 Hz ≈ 16.9M voice-update/s（stock 1.85M/s，约 9×）；效果延迟 RAM 读写也在每步 | 长跑实时性/CPU 占空比 | M4c 先测基线；跳过 inactive voice（注意 inactive 分支每步清 `ram1[1/3/5]`、`fstate`、`ram2[8..10]`，须证明跳过等价），再考虑块处理 |
| R6 | **音频背压与测试口径**：生产受 SDL 消费节流，空缓冲自旋（`:1430-1442`）；快照起点与 from-reset 轨迹有相位差（`04_cosim_spec.md:165-177`） | 长跑/对拍不稳定 | wav tap 放生产者侧；n=28 null 用同一快照/同窗口；避免跨相位直接比后缀 |
| R7 | **原生扩展字段的持久化**：`pitch_src/key/[256]` 若不在 `pcm_t` 内，`state_save/hashdump` 看不到 | 长跑恢复后状态漂移、哈希盲区 | 优先全部状态落在 `pcm_t` 可表达字段（派生态）；否则新增 hand chunk + ABI 版本，或明确豁免场景（对齐 E3） |
| R8 | **`-pcmtrace`/O5 依赖写路径**：若 hand 绕过 `MCU_Write`，trace 丢失 | O5/O6 验收不可用 | shim 保留在写路径，或把 trace 移进 shim 并保持格式/相位 |
| R9 | **`pcm_ext_active` 激活者缺失**：当前恒 0（`mcu.cpp:1658-1668`） | native 激活与 ext 窗口/select 宽度的门控不清 | 已冻结：`pcm_ext_active=1` 仅当 `-mk2cpp && -voices:n` 且 n≠28（09 §3.2）；是否另设独立 flag 见 §7 待定项 |
| R10 | **`-voices` 与 `-mk2cpp` 的组合矩阵**：默认（无 `-mk2cpp`）必须永远走 stock；`-mk2cpp` 无 native 仍走翻译控制+GT DSP | 回归红线 | 已冻结：`-mk2cpp` 下 hand 默认参与（含 n=28）；`-voices:n` 单独保持惰性+警告（09 §3.1） |
| R11 | **效果语义命名置信度**：`rc0/rc1` 的 reverb/chorus 标签、`ram2[EFF+3][8]` 的 `tv_counter` 装载是 GT 注释+`fixme`（`:707`） | 误解效果路由 | 本文按代码事实描述，不把标签当规范；M4 效果移植以逐行等价为准 |
| R12 | **page6/7 不在快照**（`04_cosim_spec.md` B1/E3） | n>32 快照不能完整恢复 | M4 若用 page6 需补 chunk，否则场景豁免 |

---

## 6. 音频抓取方案可行性结论

**可行，且是 n=28 null 验收的必要工程化手段。**

1. 现有代码**没有**任何音频落盘能力（§3.4），因此必须新增；最小方案 `-wav:<file>` 在 `MCU_PostSample` 钳位后 tap（§3.5），默认路径零行为变化（无该 flag 时无文件 I/O，仅一次空指针判断）。
2. tap 点在**生产者侧**，捕获的是实际送进 `sample_buffer`、被 SDL 消费的同一批 int16 数据（含 oversampling 的第二次输出、`master_gain` 与钳位），满足"真实数据链路"。
3. 仿真采样流由 `pcm.cycles` 驱动，与墙钟无关；同一输入下两模式 WAV 的 PCM payload 可直接 sha256 比对（跳过 44B 头）。n=28 窗口 ≥2M cycles（推荐从 200M 快照或 from-reset ≥300M，按 `plan_256.md` §6 的音频取样建议），数据量约 5.5k–830k 帧，可接受。
4. 约束：GT 运行规范（LCD 窗口+声音+超时 kill）仍适用；WAV 只做客观对拍，听感确认仍需现场。建议同时实现 `-audiohash <cycles> <file>` 以支撑 10M-cycle 级检查点。
5. 抓取选项（`-wav:`/`-audiowin`/`-audiohash`/`-snapinfo`）与验收脚本、`pcmdiff` 的完整规格见 10 §2/§5；音频基线冻结与 ≥255 素材属**待办（需用户）**（10 §6）。

---

## 7. 待定项

| # | 待定项 | 说明 |
|---|---|---|
| P1 | `config_reg_3d` bit5 与 voice 数解耦后的对外表示（= 10 D6） | **已解决（方案 A，2026-09-11，out/m4/12 §4）**：`reg_slots = pcm_ext_active ? pcm_ext_voices : ((config&31)+1)`；GT 改动 `src/pcm.cpp:587` 一行，`rom2[0x1433]` config stamp 取消；`config_reg_3d` 保持 stock `0x7b`（bit5=ROM bank 模式）；`-snapinfo` 新增标量 `pcm.ext_voices = n`，`-hashdump` 格式保持不变（保 M1–M3 历史基线兼容）；O4/S4 新期望 = `cfg3d=0x7b`（状态 dump 可得）且 `ext_voices=n`（仅 `-snapinfo`；判据字段缺失时降级 SKIP）；快照 ABI 不变 |
| P2 | 是否/何时放开到 256 声（`-voices:256`；= 10 D1） | **不冲突：活性上限 vs 容量（out/m4/12 §5.2）**——`-voices:255` 是活性声部数上限（`PCM_MAX_VOICE`），"256 复音"是槽位容量（数组 260，slot 255 哨兵）。验收维持 `-voices:255`；放开 `-voices:256` 仍需数组 ≥264/EFF 260..263 并改 CLI 上限 |
| P3 | 调制源 5 位字段的全宽替代与固件回读语义（R3） | native `pitch_src` 存全宽；需确认固件是否读回该字段。out/m4/12 §5.1 已量化：素材化路径把 voice 自身 slot（P-0x02）OR 进 `ram2[7]` 低 5 位，n>28 时 GT 的 `&31`+`pcm_mod_slot` 会错位——不阻塞 slice-3 n=28 gate，留给 M4b 全宽 `pitch_src` |
| P4 | 原生扩展字段的持久化（`pitch_src/key/fstate` 等，R7） | 落 `pcm_t` 派生态、hand chunk + ABI 版本，或明确豁免场景 |
| P5 | 是否新增独立 native 激活 flag（如 `mk2cpp_pcm_native`，R9） | 与 `pcm_ext_active`（激活条件已冻结）的关系 |
| P6 | 效果命名置信度（R11）与 page6/7 快照缺口（R12） | 效果移植以逐行等价为准；hand 原生状态不入 page6 或补快照 chunk |

---

## 附录 A：证据行号速查（本工作树）

- PCM 结构/常量：`src/pcm.h:27-29,31-61`
- 写解码：`src/pcm.cpp:79-204`（mask 82-103；wave 104-122；3c/3d 123-130；3e 131-146；3f 147-152；ram1 153-181；ram2 183-204）
- 读解码：`src/pcm.cpp:210-283`（提交 215-220；status 221-238；3f 239-241；ram1 243-257；ram2 258-268；read_latch 269-280）
- ext：`src/pcm.cpp:288-308`；GT 路由 `src/mcu.cpp:686-689,949-952`
- 工具函数：`addclip20 315-323`；`multi 325-338`；`clamp20 342-349`；`sats 354-357`；`interp_lut 359-386`；`calc_tv 388-550`；`eram_unpack/pack 552-582`
- `PCM_Update`：`584-1677`（主混音 595-703；噪声/accum 655-676；oversampling 690-702；tv_counter 705-712；效果块 716-1156；voice 循环 1164-1661；插值 1407-1435；滤波 1437-1522；IRQ 1527-1538；路由 1573-1624；总线 1626-1634；节奏 1670-1675）
- 音频：`src/mcu.cpp:1734-1741`（回调）、`1771-1833`（打开，1780 频率）、`1841-1861`（PostSample）、`1430-1442`（背压）、`1638-1656`（主线程）、`1406-1636`（work_thread）
- 观测：`src/mcu.cpp:1262,1302`（快照 pcm）、`1368`（hashdump pcm）、`934-946`（pcmtrace）
- CLI：`src/mcu.cpp:1946-2177`；`-voices` `1993-2005`；`MCU_PatchROM` `1658-1668`
- CMake：`CMakeLists.txt:200-204`（`src/hand/` 尚未编译）
- 阶段 1 记录：`tools/docs/polyphony_256_todo.md:173-183`；决策 `tools/docs/plan_256.md:6-17`
