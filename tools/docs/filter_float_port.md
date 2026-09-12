# SC-55 mk2 谐振 filter ↔ VSTi filter 算法对照 + S1 浮点移植规划

**目的**：给 Nuked-SC55 加一条「纯浮点 filter 管线」（代号 **S1**，只动 filter），CLI 可切换、不影响原整数路径。本文记录 **mk2 filter 与 VSTi（SCCore.dll）filter 的精确算法对照**（应用户要求再确认），以及修正后的 S1 浮点 filter 设计。
**姊妹篇**：`scgs_filter.md`（VSTi filter 逆向）、`scgs_pipeline.md`、`scgs_operational_boundary.md`。
**日期**：2026-09-05；**状态更新（2026-09-12）**：S1 按用户方向改为「VSTi 式不饱和」定稿。
- **`-float`（现行）**：归一化标量浮点域，保持芯片差分方程与系数映射（A1/A2/B 拆分同 mk1），状态连续、不量化到 20-bit、**不做逐加饱和**；唯一 20 位边界在输出转换：非对称钳位 `[-0x80000, +0x7ffff]` + 向零截断，并带 NaN 防护。对全部 chip 生效（覆盖 `-mk1`/`-mk2`）。`-floatsat` 已删除，不再提供逐加饱和变体。
- **修订的结论**：文中「缺饱和＝爆音根因」不再成立。int32 版爆音来自 20 位越界值在下游 `multi()` 按 bit19 符号扩展（正峰翻负）；旧 `-float` 的同类现象来自 `clamp20(+1.0)→0x80000`（已修）；另有 `-gain` 乘法 int32 溢出翻负（已修，改在 ±1 域乘后饱和）。修掉 `-gain` 后实时试听无可闻爆音，用户验收通过。
- **高 Q 谐振是正常行为**：实测触发过的系数组（g1=1.40625、g2=0.71875）极点 ≈ **-0.9991**，仍在单位圆内，属芯片自身的极高 Q 谐振/慢衰减振铃，不是异常；此前表述为「不稳定系数」不准确。若将来在更极端系数下出现可闻异常，再另行研究，不作为当前待办。
- **VSTi 为何不需要饱和**：其 A/B 来自 DLS 参数、始终在稳定区；芯片固件会写出贴界系数，前提不同。饱和并不是 mk1 算法的"补丁"，而是 20 位定点音色的一部分。
- 历史 A/B 与旧诊断逻辑见本文后续章节（归档）。

---

## 0. 结论速览

- **数值格式**：filter 内部 float（宽动态范围 / 类 "HDR"），**输出硬钳**（跟 Roland VSTi 同机制——**硬 clamp，非 soft tone curve**），再 **×524288 截断**回 20-bit。2026-09-12 修订：钳位为**非对称** 20 位补码范围 `[-0x80000, +0x7ffff]`（正端 0x7ffff；+1.0 不能映射到 0x80000，否则下游会符号扩展成正满幅翻转）。
- **归一化** = `normalize`（图形学同义）= 除以满量程 524288 → ±1.0 单位尺度。**只有「归一化（±1.0）」一种方案**；「不归一化（±524288）」只是系数写法不同（常数放大 524288 倍），结果相同。
- **算法**：mk2 与 VSTi filter **拓扑完全同构**（2 积分器 State-II 谐振低通）。系数映射：**VSTi 的 B ↔ mk2 的 g1**，**VSTi 的 A ↔ mk2 的 g2**。唯一实质差异：VSTi 输出固定低通，mk2 可选低通/残差。
- **量化**：float→int 用**截断**（C 的 `(int)float` = truncate toward zero，同 VSTi 的 `cvttss2si`），**非四舍五入**。

---

## 1. mk2 filter 精确算法（`src/pcm.cpp:1389-1414`）

### 1.1 系数来源
```
filter = ram2[11]
A1 = (int8_t)(filter >> 8);      // ram2[11] bit8-15，有符号（-128..127）
A2 = (filter >> 1) & 127;        // ram2[11] bit1-7，无符号（0..127）
B  = (ram2[6] >> 8) & 127;       // ram2[6] 高字节低 7 bit，无符号（0..127）
```

### 1.2 定点缩放
- `(x >> 6) + ((x >> 5) & 1)`  = **round-to-nearest 的 x/64**（bit5 决定进位）
- `(x >> 13) + ((x >> 12) & 1)` = **round-to-nearest 的 x/8192**（bit12 决定进位）

### 1.3 逐步展开
```
mult1 = reg1*A1;  mult2 = reg1*A2;  mult3 = reg1*B
v2     = reg3 + round(mult1/64)
v1     = v2 + round(mult2/8192)      // = reg3 + reg1·(A1/64 + A2/8192)
subvar = v1 + round(mult3/64)        // = v1 + reg1·(B/64)
ram1[3] = v1                         // state2_new（低通）
tests  = test; tests<<=12; tests>>=12;   // no-op（20-bit 不溢出）
v3     = tests − subvar              // 残差
mult4 = v3*A1;  mult5 = v3*A2
v5     = reg1 + round(mult4/64) + round(mult5/8192)   // = reg1 + v3·(A1/64 + A2/8192)
ram1[1] = v5                         // state1_new
```

### 1.4 状态方程（定义 `g1 = A1/64 + A2/8192`，`g2 = B/64`）
```
state2_new = state2 + state1·g1     // v1（低通）—— 【不含】g2
subvar     = state2_new + state1·g2  // g2 只进残差，不存 state2
out_v3     = test − subvar          // 残差
state1_new = state1 + out_v3·g1     // 【同一个 g1】（同 state2 反馈）
```
输出：`ram1[3]` = v1（低通），`v3` = 残差。下游 `sample = (ram2[6]&2)==0 ? ram1[3] : v3;`（`pcm.cpp:1442`）在低通 / 残差间选。

### 1.5 实现
- **mk1 分支**（`1368-1388`）：`multi`（20×8→25 bit）+ `addclip20`（20-bit 饱和，±524288）。
- **mk2 分支**（`1389-1414`）：**plain 32-bit int**（`reg1 * (int8_t)…` + plain `+`），**无饱和**（`// hack: use 32-bit math to avoid overflow`）。

---

## 2. VSTi filter 精确算法（`re/scgs_filter.md`，`sc.asm`）
```
state2_new = state2 + state1·B          // 低通
out        = state2_new                 // ← VSTi 固定取低通
err        = input − (state2_new + state1·A)
state1_new = state1 + err·B             // 【同一个 B】
```
实现：float32 SSE，标量（`0x18008ce70` 分发到 4 变体）+ SIMD（`0x18008d9a0`，一次 4 声部）。**每样本不饱和**（`.text` 里 `minps/maxps/blendps = 0`）。

---

## 3. 两者对比

| | mk2（pcm.cpp） | VSTi（sc.asm） |
|---|---|---|
| state2 反馈系数 | **g1 = A1/64 + A2/8192** | **B**（DLS 参数）|
| 残差额外项系数 | **g2 = B/64** | **A**（DLS 参数）|
| state1 反馈系数 | g1（同 state2）| B（同 state2）|
| 更新顺序 | 旧 state2→新 state2 → 残差（用**新** state2）→ state1（旧 state1 + 新残差）| 同左（标准 State-II）|
| 输出 | **可选** 低通 v1 / 残差 v3（ram2[6] bit1）| **固定** 低通 state2 |

**结论**：拓扑完全同构（2 积分器 State-II 谐振低通，标准更新顺序）。系数映射 **VSTi B ↔ mk2 g1**、**VSTi A ↔ mk2 g2**。唯一实质差异：**输出取法**（VSTi 固定低通；mk2 可选残差）。

---

## 4. 修正后的 S1 浮点 filter（精确对齐 mk2）
```c
// 系数
float A1 = (float)(int8_t)(filter >> 8);
float A2 = (float)((filter >> 1) & 127);
float Bc = (float)((ram2[6] >> 8) & 127);
const float s6 = 1.0f/64.0f, s13 = 1.0f/8192.0f;
const float g1 = A1*s6 + A2*s13;   // state2 / state1 反馈
const float g2 = Bc*s6;            // 残差额外项（只进 out_v3）

float xf = test / 524288.0f;       // 归一化（normalize）→ ±1.0

// 每个 stage 计算后钳 ±1.0（mk1 逐加法饱和，归一化域）——把 ring-up 驯成有界，
// 状态永不逃逸满量程（这就是 A/B 确认的爆音根因修复）。
float state2_new = sats(fstate2 + fstate1*g1);   // v1（低通，无 g2）
float subvar     = sats(state2_new + fstate1*g2);
float out_v3     = sats(xf − subvar);            // 残差
float state1_new = sats(fstate1 + out_v3*g1);    // 同 g1

// 输出：±1.0 硬钳 + ×524288 截断回 20-bit（同 Roland cvttss2si 截断）
ram1[3] = clamp20(state2_new);  // 低通
v3      = clamp20(out_v3);      // 残差
fstate1 = state1_new;  fstate2 = state2_new;
```
> 浮点用精确乘法，比 mk2 的定点 round-to-nearest 更干净——这正是 float 相对 int 的一个好处。
> **饱和是根因修复**：早期版本只在输出钳（`state2_new`/`subvar`/`out_v3` 不钳），状态在反馈环里跑逸 → 输出被硬切成爆音；改成每 stage 钳（`sats`）后干净，与 `-mk1` 同听感。
>
> **之前简化版的两处错误（已修正）**：① 把 g2 加进了 state2 更新（mk2 的 v1 **不含** B 项）；② state1 更新用了 `(xf − fstate2)` 而非完整残差 `xf − (state2_new + fstate1·g2)`。

---

## 5. S1 实现要点（范围：仅 filter）

- **CLI**：`mcu.cpp` main（`1350`）加 `-float`（仿 `-mk2`，`1396`），设全局 `int pcm_float`（`mcu.h` extern，命名跟 `mcu_mk1` 一类但语义是 DSP 模式，非 chip selector）。
- **分支**：`pcm.cpp:1368` 改三分支 `if (pcm_float) {…} else if (mcu_mk1) {…} else {…}`，后两支原样不动。
- **并行 float state**：`struct pcm_t`（`pcm.h:24`）加 `float fstate[32][2]`（state1/state2，跨样本持续、不量化，避免反馈量化噪声）；`!active` 清零处（`pcm.cpp:1542`）同步清 `fstate[slot]`。整数路径不读它，**互不影响**。
- **下游不变**：envelope `calc_tv`（`1432-1437`）读 `ram2[3/4/5/9/10/11]`，浮点 filter 不碰 `ram2`；gain/pan/mix（`1442-1527`）原样消费 `v3`/`ram1[3]`。

---

## 6. 验证

`build.bat` → 跑 `-float` 与默认 `-mk2` 对比**干音**（单声部、关 reverb/chorus），重点听高谐振音符的爆音/过载是否消失、瞬态是否还在。

---

## 7. 待办 / 后续

- **S2**（逐声部后半：filter→envelope→gain→pan 浮点）、**S3**（全后端含 reverb/chorus E-RAM + final DAC）暂缓。
- **软 tone curve**（Reinhard / soft-knee / tanh）暂缓——先用硬 clamp 对齐 Roland，听感确认后再考虑在 clamp 前加一条软曲线。
- **VSTi 最终输出**：音频是 **float（±1.0）给 VST host**，int16 由 host 转；其内部唯一 float→int 是 **27-bit 定点相位量化**（`×2²⁷` + `cvttss2si` 截断，`0x180005ce4`，保护 clamp ±16.0），**非音频样本路径**。音频侧的 ±1.0 归一化限幅在 `0x180006bdc`（`comiss`+分支硬钳，输出仍 float）。

---

## 8. 证据索引

| 项 | 位置 |
|---|---|
| mk2 filter | `src/pcm.cpp:1389-1414`（mk1 对照 `1368-1388`）|
| filter 输出选择 | `src/pcm.cpp:1442`（`sample = ram1[3] or v3`）|
| 20-bit 饱和 / multi | `src/pcm.cpp:265-288`（`addclip20` / `multi`）|
| 结构体 | `src/pcm.h:24-51`（`ram1[32][8]`、`ram2[32][16]`、`accum_l/r`）|
| VSTi filter | `re/scgs_filter.md`、`sc.asm`（`0x18008ce70` / `0x18008d9a0`）|
| VSTi ±1.0 归一化限幅 | `sc.asm` `0x180006bdc`（`comiss` + `movl 0x3f800000/0xbf800000`）|
| VSTi ×2²⁷ 量化 + ±16.0 clamp | `sc.asm` `0x180005ce4`（`cvttss2si` 截断）|
| 全管线段表 | `re/scgs_pipeline.md` §1 |
| 数值边界 | `re/scgs_operational_boundary.md` §2-§6 |
