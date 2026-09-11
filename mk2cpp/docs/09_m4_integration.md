# 09 M4 hand 覆盖表与集成设计

> **暂缓（2026-09-11）**：本文的开关矩阵中 `pcm_ext_active=1` 仅当 `-mk2cpp &&
> -voices:n` 且 n≠28 的设计随 256 扩展回滚而**暂缓**（里程碑拆分后归 M5）。
> hand 覆盖表 / `MK2CPP_*` 集成骨架（L0/L1、稀疏 override 表、
> `MK2CPP_Configure/PostReset`）本身不依赖 256，仍有效（属 M4）；
> `src/hand/pcm_enable.cpp` 当前只翻译 stock 28 的 PCM flush。

状态：已评审 v1，2026-09-11。
配套文档：[07 voice 语义](07_m4_voice_spec.md) · [08 PCM 引擎与音频路径](08_m4_pcm_api.md) · [10 验收 oracle](10_m4_oracle.md) · [00 计划](00_plan.md)。
阅读顺序：00_plan §M4 → 07 → 08 → **09（集成/分派）** → 10。
口径：目标 = **256 声同时发音**；验收上限 `-voices:255` 是 `0xff` 哨兵 + 8-bit 池计数妥协下的**阶段性上限，非最终目标**（07 §0.5）；原生引擎在 `-mk2cpp` 下默认参与（含 n=28），`pcm_ext_active=1` 仅当 `-mk2cpp && -voices:n` 且 n≠28；前期限定 L0（一次 `MK2CPP_Step` = 恰好一条 H8 指令），L1 在 n=28 null 通过前不得启用。

范围：M4「`src/hand/` 人工语义实现」如何接入现有 mk2cpp 分派；本文只定集成契约，
voice 语义（pool/alloc/free）见 07，PCM enable/mask/编程路径见 08。
行号口径：评审基线 HEAD `84d3e51`；引用 GT/mk2cpp 行号在实现推进后可能漂移。
依据：`mk2cpp/include/mk2cpp.h`、`mk2cpp/src/mk2cpp.cpp`、`src/mcu.cpp`、
`src/submcu.cpp`、`src/pcm.cpp`、`CMakeLists.txt`、`mk2cpp/docs/01..06`、
`tools/docs/plan_256.md`、`tools/docs/voice_bounds_inventory.md`、
`tools/docs/voice_memory_map.md`、`tools/docs/task_irq_map.md`。

---

## 0. 结论摘要（TL;DR）

1. **覆盖机制选方案 B（独立 override 表优先查询）**，不选 A（last-write-wins）。
   B 可以做到 **GT 调用点零改动**（`src/mcu.cpp:1451` 一行不动），
   hand/gen 来源可分辨、可统计、可单独关闭，注册顺序无关，且 hand 可在无 gen
   的构建里独立工作。
2. hand 表用**稀疏有序数组 + 二分**（条目数预计 ≤ 几十），不要镜像 2×64K 表；
   查表开销只在 `count>0` 且命中/未命中一次二分时发生。
3. **原生引擎在 `-mk2cpp` 下默认启用（含 n=28，已冻结）**，用于 A4.1 null 测试；
   `pcm_ext_active=1`（GT 扩展设备模式）只在 `-mk2cpp && -voices:n` 且 n≠28 时打开。
   `-voices:n 单独`（无 `-mk2cpp`）维持现状（惰性），打印警告。
4. **保持「一次 `MK2CPP_Step` = 恰好一条 H8 指令」** 是 M4 前期的硬约束；
   整例程替换（L1）需要新的返回计数/中断边界协议，且会改变 trace 行数、
   IRQ 轮询位置与 `ex_ignore` 语义，**在 n=28 null 通过前不得启用**。
5. **第一个切片推荐 PCM enable/disable flush**（`0x5525/0x5527`、
   `0x5662/0x5664`，见 `tools/docs/task_irq_map.md:499` O5）：叶节点、2~3 条指令、
   有现成 `-pcmtrace` oracle、直接压中 256-bit mask 契约（08 领域）。
   pool-init `0x40462`（`tools/docs/plan_256.md:66`）与 alloc/free 作为第二、三切片
   （07 领域；完整顺序见 §5.5）。
6. 现状发现两个集成缺口，建议 M4 一并补：
   (a) 生成代码**没有**复刻 `MCU_ReadInstruction` 尾部的 T 状态 trace 异常检查
   （`src/mcu.cpp:1085-1088`）；应统一挪进 `MK2CPP_Step()`；
   (b) `MK2CPP_Init()` 在参数解析中途被调用（`src/mcu.cpp:1980`），而
   `-voices:` 可能在它之后解析、ROM/PCM 复位更在其后；需要新增
   `MK2CPP_Configure(int argc, char **argv)`（argv 解析完）与 `MK2CPP_PostReset()`
   （`PCM_Reset` 之后）两个钩子。
7. ABI 改动量：头文件 +约 7 个声明/1 个全局，`mk2cpp.cpp` +约 100~150 行，
   现有函数签名全部不变；GT 侧除开关/激活钩子外可零改动。

---

## 1. 现状核对：现有分派数据流

### 1.1 ABI（`mk2cpp/include/mk2cpp.h`）

| 符号 | 位置 | 语义 |
|---|---|---|
| `mk2cpp_enabled` | `mk2cpp.h:18` | `-mk2cpp` 开关；0 = 完全 stock |
| `mk2cpp_mixed` | `mk2cpp.h:20` | 1 = 未翻译 PC 回退解释器（默认，且无任何代码写它） |
| `mk2cpp_fn` | `mk2cpp.h:22` | `void (*)(void)`，一 PC 一函数 |
| `MK2CPP_Init` | `mk2cpp.h:25` | 注册 gen 表（含 SM）；由 `-mk2cpp` 触发 |
| `MK2CPP_RegisterFn(flat,fn)` | `mk2cpp.h:27` | gen 发布一个主 CPU 翻译 PC |
| `MK2CPP_CanStep(flat)` | `mk2cpp.h:29` | 该地址是否有翻译 |
| `MK2CPP_Step()` | `mk2cpp.h:31` | 执行当前 `mcu.cp/mcu.pc` 的**恰好一条**指令 |
| `MK2CPP_SM_Register/CanStep/Step` | `mk2cpp.h:36,38,40` | SM 版（按 `sm.pc`，无页） |
| `MK2CPP_FallbackCount/TranslatedCount` | `mk2cpp.h:43-44` | 主 CPU 统计 |
| `MK2CPP_SM_FallbackCount/SM_TranslatedCount` | `mk2cpp.h:45-46` | SM 统计 |
| `MK2CPP_Version` | `mk2cpp.h:47` | 版本串 |

### 1.2 `mk2cpp/src/mk2cpp.cpp`（薄分派层）

- 分派表：`mk2cpp_tab_cp0[0x10000]`、`mk2cpp_tab_cp4[0x10000]`
  （`mk2cpp.cpp:23-24`）；SM 全 16-bit 直接索引 `mk2cpp_sm_tab[0x10000]`
  （`mk2cpp.cpp:31`）。全局计数器 4 个（`mk2cpp.cpp:25-26,32-33`）。
- 注册：`MK2CPP_RegisterFn` 取 `cp=(flat>>16)&0xff`、`off=(uint16_t)flat`，
  只接受 cp0/cp4，其它页 `fprintf` 后丢弃（`mk2cpp.cpp:35-45`）；
  `MK2CPP_SM_Register` 直接 `tab[pc]=fn`（`mk2cpp.cpp:82-85`）。
- 查询/执行：`mk2cpp_lookup`（`mk2cpp.cpp:47-55`）→ `MK2CPP_CanStep`
  （`mk2cpp.cpp:57-60`）→ `MK2CPP_Step`（`mk2cpp.cpp:62-80`）：
  命中 `fn(); mk2cpp_translated++`；未命中 `mk2cpp_fallbacks++`，
  `mixed` 则调 `MCU_ReadInstruction()`（`src/mcu.cpp:1079`），否则
  `fprintf+exit(1)`（`mk2cpp.cpp:78-79`）。注意 `CanStep` 与 `Step` 是**两次查表**
  （调用点先 `CanStep` 再 `Step`）。
- SM 同上（`mk2cpp.cpp:92-110`），fallback 调 `SM_ExecuteOneInstruction()`
  （`src/submcu.cpp:1421`）。
- 初始化：`MK2CPP_Init` 先 `MK2CPP_FillTables()`（`#ifdef MK2CPP_HAS_GEN`），
  再 `MK2CPP_SM_FillTables()`（`#ifdef MK2CPP_HAS_SM_GEN`）
  （`mk2cpp.cpp:122-134`）；然后遍历两张表计数并打印一行
  （`mk2cpp.cpp:135-146`，`n_main||n_sm` 才打印）。
- 版本：`MK2CPP_Version` 按 `MK2CPP_HAS_GEN` 返回两个字面量之一
  （`mk2cpp.cpp:159-166`），**不包含 hand 信息**。

### 1.3 GT 主机侧（`src/mcu.cpp`）

- 主循环取指处（唯一开关点）：
  `if (mk2cpp_enabled && MK2CPP_CanStep(flat)) MK2CPP_Step(); else MCU_ReadInstruction();`
  （`src/mcu.cpp:1451-1454`），其中 flat=`((uint32_t)mcu.cp<<16)|mcu.pc`。
- 每步顺序（`work_thread`）：音频背压（`:1428-1443`）→ 中断/`ex_ignore`
  （`:1444-1447`）→ 取指执行（`:1449-1455`）→ `mcu.cycles += 12`
  （`:1457`）→ `-tracepc`（`:1459-1478`）→ `-demo`（`:1480-1534`）→
  `-mocknote` / `-savesnap` → `PCM_Update(mcu.cycles)`（`:1434`）→
  `TIMER_Clock` → `SM_Update`（`:1438-1444`）→ `MCU_UpdateAnalog`。
  **翻译代码不负责 cycles、trace、设备更新、中断轮询**，全部由宿主做。
- 参数解析：`-mk2cpp` 设 `mk2cpp_enabled=1` 并**当场** `MK2CPP_Init()`，
  打印 `(mixed=..) -- <version>`（`src/mcu.cpp:1977-1983`）；
  `-voices:<n>` 只设 `pcm_ext_voices=n`、`pcm_ext_enabled=(n!=28)`
  （`src/mcu.cpp:1993-2005`），范围 28..255（`PCM_MAX_VOICE`）。
- 初始化顺序：`MCU_Init → MCU_PatchROM → MCU_Reset → SM_Reset → PCM_Reset`
  （`src/mcu.cpp:2480-2484`）。`MCU_PatchROM()` 现在是**空函数**
  （`src/mcu.cpp:1658-1668`），只有被注释的 ROM-patch 代码。
- 帮助文本仍写 "applies an in-memory ROM patch"（`src/mcu.cpp:2162-2164`），
  M4 需改措辞。

### 1.4 SM 侧（`src/submcu.cpp`）

- `SM_ExecuteOneInstruction()` = `SM_ReadAdvance()` + `SM_Opcode_Table[op]`
  （`src/submcu.cpp:1421-1425`），同时是 mk2cpp 的 SM 回退实现（注释
  `src/submcu.cpp:1418-1420`）。
- `SM_Update(mcu.cycles)`：`while (sm.cycles < cycles*5)` 内
  `SM_HandleInterrupt()` → 非 sleep 时 `mk2cpp_enabled && MK2CPP_SM_CanStep(sm.pc)`
  则 `MK2CPP_SM_Step()` 否则 `SM_ExecuteOneInstruction()`
  （`src/submcu.cpp:1436-1439`）→ `sm.cycles += 48`（`:1442`）→
  `SM_UpdateTimer()` / `SM_UpdateUART()`（`:1444-1445`）。
  **每个 SM 指令都夹在两段 SM 设备更新之间**，这是第 4 节约束的来源。

### 1.5 构建（`CMakeLists.txt`）

- `mk2cpp/src/mk2cpp.cpp` 恒入 `SC55_SRC`（`CMakeLists.txt:200-201`）；
  包含目录加 `mk2cpp/include`（`:225`）。
- `if(MK2CPP_GEN_DIR)`：`file(GLOB .../*.cpp)` 与 `.../sm/*.cpp` 追加
  （`:216-220`）；有文件时定义 `MK2CPP_HAS_GEN` / `MK2CPP_HAS_SM_GEN`
  （`:227-235`）。默认不传变量 → 不编译生成码 → `-mk2cpp` 打印
  "no translated code linked"。
- `src/gen/` 被 `.gitignore` 忽略（`mk2cpp/src/gen/` 规则，随公开 `.gitignore`）；
  `src/hand/` 目前在树内、**无 CMake 接线**（目录为空）。

### 1.6 `pcm_ext_*` 现状（与 M4 强相关）

- 全局：`pcm_ext_enabled`（flag）、`pcm_ext_voices`（n，28..255）、
  `pcm_ext_active`（"扩展已生效"）（`src/mcu.h:439-441`，定义
  `src/mcu.cpp:189-191`）。
- GT 阶段 1 设备能力（保留）：
  - 256 位 mask（32 字节 current/pending + 主窗口读时 latch）
    （`src/pcm.h:36-38`；`src/pcm.cpp:215-220`）；
  - 扩展窗 `PCM_WriteExt/ReadExt`（`src/pcm.cpp:288-308`），H8 侧 0xe800 路由
    **必须 `pcm_ext_active`**（读 `src/mcu.cpp:685-688`、写 `:948-951`）；
  - `0x3e` 在 active 时按裸 slot（`src/pcm.cpp:133-138`），
    `0x3f` 效果别名 `PCM_EFF_BASE+(v&3)`（`:147-152`）；
  - `reg_slots` 公式（active 时 `config_reg_3d+1`）`src/pcm.cpp:587`；
  - 采样节奏钳 28（active 时）`src/pcm.cpp:1672-1675`；
  - IRQ 完整 slot 经扩展窗 0x20（`src/pcm.cpp:303-306`；IRQ 置位
    `:1527-1538`）。
- **缺口**：当前没有任何路径把 `pcm_ext_active` 置 1（`MCU_PatchROM` 为空，
  patch 文件已从树中清除）。因此 `-voices:n`（无 `-mk2cpp`）只是
  "page6/7 backing 可见 + reg0 主 mask 由 4bit 放宽到 8bit"
  （主 mask 放宽依据 `src/pcm.cpp:89-90`，page6/7 依据 `src/mcu.cpp:842-850,
  1057-1064`），PCM 语义仍是 stock。M4 必须由 hand 引擎显式打开 active。

### 1.7 统计/诊断现状

- 计数器只在 `mk2cpp.cpp:25-26,32-33` 递增，API 在 `mk2cpp.h:43-46`；
  **全仓库没有任何消费点**（无启动/退出打印、无 per-PC 直方图）。
- 打印内容只有 Init 的注册计数（`mk2cpp.cpp:141-146`）。
- `mk2cpp_mixed` 无 CLI 开关，永远为 1。
- 两模式回归脚本：`mk2cpp/tests/two_mode_check.ps1`（默认解释器 vs `-mk2cpp`，
  trace + hashdump 对照）。

### 1.8 每步语义契约（hand 必须遵守）

- `MCU_ReadCodeAdvance()` 读 `(cp,pc)` 后 `pc++`（`src/mcu.h:192-196`）；
  gen 函数按 GT 顺序消耗操作数字节（示例 `mk2cpp/src/gen/mk2c_r1.cpp:16-27`），
  并在跳转/调用指令里自行写回 `mcu.pc/cp`。
- `MCU_ReadInstruction()` 在指令后检查 `if (mcu.sr & STATUS_T)`
  → 排 `EXCEPTION_SOURCE_TRACE`（`src/mcu.cpp:1085-1088`）。
  **gen 代码完全不发射这个检查**（`mk2c_r1.cpp` 中 `EXCEPTION_SOURCE_TRACE`
  出现 0 次）；当前 boot/demo 窗口未见分歧，但这是潜在缺口。
  hand 契约里应把它统一放到 `MK2CPP_Step()` 尾部，一处覆盖 gen+hand。
- gen 函数不碰 `mcu.cycles`、不轮询中断、不调设备；`sleep` 置位后宿主下一轮
  不取指只 tick（`src/mcu.cpp:1449`）。
- `ex_ignore` 只在宿主循环读一次/清零一次（`src/mcu.cpp:1444-1447`）；
  单指令粒度下 RTE/LDC/ORC/ANDC 的"跳过下一次轮询"语义天然正确。

---

## 2. hand 覆盖机制设计

### 2.1 需求与约束

1. hand 是**手工入库**代码，`src/gen/` 不入库；两者可独立存在：
   - 有 gen 无 hand：现状 M1–M3；
   - 有 hand 无 gen：M4 后期目标（本地不生成 gen 也能跑 hand 覆盖的子系统）；
   - 两者都有：hand 优先。
2. 需要能回答"这条 PC 是 hand 还是 gen""hand 命中多少次""未覆盖比例"，
   以支撑 A4.1 null 与 triage。
3. 需要 A/B 切换（hand on/off）以证明 hand 路径等价。
4. 不改 GT 取指调用形状（降低对已冻结 `src/` 的扰动）。
5. 注册必须纯静态、无 ROM 依赖：`MK2CPP_Init` 在 ROM 加载前调用
   （`src/mcu.cpp:1980`）。

### 2.2 方案 A：hand 注册在 gen 之后，last-write-wins

做法：hand 的 `*_FillTables()` 在 gen `FillTables()` 之后调用，复用
`MK2CPP_RegisterFn` 覆盖同一 PC 的指针。

- 优点：零 ABI 增量、零额外内存、`CanStep/Step` 不改。
- 缺点：
  - 来源不可分辨：一旦覆盖，无法统计"gen 覆盖但被 hand 替换"的数量；
    去掉 hand 只能重编译或加宏。
  - 顺序耦合：任何新的注册方（SM、未来动态注册）插进中间就会静默覆盖错误。
  - 无法支持"hand 无 gen 的构建"以外的清晰分层（能跑，但和 gen 混在一张表里）。
  - 版本/诊断只能靠外部宏。

### 2.3 方案 B：独立 override 表优先查询（选定）

做法：`mk2cpp.cpp` 增加 hand 表（稀疏有序数组），`MK2CPP_CanStep/Step`
内部先查 hand，再查 gen；GT 调用点不变。

```c
/* 拟议：mk2cpp.cpp 内部 */
typedef struct {
    uint32_t     flat;    /* (cp<<16)|pc */
    mk2cpp_fn    fn;
    uint32_t     hits;    /* 诊断 */
} mk2cpp_hand_ent;
static mk2cpp_hand_ent mk2cpp_hand[256];   /* 稀疏、按 flat 升序 */
static uint32_t mk2cpp_hand_n = 0;
static uint32_t mk2cpp_hand_hits = 0;
```

- `MK2CPP_HandRegister(flat, fn)`：只接受 cp0/cp4（与 gen 相同），
  插入排序/构建后二分；重复注册报错（fail fast，不 last-write-wins）。
  条目的人类可读名来自 hand 源文件头（§2.6），不占 ABI。
- `mk2cpp_lookup`：`hand_n && (e=hand_bsearch(flat))` 返回 hand；否则 gen 表。
- `MK2CPP_Step` 命中时分别对 `hand_hits`/`translated` 计数；诊断时能区分。

### 2.4 A/B 对比与选型理由

| 维度 | A last-write-wins | B 独立 override 表 |
|---|---|---|
| `src/mcu.cpp` 改动 | 0 | 0（查询在库内） |
| ABI 增量 | 0 | +hand 注册/查询/统计声明 |
| 内存 | 0 | 稀疏 ≤ 几百条；不镜像 64K 表 |
| 来源统计 | 不可 | 可分（hand/gen/fallback） |
| hand on/off | 重编译/宏 | 运行时 `mk2cpp_hand_enabled` |
| 顺序健壮性 | 差 | 无顺序依赖（hand 先于 gen） |
| 无 gen 构建 | 可 | 可（hand 命中即可） |
| 冲突检测 | 静默覆盖 | 注册期报错 |

**选 B。** 决定性理由是 M4 的验证方式本身：A4.1 要求 n=28 音频 null，
必须能一键把 hand 关掉跑同一二进制（方案 A 做不到）；triage 需要区分
"hand 与 gen 在哪个 PC 上分叉"（A 做不到）。内存/性能代价可忽略
（hand 条目 ≤ ~256，二分 8 次比较，且 `hand_n==0` 时零开销）。

### 2.5 拟议 ABI 增量（`mk2cpp.h`）

```c
/* ---- hand override (M4，冻结 API) ---- */
/* 1 = hand 覆盖启用（默认；CLI -mk2cpp-hand:0|1 切换，用于 A/B） */
extern int mk2cpp_hand_enabled;
/* 注册一个 hand 实现；重复 flat 视为错误（fail fast）。 */
void MK2CPP_HandRegister(uint32_t flat, mk2cpp_fn fn);
/* 由 hand 源文件提供，MK2CPP_Init 在 gen 填充后调用（#ifdef MK2CPP_HAS_HAND） */
void MK2CPP_HandFillTables(void);
/* 统计 */
uint32_t MK2CPP_HandCount(void);            /* 注册条目数 */
uint32_t MK2CPP_HandHitCount(void);         /* hand 命中总数 */
/* 配置/生命周期钩子（见 §3、§4） */
void MK2CPP_Configure(int argc, char **argv); /* argv 解析完成后调用 */
void MK2CPP_PostReset(void);                  /* PCM_Reset 之后调用 */
/* 可选诊断：hashdump/-exit 时打印分派统计（flat+hits） */
void MK2CPP_DumpStats(void);
```

`mk2cpp_fn` 复用现有 typedef；**不新增第二套函数指针类型**。
约 7 个函数声明、1 个全局。

### 2.6 诊断、统计与版本标记

- 统计口径（建议）：
  - `hand_hits`：命中 hand 的 host step 数；
  - `gen_hits` = `translated`（保留现名，但可考虑改名 alias）；
  - `fallback`：未命中任何表且回退解释器；
  - per-entry `hits` 仅 hand 表保留（数量少）。
- `MK2CPP_Init` 打印扩展为：
  `mk2cpp: <G> gen PCs + <H> hand overrides (<G+H 去重后> distinct), <S> SM PCs`。
- `MK2CPP_DumpStats` 打印 4 类计数 + top-N hand 条目（flat+hits；人类可读名
  取 §2.6 的文件头注释）；挂到 `-hashdump`（`src/mcu.cpp` hashdump 调用点附近）
  与 `-mk2cpp` 的 `atexit`。
- 版本标记：
  - `MK2CPP_Version()` 改为静态 buffer，内容形如
    `"mk2cpp gen h8emit 0.1.0 (rom1=8a1eb33c..) + hand 12 overrides rev 3"`；
  - 每个 `src/hand/*.cpp` 文件头固定格式（与 gen 头对齐、可被脚本 grep）：
    ```
    // HAND <subsystem>/<name> flats=0005525,0005527,0005662,0005664
    // rom1 sha256 8a1eb33c... rom2 sha256 a4c9fd82... hand_rev 3
    ```
  - hand 与 gen 的 ROM sha 一致性由 Init 启动时比对宏
    `MK2CPP_ROM1_SHA`（由 CMake/hand 头提供；不一致打印 FATAL 级警告，
    但不退出，便于实验）。

### 2.7 构建衔接：`src/hand/` 入库、`src/gen/` 不入库

CMake 拟议（只加不改现有行）：

```cmake
# hand: committed native implementations (M4); optional, enabled when files exist
file(GLOB MK2CPP_HAND_SRC "${CMAKE_CURRENT_SOURCE_DIR}/mk2cpp/src/hand/*.cpp")
if(MK2CPP_HAND_SRC)
    list(APPEND SC55_SRC ${MK2CPP_HAND_SRC})
endif()
...
if(MK2CPP_HAND_SRC)
    target_compile_definitions(nuked-sc55 PRIVATE MK2CPP_HAS_HAND)
endif()
```

说明：
- `file(GLOB)` 与现有 `MK2CPP_GEN_DIR` 处理一致；新增 hand 文件后需重跑
  `cmake`（若在意，可改显式列表；本轮不引入 `CONFIGURE_DEPENDS` 以兼容
  `cmake_minimum_required(3.2)`，`CMakeLists.txt:1`）。
- hand 目录空时（默认克隆）行为与现状逐字节一致。
- hand 源只依赖 `mk2cpp.h` + GT 头（`src/mcu.h`、`src/pcm.h` 等），
  **禁止依赖 `src/gen/` 符号**（否则无 gen 构建链接失败）；
  若 hand 需要复用 gen 的物化 helper，通过 `MK2CPP_CallGen` 弱接口，
  由 `MK2CPP_HasGen()` 查询（M4 后期再加，首切片不用）。
- 与 `MK2CPP_GEN_DIR` 完全正交：`-DMK2CPP_GEN_DIR=...` 可同时带 hand；
  也可以只带 hand 跑无 gen 构建（未覆盖 PC 回退解释器）。

---

## 3. 开关矩阵

### 3.1 五格行为定义（目标语义）

| # | 命令行 | `mk2cpp_enabled` | 主 CPU 执行 | PCM 设备模式 | 说明 |
|---|---|---|---|---|---|
| 1 | （无 flag） | 0 | 解释器 | stock | 与现状逐字节一致（红线） |
| 2 | `-mk2cpp` | 1 | hand→gen→解释器 | stock（`pcm_ext_active=0`） | M1–M3 模式；hand 若链接则必须 null 等价（A4.1 准备） |
| 3 | `-voices:n`（n≠28，无 `-mk2cpp`） | 0 | 解释器 | stock | 现状惰性；建议打印警告"需要 -mk2cpp 才生效"；`n=28` 时完全无动作 |
| 4 | `-mk2cpp -voices:n`（n≠28） | 1 | hand→gen→解释器 | **ext（`pcm_ext_active=1`，reg_slots=n）** | 原生引擎 + GT 阶段 1 设备模式，M4 主用例 |
| 5 | `-voices:n` 单独（再次强调） | 0 | 解释器 | stock | 与 #3 相同；两个 flag 的**顺序无关**（见下） |

补充：
- `-voices:n` 出现在 `-mk2cpp` 之前或之后应同义。现状顺序无关（只写全局），
  但 `-mk2cpp` 解析时立刻 `MK2CPP_Init`（`src/mcu.cpp:1980`）会打印
  未含 voices 的信息；`MK2CPP_Configure(argc, argv)` 统一在 argv 解析后决策/打印。
- `-voices:28` 即 stock 值：`pcm_ext_enabled=0`，任何格都不进入 ext 模式；
  与 #2 等价（可作 null 基线命令）。
- `-voices:n` 且 `n` 越界：维持现状忽略 + 警告（`src/mcu.cpp:1996-1999`）。

### 3.2 原生引擎何时启用（冻结）

**hand 覆盖表在 `-mk2cpp` 下始终参与分派（含 `-voices:28`）；
扩展设备模式 `pcm_ext_active=1` 仅当 `-mk2cpp && -voices:n` 且 n≠28。**

理由：
1. A4.1 要求 n=28 音频 null；只有让 hand 在 n=28 下真实执行，才能在每个
   切片落地时立刻验证"stock 等价"。若 hand 只在 n≠28 启用，n=28 的回归
   测不到 hand 代码，风险后移。
2. 不需要新增用户 flag；调试用 `-mk2cpp-hand:0|1`（冻结开关，默认 1）即可做
   同二进制 A/B（hand 关掉=纯 gen，用于 diff 首个分叉 PC）。
3. `pcm_ext_active` 语义 = "原生扩展模式已激活"，由 `MK2CPP_PostReset()` 设置，
   不再依赖 `MCU_PatchROM`（`src/mcu.cpp:1658-1668` 保持空实现或删除死注释）。
4. hand 代码内部必须自检：n>28 的行为在 `pcm_ext_active` 后才启用；
   n=28 走与 stock 相同的写序列（mask 只写主窗口，不写 0xe800）。

激活路径（二选一，建议主用 (a)）：
- (a) hand 覆盖 boot 的 PCM 初始化路径（`0x41333`/`0x4134c` 系列），
  由 hand 写 `config_reg_3d = n-1` 并完成通道初值——H8 可观测行为完整；
- (b) 兜底：`MK2CPP_PostReset()` 直接
  `pcm_ext_enabled=1; pcm_ext_voices=n; pcm_ext_active=1;
  pcm.config_reg_3d=(uint8_t)(n-1);`（实验/首个切片用，不改变 H8 trace 的
  那些写入点，注意会让 O4 config 观测与 hand 路径不一致，只限调试）。

### 3.3 与 GT 阶段 1 `pcm_ext_*` 的关系

- GT 阶段 1 是 **设备模型契约**：mask 存储/latch、0xe800 窗口、`0x3f`
  效果别名、`reg_slots` 公式、IRQ 完整 slot、采样节奏钳制（§1.6 行号）。
  M4 **不重写这些**，hand 通过 `PCM_Write/PCM_Read/PCM_WriteExt/PCM_ReadExt`
  使用它们（保持 latch 与 IRQ ack 的顺序语义）。
- `pcm_ext_enabled`（flag）与 `pcm_ext_active`（生效）的现有分工保留；
  `pcm_ext_active` 的置位者 = 原生引擎激活（`MK2CPP_PostReset()`）。
- 写 mask 的顺序必须保持 R9：**先写扩展窗（pending 高位），再写主窗口，
  最后依赖主窗口读回触发 latch**（latch 只在主窗口读 0..3 时发生，
  `src/pcm.cpp:215-220`；扩展读返回 latched 值不触发 `:301-302`）。
- `PCM_ReadExt(0x21)` 已提供 active 查询（`src/pcm.cpp:306`），hand 可用来
  自检激活状态。
- 阶段 1 快照缺口 `b_ram/b_ram2` 不入 `state_save`（`src/mcu.cpp:1225-1263`
  未含；cosim spec B1/E3）：**建议 hand 原生状态不要放 page6 SRAM**，
  而是放 hand 自己的 C++ 结构，并新增 `MK2CPP_StateSave/Load` 参与快照
  （M4 待办，首个切片若只改 mask 写序列则无此问题）。

### 3.4 未覆盖 PC 的回退策略

- 保持现状三层：**hand 命中 → gen 命中 → 解释器（`mk2cpp_mixed=1`）**；
  `mixed=0` 时保持 `exit(1)` fail-fast（`mk2cpp.cpp:78-79,108-109`）。
- 新增：hand 命中时若 hand 代码内部需要执行某个"已翻译 PC 的语义"，
  不允许直接调用 `mk2c_*` 符号（无 gen 构建会链接失败）；只能用
  GT helper / 设备 API 重写。
- 计数：hand miss 落到 gen 时不影响现有 fallback 口径；建议
  `TranslatedCount` 只统计 gen（保持 M1–M3 断言可复用），hand 单独计数。
- per-PC fallback 直方图（现状没有）：M4 建议加 `-mk2cpp-stats <file>`，
  至少输出 hand/gen/interp 三类计数与未命中 flat top-N（可选）。

---

## 4. 单指令约束与调度

### 4.1 现状：`MK2CPP_Step` = 恰好一条 H8 指令

宿主每次迭代只调用一次 `MK2CPP_Step`（`src/mcu.cpp:1451-1452`），
gen 函数"one function per PC, exactly one GT instruction each"
（`mk2cpp/src/gen/mk2c_r1.cpp:7`）；一条指令的全部可观测副作用
（内存、标志、pc/cp）必须在该函数内完成。宿主独占负责：

- 中断轮询与 `ex_ignore`（`:1444-1447`）；
- `cycles += 12`（`:1457`）；
- trace / 快照 / 设备更新（`:1459-1446`）。

SM 侧更严格：每条 SM 指令后固定 `sm.cycles += 48` 并跑
`SM_UpdateTimer/UART`（`src/submcu.cpp:1442-1445`），所以 SM 的 hand
必须同样是单指令（若将来做 SM hand）。

### 4.2 整例程替换（多指令）为什么危险

若 hand 在例程入口接管、一次执行 N 条指令后才返回：

1. **周期记账**：宿主仍会 `+=12` 一次；hand 只能补 `+12*(N-1)`。
   若有任何路径忘记补/重复补，`mcu.cycles` 立即漂移，PCM/TIMER/SM 全部错相。
2. **中断轮询位置**：stock 每指令前轮询一次（`src/mcu.cpp:1444-1447`）；
   N 条之间不再轮询，等于人为延迟中断 N 个指令周期。PCM IRQ（IRQ0）、
   FRT/定时器 IRQ 的响应点变化，`d15c`/`ex_ignore` 语义变化。
3. **trace 行数**：`-tracepc` 每个宿主 step 一行（`:1459-1470`），
   整例程后 `m` 行数骤减 → M1–M3 baseline 对照失效；cosim L0 无法再用。
4. **`ex_ignore`**：RTE/LDC/ORC/ANDC 设置的"跳过一次轮询"在块内被吞掉
   （本身无轮询可跳），语义不再逐位等价。
5. **设备更新粒度**：`PCM_Update/TIMER_Clock/SM_Update/Analog` 宿主每
   step 一次；cycles 跳跃后它们内部 catch-up 循环数值上通常一致
   （PCM 的内部步进只依赖 `pcm.cycles`），但 IRQ/GA 中断的"何时被看到"
   相对指令边界改变。
6. **返回地址/栈**：例程入口被 `pjsr` 调用时返回地址已在栈顶；
   hand 必须完整模拟例程的 ret/栈效果，任何栈偏移错误会累积。
7. **undo/回滚**：一旦 hand 多指令中途需要"中止"（例如取到中断要提前退出），
   C++ 层没有 unwind 协议，无法像生成块那样 `return RT_PC_SET + irq_abort`
   （`mk2cpp/docs/06_h8lift_design.md:265-308` 的块级协议在当前实现中并未
   启用；当前实现是每 PC 一函数）。

### 4.3 建议的入口/出口协议

分三级，M4 只允许 L0，L1 在 null 通过后按需启用，L2 冻结。

- **L0（默认，本阶段唯一允许）**：per-PC hand。
  `mk2cpp_fn` 不变；契约与 gen 完全相同（§4.1）；必须自行消耗正确的取指
  字节、写回 pc/cp、置正确标志；不得碰 cycles/中断/设备。
- **L1（整例程 hook，后置）**：签名扩展为
  `uint32_t (*mk2cpp_hand_routine_fn)(void)`，返回**本次执行的 H8 指令条数**
  n（n≥1）。宿主协议（拟议）：

  ```c
  /* mk2cpp.cpp 内部，Step 命中 routine hook 时 */
  uint32_t n = rfn();
  mcu.cycles += 12u * (n - 1u);   /* 宿主随后的 +=12 凑满 n*12 */
  ```

  - hand 例程必须保证：入口时的栈/寄存器与真实例程一致；每个被"跳过"的
    指令边界若可能被中断，例程需自行处理（见下）；结束时 pc/cp 与 stock
    例程返回后一致。
  - 为了不破坏中断语义，L1 只允许用在**固件已屏蔽中断（IML=7 或
    `ex_ignore` 保护）**的短例程上，并在 hand 头注释中写明依据；
    否则必须用 L0。
  - trace：接受 L1 下 `m` 行减少；该场景**不允许**再用 trace 逐行对照，
    验收改为音频 hash + 快照状态（10 的 oracle；`mk2cpp/docs/04_cosim_spec.md:517-524`）。
  - 可选逃生口：hand 循环中每条指令后调用
    `MK2CPP_HandBoundary()`，内部语义 = GT `:1444-1447`（轮询或清
    `ex_ignore`）；若取到中断则返回 1 让例程立即收尾并由 hand 设置
    `pc/cp` 到向量入口。**该回调不能 unwind C++ 栈**，因此例程必须自己
    逐层检查返回值并退出；复杂度高，M4 不建议。
- **L2（块/批处理）**：冻结，除非将来重做宿主调度。当前实现在库形态下
  没有 `RT_PreInstr/PostInstr/irq_abort`（那些是 06 文档的独立 `src/rt`
  设计，未随 GT 集成落地）。

兼容性建议：`mk2cpp_hand_ent` 预留 `flags`/`n_instr` 字段，L0 填
`flags=0`；L1 填 `HAND_ROUTINE`。ABI 结构先内部化，不暴露在 `mk2cpp.h`。

### 4.4 SM 侧

- SM hand 只能 L0；`MK2CPP_SM_Step` 前后 `SM_UpdateTimer/UART` 由宿主做
  （`src/submcu.cpp:1442-1445`），hand 碰 sm 设备状态需通过 GT 的
  `SM_*` helper。
- 若将来 SM 需要整段替换（如 UART 协议 native 化），必须评估 5× 时钟
  与每指令 UART/timer 推进，优先级低于主 CPU voice/PCM。

### 4.5 风险清单（供实现时登记）

| # | 风险 | 缓解 |
|---|---|---|
| R1 | T 状态 trace 异常未在 gen/hand 后检查 | 把检查移入 `MK2CPP_Step` 尾部（`src/mcu.cpp:1085-1088` 等价） |
| R2 | 多指令 hand 的 cycles 漏加/重复 | L1 协议：host 加 `12*(n-1)`；测试字段 `mcu.cycles` 快照对拍 |
| R3 | 中断延迟改变 PCM/TIMER 相位 | L1 只用于 IML=7 短例程；n=28 必须 L0 |
| R4 | hand 状态不在快照里 | 新增 `MK2CPP_StateSave/Load` 并纳入 `state_save/load`（`src/mcu.cpp:1225-1303`） |
| R5 | `pcm_t` 尺寸变化使旧快照失效 | 沿用 ABI guard（`mk2cpp/docs/04_cosim_spec.md:267-270`） |
| R6 | hand 与 gen 对同一 PC 冲突 | B 方案注册期报错 |
| R7 | `MK2CPP_Init` 早于 ROM/`PCM_Reset` | 注册纯静态；激活放 `MK2CPP_PostReset` |
| R8 | 无 gen 构建 hand 引用 `mk2c_*` | 编译规约：hand 不得引用 gen 符号 |

---

## 5. 第一个最小可测切片

### 5.1 候选（结合 07/08 领域）

| 候选 | 地址/证据 | 领域 | 规模 | 现成 oracle | 评价 |
|---|---|---|---|---|---|
| PCM enable/disable flush | `0x5525/0x5527`、`0x5662/0x5664`（`tools/docs/task_irq_map.md:499` O5；`tools/docs/mk2_polyphony_256.md:234`） | 08 mask/PCM 编程 | 各 2~3 条指令，叶节点 | `-pcmtrace`（`pc=00:5527/5664` @144M）+ 快照 `pcm.voice_mask` | **首选** |
| pool-init | `0x40462-0x40586`，5 个循环（`tools/docs/voice_bounds_inventory.md:40-47`；`tools/docs/plan_256.md:66`） | 07 pool | 中（21 数组 ×N） | 快照 `a42d=N`、数组内容 | 第二切片 |
| alloc/free | alloc `0x1823/0x187e`、free `0x19c4`（`tools/docs/voice_memory_map.md:359-366`） | 07 pool | 中 | 快照 chain/`a42d` | 第三切片 |
| 32-bit mask 累积 | `0x1ad3`（`tools/docs/voice_bounds_inventory.md:35`） | 08 | 小循环 | mask 快照 | 可与 flush 同期 |

### 5.2 推荐：PCM enable/disable flush（08 领域）

理由：
1. **叶子语义**：`0x5525-0x5529` 是"disable flush"（`r3=d150 & ~d154`、
   `r4=d152 & ~d156` → `movsw @(br,$00/$02)`），`0x5662-0x5666` 是
   "enable flush"（或值后写）；输入输出可枚举，无子调用、无循环。
2. **直接压中 M4 核心契约**：256-bit mask。n=28 时写主窗口（stock）；
   n>28 时按 R9 先写 `PCM_WriteExt` 高位、再写主窗口并读回 latch。
   一次就把"hand 如何与 GT 阶段 1 对接"验证清楚。
3. **有可复现 oracle**：`-mocknote 144000000 -pcmtrace` 在
   c≈144.05M 产生 `pc=00:5527`（disable）与 `pc=00:5664`（enable）写序列
   （`tools/docs/task_irq_map.md:204-206,499`）；n=28 下逐条比对
   `pcm_trace.log` 与无 hand 运行**完全一致**。
4. **失败可控**：即使 hand 写错，影响局限在 mask 写序列，回退开关一关即恢复。

### 5.3 实现形态与验收

- 形态：L0（per-PC）两个 hand 函数挂在 `0x5525..0x5529`、
  `0x5662..0x5666` 全 PC（这些地址在 gen 中也是独立 PC 函数）。
  hand 内部直接调 `PCM_Write/PCM_WriteExt/PCM_Read`，按 `pcm_ext_active`
  分 stock/ext 两路；n=28 时生成与 stock 等价的主窗口写。
- 验收（n=28 null，必过）：
  1. `-mk2cpp -mocknote 144000000 -pcmtrace -savesnap 150000000` 与
     不带 `-mk2cpp` 的 `pcm_trace.log` **逐行一致**（reg/val/pc/cyc）；
  2. `-mk2cpp -demo -tracepc ... 200000000 202000000` 与 baseline
     `tools/baselines/trace_200m_base.txt` 0 分歧（boot 0–3M 同）；
  3. `-hashdump 202000000` 的 `hash.pcm` 与 stock 一致；
  4. `MK2CPP_HandHitCount() > 0` 且 hand 条目在 trace 中确实被走到
     （建议 `MK2CPP_DumpStats` 打印 per-entry hits）；
  5. `-mk2cpp-hand:0` 重跑产物与 hand on 的产物**字节一致**（证明
     hand 当前切片 null 等价）。
- 验收（n>28 冒烟，非 gate）：`-mk2cpp -voices:64 -mocknote ...`：
  观察 `pcm.voice_mask[4..]` 在 0xE800 写后变化、无复位、`isr` 心跳增长。
  注意此切片尚未做 pool/alloc，64 声部不会被分配，冒烟只证明 mask 通路
  （明确写进报告，避免过度解读）。

### 5.4 回退开关

- 运行时：`-mk2cpp-hand:0`（冻结；`-mk2cpp-hand:1` 恢复）→ `mk2cpp_hand_enabled=0`，
  hand 表跳过，纯 gen（或无 gen 时纯解释器）。
- 单条目：hand 内部开关宏/常量 `#define MK2CPP_HAND_PCM_ENABLE 1`；
  关闭时该条目退化为透明（不注册），不触发重编译其他文件亦可由
  `MK2CPP_HandFillTables` 条件注册。
- 构建级：不生成 hand 文件即回到 M1–M3 状态（CMake glob 为空）。

### 5.5 后续切片顺序（冻结）

1. **PCM enable/disable flush**（§5.2–§5.4，首选切片）；`0x1ad3` mask 累积
   （08 领域）与 flush 配套、可同期进行，仍无 pool 依赖；
2. pool-init `0x40462/0x404c8/0x404d9/0x40508`（07 领域）——引入 hand 原生
   voice pool 状态与 `Voice[256]` 布局；需要决定状态快照接口（R4）；
3. alloc/free `0x1823/0x187e/0x19c4`（07 领域）——分配链 native 化；
4. per-voice materialize `0x53eb` 与 PCM 写路径 `0x54fb/0x5533/0x5626`
   （07/08 交界）——n>28 真正出声；
5. 最后才考虑 L1 例程 hook 与性能批处理（n=28 null 通过后；§4.3）。

---

## 6. 需要修改的已入库文件清单与拟议 diff 概要（不实施）

| 文件 | 改动性质 | 概要 | 约估行数 |
|---|---|---|---|
| `mk2cpp/include/mk2cpp.h` | ABI 增量 | `mk2cpp_hand_enabled`；`MK2CPP_HandRegister/HandFillTables/HandCount/HandHitCount/Configure/PostReset/DumpStats`（§2.5） | +12 |
| `mk2cpp/src/mk2cpp.cpp` | 核心 | hand 稀疏表+二分；`CanStep/Step` hand 优先；T 检查移入 `Step` 尾部；Init 调 hand fill + 计数打印；Version 组串；Configure/PostReset/DumpStats | +100~150 |
| `mk2cpp/src/hand/pcm_enable.cpp` | 新增（入库） | 首个切片两个 flush 的 L0 实现 + 文件头版本标记（§2.6） | ~150 新 |
| `CMakeLists.txt` | 构建 | hand glob + `MK2CPP_HAS_HAND`（§2.7） | +8 |
| `src/mcu.cpp` | 集成钩子 | argv 解析后 `MK2CPP_Configure(argc, argv)`（~`:2190` 前）；`PCM_Reset()` 后 `MK2CPP_PostReset()`（`:2484` 之后）；`-mk2cpp-hand:0\|1` 解析；帮助文本 2163-2164 去掉 ROM-patch 措辞；hashdump 处 `MK2CPP_DumpStats()`（可选） | +8~12 |
| `src/submcu.cpp` | 无（B 方案） | SM hand 未来再议；调用点 `:1436` 不动 | 0 |
| `src/pcm.cpp` / `src/pcm.h` | 视需要 | 首切片不需要。若要给激活加显式 API（替代直接写 `pcm_ext_*` 全局），可加 `PCM_SetExtMode(int n)`；非必须 | 0~10 |
| `mk2cpp/docs/02_conventions.md` | 文档 | 补 hand 命名/文件头/冲突规则与三层回退口径 | 文档 |
| `mk2cpp/docs/01_architecture.md` | 文档 | 架构图 hand→gen→interp 与 `pcm_ext_active` 新语义 | 文档 |

拟议 diff 片段（示意，不实际应用）：

```diff
--- a/mk2cpp/src/mk2cpp.cpp
+++ b/mk2cpp/src/mk2cpp.cpp
@@
 static mk2cpp_sm_fn mk2cpp_sm_tab[0x10000];
 static uint32_t mk2cpp_sm_fallbacks = 0;
 static uint32_t mk2cpp_sm_translated = 0;
+
+int mk2cpp_hand_enabled = 1;
+typedef struct { uint32_t flat; mk2cpp_fn fn; uint32_t hits; } mk2cpp_hand_ent;
+static mk2cpp_hand_ent mk2cpp_hand[256];
+static uint32_t mk2cpp_hand_n = 0;
+static uint32_t mk2cpp_hand_hits = 0;
@@
 void MK2CPP_Step(void)
 {
     uint32_t flat = ((uint32_t)mcu.cp << 16) | mcu.pc;
-    mk2cpp_fn fn = mk2cpp_lookup(flat);
+    mk2cpp_fn fn = mk2cpp_hand_lookup(flat);
+    if (fn) { fn(); mk2cpp_hand_hits++; mk2cpp_translated++; goto tail; }
+    fn = mk2cpp_lookup(flat);
     if (fn)
     {
         fn();
         mk2cpp_translated++;
-        return;
+        goto tail;
     }
     mk2cpp_fallbacks++;
     if (mk2cpp_mixed)
     {
         MCU_ReadInstruction();
-        return;
+        goto tail;
     }
     fprintf(stderr, "mk2cpp: no translation for %08x and mixed mode is disabled\n", flat);
     exit(1);
+tail:
+    if (mcu.sr & 0x8000)                 /* STATUS_T; 对齐 src/mcu.cpp:1085-1088 */
+        MCU_Interrupt_Exception(EXCEPTION_SOURCE_TRACE);
 }
```

（实际实现需 include `mcu_interrupt.h`；`goto` 仅为 diff 示意，落地用 helper 函数。）

```diff
--- a/src/mcu.cpp
+++ b/src/mcu.cpp
@@ void MCU_PatchROM(void)
+    /* M4: ROM patch route removed; native hand engine activates via MK2CPP_PostReset. */
 }
@@ int main(...)
     if (demo_seq_enabled && mocknote_enabled) { ... }
+    if (mk2cpp_enabled)
+        MK2CPP_Configure(argc, argv); /* after argv, before ROM load */
@@
     MCU_Reset();
     SM_Reset();
     PCM_Reset();
+    if (mk2cpp_enabled)
+        MK2CPP_PostReset();          /* 激活 pcm_ext_active / hand reset */
```

---

## 7. 待定项

| # | 待定项 | 现状/方向 |
|---|---|---|
| S1 | `-voices:n` 单独（无 `-mk2cpp`）的行为 | 维持惰性+警告（建议）、报错拒绝，还是等价于 `-mk2cpp -voices:n` |
| S2 | 引擎状态归属 | hand 原生 `Voice[256]`（需 reset/save/load 钩子、快照 ABI 变化；与 07 Q7/Q8 强相关）还是以 GT `sram`/page6 影子为真值源 |
| S3 | L1 整例程 hook 范围与周期协议 | L0 先行已冻结（n=28 null 通过前不得启用）；L1 是否/何时进入 M4、哪些 IML=7 短例程适用；周期用"host 加 `12*(n-1)`"还是 hand 内部记账 |
| S4 | T 检查缺口修复 | 是否把 trace 异常检查移入 `MK2CPP_Step`（改变 gen 路径极端窗口行为；已有 oracle 窗口应不受影响） |
| S5 | `-mk2cpp-stats <file>` 诊断开关 | 是否落地 per-PC 直方图；`-mk2cpp-hand:0\|1` 与 per-entry hits 已冻结/已设计 |
| S6 | hand 版本门禁 | ROM sha 不匹配时仅警告（建议）还是拒绝启动 |

与 07 §6、08 §7、10 §6 的待定项互为补充。

---

## 8. 对现有 mk2cpp ABI 的改动量评估

- **不改**：`mk2cpp_fn` 签名、`MK2CPP_RegisterFn/CanStep/Step`、
  SM 三件套、现有 4 个计数 API、`mk2cpp_enabled/mk2cpp_mixed` 语义。
- **新增**（约 7 个函数 + 1 全局）：
  `mk2cpp_hand_enabled`、`MK2CPP_HandRegister`、`MK2CPP_HandFillTables`
  （由 `MK2CPP_HAS_HAND` 门控）、`MK2CPP_HandCount`、`MK2CPP_HandHitCount`、
  `MK2CPP_Configure(argc, argv)`、`MK2CPP_PostReset`、`MK2CPP_DumpStats`。
- **行为兼容**：hand 表为空时，`CanStep/Step` 输出与现状逐位相同；
  `src/mcu.cpp:1451` 调用形状不变；默认构建（无 hand 文件、无
  `MK2CPP_GEN_DIR`）与当前二进制行为等价。
- **风险最大项**：T 检查移入 `Step`（R1）与 `pcm_ext_active` 激活钩子
  （R7）；两者都可在 `-mk2cpp-hand:0` / 无 `-voices` 下退化为现状。
- 结论：**低风险、可回退**；主要成本在 hand 语义实现本身（07/08），
  不在分派层。
