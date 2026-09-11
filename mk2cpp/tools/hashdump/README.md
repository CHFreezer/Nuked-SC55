# hashdump — deterministic full-state dump for mk2cpp co-simulation

Purpose: `mk2cpp` runs GT in two modes (default interpreter vs `-mk2cpp`
translated core, with mixed fallback) and needs to compare the **full machine
state** at checkpoints, not just `-tracepc` PC traces. `-hashdump` writes one
deterministic, human-readable text dump with FNV-1a 64 hashes over the raw
state ranges.

## Usage

```
nuked-sc55.exe -mk2 [-mk2cpp] -hashdump <cycles> <file>
```

* Off by default; flag absent leaves execution untouched (`hashdump_at == 0`).
* Fires **once**, the first time `mcu.cycles >= <cycles>`.
* Trigger point is the same point as `-savesnap` in `work_thread`: after the
  instruction step and the `mcu.cycles += 12` increment (and after the
  tracepc/demo/mocknote bookkeeping), **before** `PCM_Update`, `TIMER_Clock`,
  `SM_Update`, so both modes dump at the identical scheduling point.
* The emulator keeps running after the dump (GT never self-exits).

## Format (`hashdump v1`)

One `key = value` per line. Raw field values first, then hashes:

```
version = hashdump v1
requested_cycles = 4000000
mcu.cp = 00
mcu.pc = 0492
mcu.sr = 0000
mcu.cycles = 4000008
pcm.config_reg_3c = c3
pcm.config_reg_3d = 7b
pcm.irq_assert = 0
pcm.cycles = 4000100
sm.pc = f151
sm.cycles = 20000016
hash.mcu = e273f6a40319b3eb
...
```

FNV-1a 64: basis `0xcbf29ce484222325`, prime `0x100000001b3`, printed as
16 lowercase hex digits (`hash.<name> = <16 hex>`). Hashed ranges (raw bytes):

| key | object |
|---|---|
| `hash.mcu` | `mcu` (`sizeof(mcu_t)`) |
| `hash.ram` / `hash.sram` | `ram`, `sram` |
| `hash.dev_register` | `dev_register` |
| `hash.frt` / `hash.timer` / `hash.timer_cycles` / `hash.timer_tempreg` | FRT + timer state |
| `hash.ad_val` | ADC latches (`analog` array does not exist in GT) |
| `hash.analog_end_time` | analog scan end time |
| `hash.ga_int` / `hash.io_sd` / `hash.mcu_p0_data` / `hash.mcu_p1_data` | gate array ints, switch/IO latches |
| `hash.pcm` | whole `pcm` struct (`pcm.h`), incl. `eram`, voice masks, `fstate` |
| `hash.sm` | whole `sm` struct (`submcu.h`) |
| `hash.sm_ram` / `hash.sm_shared_ram` / `hash.sm_access` / `hash.sm_device_mode` | SM RAM/register views |
| `hash.sm_p0_dir` / `hash.sm_p1_dir` / `hash.sm_cts` | SM port/status |
| `hash.sm_timer_cycles` / `hash.sm_timer_prescaler` / `hash.sm_timer_counter` | SM timer |
| `hash.uart_buffer` / `hash.uart_rx_byte` | main↔SM UART path |
| `hash.lcd_state` | `LCD_StateSave()` byte stream via `tmpfile()` |

## Fields not included and why

* `analog`: no such array exists in GT (`mcu.cpp` only has `analog_end_time`,
  which is hashed).
* `lcd_enable`: declared `static` in `src/lcd.cpp`, not exported, so it cannot
  be printed directly. It is covered by `hash.lcd_state` (last `uint32_t` of the
  `LCD_StateSave` stream). Add an accessor if a raw value is ever needed.
* Cross-build comparison is not supported: hashes include padding bytes and
  `float`/`double` bit patterns of the same binary. Compare **same-binary**
  runs/modes only.

## Self-test evidence (2026-09-11, Release Ninja clang-cl)

```powershell
cmake --build build
build\nuked-sc55.exe -mk2     -hashdump 4000000 out\h1.txt   # interpreter
build\nuked-sc55.exe -mk2     -hashdump 4000000 out\h2.txt   # rerun
build\nuked-sc55.exe -mk2cpp  -hashdump 4000000 out\h3.txt   # pure fallback
```

```powershell
Get-FileHash out\h1.txt, out\h2.txt, out\h3.txt -Algorithm SHA256
```

All three = `DA9025BCFF0EA3CF487361DDD7E2F268695153389BA42EBC67325DC5603C72D9`.
No-flag run stays alive (no crash, empty stderr). Build configured without
`MK2CPP_GEN_DIR`, so `-mk2cpp` links no translated code and uses the
interpreter fallback path.
