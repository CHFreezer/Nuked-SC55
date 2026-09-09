#!/usr/bin/env python3
# Cross-validate h8dasm decode against ground truth (VM trace == GT trace).
#
# Phase A (length/target): every executed PC's decoded length must agree with
#   the observed sequential transitions in the PC trace; every observed
#   non-sequential transition must match the decoded branch target.
#   usage: verify_dasm.py --mach mach.txt --trace vm_pc.txt
#
# Phase B (semantics): with a full-register trace, verify register/SP/sr
#   effects of every executed instruction against the VM's semantics
#   (tools/vm/h8vm_body.c). Memory-operand values are unknown, so only
#   register-deterministic effects are checked.
#   usage: verify_dasm.py --mach mach.txt --regtrace vm.log
import sys, re, collections

MACH = re.compile(r'^([0-9a-f]{8}) (\d+) (\d+) ([0-9a-f]{2}) ([0-9a-f]{4}) ([0-9a-f]{2}) (\d+) (\d+) (\d+) (\d+) (\d+)$')

def load_mach(path):
    dec = {}
    with open(path) as f:
        for line in f:
            m = MACH.match(line.strip())
            if not m:
                print(f"MACH parse fail: {line!r}")
                sys.exit(1)
            pc, ln, kind, tp, to, top, reg, siz, ocode, ore, ext = m.groups()
            dec[int(pc, 16)] = dict(
                len=int(ln), kind=int(kind), tpage=int(tp, 16), toff=int(to, 16),
                top=int(top, 16), reg=int(reg), siz=int(siz),
                ocode=int(ocode), ore=int(ore), ext=int(ext))
    return dec

def load_trace(path, with_regs=False):
    """Return list of (cycles, cp, pc[, regs dict])."""
    out = []
    RL = re.compile(r'^m (\d+) ([0-9a-f]{2}):([0-9a-f]{4})(?: sr=([0-9a-f]{4})'
                    r' r0=([0-9a-f]{4}) r1=([0-9a-f]{4}) r2=([0-9a-f]{4}) r3=([0-9a-f]{4})'
                    r' r4=([0-9a-f]{4}) r5=([0-9a-f]{4}) r6=([0-9a-f]{4}) r7=([0-9a-f]{4})'
                    r' br=([0-9a-f]{2}) dp=([0-9a-f]{2}) ep=([0-9a-f]{2}) tp=([0-9a-f]{2}))?$')
    with open(path) as f:
        for line in f:
            m = RL.match(line.strip())
            if not m:
                continue
            cyc, cp, pc = int(m.group(1)), int(m.group(2), 16), int(m.group(3), 16)
            if with_regs:
                if m.group(4) is None:
                    continue
                g = m.groups()[3:]
                regs = dict(sr=int(g[0], 16))
                for i in range(8):
                    regs[f'r{i}'] = int(g[1 + i], 16)
                for k in range(4):
                    regs[('br', 'dp', 'ep', 'tp')[k]] = int(g[9 + k], 16)
                out.append((cyc, cp, pc, regs))
            else:
                out.append((cyc, cp, pc))
    return out

def phaseA(dec, tr, vec_next=frozenset()):
    # Model (validated against VM loop order in h8vm_main.c):
    #   line k records (pc, state) AFTER processing iteration k, where each
    #   iteration is: [handle one pending interrupt/exception] then
    #   [execute one instruction if not sleeping]. So between consecutive
    #   lines the observed next PC is one of:
    #     a) fall-through / branch target of the instr at s, or
    #     b) vector + len(first instr at vector)   (interrupt dispatch), or
    #     c) s itself                              (CPU slept, nothing ran)
    bad_len, bad_tgt, unexp = [], [], []
    missing = 0
    interrupts = 0
    sleeps = 0
    for i in range(len(tr) - 1):
        s = (tr[i][1] << 16) | tr[i][2]
        u = (tr[i + 1][1] << 16) | tr[i + 1][2]
        d = dec.get(s)
        if d is None:
            missing += 1
            continue
        if u == s:
            sleeps += 1
            continue
        tgt = (d['tpage'] << 16) | d['toff']
        scp = s >> 16; soff = s & 0xffff
        seq = (scp << 16) | ((soff + d['len']) & 0xffff)
        if u in (seq, tgt):
            continue
        if u in vec_next:
            # interrupt/exception dispatch: instr at s never executed
            interrupts += 1
            continue
        if d['kind'] == 3:
            bad_len.append((s, d, u))
        elif d['kind'] in (1, 2):
            bad_tgt.append((s, d, u))
        elif d['kind'] == 0:
            unexp.append((s, d, u))
        # kind 4/5 (ret / reg-indirect): any target allowed
    print(f"[INFO] trace transitions from PCs missing in mach set: {missing}")
    print(f"[INFO] sleep no-op transitions: {sleeps}")
    print(f"[INFO] interrupt/exception dispatch transitions: {interrupts}")
    report("BAD_FALLTHRU_OR_LEN (cond: next is neither s+len nor target; none: next != s+len)", bad_len, 30)
    report("BAD_BRANCH_TARGET (uncond/call: observed dst != decoded target)", bad_tgt, 30)
    report("UNEXPECTED_JMP from kind=0 instr (int/trap/undecoded transfer?)", unexp, 30)
    return bad_len, bad_tgt, unexp

def report(name, items, limit):
    if not items:
        print(f"[OK]   {name}: 0")
        return
    print(f"[FAIL] {name}: {len(items)}")
    for it in items[:limit]:
        s, d, extra = it[0], it[1], it[2]
        print(f"        {s:08x} len={d['len']} kind={d['kind']} tgt={d['tpage']:02x}:{d['toff']:04x} "
              f"top={d['top']:02x} reg={d['reg']} siz={d['siz']} ocode={d['ocode']} ore={d['ore']} ext={d['ext']} | {extra}")

def die(msg):
    print(msg)
    sys.exit(1)

def rom_byte(rom1, rom2, addr):
    if addr >> 16 == 0 and addr & 0xffff < 0x8000:
        return rom1[addr & 0xffff]
    idx = addr & 0x3ffff
    if addr & 0x80000:
        idx |= 0x40000
    return rom2[idx & 0x7ffff]

def vector_set(rom1, rom2):
    # MCU_Read32 -> 32-bit big-endian; StartVector: cp = v32>>16, pc = v32&0xffff
    out = []
    for v in range(64):
        b = [rom_byte(rom1, rom2, v * 4 + k) for k in range(4)]
        v32 = (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]
        addr = (((v32 >> 16) & 0xff) << 16) | (v32 & 0xffff)
        out.append((addr, v))
    return out

def main():
    args = sys.argv[1:]
    mach = tr = reg = rom1p = rom2p = None
    i = 0
    while i < len(args):
        if args[i] == '--mach': mach = args[i + 1]; i += 2
        elif args[i] == '--trace': tr = args[i + 1]; i += 2
        elif args[i] == '--regtrace': reg = args[i + 1]; i += 2
        elif args[i] == '--rom1': rom1p = args[i + 1]; i += 2
        elif args[i] == '--rom2': rom2p = args[i + 1]; i += 2
        else: die(f"unknown arg {args[i]}")
    dec = load_mach(mach)
    print(f"mach: {len(dec)} decoded PCs")
    total = 0
    if tr:
        tr = load_trace(tr)
        print(f"trace: {len(tr)} main instr")
        vec_next = frozenset()
        if rom1p and rom2p:
            r1 = open(rom1p, 'rb').read(); r2 = open(rom2p, 'rb').read()
            vec_next = set()
            novlen = []
            for v, num in vector_set(r1, r2):
                dv = dec.get(v)
                if dv is None:
                    novlen.append((v, num))
                    continue
                vec_next.add((v >> 16) << 16 | ((v & 0xffff) + dv['len']) & 0xffff)
                if dv['kind'] in (1, 2, 3, 4, 5):
                    vec_next.add((dv['tpage'] << 16) | dv['toff'])
            if novlen:
                print(f"[WARN] vector entries not in mach set (no length; dispatch via them uncheckable): "
                      + ", ".join(f"v{num}={v:06x}" for v, num in novlen))
        bad_len, bad_tgt, unexp = phaseA(dec, tr, vec_next)
        total += len(bad_len) + len(bad_tgt) + len(unexp)
    if reg:
        rt = load_trace(reg, with_regs=True)
        print(f"regtrace: {len(rt)} main instr with regs")
        import os
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from semantics import phaseB
        phaseB(dec, rt)
        import semantics
        total += semantics.fail_count
    print(f"\nTOTAL FAILURES: {total}")
    sys.exit(1 if total else 0)

if __name__ == '__main__':
    main()
