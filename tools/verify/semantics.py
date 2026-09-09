#!/usr/bin/env python3
# Phase B: semantic verification of h8dasm-decoded instructions against a
# full-register VM trace (== GT). Implements the exact semantics of
# tools/vm/h8vm_body.c. For each executed instruction we check:
#   - register values that are determinable without unknown memory reads
#   - sr N/Z/V/C nibble (touched -> must match; untouched -> must be equal)
#   - next-PC for direct transfers
# Memory-operand values are unknown; those effects are skipped, but any
# register the instruction does NOT touch must be unchanged.
import sys

fail_count = 0
fails = []
import collections
fail_pat = collections.Counter()

N, Z, V, C = 0x08, 0x04, 0x02, 0x01

def W(v): return v & 0xffff
def B(v): return v & 0xff
def sb(v): v &= 0xff; return v - 256 if v >= 128 else v
def sw(v): v &= 0xffff; return v - 65536 if v >= 0x8000 else v

def rom_byte(rom1, rom2, addr):
    if addr >> 16 == 0 and addr & 0xffff < 0x8000:
        return rom1[addr & 0xffff]
    idx = addr & 0x3ffff
    if addr & 0x80000:
        idx |= 0x40000
    return rom2[idx & 0x7ffff]

def rword(rom1, rom2, addr):
    return rom_byte(rom1, rom2, addr) << 8 | rom_byte(rom1, rom2, addr + 1)

def popcount(v): return bin(v).count('1')

class Ctx:
    def __init__(self, rom1, rom2):
        self.rom1 = rom1; self.rom2 = rom2
        self.idx = 0
        self.checked = 0
        self.sr_mask = 0x0f  # N/Z/V/C

    def rom(self, pc, off=0):
        return rom_byte(self.rom1, self.rom2, pc + off)

    def rw(self, pc, off=0):
        return rword(self.rom1, self.rom2, pc + off)

    def fail(self, tag, s, d, before, after, extra=''):
        global fail_count
        fail_count += 1
        fail_pat[(tag, self.rom(s), d['top'], d['reg'], d['siz'], d['ocode'], d['ore'])] += 1
        r = before['r']
        diff = ''.join(f'{chr(65+i)}:{r[i]:04x}->{after["r"][i]:04x} ' for i in range(8) if r[i] != after['r'][i])
        if len(fails) < 60:
            fails.append(f"{tag} pc={s:08x} op={self.rom(s):02x} len={d['len']} kind={d['kind']} "
                         f"top={d['top']:02x} reg={d['reg']} siz={d['siz']} ocode={d['ocode']} ore={d['ore']} ext={d['ext']} "
                         f"sr={before['sr']:04x}->{after['sr']:04x} {diff}{extra}")

    def expect_regs(self, s, d, before, after, sets, sr_exp=None):
        """sets: dict regidx -> expected value (None = unknown/skip),
        or None = skip register checks entirely. sr_exp: int or None.
        Pre/post-decrement addressing (top b0/c0) implicitly modifies r[reg]
        in GT MCU_Operand_General regardless of the opcode; fold that in."""
        skip_all = (sets is None)
        sdict = {} if skip_all else dict(sets)
        top = d['top']; areg = d['reg']; siz = d['siz']
        if top in (0xb0, 0xc0) and areg not in sdict:
            delta = 2 if (siz or areg == 7) else 1
            if top == 0xb0:
                delta = -delta
            sdict[areg] = W(before['r'][areg] + delta)
        for i in range(8):
            if skip_all:
                continue
            if i in sdict:
                if sdict[i] is not None:
                    exp = W(sdict[i])
                    if exp != after['r'][i]:
                        self.fail('REG', s, d, before, after, f' r{i} exp={exp:04x} got={after["r"][i]:04x}')
                # sdict[i] is None: register written but value unknown -> skip
            else:
                # register not written by instruction: must be unchanged
                if before['r'][i] != after['r'][i]:
                    self.fail('REGCHG', s, d, before, after, f' r{i} {before["r"][i]:04x}->{after["r"][i]:04x}')
        if sr_exp is not None:
            got = B(after['sr']) & 0x0f
            if got != (sr_exp & 0x0f):
                self.fail('SR', s, d, before, after, f' NVCZ exp={sr_exp:04x} got={B(after["sr"]):02x}')

    def expect_unchanged(self, s, d, before, after):
        self.expect_regs(s, d, before, after, {}, None)

def sub_common(t1, t2, siz):
    m = 0xffff if siz else 0xff
    t1 = W(t1); t2 = W(t2)
    t1 -= t2
    t1 &= m
    st1 = sw(t1) if siz else sb(t1)
    st2 = sw(t2) if siz else sb(t2)
    s1 = W(before16(t1, t2, siz, 1))
    return t1

def before16(a, b, siz, sign):
    # emulate signed result range for overflow detection
    if siz:
        va = a if a < 0x8000 else a - 0x10000
        vb = b if b < 0x8000 else b - 0x10000
        return va + vb if sign else va - vb
    else:
        va = a if a < 0x80 else a - 0x100
        vb = b if b < 0x80 else b - 0x100
        return va + vb if sign else va - vb

def flags_add(t1, t2, c_bit, siz):
    m = 0xffff if siz else 0xff
    t1 &= m; t2 &= m
    res = (t1 + t2 + c_bit) & m
    sres = before16(t1, t2, siz, 1) + c_bit
    if siz:
        ov = sres < -0x8000 or sres > 0x7fff
        nn = res & 0x8000
    else:
        ov = sres < -0x80 or sres > 0x7f
        nn = res & 0x80
    fl = 0
    if nn: fl |= N
    if res == 0: fl |= Z
    if (t1 + t2 + c_bit) > m: fl |= C
    if ov: fl |= V
    return res, fl

def flags_sub(t1, t2, c_bit, siz):
    m = 0xffff if siz else 0xff
    t1 &= m; t2 &= m
    res = (t1 - t2 - c_bit) & m
    sres = before16(t1, t2, siz, 0) - c_bit
    if siz:
        ov = sres < -0x8000 or sres > 0x7fff
        nn = res & 0x8000
    else:
        ov = sres < -0x80 or sres > 0x7f
        nn = res & 0x80
    fl = 0
    if nn: fl |= N
    if res == 0: fl |= Z
    if (t1 - t2 - c_bit) < 0: fl |= C
    if ov: fl |= V
    return res, fl

def status_common(val, siz):
    m = 0xffff if siz else 0xff
    val &= m
    fl = 0
    if (val & (0x8000 if siz else 0x80)): fl |= N
    if val == 0: fl |= Z
    return fl

def merge_reg(rval, newval, siz):
    # in-place byte/word operand write: preserve the untouched byte
    return W(newval) if siz else (W(rval) & 0xff00) | (W(newval) & 0xff)

def setcommon_sr(sr, val, siz):
    # GT MCU_SetStatusCommon: N,Z from val; V cleared; C untouched
    return (sr & ~0x0e) | status_common(val, siz)

def bcc_taken(cond, sr):
    n = 1 if sr & N else 0; z = 1 if sr & Z else 0
    v = 1 if sr & V else 0; c = 1 if sr & C else 0
    t = {0x0: 1, 0x1: 0, 0x2: (c | z) == 0, 0x3: (c | z) == 1,
         0x4: c == 0, 0x5: c == 1, 0x6: z == 0, 0x7: z == 1,
         0x8: v == 0, 0x9: v == 1, 0xa: n == 0, 0xb: n == 1,
         0xc: (n ^ v) == 0, 0xd: (n ^ v) == 1,
         0xe: (z | (n ^ v)) == 0, 0xf: (z | (n ^ v)) == 1}[cond]
    return t

def check_short(ctx, s, d, before, after, nxt):
    op = ctx.rom(s)
    r = before['r']; sr = before['sr']
    if op == 0x00:
        ctx.expect_unchanged(s, d, before, after); return
    if op in (0x01, 0x06, 0x07):
        b = ctx.rom(s, 1); reg = b & 7; h = b >> 3
        if h == 0x17:
            disp = sb(ctx.rom(s, 2))
            cond = (op == 0x01) or ((op == 0x06) and (sr & Z)) or ((op == 0x07) and not (sr & Z))
            if cond:
                nr = W(r[reg] - 1)
                exp_next = (s + 3 + disp) & 0xffff if nr != 0xffff else s + 3
                sets = {reg: nr}
                ctx.expect_regs(s, d, before, after, sets)
                if (nxt & 0xffff) != (exp_next & 0xffff):
                    ctx.fail('JMP', s, d, before, after, f' exp={exp_next:04x} got={nxt:04x}')
            else:
                ctx.expect_unchanged(s, d, before, after)
                if (nxt & 0xffff) != (s + 3) & 0xffff:
                    ctx.fail('JMP', s, d, before, after, f' exp fall s+3 got={nxt:04x}')
        else:
            # error trap in VM: no state change
            ctx.expect_unchanged(s, d, before, after)
            if (nxt & 0xffff) != (s + 3) & 0xffff:
                ctx.fail('JMP', s, d, before, after, f' trap exp s+3 got={nxt:04x}')
        return
    if op == 0x02:  # LDM
        rlist = ctx.rom(s, 1)
        n = popcount(rlist)
        sets = {i: None for i in range(8) if (rlist >> i) & 1 and i != 7}
        sets[7] = r[7] + 2 * n
        ctx.expect_regs(s, d, before, after, sets)
        return
    if op == 0x12:  # STM
        n = popcount(ctx.rom(s, 1))
        ctx.expect_regs(s, d, before, after, {7: r[7] - 2 * n})
        return
    if op == 0x03:  # PJSR
        pg = ctx.rom(s, 1); ad = ctx.rw(s, 2)
        ctx.expect_regs(s, d, before, after, {7: r[7] - 4})
        if nxt != (pg << 16) | ad:
            ctx.fail('JMP', s, d, before, after, f' exp={(pg<<16)|ad:06x} got={nxt:06x}')
        return
    if op == 0x13:  # PJMP
        pg = ctx.rom(s, 1); ad = ctx.rw(s, 2)
        ctx.expect_unchanged(s, d, before, after)
        if nxt != (pg << 16) | ad:
            ctx.fail('JMP', s, d, before, after, f' exp={(pg<<16)|ad:06x} got={nxt:06x}')
        return
    if op == 0x10:  # JMP
        ad = ctx.rw(s, 1)
        ctx.expect_unchanged(s, d, before, after)
        if (nxt & 0xffff) != ad:
            ctx.fail('JMP', s, d, before, after, f' exp={ad:04x} got={nxt:04x}')
        return
    if op == 0x18:  # JSR
        ad = ctx.rw(s, 1)
        ctx.expect_regs(s, d, before, after, {7: r[7] - 2})
        if (nxt & 0xffff) != ad:
            ctx.fail('JMP', s, d, before, after, f' exp={ad:04x} got={nxt:04x}')
        return
    if op == 0x0e:  # BSR
        disp = sb(ctx.rom(s, 1))
        ctx.expect_regs(s, d, before, after, {7: r[7] - 2})
        if (nxt & 0xffff) != (s + 2 + disp) & 0xffff:
            ctx.fail('JMP', s, d, before, after, f' exp={(s+2+disp)&0xffff:04x} got={nxt:04x}')
        return
    if op == 0x1e:  # BSR16
        disp = sw(ctx.rw(s, 1))
        ctx.expect_regs(s, d, before, after, {7: r[7] - 2})
        if (nxt & 0xffff) != (s + 3 + disp) & 0xffff:
            ctx.fail('JMP', s, d, before, after, f' exp={(s+3+disp)&0xffff:04x} got={nxt:04x}')
        return
    if op == 0x0a:  # RTE
        ctx.expect_regs(s, d, before, after, {7: r[7] + 6})
        return
    if op == 0x19:  # RTS
        ctx.expect_regs(s, d, before, after, {7: r[7] + 2})
        return
    if op == 0x14:  # RTD
        imm = sb(ctx.rom(s, 1))
        ctx.expect_regs(s, d, before, after, {7: r[7] + 2 + imm})
        return
    if op == 0x1c:  # RTD (trap)
        ctx.expect_unchanged(s, d, before, after)
        if (nxt & 0xffff) != (s + 2) & 0xffff:
            ctx.fail('JMP', s, d, before, after, f' trap exp s+2 got={nxt:04x}')
        return
    if op == 0x1a:  # sleep
        ctx.expect_unchanged(s, d, before, after); return
    if op == 0x08:  # TRAPA
        b = ctx.rom(s, 1)
        if (b & 0xf0) == 0x10:
            # trapa: interrupt taken after this instr; next = vector (unknown here without
            # full vector-table eval in this module; phase A already validated target)
            ctx.expect_unchanged(s, d, before, after)
        else:
            ctx.expect_unchanged(s, d, before, after)
            if (nxt & 0xffff) != (s + 2) & 0xffff:
                ctx.fail('JMP', s, d, before, after, f' trap exp s+2 got={nxt:04x}')
        return
    if op == 0x11:  # general jump
        b = ctx.rom(s, 1); h = b >> 3; ol = b & 7
        if b == 0x19:
            ctx.expect_regs(s, d, before, after, {7: r[7] + 4})
        elif h == 0x19:
            rr = ol & ~1
            ctx.expect_regs(s, d, before, after, {7: r[7] - 4})
            if nxt != (B(r[rr]) << 16) | r[rr + 1]:
                ctx.fail('JMP', s, d, before, after, f' exp={(B(r[rr])<<16)|r[rr+1]:06x} got={nxt:06x}')
        elif h == 0x1a:
            ctx.expect_unchanged(s, d, before, after)
            if (nxt & 0xffff) != r[ol]:
                ctx.fail('JMP', s, d, before, after, f' exp={r[ol]:04x} got={nxt:04x}')
        elif h == 0x1b:
            ctx.expect_regs(s, d, before, after, {7: r[7] - 2})
            if (nxt & 0xffff) != r[ol]:
                ctx.fail('JMP', s, d, before, after, f' exp={r[ol]:04x} got={nxt:04x}')
        else:
            ctx.expect_unchanged(s, d, before, after)
            if (nxt & 0xffff) != (s + 2) & 0xffff:
                ctx.fail('JMP', s, d, before, after, f' trap exp s+2 got={nxt:04x}')
        return
    if 0x20 <= op <= 0x3f:  # BCC
        c = op & 0xf
        disp = sb(ctx.rom(s, 1)) if not (op & 0x10) else sw(ctx.rw(s, 1))
        ln = d['len']
        taken = bcc_taken(c, sr)
        exp_next = (s + ln + disp) & 0xffff if taken else (s + ln) & 0xffff
        ctx.expect_unchanged(s, d, before, after)
        if (nxt & 0xffff) != exp_next:
            ctx.fail('JMP', s, d, before, after, f' cond={c} taken={taken} exp={exp_next:04x} got={nxt:04x}')
        return
    if 0x40 <= op <= 0x4f:  # CMP
        reg = op & 7; siz = 1 if (op & 0x8) else 0
        t2 = ctx.rw(s, 1) if siz else ctx.rom(s, 1)
        res, fl = flags_sub(r[reg], t2, 0, siz)
        ctx.expect_regs(s, d, before, after, {}, sr_exp=(sr & ~0x0f) | fl)
        return
    if 0x50 <= op <= 0x57:  # MOVE (SetStatusCommon: N,Z set, V clr, C kept)
        reg = op & 7; imm = ctx.rom(s, 1)
        nr = (r[reg] & 0xff00) | imm
        ctx.expect_regs(s, d, before, after, {reg: nr}, sr_exp=(sr & ~0x0e) | status_common(imm, 0))
        return
    if 0x58 <= op <= 0x5f:  # MOVI (SetStatusCommon: N,Z set, V clr, C kept)
        reg = op & 7; imm = ctx.rw(s, 1)
        ctx.expect_regs(s, d, before, after, {reg: imm}, sr_exp=(sr & ~0x0e) | status_common(imm, 1))
        return
    if 0x60 <= op <= 0x6f:  # MOVL: mem->reg, value unknown
        return
    if 0x70 <= op <= 0x7f:  # MOVS: reg->mem
        reg = op & 7; siz = 1 if (op & 0x8) else 0
        data = r[reg] if siz else B(r[reg])
        ctx.expect_unchanged(s, d, before, after)
        # sr set from data (SetStatusCommon: N,Z set, V clr, C kept); check sr nibble
        fl = (B(sr) & 0x01) | status_common(data, siz)
        if (B(after['sr']) & 0x0f) != fl:
            ctx.fail('SR', s, d, before, after, f' exp={fl:04x} got={B(after["sr"]):02x}')
        return
    if 0x80 <= op <= 0x9f:  # MOVF
        reg = op & 7; siz = 1 if (op & 0x8) else 0
        if (op & 0x10) == 0:
            return  # mem->reg, unknown
        # write: VM quirk: siz -> byte write w/ word status; !siz -> word write
        if siz:
            data = B(r[reg]); fl = status_common(data, 0)
        else:
            data = r[reg]; fl = status_common(data, 1)
        ctx.expect_unchanged(s, d, before, after)
        if (B(after['sr']) & 0x0f) != fl:
            ctx.fail('SR', s, d, before, after, f' exp={fl:04x} got={B(after["sr"]):02x}')
        return

def check_general(ctx, s, d, before, after, nxt):
    op = ctx.rom(s)
    top = d['top']; reg = d['reg']; siz = 1 if d['siz'] else 0
    ocode = d['ocode']; ore = d['ore']; ext = d['ext']
    r = before['r']; sr = before['sr']
    opbyte_off = 1
    if top == 0xe0: opbyte_off = 2
    elif top == 0xf0: opbyte_off = 3
    elif top == 0x00 and reg == 5: opbyte_off = 2
    elif top == 0x00 and reg == 4: opbyte_off = 2 + (1 if siz else 0)
    elif top == 0x10 and reg == 5: opbyte_off = 3
    opb = ctx.rom(s, opbyte_off)
    if ext:
        ocode2 = ctx.rom(s, opbyte_off + 1)
        ore2 = ocode2 & 7
        # (h8dasm already folds ext into ocode/ore)
    imm_operand = (top == 0x00 and reg == 4)
    imm = ctx.rw(s, 1) if (imm_operand and siz) else (ctx.rom(s, 1) if imm_operand else 0)
    direct = (top == 0xa0)

    # registers the instruction is allowed to write
    def none_regs():
        return None

    if ocode == 0:  # MOVG_Immediate: mem only / traps -> regs unchanged
        ctx.expect_unchanged(s, d, before, after)
        return
    if ocode == 1:  # ADDQ
        if ore not in (0, 1, 4, 5):
            ctx.expect_unchanged(s, d, before, after)
            if (nxt & 0xffff) != (s + d['len']) & 0xffff:
                ctx.fail('JMP', s, d, before, after, f' trap exp s+len got={nxt:04x}')
            return
        t2 = {0: 1, 1: 2, 4: -1, 5: -2}[ore]
        if direct:
            res, fl = flags_add(r[reg], t2, 0, siz)
            ctx.expect_regs(s, d, before, after, {reg: merge_reg(r[reg], res, siz)}, sr_exp=(sr & ~0x0f) | fl)
        else:
            ctx.expect_unchanged(s, d, before, after)
        return
    if ocode == 2:  # CLR family (in-place operand op)
        if direct:
            data = r[reg]
            if ore == 3:   # CLR: operand=0; N=0,Z=1,V=0,C=0
                ctx.expect_regs(s, d, before, after, {reg: merge_reg(data, 0, siz)}, sr_exp=(sr & ~0x0f) | Z)
            elif ore == 6:  # TST: status from operand, C=0
                ctx.expect_unchanged(s, d, before, after)
                if (B(after['sr']) & 0x0f) != (status_common(data, siz) | 0):
                    ctx.fail('SR', s, d, before, after, f' exp={status_common(data,siz):04x} got={B(after["sr"]):02x}')
            elif ore == 2 and not siz:  # EXTU: r=(uint16)B(r); N=0,Z=Z,V=0,C=0
                nr = B(data)
                ctx.expect_regs(s, d, before, after, {reg: nr}, sr_exp=(sr & ~0x0f) | (Z if nr == 0 else 0))
            elif ore == 0 and not siz:  # SWAP: swap bytes; SetStatusCommon(word)
                nr = (B(data) << 8) | (data >> 8)
                ctx.expect_regs(s, d, before, after, {reg: nr}, sr_exp=setcommon_sr(sr, nr, 1))
            elif ore == 5:  # NOT: operand=~operand; SetStatusCommon
                m = 0xffff if siz else 0xff
                nv = W(~data & m)
                ctx.expect_regs(s, d, before, after, {reg: merge_reg(data, nv, siz)}, sr_exp=setcommon_sr(sr, nv, siz))
            elif ore == 4:  # NEG: operand=-operand (SUB_Common: N,Z,V,C)
                res, fl = flags_sub(0, data, 0, siz)
                ctx.expect_regs(s, d, before, after, {reg: merge_reg(data, res, siz)}, sr_exp=(sr & ~0x0f) | fl)
            elif ore == 1 and not siz:  # EXTS: r=(uint16)(int8)r; SetStatusCommon(word)
                nr = W(sb(B(data)) & 0xffff)
                ctx.expect_regs(s, d, before, after, {reg: nr}, sr_exp=setcommon_sr(sr, nr, 1))
            else:  # trap
                ctx.expect_unchanged(s, d, before, after)
                if (nxt & 0xffff) != (s + d['len']) & 0xffff:
                    ctx.fail('JMP', s, d, before, after, f' trap exp s+len got={nxt:04x}')
        else:
            ctx.expect_unchanged(s, d, before, after)
        return
    if ocode == 3:  # SHLR family
        if not direct:
            ctx.expect_unchanged(s, d, before, after); return
        data = r[reg]
        c_in = 1 if sr & C else 0
        m = 0xffff if siz else 0xff
        dv = data & m
        if ore == 3:    # SHLR
            c_out = dv & 1; nv = W(dv >> 1)
        elif ore == 2:  # SHLL
            c_out = 1 if (dv & (0x8000 if siz else 0x80)) else 0; nv = W(dv << 1)
        elif ore == 6:  # ROTXL
            c_out = 1 if (dv & (0x8000 if siz else 0x80)) else 0; nv = W((dv << 1) | c_in)
        elif ore == 4:  # ROTL
            c = 1 if (dv & (0x8000 if siz else 0x80)) else 0; nv = W((dv << 1) | c)
            c_out = c
        elif ore == 0:  # SHAL
            c_out = 1 if (dv & (0x8000 if siz else 0x80)) else 0; nv = W(dv << 1)
        elif ore == 1:  # SHAR
            c_out = dv & 1; msb = dv & (0x8000 if siz else 0x80)
            nv = W((dv & m) >> 1 | msb)
        elif ore == 5:  # ROTR
            c_out = dv & 1; nv = W((dv >> 1) | (c_out << (15 if siz else 7)))
        else:
            ctx.expect_unchanged(s, d, before, after)
            if (nxt & 0xffff) != (s + d['len']) & 0xffff:
                ctx.fail('JMP', s, d, before, after, f' trap exp s+len got={nxt:04x}')
            return
        nv &= m
        nr = merge_reg(data, nv, siz)
        fl = status_common(nv, siz)
        if c_out: fl |= C
        else: fl &= ~C
        ctx.expect_regs(s, d, before, after, {reg: nr}, sr_exp=(sr & ~0x0f) | fl)
        return
    if ocode in (4, 5, 6, 7, 8, 10, 12):  # ADD/ADDS/SUB/SUBS/OR/AND/XOR
        m = 0xffff if siz else 0xff
        if ocode in (5, 7):  # ADDS/SUBS: full-word op on sign-extended operand
            if direct:
                t2 = r[reg] if siz else sb(r[reg])
                res = W(r[ore] + t2) if ocode == 5 else W(r[ore] - t2)
                ctx.expect_regs(s, d, before, after, {ore: res})
            elif imm_operand:
                t2 = imm if siz else sb(imm)
                res = W(r[ore] + t2) if ocode == 5 else W(r[ore] - t2)
                ctx.expect_regs(s, d, before, after, {ore: res})
            else:
                ctx.expect_regs(s, d, before, after, {ore: None}, None)
            return
        # ocode in (4,6,8,10,12): status-setting, dest=r[ore], size-respecting
        if direct:
            t2 = r[reg] & m
        elif imm_operand:
            t2 = imm & m
        else:
            # memory source: r[ore] becomes unknown
            ctx.expect_regs(s, d, before, after, {ore: None}, None)
            return
        if ocode == 4:
            res, fl = flags_add(r[ore], t2, 0, siz)
            nr = merge_reg(r[ore], res, siz)
            sr_exp = (sr & ~0x0f) | fl
        elif ocode == 6:
            res, fl = flags_sub(r[ore], t2, 0, siz)
            nr = merge_reg(r[ore], res, siz)
            sr_exp = (sr & ~0x0f) | fl
        else:
            op2 = {8: lambda a, b: a | b, 10: lambda a, b: a & b, 12: lambda a, b: a ^ b}[ocode]
            res = W(op2(r[ore], t2)) & m
            nr = merge_reg(r[ore], res, siz)
            sr_exp = setcommon_sr(sr, res, siz)
        ctx.expect_regs(s, d, before, after, {ore: nr}, sr_exp=sr_exp)
        return
    if ocode == 9:  # BSET_ORC
        ctx.expect_unchanged(s, d, before, after); return
    if ocode == 11:  # BCLR_ANDC
        ctx.expect_unchanged(s, d, before, after); return
    if ocode == 13:  # NotImplemented
        ctx.expect_unchanged(s, d, before, after)
        if (nxt & 0xffff) != (s + d['len']) & 0xffff:
            ctx.fail('JMP', s, d, before, after, f' trap exp s+len got={nxt:04x}')
        return
    if ocode == 14:  # CMP
        if direct:
            t2 = r[reg]
        elif imm_operand:
            t2 = (imm & 0xff) if not siz else imm
        else:
            t2 = None
        if t2 is not None:
            res, fl = flags_sub(r[ore], t2, 0, siz)
            ctx.expect_regs(s, d, before, after, {}, sr_exp=(sr & ~0x0f) | fl)
        else:
            ctx.expect_unchanged(s, d, before, after)
        return
    if ocode == 15:  # BTST
        ctx.expect_unchanged(s, d, before, after); return
    if ocode == 16:  # MOVG read: r[ore] = operand; SetStatusCommon(operand)
        if direct:
            nr = merge_reg(r[ore], r[reg], siz)
            ctx.expect_regs(s, d, before, after, {ore: nr}, sr_exp=setcommon_sr(sr, r[reg], siz))
        elif imm_operand:
            nr = merge_reg(r[ore], imm, siz)
            ctx.expect_regs(s, d, before, after, {ore: nr}, sr_exp=setcommon_sr(sr, imm, siz))
        else:
            ctx.expect_regs(s, d, before, after, {ore: None}, None)
        return
    elif ocode == 17:  # LDC
        if imm_operand:
            ctx.expect_unchanged(s, d, before, after)
            if ore == 0:
                exp_sr = W(imm & 0x870f)
                if after['sr'] != exp_sr:
                    ctx.fail('SR', s, d, before, after, f' ldc exp={exp_sr:04x} got={after["sr"]:04x}')
            elif ore in (3, 4, 5, 7):
                got = after[('br', 'ep', 'dp', 'tp')[ore - 3]]
                if got != B(imm):
                    ctx.fail('CR', s, d, before, after, f' cr{ore} exp={B(imm):02x} got={got:02x}')
        else:
            ctx.expect_unchanged(s, d, before, after)
        return
    if ocode == 18:  # MOVG write / XCH
        if direct:
            if siz:
                sets = {ore: r[reg], reg: r[ore]}
                ctx.expect_regs(s, d, before, after, sets)
            else:
                # XCH byte -> trap
                ctx.expect_unchanged(s, d, before, after)
                if (nxt & 0xffff) != (s + d['len']) & 0xffff:
                    ctx.fail('JMP', s, d, before, after, f' trap exp s+len got={nxt:04x}')
        else:
            # reg->mem: r[ore] unchanged, status set from r[ore]
            ctx.expect_regs(s, d, before, after, {}, sr_exp=setcommon_sr(sr, r[ore], siz))
        return
    if ocode == 19:  # STC
        ctx.expect_unchanged(s, d, before, after); return
    if ocode == 20:  # ADDX
        if direct:
            c_bit = 1 if sr & C else 0
            res, fl = flags_add(r[ore], r[reg], c_bit, siz)
            # VM quirk: Z_after = Z_before && (res == 0)
            if not (sr & Z) or res != 0:
                fl &= ~Z
            ctx.expect_regs(s, d, before, after, {ore: res}, sr_exp=(sr & ~0x0f) | fl)
        else:
            ctx.expect_unchanged(s, d, before, after)
        return
    if ocode == 21:  # MULXU
        if direct:
            t1 = r[reg]; t2 = r[ore]
            if siz:
                prod = W(t1) * W(t2)
                prod &= 0xffffffff
                rr = ore & ~1
                sets = {rr: W(prod >> 16), rr | 1: W(prod)}
                nn = 1 if prod & 0x80000000 else 0
                zz = 1 if prod == 0 else 0
            else:
                prod = B(t1) * B(t2)
                sets = {ore: W(prod)}
                nn = 1 if prod & 0x8000 else 0
                zz = 1 if prod == 0 else 0
            fl = (N if nn else 0) | (Z if zz else 0)
            ctx.expect_regs(s, d, before, after, sets, sr_exp=(sr & ~0x0f) | fl)
        else:
            return
        return
    if ocode == 22:  # SUBX
        if direct:
            c_bit = 1 if sr & C else 0
            res, fl = flags_sub(r[ore], r[reg], c_bit, siz)
            ctx.expect_regs(s, d, before, after, {ore: res}, sr_exp=(sr & ~0x0f) | fl)
        else:
            ctx.expect_unchanged(s, d, before, after)
        return
    if ocode == 23:  # DIVXU
        if direct:
            t1 = r[reg]; t2 = r[ore]
            if siz:
                rr = ore & ~1
                t1w = W(t1)
                t2w = (r[rr] << 16) | r[rr | 1]
            else:
                t1w = B(t1)
                t2w = B(t2)
            if t1w == 0:
                ctx.expect_unchanged(s, d, before, after)
                return
            R = t2w % t1w; Q = t2w // t1w
            if siz:
                if Q > 0xffff:
                    # overflow: V=1, regs unchanged
                    ctx.expect_regs(s, d, before, after, {}, sr_exp=(sr & ~0x0f) | V)
                    return
                sets = {rr: W(R), rr | 1: W(Q)}
                fl = status_common(Q, 1)
            else:
                if Q > 0xff:
                    ctx.expect_regs(s, d, before, after, {}, sr_exp=(sr & ~0x0f) | V)
                    return
                sets = {ore: W((R << 8) | Q)}
                fl = status_common(Q, 0)
            ctx.expect_regs(s, d, before, after, sets, sr_exp=(sr & ~0x0f) | fl)
        else:
            return
        return
    if ocode in (24, 25, 26, 27, 28, 29, 30, 31):  # BSET/BCLR/BNOTI/BTSTI
        ctx.expect_unchanged(s, d, before, after); return

def mkstate(regs):
    # loader gives {'sr', 'r0'..'r7', 'br','dp','ep','tp'}; checkers want {'r':[...], 'sr', ...}
    return {'r': [regs[f'r{i}'] for i in range(8)], 'sr': regs['sr'],
            'br': regs['br'], 'dp': regs['dp'], 'ep': regs['ep'], 'tp': regs['tp']}

def build_vec_next(dec, rom1, rom2):
    # PCs an interrupt/exception dispatch can land on: vector + len(first instr),
    # plus that first instr's branch target. Used to detect dispatch on kind-4/5 lines.
    out = set()
    for v in range(64):
        b = [rom_byte(rom1, rom2, v * 4 + k) for k in range(4)]
        v32 = (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]
        addr = (((v32 >> 16) & 0xff) << 16) | (v32 & 0xffff)
        dv = dec.get(addr)
        if dv is None:
            continue
        out.add(((addr >> 16) << 16) | ((addr & 0xffff) + dv['len']) & 0xffff)
        if dv['kind'] in (1, 2, 3, 4, 5):
            out.add((dv['tpage'] << 16) | dv['toff'])
    return frozenset(out)

def phaseB(dec, rt):
    import collections
    rom1 = open(sys.argv[sys.argv.index('--rom1') + 1], 'rb').read()
    rom2 = open(sys.argv[sys.argv.index('--rom2') + 1], 'rb').read()
    ctx = Ctx(rom1, rom2)
    vec_next = build_vec_next(dec, rom1, rom2)
    counts = collections.Counter()
    sleeps = 0
    interrupts = 0
    checked = 0
    for i in range(len(rt) - 1):
        cyc, cp, soff, before_r = rt[i]
        cyc2, cp2, nxt, after_r = rt[i + 1]
        before = mkstate(before_r)
        after = mkstate(after_r)
        s = (cp << 16) | soff
        u = (cp2 << 16) | nxt
        d = dec.get(s)
        if d is None:
            continue
        tgt = (d['tpage'] << 16) | d['toff']
        seq = (cp << 16) | ((soff + d['len']) & 0xffff)
        if u == s and u != tgt:
            sleeps += 1  # CPU sleeping: repeated PC, no instruction executed
            continue
        if d['kind'] in (4, 5):
            # ret / reg-indirect: target is runtime-determined, so only an
            # interrupt dispatch (u in vec_next) tells us the instr did NOT run.
            executed = (u not in vec_next)
        else:
            executed = (u == seq) or (u == tgt)
        if not executed:
            # interrupt/exception was dispatched this iteration: the
            # instruction at s never ran; its semantics are covered by
            # the vector line itself later in the trace.
            interrupts += 1
            continue
        op = rom_byte(rom1, rom2, s)
        if (op == 0x04 or op == 0x05 or op == 0x0c or op == 0x0d or op == 0x15 or op == 0x1d) or op >= 0xa0:
            check_general(ctx, s, d, before, after, u)
        else:
            check_short(ctx, s, d, before, after, u)
        counts[op >> 4] += 1
        checked += 1
    if sleeps:
        print(f"[INFO] sleeping transitions skipped: {sleeps}")
    if interrupts:
        print(f"[INFO] interrupt-dispatch transitions skipped: {interrupts}")
    print(f"[INFO] semantic instructions checked: {checked}")
    if fails:
        print(f"[FAIL] semantic failures: {fail_count}")
        print("  by (tag,op,top,reg,siz,ocode,ore):")
        for pat, n in fail_pat.most_common():
            tag, op, top, reg, siz, ocode, ore = pat
            print(f"    {tag:8s} op={op:02x} top={top:02x} reg={reg} siz={siz} ocode={ocode:2d} ore={ore} : {n}")
        for f in fails:
            print("       ", f)
    else:
        print(f"[OK]   semantic failures: 0")
