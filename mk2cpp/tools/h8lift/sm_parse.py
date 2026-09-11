#!/usr/bin/env python3
"""sm_parse.py -- generate / verify the SM opcode tables embedded in smemit.c.

smemit.c keeps two 256-entry tables:

  sm_op_impl[256]  1 if GT implements the opcode, 0 for SM_Opcode_NotImplemented
  sm_op_len[256]   operand byte count (instruction length = 1 + len); 0 for
                   opcodes GT does not implement (they trap, no operand fetch)

Both are derived from committed sources, never hand-computed:

  impl  <- src/submcu.cpp `SM_Opcode_Table[256]` (GT ground truth).  GT only
           implements 165 of 256 slots; the rest route to SM_ErrorTrap.
  len   <- operand bytes actually fetched along each opcode's GT handler path
           (SM_ReadAdvance = 1, SM_ReadAdvance16 = 2).  Because GT is
           incomplete, every implemented opcode is cross-checked against the
           complete M37450 addressing-mode table in tools/disasm/smdasm.c
           (AM[256] -> operand bytes); the two derivations must agree 165/165.

Coverage invariants (fail loudly if a GT change is not mirrored in smemit.c):

  * `emit_body` case labels == implemented opcode set (exactly).
  * `sm_op_load_expr` case labels == implemented load-family opcodes with an
    operand (LDA/LDX/LDY/ORA/AND/CMP/CPX/CPY), exactly.
  * `sm_op_store_dest` case labels == implemented store-family opcodes with an
    operand (STA/STX/STY, DEC/INC memory forms), exactly.

Usage:
  python sm_parse.py [--repo-root DIR] [--out FILE]   # print/write tables
  python sm_parse.py --check mk2cpp/tools/h8lift/smemit.c
                                                      # verify embedded tables

Exit status: 0 on success, 1 on validation/check failure, 2 on bad input.
Paths: all optional; defaults resolve relative to this script, never to an
absolute machine-specific location.
"""

import argparse
import re
import sys
from pathlib import Path

HEADER = ("/* GENERATED from src/submcu.cpp SM_Opcode_Table by "
          "mk2cpp/tools/h8lift/sm_parse.py. DO NOT EDIT. */")

# smdasm.c addressing modes (see its header comment) -> operand byte count.
# AM numbering must match tools/disasm/smdasm.c `AddressModes`.
AM_OPERAND_BYTES = {
    0: 0,   # Illegal
    1: 1,   # Immediate
    2: 0,   # Accumulator
    3: 1,   # ZeroPage
    4: 1,   # ZeroPageX
    5: 1,   # ZeroPageY
    6: 2,   # Absolute
    7: 2,   # AbsoluteX
    8: 2,   # AbsoluteY
    9: 0,   # Implied
    10: 1,  # Relative
    11: 1,  # IndirectX
    12: 1,  # IndirectY
    13: 2,  # IndirectAbsolute
    14: 1,  # ZeroPageIndirect
    15: 1,  # SpecialPage
    16: 1,  # ZeroPageBit
    17: 0,  # AccumulatorBit
    18: 1,  # AccumulatorBitRelative
    19: 2,  # ZeroPageBitRelative
    20: 2,  # ZeroPageImmediate
}

LOAD_FAMILY = {"SM_Opcode_LDA", "SM_Opcode_LDX", "SM_Opcode_LDY",
               "SM_Opcode_ORA", "SM_Opcode_AND", "SM_Opcode_CMP",
               "SM_Opcode_CPX", "SM_Opcode_CPY"}
STORE_FAMILY = {"SM_Opcode_STA", "SM_Opcode_STX", "SM_Opcode_STY"}
RMEM_FAMILY = {"SM_Opcode_DEC", "SM_Opcode_INC"}  # memory forms only


def fail(msg):
    print("sm_parse: error: %s" % msg, file=sys.stderr)
    sys.exit(1)


def read_text(path):
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        fail("cannot read %s: %s" % (path, exc))


def func_body(src, name):
    m = re.search(r"static [^{]*\b%s\([^)]*\)\s*\n\{(.*?)\n\}" % re.escape(name),
                  src, re.S)
    if not m:
        fail("function %s not found in smemit.c" % name)
    return m.group(1)


def case_labels(src):
    return set(int(x, 16) for x in re.findall(r"case 0x([0-9a-fA-F]{2}):", src))


def parse_gt_table(text):
    m = re.search(r"void \(\*SM_Opcode_Table\[256\]\)\(uint8_t opcode\)\s*\{(.*?)\n\};",
                  text, re.S)
    if not m:
        fail("SM_Opcode_Table[256] not found in src/submcu.cpp")
    entries = re.findall(r"(SM_Opcode_\w+)\s*,\s*//\s*([0-9a-fA-F]{2})",
                         m.group(1))
    if len(entries) != 256:
        fail("SM_Opcode_Table has %d entries, expected 256" % len(entries))
    table = [None] * 256
    for pos, (name, idx) in enumerate(entries):
        i = int(idx, 16)
        if i != pos:
            fail("SM_Opcode_Table entry %d is commented // %02x" % (pos, i))
        table[i] = name
    return table


def parse_gt_handlers(text):
    handlers = {}
    for m in re.finditer(r"void (SM_Opcode_\w+)\(uint8_t opcode\)[^{]*\n\{(.*?)\n\}",
                         text, re.S):
        handlers[m.group(1)] = m.group(2)
    return handlers


def _strip_untaken_zp(body, zp_true):
    """Remove `if (!zp) {...}` / `else {...}` blocks not taken for this opcode.

    Only BBC_BBS and SEB_CLB branch on `(opcode & 4)`, both with plain braced
    if/else, so a small brace matcher is sufficient."""
    out = body
    while True:
        m = re.search(r"if \(!zp\)\s*\{", out)
        if not m:
            return out
        open_i = out.index("{", m.start())
        depth = 0
        for j in range(open_i, len(out)):
            if out[j] == "{":
                depth += 1
            elif out[j] == "}":
                depth -= 1
                if depth == 0:
                    break
        end = j + 1
        e = out.find("else", end)
        el_start = el_end = -1
        if e != -1 and out[end:e].strip() == "":
            open_e = out.index("{", e)
            depth = 0
            for j in range(open_e, len(out)):
                if out[j] == "{":
                    depth += 1
                elif out[j] == "}":
                    depth -= 1
                    if depth == 0:
                        break
            el_start, el_end = e, j + 1
        if zp_true:
            out = out[:m.start()] + (out[el_start:] if el_start != -1
                                     else out[end:])
        else:
            if el_start != -1:
                out = out[:m.start()] + out[end:el_start] + out[el_end:]
            else:
                out = out[:m.start()] + out[end:]
    return out


def _count_fetches(text):
    t = text.replace("SM_ReadAdvance16(", "\x00")
    return t.count("\x00") * 2 + t.replace("\x00", "SM_ReadAdvance16(").count(
        "SM_ReadAdvance(")


def handler_path_len(handler, opcode):
    """Operand bytes GT fetches for `opcode` along its handler path."""
    if "switch (opcode)" in handler:
        cases = dict(re.findall(
            r"case 0x([0-9a-fA-F]{2}):(.*?)(?=case 0x|default:|$)",
            handler, re.S))
        key = "%02x" % opcode
        if key not in cases:
            fail("handler has no case 0x%s" % key)
        return _count_fetches(cases[key])
    zp = (opcode & 4) != 0
    if "!zp" in handler:
        return _count_fetches(_strip_untaken_zp(handler, zp))
    return _count_fetches(handler)


def parse_smdasm_am(text):
    m = re.search(r"static const unsigned char AM\[256\] = \{(.*?)\};",
                  text, re.S)
    if not m:
        fail("AM[256] not found in tools/disasm/smdasm.c")
    am = [int(x) for x in re.findall(r"\d+", m.group(1))]
    if len(am) != 256:
        fail("AM[256] has %d entries, expected 256" % len(am))
    for v in am:
        if v not in AM_OPERAND_BYTES:
            fail("AM value %d outside the known 0..20 addressing modes" % v)
    return am


def parse_embedded_tables(text):
    def arr(name):
        m = re.search(r"static const uint8_t %s\[256\] = \{(.*?)\};" % name,
                      text, re.S)
        if not m:
            fail("embedded table %s[256] not found" % name)
        vals = [int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})",
                                               m.group(1))]
        if len(vals) != 256:
            fail("embedded table %s has %d entries" % (name, len(vals)))
        return vals
    return arr("sm_op_impl"), arr("sm_op_len")


def render(impl, length):
    lines = [HEADER,
             "static const uint8_t sm_op_impl[256] = {"]
    for i in range(0, 256, 16):
        lines.append("    " + " ".join("0x%02x," % v for v in impl[i:i + 16]))
    lines.append("};")
    lines.append("")
    lines.append("static const uint8_t sm_op_len[256] = {")
    for i in range(0, 256, 16):
        lines.append("    " + " ".join("0x%02x," % v for v in length[i:i + 16]))
    lines.append("};")
    return "\n".join(lines) + "\n"


def build(repo_root):
    gt = read_text(repo_root / "src" / "submcu.cpp")
    smdasm = read_text(repo_root / "tools" / "disasm" / "smdasm.c")

    gt_tab = parse_gt_table(gt)
    handlers = parse_gt_handlers(gt)
    am = parse_smdasm_am(smdasm)

    impl = [0 if name == "SM_Opcode_NotImplemented" else 1 for name in gt_tab]
    length = [0] * 256
    for op in range(256):
        if not impl[op]:
            continue
        handler = handlers.get(gt_tab[op])
        if handler is None:
            fail("handler %s not parsed" % gt_tab[op])
        length[op] = handler_path_len(handler, op)

    # cross-check against the complete M37450 table (GT is incomplete)
    for op in range(256):
        if impl[op] and AM_OPERAND_BYTES[am[op]] != length[op]:
            fail("opcode %02x: GT handler fetches %d operand byte(s), "
                 "smdasm AM=%d says %d" % (op, length[op], am[op],
                                           AM_OPERAND_BYTES[am[op]]))

    # smemit.c coverage: emitter must handle every implemented opcode
    smemit = read_text(repo_root / "mk2cpp" / "tools" / "h8lift" / "smemit.c")
    impl_set = set(i for i in range(256) if impl[i])
    emit_cases = case_labels(func_body(smemit, "emit_body"))
    if emit_cases != impl_set:
        fail("emit_body case labels != implemented opcodes (missing %s, "
             "extra %s)" % (sorted(hex(x) for x in impl_set - emit_cases),
                            sorted(hex(x) for x in emit_cases - impl_set)))
    load_cases = case_labels(func_body(smemit, "sm_op_load_expr"))
    store_cases = case_labels(func_body(smemit, "sm_op_store_dest"))
    want_load = set(i for i in impl_set
                    if gt_tab[i] in LOAD_FAMILY and length[i] > 0)
    want_store = set(i for i in impl_set
                     if (gt_tab[i] in STORE_FAMILY
                         or (gt_tab[i] in RMEM_FAMILY and length[i] > 0)))
    if load_cases != want_load:
        fail("sm_op_load_expr cases != load-family opcodes (missing %s, "
             "extra %s)" % (sorted(hex(x) for x in want_load - load_cases),
                            sorted(hex(x) for x in load_cases - want_load)))
    if store_cases != want_store:
        fail("sm_op_store_dest cases != store-family opcodes (missing %s, "
             "extra %s)" % (sorted(hex(x) for x in want_store - store_cases),
                            sorted(hex(x) for x in store_cases - want_store)))
    return impl, length


def main():
    default_root = Path(__file__).resolve().parents[3]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo-root", type=Path, default=default_root,
                    help="repository root (default: inferred from this script)")
    ap.add_argument("--out", type=Path,
                    help="write the generated C tables here (default: stdout)")
    ap.add_argument("--check", type=Path, metavar="SMEMIT_C",
                    help="compare the tables embedded in this smemit.c")
    args = ap.parse_args()

    impl, length = build(args.repo_root)
    text = render(impl, length)

    if args.check:
        target = read_text(args.check)
        emb_impl, emb_len = parse_embedded_tables(target)
        bad_impl = [i for i in range(256) if emb_impl[i] != impl[i]]
        bad_len = [i for i in range(256) if emb_len[i] != length[i]]
        if bad_impl or bad_len:
            for i in bad_impl:
                print("sm_parse: impl[%02x]: embedded %d, derived %d"
                      % (i, emb_impl[i], impl[i]), file=sys.stderr)
            for i in bad_len:
                print("sm_parse: len[%02x]: embedded %d, derived %d"
                      % (i, emb_len[i], length[i]), file=sys.stderr)
            fail("%s tables are stale" % args.check)
        print("sm_parse: %s tables OK (%d implemented, %d operand slots "
              "cross-checked against smdasm AM)"
              % (args.check, sum(impl), sum(1 for i in range(256)
                                           if impl[i] and length[i] >= 0)))
        return

    if args.out:
        args.out.write_text(text, encoding="utf-8", newline="\n")
        print("sm_parse: wrote %s (%d implemented)" % (args.out, sum(impl)))
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
