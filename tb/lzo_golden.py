#!/usr/bin/env python3
"""
lzo_golden.py - reference compressor/decompressor for the lzo_pl FPGA project.

This defines the EXACT byte-level token format that the SystemVerilog PL core
(rtl/lzo_comp.sv + rtl/lzo_decomp.sv) must reproduce.  It is an LZO1X-style
format (literal runs + length/distance back-references, byte-aligned tokens,
0xFF length-extension) - "self-consistent": our compressor's chosen tokens are
not guaranteed identical to PC `lzop`, but they ARE valid for our decompressor
and the round-trip is provably lossless.

Block model
-----------
The file is split into independent blocks of BLK = 4096 bytes (last block may be
shorter).  Each block is compressed alone, so back-reference distances never
exceed the block (window fits a single 4 KB BRAM in the PL, distance <= 4096).
The PS keeps a per-block header (orig_len, comp_len); there is no in-stream EOF
marker - the decoder simply stops once it has emitted orig_len bytes.

Token grammar (decoder view), reading control byte t:
  * t & 0x80 == 0   -> LITERAL run
        n = t & 0x7F
        if n != 0:  L = n                       (1..127 literals)
        if n == 0:  L = 128                      (extended)
                    while (b = next) == 0xFF: L += 255
                    L += b
        then copy L literal bytes verbatim.
  * t & 0x80 != 0   -> MATCH (copy from history)
        lf = (t >> 4) & 0x07
        if lf != 0: M = lf + 2                   (match length 3..9)
        if lf == 0: M = 10                       (extended)
                    while (b = next) == 0xFF: M += 255
                    M += b
        dist_hi = t & 0x0F
        dist_lo = next
        D = ((dist_hi << 8) | dist_lo) + 1       (distance 1..4096)
        copy M bytes from out[pos-D].
MIN_MATCH = 3.  Matches with length < 3 are never emitted (literals are cheaper).
"""

import sys, os, struct, random

BLK        = 4096       # uncompressed block size (== PL window depth)
MIN_MATCH  = 3
MAX_DIST   = 4096       # 12-bit distance + 1
HASH_BITS  = 13         # hash table entries = 8192
HASH_SIZE  = 1 << HASH_BITS

# worst-case expansion: a fully-incompressible block costs at most
# ceil(BLK/127) control bytes + BLK literals.  Caller sizes the out buffer with
# this bound; the PL uses the same.
def max_comp_len(n):
    return n + (n // 127) + 16


# ---------------------------------------------------------------------------
# compressor : greedy single-candidate hash match finder (hardware-friendly)
# ---------------------------------------------------------------------------
def _hash3(b0, b1, b2):
    # cheap multiplicative hash of 3 bytes -> HASH_BITS.  The PL uses the same.
    h = (b0 * 0x9E37 + b1 * 0x0185 + b2 * 0x0001) & 0xFFFFFFFF
    return (h >> (24 - HASH_BITS)) & (HASH_SIZE - 1)


def _emit_literals(out, src, start, count):
    # one literal run can carry 1..127 in the control byte, or 128.. extended.
    i = start
    end = start + count
    while i < end:
        n = end - i
        if n <= 127:
            out.append(n)
            out.extend(src[i:i + n])
            i += n
        else:
            # extended literal run: control 0x00, then 0xFF*k + remainder, base 128
            out.append(0x00)
            rem = n - 128
            while rem >= 255:
                out.append(0xFF)
                rem -= 255
            out.append(rem)
            out.extend(src[i:i + n])
            i += n


def _emit_match(out, length, dist):
    d = dist - 1
    dist_hi = (d >> 8) & 0x0F
    dist_lo = d & 0xFF
    if length <= 9:
        out.append(0x80 | ((length - 2) << 4) | dist_hi)
        out.append(dist_lo)
    else:
        # extended: control, then length-extension bytes, THEN dist_lo
        # (decoder reads the 0xFF chain immediately after the control byte).
        out.append(0x80 | (0 << 4) | dist_hi)   # lf=0 -> extended
        rem = length - 10
        while rem >= 255:
            out.append(0xFF)
            rem -= 255
        out.append(rem)
        out.append(dist_lo)


def compress_block(src):
    n = len(src)
    out = bytearray()
    head = [-1] * HASH_SIZE          # hash -> last position (single candidate)
    lit_start = 0                    # start of pending literal run
    i = 0
    while i < n:
        best_len = 0
        best_dist = 0
        if i + MIN_MATCH <= n:
            h = _hash3(src[i], src[i + 1], src[i + 2])
            cand = head[h]
            head[h] = i
            if cand >= 0 and (i - cand) <= MAX_DIST:
                # verify + extend
                ml = 0
                maxl = n - i
                while ml < maxl and src[cand + ml] == src[i + ml]:
                    ml += 1
                if ml >= MIN_MATCH:
                    best_len = ml
                    best_dist = i - cand
        else:
            pass

        if best_len >= MIN_MATCH:
            # flush pending literals, then the match
            if i > lit_start:
                _emit_literals(out, src, lit_start, i - lit_start)
            _emit_match(out, best_len, best_dist)
            # insert hashes for the covered span so future matches can see them
            j = i + 1
            end = i + best_len
            while j < end and j + MIN_MATCH <= n:
                head[_hash3(src[j], src[j + 1], src[j + 2])] = j
                j += 1
            i += best_len
            lit_start = i
        else:
            i += 1
    if n > lit_start:
        _emit_literals(out, src, lit_start, n - lit_start)
    return bytes(out)


# ---------------------------------------------------------------------------
# decompressor : token interpreter (mirrors the PL state machine)
# ---------------------------------------------------------------------------
def decompress_block(comp, orig_len):
    out = bytearray()
    ip = 0
    clen = len(comp)
    while len(out) < orig_len:
        t = comp[ip]; ip += 1
        if (t & 0x80) == 0:
            n = t & 0x7F
            if n == 0:
                L = 128
                while comp[ip] == 0xFF:
                    L += 255; ip += 1
                L += comp[ip]; ip += 1
            else:
                L = n
            out.extend(comp[ip:ip + L]); ip += L
        else:
            lf = (t >> 4) & 0x07
            if lf == 0:
                M = 10
                while comp[ip] == 0xFF:
                    M += 255; ip += 1
                M += comp[ip]; ip += 1
            else:
                M = lf + 2
            dist_hi = t & 0x0F
            dist_lo = comp[ip]; ip += 1
            D = ((dist_hi << 8) | dist_lo) + 1
            src = len(out) - D
            for k in range(M):           # byte-by-byte (overlap-correct copy)
                out.append(out[src + k])
    assert len(out) == orig_len, (len(out), orig_len)
    return bytes(out), ip


# ---------------------------------------------------------------------------
# file-level helpers + self test
# ---------------------------------------------------------------------------
def compress_file(data):
    blocks = []
    for off in range(0, len(data), BLK):
        chunk = data[off:off + BLK]
        c = compress_block(chunk)
        # never store an expanded block: fall back to "stored" (handled by PS)
        blocks.append((len(chunk), c))
    return blocks


def make_sample(path, size=20000):
    words = ("the quick brown fox jumps over the lazy dog ".split())
    lines = []
    rng = random.Random(1234)
    while sum(len(x) for x in lines) < size:
        k = rng.randint(4, 12)
        lines.append(" ".join(rng.choice(words) for _ in range(k)) + ".\n")
    txt = "".join(lines)[:size]
    with open(path, "w", newline="\n") as f:
        f.write(txt)
    return txt.encode("latin-1")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    proj = os.path.dirname(here)
    sample = os.path.join(proj, "golden", "original_file.txt")
    data = make_sample(sample)
    print(f"sample: {sample}  ({len(data)} bytes)")

    blocks = compress_file(data)
    total_c = 0
    recon = bytearray()
    for i, (olen, c) in enumerate(blocks):
        d, _ = decompress_block(c, olen)
        assert d == data[i * BLK: i * BLK + olen], f"block {i} mismatch"
        total_c += len(c)
        recon.extend(d)
    assert bytes(recon) == data, "round-trip mismatch!"
    print(f"blocks={len(blocks)}  orig={len(data)}  comp={total_c}  "
          f"ratio={len(data)/total_c:.3f}x  (round-trip OK)")

    # emit xsim vectors for the FIRST full block
    sim = os.path.join(proj, "sim")
    os.makedirs(sim, exist_ok=True)
    b0 = data[:BLK]
    c0 = compress_block(b0)
    d0, _ = decompress_block(c0, len(b0))
    assert d0 == b0
    _write_hex(os.path.join(sim, "blk_in.hex"),   b0)
    _write_hex(os.path.join(sim, "blk_comp.hex"), c0)
    with open(os.path.join(sim, "blk_meta.txt"), "w") as f:
        f.write(f"{len(b0)} {len(c0)}\n")
    print(f"sim vectors: blk_in.hex ({len(b0)}B) blk_comp.hex ({len(c0)}B)  "
          f"ratio={len(b0)/len(c0):.3f}x")


def _write_hex(path, b):
    with open(path, "w") as f:
        for x in b:
            f.write(f"{x:02x}\n")


if __name__ == "__main__":
    main()
