# DESIGN.md — lzo_pl internals

How the LZO compressor works, and how every RTL module implements it.

- [Part 1 — The LZO compression algorithm](#part-1--the-lzo-compression-algorithm)
- [Part 2 — System architecture (PS + PL)](#part-2--system-architecture-ps--pl)
- [Part 3 — RTL modules](#part-3--rtl-modules)
- [Part 4 — Design decisions & gotchas](#part-4--design-decisions--gotchas)

---

# Part 1 — The LZO compression algorithm

## 1.1 The idea: LZ77 dictionary compression

LZO belongs to the **LZ77** family. The insight: real data repeats. Instead of
storing repeated bytes again, replace a repeat with a *pointer back* to where it
appeared before. The compressed stream is therefore a sequence of two kinds of
instructions:

```
   LITERAL run : "emit these N new bytes verbatim"
   MATCH       : "copy M bytes from D positions back in what you've already output"
```

A match is written as a `(length, distance)` pair. The decompressor keeps a
*history* of everything it has emitted so far; a match just copies from that
history. Nothing is ever "looked up" in an external dictionary — the dictionary
*is* the recently decompressed output (a "sliding window").

```
   input :  the cat sat on the mat
            └─────────┘       └──┘
            literals          "the " already seen 13 bytes back
                              -> MATCH(length=4, distance=13)
```

LZO specifically is tuned for **speed** (especially decompression) and a simple,
byte-aligned token format — no entropy coding, no bit-level packing. That is
exactly what makes it a good fit for a small FPGA: the decoder is a tight state
machine that copies bytes, and the encoder needs only a hash table and a
byte-compare.

## 1.2 Our token format (LZO1X-style, self-consistent)

This project uses an LZO1X-*style* byte-aligned format defined once in
`tb/lzo_golden.py` and mirrored exactly by the RTL and `lzo_sw.c`. It keeps
LZO's essential structure (literal runs + length/distance matches, 0xFF
length-extension) and is **lossless by construction**, but the encoder's chosen
tokens are not guaranteed bit-identical to PC `lzop`. `MIN_MATCH = 3`.

Reading one **control byte `t`**:

```
 bit:  7 6 5 4 3 2 1 0
       ┌─┬─────┬───────┐
 t  =  │M│ len │ dhi   │
       └─┴─────┴───────┘
        │  │      └── distance high nibble  (matches only)
        │  └───────── length field          (matches only)
        └──────────── 0 = LITERAL run, 1 = MATCH
```

### Literal run  (t[7] == 0)
```
   n = t[6:0]
   if n != 0 :  L = n                         (1..127 literal bytes follow)
   if n == 0 :  L = 128                        (extended)
                while (next == 0xFF) L += 255
                L += next                       (final non-0xFF byte)
   then copy L literal bytes straight through.
```
A run longer than 127 can always be written as several ≤127 runs, so the encoder
in this project never actually emits the extended literal form — the decoder
still supports it.

### Match  (t[7] == 1)
```
   lf = t[6:4]                                 (3-bit length field)
   if lf != 0 :  M = lf + 2                    (match length 3..9)
   if lf == 0 :  M = 10                        (extended)
                 while (next == 0xFF) M += 255
                 M += next
   dist_hi = t[3:0]                            (high 4 bits of distance-1)
   dist_lo = next byte                         (low 8 bits of distance-1)
   D = ((dist_hi << 8) | dist_lo) + 1          (distance 1..4096)
   copy M bytes from history[current - D].
```

So a typical match is **2 bytes** (control + dist_lo) encoding length 3..9 and
distance 1..4096; longer matches add `0xFF…` length bytes between the control
byte and `dist_lo`.

```
  short match (len 3..9):      [ 1 lll dddd ] [ dddddddd ]
                                  └ctrl       └dist_lo
  long  match (len ≥ 10):      [ 1 000 dddd ] [0xFF…][rem] [ dddddddd ]
                                  └ctrl=lf0    └length ext   └dist_lo
```

## 1.3 Worked example — compressing `abcabcabcabc` (12 bytes)

```
 index : 0 1 2 3 4 5 6 7 8 9 10 11
 byte  : a b c a b c a b c a  b  c
```

The encoder scans left to right, hashing 3 bytes at each position and recording
"the last place I saw this 3-byte key":

```
 i=0  key "abc"  table empty -> remember pos0;  no match -> 'a' pending
 i=1  key "bca"  empty       -> remember pos1;  no match -> 'b' pending
 i=2  key "cab"  empty       -> remember pos2;  no match -> 'c' pending
 i=3  key "abc"  -> found pos0! distance = 3
                   compare input[0..] vs input[3..]:
                   a=a b=b c=c a=a b=b c=c a=a b=b c=c  -> 9 bytes match
                   length = 9, distance = 3
```

Encoder output:

```
   flush pending literals "abc"  ->  ctrl 0x03, 'a','b','c'
   emit MATCH(len=9, dist=3)     ->  ctrl = 0x80 | ((9-2)<<4) | ((3-1)>>8)
                                            = 0x80 | 0x70 | 0x00 = 0xF0
                                     dist_lo = (3-1) & 0xFF = 0x02
   compressed = 03 61 62 63 F0 02            (6 bytes)
```

12 bytes → 6 bytes = **2×**. Note the match length (9) is *larger than the
distance* (3): the copy overlaps itself, which is how LZ77 expresses "repeat this
short pattern". The decoder handles that naturally (next section).

## 1.4 Worked example — decompressing `03 61 62 63 F0 02`

```
 out = "" (empty history)

 read 0x03 : t[7]=0 -> literal, n=3 -> copy 3 bytes: 'a''b''c'
             out = "abc"

 read 0xF0 : t[7]=1 -> match
             lf = (0xF0>>4)&7 = 7 -> M = 7+2 = 9
             dist_hi = 0xF0 & 0x0F = 0
             dist_lo = 0x02
             D = (0<<8 | 2) + 1 = 3
             src = len(out) - D = 3 - 3 = 0
             copy 9 bytes, one at a time, from src forward:
               k0: out[0]='a' -> out="abca"
               k1: out[1]='b' -> out="abcab"
               k2: out[2]='c' -> out="abcabc"
               k3: out[3]='a' -> out="abcabca"   (reading a byte we just wrote!)
               ... continues ...
             out = "abcabcabcabc"   (12 bytes)

 out length == orig_len(12) -> done.
```

The **byte-at-a-time** copy is what makes overlapping matches (`D < M`) work: by
the time we read `out[src+3]`, we have already written it. This is the single
most important correctness property of an LZ decoder, and the RTL preserves it
(see `lzo_decomp`).

## 1.5 Finding matches: a greedy hash table

The expensive part of compression is *finding* a previous occurrence of the
current bytes. A brute-force search (compare against every earlier position) is
O(n²). LZO-class encoders use a **hash table** instead:

```
   key  = the next 3 input bytes              (3 bytes = minimum match)
   h    = hash(key)                           -> small index
   cand = head[h]                             -> last position with this key
   head[h] = current position                 -> update for next time
```

`head[]` holds *one* candidate per hash bucket (single-entry, "greedy"). At each
position the encoder:

1. computes `h` from `input[i..i+2]`,
2. reads `cand = head[h]`, then stores `head[h] = i`,
3. if `cand` is valid, within max distance, and the bytes actually match
   (verified — hashes can collide), it extends the match as far as it goes,
4. if the match is ≥ `MIN_MATCH` it emits a match and jumps `i` past it;
   otherwise this byte becomes a literal and `i` advances by 1.

"Greedy" = take the first match found, don't search for a longer one later. It is
slightly suboptimal but very cheap in hardware (one candidate, one compare loop).

```
        input ───────────────► scan position i
        ┌───────────────────────────────────────────┐
        │  ... seen ...  [i][i+1][i+2] ...  future   │
        └───────────────────────────────────────────┘
                          │
                    hash 3 bytes
                          │
                          ▼
                   ┌──────────────┐   head[h] = previous pos with same key
        head[] :   │  hash table  │──────────────► cand
                   └──────────────┘
                          │ verify input[cand..] == input[i..], extend
                          ▼
                   length ≥ 3 ?  ── yes ─► emit MATCH(length, i-cand)
                          │ no
                          └────────────► emit literal, i++
```

## 1.6 Block model

The file is split into independent **4 KB blocks**. Each block is compressed on
its own, so a match's distance can never exceed the block (≤ 4096), and the whole
search window fits in one small on-chip RAM. The PS keeps a per-block header
(`original length`, `compressed length`); there is **no end-of-stream marker** —
the decoder simply stops once it has produced `orig_len` bytes. If a block does
not shrink (incompressible), the PS stores it raw with a "stored" flag.

```
 compressed_file.txt container (built by the PS, lzo_app.c):
   "LZP1" | u32 orig_total | u32 blk(4096) | u32 nblocks
   then per block:  u8 method(0=stored,1=lzo) | u32 olen | u32 slen | slen bytes
```

---

# Part 2 — System architecture (PS + PL)

```
   USB drive (FAT32)          Zynq PS  (Linux, Cortex-A9)            Zynq PL (fabric @100 MHz)
  ┌────────────────┐  fopen   ┌────────────────────────────┐ /dev/mem ┌──────────────────────────┐
  │ original_file   │─fread──► │ lzo_app                     │  mmap    │ lzo_top (AXI4-Lite slave) │
  │ compressed_file │◄fwrite─ │  • split into 4 KB blocks   │ AXI-Lite │   ┌─ bufA  (input block)  │
  │ uncompressed..  │◄fwrite─ │  • per block: write→start   │◄───────► │   ├─ bufB  (output block) │
  └────────────────┘          │    →poll BUSY→read          │          │   ├─ hash BRAM            │
        UART ◄── printf        │  • write container/files    │          │   ├─ lzo_comp             │
        (sizes, ratio, PASS)   └────────────────────────────┘          │   └─ lzo_decomp           │
                                                                        └──────────────────────────┘
```

There is **no DMA master**. The 4 KB input/output block buffers live directly in
the AXI-Lite address window, so the PS streams a block in word-by-word, pulses
start, polls `BUSY`, and reads the block out. Simple and robust; fast enough
because the files are small and each AXI-Lite access is a single `/dev/mem` store
or load.

### Per-block PS↔PL handshake
```
   PS                              PL (lzo_top)
   │ write IN_LEN / ORIG_LEN        │
   │ write BUF_IN[0..]  (block)     │
   │ write CTRL = start|mode  ──────► run-FSM: start engine, BUSY=1
   │ poll STATUS until BUSY rises   │   (compress or decompress in fabric)
   │ poll STATUS until BUSY falls ◄─── BUSY=0, OUT_LEN valid, DONE=1
   │ read OUT_LEN                   │
   │ read BUF_OUT[0..]  (result)    │
```

---

# Part 3 — RTL modules

```
                         lzo_top.sv  (AXI4-Lite slave)
   ┌──────────────────────────────────────────────────────────────────┐
   │  AXI-Lite write/read FSMs   control regs   run-FSM (start/ack/run) │
   │                                                                    │
   │   bufA (input, 32b)        bufB (output, 32b, byte-we)   hashBRAM  │
   │   ▲   │ byteA/byteB           ▲ write   │ history(byteB)    ▲  │    │
   │   │   ▼                       │         ▼                   │  ▼    │
   │  ┌─────────────┐            ┌──────────────┐          ┌──────────┐ │
   │  │  lzo_comp   │───tokens──►│   (bufB)     │          │  (hash)  │ │
   │  │             │◄──bytes────│   (bufA)     │          └──────────┘ │
   │  ├─────────────┤            ├──────────────┤                       │
   │  │ lzo_decomp  │◄─tokens────│   (bufA)     │                       │
   │  │             │───bytes───►│   (bufB)     │                       │
   │  └─────────────┘            └──────────────┘                       │
   └──────────────────────────────────────────────────────────────────┘
        lzo_top_v.v = thin Verilog wrapper (lets the block design instance it)
```

All memories are **synchronous** and all **read addresses are driven
combinationally** by the engines, so read data is valid the cycle *after* the
address is presented (the BRAM's own output register provides the one cycle of
latency). This is a deliberate, uniform timing contract — see Part 4.

---

## 3.1 `lzo_comp.sv` — the compressor

Greedy, single-candidate, hash-based LZ77 encoder that emits the token format of
Part 1.

### Interface
```
  start, in_len[AW:0]                          control
  done, out_len[OAW:0]                         status
  in_addra/in_doa, in_addrb/in_dob             input RAM, TRUE-DUAL-PORT read
                                                 A = current position, B = candidate
  out_we/out_addr/out_do                       output RAM, write
  ht_we/ht_addr/ht_wdata/ht_rdata              hash table, 1 read-or-write port
                                                 data = {valid, position}
```

### Why two input read ports?
Verifying/extending a match means comparing `input[i+k]` against
`input[cand+k]` for k = 0,1,2,… — two *different* addresses every cycle. A
true-dual-port RAM serves both in one cycle (port A = `i+k`, port B = `cand+k`).

### The hash
```
   hk = {b0,b1} ^ {b1,b2}        // 16-bit mix of the 3 bytes
   hp = hk * 16'd40503           // one DSP48 (odd constant spreads bits)
   h  = hp[28:16]                // 13-bit index -> 8192-entry table
```
Computed from **registered** bytes `b0,b1,b2` (not straight off the BRAM output)
so the multiply sits on a short register-to-register path — this is what closed
timing at 100 MHz (Part 4). Hash quality only affects *ratio*, never
correctness (matches are always byte-verified).

### State machine
```
        start
          │
        ┌─▼──────┐  clear 8192 hash entries (valid=0), one per cycle
        │ S_CLR  │◄─┐
        └─┬──────┘  │ clr_a++ until last
          ▼ (i=0)   ──┘
   ┌───►┌────────┐  i ≥ in_len ─────────────────────────────► S_FLUSH
   │    │ S_SCAN │  i+3 > in_len ─► S_LITERAL_ADV (tail bytes are literals)
   │    └─┬──────┘  else
   │      ▼
   │  S_RD0 ─ S_RD1 ─ S_RD2   read input[i],[i+1],[i+2] -> b0,b1,b2
   │      ▼
   │  S_HASH    present ht_addr = hash(b0,b1,b2)   (read head[h])
   │      ▼
   │  S_HRD     cand,cand_v <= ht_rdata
   │      ▼
   │  S_HWR     head[h] <= {1, i}                  (record current position)
   │      ▼
   │  S_DECIDE  cand valid & in range & cand<i ?
   │            │ no ───────────────────────────► S_LITERAL_ADV (i++) ─┐
   │            │ yes                                                   │
   │      ┌─────▼──────┐  ml=0                                          │
   │      │ S_CMP_SET  │◄─┐ present in_addra=i+ml, in_addrb=cand+ml     │
   │      └─────┬──────┘  │                                            │
   │      ┌─────▼──────┐  │ bytes equal & i+ml<in_len -> ml++          │
   │      │ S_CMP_GET  │──┘                                            │
   │      └─────┬──────┘  else: mlen=ml, mdist=i-cand                  │
   │            ▼                                                       │
   │      S_EMIT_DECIDE  mlen ≥ 3 ? ── no ──► S_LITERAL_ADV ───────────┤
   │            │ yes                                                   │
   │      ┌─────▼──────────────┐  flush pending literals [lit_start,i) │
   │      │ S_LIT_HDR/S_LIT_SET│  in ≤127-byte chunks                  │
   │      └─────┬──────────────┘                                       │
   │      ┌─────▼───────────────────────────┐ emit match token        │
   │      │ S_MAT_C0 ─►(S_MAT_EXT─►S_MAT_REM)│ control [+0xFF ext] ... │
   │      │          ─► S_MAT_DLO            │ ... dist_lo             │
   │      └─────┬───────────────────────────┘ i += mlen; lit_start=i  │
   │            └───────────────────────────────────────────────────►─┘
   │                                                          (back to S_SCAN)
   │    S_FLUSH  flush trailing literals [lit_start,in_len) ─► S_DONE
   └─── S_DONE   done=1 ─► S_IDLE
```

Key behaviours encoded above:
- **Literals are deferred.** Bytes with no match accumulate as a "pending
  literal run" `[lit_start, i)`; the run is flushed only when a match is found or
  at end-of-block, chunked into ≤127-byte literal tokens.
- **Match length extension.** `S_MAT_C0` writes the control byte; for length > 9
  `S_MAT_EXT/S_MAT_REM` write the `0xFF…/rem` chain, then `S_MAT_DLO` writes
  `dist_lo`.
- **The hash table is cleared every block** (`S_CLR`, 8192 cycles ≈ 82 µs) so a
  new block never matches against stale positions from the previous block.

### Output buffer write timing
`out_addr` is the running output length `olen`; `olen` increments the same cycle a
byte is written, so writes land at consecutive addresses with no gaps.

---

## 3.2 `lzo_decomp.sv` — the decompressor

A token interpreter: read control byte → either copy literals from the
compressed input, or copy a match from the history. The **output RAM doubles as
the history** via a second read port.

### Interface
```
  start, orig_len[AW:0]                 control (stop when out_count == orig_len)
  done, out_count[AW:0]                 status
  cin_addr / cin_do                     compressed input RAM, read
  out_we/out_addra/out_wdata            output RAM port A: WRITE
  out_addrb/out_dob                     output RAM port B: history READ
```

### State machine
```
        start (ip=0, oc=0)
          ▼
   ┌───►┌────────┐  oc ≥ orig_len ─► S_DONE
   │    │ S_LOOP │  else present cin_addr = ip
   │    └─┬──────┘
   │      ▼
   │   S_RDT     t <= cin_do ; ip++
   │      ▼
   │   S_DECODE  t[7]==0 ? LITERAL : MATCH
   │     │                               │
   │     │ literal                       │ match
   │     │ n=t[6:0]                      │ lf=t[6:4], dist_hi=t[3:0]
   │     │ n!=0: cnt=n                   │ lf!=0: cnt=lf+2
   │     │ n==0: ─► S_ELEN_* (0xFF ext)  │ lf==0: ─► S_ELEN_* (0xFF ext)
   │     ▼                               ▼
   │  ┌─ S_LIT_RD0 ◄─┐               S_DLO_RD0 ─ S_DLO_RD1   dist_lo <= cin_do
   │  │  cnt==0►LOOP │                   ▼
   │  │  ▼           │               S_MAT_PREP   src = oc - dist
   │  │ S_LIT_WR ────┘               ┌─ S_MAT_RD0 ◄─┐ cnt==0 ► LOOP
   │  │  out[oc]=cin_do              │  present out_addrb = src
   │  │  oc++,ip++,cnt--             │  ▼            │
   │  └─────────────                 │ S_MAT_RD1 ────┘ out[oc]=out_dob(history)
   │                                 │  oc++, src++, cnt--
   └─────────────────────────────────┘
        S_DONE  done=1 ─► S_IDLE
```

`S_ELEN_RD0/RD1/ADD` implement the shared `0xFF…` length-extension loop for both
literals and matches (`acc += 255` per `0xFF`, then `+ final byte`).

### Overlap correctness (the important bit)
A match copies `out[src+k]` → `out[oc+k]` one byte per pass through
`S_MAT_RD0 → S_MAT_RD1`. Because:
- it only ever reads `src = oc - D` with `D ≥ 1` (so `src < oc`, never the byte
  being written this cycle), and
- the nearest case `D = 1` needs `out[oc-1]`, which was written **two cycles
  earlier** (the previous `S_MAT_RD1`), so the true-dual-port BRAM has already
  committed it before the read address is presented,

there is never a same-cycle read/write hazard, and overlapping copies
(`D < M`, e.g. the `abcabc…` example) reproduce exactly.

---

## 3.3 `lzo_top.sv` — AXI4-Lite SoC wrapper

Wraps both engines, the buffers, and the hash RAM behind one AXI4-Lite slave.

### Register / memory map (base `0x4000_0000`, 64 KB)
```
   0x00000 CTRL    (W)  bit0 start pulse, bit1 mode (0=compress 1=decompress)
   0x00004 STATUS  (R)  bit0 done(latched), bit1 busy
   0x00008 IN_LEN  (RW) input length in bytes
   0x0000C ORIG_LEN(RW) decompress: expected output length
   0x00010 OUT_LEN (R)  bytes produced by the last run
   0x02000.. BUF_IN  (W) input block,  32-bit word w at 0x2000 + 4*w
   0x04000.. BUF_OUT (R) output block, 32-bit word w at 0x4000 + 4*w
```

### Buffers and byte/word packing
The PS accesses the buffers as 32-bit words; the engines work in bytes. Both
buffers are 32-bit-wide BRAMs (2048 words = 8 KB) with a byte-lane shim:

```
  bufA (input):  port A = PS word-write (idle) / engine word-read (run)
                 port B = compressor's candidate read
     engine byte at address a:  word = bufA[a>>2];  byte = word[ a[1:0]*8 +: 8 ]
        (the low address bits are registered alongside the word so the lane
         select lines up with the registered read data)

  bufB (output): port A = engine BYTE-write (run) / PS word-read (idle)
                 port B = decompressor's history read
     engine byte write to a:  bufB[a>>2][ a[1:0]*8 +: 8 ] <= byte   (byte-enable)
```

Each BRAM uses **at most 2 ports**: writes and the "other side" reads are
time-multiplexed by `busy` (PS touches the buffers only while idle, the engines
only while running), so nothing needs a third port.

### Run-control FSM (start → acknowledge → complete)
The engines hold `done` **high as a level** after finishing (cleared only by
their next start). Naively checking `done` right after issuing start would see the
*previous* run's completion. So the run-FSM uses a handshake:

```
        start_pulse (CTRL write, bit0)
              ▼
        ┌──────────┐  busy<=1, done_latched<=0, assert c_start/d_start (held)
        │  R_IDLE  │───────────────────────────────────────────► R_ACK
        └──────────┘
        ┌──────────┐  wait until the selected engine DEASSERTS done
        │  R_ACK   │  (= it accepted start) -> drop start  ─────► R_RUN
        └──────────┘
        ┌──────────┐  wait until the selected engine ASSERTS done
        │  R_RUN   │  (= real completion): busy<=0, done_latched<=1,
        └──────────┘   OUT_LEN<=engine length  ─────────────────► R_IDLE
```

This guarantees `busy` stays high for the *entire* operation and `OUT_LEN`/`DONE`
reflect *this* run — the multi-block correctness fix (Part 4).

### AXI-Lite slave
Two small always-blocks implement the write channel (latch AW+W, decode address,
pulse `start`/update regs/write `BUF_IN`, return B) and the read channel (register
the address, then drive `RDATA` from a register or from `BUF_OUT`). Both use
single-beat handshakes; there is no burst logic (AXI-Lite).

## 3.4 `lzo_top_v.v`
A thin Verilog wrapper with flat `s_axil_*` ports so the IP-integrator block
design infers the AXI4-Lite interface and can connect it to the PS `M_AXI_GP0`
through a SmartConnect. It just instances `lzo_top #(.C_AXIL_AW(16))`.

---

# Part 4 — Design decisions & gotchas

### Combinational read addresses (uniform 1-cycle RAM latency)
Every engine drives RAM **read addresses combinationally**; the synchronous BRAM
registers the address internally, so data is valid exactly one cycle later. If
the address were *registered* inside the FSM instead, a synchronous BRAM would
add a *second* cycle and every "data valid next state" assumption would be off by
one. Picking one convention and applying it everywhere keeps the FSMs simple and
correct.

### One-DSP hash from registered bytes (timing)
The first bitstream **failed timing** (WNS −2.7 ns): the path was BRAM-output →
24×32 multiplicative hash (2 DSP48s) → hash-RAM address, all combinational in one
cycle. Replacing it with a 16-bit one-DSP hash fed by *registered* bytes (a clean
reg→DSP→reg path, with the hash read presented the next cycle) closed timing at
+0.77 ns. Because matches are always byte-verified, the cheaper hash costs only a
hair of ratio (2.63× vs 2.65× on the test block).

### Run-FSM start/ack/complete — the multi-block bug
The very first hardware run compressed **block 0 correctly but reused block 0's
result for every later block.** Root cause: the engines' `done` is a level that
stays high after completion, and the original run-FSM sampled it the cycle after
start — so the *second* use of the same engine instantly "completed" with the old
`OUT_LEN` and stale output buffer. Block 0 worked only because `done` was 0 from
reset. The fix is the `R_IDLE→R_ACK→R_RUN` handshake above (wait for the engine
to drop `done` before watching for it to rise). The simulation had missed it
because it only ever used each engine *once* (compress then decompress);
`tb_top` now runs **two passes on the same engine** to cover it.

### Poll BUSY, not DONE (PS side)
For the same level-vs-edge reason, the PS driver waits on **BUSY** (rise then
fall), not the latched `DONE`, after issuing start. Operations run for tens of µs,
so `BUSY` is always observed.

### Block-independent, no in-stream EOF
Compressing 4 KB blocks independently bounds match distance to 4096 (fits a single
on-chip window) and lets the PS parallelise/skip incompressible blocks. The
decoder needs no end-of-stream token — it stops at `orig_len`, supplied per block
by the PS.

### Verification ladder
```
   tb_lzo : RTL→RTL round-trip · RTL decodes golden bytes · dist=1 overlap
   tb_top : full compress+decompress over AXI-Lite, two passes (same engine)
   golden : Python reference self round-trips and also decodes RTL output
   host   : lzo_app.c + lzo_sw.c (HOST_MOCK) — container/flow on a PC
   board  : 20000 → 7776 (2.59×), uncompressed == original, report on UART
```
