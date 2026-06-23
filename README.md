# lzo_pl — LZO file compression on the Zynq FPGA

Hardware-accelerated **LZO1X-style** file compression *and* decompression on the
Zynq FPGA. The compress/decompress engines are
hand-written SystemVerilog running in the FPGA fabric; the CPU runs light weight **Linux** and
does the USB file I/O, drives the FPGA over `/dev/mem`, and prints the report on
the UART console.

**Verified on real hardware:** `original_file.txt` 20000 B → `compressed_file.txt`
**7776 B** (2.59×, 61 % saved) → `uncompressed_file.txt` 20000 B, byte-identical
to the original.

## Flow
1. CPU reads `original_file.txt` from the USB drive.
2. CPU streams it in 4 KB blocks to the FPGA, which **compresses** in fabric; CPU
   writes `compressed_file.txt` to the USB drive.
3. CPU reads `compressed_file.txt` back and streams it to the FPGA, which
   **decompresses** in fabric; CPU writes `uncompressed_file.txt`.
4. CPU verifies `original == uncompressed` and prints sizes + ratio on the UART.

## Repo layout
```
rtl/   the FPGA design (SystemVerilog)
  lzo_comp.sv     greedy hash-based LZO1X-style block compressor
  lzo_decomp.sv   LZO1X-style block decompressor (history copy)
  lzo_top.sv      AXI4-Lite slave: control regs + in/out block buffers + both
                  engines + start/ack/complete run-FSM (no DMA master)
  lzo_top_v.v     thin Verilog wrapper so the block design can instance lzo_top

tb/    testbenches + vectors + reference model
  tb_lzo.sv       core: RTL↔RTL round-trip, RTL-decodes-golden, dist=1 overlap
  tb_top.sv       full compress+decompress over AXI-Lite, two passes (same engine
                  reused) to cover back-to-back operation
  lzo_golden.py   reference codec + token-format spec; emits the vectors below
  blk_in.hex      sample 4 KB input block          (one byte/line, hex)
  blk_comp.hex    golden-compressed bytes for blk_in
  blk_meta.txt    "N comp_len"  (block length, golden compressed length)
  sim.ps1         xsim runner for this layout

usb/
original_file.txt
compressed_file.txt
uncompressed_file.txt
```

## Token format (LZO1X-style, self-consistent)
Byte-aligned; defined once in `tb/lzo_golden.py` and mirrored exactly by the RTL
and `lzo_sw.c`. Reading control byte `t`:
- `t[7]==0` → literal run: `n=t[6:0]`; `n!=0` → L=n, `n==0` → L=128 + 0xFF-chain.
- `t[7]==1` → match: `lf=t[6:4]`; length `lf+2`, or `lf==0` → 10 + 0xFF-chain;
  distance `((t[3:0]<<8)|next_byte)+1` (range 1..4096). `MIN_MATCH=3`.

Round-trip is lossless by construction; output is not byte-identical to PC `lzop`.

## AXI-Lite map (base 0x40000000, 64 KB)
| Offset | Name | Dir | Meaning |
|--------|------|-----|---------|
| 0x00000 | CTRL | W | bit0 start, bit1 mode (0=compress 1=decompress) |
| 0x00004 | STATUS | R | bit0 done(latched), bit1 busy |
| 0x00008 | IN_LEN | RW | input length (bytes) |
| 0x0000C | ORIG_LEN | RW | decompress: expected output length |
| 0x00010 | OUT_LEN | R | bytes produced by the last run |
| 0x02000.. | BUF_IN | W | input block, 32-bit word at +4·w |
| 0x04000.. | BUF_OUT | R | output block, 32-bit word at +4·w |

The CPU writes a block in, starts the engine, **polls BUSY (rise then fall)**, and
reads the block out. (Polling the latched DONE right after start races against the
previous run — use BUSY.)

