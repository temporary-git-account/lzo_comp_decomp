`timescale 1ns/1ps
//============================================================================
// lzo_comp - greedy hash-based LZO1X-style block compressor.
//
//   Reproduces the TOKEN FORMAT of script/lzo_golden.py (not its exact match
//   choices); output decodes losslessly via lzo_decomp / golden.
//
//   At pos i: h = hash3(in[i..i+2]); cand = head[h]; head[h] = i;
//   if cand within MAX_DIST, extend the match; length>=3 -> emit match, else
//   the byte joins the pending literal run.  Literal runs emit in chunks of
//   <=127 (extended-literal encoding unused).  Matches >9 use the 0xFF
//   length-extension.
//
//   Memories are external & synchronous; READ ADDRESSES ARE COMBINATIONAL so
//   data is valid the cycle after the address is presented.  Hash table is
//   cleared (valid=0) at the start of every block.
//============================================================================
module lzo_comp #(
    parameter int AW   = 13,
    parameter int OAW  = 14,
    parameter int HW   = 13,
    parameter int MAXD = 4096,
    parameter int MINM = 3
)(
    input  logic            clk,
    input  logic            rst,
    input  logic            start,
    input  logic [AW:0]     in_len,
    output logic            done,
    output logic [OAW:0]    out_len,

    output logic [AW-1:0]   in_addra,
    input  logic [7:0]      in_doa,
    output logic [AW-1:0]   in_addrb,
    input  logic [7:0]      in_dob,

    output logic            out_we,
    output logic [OAW-1:0]  out_addr,
    output logic [7:0]      out_do,

    output logic            ht_we,
    output logic [HW-1:0]   ht_addr,
    output logic [AW:0]     ht_wdata,
    input  logic [AW:0]     ht_rdata
);
    localparam int HSIZE = (1<<HW);

    typedef enum logic [4:0] {
        S_IDLE, S_CLR, S_SCAN,
        S_RD0, S_RD1, S_RD2, S_HASH, S_HRD, S_HWR, S_DECIDE,
        S_CMP_SET, S_CMP_GET, S_EMIT_DECIDE,
        S_LIT_HDR, S_LIT_SET,
        S_MAT_C0, S_MAT_EXT, S_MAT_REM, S_MAT_DLO,
        S_LITERAL_ADV, S_FLUSH, S_DONE
    } state_t;

    state_t st, lit_ret;

    logic [AW:0]   i, lit_start;
    logic [OAW:0]  olen;
    logic [7:0]    b0, b1, b2;
    logic [HW-1:0] h_reg;
    logic [AW:0]   cand;
    logic          cand_v;
    logic [AW:0]   ml, mlen, mdist;
    logic [HW-1:0] clr_a;
    logic [AW:0]   lf_pos, lf_end;
    logic [7:0]    lf_chunk, lf_k;
    logic [AW:0]   mrem;

    // one-DSP hash from REGISTERED bytes (short, timing-friendly path).
    // hash quality only affects ratio, not correctness.
    function automatic logic [HW-1:0] hashf(input logic [7:0] x0,x1,x2);
        logic [15:0] hk; logic [31:0] hp;
        begin hk = {x0,x1} ^ {x1,x2}; hp = hk * 16'd40503; hashf = hp[16 +: HW]; end
    endfunction

    wire [AW:0]   lit_remain = lf_end - lf_pos;
    wire [7:0]    lit_chunk  = (lit_remain >= 127) ? 8'd127 : lit_remain[7:0];
    wire [HW-1:0] h_comb     = hashf(b0, b1, b2);       // valid from S_HASH (regs)
    wire [7:0]    mat_c0     = (mlen <= 9)
                  ? (8'h80 | ((mlen[3:0]-4'd2) << 4) | (((mdist-1) >> 8) & 8'h0F))
                  : (8'h80 | (((mdist-1) >> 8) & 8'h0F));

    assign out_len = olen;

    // ---------------- combinational memory interface ----------------------
    always_comb begin
        in_addra = i[AW-1:0];
        in_addrb = cand[AW-1:0];
        out_we   = 1'b0;  out_addr = olen[OAW-1:0];  out_do = 8'h00;
        ht_we    = 1'b0;  ht_addr  = h_reg;          ht_wdata = {1'b1, i[AW-1:0]};
        case (st)
            S_RD0: in_addra = i[AW-1:0] + 1;
            S_RD1: in_addra = i[AW-1:0] + 2;
            S_HASH: ht_addr = h_comb;                       // present hash read (regs->1 DSP)
            S_HWR: begin ht_we=1'b1; ht_addr=h_reg; ht_wdata={1'b1, i[AW-1:0]}; end
            S_CLR: begin ht_we=1'b1; ht_addr=clr_a; ht_wdata='0; end
            S_CMP_SET: begin in_addra=i[AW-1:0]+ml[AW-1:0]; in_addrb=cand[AW-1:0]+ml[AW-1:0]; end
            S_LIT_HDR: begin in_addra=lf_pos[AW-1:0];
                       if (lf_pos<lf_end) begin out_we=1'b1; out_addr=olen[OAW-1:0]; out_do=lit_chunk; end end
            S_LIT_SET: begin in_addra=lf_pos[AW-1:0]+1;     // prefetch next literal
                       if (lf_k<lf_chunk) begin out_we=1'b1; out_addr=olen[OAW-1:0]; out_do=in_doa; end end
            S_MAT_C0:  begin out_we=1'b1; out_addr=olen[OAW-1:0]; out_do=mat_c0; end
            S_MAT_EXT: if (mrem>=255) begin out_we=1'b1; out_addr=olen[OAW-1:0]; out_do=8'hFF; end
            S_MAT_REM: begin out_we=1'b1; out_addr=olen[OAW-1:0]; out_do=mrem[7:0]; end
            S_MAT_DLO: begin out_we=1'b1; out_addr=olen[OAW-1:0]; out_do=(mdist-1)&8'hFF; end
            default: ;
        endcase
    end

    // ---------------- sequential state ------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin st <= S_IDLE; done <= 1'b0; end
        else case (st)
        S_IDLE: if (start) begin done<=1'b0; clr_a<='0; st<=S_CLR; end
        S_CLR:  if (clr_a==HSIZE-1) begin i<='0; lit_start<='0; olen<='0; st<=S_SCAN; end
                else clr_a <= clr_a + 1'b1;
        S_SCAN: if (i>=in_len) st<=S_FLUSH;
                else if (i+MINM>in_len) st<=S_LITERAL_ADV;
                else st<=S_RD0;
        S_RD0:  begin b0<=in_doa; st<=S_RD1; end
        S_RD1:  begin b1<=in_doa; st<=S_RD2; end
        S_RD2:  begin b2<=in_doa; st<=S_HASH; end
        S_HASH: begin h_reg<=h_comb; st<=S_HRD; end
        S_HRD:  begin cand<=ht_rdata[AW-1:0]; cand_v<=ht_rdata[AW]; st<=S_HWR; end
        S_HWR:  st<=S_DECIDE;
        S_DECIDE: if (cand_v && (i-cand)<=MAXD && cand<i) begin ml<='0; st<=S_CMP_SET; end
                  else st<=S_LITERAL_ADV;
        S_CMP_SET: st<=S_CMP_GET;
        S_CMP_GET: if ((i+ml)<in_len && in_doa==in_dob) begin ml<=ml+1'b1; st<=S_CMP_SET; end
                   else begin mlen<=ml; mdist<=i-cand; st<=S_EMIT_DECIDE; end
        S_EMIT_DECIDE: if (mlen>=MINM) begin
                           lf_pos<=lit_start; lf_end<=i; lit_ret<=S_MAT_C0; st<=S_LIT_HDR;
                       end else st<=S_LITERAL_ADV;
        S_LIT_HDR: if (lf_pos>=lf_end) st<=lit_ret;
                   else begin lf_chunk<=lit_chunk; lf_k<='0; olen<=olen+1'b1; st<=S_LIT_SET; end
        S_LIT_SET: if (lf_k==lf_chunk) st<=S_LIT_HDR;
                   else begin olen<=olen+1'b1; lf_k<=lf_k+1'b1; lf_pos<=lf_pos+1'b1; st<=S_LIT_SET; end
        S_MAT_C0:  begin olen<=olen+1'b1;
                       if (mlen<=9) st<=S_MAT_DLO;
                       else begin mrem<=mlen-10; st<=S_MAT_EXT; end end
        S_MAT_EXT: if (mrem>=255) begin olen<=olen+1'b1; mrem<=mrem-255; st<=S_MAT_EXT; end
                   else st<=S_MAT_REM;
        S_MAT_REM: begin olen<=olen+1'b1; st<=S_MAT_DLO; end
        S_MAT_DLO: begin olen<=olen+1'b1; i<=i+mlen; lit_start<=i+mlen; st<=S_SCAN; end
        S_LITERAL_ADV: begin i<=i+1'b1; st<=S_SCAN; end
        S_FLUSH: if (lit_start>=in_len) st<=S_DONE;
                 else begin lf_pos<=lit_start; lf_end<=in_len; lit_start<=in_len;
                            lit_ret<=S_DONE; st<=S_LIT_HDR; end
        S_DONE: begin done<=1'b1; st<=S_IDLE; end
        default: st<=S_IDLE;
        endcase
    end
endmodule
