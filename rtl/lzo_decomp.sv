`timescale 1ns/1ps
//============================================================================
// lzo_decomp - LZO1X-style block decompressor (exact inverse of the format in
//   script/lzo_golden.py).  Reads tokens from `cin`, emits orig_len bytes into
//   `out` (which doubles as back-reference history via a 2nd read port).
//
//   Token: control t -
//     t[7]==0 : literal run, n=t[6:0]; n!=0 -> L=n; n==0 -> L=128 + 0xFF-chain.
//     t[7]==1 : match, lf=t[6:4]; lf!=0 -> M=lf+2; lf==0 -> M=10 + 0xFF-chain;
//               dist = ((t[3:0]<<8)|next_byte) + 1; copy M from out[oc-dist].
//
//   READ ADDRESSES ARE COMBINATIONAL (1-cycle sync-RAM latency).  Copies read
//   out[src] with src<oc only, and the nearest (dist==1) source byte was
//   written two cycles earlier, so no same-cycle read/write conflict.
//============================================================================
module lzo_decomp #(
    parameter int AW  = 13,
    parameter int CAW = 14
)(
    input  logic            clk,
    input  logic            rst,
    input  logic            start,
    input  logic [AW:0]     orig_len,
    output logic            done,
    output logic [AW:0]     out_count,

    output logic [CAW-1:0]  cin_addr,
    input  logic [7:0]      cin_do,

    output logic            out_we,
    output logic [AW-1:0]   out_addra,
    output logic [7:0]      out_wdata,
    output logic [AW-1:0]   out_addrb,
    input  logic [7:0]      out_dob
);
    typedef enum logic [4:0] {
        S_IDLE, S_LOOP, S_RDT, S_DECODE,
        S_ELEN_RD0, S_ELEN_RD1, S_ELEN_ADD,
        S_LIT_RD0, S_LIT_WR,
        S_DLO_RD0, S_DLO_RD1, S_MAT_PREP, S_MAT_RD0, S_MAT_RD1,
        S_DONE
    } state_t;

    state_t st, ext_ret;

    logic [CAW-1:0] ip;
    logic [AW:0]    oc;
    logic [7:0]     t, eb, dist_lo;
    logic [3:0]     dist_hi;
    logic [AW:0]    acc, cnt, src;

    wire [11:0] dist12 = {dist_hi, dist_lo};               // 0..4095
    wire [AW:0] bdist  = {{(AW-11){1'b0}}, dist12} + 1'b1; // 1..4096

    assign out_count = oc;

    // ---------------- combinational memory interface ----------------------
    always_comb begin
        cin_addr  = ip;
        out_we    = 1'b0;  out_addra = oc[AW-1:0];  out_wdata = cin_do;
        out_addrb = src[AW-1:0];
        case (st)
            S_LIT_WR:  begin out_we=1'b1; out_addra=oc[AW-1:0]; out_wdata=cin_do;  end
            S_MAT_RD1: begin out_we=1'b1; out_addra=oc[AW-1:0]; out_wdata=out_dob; end
            default: ;
        endcase
    end

    // ---------------- sequential state ------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin st <= S_IDLE; done <= 1'b0; end
        else case (st)
        S_IDLE: if (start) begin done<=1'b0; ip<='0; oc<='0; st<=S_LOOP; end
        S_LOOP: if (oc>=orig_len) st<=S_DONE; else st<=S_RDT;
        S_RDT:  begin t<=cin_do; ip<=ip+1'b1; st<=S_DECODE; end
        S_DECODE:
            if (t[7]==1'b0) begin                          // literal
                if (t[6:0]!=7'd0) begin cnt<={{(AW-6){1'b0}}, t[6:0]}; st<=S_LIT_RD0; end
                else begin acc<='d128; ext_ret<=S_LIT_RD0; st<=S_ELEN_RD0; end
            end else begin                                 // match
                dist_hi<=t[3:0];
                if (t[6:4]!=3'd0) begin cnt<={{(AW-2){1'b0}}, t[6:4]} + 'd2; st<=S_DLO_RD0; end
                else begin acc<='d10; ext_ret<=S_DLO_RD0; st<=S_ELEN_RD0; end
            end
        // ---- 0xFF length-extension chain (shared) ------------------------
        S_ELEN_RD0: st<=S_ELEN_RD1;
        S_ELEN_RD1: begin eb<=cin_do; ip<=ip+1'b1; st<=S_ELEN_ADD; end
        S_ELEN_ADD: if (eb==8'hFF) begin acc<=acc+'d255; st<=S_ELEN_RD0; end
                    else begin cnt<=acc+eb; st<=ext_ret; end
        // ---- literal copy ------------------------------------------------
        S_LIT_RD0: if (cnt==0) st<=S_LOOP; else st<=S_LIT_WR;
        S_LIT_WR:  begin oc<=oc+1'b1; ip<=ip+1'b1; cnt<=cnt-1'b1; st<=S_LIT_RD0; end
        // ---- match: read dist_lo, then copy from history ----------------
        S_DLO_RD0: st<=S_DLO_RD1;
        S_DLO_RD1: begin dist_lo<=cin_do; ip<=ip+1'b1; st<=S_MAT_PREP; end
        S_MAT_PREP: begin src<=oc-bdist; st<=S_MAT_RD0; end
        S_MAT_RD0: if (cnt==0) st<=S_LOOP; else st<=S_MAT_RD1;
        S_MAT_RD1: begin oc<=oc+1'b1; src<=src+1'b1; cnt<=cnt-1'b1; st<=S_MAT_RD0; end
        S_DONE: begin done<=1'b1; st<=S_IDLE; end
        default: st<=S_IDLE;
        endcase
    end
endmodule
