`timescale 1ns/1ps
//============================================================================
// lzo_top - AXI4-Lite slave wrapping lzo_comp + lzo_decomp for the Z-turn PL.
//
//   No AXI master / DMA: the input and output block buffers are mapped DIRECTLY
//   in the AXI-Lite window so the PS (Linux, /dev/mem) streams a 4 KB block in,
//   pulses start, polls done, and reads the result block out.
//
//   Register / memory map (byte offsets within the 64 KB AXI-Lite space):
//     0x00000 CTRL    (W)  bit0: start pulse   bit1: mode (0=compress 1=decompress)
//     0x00004 STATUS  (R)  bit0: done(latched) bit1: busy
//     0x00008 IN_LEN  (RW) input length in bytes
//     0x0000C ORIG_LEN(RW) decompress: expected output length (= original block size)
//     0x00010 OUT_LEN (R)  bytes produced by the last run
//     0x02000.. BUF_IN  (W) input block, 32-bit word w at 0x2000 + 4*w  (<=2048 words)
//     0x04000.. BUF_OUT (R) output block, 32-bit word w at 0x4000 + 4*w
//============================================================================
module lzo_top #(
    parameter int C_AXIL_AW = 16,
    parameter int AW   = 13,          // engine byte-addr width
    parameter int OAW  = 14,
    parameter int HW   = 13,
    parameter int WORDS = 2048        // buffer depth in 32-bit words (8 KB)
)(
    input  logic                  clk,
    input  logic                  resetn,

    input  logic [C_AXIL_AW-1:0]  s_axil_awaddr,
    input  logic [2:0]            s_axil_awprot,
    input  logic                  s_axil_awvalid,
    output logic                  s_axil_awready,
    input  logic [31:0]           s_axil_wdata,
    input  logic [3:0]            s_axil_wstrb,
    input  logic                  s_axil_wvalid,
    output logic                  s_axil_wready,
    output logic [1:0]            s_axil_bresp,
    output logic                  s_axil_bvalid,
    input  logic                  s_axil_bready,
    input  logic [C_AXIL_AW-1:0]  s_axil_araddr,
    input  logic [2:0]            s_axil_arprot,
    input  logic                  s_axil_arvalid,
    output logic                  s_axil_arready,
    output logic [31:0]           s_axil_rdata,
    output logic [1:0]            s_axil_rresp,
    output logic                  s_axil_rvalid,
    input  logic                  s_axil_rready
);
    wire rst = ~resetn;
    localparam int WAW = $clog2(WORDS);

    // ---------------- control / status regs ----------------
    logic        start_pulse, mode;          // mode: 0=comp 1=decomp
    logic        busy, done_latched;
    logic [AW:0] r_in_len, r_orig_len;
    logic [OAW:0] r_out_len;
    logic        running_comp, running_decomp;

    // ---------------- engines ----------------
    logic            c_start, c_done;  logic [OAW:0] c_out_len;
    logic [AW-1:0]   c_in_addra, c_in_addrb;  logic [7:0] c_in_doa, c_in_dob;
    logic            c_out_we;  logic [OAW-1:0] c_out_addr;  logic [7:0] c_out_do;
    logic            c_ht_we;   logic [HW-1:0]  c_ht_addr;   logic [AW:0] c_ht_wdata, c_ht_rdata;

    logic            d_start, d_done;  logic [AW:0] d_out_count;
    logic [13:0]     d_cin_addr;  logic [7:0] d_cin_do;
    logic            d_out_we;  logic [AW-1:0] d_out_addra, d_out_addrb;
    logic [7:0]      d_out_wdata, d_out_dob;

    lzo_comp #(.AW(AW),.OAW(OAW),.HW(HW)) UC (
        .clk(clk),.rst(rst),.start(c_start),.in_len(r_in_len),
        .done(c_done),.out_len(c_out_len),
        .in_addra(c_in_addra),.in_doa(c_in_doa),.in_addrb(c_in_addrb),.in_dob(c_in_dob),
        .out_we(c_out_we),.out_addr(c_out_addr),.out_do(c_out_do),
        .ht_we(c_ht_we),.ht_addr(c_ht_addr),.ht_wdata(c_ht_wdata),.ht_rdata(c_ht_rdata));

    lzo_decomp #(.AW(AW),.CAW(14)) UD (
        .clk(clk),.rst(rst),.start(d_start),.orig_len(r_orig_len),
        .done(d_done),.out_count(d_out_count),
        .cin_addr(d_cin_addr),.cin_do(d_cin_do),
        .out_we(d_out_we),.out_addra(d_out_addra),.out_wdata(d_out_wdata),
        .out_addrb(d_out_addrb),.out_dob(d_out_dob));

    // ---------------- hash BRAM (compressor only) ----------------
    (* ram_style="block" *) logic [AW:0] htmem [0:(1<<HW)-1];
    always_ff @(posedge clk) begin
        if (c_ht_we) htmem[c_ht_addr] <= c_ht_wdata;
        c_ht_rdata <= htmem[c_ht_addr];
    end

    // ---------------- AXI-Lite write channel ----------------
    localparam logic [1:0] SEL_REG=2'b00, SEL_IN=2'b01, SEL_OUT=2'b10;
    wire [1:0]      wsel = s_axil_awaddr[14:13];
    wire [WAW-1:0]  wword = s_axil_awaddr[2+WAW-1:2];

    logic [C_AXIL_AW-1:0] awaddr_q;  logic [31:0] wdata_q;  logic aw_hs, w_hs;
    logic        bufin_we;  logic [WAW-1:0] bufin_waddr;  logic [31:0] bufin_wdata;

    assign s_axil_bresp = 2'b00;
    always_ff @(posedge clk) begin
        if (rst) begin
            s_axil_awready<=0; s_axil_wready<=0; s_axil_bvalid<=0;
            aw_hs<=0; w_hs<=0; start_pulse<=0; mode<=0;
            r_in_len<=0; r_orig_len<=0; bufin_we<=0;
        end else begin
            start_pulse<=0; bufin_we<=0;
            if (s_axil_awvalid && !aw_hs) begin s_axil_awready<=1; awaddr_q<=s_axil_awaddr; aw_hs<=1; end
            else s_axil_awready<=0;
            if (s_axil_wvalid && !w_hs) begin s_axil_wready<=1; wdata_q<=s_axil_wdata; w_hs<=1; end
            else s_axil_wready<=0;
            if (aw_hs && w_hs && !s_axil_bvalid) begin
                case (awaddr_q[14:13])
                  SEL_REG: case (awaddr_q[7:0])
                      8'h00: begin start_pulse<=wdata_q[0]; mode<=wdata_q[1]; end
                      8'h08: r_in_len  <= wdata_q[AW:0];
                      8'h0C: r_orig_len<= wdata_q[AW:0];
                      default: ;
                  endcase
                  SEL_IN: begin bufin_we<=1; bufin_waddr<=awaddr_q[2+WAW-1:2]; bufin_wdata<=wdata_q; end
                  default: ;
                endcase
                s_axil_bvalid<=1; aw_hs<=0; w_hs<=0;
            end else if (s_axil_bvalid && s_axil_bready) s_axil_bvalid<=0;
        end
    end

    // ---------------- run control FSM ----------------
    // The engines hold `done` HIGH after finishing (level, cleared only by their
    // next start).  So we must not sample done right after issuing start - it
    // would still show the PREVIOUS run's completion.  Sequence: assert start,
    // wait for the engine to ACKNOWLEDGE by deasserting done, THEN wait for done
    // to re-assert = real completion.  (start is held until acknowledged.)
    typedef enum logic [1:0] { R_IDLE, R_ACK, R_RUN } rstate_t;
    rstate_t rstate;
    wire sel_done = running_comp ? c_done : d_done;
    always_ff @(posedge clk) begin
        if (rst) begin
            busy<=0; done_latched<=0; running_comp<=0; running_decomp<=0;
            c_start<=0; d_start<=0; r_out_len<=0; rstate<=R_IDLE;
        end else begin
            case (rstate)
            R_IDLE: if (start_pulse) begin
                busy<=1; done_latched<=0;
                if (mode==1'b0) begin running_comp<=1; running_decomp<=0; c_start<=1; end
                else            begin running_decomp<=1; running_comp<=0; d_start<=1; end
                rstate<=R_ACK;
            end
            R_ACK: if (sel_done == 1'b0) begin     // engine accepted start
                c_start<=0; d_start<=0; rstate<=R_RUN;
            end
            R_RUN: if (sel_done) begin             // real completion
                busy<=0; done_latched<=1;
                r_out_len <= running_comp ? c_out_len : {1'b0, d_out_count};
                running_comp<=0; running_decomp<=0; rstate<=R_IDLE;
            end
            default: rstate<=R_IDLE;
            endcase
        end
    end

    // ================= buffer A (input) : 32-bit, 2 ports =================
    //   port A: PS write & read (idle) / engine word read (run)   port B: comp candidate read
    (* ram_style="block" *) logic [31:0] bufA [0:WORDS-1];
    wire [AW-1:0]  engA_baddr = running_decomp ? d_cin_addr[AW-1:0] : c_in_addra;
    wire [WAW-1:0] bufA_aA = busy ? engA_baddr[2+WAW-1:2] : bufin_waddr;
    wire [WAW-1:0] bufA_aB = c_in_addrb[2+WAW-1:2];
    logic [31:0] bufA_qA, bufA_qB;
    logic [1:0]  laneA_q, laneB_q;
    always_ff @(posedge clk) begin
        if (!busy && bufin_we) bufA[bufA_aA] <= bufin_wdata;   // write shares port A
        bufA_qA <= bufA[bufA_aA];  laneA_q <= engA_baddr[1:0];
        bufA_qB <= bufA[bufA_aB];  laneB_q <= c_in_addrb[1:0];
    end
    wire [7:0] byteA = bufA_qA[laneA_q*8 +: 8];
    wire [7:0] byteB = bufA_qB[laneB_q*8 +: 8];
    assign c_in_doa = byteA;  assign c_in_dob = byteB;  assign d_cin_do = byteA;

    // ================= buffer B (output) : 32-bit, 2 ports, byte-we =================
    //   port A: engine byte-write (run) / PS word read (idle)   port B: decomp history read
    (* ram_style="block" *) logic [31:0] bufB [0:WORDS-1];
    wire           engB_we   = running_comp ? c_out_we   : d_out_we;
    wire [OAW-1:0] engB_ba   = running_comp ? c_out_addr : {1'b0,d_out_addra};
    wire [7:0]     engB_wd   = running_comp ? c_out_do   : d_out_wdata;
    wire [WAW-1:0] engB_word = engB_ba[2+WAW-1:2];
    wire [1:0]     engB_lane = engB_ba[1:0];
    wire [WAW-1:0] bufB_aA   = busy ? engB_word : s_axil_araddr[2+WAW-1:2];
    logic [31:0] bufB_qPS;
    always_ff @(posedge clk) begin
        if (busy && engB_we) bufB[bufB_aA][engB_lane*8 +: 8] <= engB_wd;  // port A write
        bufB_qPS <= bufB[bufB_aA];                                        // port A read (PS, idle)
    end
    logic [31:0] bufB_qH;  logic [1:0] laneH_q;
    always_ff @(posedge clk) begin
        bufB_qH <= bufB[d_out_addrb[2+WAW-1:2]];                          // port B read (history)
        laneH_q <= d_out_addrb[1:0];
    end
    assign d_out_dob = bufB_qH[laneH_q*8 +: 8];

    // ---------------- AXI-Lite read channel ----------------
    logic [C_AXIL_AW-1:0] araddr_q;
    logic [1:0] rsel_q;
    always_ff @(posedge clk) begin
        if (rst) begin s_axil_arready<=0; s_axil_rvalid<=0; s_axil_rdata<=0; end
        else begin
            if (s_axil_arvalid && !s_axil_arready && !s_axil_rvalid) begin
                s_axil_arready<=1; araddr_q<=s_axil_araddr; rsel_q<=s_axil_araddr[14:13];
            end else s_axil_arready<=0;

            if (s_axil_arready) begin
                s_axil_rvalid<=1;            // bufB_qPS already reflects araddr (registered read)
                if (rsel_q==SEL_OUT) s_axil_rdata <= bufB_qPS;
                else case (araddr_q[7:0])
                    8'h04: s_axil_rdata <= {30'd0, busy, done_latched};
                    8'h08: s_axil_rdata <= {{(32-AW-1){1'b0}}, r_in_len};
                    8'h0C: s_axil_rdata <= {{(32-AW-1){1'b0}}, r_orig_len};
                    8'h10: s_axil_rdata <= {{(32-OAW-1){1'b0}}, r_out_len};
                    default: s_axil_rdata <= 32'd0;
                endcase
            end else if (s_axil_rvalid && s_axil_rready) s_axil_rvalid<=0;
        end
    end
    assign s_axil_rresp = 2'b00;
endmodule
