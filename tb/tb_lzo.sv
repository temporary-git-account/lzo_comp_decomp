`timescale 1ns/1ps
//============================================================================
// tb_lzo - self-checking testbench for lzo_comp + lzo_decomp.
//   Phase A: RTL-compress blk_in.hex            -> out_c[], rc bytes
//   Phase B: RTL-decompress out_c (orig=N)      -> compare to blk_in (round-trip)
//   Phase C: RTL-decompress golden blk_comp.hex -> compare to blk_in (format)
//   Also exercises a dist=1 overlap run to stress the history-copy path.
//============================================================================
module tb_lzo;
    localparam int AW=13, OAW=14, HW=13;

    logic clk=0, rst=1;
    always #5 clk = ~clk;

    // ----------------- memories -----------------
    logic [7:0]  inbuf [0:8191];
    logic [7:0]  out_c [0:16383];
    logic [AW:0] hmem  [0:(1<<HW)-1];
    logic [7:0]  cinm  [0:16383];
    logic [7:0]  outh  [0:8191];

    integer N, GC, k, errors=0;

    // ----------------- comp DUT signals -----------------
    logic            c_start, c_done;
    logic [AW:0]     c_in_len;
    logic [OAW:0]    c_out_len;
    logic [AW-1:0]   c_in_addra, c_in_addrb;
    logic [7:0]      c_in_doa,   c_in_dob;
    logic            c_out_we;
    logic [OAW-1:0]  c_out_addr;
    logic [7:0]      c_out_do;
    logic            c_ht_we;
    logic [HW-1:0]   c_ht_addr;
    logic [AW:0]     c_ht_wdata, c_ht_rdata;

    lzo_comp #(.AW(AW),.OAW(OAW),.HW(HW)) UC (
        .clk(clk),.rst(rst),.start(c_start),.in_len(c_in_len),
        .done(c_done),.out_len(c_out_len),
        .in_addra(c_in_addra),.in_doa(c_in_doa),
        .in_addrb(c_in_addrb),.in_dob(c_in_dob),
        .out_we(c_out_we),.out_addr(c_out_addr),.out_do(c_out_do),
        .ht_we(c_ht_we),.ht_addr(c_ht_addr),.ht_wdata(c_ht_wdata),.ht_rdata(c_ht_rdata));

    // comp memories (sync read, comb addr from DUT)
    always_ff @(posedge clk) begin
        c_in_doa <= inbuf[c_in_addra];
        c_in_dob <= inbuf[c_in_addrb];
        if (c_out_we) out_c[c_out_addr] <= c_out_do;
        if (c_ht_we)  hmem[c_ht_addr]   <= c_ht_wdata;
        c_ht_rdata <= hmem[c_ht_addr];
    end

    // ----------------- decomp DUT signals -----------------
    logic            d_start, d_done;
    logic [AW:0]     d_orig_len, d_out_count;
    logic [13:0]     d_cin_addr;
    logic [7:0]      d_cin_do;
    logic            d_out_we;
    logic [AW-1:0]   d_out_addra, d_out_addrb;
    logic [7:0]      d_out_wdata, d_out_dob;

    lzo_decomp #(.AW(AW),.CAW(14)) UD (
        .clk(clk),.rst(rst),.start(d_start),.orig_len(d_orig_len),
        .done(d_done),.out_count(d_out_count),
        .cin_addr(d_cin_addr),.cin_do(d_cin_do),
        .out_we(d_out_we),.out_addra(d_out_addra),.out_wdata(d_out_wdata),
        .out_addrb(d_out_addrb),.out_dob(d_out_dob));

    always_ff @(posedge clk) begin
        d_cin_do <= cinm[d_cin_addr];
        if (d_out_we) outh[d_out_addra] <= d_out_wdata;
        d_out_dob <= outh[d_out_addrb];
    end

    // ----------------- helpers -----------------
    task automatic run_comp;
        begin
            @(posedge clk); c_start<=1; @(posedge clk); c_start<=0;
            wait (c_done==0); wait (c_done==1); @(posedge clk);
        end
    endtask
    task automatic run_decomp(input integer orig);
        begin
            d_orig_len = orig[AW:0];
            @(posedge clk); d_start<=1; @(posedge clk); d_start<=0;
            wait (d_done==0); wait (d_done==1); @(posedge clk);
        end
    endtask

    integer fd, rc;
    initial begin
        c_start=0; d_start=0;
        // read meta (N comp_len)
        fd = $fopen("blk_meta.txt","r");
        if (fd==0) begin $display("FATAL: blk_meta.txt missing"); $finish; end
        rc = $fscanf(fd, "%d %d", N, GC); $fclose(fd);
        $display("meta: N=%0d  golden_comp=%0d", N, GC);

        for (k=0;k<8192;k++)  inbuf[k]=8'h00;
        for (k=0;k<16384;k++) out_c[k]=8'h00;
        $readmemh("blk_in.hex",   inbuf);

        repeat(4) @(posedge clk); rst<=0; repeat(2) @(posedge clk);

        // ---- Phase A: compress ----
        c_in_len = N[AW:0];
        run_comp();
        rc = c_out_len;
        $display("PhaseA compress: %0d -> %0d bytes (ratio %0d.%02dx)",
                 N, rc, (N*100/rc)/100, (N*100/rc)%100);
        // dump rtl_comp.hex
        fd = $fopen("rtl_comp.hex","w");
        for (k=0;k<rc;k++) $fwrite(fd,"%02x\n", out_c[k]);
        $fclose(fd);

        // ---- Phase B: decompress RTL output, expect == inbuf ----
        for (k=0;k<16384;k++) cinm[k]=8'h00;
        for (k=0;k<rc;k++)    cinm[k]=out_c[k];
        for (k=0;k<8192;k++)  outh[k]=8'h00;
        run_decomp(N);
        $display("PhaseB decompress: produced %0d bytes", d_out_count);
        for (k=0;k<N;k++) if (outh[k]!==inbuf[k]) begin
            errors++; if (errors<=8) $display("  B mismatch @%0d: got %02x exp %02x",k,outh[k],inbuf[k]);
        end
        $display("PhaseB round-trip: %s", (errors==0)?"PASS":"FAIL");

        // ---- Phase C: decompress GOLDEN bytes, expect == inbuf ----
        begin integer e0; e0=errors;
            for (k=0;k<16384;k++) cinm[k]=8'h00;
            $readmemh("blk_comp.hex", cinm);
            for (k=0;k<8192;k++) outh[k]=8'h00;
            run_decomp(N);
            $display("PhaseC decompress(golden): produced %0d bytes", d_out_count);
            for (k=0;k<N;k++) if (outh[k]!==inbuf[k]) begin
                errors++; if (errors-e0<=8) $display("  C mismatch @%0d: got %02x exp %02x",k,outh[k],inbuf[k]);
            end
            $display("PhaseC golden-format: %s", (errors==e0)?"PASS":"FAIL");
        end

        // ---- Phase D: dist=1 overlap stress (RLE) ----
        begin integer e0; integer M; e0=errors; M=300;
            // build a compressed stream: 1 literal 'A', then match len=M dist=1
            // control literal: n=1 -> 0x01, byte 'A'
            cinm[0]=8'h01; cinm[1]=8'h41;
            // match extended: lf=0 -> 0x80 | dist_hi(0); len=M -> rem=M-10=290 ->255,35; dist_lo=0 (dist=1)
            cinm[2]=8'h80; cinm[3]=8'hFF; cinm[4]=8'd35; cinm[5]=8'h00;
            for (k=0;k<8192;k++) outh[k]=8'h00;
            run_decomp(1+M);
            for (k=0;k<1+M;k++) if (outh[k]!==8'h41) begin
                errors++; if (errors-e0<=8) $display("  D mismatch @%0d: got %02x",k,outh[k]);
            end
            $display("PhaseD overlap(dist=1,len=%0d): %s", M, (errors==e0)?"PASS":"FAIL");
        end

        $display("==== TB DONE, errors=%0d ====", errors);
        $finish;
    end

    initial begin #20000000; $display("TIMEOUT"); $finish; end
endmodule
