`timescale 1ns/1ps
//============================================================================
// tb_top - drive lzo_top over AXI4-Lite: compress a block, read it back,
//   then decompress it back through the same memory-mapped interface, and
//   check the round-trip == original.
//============================================================================
module tb_top;
    localparam int AWID = 16;

    logic clk=0, resetn=0;
    always #5 clk=~clk;

    logic [AWID-1:0] awaddr; logic awvalid, awready;
    logic [31:0]     wdata;  logic wvalid, wready; logic [3:0] wstrb=4'hF;
    logic [1:0]      bresp;  logic bvalid; logic bready;
    logic [AWID-1:0] araddr; logic arvalid, arready;
    logic [31:0]     rdata;  logic [1:0] rresp; logic rvalid; logic rready;

    lzo_top #(.C_AXIL_AW(AWID)) DUT (
        .clk(clk),.resetn(resetn),
        .s_axil_awaddr(awaddr),.s_axil_awprot(3'b0),.s_axil_awvalid(awvalid),.s_axil_awready(awready),
        .s_axil_wdata(wdata),.s_axil_wstrb(wstrb),.s_axil_wvalid(wvalid),.s_axil_wready(wready),
        .s_axil_bresp(bresp),.s_axil_bvalid(bvalid),.s_axil_bready(bready),
        .s_axil_araddr(araddr),.s_axil_arprot(3'b0),.s_axil_arvalid(arvalid),.s_axil_arready(arready),
        .s_axil_rdata(rdata),.s_axil_rresp(rresp),.s_axil_rvalid(rvalid),.s_axil_rready(rready));

    localparam REG=16'h0000, BIN=16'h2000, BOUT=16'h4000;

    task automatic axil_write(input [AWID-1:0] a, input [31:0] d);
        begin
            @(posedge clk); awaddr<=a; wdata<=d; awvalid<=1; wvalid<=1;
            fork
                begin wait(awready); @(posedge clk); awvalid<=0; end
                begin wait(wready);  @(posedge clk); wvalid<=0;  end
            join
            bready<=1; wait(bvalid); @(posedge clk); bready<=0;
        end
    endtask
    task automatic axil_read(input [AWID-1:0] a, output [31:0] d);
        begin
            @(posedge clk); araddr<=a; arvalid<=1;
            wait(arready); @(posedge clk); arvalid<=0;
            rready<=1; wait(rvalid); d=rdata; @(posedge clk); rready<=0;
        end
    endtask

    logic [7:0] inbuf [0:8191];
    logic [7:0] cap   [0:8191];
    integer N, GC, k, rc, rc2, errors=0, fd, st, pass;
    logic [31:0] w;

    // start an op and wait via BUSY (rise then fall) - same as the PS app, and
    // robust against the latched done bit.
    task automatic run_op(input [31:0] ctrl);
        begin
            axil_write(REG+16'h00, ctrl);
            do axil_read(REG+16'h04, w); while (w[1]==1'b0);   // busy rises
            do axil_read(REG+16'h04, w); while (w[1]==1'b1);   // busy falls
        end
    endtask

    initial begin
        awvalid=0; wvalid=0; bready=0; arvalid=0; rready=0;
        fd=$fopen("blk_meta.txt","r"); st=$fscanf(fd,"%d %d",N,GC); $fclose(fd);
        for (k=0;k<8192;k++) inbuf[k]=0;
        $readmemh("blk_in.hex", inbuf);
        repeat(6) @(posedge clk); resetn<=1; repeat(4) @(posedge clk);

        // TWO passes reusing the SAME engines back-to-back (pass 1 uses modified
        // data) - this is what exposes the stale-done bug the single-block test missed.
        for (pass=0; pass<2; pass++) begin
            if (pass==1) for (k=0;k<N;k++) inbuf[k] = inbuf[k] ^ 8'h25;  // different data

            // ---- compress ----
            axil_write(REG+16'h08, N);                   // IN_LEN
            for (k=0;k<(N+3)/4;k++)
                axil_write(BIN + k*4, {inbuf[k*4+3],inbuf[k*4+2],inbuf[k*4+1],inbuf[k*4+0]});
            run_op(32'h1);                               // start compress
            axil_read(REG+16'h10, w); rc=w;
            $display("pass%0d compress: %0d -> %0d bytes (ratio %0d.%02dx)", pass, N, rc, (N*100/rc)/100,(N*100/rc)%100);
            for (k=0;k<rc;k++) begin axil_read(BOUT + (k/4)*4, w); cap[k]=w[(k%4)*8 +: 8]; end

            // ---- decompress ----
            axil_write(REG+16'h0C, N);                   // ORIG_LEN
            for (k=0;k<(rc+3)/4;k++)
                axil_write(BIN + k*4, {cap[k*4+3],cap[k*4+2],cap[k*4+1],cap[k*4+0]});
            run_op(32'h3);                               // start decompress
            axil_read(REG+16'h10, w); rc2=w;

            for (k=0;k<N;k++) begin
                axil_read(BOUT + (k/4)*4, w);
                if (w[(k%4)*8 +: 8] !== inbuf[k]) begin
                    errors++; if (errors<=8) $display("  pass%0d mismatch @%0d got %02x exp %02x", pass, k, w[(k%4)*8+:8], inbuf[k]);
                end
            end
            $display("pass%0d round-trip: %s (decomp %0d bytes)", pass, (errors==0)?"PASS":"FAIL", rc2);
        end
        $display("==== tb_top errors=%0d ====", errors);
        $finish;
    end
    initial begin #80000000; $display("TIMEOUT"); $finish; end
endmodule
