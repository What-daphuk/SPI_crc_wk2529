// spi_stimulus.v
`timescale 1ns / 1ps
module spi_stimulus (
    output reg   clk,
    output reg   rst,
    output reg   we,
    output reg   re,
    output reg   spi_start,
    output reg [7:0]  spi_cmd,
    output reg [23:0] spi_addr,
    output reg [31:0] spi_data,
    output reg [2:0]  addr,
    input        data_end,
    input        spi_done,
    input        crc_ok,
    input [31:0] spi_resp
);

    task spi_transaction;
        input [7:0] cmd;
        input [23:0] address;
        input [31:0] data;
        begin
            @(posedge clk);
            spi_cmd   = cmd;
            spi_addr  = address;
            spi_data  = data;
            we        = 1;
            spi_start = 1;
            @(posedge clk);
            we        = 0;
            spi_start = 0;
#200;
            wait(data_end);
            @(posedge clk);

            $display("  SPI_CMD: 0x%02h", cmd);
            $display("  Sent Data: 0x%08h", data);
            $display("  Received Loopback: 0x%08h", spi_resp);
            $display("  CRC OK: %b", crc_ok);
            if (crc_ok)
                $display("  [PASS] CRC OK");
            else
                $display("  [FAIL] CRC Failed");
        end
    endtask
//clk = 0;
initial begin
    clk = 0;
    forever #5 clk = ~clk;
end
    initial begin
        // Initialize
        rst = 1;
        we = 0;
        re = 0;
        spi_start = 0;
        spi_cmd = 0;
        spi_addr = 0;
        spi_data = 0;
        addr = 0;
        #20;
        rst = 0;

        // Test Cases
        $display("\n--- Stimulus: Standard Payload ---");
        spi_transaction(8'h00, 24'h123456, 32'hF0F0A5A5);

#200;
        $display("\n--- Stimulus: CRC Error Test ---");
        spi_transaction(8'hFF, 24'hABCDEF, 32'hDEADBEEF);

        #400;
        $finish;
    end

endmodule