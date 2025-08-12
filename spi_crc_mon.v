// spi_monitor.v
module spi_monitor (
    input clk,
    input rst,
    input we,
    input re,
    input spi_start,
    input [7:0] spi_cmd,
    input [23:0] spi_addr,
    input [31:0] spi_data,
    input [31:0] spi_resp,
    input spi_done,
    input data_end,
    input crc_ok
);
    always @(posedge clk) begin
        $display("[%0t ns] CLK=%b RST=%b WE=%b RE=%b SPI_START=%b CMD=0x%02h ADDR=0x%06h DATA=0x%08h RESP=0x%08h DONE=%b CRC=%b",
            $time, clk, rst, we, re, spi_start, spi_cmd, spi_addr, spi_data, spi_resp, spi_done, crc_ok);
    end
endmodule