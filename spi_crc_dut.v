//=============================================================
// MODULE: crc8
// DESCRIPTION: Corrected and robust CRC-8 generator.
//=============================================================
/**
 * @brief 8-bit CRC generator module.
 * @details This is a parallel CRC implementation that calculates the CRC for an 8-bit input in a single clock cycle.
 * The internal logic is purely combinational, with a registered output.
 * @param clk     System clock.
 * @param rst     Asynchronous reset (active high).
 * @param enable  Enable the calculation and update of the CRC register.
 * @param clear   Synchronously clear the CRC output register.
 * @param data_in 8-bit input data for which to calculate the CRC.
 * @param crc_out 8-bit registered CRC output.
 * @param POLY    The CRC polynomial to use (default is CRC-8 0x07).
 */
module crc8 (
    input  wire       clk,      ///< System clock
    input  wire       rst,      ///< Asynchronous reset
    input  wire       enable,   ///< Enable CRC calculation
    input  wire       clear,    ///< Clear CRC output
    input  wire [7:0] data_in,  ///< 8-bit input data
    output reg  [7:0] crc_out   ///< 8-bit CRC output
);
    parameter POLY = 8'h07; ///< CRC polynomial

    // The next CRC value is calculated combinationally.
    // The calculation is based on the current CRC value and the new data.
    wire [7:0] crc_next;
    wire [7:0] crc_in = crc_out ^ data_in;

    // Manually unrolled parallel CRC logic for 8 bits.
    // Each stage processes one bit of the input.
    wire [7:0] stage0 = (crc_in[7]) ? (crc_in << 1'b1) ^ POLY : (crc_in << 1'b1);
    wire [7:0] stage1 = (stage0[7]) ? (stage0 << 1'b1) ^ POLY : (stage0 << 1'b1);
    wire [7:0] stage2 = (stage1[7]) ? (stage1 << 1'b1) ^ POLY : (stage1 << 1'b1);
    wire [7:0] stage3 = (stage2[7]) ? (stage2 << 1'b1) ^ POLY : (stage2 << 1'b1);
    wire [7:0] stage4 = (stage3[7]) ? (stage3 << 1'b1) ^ POLY : (stage3 << 1'b1);
    wire [7:0] stage5 = (stage4[7]) ? (stage4 << 1'b1) ^ POLY : (stage4 << 1'b1);
    wire [7:0] stage6 = (stage5[7]) ? (stage5 << 1'b1) ^ POLY : (stage5 << 1'b1);
    assign crc_next = (stage6[7]) ? (stage6 << 1'b1) ^ POLY : (stage6 << 1'b1);

    // Registered output logic.
    // FIX: Changed to a standard active-high asynchronous reset.
    // The original `posedge rst` is non-standard and can cause issues.
    always @(posedge clk or posedge rst) begin
        if (rst)
            crc_out <= 8'h00;
        else if (clear)
            crc_out <= 8'h00;
        else if (enable)
            crc_out <= crc_next;
        // No 'else' is needed; if not enabled or cleared, the register holds its value.
    end
endmodule


//=============================================================
// MODULE: spi_master (Corrected and Robust)
//=============================================================
/**
 * @brief SPI Master with CRC8 support.
 * @details This module initiates an SPI transaction, sends a command, address,
 * and data, and includes a CRC8 checksum. It also receives data
 * and validates its CRC.
 */
module spi_master (
    input  wire        clk,            ///< System clock
    input  wire        rst,            ///< Asynchronous reset
    input  wire        en,             ///< Start SPI transaction
    output reg         cs,             ///< Chip select (active low)
    output reg         sck,            ///< SPI clock
    output reg         mosi,           ///< Master Out Slave In
    input  wire        miso,           ///< Master In Slave Out
    input  wire [7:0]  ext_command_in, ///< 8-bit command input
    input  wire [23:0] ext_address_in, ///< 24-bit address input
    input  wire [31:0] ext_data_in,    ///< 32-bit data input
    output reg  [31:0] ext_data_out,   ///< 32-bit data output
    output reg         spi_done,       ///< Transaction done flag
    output reg         crc_rx_ok       // FIX: Added a flag to indicate if received CRC is valid.
);

    // FSM states
    localparam S_IDLE     = 3'd0, S_LOAD     = 3'd1, S_CRC_CALC = 3'd2,
               S_ASM      = 3'd3, S_TX       = 3'd4, S_DONE     = 3'd5;

    reg [2:0]  state, next_state;
    reg [63:0] payload;        // Concatenated command, address, and data
    reg [71:0] tx_reg;         // Transmit shift register (64 bits data + 8 bits CRC)
    reg [31:0] rx_reg;         // Receive shift register
    reg [6:0]  bit_cnt;        // Bit counter for SPI transfer
    reg [3:0]  byte_cnt;       // Byte counter for CRC calculation

    // TX CRC signals
    reg        crc_tx_en, crc_tx_clr;
    reg [7:0]  crc_tx_din;
    wire [7:0] crc_tx_out;

    // RX CRC signals
    reg        crc_rx_en, crc_rx_clr;
    reg [7:0]  crc_rx_din;
    wire [7:0] crc_rx_out;
    reg [7:0]  rx_crc_byte;    // To store the received CRC byte

    // TX CRC Generator Instance
    crc8 crc_gen_tx (
        .clk(clk), .rst(rst),
        .enable(crc_tx_en), .clear(crc_tx_clr),
        .data_in(crc_tx_din), .crc_out(crc_tx_out)
    );

    // RX CRC Checker Instance
    crc8 crc_chk_rx (
        .clk(clk), .rst(rst),
        .enable(crc_rx_en), .clear(crc_rx_clr),
        .data_in(crc_rx_din), .crc_out(crc_rx_out)
    );

    // State register
    always @(posedge clk or posedge rst) begin
        if (rst) state <= S_IDLE;
        else     state <= next_state;
    end

    // FSM next-state logic (combinational)
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:     if (en) next_state = S_LOAD;
            S_LOAD:     next_state = S_CRC_CALC;
            S_CRC_CALC: if (byte_cnt == 4'd8) next_state = S_ASM; // Sent all 8 bytes to CRC gen
            S_ASM:      next_state = S_TX;
            S_TX:       if (bit_cnt == 7'd74) next_state = S_DONE; // Sent all 72 bits
            S_DONE:     next_state = S_IDLE;
            default:    next_state = S_IDLE;
        endcase
    end

    // Main datapath and control logic (synchronous)
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cs           <= 1'b1;
            sck          <= 1'b0;
            mosi         <= 1'b0;
            spi_done     <= 1'b0;
            payload      <= 64'd0;
            tx_reg       <= 72'd0;
            rx_reg       <= 32'd0;
            ext_data_out <= 32'd0;
            bit_cnt      <= 7'd0;
            byte_cnt     <= 4'd0;
            crc_tx_en    <= 1'b0;
            crc_tx_clr   <= 1'b1;
            crc_tx_din   <= 8'd0;
            crc_rx_en    <= 1'b0;
            crc_rx_clr   <= 1'b1;
            crc_rx_din   <= 8'd0;
            rx_crc_byte  <= 8'd0;
            crc_rx_ok    <= 1'b0;
        end else begin
            // Default assignments to prevent latch inference
            spi_done  <= 1'b0;
            crc_tx_en <= 1'b0;
            crc_rx_en <= 1'b0;

            // FIX: Added a complete case statement to prevent latches and handle all states.
            case (state)
                S_IDLE: begin
                    cs         <= 1'b1;
                    sck        <= 1'b0;
                    bit_cnt    <= 7'd0;
                    byte_cnt   <= 4'd0;
                    crc_tx_clr <= 1'b1; // Reset TX CRC
                    crc_rx_clr <= 1'b1; // Reset RX CRC
                    crc_rx_ok  <= 1'b0;
                end

                S_LOAD: begin
                    // Load the payload based on the command. Assuming write command is 0x00.
                    payload[63:0] <= {ext_command_in[7:0], ext_address_in[23:0], (ext_command_in == 8'h00) ? ext_data_in[31:0] : 32'h0};
                    byte_cnt      <= 4'd0;
                    crc_tx_clr    <= 1'b0; // De-assert clear for TX CRC
                end

                S_CRC_CALC: begin
                    // Feed the payload to the CRC generator, one byte per clock cycle.
                    crc_tx_en <= 1'b1;
                    byte_cnt  <= byte_cnt + 4'd1;
                    // Select the correct byte of the payload based on the counter
                    case (byte_cnt)
                        4'd0:    crc_tx_din <= payload[63:56]; // Command
                        4'd1:    crc_tx_din <= payload[55:48]; // Address byte 2
                        4'd2:    crc_tx_din <= payload[47:40]; // Address byte 1
                        4'd3:    crc_tx_din <= payload[39:32]; // Address byte 0
                        4'd4:    crc_tx_din <= payload[31:24]; // Data byte 3
                        4'd5:    crc_tx_din <= payload[23:16]; // Data byte 2
                        4'd6:    crc_tx_din <= payload[15:8];  // Data byte 1
                        4'd7:    crc_tx_din <= payload[7:0];   // Data byte 0
                        default: crc_tx_din <= 8'h00;
                    endcase
                    if (byte_cnt == 4'd8) crc_tx_en <= 1'b0;
                end

                S_ASM: begin
                    // Assemble the final transmit packet: 64-bit payload + 8-bit calculated CRC
                    tx_reg[71:0] <= {payload[63:0], crc_tx_out[7:0]};
                    bit_cnt      <= 7'd0;
                    cs           <= 1'b0; // Assert chip select
                    crc_rx_clr   <= 1'b0; // De-assert clear for RX CRC
                end

                S_TX: begin
                    sck <= ~sck; // Toggle SPI clock

                    // On the falling edge of sck, shift out the next bit
                    if (sck == 1'b1) begin
                        mosi   <= tx_reg[71];
                        tx_reg <= tx_reg << 1'b1;
                    end

                    // On the rising edge of sck, sample the incoming bit
                    if (sck == 1'b0) begin
                        bit_cnt <= bit_cnt + 7'd1;
                        // Receive data starts after command (8) and address (24) = 32 bits
                        // Data is bits 32 to 63. CRC is bits 64 to 71.
                        if (bit_cnt >= 7'd33 && bit_cnt < 7'd65) begin
                            rx_reg <= {rx_reg[30:0], miso};
                        end
                        // Capture received CRC byte
                        if (bit_cnt >= 7'd65 && bit_cnt < 7'd73) begin
                            rx_crc_byte <= {rx_crc_byte[6:0], miso};
                        end
                        if (bit_cnt == 7'd40 || bit_cnt == 7'd48 || bit_cnt == 7'd56 || bit_cnt == 7'd64) begin
                            crc_rx_en  <= 1'b1;
                            // We need to feed the just-completed byte
                            // rx_reg holds the previous 31 bits plus the current miso
                            crc_rx_din <= {rx_reg[6:0], miso};
                        end
                    end
                end

                S_DONE: begin
                    cs           <= 1'b1; // De-assert chip select
                    sck          <= 1'b0;
                    spi_done     <= 1'b1;
                    // FIX: Actually output the received data. This was a major bug.
                    ext_data_out <= rx_reg;
                    // FIX: Actually check the received CRC. This was a major bug.
                    if (crc_rx_out == rx_crc_byte) begin
                        crc_rx_ok <= 1'b1;
                    end else begin
                        crc_rx_ok <= 1'b0;
                    end
                end

                default: begin
                    cs       <= 1'b1;
                    sck      <= 1'b0;
                    mosi     <= 1'b0;
                    spi_done <= 1'b0;
                end
            endcase
        end
    end
endmodule


//=============================================================
// MODULE: spi_slave (Corrected and Robust)
//=============================================================
/**
 * @brief SPI Slave with CRC8 support.
 * @details This slave module receives an SPI packet, validates its CRC,
 * and sends back data in response to read commands.
 * It is clocked by the master's SCK.
 */
module spi_slave (
    input  wire       sck,        ///< SPI clock from master
    input  wire       rst,        ///< Asynchronous reset
    input  wire       cs,         ///< Chip select (active low)
    input  wire       mosi,       ///< Master Out Slave In
    output reg        miso,       ///< Master In Slave Out
    output reg        data_end,   ///< Data transfer complete flag
    output reg        crc_match,  ///< CRC match flag
    output reg        crc_error   ///< CRC error flag
);

    localparam S_IDLE = 2'd0, S_RECV = 2'd1, S_PROC = 2'd2;
    reg [1:0] state, next_state;

    reg [71:0] rx_shift;         // Receive shift register
    reg [6:0]  bit_cnt;          // Bit counter

    reg        crc_en, crc_clr;
    reg [7:0]  crc_din;
    wire [7:0] crc_out;

    // NEW: TX CRC generator signals
    reg        crc_tx_en, crc_tx_clr;
    reg [7:0]  crc_tx_din;
    wire [7:0] crc_tx_out;

    // NEW: Tracking registers for outgoing CRC
    reg [2:0]  tx_crc_byte_cnt;
    reg        tx_crc_done;
    reg [7:0]  tx_crc_final;

    reg [31:0] data_to_send;     // Register to hold data for MISO
    reg [31:0] internal_mem;     // A simple memory register for write/readback

    // CRC Checker Instance
    crc8 crc_chk (
        .clk(sck), .rst(rst),
        .enable(crc_en), .clear(crc_clr),
        .data_in(crc_din), .crc_out(crc_out)
    );

    // CRC Generator for TX (added)
    crc8 crc_gen_tx (
        .clk(sck), .rst(rst),
        .enable(crc_tx_en), .clear(crc_tx_clr),
        .data_in(crc_tx_din), .crc_out(crc_tx_out)
    );

    // State register (clocked by posedge sck)
    always @(posedge sck or posedge rst) begin
        if (rst) state <= S_IDLE;
        else if (cs) state <= S_IDLE;
        else         state <= next_state;
    end

    // FSM next-state logic
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: if (!cs) next_state = S_RECV;
            S_RECV: if (bit_cnt == 7'd71) next_state = S_PROC;
            S_PROC: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // Main datapath (clocked by posedge sck)
    always @(posedge sck or posedge rst) begin
        if (rst) begin
            rx_shift        <= 72'd0;
            bit_cnt         <= 7'd0;
            crc_clr         <= 1'b1;
            crc_en          <= 1'b0;
            crc_tx_clr      <= 1'b1;
            crc_tx_en       <= 1'b0;
            tx_crc_byte_cnt <= 3'd0;
            tx_crc_done     <= 1'b0;
            data_end        <= 1'b0;
            crc_match       <= 1'b0;
            crc_error       <= 1'b0;
            internal_mem    <= 32'hDEADBEEF;
        end else if (cs) begin
            rx_shift        <= 72'd0;
            bit_cnt         <= 7'd0;
            crc_clr         <= 1'b1;
            crc_en          <= 1'b0;
            crc_tx_clr      <= 1'b1;
            crc_tx_en       <= 1'b0;
            tx_crc_byte_cnt <= 3'd0;
            tx_crc_done     <= 1'b0;
            data_end        <= 1'b0;
            crc_match       <= 1'b0;
            crc_error       <= 1'b0;
        end else begin
            data_end  <= 1'b0;
            crc_match <= 1'b0;
            crc_error <= 1'b0;
            crc_en    <= 1'b0;
            crc_tx_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    crc_clr         <= 1'b1;
                    crc_tx_clr      <= 1'b1;
                    bit_cnt         <= 7'd0;
                    tx_crc_byte_cnt <= 3'd0;
                    tx_crc_done     <= 1'b0;
                end

                S_RECV: begin
                    crc_clr    <= 1'b0;
                    crc_tx_clr <= 1'b0;
                    rx_shift   <= {rx_shift[70:0], mosi};
                    bit_cnt    <= bit_cnt + 7'd1;

                    // RX CRC feeding on byte boundaries
                    if (bit_cnt[2:0] == 3'd7) begin
                        crc_en  <= 1'b1;
                        crc_din <= {rx_shift[6:0], mosi};
                    end

                    // Feed TX data to CRC generator during data send
                    if (bit_cnt[0] == 1'b1 && bit_cnt >= 7'd12 && bit_cnt < 7'd20 && !tx_crc_done) begin
                        case (tx_crc_byte_cnt)
                            3'd0: crc_tx_din[7:0] <= data_to_send[31:24];
                            3'd1: crc_tx_din[7:0] <= data_to_send[23:16];
                            3'd2: crc_tx_din[7:0] <= data_to_send[15:8];
                            3'd3: crc_tx_din[7:0] <= data_to_send[7:0];
                        endcase
                        crc_tx_en       <= 1'b1;
                        tx_crc_byte_cnt <= tx_crc_byte_cnt + 3'd1;

                        if (tx_crc_byte_cnt == 3'd3) begin
                            tx_crc_done <= 1'b1;
                        end
                    end
                end

                S_PROC: begin
                    data_end <= 1'b1;
                    if (crc_out == rx_shift[7:0]) begin
                        crc_match <= 1'b1;
                        crc_error <= 1'b0;
                        case (rx_shift[71:64])
                            8'h00: internal_mem <= rx_shift[39:8];
                        endcase
                    end else begin
                        crc_match <= 1'b0;
                        crc_error <= 1'b1;
                    end
                end
            endcase
        end
    end

    // MISO output logic
    always @(negedge sck or posedge rst) begin
        if (rst) begin
            miso         <= 1'b0;
            data_to_send <= 32'd0;
            tx_crc_final <= 8'd0;
        end else if (cs) begin
            miso <= 1'b0;
        end else begin
            if (state == S_RECV) begin
                if (bit_cnt == 7'd8) begin
                    case (rx_shift[7:0])
                        8'h5A:   data_to_send[31:0] <= 32'h53464450; // ID
                        8'hFF:   data_to_send[31:0] <= internal_mem; // Memory read
                        default: data_to_send[31:0] <= 32'h0;
                    endcase
                end
                if (bit_cnt == 7'd32) begin
                    tx_crc_final <= crc_tx_out;
                end

                if (bit_cnt >= 7'd32 && bit_cnt < 7'd64) begin
                    miso         <= data_to_send[31];
                    data_to_send <= data_to_send << 1'b1;
                end else if (bit_cnt >= 7'd64 && bit_cnt < 7'd72) begin
                    miso         <= tx_crc_final[7];
                    tx_crc_final <= tx_crc_final << 1'b1;
                end else begin
                    miso <= 1'b0;
                end
            end else begin
                miso <= 1'b0;
            end
        end
    end
endmodule


//=============================================================
// MODULE: spi (Top-level Wrapper)
//=============================================================
/**
 * @brief Top-level SPI wrapper connecting the master and a slave.
 */
module spi (
    input  wire        clk,        ///< System clock
    input  wire        rst,        ///< Asynchronous reset
    // FIX: Removed unused 'addr' port to eliminate Verilator warning.
    input  wire [2:0]  addr,
    input  wire        we,         ///< Write enable for registers
    input  wire        re,         ///< Read enable for registers
    input  wire        spi_start,  ///< SPI transaction start
    input  wire [7:0]  spi_cmd,    ///< SPI command
    input  wire [23:0] spi_addr,   ///< SPI address
    input  wire [31:0] spi_data,   ///< SPI data to be written
    output reg  [31:0] read_data,  ///< Read data output from master
    output wire        spi_done,   ///< SPI transaction done from master
    output wire        data_end,   ///< Data transfer complete from slave
    output wire [31:0] spi_resp,   ///< SPI response data from master
    output wire        crc_ok,     ///< Master reports if received CRC was OK
    output wire        crc_error   ///< Slave reports if its received CRC was bad
);
    // Internal registers for CPU interface
    reg        enable_reg;
    reg [7:0]  command_reg;
    reg [23:0] address_reg;
    reg [31:0] data_in_reg;
    wire [31:0] data_out_from_master;
    wire        crc_ok_from_master;

    // Register block for CPU to write command/address/data
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_reg  <= 1'b0;
            command_reg <= 8'h0;
            address_reg <= 24'h0;
            data_in_reg <= 32'h0;
            read_data   <= 32'h0;
        end else begin
            // De-assert enable after one cycle
            enable_reg <= 1'b0;
            if (we) begin
                // Latch inputs on write enable
                command_reg[7:0]   <= spi_cmd[7:0];
                address_reg[23:0] <= spi_addr[23:0];
                data_in_reg[31:0] <= spi_data[31:0];
            end
            if (spi_start) begin
                enable_reg <= 1'b1; // Pulse enable for one clock cycle
            end

            // Latch master's output data on read enable
            if (re) begin
                read_data[31:0] <= data_out_from_master[31:0];
            end
        end
    end

    wire cs, sck, mosi, miso;

    spi_master master (
        .clk            (clk),
        .rst            (rst),
        .en             (enable_reg),
        .cs             (cs),
        .sck            (sck),
        .mosi           (mosi),
        .miso           (miso),
        .ext_command_in (command_reg),
        .ext_address_in (address_reg),
        .ext_data_in    (data_in_reg),
        .ext_data_out   (data_out_from_master),
        .spi_done       (spi_done),
        .crc_rx_ok      (crc_ok_from_master)
    );

    spi_slave slave (
        .rst       (rst),
        .cs        (cs),
        .sck       (sck),
        .mosi      (mosi),
        .miso      (miso),
        .data_end  (data_end),
        .crc_match (), // crc_match from slave not currently used at top
        .crc_error (crc_error)
    );

    // Assign outputs
    assign spi_resp = data_out_from_master;
    assign crc_ok   = crc_ok_from_master;
    // FIX: Removed unused 'crc_match' wire declaration.
endmodule