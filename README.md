# SPI_crc_wk2529
This Verilog project provides a robust SPI master-slave communication system with integrated CRC8 checksums for data integrity. The master initiates transactions, sending command, address, and data, while the slave validates incoming packets and responds. The design is corrected for reliability, making it a solid base for dependable SPI interfaces.

# SPI Master-Slave System with CRC-8

This repository contains a Verilog implementation of an SPI (Serial Peripheral Interface) master and slave, featuring integrated CRC-8 checksum generation and validation. The system is designed for robust data transfer, ensuring data integrity across SPI communication.

## Overview

This project provides a complete SPI communication system in Verilog. It includes a master module that initiates transactions, sends commands, addresses, and data, and calculates a CRC-8 checksum for the transmitted packet. It also receives data and validates its associated CRC. The slave module receives SPI packets, validates the incoming CRC, processes commands (e.g., writes to internal memory, reads ID), and sends back data along with a newly calculated CRC for its response. A top-level wrapper module connects the master and slave, providing an interface for a CPU or other control logic to initiate SPI transactions.

-----

## Module Descriptions

### `crc8`

This module implements a **parallel CRC-8 generator**.

  * **Description**: Calculates the CRC-8 checksum for an 8-bit input in a single clock cycle. It features purely combinational logic for CRC calculation, with a registered output.
  * **Parameters**:
      * `POLY`: The CRC polynomial to use. Default is `8'h07` (CRC-8).
  * **Inputs**:
      * `clk`: System clock.
      * `rst`: Asynchronous reset (active high).
      * `enable`: Enables the calculation and update of the CRC register.
      * `clear`: Synchronously clears the CRC output register.
      * `data_in [7:0]`: 8-bit input data for CRC calculation.
  * **Outputs**:
      * `crc_out [7:0]`: 8-bit registered CRC output.

### `spi_master`

This module acts as the **SPI Master** in the system.

  * **Description**: Initiates SPI transactions. It constructs a packet comprising a command, address, and data, calculates a CRC-8 for this outgoing packet, and transmits it serially. It also receives data from the slave and performs a CRC-8 validation on the received data.
  * **Inputs**:
      * `clk`: System clock.
      * `rst`: Asynchronous reset.
      * `en`: Starts an SPI transaction.
      * `miso`: Master In Slave Out (data from slave).
      * `ext_command_in [7:0]`: 8-bit command to send.
      * `ext_address_in [23:0]`: 24-bit address to send.
      * `ext_data_in [31:0]`: 32-bit data to send.
  * **Outputs**:
      * `cs`: Chip select (active low).
      * `sck`: SPI clock.
      * `mosi`: Master Out Slave In (data to slave).
      * `ext_data_out [31:0]`: 32-bit received data from the slave.
      * `spi_done`: Flag indicating the completion of an SPI transaction.
      * `crc_rx_ok`: Flag indicating if the received CRC from the slave was valid.

### `spi_slave`

This module functions as the **SPI Slave**.

  * **Description**: Responds to SPI transactions initiated by the master. It receives the incoming serial data, validates its CRC, processes the command (e.g., writes to an internal memory, reads a device ID), and if a read command, sends back the requested data along with a calculated CRC-8.
  * **Inputs**:
      * `sck`: SPI clock from master.
      * `rst`: Asynchronous reset.
      * `cs`: Chip select (active low) from master.
      * `mosi`: Master Out Slave In (data from master).
  * **Outputs**:
      * `miso`: Master In Slave Out (data to master).
      * `data_end`: Flag indicating the end of a data transfer from the slave's perspective.
      * `crc_match`: Flag indicating if the received CRC from the master matched.
      * `crc_error`: Flag indicating if the received CRC from the master did not match.

### `spi`

This is the **Top-level Wrapper** module.

  * **Description**: Connects the `spi_master` and `spi_slave` modules. It provides an abstraction layer for a CPU or higher-level logic to interact with the SPI system, allowing for sending commands and data, and receiving responses. It also includes simple registers for interfacing with the master.
  * **Inputs**:
      * `clk`: System clock.
      * `rst`: Asynchronous reset.
      * `addr [2:0]`: (Removed unused, previously caused Verilator warning)
      * `we`: Write enable for internal registers.
      * `re`: Read enable for internal registers.
      * `spi_start`: Pulses high to initiate an SPI transaction.
      * `spi_cmd [7:0]`: Command to be sent by the master.
      * `spi_addr [23:0]`: Address to be sent by the master.
      * `spi_data [31:0]`: Data to be written by the master.
  * **Outputs**:
      * `read_data [31:0]`: Latched data read from the master's output.
      * `spi_done`: Master's transaction done flag.
      * `data_end`: Slave's data transfer complete flag.
      * `spi_resp [31:0]`: Direct output of received data from the master.
      * `crc_ok`: Master's flag indicating valid received CRC.
      * `crc_error`: Slave's flag indicating invalid received CRC.

-----

## Features

  * **Parallel CRC-8**: Fast CRC calculation in a single clock cycle.
  * **Full Duplex SPI**: Simultaneous data transmission and reception.
  * **CRC on Tx and Rx**: Ensures data integrity for both master-to-slave and slave-to-master communication.
  * **Modular Design**: Clear separation of CRC, master, and slave functionalities.
  * **Asynchronous Reset**: Robust reset mechanism.
  * **Synchronous Control**: All operations are synchronized to the clock edge.
  * **Write/Read Memory Emulation**: The slave includes a basic internal memory for demonstration of write and read commands (`8'h00` for write, `8'hFF` for read).
  * **Device ID Read**: Slave can respond with a fixed ID (`8'h5A` command returns `32'h53464450`).

-----

## Usage

This Verilog code can be simulated using any Verilog-2001 compliant simulator (e.g., Icarus Verilog, ModelSim, VCS). For synthesis, target FPGA or ASIC tools.

To use the system:

1.  **Instantiate the `spi` module** in your top-level design.
2.  **Provide clock and reset signals.**
3.  **To initiate a transaction**:
      * Set `spi_cmd`, `spi_addr`, and `spi_data` to the desired values.
      * Pulse the `spi_start` signal high for one clock cycle.
4.  **Monitor `spi_done`** to know when the transaction is complete.
5.  **Check `crc_ok`** from the master to verify the integrity of the data received from the slave.
6.  **Read `spi_resp` or `read_data`** to get the data returned by the slave.
7.  **Monitor `crc_error`** from the slave to detect if the master's transmitted data had a CRC error.

-----

## CRC-8 Polynomial

The CRC-8 polynomial used in this implementation is defined as `POLY = 8'h07`. This corresponds to the polynomial $x^8 + x^2 + x^1 + 1$.
