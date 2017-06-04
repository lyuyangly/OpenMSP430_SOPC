/*
************************************************************************************************
*	File   : OpenMSP430_FPGA.v
*	Module : 
*	Author : Lyu Yang
*	Data   : 10,11,2014
*	Description :
************************************************************************************************
*/
`timescale 1ns / 100ps

`include "./omsp430/openMSP430_defines.v"

module OpenMSP430_FPGA (
	input				clk,
	input				rst_pin,
	input	[7:0]		p1_in,
	output	[7:0]		p1_out,
	output	[3:0]		seg_sel,
	output	[7:0]		seg_hex,
	input				uart_rxd_in,
	output				uart_txd_out
);

//=============================================================================
// 1)  INTERNAL WIRES/REGISTERS/PARAMETERS DECLARATION
//=============================================================================

// openMSP430 output buses
wire        [13:0]	per_addr;
wire        [15:0]	per_din;
wire         [1:0]	per_we;
wire [`DMEM_MSB:0]	dmem_addr;
wire        [15:0]	dmem_din;
wire         [1:0]	dmem_wen;
wire [`PMEM_MSB:0]	pmem_addr;
wire        [15:0]	pmem_din;
wire         [1:0]	pmem_wen;
wire        [13:0]	irq_acc;

// openMSP430 input buses
wire        [13:0]	irq_bus;
wire        [15:0]	per_dout;
wire        [15:0]	dmem_dout;
wire        [15:0]	pmem_dout;
wire				mclk;
wire				puc_rst;
// GPIO
wire         [7:0] p1_din;
wire         [7:0] p1_dout;
wire         [7:0] p1_dout_en;
wire         [7:0] p1_sel;
wire         [7:0] p2_din;
wire         [7:0] p2_dout;
wire         [7:0] p2_dout_en;
wire         [7:0] p2_sel;
wire         [7:0] p3_din;
wire         [7:0] p3_dout;
wire         [7:0] p3_dout_en;
wire         [7:0] p3_sel;
wire        [15:0] per_dout_dio;
wire        [15:0] per_dout_seg;


//=============================================================================
// 4)  OPENMSP430
//=============================================================================

openMSP430 openMSP430_inst (
    .reset_n      (rst_n),      // Reset Pin (low active, asynchronous and non-glitchy)
    .aclk         (),             // ASIC ONLY: ACLK
    .aclk_en      (aclk_en),      // FPGA ONLY: ACLK enable
    .lfxt_enable  (),             // ASIC ONLY: Low frequency oscillator enable
    .lfxt_wkup    (),             // ASIC ONLY: Low frequency oscillator wake-up (asynchronous)
    .lfxt_clk     (1'b0),         // Low frequency oscillator (typ 32kHz)
    .nmi          (1'b0),          // Non-maskable interrupt (asynchronous)
    .cpu_en       (1'b1),         // Enable CPU code execution (asynchronous and non-glitchy)
    .dbg_en       (1'b1),         // Debug interface enable (asynchronous and non-glitchy)
    .dbg_uart_rxd (uart_rxd_in), // Debug interface: UART RXD (asynchronous)
    .dbg_uart_txd (uart_rxd_out), // Debug interface: UART TXD
    .dbg_freeze   (dbg_freeze),   // Freeze peripherals
    .scan_enable  (1'b0),         // ASIC ONLY: Scan enable (active during scan shifting)
    .scan_mode    (1'b0),         // ASIC ONLY: Scan mode
    .wkup         (1'b0),         // ASIC ONLY: System Wake-up (asynchronous and non-glitchy)
    .dco_enable   (),             // ASIC ONLY: Fast oscillator enable
    .dco_wkup     (),             // ASIC ONLY: Fast oscillator wake-up (asynchronous)
    .dco_clk      (clk),      	  // Fast oscillator (fast clock)
    .smclk        (),             // ASIC ONLY: SMCLK
    .smclk_en     (smclk_en),     // FPGA ONLY: SMCLK enable
    .mclk         (mclk),         // Main system clock
    .puc_rst      (puc_rst),      // Main system reset
	
	// Peripherals
    .per_en       (per_en),       // Peripheral enable (high active)
    .per_we       (per_we),       // Peripheral write enable (high active)
    .per_addr     (per_addr),     // Peripheral address
    .per_din      (per_din),      // Peripheral data input
    .per_dout     (per_dout),     // Peripheral data output
	
	// Program Memory
    .pmem_cen     (pmem_cen),     // Program Memory chip enable (low active)
    .pmem_din     (pmem_din),     // Program Memory data input (optional)
    .pmem_wen     (pmem_wen),     // Program Memory write enable (low active) (optional)
    .pmem_addr    (pmem_addr),    // Program Memory address
    .pmem_dout    (pmem_dout),    // Program Memory data output
	
	// Data Memory
    .dmem_cen     (dmem_cen),     // Data Memory chip enable (low active)
    .dmem_wen     (dmem_wen),     // Data Memory write enable (low active)	
    .dmem_addr    (dmem_addr),    // Data Memory address
    .dmem_din     (dmem_din),     // Data Memory data input
    .dmem_dout    (dmem_dout),    // Data Memory data output
	
	// Maskable interrupts
    .irq          (irq_bus),      // Maskable interrupts
    .irq_acc      (irq_acc)      // Interrupt request accepted (one-hot signal)	
);

// RAM
`ifdef ALTERA
// reset pin
assign rst_n = rst_pin;

//cyclone's M4k cells - just an example of instantiating 16-bit M4K altera RAM
ram16x512 ram_inst (
        .address 		(dmem_addr),
        .clken   		(~dmem_cen),
        .clock   		(mclk),
        .data    		(dmem_din),
        .q       		(dmem_dout),
        .wren    		( ~(&dmem_wen[1:0]) ),
        .byteena 		( ~dmem_wen[1:0] )
);

// ROM - DEBUG ACCESS removed. If you need it, use as example original diligent's sources
rom16x2048 rom_inst (
        .clock  		(mclk),
        .clken  		(~pmem_cen),
        .address        (pmem_addr),
        .q              ( pmem_dout )
);
`else 
// reset pin
assign rst_n = ~rst_pin;

// RAM
ram16x512 ram_inst (
		.clka			(mclk),
		.ena			(~dmem_cen),
		.wea			(~dmem_wen),
		.addra			(dmem_addr),
		.dina			(dmem_din),
		.douta			(dmem_dout)
);

// ROM
rom16x2048 rom_inst (
		.clka			(mclk),
		.ena			(~pmem_cen),
		.addra			(pmem_addr),
		.douta			(pmem_dout)
);
`endif

//
// Digital I/O
//-------------------------------

omsp_gpio #(.P1_EN(1),
            .P2_EN(1),
            .P3_EN(1),
            .P4_EN(0),
            .P5_EN(0),
            .P6_EN(0)) gpio_inst (

// OUTPUTs
    .irq_port1    (irq_port1),     // Port 1 interrupt
    .irq_port2    (irq_port2),     // Port 2 interrupt
    .p1_dout      (p1_dout),       // Port 1 data output
    .p1_dout_en   (p1_dout_en),    // Port 1 data output enable
    .p1_sel       (p1_sel),        // Port 1 function select
    .p2_dout      (p2_dout),       // Port 2 data output
    .p2_dout_en   (p2_dout_en),    // Port 2 data output enable
    .p2_sel       (p2_sel),        // Port 2 function select
    .p3_dout      (p3_dout),       // Port 3 data output
    .p3_dout_en   (p3_dout_en),    // Port 3 data output enable
    .p3_sel       (p3_sel),        // Port 3 function select
    .p4_dout      (),              // Port 4 data output
    .p4_dout_en   (),              // Port 4 data output enable
    .p4_sel       (),              // Port 4 function select
    .p5_dout      (),              // Port 5 data output
    .p5_dout_en   (),              // Port 5 data output enable
    .p5_sel       (),              // Port 5 function select
    .p6_dout      (),              // Port 6 data output
    .p6_dout_en   (),              // Port 6 data output enable
    .p6_sel       (),              // Port 6 function select
    .per_dout     (per_dout_dio),  // Peripheral data output

// INPUTs
    .mclk         (mclk),          // Main system clock
    .p1_din       (p1_din),        // Port 1 data input
    .p2_din       (p2_din),        // Port 2 data input
    .p3_din       (p3_din),        // Port 3 data input
    .p4_din       (8'h00),         // Port 4 data input
    .p5_din       (8'h00),         // Port 5 data input
    .p6_din       (8'h00),         // Port 6 data input
    .per_addr     (per_addr),      // Peripheral address
    .per_din      (per_din),       // Peripheral data input
    .per_en       (per_en),        // Peripheral enable (high active)
    .per_we       (per_we),        // Peripheral write enable (high active)
    .puc_rst      (puc_rst)        // Main system reset
);

//
// Combine peripheral data buses
//-------------------------------

assign per_dout = per_dout_dio|per_dout_seg;



seg7 seg7_inst (
    .mclk		(mclk),
    .puc_rst	(puc_rst),
    .per_addr	(per_addr),
    .per_din	(per_din),
    .per_en		(per_en),
    .per_we		(per_we),
    .per_dout	(per_dout_seg),
	.seg_sel	(seg_sel),
	.seg_hex	(seg_hex)
);

//
// Assign interrupts
//-------------------------------

assign irq_bus    = {1'b0,         // Vector 13  (0xFFFA)
                     1'b0,         // Vector 12  (0xFFF8)
                     1'b0,         // Vector 11  (0xFFF6)
                     1'b0,         // Vector 10  (0xFFF4) - Watchdog -
                     1'b0,      // Vector  9  (0xFFF2)
                     1'b0,      // Vector  8  (0xFFF0)
                     1'b0,         // Vector  7  (0xFFEE)
                     1'b0,         // Vector  6  (0xFFEC)
                     1'b0,         // Vector  5  (0xFFEA)
                     1'b0,         // Vector  4  (0xFFE8)
                     irq_port2,    // Vector  3  (0xFFE6)
                     irq_port1,    // Vector  2  (0xFFE4)
                     1'b0,         // Vector  1  (0xFFE2)
                     1'b0};        // Vector  0  (0xFFE0)


assign p3_din[7:0] = p1_in;

assign p1_out = p3_dout[7:0] & p3_dout_en[7:0];

endmodule
