/*
************************************************************************************************
*	File   : *.v
*	Module : 
*	Author : Lyu Yang
*	Date   : 2014
*	Description :
************************************************************************************************
*/
`timescale 1ns / 100ps
module  seg7 (
    mclk,                           // Main system clock
    puc_rst,                         // Main system reset
    per_addr,                       // Peripheral address
    per_din,                        // Peripheral data input
    per_en,                         // Peripheral enable (high active)
    per_we,                         // Peripheral write enable (high active)
    per_dout,                       // Peripheral data output
	seg_sel,
	seg_hex
);

input              mclk;            // Main system clock
input       [13:0] per_addr;        // Peripheral address
input       [15:0] per_din;         // Peripheral data input
input              per_en;          // Peripheral enable (high active)
input        [1:0] per_we;          // Peripheral write enable (high active)
input              puc_rst;         // Main system reset
output      [15:0] per_dout;        // Peripheral data output
output	reg [3:0]  seg_sel;
output	reg [7:0]  seg_hex;

reg 	[7:0]	hex0, hex1, hex2, hex3;
//=============================================================================
// 1)  PARAMETER DECLARATION
//=============================================================================

// Register base address (must be aligned to decoder bit width)
parameter       [14:0] BASE_ADDR = 15'h0090;

//============================================================================
// 2)  REGISTER DECODER
//============================================================================

// Local register selection
wire              reg_sel      =  per_en & (per_addr[13:1]==BASE_ADDR[14:2]);


// Read/Write probes
wire              reg_lo_write =  per_we[0] & reg_sel;
wire              reg_hi_write =  per_we[1] & reg_sel;
wire              reg_read     = ~|per_we   & reg_sel;

// Read/Write vectors
always@(posedge mclk or posedge puc_rst)
begin
    if(puc_rst)
    begin
        hex0 <= 8'hc0; hex1 <= 8'hf9; hex2 <= 8'ha4; hex3 <= 8'hb0;
    end
    else begin
        if((!per_addr[0]) && reg_lo_write)
            hex0 <= per_din[7:0];
        else if ((!per_addr[0]) && reg_hi_write)
            hex1 <= per_din[7:0];
        else if(per_addr[0] && reg_lo_write)
            hex2 <= per_din[7:0];
        else if (per_addr[0] && reg_hi_write)
            hex3 <= per_din[7:0];
        else;
    end
end

// display delay
reg     [15:0]   delaycnt;
always@(posedge mclk or posedge puc_rst)
    if(puc_rst)
        delaycnt <= 'd0;
    else delaycnt <= delaycnt + 1'b1;

always@(posedge mclk or posedge puc_rst)
	if(puc_rst)
	begin
		seg_sel <= 4'b1110;
		seg_hex <= 8'hff;
	end
	else begin if(delaycnt == 'd0)
		case(seg_sel)
			4'b1110: begin seg_hex <= hex1; seg_sel <= 4'b1101; end
			4'b1101: begin seg_hex <= hex2; seg_sel <= 4'b1011; end
			4'b1011: begin seg_hex <= hex3; seg_sel <= 4'b0111; end
			4'b0111: begin seg_hex <= hex0; seg_sel <= 4'b1110; end
			default : seg_sel <= 4'b1110;
		endcase
	end

endmodule
