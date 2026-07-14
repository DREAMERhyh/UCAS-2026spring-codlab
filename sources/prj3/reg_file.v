`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

// 定义32个32-bit寄存器数组（无需初始化）
reg [`DATA_WIDTH - 1:0] rf [0:(1 << `ADDR_WIDTH) - 1];

// 读端口1：逻辑运算实现$zero恒0（组合逻辑）
assign rdata1 = ({`DATA_WIDTH{|raddr1}} & rf[raddr1]) | ({`DATA_WIDTH{~|raddr1}} & `DATA_WIDTH'd0);

// 读端口2：逻辑运算实现$zero恒0（组合逻辑）
assign rdata2 = ({`DATA_WIDTH{|raddr2}} & rf[raddr2]) | ({`DATA_WIDTH{~|raddr2}} & `DATA_WIDTH'd0);

// 写端口：同步写 + 禁止写$zero（时序逻辑）

always @(posedge clk) begin
	if (wen && (waddr != `ADDR_WIDTH'b0)) begin
		rf[waddr] <= wdata;
	end
end
	
endmodule
