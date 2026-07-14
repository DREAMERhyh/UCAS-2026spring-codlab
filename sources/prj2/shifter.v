`timescale 10 ns / 1 ns
`define DATA_WIDTH 32

module shifter (
    input  [`DATA_WIDTH - 1:0] A,
    input  [              4:0] B,
    input  [              1:0] Shiftop,
    output [`DATA_WIDTH - 1:0] Result
);

// 逻辑左移
wire [`DATA_WIDTH - 1:0] sll_result;
assign sll_result = A << B;

// 逻辑右移
wire [`DATA_WIDTH - 1:0] srl_result;
assign srl_result = A >> B;

// 算术右移手动补符号位
wire [`DATA_WIDTH - 1:0] sign_mask;
wire [`DATA_WIDTH - 1:0] sra_result;

// 如果 A 是正数，sign_mask = 0
// 如果 A 是负数，sign_mask 根据 B 生成高位补 1 的掩码
assign sign_mask =
    (B == 5'd0) ? 32'h0000_0000 :
    A[31]      ? (32'hffff_ffff << (6'd32 - {1'b0, B})) :
                 32'h0000_0000;

assign sra_result = srl_result | sign_mask;

// 最终选择
assign Result =
    (Shiftop == 2'b00) ? sll_result :
    (Shiftop == 2'b10) ? srl_result :
    (Shiftop == 2'b11) ? sra_result :
                         32'd0;

endmodule