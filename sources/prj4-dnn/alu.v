`timescale 10 ns / 1 ns
`define DATA_WIDTH 32

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              3:0]  ALUop,       // ~3-bit ALU操作码，严格按PPT表格定义~，新增乘法
	output                      Overflow,    // 有符号溢出标志
	output                      CarryOut,    // 无符号进位/借位标志
	output                      Zero,        // 结果为零标志
	output [`DATA_WIDTH - 1:0]  Result       // 运算结果
);
// ====================== 第一步：定义中间信号（纯wire，组合逻辑）======================
// 1. 减法控制信号：ALUop=110(SUB)/111(SLT)/011(SLTU)时，需要执行减法（A-B）
//    注意：SLTU也需要减法来判断大小关系（无符号比较）
wire sub_ctrl = (ALUop == 4'b0110) || (ALUop == 4'b0111) || (ALUop == 4'b0011);

// 2. B操作数处理：减法时取反B（补码减法：A-B = A + (~B) + 1）
wire [`DATA_WIDTH - 1:0] B_processed = sub_ctrl ? ~B : B;

// 3. 加法器核心：复用同一套加法器实现ADD/SUB/SLT/SLTU
//    add_result[32]：最高位为进位/借位标志，低32位为运算结果
wire [32:0] add_result = A + B_processed + sub_ctrl; // +sub_ctrl实现补码减法的+1

// 4. 各运算的中间结果（并行计算，组合逻辑）
wire [`DATA_WIDTH - 1:0] and_result = A & B;    // AND运算结果
wire [`DATA_WIDTH - 1:0] or_result  = A | B;    // OR运算结果
wire [`DATA_WIDTH - 1:0] xor_result = A ^ B;    // ✅ 新增：XOR运算结果
wire [`DATA_WIDTH - 1:0] nor_result = ~(A | B); // ✅ 新增：NOR运算结果

// 5. SLT（有符号比较）结果：A < B 时最低位为1，其余为0
//    判定逻辑：
//    - 若A和B符号不同：A为负（A[31]=1）则A < B
//    - 若A和B符号相同：减法结果的符号位异或溢出标志，为1则A < B
wire slt_flag = add_result[31] ^ Overflow;
wire [`DATA_WIDTH - 1:0] slt_result;
assign slt_result[0] = slt_flag;          // 最低位赋值为比较结果
assign slt_result[31:1] = 31'd0;          // 高31位直接赋0

// 6. SLTU（无符号比较）结果：A < B 时最低位为1，其余为0
//    无符号比较只需判断借位：A < B 时减法会产生借位 → add_result[32] = 0
//    因此：A < B 时 sltu_flag = 1，否则为0
wire sltu_flag = ~add_result[32];          // ✅ 新增：无符号比较标志
wire [`DATA_WIDTH - 1:0] sltu_result;
assign sltu_result[0] = sltu_flag;         // 最低位赋值为比较结果
assign sltu_result[31:1] = 31'd0;         // 高31位直接赋0
// 将输入转为有符号后相乘，再截取低32位
wire signed [`DATA_WIDTH-1:0] A_signed = A;
wire signed [`DATA_WIDTH-1:0] B_signed = B;
wire signed [`DATA_WIDTH-1:0] mul_result;
assign mul_result = A_signed * B_signed;

// ====================== 第二步：运算结果选择（MUX，组合逻辑）======================
// 根据ALUop选择最终输出的Result，严格匹配PPT中的ALUop编码
assign Result = (ALUop == 4'b0000) ? and_result  : // 逻辑按位与 (AND)
                (ALUop == 4'b0001) ? or_result   : // 逻辑按位或 (OR)
                (ALUop == 4'b0100) ? xor_result  : // ✅ 逻辑按位异或 (XOR)
                (ALUop == 4'b0101) ? nor_result  : // ✅ 逻辑按位或非 (NOR)
                (ALUop == 4'b0010) ? add_result[31:0] : // 算术加法 (ADD)
                (ALUop == 4'b0110) ? add_result[31:0] : // 算术减法 (SUB)
                (ALUop == 4'b0111) ? slt_result  : // 有符号整数比较 (SLT)
                (ALUop == 4'b0011) ? sltu_result : // ✅ 无符号整数比较 (SLTU)
                (ALUop == 4'b1000) ? mul_result  : // 新增
                `DATA_WIDTH'd0; // 默认值，避免不定态

// ====================== 第三步：标志位计算（纯组合逻辑）======================
// 1. Zero标志：Result全0时为1
assign Zero = (Result == `DATA_WIDTH'd0);

// 2. Overflow（有符号数溢出）：仅ADD/SUB/SLT时有效，SLTU不考虑溢出
//    判定规则：两个操作数符号相同，且结果符号与操作数不同 → 溢出
assign Overflow = ((ALUop == 4'b0010) || (ALUop == 4'b0110) || (ALUop == 4'b0111)) ? 
                  ((A[31] == B_processed[31]) && (A[31] != add_result[31])) : 
                  1'b0;

// 3. CarryOut（无符号数进位/借位）：
//    - ADD：CarryOut = add_result[32]（进位）
//    - SUB/SLT/SLTU：CarryOut = ~add_result[32]（借位 = 进位取反）
//    - 其他运算：CarryOut = 0
assign CarryOut = (ALUop == 4'b0010) ? add_result[32] :       // ADD：进位
                  (ALUop == 4'b0110) ? ~add_result[32] :      // SUB：借位
                  (ALUop == 4'b0111) ? ~add_result[32] :      // SLT：借位（虽然不使用，但保持一致性）
                  (ALUop == 4'b0011) ? ~add_result[32] :      // ✅ SLTU：借位（用于无符号比较）
                  1'b0;                                      // AND/OR/XOR/NOR：0

endmodule